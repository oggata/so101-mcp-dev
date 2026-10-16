"""
SO-101 follower MCP server (stdio)

ツール:
  接続・状態   : connect_robot / get_state / disconnect_robot
  動かす       : move_joints / move_gripper / move_sequence / move_to_pose
  教える       : save_pose / set_torque / record_motion / play_motion
  見る         : capture_image

単位:
  - 関節は「度」。0 = キャリブレーション時の中央位置（LeRobot 0.6 の use_degrees=True）
  - グリッパーは 0〜100

安全設計:
  - SAFE_LIMITS（キャリブ範囲の SAFE_RATIO 倍）にクランプ。接続時にキャリブファイルから計算
  - 加減速つき軌道（最小ジャーク / 区間ごとの5次エルミート補間）+ max_relative_target で急な動きを防止
  - 重さで垂れて目標に届かない分を、COMP_MAX_DEG まで指令に上乗せして補正
  - stdio を汚さないよう、ログはすべて stderr へ
"""
import sys
import json
import time
import threading
import contextlib
import logging
from pathlib import Path
from typing import Any

try:  # mcp 2.x
    from mcp.server.mcpserver import MCPServer, Image
except ImportError:  # mcp 1.x
    from mcp.server.fastmcp import FastMCP as MCPServer, Image

# LeRobot のバージョンごとに import パスが違うので順に試す
try:  # 新しい構成（SO-100/101 統合, SO101 名のエイリアスあり）
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
except ImportError:
    try:  # 新しい構成（統合クラス名のみ）
        from lerobot.robots.so_follower import (
            SOFollower as SO101Follower,
            SOFollowerRobotConfig as SO101FollowerConfig,
        )
    except ImportError:
        try:  # 旧構成
            from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
        except ImportError:  # さらに古い構成
            from lerobot.common.robots.so101_follower import SO101Follower, SO101FollowerConfig

# ===== 設定 =====
PORT = "/dev/tty.usbmodem5AB90669021"
ROBOT_ID = "so101_follower"

# SAFE_RATIO はキャリブ範囲の何割まで使うか。0.95 なら両端 5% を残す。
SAFE_RATIO = 0.95
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_JOINTS = JOINTS + ["gripper"]

TICKS_PER_REV = 4096  # STS3215 のエンコーダ分解能
# LeRobot の max_relative_target：目標を「現在位置 ± この値[度]」に制限する暴走防止。
# 小さすぎると、重力負荷のある関節（肘・肩）が動き出す力を出せず止まる・垂れる。
MAX_STEP = 15.0
CONTROL_HZ = 50

# 時間を指定しないときの速さ。一番大きく動く関節がこの速さ[度/秒]になるよう時間を決める
AUTO_SPEED_DEG_S = 45.0
AUTO_MIN_DURATION = 1.0

# 垂れ補正：動き終わった後、この関節が目標から COMP_TOL_DEG 以上ずれていたら
# 指令を最大 COMP_MAX_DEG まで上乗せする（グリッパーは物を掴むと止まるのが正常なので対象外）
COMP_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]
COMP_TOL_DEG = 1.0
COMP_MAX_DEG = 6.0
COMP_GAIN = 0.3
COMP_STEP_DEG = 0.5  # 1 周期で上乗せを増やせる上限（急に押し込まない）
SETTLE_SEC = 0.6
REACH_TOL_DEG = 3.0  # これ以上ずれていたら「届いていない」と報告

BASE_DIR = Path(__file__).resolve().parent
POSES_FILE = BASE_DIR / "poses.json"
MOTIONS_DIR = BASE_DIR / "motions"
RECORD_HZ = 30
# =================

JOINT_GUIDE = {
    "shoulder_pan": "F1 根元の旋回。+ でロボットの右（ロボットの後ろから前を見て右）、- で左。",
    "shoulder_lift": "F2 肩。+ で腕を前に倒す（先端が前・下へ）、- で腕を起こす。リーチと高さの主役。",
    "elbow_flex": "F3 肘。+ で前腕を下に曲げる（先端が下がる）、- で上げる。shoulder_lift と組で高さ・距離を決める。",
    "wrist_flex": "F4 手首の上下。+ で手先を下に向ける、- で上に向ける。爪の角度合わせ用。",
    "wrist_roll": "F5 手首のひねり。爪の開く向きを物の向きに合わせる。",
    "gripper": "F6 グリッパー 0〜100。0 付近が閉、大きいほど開く。",
}

INSTRUCTIONS = f"""
SO-101 は 6 軸（F1〜F6）のロボットアーム。関節値の単位は「度」で、0 はキャリブレーション時の中央位置。
グリッパーだけは 0〜100（0 付近が閉）。

関節の役割と向き:
""" + "\n".join(f"- {j}: {t}" for j, t in JOINT_GUIDE.items()) + """

目安（URDF からの計算。実機の見え方で確かめること）:
- 爪の下向き角度 ≈ shoulder_lift + elbow_flex + wrist_flex（0 で水平前向き、90 で真下）
- 物に手を伸ばす・掴む・持ち上げるといった主要な動きは、shoulder_lift と elbow_flex を 30〜80 度動かすのが普通。
  10〜20 度は微調整の量。手首とグリッパーだけで済ませようとしないこと。

動かし方のコツ:
1. まず get_state を呼び、保存済みのポーズ（poses）とモーション（motions）を確認する。
   人が実機で教えたポーズは最も信頼できる基準点なので、近いポーズがあればそこから始める。
2. 一連の動作は move_sequence でまとめて渡す（止まらずになめらかにつながる）。
   掴む例: 構え → 物の真上（爪を開く）→ 下ろす → 爪を閉じる → 持ち上げる。
   各区間で肩・肘・手首を同時に動かし、爪の向きは上の「下向き角度」の式で保つ。
3. 動いた後は capture_image で結果を確かめ、ずれを小さく直す。
4. 人が「この動きを覚えて」と言ったら record_motion、「この姿勢を覚えて」と言ったら save_pose。
5. set_torque(False) と record_motion はトルクが切れてアームが落ちる。
   人がアームを支えていると確認できたときだけ呼ぶこと。
"""

logging.basicConfig(stream=sys.stderr, level=logging.INFO)
log = logging.getLogger("so101_mcp")

mcp = MCPServer("so101", version="0.2.0", instructions=INSTRUCTIONS)
_robot = None
_lock = threading.Lock()
SAFE_LIMITS: dict[str, tuple[float, float]] = {}  # 接続時にキャリブから埋める
_calib_info: dict = {}
_last_goal: dict[str, float] = {}  # 最後にモーターへ送った目標（垂れ補正込み）


# ---------- 基本 ----------

def _clamp(joint: str, value: float) -> float:
    lo, hi = SAFE_LIMITS[joint]
    return max(lo, min(hi, float(value)))


def _require_robot():
    if _robot is None:
        raise RuntimeError("ロボット未接続です。先に connect_robot を呼んでください。")
    return _robot


def _positions(robot) -> dict:
    obs = robot.get_observation()
    return {k.removesuffix(".pos"): round(float(v), 1)
            for k, v in obs.items() if k.endswith(".pos")}


def _send(robot, goal: dict[str, float]):
    robot.send_action({f"{j}.pos": v for j, v in goal.items()})
    _last_goal.update(goal)


def _check_joints(names):
    unknown = set(names) - set(ALL_JOINTS)
    if unknown:
        raise ValueError(f"不明な関節名: {unknown}（使えるのは {ALL_JOINTS}）")


def _auto_duration(start: dict, goal: dict) -> float:
    span = max((abs(goal[j] - start[j]) for j in goal if j != "gripper"), default=0.0)
    if "gripper" in goal:  # グリッパーは 0〜100 なので度とは別に見る
        span = max(span, abs(goal["gripper"] - start["gripper"]) * 0.6)
    return max(AUTO_MIN_DURATION, span / AUTO_SPEED_DEG_S)


# ---------- 軌道 ----------

def _tangents(times: list[float], values: list[float]) -> list[float]:
    """各通過点での速度。両端は 0、向きが変わる点も 0（行き過ぎを防ぐ）。"""
    n = len(values)
    m = [0.0] * n
    for i in range(1, n - 1):
        d0 = (values[i] - values[i - 1]) / (times[i] - times[i - 1])
        d1 = (values[i + 1] - values[i]) / (times[i + 1] - times[i])
        if d0 * d1 > 0:
            m[i] = 2.0 / (1.0 / d0 + 1.0 / d1)  # 調和平均（Fritsch–Butland）
    return m


def _quintic(p0, p1, m0, m1, T, s):
    """5次エルミート補間（両端の加速度 0）。m0=m1=0 なら最小ジャーク軌道と同じ。"""
    s2, s3 = s * s, s * s * s
    s4, s5 = s3 * s, s3 * s * s
    h00 = 1 - 10 * s3 + 15 * s4 - 6 * s5
    h10 = s - 6 * s3 + 8 * s4 - 3 * s5
    h01 = 10 * s3 - 15 * s4 + 6 * s5
    h11 = -4 * s3 + 7 * s4 - 3 * s5
    return h00 * p0 + h10 * m0 * T + h01 * p1 + h11 * m1 * T


def _run_trajectory(robot, points: list[dict[str, float]], durations: list[float]):
    """points[0]（現在の指令値）から points[-1] まで、加減速つきで止まらずに動かす。"""
    times = [0.0]
    for d in durations:
        times.append(times[-1] + d)
    joints = list(points[0])
    tangents = {j: _tangents(times, [p[j] for p in points]) for j in joints}

    total = times[-1]
    steps = max(1, int(round(total * CONTROL_HZ)))
    t0 = time.perf_counter()
    seg = 0
    for i in range(1, steps + 1):
        t = total * i / steps
        while seg < len(durations) - 1 and t > times[seg + 1]:
            seg += 1
        T = durations[seg]
        s = min(1.0, (t - times[seg]) / T)
        cmd = {}
        for j in joints:
            v = _quintic(points[seg][j], points[seg + 1][j],
                         tangents[j][seg], tangents[j][seg + 1], T, s)
            cmd[j] = _clamp(j, v)
        _send(robot, cmd)
        # 締め切り基準で待つ（sleep の誤差を積み重ねない）
        delay = t0 + t - time.perf_counter()
        if delay > 0:
            time.sleep(delay)


def _settle(robot, goal: dict[str, float]) -> dict:
    """目標で保持しつつ、重さで垂れている関節は指令を上乗せして寄せる。"""
    cmd = dict(goal)
    now = _positions(robot)
    for _ in range(int(SETTLE_SEC * CONTROL_HZ)):
        for j in COMP_JOINTS:
            if j not in goal:
                continue
            err = goal[j] - now[j]
            if abs(err) > COMP_TOL_DEG:
                step = max(-COMP_STEP_DEG, min(COMP_STEP_DEG, COMP_GAIN * err))
                v = cmd[j] + step
                v = max(goal[j] - COMP_MAX_DEG, min(goal[j] + COMP_MAX_DEG, v))
                cmd[j] = _clamp(j, v)
        _send(robot, cmd)
        time.sleep(1 / CONTROL_HZ)
        now = _positions(robot)
    return now


def _report(goal: dict, final: dict, requested: dict | None = None) -> dict:
    out = {"goal": {j: round(v, 1) for j, v in goal.items()}, "actual": final}
    off = {j: round(final[j] - goal[j], 1) for j in goal
           if abs(final[j] - goal[j]) > REACH_TOL_DEG}
    if off:
        out["not_reached"] = off
        out["hint"] = "not_reached の関節は物や机に当たっているか、可動範囲の端の可能性があります。"
    if requested:
        clamped = {j: goal[j] for j in requested if round(goal[j], 3) != round(float(requested[j]), 3)}
        if clamped:
            out["clamped"] = clamped
    return out


def _move(robot, waypoints: list[dict[str, float]], durations: list[float | None]) -> dict:
    """waypoints は関節名→値（指定しない関節は前の点のまま）。durations の None は自動。"""
    start = {j: _last_goal.get(j, v) for j, v in _positions(robot).items()}
    points = [start]
    fixed_durs = []
    for wp, dur in zip(waypoints, durations):
        p = dict(points[-1])
        p.update({j: _clamp(j, v) for j, v in wp.items()})
        fixed_durs.append(max(0.3, min(10.0, float(dur))) if dur else _auto_duration(points[-1], p))
        points.append(p)
    _run_trajectory(robot, points, fixed_durs)
    goal = points[-1]
    final = _settle(robot, goal)
    out = _report(goal, final, waypoints[-1] if len(waypoints) == 1 else None)
    out["duration"] = round(sum(fixed_durs), 2)
    return out


# ---------- ポーズの保存 ----------

def _load_poses() -> dict:
    if POSES_FILE.exists():
        return json.loads(POSES_FILE.read_text())
    return {}


def _pose_targets(name: str) -> dict[str, float]:
    poses = _load_poses()
    if name not in poses:
        raise ValueError(f"ポーズ '{name}' がありません。保存済み: {list(poses)}")
    return poses[name]["positions"]


def _list_motions() -> dict:
    out = {}
    for f in sorted(MOTIONS_DIR.glob("*.json")):
        data = json.loads(f.read_text())
        out[f.stem] = {"description": data.get("description", ""),
                       "seconds": round(len(data["frames"]) / data["hz"], 1)}
    return out


# ---------- ツール ----------

@mcp.tool()
def connect_robot() -> str:
    """SO-101 フォロワーアームに接続し、トルクをオンにする。"""
    global _robot
    with _lock:
        if _robot is not None:
            return "既に接続済みです。"
        try:
            config = SO101FollowerConfig(port=PORT, id=ROBOT_ID, max_relative_target=MAX_STEP,
                                         use_degrees=True)
        except TypeError:  # use_degrees が無い古い LeRobot
            raise RuntimeError("この LeRobot は角度モード(use_degrees)に対応していません。LeRobot を更新してください。")
        robot = SO101Follower(config)
        # LeRobot の print が stdio(MCP通信) を壊さないよう stderr に逃がす
        with contextlib.redirect_stdout(sys.stderr):
            # calibrate=True だと input() 待ちで固まる可能性があるので False
            robot.connect(calibrate=False)
            if not robot.is_calibrated:
                if not robot.calibration:
                    robot.disconnect()
                    raise RuntimeError("キャリブレーションファイルが見つかりません。lerobot-calibrate を先に実行してください。")
                # 保存済みキャリブをモーターに書き込む
                robot.bus.write_calibration(robot.calibration)
        # キャリブ範囲から安全範囲[度]を計算（0 = 範囲の中央）
        _calib_info.clear()
        SAFE_LIMITS.clear()
        for name, cal in robot.calibration.items():
            span_deg = (cal.range_max - cal.range_min) * 360.0 / TICKS_PER_REV
            _calib_info[name] = {"range_ticks": [cal.range_min, cal.range_max],
                                 "range_deg": round(span_deg, 1)}
            if name == "gripper":
                SAFE_LIMITS[name] = (100.0 * (1 - SAFE_RATIO), 100.0 * SAFE_RATIO)
            else:
                half = min(span_deg / 2, 175.0) * SAFE_RATIO
                SAFE_LIMITS[name] = (round(-half, 1), round(half, 1))
        _robot = robot
        _last_goal.clear()
        _last_goal.update(_positions(robot))
        return f"接続しました。現在位置[度]: {_positions(robot)}"


@mcp.tool()
def get_state() -> dict:
    """
    現在の関節値[度]、安全範囲、関節の役割、保存済みのポーズとモーションを返す。
    動かす前にまず呼んで、使えるポーズがあるか確認すること。
    """
    with _lock:
        robot = _require_robot()
        poses = {n: p.get("description", "") for n, p in _load_poses().items()}
        return {
            "positions": _positions(robot),
            "safe_limits": SAFE_LIMITS,
            "joint_guide": JOINT_GUIDE,
            "poses": poses,
            "motions": _list_motions(),
            "calibration": _calib_info,
        }


@mcp.tool()
def move_joints(targets: dict[str, float], duration: float | None = None) -> dict:
    """
    指定した関節を目標値まで加減速つきでなめらかに動かす。指定しなかった関節はそのまま。
    targets 例: {"shoulder_lift": 40, "elbow_flex": 30, "wrist_flex": 20}
    関節名: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll（単位: 度）, gripper（0〜100）
    手を伸ばす・下ろすなどは肩・肘・手首を一緒に指定すると自然な動きになる。
    duration は秒（0.3〜10）。省略すると動く量から自動で決める。
    値は安全範囲に自動でクランプされる（clamped に記録）。届かなかった関節は not_reached に出る。
    """
    with _lock:
        robot = _require_robot()
        _check_joints(targets)
        return _move(robot, [targets], [duration])


@mcp.tool()
def move_gripper(opening: float, duration: float | None = None) -> dict:
    """グリッパーだけを動かすショートカット。opening は 0〜100（0 付近が閉）。"""
    return move_joints({"gripper": opening}, duration)


@mcp.tool()
def move_sequence(waypoints: list[dict[str, Any]]) -> dict:
    """
    複数の通過点を、途中で止まらずになめらかにつないで動かす。掴む・置くなど一連の動作に使う。
    各通過点は辞書で、次のキーを持てる（指定しない関節は前の通過点の値を引き継ぐ）:
      - 関節名: 値（度 / gripper は 0〜100）
      - "pose": 保存済みポーズ名（そのポーズを基準に、同じ辞書の関節値で上書き）
      - "duration": その区間にかける秒数（省略で自動）
    例: [{"pose": "ready"},
         {"shoulder_pan": 15, "shoulder_lift": 35, "elbow_flex": 20, "gripper": 80},
         {"shoulder_lift": 55, "elbow_flex": 30, "duration": 1.5},
         {"gripper": 10, "duration": 0.8},
         {"shoulder_lift": 30, "elbow_flex": 10}]
    同じ値が続く関節はその区間で止まり、動きの向きが変わる関節は通過点で一度止まる（行き過ぎ防止）。
    """
    with _lock:
        robot = _require_robot()
        if not waypoints:
            raise ValueError("waypoints が空です。")
        if len(waypoints) > 30:
            raise ValueError("通過点は 30 個までです。")
        targets, durations = [], []
        for wp in waypoints:
            wp = dict(wp)
            dur = wp.pop("duration", None)
            t = dict(_pose_targets(wp.pop("pose"))) if "pose" in wp else {}
            _check_joints(wp)
            t.update({j: float(v) for j, v in wp.items()})
            targets.append(t)
            durations.append(dur)
        return _move(robot, targets, durations)


@mcp.tool()
def move_to_pose(name: str, duration: float | None = None) -> dict:
    """保存済みのポーズ（save_pose で人が教えた姿勢）へ動かす。名前は get_state の poses で確認。"""
    with _lock:
        robot = _require_robot()
        return _move(robot, [_pose_targets(name)], [duration])


@mcp.tool()
def save_pose(name: str, description: str = "") -> dict:
    """
    今の姿勢を名前つきで保存する（poses.json）。同じ名前は上書き。
    人が set_torque(False) で手で姿勢を作ってから呼ぶと、実機に合った基準点になる。
    description には「物の真上で爪を開いた構え」のように、どんな姿勢かを書く。
    """
    with _lock:
        robot = _require_robot()
        poses = _load_poses()
        poses[name] = {"positions": _positions(robot), "description": description}
        POSES_FILE.write_text(json.dumps(poses, ensure_ascii=False, indent=2))
        return {"saved": name, **poses[name]}


@mcp.tool()
def set_torque(enabled: bool) -> str:
    """
    トルクのオン/オフ。オフにすると手でアームを動かせる（ポーズを教えるとき用）。
    注意: オフにするとアームが重さで落ちる。人がアームを支えていると確認できたときだけ False にすること。
    オンに戻すと、その場の姿勢で止まる。
    """
    with _lock:
        robot = _require_robot()
        with contextlib.redirect_stdout(sys.stderr):
            if enabled:
                _hold_here(robot)
                return f"トルクをオンにしました。現在位置で保持: {_positions(robot)}"
            robot.bus.disable_torque()
        return "トルクをオフにしました。手で動かせます。終わったら save_pose か set_torque(True) を。"


def _hold_here(robot):
    """目標を今の位置にしてからトルクを入れる（古い目標へ飛ぶのを防ぐ）。"""
    now = _positions(robot)
    robot.bus.sync_write("Goal_Position", now)
    robot.bus.enable_torque()
    _last_goal.clear()
    _last_goal.update(now)


@mcp.tool()
def record_motion(name: str, seconds: float = 10.0, description: str = "") -> dict:
    """
    人がアームを手で動かした動きを記録する（模倣と同じく、全関節の動きをそのまま覚える）。
    呼ぶとすぐトルクが切れ、seconds 秒間 30Hz で記録し、最後の姿勢でトルクを戻して保持する。
    注意: 呼ぶ前に、人がアームを支えていて準備ができていることを必ず確認すること。
    保存先は motions/<name>.json。play_motion で再生できる。
    """
    seconds = max(1.0, min(60.0, float(seconds)))
    with _lock:
        robot = _require_robot()
        frames = []
        with contextlib.redirect_stdout(sys.stderr):
            robot.bus.disable_torque()
            try:
                t0 = time.perf_counter()
                n = int(seconds * RECORD_HZ)
                for i in range(n):
                    frames.append(_positions(robot))
                    delay = t0 + (i + 1) / RECORD_HZ - time.perf_counter()
                    if delay > 0:
                        time.sleep(delay)
            finally:
                _hold_here(robot)
        frames = _smooth(frames)
        MOTIONS_DIR.mkdir(exist_ok=True)
        data = {"hz": RECORD_HZ, "description": description, "frames": frames}
        (MOTIONS_DIR / f"{name}.json").write_text(json.dumps(data, ensure_ascii=False))
        moved = {j: round(max(f[j] for f in frames) - min(f[j] for f in frames), 1) for j in ALL_JOINTS}
        return {"saved": name, "seconds": seconds, "frames": len(frames), "range_of_each_joint": moved}


def _smooth(frames: list[dict], k: int = 2) -> list[dict]:
    """手ぶれを軽くならす（前後 k フレームの移動平均）。"""
    out = []
    for i in range(len(frames)):
        win = frames[max(0, i - k): i + k + 1]
        out.append({j: round(sum(f[j] for f in win) / len(win), 2) for j in frames[i]})
    return out


@mcp.tool()
def play_motion(name: str, speed: float = 1.0) -> dict:
    """
    record_motion で記録した動きを再生する。まず開始姿勢までなめらかに移動してから再生する。
    speed は再生速度の倍率（0.25〜1.5）。初めて再生するときは 0.5 くらいから試すとよい。
    """
    speed = max(0.25, min(1.5, float(speed)))
    path = MOTIONS_DIR / f"{name}.json"
    if not path.exists():
        raise ValueError(f"モーション '{name}' がありません。保存済み: {list(_list_motions())}")
    data = json.loads(path.read_text())
    frames = [{j: _clamp(j, v) for j, v in f.items() if j in ALL_JOINTS} for f in data["frames"]]
    with _lock:
        robot = _require_robot()
        _move(robot, [frames[0]], [None])  # 開始姿勢へ
        dt = 1.0 / data["hz"] / speed
        t0 = time.perf_counter()
        for i, f in enumerate(frames[1:], start=1):
            _send(robot, f)
            delay = t0 + i * dt - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
        final = _settle(robot, frames[-1])
        out = _report(frames[-1], final)
        out["played"] = name
        out["seconds"] = round(len(frames) * dt, 1)
        return out


@mcp.tool()
def capture_image(camera: str = "all") -> list:
    """
    カメラの最新画像を返す。camera_daemon.py が起動している必要がある。
    camera: "front"（外から全体）/ "wrist"（アーム先端）/ "all"（両方、front→wrist の順）
    """
    import glob
    import os

    if camera == "all":
        paths = sorted(glob.glob("/tmp/so101_cam_*.jpg"), key=lambda p: "wrist" in p)
        paths = [p for p in paths if ".tmp." not in p]
    else:
        paths = [f"/tmp/so101_cam_{camera}.jpg"]

    if not paths:
        raise RuntimeError("画像がありません。ターミナルで camera_daemon.py を起動してください。")

    results = []
    for path in paths:
        name = os.path.basename(path).removeprefix("so101_cam_").removesuffix(".jpg")
        if not os.path.exists(path):
            raise RuntimeError(f"カメラ '{name}' の画像がありません。名前か camera_daemon.py の設定を確認してください。")
        age = time.time() - os.path.getmtime(path)
        if age > 3.0:
            results.append(f"[{name}] 注意: {age:.0f} 秒前の画像です（camera_daemon.py が止まっている可能性）")
        else:
            results.append(f"[{name}]")
        with open(path, "rb") as f:
            results.append(Image(data=f.read(), format="jpeg"))
    return results


@mcp.tool()
def disconnect_robot() -> str:
    """切断してトルクをオフにする。注意: アームが重力で落ちるので、事前に人が支えるかレスト姿勢にすること。"""
    global _robot
    with _lock:
        if _robot is None:
            return "接続されていません。"
        with contextlib.redirect_stdout(sys.stderr):
            _robot.disconnect()
        _robot = None
        return "切断しました（トルクオフ）。"


if __name__ == "__main__":
    mcp.run()
