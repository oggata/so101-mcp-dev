# SO-101 MCP サーバー

SO-101 フォロワーアームを Claude から MCP（Model Context Protocol）経由で操作するための小さなサーバーです。
Claude に「グリッパーを開いて」「カメラで机の上を見て」と頼むと、関節の移動やカメラ画像の取得をツールとして実行します。

```
Claude (Desktop / Code)
   │  stdio (MCP)
   ▼
so101_mcp.py ──USB──▶ SO-101 フォロワーアーム（LeRobot 経由）
   │
   └─ /tmp/so101_cam_*.jpg を読む ◀── camera_daemon.py ◀── USB カメラ（front / wrist）
```

## ファイル構成

| ファイル | 役割 |
| --- | --- |
| `so101_mcp.py` | MCP サーバー本体（stdio）。アームの接続・状態取得・関節移動・画像取得のツールを提供 |
| `camera_daemon.py` | カメラ常駐スクリプト。各カメラの最新フレームを `/tmp/so101_cam_<名前>.jpg` に書き続ける |
| `poses.json` | `save_pose` で保存したポーズ（自動で作られる） |
| `motions/` | `record_motion` で記録した動き（自動で作られる） |

カメラを MCP サーバーから直接開かず別プロセスにしているのは、MCP サーバーがカメラ権限を持てない／起動のたびにカメラを開き直すと遅い、という問題を避けるためです。

## 必要なもの

- macOS（`camera_daemon.py` は AVFoundation を使用）
- SO-101 フォロワーアーム（STS3215 サーボ）と USB ケーブル
- USB カメラ（任意。外から全体を映す `front` と、アーム先端の `wrist` を想定）
- [LeRobot](https://github.com/huggingface/lerobot) をインストールした Python 環境（例: conda の `lerobot` 環境）
- 同じ環境に `mcp` と `opencv-python`

```bash
conda activate lerobot
```

```bash
pip install mcp opencv-python
```

## セットアップ

### 1. アームのシリアルポートを調べる

```bash
lerobot-find-port
```

見つかったポート（例: `/dev/tty.usbmodem5AB90669021`）を `so101_mcp.py` の `PORT` に書きます。

### 2. キャリブレーション

MCP サーバーは対話入力を待てないため、**キャリブレーションは事前に済ませておく必要があります**（未実施だと `connect_robot` がエラーになります）。

キャリブレーションでは「関節の中央位置」と「各関節の可動範囲（min〜max）」をモーターごとに記録します。MCP で使う関節の角度は「中央位置 = 0 度」が基準なので、**やり直すと同じ数値でも実際の姿勢が少し変わります**（保存したポーズやモーションも撮り直しが必要になることがあります）。

以下は LeRobot 0.6.1 での手順です。

#### 2-0.（組み立て時に 1 回だけ）モーター ID の設定

新品のサーボは ID がすべて同じなので、組み立て時に 1 個ずつ ID を書き込みます。組み立て済みのアームでは不要です。

```bash
lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/tty.usbmodem5AB90669021
```

`Connect the controller board to the 'gripper' motor only` のように聞かれるので、指示されたモーター 1 個だけをつないで Enter、を gripper → wrist_roll → wrist_flex → elbow_flex → shoulder_lift → shoulder_pan の順にくり返します。

#### 2-1. キャリブレーションの実行

アームの電源を入れ、USB をつないでから実行します。

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/tty.usbmodem5AB90669021 --robot.id=so101_follower
```

1. **既存ファイルの確認**（キャリブ済みの場合のみ）
   `Press ENTER to use provided calibration file ... or type 'c' ...` と聞かれます。
   - Enter：保存済みのキャリブをモーターに書き込むだけで終了
   - `c` + Enter：やり直す（以下に進む）
2. **中央位置を記録する**
   トルクが切れて、アームを手で動かせる状態になります。
   `Move ... to the middle of its range of motion and press ENTER` と出たら、**全関節をそれぞれの可動範囲のだいたい真ん中**に置いて Enter を押します。
   （目安：アームを前に向けて、肘を軽く曲げて立たせた姿勢。グリッパーは半開き）
3. **可動範囲を記録する**
   `Move all joints except 'wrist_roll' sequentially through their entire ranges of motion` と出たら、**wrist_roll 以外の各関節を 1 つずつ、端から端までゆっくり動かします**。
   - shoulder_pan：左右いっぱいまで
   - shoulder_lift：前後いっぱいまで
   - elbow_flex：曲げきる〜伸ばしきるまで
   - wrist_flex：上下いっぱいまで
   - gripper：全閉〜全開まで

   画面に各関節の MIN / POS / MAX が表示されるので、MIN と MAX が更新されなくなったことを確認してから Enter を押します。
   wrist_roll は 1 回転モーター扱いで、範囲は自動的に 0〜4095（全周）になるので、動かさなくて大丈夫です。
4. **保存**
   `Calibration saved to ...` と表示されたら完了です。

#### 2-2. キャリブファイルの場所

```
~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json
```

- ファイル名は `--robot.id`（= `so101_mcp.py` の `ROBOT_ID`）です。**ID が違うと別のファイルを探しに行くので、必ず一致させてください。**
- LeRobot 0.6 以降はフォルダ名が `so_follower/` です。古い版で作った `robots/so101_follower/` のファイルは今の版では使われません。
- 中身は関節ごとの `homing_offset`（中央位置）と `range_min` / `range_max`（エンコーダ値 0〜4095）です。やり直す前にバックアップしておくと、元に戻したくなったときに便利です。

```bash
cp ~/.cache/huggingface/lerobot/calibration/robots/so_follower/so101_follower.json ~/so101_follower.backup.json
```

#### 2-3. キャリブがうまくいったかの確認

MCP から `connect_robot` → `get_state` を呼んで、`calibration` の `range_deg`（各関節の可動角度）が実際の動く範囲と大きくずれていないかを見ます。
極端に小さい関節がある場合は、手順 3 でその関節を端まで動かしきれていません。やり直してください。

#### うまくいかないとき

| 症状 | 対処 |
| --- | --- |
| 特定の関節がほとんど動かない | 手順 3 でその関節を端まで動かしていない。やり直す |
| 動かすと逆方向に行く、端でガクッとする | 手順 2 の中央位置がずれている。中央に置き直してやり直す |
| `Failed to ... motor` などで止まる | 電源・ケーブル・ポートを確認。モーター ID が未設定なら 2-0 を実行 |

### 3. カメラ番号を調べる

```bash
lerobot-find-cameras opencv
```

`camera_daemon.py` の `CAMERAS` に名前と番号を設定します。

```python
CAMERAS = {
    "front": 0,   # 外から全体を映すカメラ
    "wrist": 1,   # アームに付いているカメラ
}
```

### 4. 設定値の確認（`so101_mcp.py` 冒頭）

| 定数 | 意味 |
| --- | --- |
| `PORT` | アームのシリアルポート |
| `ROBOT_ID` | キャリブレーション時の `--robot.id` |
| `SAFE_RATIO` | キャリブ範囲の何割まで動かしてよいか（0.95 なら両端 5% を残す） |
| `MAX_STEP` | 1 回の指令で現在位置から動ける上限[度]（暴走防止）。小さすぎると肩・肘が重力で動けなくなる |
| `CONTROL_HZ` | 軌道を送る制御周波数（50Hz） |
| `AUTO_SPEED_DEG_S` | `duration` を省略したときの速さ。一番大きく動く関節がこの速さ[度/秒]になる |
| `COMP_MAX_DEG` | 重さで垂れた関節に上乗せしてよい最大量[度] |
| `RECORD_HZ` | `record_motion` の記録周波数 |

## MCP への登録

Python は **LeRobot をインストールした環境のフルパス** を指定します（`which python` で確認）。

### Claude Desktop（デスクトップアプリの Code タブも共通）

`~/Library/Application Support/Claude/claude_desktop_config.json` に追記します。

```json
{
  "mcpServers": {
    "so101": {
      "command": "/Volumes/so101/miniconda3/envs/lerobot/bin/python",
      "args": [
        "./Documents/so101-dev/so101_mcp.py"
      ]
    }
  }
}
```

保存したら Claude アプリを再起動します。ツール一覧に `so101` の 12 個のツールが出ていれば成功です。
`so101_mcp.py` を書き換えたときも、反映するにはアプリ（または MCP サーバー）の再起動が必要です。

### Claude Code（CLI）

```bash
claude mcp add so101 -- /Volumes/so101/miniconda3/envs/lerobot/bin/python ./Documents/so101-dev/so101_mcp.py
```

`claude mcp list` で `so101` が `connected` になっていれば OK です。

## 使い方（一連の流れ）

### 1. カメラを起動する（画像を使う場合）

ターミナルで起動したままにしておきます。初回は macOS がターミナルにカメラ権限を求めるので許可してください。

```bash
/Volumes/so101/miniconda3/envs/lerobot/bin/python ./Documents/so101-dev/camera_daemon.py
```

`[front] カメラ 0 を開きました` のように表示されれば動いています。停止は `Ctrl+C`。

### 2. アームの電源を入れ、USB を接続する

アームは安全な姿勢（レスト姿勢）にしておきます。

### 3. Claude に指示する

チャットで自然な言葉で頼めば、Claude が必要なツールを順に呼び出します。典型的な流れ:

1. **接続** — 「アームに接続して」→ `connect_robot`（トルクオン、現在位置を返す）
2. **状態確認** — 「今の姿勢と保存済みのポーズを教えて」→ `get_state`
3. **周囲を見る** — 「カメラで机の上を見て」→ `capture_image`
4. **動かす** — 「構えの姿勢にして」「ブロックを掴んで持ち上げて」→ `move_to_pose` / `move_sequence` / `move_joints`
5. **確認** — 動いた後に再度 `capture_image` や `get_state` で結果を確認
6. **終了** — 「rest に戻してから切断して」→ `move_to_pose` → `disconnect_robot`

指示の例:

```
アームに接続して、今の関節の値と保存済みのポーズを教えて
フロントカメラで机の上に何があるか見て
ready の姿勢にして
赤いブロックを掴んで持ち上げて。肩と肘も使って、1 回の move_sequence でつなげて
pick_and_place を 0.5 倍速で再生して
rest に戻して切断して
```

### 4. 最初にやっておくとよいこと：ポーズと動きを教える

Claude は関節の向きや角度の目安を知っていますが、**実機の手で作った姿勢を見本として持っていると、動きが大きく自然になります**（模倣学習と同じ考え方）。最初に次の 2 つを教えておくのがおすすめです。

#### ポーズを教える（`save_pose`）

1. 「トルクを切って。アームは支えてる」→ `set_torque(False)`（**アームが落ちるので必ず手で支える**）
2. アームを手で目的の姿勢にする
3. 「この姿勢を ready として保存して。説明は『机の上 10cm で爪を下に向けた構え』」→ `save_pose`
   （`save_pose` は今の姿勢をそのまま保存します。トルクは切れたままなので、続けて別の姿勢も教えられます）
4. 終わったら「トルクを入れて」→ `set_torque(True)`（その場の姿勢で止まります）

最低限、次のようなポーズがあると便利です。

| 名前の例 | 姿勢 |
| --- | --- |
| `rest` | 切断前に戻すたたんだ姿勢（トルクを切っても落ちない姿勢） |
| `ready` | 作業を始める構え（机の上で爪を下に向ける） |
| `reach_far` | 前に大きく手を伸ばした姿勢 |
| `place` | 物を置く場所の上 |

保存したポーズは `poses.json` に入り、`move_to_pose("ready")` や `move_sequence` の `{"pose": "ready"}` で使えます。

#### 動きを教える（`record_motion` / `play_motion`）

1. アームを支えて、「今から 10 秒、掴む動きを pick として記録して」→ `record_motion`
   呼ばれた瞬間にトルクが切れ、記録が始まります。手でアームを動かして見本を見せます
2. 時間が来ると、最後の姿勢でトルクが入って止まります
3. 「pick を 0.5 倍速で再生して」→ `play_motion`（開始姿勢までなめらかに移動してから再生）

記録した動きは `motions/<名前>.json` に入ります。

#### 関節の向き（Claude にも伝えてある内容）

| 関節 | サーボ | 役割 | + 方向 |
| --- | --- | --- | --- |
| `shoulder_pan` | F1 | 根元の旋回 | ロボットの右（後ろから前を見て右） |
| `shoulder_lift` | F2 | 肩。リーチと高さの主役 | 腕を前に倒す（先端が前・下へ） |
| `elbow_flex` | F3 | 肘 | 前腕を下に曲げる |
| `wrist_flex` | F4 | 手首の上下 | 手先を下に向ける |
| `wrist_roll` | F5 | 手首のひねり | — |
| `gripper` | F6 | 爪（0〜100） | 開く |

目安として「爪の下向き角度 ≈ shoulder_lift + elbow_flex + wrist_flex（0 で水平、90 で真下）」も伝えています。
この表と式は **URDF（`so101_new_calib.urdf`）から計算したもの**です。初回は「shoulder_lift を +20 度動かして」などで実機の向きと合っているか確認し、違っていたら `so101_mcp.py` の `JOINT_GUIDE` と `INSTRUCTIONS` を直してください。

## ツール一覧

| ツール | 引数 | 説明 |
| --- | --- | --- |
| `connect_robot` | なし | アームに接続しトルクをオン。保存済みキャリブをモーターに書き込む |
| `get_state` | なし | 現在値[度]、安全範囲、関節の役割、保存済みのポーズとモーション、キャリブ情報を返す |
| `move_joints` | `targets`, `duration`（省略で自動） | 指定関節を目標値まで加減速つきで動かす。指定しない関節はそのまま |
| `move_gripper` | `opening`, `duration`（省略で自動） | グリッパーだけを動かすショートカット（0〜100） |
| `move_sequence` | `waypoints` | 複数の通過点を止まらずにつないで動かす。通過点には関節値・`pose`・`duration` を書ける |
| `move_to_pose` | `name`, `duration`（省略で自動） | 保存済みのポーズへ動かす |
| `save_pose` | `name`, `description` | 今の姿勢を名前つきで `poses.json` に保存 |
| `set_torque` | `enabled` | トルクのオン/オフ。オフで手で動かせる（アームが落ちるので支える） |
| `record_motion` | `name`, `seconds=10`, `description` | トルクを切って、手で動かした動きを 30Hz で記録 |
| `play_motion` | `name`, `speed=1.0` | 記録した動きを再生（0.25〜1.5 倍速） |
| `capture_image` | `camera="all"` | `front` / `wrist` / `all` の最新画像を返す（`camera_daemon.py` が必要） |
| `disconnect_robot` | なし | 切断してトルクオフ |

`move_sequence` の例（構え → 物の上で爪を開く → 下ろす → 掴む → 持ち上げる）:

```json
[
  {"pose": "ready"},
  {"shoulder_pan": 15, "shoulder_lift": 35, "elbow_flex": 20, "gripper": 80},
  {"shoulder_lift": 55, "elbow_flex": 30, "duration": 1.5},
  {"gripper": 10, "duration": 0.8},
  {"shoulder_lift": 30, "elbow_flex": 10}
]
```

### 関節の単位と範囲

| 関節 | 単位 | 範囲 |
| --- | --- | --- |
| `shoulder_pan` / `shoulder_lift` / `elbow_flex` / `wrist_flex` / `wrist_roll` | 度（0 = キャリブ時の中央位置） | キャリブ範囲の ±半分 × `SAFE_RATIO`（接続時に計算。`get_state` の `safe_limits` で確認） |
| `gripper` | 0〜100 | 5〜95 |

LeRobot 0.6 以降の SO-101 は角度（度）が標準なので、それに合わせています（以前の版はこの README で -100〜100 と書いていましたが、実際も度で動いていました）。
範囲外の値は自動でクランプされ、戻り値の `clamped` に記録されます。目標に届かなかった関節は `not_reached` に出ます。

## なめらかに動かすための工夫

- **加減速つきの軌道**：動き始めと止まり際はゆっくり、途中は速く（最小ジャーク軌道）。以前の一定速度の補間で起きていた、動き始めと止まり際の「ガクッ」がなくなります
- **通過点をつなぐ**：`move_sequence` は通過点で止まらず、速度がつながる曲線（5 次エルミート補間）で動きます。関節の動く向きが変わる点だけは、行き過ぎを防ぐために一度止まります
- **時間の自動決定**：`duration` を省略すると動く量から時間を決めるので、大きな動きが急すぎたり、小さな動きが遅すぎたりしません
- **垂れ補正**：肩・肘・手首が重さで目標に届かないとき、指令を少しずつ（最大 `COMP_MAX_DEG` 度まで）上乗せして寄せます
- **正確な周期**：50Hz で、`sleep` の誤差がたまらない時間管理

## 安全設計

- 値は `SAFE_LIMITS`（キャリブ範囲 × `SAFE_RATIO`）にクランプ
- 加減速つきの軌道に加え、`max_relative_target`（`MAX_STEP`）で急な動きを制限
- `duration` は 1 区間 0.3〜10 秒、`move_sequence` は通過点 30 個まで
- トルクを入れ直すときは、目標位置を今の位置にしてから入れる（古い目標へ飛ばない）
- ツール呼び出しはロックで直列化（同時に 2 つの動作が走らない）
- LeRobot の `print` やログはすべて stderr に流し、MCP の stdio 通信を壊さない

> **注意:** `disconnect_robot` を呼ぶとトルクが切れ、アームは重力で落ちます。必ず先にレスト姿勢に戻すか、手で支えてから切断してください。
> また、初めて動かすときは小さな値・長めの `duration` から試し、いつでも電源を抜ける状態にしておいてください。

## トラブルシューティング

| 症状 | 対処 |
| --- | --- |
| `キャリブレーションファイルが見つかりません` | `lerobot-calibrate` を実行。`--robot.id` と `ROBOT_ID` が一致しているか確認 |
| 接続できない／ポートが見つからない | `lerobot-find-port` でポートを再確認（USB の挿し直しで変わることがある） |
| `画像がありません` | `camera_daemon.py` を起動する |
| `○ 秒前の画像です` と出る | `camera_daemon.py` が止まっている。ターミナルを確認 |
| `カメラ N を開けません` | カメラ番号（`lerobot-find-cameras opencv`）と macOS のカメラ権限を確認 |
| 肩・肘が動かない、垂れる | `MAX_STEP` が小さすぎる可能性。値を上げる。少しだけ届かないなら `COMP_MAX_DEG` を上げる |
| 動きが手首だけで小さい | ポーズ（`ready` / `reach_far` など）を教えておく。「肩と肘も使って」と指示に入れる |
| 関節が思った向きと逆に動く | `JOINT_GUIDE` の向きが実機と違う。「＋方向」の表を実機で確かめて直す |
| `not_reached` が出る | 物や机に当たっているか、可動範囲の端。姿勢を見直す |
| 再生した動きがカクつく | `play_motion` の `speed` を下げる。記録中に手が止まった部分はそのまま再生される |
| Claude にツールが出てこない | 設定ファイルの Python パス・スクリプトパスを確認し、アプリを再起動 |
| `ImportError`（lerobot） | LeRobot のバージョン差。`so101_mcp.py` は新旧 4 通りの import パスを試すので、LeRobot 自体が入った環境の Python を指定しているか確認 |
