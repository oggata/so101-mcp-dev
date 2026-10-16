"""
カメラ常駐スクリプト（ターミナルから起動する）・複数カメラ対応
各カメラの最新フレームを /tmp/so101_cam_<名前>.jpg に書き続ける。
MCP サーバーはこのファイルを読むだけ。

起動するとプレビューウィンドウも開く（2台を横並び表示）。
  python camera_daemon.py            # 書き出し + プレビュー
  python camera_daemon.py --no-view  # 書き出しだけ

プレビュー上で q / ESC → ウィンドウだけ閉じる（書き出しは続く）
               s       → スナップショット保存
ターミナルで Ctrl+C → 全部停止
"""
import argparse
import os
import threading
import time

import cv2
import numpy as np

# 名前: カメラ番号。番号は lerobot-find-cameras opencv で確認する
# 2026-10-16: 映像が入れ替わっていたので front/wrist の番号を交換
CAMERAS = {
    "front": 1,   # 外から全体を映すカメラ
    "wrist": 0,   # アームに付いているカメラ
}
OUT_DIR = "/tmp"
CAPTURE_SIZE = (1280, 720)  # カメラから取る解像度（USB帯域を節約）
SAVE_WIDTH = 640            # 保存する画像の幅
INTERVAL = 0.2              # 書き出し間隔（秒）
VIEW_HEIGHT = 480           # プレビューの高さ

stop_event = threading.Event()
latest = {name: None for name in CAMERAS}       # プレビュー用の最新フレーム
latest_time = {name: 0.0 for name in CAMERAS}
lock = threading.Lock()


def run_camera(name: str, index: int):
    out_path = os.path.join(OUT_DIR, f"so101_cam_{name}.jpg")
    tmp_path = os.path.join(OUT_DIR, f"so101_cam_{name}.tmp.jpg")

    cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        print(f"[{name}] カメラ {index} を開けません。番号と権限を確認してください。")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_SIZE[1])
    print(f"[{name}] カメラ {index} を開きました → {out_path}")

    fails = 0
    try:
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                fails += 1
                if fails % 50 == 0:
                    print(f"[{name}] フレーム取得に失敗し続けています")
                time.sleep(0.1)
                continue
            fails = 0
            h, w = frame.shape[:2]
            frame = cv2.resize(frame, (SAVE_WIDTH, int(h * SAVE_WIDTH / w)))
            cv2.imwrite(tmp_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            os.replace(tmp_path, out_path)  # 読み込み途中の壊れた画像を防ぐ
            with lock:
                latest[name] = frame
                latest_time[name] = time.time()
            time.sleep(INTERVAL)
    finally:
        cap.release()
        print(f"[{name}] 停止しました")


def make_tile(name):
    with lock:
        frame = None if latest[name] is None else latest[name].copy()
        age = time.time() - latest_time[name]
    if frame is None:
        frame = np.zeros((VIEW_HEIGHT, int(VIEW_HEIGHT * 16 / 9), 3), np.uint8)
        text, color = f"{name}: waiting...", (0, 0, 255)
    else:
        h, w = frame.shape[:2]
        frame = cv2.resize(frame, (int(w * VIEW_HEIGHT / h), VIEW_HEIGHT))
        stale = age > 2.0
        text = f"{name}" + (f"  STALE {age:.0f}s" if stale else "")
        color = (0, 0, 255) if stale else (0, 255, 0)
    h, w = frame.shape[:2]
    # 中心の十字線（位置合わせ用）
    cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
    cv2.line(frame, (0, h // 2), (w, h // 2), (255, 255, 0), 1)
    cv2.rectangle(frame, (0, 0), (w, 32), (0, 0, 0), -1)
    cv2.putText(frame, text, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-view", action="store_true", help="プレビューを開かない")
    args = ap.parse_args()

    threads = [
        threading.Thread(target=run_camera, args=(name, idx), daemon=True)
        for name, idx in CAMERAS.items()
    ]
    for t in threads:
        t.start()
        time.sleep(0.5)  # 同時オープンで失敗しないよう少しずらす

    print("書き出し中（Ctrl+C で停止）")
    view = not args.no_view
    win = "SO-101 cameras (q: close view / s: snapshot)"
    try:
        while any(t.is_alive() for t in threads):
            if not view:
                time.sleep(0.5)
                continue
            # macOS では imshow はメインスレッドで呼ぶ必要がある
            cv2.imshow(win, cv2.hconcat([make_tile(n) for n in CAMERAS]))
            key = cv2.waitKey(66) & 0xFF  # 約15fps
            if key in (ord("q"), 27):
                cv2.destroyWindow(win)
                cv2.waitKey(1)
                view = False
                print("プレビューを閉じました（書き出しは継続中）")
            elif key == ord("s"):
                fn = time.strftime("snapshot_%Y%m%d_%H%M%S.jpg")
                cv2.imwrite(fn, cv2.hconcat([make_tile(n) for n in CAMERAS]))
                print("saved", fn)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=2)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()