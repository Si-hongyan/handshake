"""阶段 A 主程序（在 Windows 原生 Python 运行）。

流程：
  摄像头(只留最新帧) -> MediaPipe 异步推理(只留最新结果)
  -> 张手判定 / ROI 判定 -> 组装 VisionState -> UDP 发往 WSL
并显示：手部关键点、ROI、当前视觉布尔、稳定计时器、FPS/延迟。

键位：
  q  退出
  s  保存当前 ROI 到 config/roi.yaml
  鼠标左键拖拽：重画 ROI

用法（Windows PowerShell，项目已通过 \\wsl$ 或复制到 Windows 侧）：
  python -m windows_vision.main --config-dir config
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import yaml  # noqa: E402

from shared.protocol import VisionState  # noqa: E402
from windows_vision.camera import LatestFrameCamera  # noqa: E402
from windows_vision.hand_landmarker import HandLandmarkerAsync  # noqa: E402
from windows_vision.open_palm import is_open_palm, wrist_xy  # noqa: E402
from windows_vision.roi import load_roi, save_roi, draw_roi, RoiMouseEditor, point_in_roi  # noqa: E402
from windows_vision.vision_state import StableGestureTracker, compute_latency_ok  # noqa: E402
from windows_vision.sender import UdpSender  # noqa: E402


def _load(cfg_dir, name):
    with open(os.path.join(cfg_dir, name), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config-dir", default="config")
    ap.add_argument("--no-window", action="store_true", help="不弹窗（无头调试）")
    ap.add_argument("--send-host", default=None,
                    help="覆盖 UDP 目标地址。WSL2 NAT 模式下填 WSL 的 eth0 IP（如 172.17.36.149）")
    ap.add_argument("--send-port", type=int, default=None, help="覆盖 UDP 目标端口")
    args = ap.parse_args()

    import cv2

    vision_cfg = _load(args.config_dir, "vision.yaml")
    net_cfg = _load(args.config_dir, "network.yaml")
    roi = load_roi(os.path.join(args.config_dir, "roi.yaml"))
    roi_path = os.path.join(args.config_dir, "roi.yaml")

    cam_cfg = vision_cfg["camera"]
    mp_cfg = vision_cfg["mediapipe"]
    palm_cfg = vision_cfg["open_palm"]
    stab_cfg = vision_cfg["stable_gesture"]
    lat_cfg = vision_cfg["latency"]
    conf_cfg = vision_cfg["confidence"]

    model_path = mp_cfg["model_path"]
    if not os.path.isfile(model_path):
        print(f"[FATAL] 找不到 MediaPipe 模型: {model_path}")
        print("请先下载 hand_landmarker.task（见 README），程序拒绝启动。")
        sys.exit(2)

    cam = LatestFrameCamera(cam_cfg["index"], cam_cfg["width"], cam_cfg["height"], cam_cfg["target_fps"])
    cam.open()
    landmarker = HandLandmarkerAsync(
        model_path=model_path,
        num_hands=mp_cfg["num_hands"],
        min_hand_detection_confidence=mp_cfg["min_hand_detection_confidence"],
        min_hand_presence_confidence=mp_cfg["min_hand_presence_confidence"],
        min_tracking_confidence=mp_cfg["min_tracking_confidence"],
    )
    send_host = args.send_host or net_cfg["udp"]["send_host"]
    send_port = args.send_port or net_cfg["udp"]["port"]
    sender = UdpSender(send_host, send_port)
    print(f"[INFO] UDP 目标: {send_host}:{send_port}")
    tracker = StableGestureTracker(stab_cfg["dwell_seconds"], stab_cfg["max_bad_frame_ratio"])

    editor = RoiMouseEditor(roi)
    win = "Handshake Vision (stage A)"
    if not args.no_window:
        cv2.namedWindow(win)
        cv2.setMouseCallback(win, editor.on_mouse)

    seq = 0
    fps = 0.0
    last_t = time.time()
    print("[INFO] 阶段A 视觉启动。q 退出 / s 保存ROI / 拖拽调整ROI。")

    try:
        while True:
            frame, frame_t = cam.read_latest()
            now = time.time()
            if frame is None:
                if not args.no_window:
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break
                time.sleep(0.005)
                continue

            h, w = frame.shape[:2]
            editor.set_image_size(w, h)

            t0 = time.time()
            landmarker.submit(frame, int(now * 1000))
            landmarks, confidence, _ = landmarker.get_latest()
            latency_ms = (time.time() - t0) * 1000.0

            camera_alive = cam.is_alive(lat_cfg["camera_alive_timeout_s"])
            latency_ok = compute_latency_ok(latency_ms, lat_cfg["max_frame_latency_ms"])

            hand_detected = bool(landmarks) and confidence >= conf_cfg["min_detection_confidence"]
            open_palm = False
            inside_zone = False
            wxy = None
            if hand_detected:
                open_palm = is_open_palm(
                    landmarks, palm_cfg["min_extended_fingers"], palm_cfg["include_thumb"]
                )
                wxy = wrist_xy(landmarks)
                inside_zone = point_in_roi(wxy[0], wxy[1], roi)

            cond = hand_detected and open_palm and inside_zone and camera_alive and latency_ok
            stable = tracker.update(cond, now)

            # FPS 估计（EMA）
            dt = now - last_t
            last_t = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            seq += 1
            state = VisionState(
                seq=seq, t_send=now,
                hand_detected=hand_detected, open_palm=open_palm, inside_zone=inside_zone,
                stable_gesture=stable, camera_alive=camera_alive, latency_ok=latency_ok,
                confidence=confidence, wrist_xy=wxy, latency_ms=latency_ms, fps=fps,
            )
            sender.send(state)

            if not args.no_window:
                _draw_overlay(cv2, frame, roi, editor.roi, state, tracker, now, stab_cfg["dwell_seconds"], landmarks)
                cv2.imshow(win, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    save_roi(roi_path, editor.roi)
                    print(f"[INFO] ROI 已保存: {roi_path}")
    finally:
        cam.release()
        landmarker.close()
        sender.close()
        if not args.no_window:
            cv2.destroyAllWindows()
        print("[INFO] 阶段A 视觉已退出。")


def _draw_overlay(cv2, frame, roi, live_roi, state, tracker, now, dwell, landmarks):
    draw_roi(frame, live_roi, inside=state.inside_zone)
    if landmarks:
        h, w = frame.shape[:2]
        for lm in landmarks:
            cv2.circle(frame, (int(lm[0] * w), int(lm[1] * h)), 3, (0, 255, 0), -1)
    lines = [
        f"hand={int(state.hand_detected)} palm={int(state.open_palm)} zone={int(state.inside_zone)}",
        f"cam={int(state.camera_alive)} lat_ok={int(state.latency_ok)} stable={int(state.stable_gesture)}",
        f"conf={state.confidence:.2f} lat={state.latency_ms:.0f}ms fps={state.fps:.0f}",
        f"dwell {tracker.elapsed(now):.2f}/{dwell:.2f}s",
    ]
    y = 20
    for ln in lines:
        cv2.putText(frame, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        y += 20


if __name__ == "__main__":
    main()
