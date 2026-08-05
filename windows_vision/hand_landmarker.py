"""MediaPipe Tasks HandLandmarker 封装（Windows 侧）。

使用最新版 Tasks API，LIVE_STREAM 异步模式：
  - detect_async 提交帧 + 时间戳，结果通过回调异步返回；
  - 我们只保留“最新结果”，天然低延迟、不堆积。

模型文件 hand_landmarker.task 需预先下载到 config/vision.yaml: mediapipe.model_path。
"""

from __future__ import annotations

import threading
from typing import List, Optional


class HandLandmarkerAsync:
    def __init__(
        self,
        model_path: str,
        num_hands: int = 1,
        min_hand_detection_confidence: float = 0.5,
        min_hand_presence_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
    ):
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision

        self._mp = mp
        self._mp_vision = mp_vision

        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.LIVE_STREAM,
            num_hands=num_hands,
            min_hand_detection_confidence=min_hand_detection_confidence,
            min_hand_presence_confidence=min_hand_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
            result_callback=self._on_result,
        )
        self._landmarker = mp_vision.HandLandmarker.create_from_options(options)

        self._lock = threading.Lock()
        self._latest_landmarks: Optional[List[List[float]]] = None
        self._latest_confidence: float = 0.0
        self._latest_ts_ms: int = 0

    def _on_result(self, result, output_image, timestamp_ms: int):
        landmarks = None
        confidence = 0.0
        if result and result.hand_landmarks:
            hand = result.hand_landmarks[0]
            landmarks = [[lm.x, lm.y, lm.z] for lm in hand]
            # handedness 分数作为置信度参考
            if result.handedness and result.handedness[0]:
                confidence = float(result.handedness[0][0].score)
            else:
                confidence = 1.0
        with self._lock:
            self._latest_landmarks = landmarks
            self._latest_confidence = confidence
            self._latest_ts_ms = timestamp_ms

    def submit(self, frame_bgr, timestamp_ms: int) -> None:
        """提交一帧做异步推理。frame_bgr 为 OpenCV BGR ndarray。"""
        import cv2

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        self._landmarker.detect_async(mp_image, timestamp_ms)

    def get_latest(self):
        """返回 (landmarks 或 None, confidence, ts_ms)。"""
        with self._lock:
            return self._latest_landmarks, self._latest_confidence, self._latest_ts_ms

    def close(self) -> None:
        try:
            self._landmarker.close()
        except Exception:
            pass
