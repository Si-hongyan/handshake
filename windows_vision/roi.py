"""固定二维握手区域 ROI —— 纯逻辑部分无 cv2 依赖，可视化部分延迟导入 cv2。

- point_in_roi / load_roi / save_roi 为纯逻辑，可在 WSL 上做单元测试。
- draw_roi / RoiMouseEditor 需要 cv2，仅 Windows 侧运行时使用（函数内导入）。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Tuple

import yaml


@dataclass
class Roi:
    """归一化矩形 ROI：左上 (x, y)，宽高 (w, h)，均为 [0,1] 画面比例。"""

    x: float = 0.55
    y: float = 0.30
    w: float = 0.35
    h: float = 0.40
    color_bgr: Tuple[int, int, int] = (0, 200, 255)
    thickness: int = 2
    label: str = "HANDSHAKE ZONE"

    def clamp(self) -> "Roi":
        """把矩形夹到画面内，保证 0<=x, x+w<=1 等。"""
        self.x = min(max(self.x, 0.0), 1.0)
        self.y = min(max(self.y, 0.0), 1.0)
        self.w = min(max(self.w, 0.01), 1.0 - self.x)
        self.h = min(max(self.h, 0.01), 1.0 - self.y)
        return self

    def contains(self, px: float, py: float) -> bool:
        """归一化点 (px, py) 是否在 ROI 内。"""
        return (self.x <= px <= self.x + self.w) and (self.y <= py <= self.y + self.h)

    def to_pixels(self, img_w: int, img_h: int) -> Tuple[int, int, int, int]:
        return (
            int(self.x * img_w),
            int(self.y * img_h),
            int(self.w * img_w),
            int(self.h * img_h),
        )


def point_in_roi(px: float, py: float, roi: Roi) -> bool:
    return roi.contains(px, py)


def load_roi(path: str) -> Roi:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    r = data.get("roi", {})
    roi = Roi(
        x=float(r.get("x", 0.55)),
        y=float(r.get("y", 0.30)),
        w=float(r.get("w", 0.35)),
        h=float(r.get("h", 0.40)),
        color_bgr=tuple(r.get("color_bgr", [0, 200, 255])),
        thickness=int(r.get("thickness", 2)),
        label=str(r.get("label", "HANDSHAKE ZONE")),
    )
    return roi.clamp()


def save_roi(path: str, roi: Roi) -> None:
    """把当前 ROI 写回 YAML（鼠标拖拽后按 's' 保存）。"""
    payload = {
        "roi": {
            "x": round(roi.x, 4),
            "y": round(roi.y, 4),
            "w": round(roi.w, 4),
            "h": round(roi.h, 4),
            "color_bgr": list(roi.color_bgr),
            "thickness": roi.thickness,
            "label": roi.label,
        }
    }
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
    os.replace(tmp, path)


def draw_roi(frame, roi: Roi, inside: bool = False):
    """在 BGR 图像上绘制 ROI。需要 cv2（Windows 侧）。"""
    import cv2  # 延迟导入，保证纯逻辑可在无 cv2 环境测试

    h, w = frame.shape[:2]
    x, y, rw, rh = roi.to_pixels(w, h)
    color = (0, 255, 0) if inside else roi.color_bgr
    cv2.rectangle(frame, (x, y), (x + rw, y + rh), color, roi.thickness)
    cv2.putText(
        frame, roi.label, (x, max(0, y - 8)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
    )
    return frame


class RoiMouseEditor:
    """鼠标拖拽调整 ROI（Windows 侧）。左键拖拽画新矩形。"""

    def __init__(self, roi: Roi):
        self.roi = roi
        self._dragging = False
        self._start = (0.0, 0.0)
        self._img_wh = (1, 1)

    def set_image_size(self, w: int, h: int):
        self._img_wh = (max(w, 1), max(h, 1))

    def on_mouse(self, event, x, y, flags, param):
        import cv2  # 延迟导入

        w, h = self._img_wh
        nx, ny = x / w, y / h
        if event == cv2.EVENT_LBUTTONDOWN:
            self._dragging = True
            self._start = (nx, ny)
        elif event == cv2.EVENT_MOUSEMOVE and self._dragging:
            self._update_rect(nx, ny)
        elif event == cv2.EVENT_LBUTTONUP and self._dragging:
            self._dragging = False
            self._update_rect(nx, ny)

    def _update_rect(self, nx: float, ny: float):
        x0, y0 = self._start
        self.roi.x = min(x0, nx)
        self.roi.y = min(y0, ny)
        self.roi.w = abs(nx - x0)
        self.roi.h = abs(ny - y0)
        self.roi.clamp()
