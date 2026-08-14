#!/usr/bin/env python3
"""display_utils.py - 显示工具（中文支持 + GTK3/QT5 全屏兼容）"""
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

_FONT_PATH = '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc'
_font_cache = {}


def _get_font(size):
    size = int(size)
    if size not in _font_cache:
        _font_cache[size] = ImageFont.truetype(_FONT_PATH, size)
    return _font_cache[size]


def draw_display(qr_text, sign_text, qr_detected, direction="", width=640, height=480):
    """绘制显示画面（PIL 中文支持），返回 OpenCV BGR 图像
    direction: "顺时针" / "逆时针" / "" 显示在二维码数字下方
    """
    pil_img = Image.new('RGB', (width, height), (0, 0, 0))
    draw = ImageDraw.Draw(pil_img)

    color = (0, 255, 0) if qr_detected else (255, 0, 0)
    font_large = _get_font(60)
    bbox = draw.textbbox((0, 0), qr_text, font=font_large)
    tw = bbox[2] - bbox[0]
    draw.text(((width - tw) // 2, 100), qr_text, font=font_large, fill=color)

    if direction:
        font_dir = _get_font(40)
        dir_color = (0, 255, 0) if qr_detected else (128, 128, 128)
        bbox = draw.textbbox((0, 0), direction, font=font_dir)
        dw = bbox[2] - bbox[0]
        draw.text(((width - dw) // 2, 180), direction, font=font_dir, fill=dir_color)

    if sign_text:
        font_small = _get_font(28)
        raw_lines = sign_text.split('\n')
        lines = []
        for raw in raw_lines:
            while len(raw) > 15:
                lines.append(raw[:15])
                raw = raw[15:]
            if raw:
                lines.append(raw)
        y = 270
        for line in lines:
            bbox = draw.textbbox((0, 0), line, font=font_small)
            sw = bbox[2] - bbox[0]
            draw.text(((width - sw) // 2, y), line, font=font_small, fill=(0, 255, 255))
            y += 35
    else:
        font_small = _get_font(24)
        hint = "Waiting sign result..."
        bbox = draw.textbbox((0, 0), hint, font=font_small)
        hw = bbox[2] - bbox[0]
        draw.text(((width - hw) // 2, 370), hint, font=font_small, fill=(128, 128, 128))

    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


_screen_size = None


def _get_screen_size():
    global _screen_size
    if _screen_size is not None:
        return _screen_size
    try:
        import tkinter as tk
        root = tk.Tk()
        _screen_size = (root.winfo_screenwidth(), root.winfo_screenheight())
        root.destroy()
    except Exception:
        _screen_size = (0, 0)
    return _screen_size


def set_fullscreen(window_name):
    """设置窗口全屏（GTK3/QT5 兼容），每次调用都尝试设置"""
    try:
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    except Exception:
        pass
    try:
        cv2.moveWindow(window_name, 0, 0)
    except Exception:
        pass
    sw, sh = _get_screen_size()
    if sw > 0 and sh > 0:
        try:
            cv2.resizeWindow(window_name, sw, sh)
        except Exception:
            pass
