#!/usr/bin/env python3
"""
HSV 颜色阈值调参工具 — 纯 OpenCV 键盘版 (无 trackbar，无 matplotlib)

用法:
    python3 hsv_tuner.py
    python3 hsv_tuner.py --camera 2
    python3 hsv_tuner.py --image test.jpg

操作 (全部键盘，HUD 显示当前值):
    q/w     H_low  ±1 / ±5
    a/s     H_high ±1 / ±5
    z/x     S_low  ±1 / ±5
    d/c     S_high ±1 / ±5
    r/f     V_low  ±1 / ±5
    t/g     V_high ±1 / ±5
    1/2/3   切换 Group
    ←/→     切换颜色
    o       开关叠加显示
    p       暂停/继续
    v       Ctrl+S 保存 YAML
    ESC     退出
"""

import cv2
import yaml
import argparse
import numpy as np
import os
import sys
from pathlib import Path
from collections import OrderedDict

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG = SCRIPT_DIR.parent / "config" / "color_thresholds.yaml"

COLOR_NAMES = [
    "red_team",
    "blue_team",
    # ✅ 2026-09-20: golden_mustika → 排球三色 (v1.1: Mikasa V300W 黄底+蓝纹+白字)
    "yellow_ball",
    "blue_mark",
    "white_mark",
    "dark_green_pillar",
    "brown_core_pillar",
    "green_building_spot",
    "brown_fence",
    "light_colors",
]


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    config = {}
    for name in COLOR_NAMES:
        if name in raw and "groups" in raw[name]:
            config[name] = raw[name]["groups"]
        else:
            config[name] = []
    return config


def save_config(path, config):
    comments = {
        "red_team":            "# ── 红色队伍 (RGB: 223-34-34) ──",
        "blue_team":           "# ── 蓝色队伍 (RGB: 50-0-255) ──",
        # ✅ 2026-09-20: 穆斯蒂卡按 v1.1 改正为排球(黄底+蓝纹+白字), 原 golden_mustika 作废
        "yellow_ball":         "# ── 排球黄底 (主判据; Pantone 1235C 近似) ──",
        "blue_mark":           "# ── 排球蓝纹 (容忍色, 不否决黄底) ──",
        "white_mark":          "# ── 排球白字 (容忍色, 不否决黄底) ──",
        "dark_green_pillar":   "# ── 深绿色穆斯蒂卡柱 (RGB: 40-100-50) ──",
        "brown_core_pillar":   "# ── 棕色核心支柱 (RGB: 100-62-0) ──",
        "green_building_spot": "# ── 绿色建筑点位 (RGB: 40-100-50) ──",
        "brown_fence":         "# ── 棕色围栏/边界 (RGB: 100-62-0) ──",
        "light_colors":        "# ── 浅色区域 (地板、白线等) ──",
    }

    def format_groups(groups):
        lines = []
        for i, g in enumerate(groups):
            lo = g["lower"]
            hi = g["upper"]
            labels = ["  # 主阈值", "  # 暗光 / 低饱和补偿", "  # 极低饱和兜底"]
            label = labels[i] if i < len(labels) else ""
            lines.append(
                f"    - {{ lower: [{lo[0]:>4}, {lo[1]:>4}, {lo[2]:>4}],"
                f" upper: [{hi[0]:>4}, {hi[1]:>4}, {hi[2]:>4}] }}{label}"
            )
        return "\n".join(lines)

    with open(path, "w", encoding="utf-8") as f:
        f.write(
            "# ═══════════════════════════════════════════════════════════════════════════════\n"
            "# color_thresholds.yaml — 传统 CV HSV 颜色阈值\n"
            "#\n"
            "# 每个颜色可配置 1~N 组阈值，检测时取并集 (mask = group1 | group2 | ...).\n"
            "# 只在确实需要多段覆盖时才加多组 (如红色跨 0°/180°).\n"
            "# ⚠️  比赛现场必须重新校准！\n"
            "# ═══════════════════════════════════════════════════════════════════════════════\n"
            "\n"
        )
        for name in COLOR_NAMES:
            groups = config.get(name, [])
            f.write(f"{comments.get(name, f'# ── {name} ──')}\n")
            f.write(f"{name}:\n")
            f.write(f"  groups:\n")
            f.write(format_groups(groups))
            f.write("\n\n")

        f.write(
            "# ── 后处理参数 ──\n"
            "morphology_kernel_size: 3         # 形态学操作 (开/闭运算) 核大小\n"
            "min_color_pixel_ratio: 0.7         # 判定颜色归属的最小像素占比\n"
        )

    print(f"\n[✓] 已保存 → {path}")
    print(f"    共 {sum(len(v) for v in config.values())} 组阈值")


class HSVTuner:
    def __init__(self, config_path, camera_id=None, image_path=None):
        self.config_path = Path(config_path)
        self.config = load_config(self.config_path)

        self.color_idx = 0
        self.group_idx = 0
        self.show_overlay = True
        self.paused = False
        self.step = 1  # 微调步长

        # ── 视频源 ──
        self.use_image = image_path is not None
        if self.use_image:
            self.frame = cv2.imread(image_path)
            if self.frame is None:
                raise FileNotFoundError(f"无法读取图片: {image_path}")
        else:
            cam = camera_id if camera_id is not None else 0
            self.cap = cv2.VideoCapture(cam)
            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开摄像头 index={cam}")
            self._update_frame()

        self.win_name = "HSV Tuner — ROBOCON 2027"
        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win_name, 1280, 780)

        self._print_help()

    @property
    def current_color_name(self):
        return COLOR_NAMES[self.color_idx]

    @property
    def current_groups(self):
        return self.config[self.current_color_name]

    @property
    def num_groups(self):
        return len(self.current_groups)

    def _update_frame(self):
        if not self.use_image:
            ret, self.frame = self.cap.read()
            if not ret:
                return False
        return True

    def _current_group(self):
        """返回当前正在编辑的 group"""
        groups = self.current_groups
        while self.group_idx >= len(groups):
            groups.append({"lower": [0, 0, 0], "upper": [180, 255, 255]})
        return groups[self.group_idx]

    def _print_help(self):
        print("═" * 55)
        print("HSV 颜色阈值调参工具 — 键盘操控")
        print("═" * 55)
        print(f"配置: {self.config_path}")
        print()
        print("  [q/w] H_low  [a/s] H_high   ±1 / ±5")
        print("  [z/x] S_low  [d/c] S_high   ±1 / ±5")
        print("  [r/f] V_low  [t/g] V_high   ±1 / ±5")
        print("  [1/2/3]切换Group  [←/→]换颜色  [v]保存")
        print("  [o]叠加  [p]暂停  [ESC]退出")
        print("═" * 55)
        print(f"当前: {self.current_color_name} | Group {self.group_idx + 1}")
        self._print_values()

    def _print_values(self):
        g = self._current_group()
        lo, hi = g["lower"], g["upper"]
        print(f"  H:[{lo[0]:>3},{hi[0]:>3}]  S:[{lo[1]:>3},{hi[1]:>3}]  V:[{lo[2]:>3},{hi[2]:>3}]  step={self.step}")

    def _draw_hud(self, display, mask_cur, mask_all):
        """在图像上叠加 HUD 文字"""
        h, w = display.shape[:2]

        def put(x, y, text, color=(0, 255, 255), scale=0.55):
            cv2.putText(display, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2,
                        cv2.LINE_AA)

        # 半透明黑色底栏
        bar_h = 130
        overlay = display.copy()
        cv2.rectangle(overlay, (0, h - bar_h), (w, h), (0, 0, 0), -1)
        display = cv2.addWeighted(display, 1.0, overlay, 0.7, 0)

        g = self._current_group()
        lo, hi = g["lower"], g["upper"]
        y0 = h - bar_h + 22

        put(10, y0, f"Color: {self.current_color_name} | Group: {self.group_idx + 1}/{max(self.num_groups, 1)} | step={self.step}",
            (0, 255, 255), 0.6)
        put(10, y0 + 28, f"H_low:{lo[0]:>4}  H_high:{hi[0]:>4}  S_low:{lo[1]:>4}  S_high:{hi[1]:>4}  V_low:{lo[2]:>4}  V_high:{hi[2]:>4}",
            (0, 255, 0), 0.65)

        mask_pct = 100.0 * mask_cur.sum() / max(mask_cur.size, 1)
        put(10, y0 + 55, f"Cur mask: {mask_cur.sum():>6} px  ({mask_pct:.1f}%)   All mask: {mask_all.sum():>6} px",
            (200, 200, 200), 0.5)

        # 操作提示
        put(10, y0 + 82, "[q/w]Hl [a/s]Hh [z/x]Sl [d/c]Sh [r/f]Vl [t/g]Vh  [1/2/3]Grp [←→]Clr [o]Overlay [p]Pause [v]SAVE [ESC]Quit",
            (150, 150, 150), 0.4)

        return display

    def run(self):
        """主循环"""
        while True:
            if not self.paused:
                if not self._update_frame():
                    break

            frame = self.frame
            if frame is None:
                continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h, w = frame.shape[:2]

            # ── 所有 groups 的联合 mask ──
            mask_all = np.zeros((h, w), dtype=np.uint8)
            for g in self.current_groups:
                lo = np.array(g["lower"], dtype=np.uint8)
                hi = np.array(g["upper"], dtype=np.uint8)
                mask_all |= cv2.inRange(hsv, lo, hi)

            # ── 当前 group 的 mask ──
            g = self._current_group()
            lo_cur = np.array(g["lower"], dtype=np.uint8)
            hi_cur = np.array(g["upper"], dtype=np.uint8)
            mask_cur = cv2.inRange(hsv, lo_cur, hi_cur)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask_cur_clean = cv2.morphologyEx(mask_cur, cv2.MORPH_OPEN, kernel)

            # ── 构建三拼图显示 ──
            # 1) 原图 + 叠加
            overlay = frame.copy()
            if self.show_overlay:
                overlay[mask_cur_clean > 0] = overlay[mask_cur_clean > 0] // 2 + np.array([0, 255, 0], dtype=np.uint8) // 2
                contours_all, _ = cv2.findContours(mask_all, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(overlay, contours_all, -1, (255, 0, 0), 1)

            # 2) 当前 group 遮罩 (彩色)
            mask_cur_color = cv2.cvtColor(mask_cur_clean, cv2.COLOR_GRAY2BGR)
            # 3) 全部遮罩
            mask_all_color = cv2.cvtColor(mask_all, cv2.COLOR_GRAY2BGR)

            # 缩放到统一高度
            disp_h = min(400, h)
            scale = disp_h / h
            disp_w = int(w * scale)

            panels = [overlay, mask_cur_color, mask_all_color]
            panels_s = [cv2.resize(p, (disp_w, disp_h)) for p in panels]

            display = np.hstack(panels_s)

            # 添加 HUD
            display = self._draw_hud(display, mask_cur_clean, mask_all)

            # 标签
            labels = ["Original + Overlay (green=cur, blue=all)", "Current Group Mask", "All Groups Mask"]
            lh = 25
            label_bar = np.zeros((lh, display.shape[1], 3), dtype=np.uint8)
            for i, label in enumerate(labels):
                x = i * disp_w + 10
                cv2.putText(label_bar, label, (x, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            display = np.vstack([label_bar, display])

            cv2.imshow(self.win_name, display)

            # ── 键盘 ──
            key = cv2.waitKey(30) & 0xFF
            if key == 27:  # ESC
                break
            if not self._handle_key(key):
                continue

        if not self.use_image:
            self.cap.release()
        cv2.destroyAllWindows()

    def _handle_key(self, key):
        g = self._current_group()
        lo, hi = g["lower"], g["upper"]

        # 判断是否 Shift 按下 (大步长)
        step_size = 5 if self.step == 5 else 1

        # ── H_low ──
        if key == ord("q"):
            lo[0] = max(0, lo[0] - step_size)
        elif key == ord("w"):
            lo[0] = min(180, lo[0] + step_size)
        # ── H_high ──
        elif key == ord("a"):
            hi[0] = max(0, hi[0] - step_size)
        elif key == ord("s"):
            hi[0] = min(180, hi[0] + step_size)
        # ── S_low ──
        elif key == ord("z"):
            lo[1] = max(0, lo[1] - step_size)
        elif key == ord("x"):
            lo[1] = min(255, lo[1] + step_size)
        # ── S_high ──
        elif key == ord("d"):
            hi[1] = max(0, hi[1] - step_size)
        elif key == ord("c"):
            hi[1] = min(255, hi[1] + step_size)
        # ── V_low ──
        elif key == ord("r"):
            lo[2] = max(0, lo[2] - step_size)
        elif key == ord("f"):
            lo[2] = min(255, lo[2] + step_size)
        # ── V_high ──
        elif key == ord("t"):
            hi[2] = max(0, hi[2] - step_size)
        elif key == ord("g"):
            hi[2] = min(255, hi[2] + step_size)
        # ── 步长切换 ──
        elif key == ord("0"):
            self.step = 1 if self.step == 5 else 5
            print(f"[步长] → {self.step}")
        # ── 颜色切换 ──
        elif key == 81:  # 左箭头
            self.color_idx = (self.color_idx - 1) % len(COLOR_NAMES)
            self.group_idx = 0
            print(f"[颜色] → {self.current_color_name}")
            self._print_values()
        elif key == 83:  # 右箭头
            self.color_idx = (self.color_idx + 1) % len(COLOR_NAMES)
            self.group_idx = 0
            print(f"[颜色] → {self.current_color_name}")
            self._print_values()
        # ── Group 切换 ──
        elif key in (ord("1"), ord("2"), ord("3")):
            self.group_idx = int(chr(key)) - 1
            print(f"[Group] → {self.group_idx + 1}")
            self._print_values()
        # ── 叠加 ──
        elif key == ord("o"):
            self.show_overlay = not self.show_overlay
            print(f"[叠加] → {'ON' if self.show_overlay else 'OFF'}")
        # ── 暂停 ──
        elif key == ord("p"):
            self.paused = not self.paused
            print(f"[{'暂停' if self.paused else '运行'}]")
        # ── 保存 ──
        elif key == ord("v"):
            save_config(self.config_path, self.config)
            print(f"[✓] 已保存 (步长={self.step} 时保存)")
        else:
            return True  # 未知按键，不打印

        if key in (ord("q"), ord("w"), ord("a"), ord("s"), ord("z"), ord("x"),
                   ord("d"), ord("c"), ord("r"), ord("f"), ord("t"), ord("g")):
            # 数值变动了，不打印每个键，静默更新
            pass

        return True


def main():
    parser = argparse.ArgumentParser(description="HSV 颜色阈值调参工具 (纯 OpenCV)")
    parser.add_argument("--camera", type=int, default=None)
    parser.add_argument("--image", type=str, default=None)
    parser.add_argument("--config", type=str, default=str(DEFAULT_CONFIG))
    args = parser.parse_args()

    tuner = HSVTuner(
        config_path=args.config,
        camera_id=args.camera,
        image_path=args.image,
    )
    tuner.run()


if __name__ == "__main__":
    main()
