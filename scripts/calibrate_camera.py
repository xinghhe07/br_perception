#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_camera.py — 相机内参 + 畸变标定 (任务书 v1.7 §8.1)
================================================================
产出直接落到 `config/camera_params.yaml`, 并**顺带生成 Fast-Calib 需要的
`qr_params.yaml`** (§8.2 的雷达-相机外参标定依赖本步骤的结果)。

两条路线 —— 本脚本**不做标定算法**, 只做胶水
------------------------------------------------
标定数学交给成熟工具; 本脚本负责的是**它们都不管的那一段**: 结果写回
`config/camera_params.yaml` (ROS params 结构 + 保住注释), 以及生成
Fast-Calib 要的 `qr_params.yaml`。

  【主路径】`kalibr` —— **Kalibr**, 精度更高, 官方推荐。
     ⚠ Kalibr 官方只有 ROS 1; ROS 2 Humble 上要么 Docker + bag 转换
       (rosbags-convert), 要么用 CO-Calib (原生吃 rosbag2)。
     本子命令不代跑 Docker, 只生成所需文件 + 打印可直接照抄的命令。

  【降级】`images` / `capture` —— **内置 OpenCV 棋盘格标定**。
     存在的理由: Kalibr 在 22.04 + Humble 上装不起来时, 这是唯一能跑的路。
     (本机实测: 合成图 22 张, 回收 fx 误差 0.26 px, 重投影 RMS 0.102 px)
     够用于: 去掉占位值、让 ROI 投影 (p_img = K[R|t]P_world) 先动起来。
     **不是首选** —— 有 Kalibr 就用 Kalibr。

子命令
------
  capture  实时采集棋盘格图像 (空格存图 / q 退出 / c 切换相机)
  images   对已有图像目录标定
  kalibr   生成 Kalibr 的 target/rig yaml + 打印全流程命令
  apply    把标定结果写回 config/camera_params.yaml (+ qr_params.yaml)
  show     打印当前 config 里的内外参 (核对用)

用法示例
--------
  python3 calibrate_camera.py capture --camera 0 --out calib_imgs/front
  python3 calibrate_camera.py images  --dir calib_imgs/front --camera front --apply
  python3 calibrate_camera.py kalibr  --camera front
  python3 calibrate_camera.py show
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
# ⚠️ 2026-09-21: 内外参已从 camera_params.yaml **拆到** camera_hardware.yaml
#   —— 那些不是 ROS 节点参数, 混在 params 文件里会让整份加载失败。
#   本脚本读写的是**硬件规格文件**, 不是节点参数文件。
HARDWARE = ROOT / "config" / "camera_hardware.yaml"

# 内置路线默认棋盘格: 内角点数 (列, 行)。棋盘格 9×6 内角点 = 10×7 个方格。
DEFAULT_PATTERN = (9, 6)
# 方格边长 (米) —— ⚠ 换成你们实际打印/采购的靶标尺寸!
DEFAULT_SQUARE = 0.025

# 重投影误差验收线 (任务书 §8.1: 应 < 0.3 像素)
RMS_ACCEPT = 0.3


# ══════════════════════════════════════════════════════════════════════════
# 非 ASCII 路径的读写
# ⚠ cv2.imread / cv2.imwrite 在 Windows 上**处理不了中文路径**, 而且失败时
#   不抛异常、只静默返回 None/False —— 本项目在 C:\Users\<中文>\ 下开发,
#   必须绕开。走 numpy 的 fromfile/tofile + imdecode/imencode。
# ══════════════════════════════════════════════════════════════════════════
def imread_u(path, flags=cv2.IMREAD_COLOR):
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, flags)


def imwrite_u(path, img):
    ext = Path(path).suffix or ".png"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        return False
    buf.tofile(str(path))
    return True


# ══════════════════════════════════════════════════════════════════════════
# YAML 写回 —— 行级替换, 保住文件里的注释和排版 (不用 yaml.dump 覆盖)
# ══════════════════════════════════════════════════════════════════════════
def _fmt_arr(vals, per_line=3, indent=24):
    """把一串浮点排成 config 里那种多行数组写法"""
    out, line = [], []
    for v in vals:
        line.append("%.6f" % v)
        if len(line) == per_line:
            out.append(", ".join(line))
            line = []
    if line:
        out.append(", ".join(line))
    pad = " " * indent
    return (",\n" + pad).join(out)


def _camera_block_bounds(lines, camera):
    """定位 camera_wide / camera_tele 段的 [起, 止) 行号"""
    start = None
    for i, ln in enumerate(lines):
        if ln.strip().startswith(camera + ":"):
            start = i
            break
    if start is None:
        raise SystemExit("config 里找不到相机段: %s" % camera)
    indent = len(lines[start]) - len(lines[start].lstrip())
    for j in range(start + 1, len(lines)):
        s = lines[j]
        if s.strip() and (len(s) - len(s.lstrip())) <= indent:
            return start, j
    return start, len(lines)


def _replace_key_block(lines, lo, hi, key, new_text):
    """在 [lo,hi) 里把 `key:` 开头的那个键整体换成 new_text (单行或多行)"""
    kstart = None
    for i in range(lo, hi):
        if lines[i].strip().startswith(key + ":"):
            kstart = i
            break
    if kstart is None:
        raise SystemExit("段内找不到键: %s" % key)
    kind = len(lines[kstart]) - len(lines[kstart].lstrip())
    kend = kstart + 1
    while kend < hi:
        s = lines[kend]
        if s.strip() and (len(s) - len(s.lstrip())) <= kind:
            break
        kend += 1
    # 不要把尾随空行吃进替换块 —— 那是下一段的间隔, 吃掉会让排版粘连
    while kend - 1 > kstart and not lines[kend - 1].strip():
        kend -= 1
    return lines[:kstart] + [new_text] + lines[kend:]


def write_camera_config(camera, K, dist, K_path, dist_path):
    """把内参写回 config/camera_params.yaml (只动目标相机段的两处值)"""
    lines = HARDWARE.read_text(encoding="utf-8").splitlines(keepends=True)
    lo, hi = _camera_block_bounds(lines, camera)

    # 缩进对齐 config 里 camera_wide/camera_tele 两段的内层级 (8 空格)
    cm = " " * 8 + "camera_matrix: [%s]\n" % _fmt_arr(
        K.reshape(-1), 3, 8 + len("camera_matrix: ["))
    dc = " " * 8 + "distortion_coeffs: [%s]\n" % _fmt_arr(
        dist.reshape(-1), 5, 8 + len("distortion_coeffs: ["))

    lines = _replace_key_block(lines, lo, hi, "camera_matrix", cm)
    lo2, hi2 = _camera_block_bounds(lines, camera)
    lines = _replace_key_block(lines, lo2, hi2, "distortion_coeffs", dc)
    HARDWARE.write_text("".join(lines), encoding="utf-8")


def write_qr_params(camera, K, dist, out_path, shape):
    """Fast-Calib (§8.2) 需要的 qr_params.yaml —— 它的相机内参输入格式。

    shape = (width, height), **必须传真实图像尺寸**。早期版本拿 cx*2 / cy*2
    反推, 主点有标定偏差时会算出 721 这种非整数高度。
    """
    w, h = int(shape[0]), int(shape[1])
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# Fast-Calib 相机内参输入 — 由 calibrate_camera.py 生成\n")
        f.write("# 用法: 在 FAST-Calib 的 config 里把本文件配成 qr_params.yaml\n")
        f.write("camera_name: %s\n" % camera)
        f.write("image_width: %d\n" % w)
        f.write("image_height: %d\n" % h)
        f.write("camera_matrix:\n")
        f.write("  rows: 3\n  cols: 3\n  data: [%s]\n" % _fmt_arr(K.reshape(-1), 3, 8))
        f.write("distortion_model: plumb_bob\n")
        f.write("distortion_coefficients:\n")
        f.write("  rows: 1\n  cols: %d\n  data: [%s]\n" % (dist.size, _fmt_arr(dist.reshape(-1), 5, 8)))


# ══════════════════════════════════════════════════════════════════════════
# 标定核心
# ══════════════════════════════════════════════════════════════════════════
def objp(pattern, square):
    cols, rows = pattern
    p = np.zeros((rows * cols, 3), np.float32)
    p[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    return p * square


def detect(gray, pattern):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)


def run_calib(img_pts, obj_pts, shape):
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, shape, None, None, flags=cv2.CALIB_FIX_K3 if len(img_pts) < 12 else 0
    )
    # 逐图重投影误差, 用于剔除坏图
    # ⚠ 两边都显式 reshape 成 (N,2): findChessboardCorners / projectPoints 在
    #   OpenCV 各版本返回形状不一 (N,1,2) / (N,2), 直接喂 cv2.norm 会 type mismatch。
    per = []
    for i in range(len(obj_pts)):
        proj, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], K, dist)
        a = np.asarray(img_pts[i], dtype=np.float32).reshape(-1, 2)
        b = np.asarray(proj, dtype=np.float32).reshape(-1, 2)
        per.append(float(cv2.norm(a, b, cv2.NORM_L2) / len(a)))
    return rms, K, dist, per


# ══════════════════════════════════════════════════════════════════════════
# 子命令
# ══════════════════════════════════════════════════════════════════════════
def cmd_capture(a):
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        raise SystemExit("打不开相机 %s" % a.camera)
    n = len(list(out.glob("*.png")))
    print("空格存图 / q 退出 / c 切换相机。已存 %d 张 → %s" % (n, out))
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = detect(gray, tuple(a.pattern))
        if c is not None:
            cv2.drawChessboardCorners(frame, tuple(a.pattern), c, True)
        cv2.putText(frame, "saved=%d" % n, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("capture", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(" "):
            p = out / ("%03d.png" % n)
            imwrite_u(p, frame)
            print("  +", p.name, "(检测到角点)" if c is not None else "(未检测到, 仍保存)")
            n += 1
        elif k == ord("q") or k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


def cmd_images(a):
    d = Path(a.dir)
    files = sorted([p for p in d.iterdir() if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp")])
    if not files:
        raise SystemExit("目录里没有图片: %s" % d)
    obj_pts, img_pts, used, shape = [], [], [], None
    for p in files:
        img = imread_u(p)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        c = detect(gray, tuple(a.pattern))
        if c is None:
            print("  · 跳过 (无角点):", p.name)
            continue
        shape = gray.shape[::-1]
        obj_pts.append(objp(tuple(a.pattern), a.square))
        img_pts.append(c)
        used.append(p.name)
    print("\n用于标定: %d / %d 张" % (len(used), len(files)))
    if len(used) < 8:
        raise SystemExit("有效图太少 (<8), 挪不动。至少 20 张、多角度多距离。")

    rms, K, dist, per = run_calib(img_pts, obj_pts, shape)
    print("\n内参 K =\n%s" % np.array2string(K, precision=3, suppress_small=True))
    print("畸变系数 =", np.array2string(dist.reshape(-1), precision=5))
    print("整体重投影误差 RMS = %.4f px" % rms)
    print("逐图:", " ".join("%.3f" % v for v in per))
    worst = int(np.argmax(per))
    if per[worst] > RMS_ACCEPT * 3:
        print("  ⚠ 最差一张 %s = %.3f px —— 建议删掉重标" % (used[worst], per[worst]))

    verdict = "通过" if rms < RMS_ACCEPT else "未达标 (任务书 §8.1 要 < %.1f px)" % RMS_ACCEPT
    print("验收: %s" % verdict)

    if a.apply:
        write_camera_config(a.camera, K, dist, None, None)
        print("已写回:", HARDWARE)
        qp = ROOT / "config" / ("qr_params_%s.yaml" % a.camera)
        write_qr_params(a.camera, K, dist, qp, shape)
        print("已生成 Fast-Calib 内参:", qp)
    else:
        print("\n(加 --apply 才写回 config)")


def cmd_kalibr(a):
    """生成 Kalibr 的 target/rig yaml, 并打印可直接照抄的全流程命令"""
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    tgt = out / "aprilgrid_6x6.yaml"
    tgt.write_text(
        "# Aprilgrid 标定板 —— 供 Kalibr 使用\n"
        "# ⚠ 参数必须与你们实际打印的靶一致, 打印后用尺子复核!\n"
        "target_type: 'aprilgrid'\n"
        "tagCols: 6\ntagRows: 6\n"
        "tagSize: 0.088\ntagSpacing: 0.3\n",
        encoding="utf-8",
    )
    rig = out / "cam-%s.yaml" % a.camera
    rig.write_text(
        "# 单相机 rig —— 供 Kalibr 使用\n"
        "cam0:\n"
        "  camera_model: pinhole\n"
        "  distortion_model: radtan\n"
        "  rostopic: /%s/image_raw\n" % a.camera,
        encoding="utf-8",
    )
    print("已生成:")
    print(" ", tgt)
    print(" ", rig)
    print("""
════════════════════════════════════════════════════════════════════════
Kalibr 全流程 (ROS 2 Humble —— Kalibr 官方只有 ROS 1, 必须走转换)
════════════════════════════════════════════════════════════════════════

1) 录 ROS 2 bag (相机 + IMU 都录, 顺带能做相机-IMU 外参)
     ros2 bag record -o calib_bag /%s/image_raw /livox/imu

2) ROS2 bag → ROS1 bag
     pip3 install rosbags
     rosbags-convert calib_bag --dst calib_bag.bag

3) Kalibr 标内参 (Docker; 官方镜像只有 ROS1)
     docker run --rm -v $PWD:/data -w /data \\
       stereolabs/kalibr:kinetic \\
       kalibr_calibrate_cameras \\
         --bag calib_bag.bag \\
         --topics /%s/image_raw \\
         --models pinhole-radtan \\
         --target aprilgrid_6x6.yaml

4) 把 camchain-*.yaml 里的 intrinsics/distortion_coeffs 抄进 config/camera_params.yaml
   (Kalibr 的 radtan ≡ config 的 plumb_bob, k3 置 0)

5) 生成 Fast-Calib 要的 qr_params.yaml (本脚本 apply 子命令可直接产出)

更低摩擦的替代: CO-Calib (HKUST-Aerial-Robotics) —— 原生吃 rosbag2,
内部拉起 Kalibr Docker, 一条命令。见 README。
════════════════════════════════════════════════════════════════════════
""" % (a.camera, a.camera))


def cmd_show(a):
    import yaml
    txt = HARDWARE.read_text(encoding="utf-8")
    doc = yaml.safe_load(txt)
    p = doc          # 硬件文件是平铺的 (camera_wide / camera_tele / camera_depth)
    for cam in ("camera_wide", "camera_tele"):
        c = p.get(cam, {})
        K = c.get("intrinsics", {}).get("camera_matrix")
        d = c.get("intrinsics", {}).get("distortion_coeffs")
        print("── %s ──" % cam)
        print("  camera_matrix     :", K)
        print("  distortion_coeffs :", d)
        fx = K[0] if K else None
        if fx == 800.0:
            print("  ⚠ fx=800.0 —— 仍是占位值, 未标定")
        ex = c.get("extrinsics", {})
        print("  extrinsics R      :", ex.get("rotation"))
        print("  extrinsics T      :", ex.get("translation"))
        print()


def main():
    ap = argparse.ArgumentParser(description="相机内参标定 (v1.7 §8.1)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add_pattern(p):
        p.add_argument("--pattern", type=int, nargs=2, default=list(DEFAULT_PATTERN),
                       help="棋盘格内角点数 列 行 (默认 %s)" % (DEFAULT_PATTERN,))
        p.add_argument("--square", type=float, default=DEFAULT_SQUARE, help="方格边长 (米)")

    c = sub.add_parser("capture", help="实时采集棋盘格图像")
    c.add_argument("--camera", default="0")
    c.add_argument("--out", default="calib_imgs")
    add_pattern(c)
    c.set_defaults(func=cmd_capture)

    i = sub.add_parser("images", help="对已有图像目录标定")
    i.add_argument("--dir", required=True)
    i.add_argument("--camera", default="camera_wide", choices=["camera_wide", "camera_tele"])
    i.add_argument("--apply", action="store_true")
    add_pattern(i)
    i.set_defaults(func=cmd_images)

    k = sub.add_parser("kalibr", help="生成 Kalibr 配置 + 打印流程")
    k.add_argument("--camera", default="camera_wide", choices=["camera_wide", "camera_tele"])
    k.add_argument("--out-dir", default="calib_kalibr")
    k.set_defaults(func=cmd_kalibr)

    s = sub.add_parser("show", help="打印当前 config 内外参")
    s.set_defaults(func=cmd_show)

    a = ap.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
