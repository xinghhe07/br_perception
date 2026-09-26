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
  capture        实时采集棋盘格图 —— **UVC 相机**（走 `/dev/video*`, cv2.VideoCapture）
  capture-topic  实时采集棋盘格图 —— **GigE 相机**（走 ROS2 话题, 需先起 camera_bringup）
  images         对已有图像目录标定
  kalibr         生成 Kalibr 的 target/rig yaml + 打印全流程命令
  apply          把标定结果写回 config/camera_params.yaml (+ qr_params.yaml)
  show           打印当前 config 里的内外参 (核对用)

⚠️ 两个采集子命令**不是二选一, 是两路相机各走各的**（2026-09-26 加）:
   海康 `MV-CS020-10GC` 是 **GigE, 不产生 `/dev/video*`** ⇒ `capture` 够不着它。
   哪路走哪条, 见 `launch/camera_bringup.launch.py` 的台位分工。

用法示例
--------
  # wide = JR（UVC）—— 直接抓
  python3 calibrate_camera.py capture --camera 0 --out calib_imgs/wide
  # tele = 海康（GigE）—— 先起驱动, 再从话题抓
  ros2 launch br_perception camera_bringup.launch.py
  python3 calibrate_camera.py capture-topic --topic /camera_tele/image_raw \\
          --out calib_imgs/tele

  python3 calibrate_camera.py images  --dir calib_imgs/wide --camera camera_wide --apply
  python3 calibrate_camera.py kalibr  --camera camera_wide
  python3 calibrate_camera.py show
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

# ⚠️ 2026-09-21: 内外参已从 camera_params.yaml **拆到** camera_hardware.yaml
#   —— 那些不是 ROS 节点参数, 混在 params 文件里会让整份加载失败。
#   本脚本读写的是**硬件规格文件**, 不是节点参数文件。
#
# ⚠️ 「这条内参是不是占位值」的判据从 calib_common 来 —— 原先本脚本的 `show`
#   子命令**自己写了一份**（只判 `fx == 800.0`），与 §8.2 那份是同一个错。
from calib_common import (  # noqa: E402
    CAMERAS, HARDWARE, configured_resolution, frame_size_note, placeholder_reasons,
    print_runtime_debt, resolution_note, runtime_note, sky_block_note,
)

# 内置路线默认棋盘格: 内角点数 (列, 行)。
# ⚠️ 2026-09-26 改成 **(6, 4)** —— 那是**用户手上实际那块**（"横七格竖5格"
#    ⇒ 内角点横竖各少 1 = 6×4）。
#    ⚠️ 判据：数的是**内角点**（黑白格相交的十字点），**不是方格**。
#    ⚠️ 之前是 (9, 6)：那只是个任意默认值，与实物不符 ⇒ 忘记传 `--pattern`
#       会**整轮抓图都检测不到角点**（窗口不画角标），白跑一趟。
#       传错是**响亮失败**（检测不到），不会静默算歪 —— 但仍然是白跑。
#    ⚠️ 换板子就改这一行，或命令行 `--pattern 列 行` 覆盖。
DEFAULT_PATTERN = (6, 4)
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
        # ⚠️⚠️ 2026-09-26 修：原为 `cv2.norm(a, b, NORM_L2) / len(a)` ——
        #   那是 **L2 范数 ÷ 点数**，**不是 RMS**。真正的 RMS 是
        #   `sqrt(Σd² / N)`，两者差一个 **√N** 因子（6×4 的板 N=24 ⇒ 差 4.9 倍）。
        #   后果：逐图那列**比整体 RMS 小约 5 倍**，而整体 RMS 是 OpenCV 按真 RMS 算的
        #   ⇒ **两列不同源、不能对着看**（现场症状：整体报 1.25，逐图最大才 0.39，
        #     看着自相矛盾）。更实际的是下面那条判据**因此形同虚设**：
        #   `per[worst] > RMS_ACCEPT*3 = 0.9` 换算过去其实是 **4.4 px** 才报警，
        #   于是"点名最差那张"从来不触发。
        per.append(float(np.sqrt(np.sum((a - b) ** 2) / len(a))))
    return rms, K, dist, per


# ══════════════════════════════════════════════════════════════════════════
# 子命令
# ══════════════════════════════════════════════════════════════════════════
def _cv2_device(spec):
    """把 `--camera` 的值变成 `cv2.VideoCapture` 认的东西。

    ⚠️⚠️ **必须是 int，不能是字符串**（2026-09-26 修 —— 这个 bug **一直存在**）:

      `cv2.VideoCapture("0")` 走的是**「文件名 / GStreamer 管线」**那条分支，
      **不是设备索引**。实测它会去开 `/dev/video1`（**错一个号**）然后失败，
      而报错**跟"相机没插"长得一模一样**（`cap_v4l.cpp: open VIDEOIO(V4L2:/dev/video1)`）。

      而 argparse 的 `--camera` 默认值 `"0"` 是**字符串** ⇒ 这条子命令
      **从来没走对过**。（早先它每次失败，我都归因到相机掉线 —— 那是另一码事，
      掉线也真实存在，但**这个才是每次必失败的原因**。）

    ⇒ 纯数字转 `int`；否则当**设备路径**用（这样 `--camera /dev/video2` 也能用）。
    """
    s = str(spec).strip()
    return int(s) if s.isdigit() else s


def cmd_capture(a):
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(_cv2_device(a.camera))
    if not cap.isOpened():
        # ⚠️⚠️ **"打不开"有三档完全不同的原因, 别只猜设备号**（2026-09-26 修）:
        #    ① **被别的进程占着** —— V4L2 默认**独占**! 另一个 `capture`、一个
        #       相机探针、一个没退干净的 ROS 节点都会让这里失败, 而症状**一模一样**。
        #       ⇒ 最容易被忽略、也最容易**自己造成**（本仓库刚踩过: 调试时的探针
        #         把用户同时在跑的采集命令挤掉了）。
        #    ② 设备号不对。
        #    ③ 节点是相机掉线后的**残留**（`ls` 看得到、实际打不开）。
        #    ⚠️ 原实现只想到 ②③, 而且靠**猜**一个号让人试 —— 但 UVC 相机的
        #       偶数号才是采集节点、**奇数号是元数据节点**（打不开是正常的）
        #       ⇒ 猜错方向白跑一趟（实测: 它建议了 `--camera 1`，而 1 正是元数据节点）。
        #    ⇒ 改成**逐个节点实测**（开→读一帧→立刻释放），把事实摆出来。
        avail = sorted(p.name for p in Path("/dev").glob("video*"))
        lines = ["打不开相机 %s（= cv2.VideoCapture(%s)）" % (a.camera, a.camera)]
        if avail:
            lines.append("  逐个节点实测（打开 / 读一帧 / 立刻释放）:")
            for n in avail:
                if not n[5:].isdigit():
                    continue
                icap = cv2.VideoCapture(int(n[5:]))
                if not icap.isOpened():
                    lines.append("    /dev/%-9s ✗ 打不开 —— 被占着？或掉线残留" % n)
                    continue
                ok, fr = icap.read()
                icap.release()
                if ok:
                    lines.append("    /dev/%-9s ✓ **能出图 %dx%d** ⇒ 用它:  --camera %s"
                                 % (n, fr.shape[1], fr.shape[0], n[5:]))
                else:
                    lines.append("    /dev/%-9s · 打得开但读不到帧 —— UVC 的"
                                 "**元数据节点**，正常，别用它" % n)
        else:
            lines.append("  ⚠️ 系统里**一个 /dev/video* 都没有** ⇒ 相机没插上、"
                         "或插了没被内核认到")
            lines.append("     查: `lsusb` 里应能看到摄像头")
            lines.append("     插上后再看 `ls /sys/class/video4linux/`（空 = 还没认到）")
        lines.append("  ⇒ 上面若**一个能出图的都没有**, 先看是不是被占着:")
        lines.append("       fuser -v /dev/video*        # 有输出 = 别的进程占着")
        lines.append("     ⚠️ V4L2 是**独占**的 —— 同时开两个采集会有一边失败,"
                     "而报错跟『没插相机』长得一样")
        lines.append("  ⚠️ 本子命令走 **V4L2**（cv2.VideoCapture）⇒ 只认**产生 "
                     "/dev/video* 的 UVC 相机**。")
        lines.append("     海康 GigE 那一路够不着（它不产生设备节点），用 capture-topic。")
        raise SystemExit("\n".join(lines))

    # ── ⚠️⚠️ 必须先**设**分辨率并把实际值**读回验**（2026-09-26 修）──
    #   不设的话就是"相机默认档"，**与流水线跑的那一档未必相同** ——
    #   本机实测那台 JR 默认出 **640×480**，而 camera_hardware.yaml 声明 **1280×720**。
    #   后果：标出来的内参对应的是 640×480 那张图，拿去算 1280×720 的投影
    #   会**系统性偏**，而且两边都不报错（内参和分辨率是两个文件里的两个数字）。
    #   ⚠️ V4L2 切档要读**若干帧**才生效 ⇒ 下面先读掉几帧再回读。
    want = (a.width, a.height)
    # ⚠️⚠️ **必须显式要 MJPG**（2026-09-26 加）。**帧率由格式决定, 不由尺寸决定**：
    #   本机那台 JR 实测（USB2 口）——
    #       720p MJPG = **30.0 fps**   720p YUYV = **10.0 fps**   480p MJPG = 30.0 fps
    #   ⇒ 降尺寸**一点帧率都换不来**（720p 与 480p 都是 30），
    #     而**不指定格式时 cv2 默认走 YUYV ⇒ 只有 10 fps**。
    #   ⚠️ 与运行时**不一致**就麻烦了：`v4l2_camera` 节点传的是 `pixel_format: 'MJPG'`
    #     （`config/camera_params.yaml`），标定时却是 YUYV —— 预览延迟差 3 倍。
    #   ⚠️ 对**静止的棋盘格**本身无所谓（10fps 也够拍），但预览跟手感差很多，
    #     而且"标定环境和运行环境不一致"是没必要留的差异。
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, want[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, want[1])
    for _ in range(5):
        cap.read()
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != want:
        cap.release()
        raise SystemExit(
            "相机给的档位不对：要 %dx%d，实得 %dx%d\n"
            "  ⚠️ 换档位 = **换内参**，所以这里**不将就** —— 拿这一档标出来的 K\n"
            "     用到另一档上会系统性偏，而且不报错。\n"
            "     · 想用实得的那一档标：显式写 --width %d --height %d，"
            "并同步改\n       `config/camera_hardware.yaml` 的 `resolution`\n"
            "     · 想要这个尺寸但相机给不了 ⇒ 这台相机不支持，换一档或换相机"
            % (want[0], want[1], got[0], got[1], got[0], got[1]))
    n = len(list(out.glob("*.png")))
    # ⚠️ 2026-09-26: 原文写「c 切换相机」—— **那个键从来没实现过**（代码里只有
    #    空格与 q/ESC 两个分支）。按 c 什么都不发生，白按。删掉这句。
    print("空格存图 / q 退出。已存 %d 张 → %s" % (n, out))
    print("  ⚠️ 数**内角点**（黑白格相交的十字点），不是方格 —— 传错了**检测不到**"
          "（窗口里不画角标、存图提示『未检测到』）")
    print("  ⚠️ 图上有彩色角标 = 检测到了 ✓；没有 = 这张是废图，**删掉**")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = detect(gray, tuple(a.pattern))
        # ⚠️ 角标只画在**显示副本**上, 存下来的是**干净帧**。
        #    2026-09-26 修: 原先直接在 `frame` 上画完就存, 于是每张标定图上
        #    **每个角点都盖着一个彩点** —— 而 `images` 子命令会在这张图上
        #    重新跑 findChessboardCorners, 检测会退化甚至失败。
        view = frame.copy()
        if c is not None:
            cv2.drawChessboardCorners(view, tuple(a.pattern), c, True)
        cv2.putText(view, "saved=%d" % n, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("capture", view)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(" "):
            p = out / ("%03d.png" % n)
            imwrite_u(p, frame)          # ← 干净帧（不是 view）
            print("  +", p.name, "(检测到角点)" if c is not None else "(未检测到, 仍保存)")
            n += 1
        elif k == ord("q") or k == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


def cmd_capture_topic(a):
    """从 **ROS 2 话题**采集棋盘格图 —— 给海康 GigE 那一路（`capture` 够不着它）。

    ⚠️ 为什么必须另开一条路
    ----------------------
    `capture` 走 `cv2.VideoCapture`（V4L2）⇒ 只认**产生 `/dev/video*` 的 UVC 相机**。
    海康 `MV-CS020-10GC` 是 **GigE, 不产生设备节点**（`v4l2-ctl --list-devices` 里
    没有它, 见 `doc/模块进度.md` 9.14）⇒ cv2 那条路**根本够不着**。
    它只能先由 `launch/camera_bringup.launch.py` 的 `mvs_camera` 节点出图,
    再从话题上取 —— 就是本子命令。

    ⚠️ QoS **必须 best_effort + depth 1**, 与驱动端**逐字对齐**
    （`mvs_camera_node.cpp:101` / `v4l2_camera_node.cpp:96`）。
    rclpy 的**默认 QoS 是 RELIABLE** ⇒ 与 best_effort 的发布端**不兼容**,
    订阅端**一帧都收不到, 而且不报错**。
    判据: 下面窗口里的 `recv=` 是不是一直 0。

    ⚠️ 采集循环用 `spin_once()` 在**主线程**里转 —— 回调与读帧因此**同线程**,
       没有竞态, 不需要锁, 也不需要额外起 executor。
    """
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import Image

    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    n = len(list(out.glob("*.png")))

    rclpy.init()
    node = rclpy.create_node("calib_capture_topic")
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    bridge = CvBridge()
    state = {"frame": None, "recv": 0, "size_checked": False}

    def on_image(msg):
        # ⚠️⚠️ **校验来图尺寸对不对**（2026-09-26 加）—— 与 `capture` 那条同源的风险：
        #   内参**与分辨率绑定**。若相机没接受请求的 ROI（静默回退），话题上来的就是
        #   另一档；这时标出来的 K 对应的是**另一张图**，而**两边都不报错**。
        #   ⚠️ 这条子命令尤其需要它：长焦那路的 ROI 是 `camera_hardware.yaml` 的
        #      `camera_tele.roi`（相机内裁 + 偏移），而 `mvs_camera` 的偏移/裁切代码
        #      是 2026-09-26 才加的 —— **第一次跑真机之前从没验证过**。
        #   只报一次（逐帧刷会把人埋掉）。
        if not state["size_checked"]:
            state["size_checked"] = True
            want = configured_resolution(a.camera)
            got = (msg.width, msg.height)
            if want is None:
                print("  · 来图 %dx%d（⚠️ config 里没声明 %s 的 resolution，无法校验）"
                      % (got[0], got[1], a.camera))
            elif got != want:
                print("\n  ✗✗ **来图尺寸与配置不符**: 实得 %dx%d，而 `camera_hardware.yaml` "
                      "的 `%s.resolution` 是 %dx%d"
                      % (got[0], got[1], a.camera, want[0], want[1]))
                print("     ⇒ **别用这批图标定** —— 标出来的内参对应的是实得那一档，"
                      "拿到配置那一档上用会系统性偏，而且不报错。")
                print("     ⇒ 先查驱动有没有真的设上 ROI（看 `mvs_camera` 那行 "
                      "『输出 ROI = ...』），再把 camera_hardware.yaml 的\n"
                      "        `roi` / `resolution` 对齐到实际值。")
            else:
                print("  ✓ 来图 %dx%d，与 camera_hardware.yaml 的 resolution 一致"
                      % (got[0], got[1]))
        try:
            state["frame"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:      # noqa: BLE001 —— 单帧转换失败不该让采集整体崩
            print("  ⚠ cv_bridge 转换失败:", exc)
            return
        state["recv"] += 1

    node.create_subscription(Image, a.topic, on_image, qos)
    print("订 %s  (best_effort / depth1)  空格存图 / q 退出。已存 %d 张 → %s"
          % (a.topic, n, out))
    t0, warned = time.time(), False
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            frame = state["frame"]
            if frame is None:
                # ⚠️ 判据不是"节点活着", 而是**有数** —— 同 camera_bringup 的提示
                if not warned and time.time() - t0 > 5.0:
                    warned = True
                    print("\n  ⚠ 5 秒了还没收到一帧。两个常见原因:")
                    print("     ① 驱动没起: ros2 launch br_perception camera_bringup.launch.py")
                    print("     ② 话题名不对。先确认有数（⚠️ 发布端是 best_effort，"
                          "`ros2 topic hz` 默认 RELIABLE，量到 0 不等于没图）:")
                    print("          ros2 topic hz %s --qos best_effort" % a.topic)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            c = detect(gray, tuple(a.pattern))
            # ⚠️ 同 cmd_capture: 角标只画在显示副本上, 存的是干净帧
            view = frame.copy()
            if c is not None:
                cv2.drawChessboardCorners(view, tuple(a.pattern), c, True)
            cv2.putText(view, "saved=%d recv=%d" % (n, state["recv"]),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("capture-topic", view)
            k = cv2.waitKey(1) & 0xFF
            if k == ord(" "):
                p = out / ("%03d.png" % n)
                imwrite_u(p, frame)      # ← 干净帧（不是 view）
                print("  +", p.name, "(检测到角点)" if c is not None else "(未检测到, 仍保存)")
                n += 1
            elif k == ord("q") or k == 27:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        # ⚠️ 退出的两条路都要接住, 否则**交互时会打一坨 traceback**（实测）:
        #    · Ctrl-C   → rclpy 自己的信号处理器先 rcl_shutdown, spin_once 抛
        #                 ExternalShutdownException（不是 KeyboardInterrupt）
        #    · Ctrl-\\ / kill → 同上
        #    退出是**正常路径**, 不是错误。
        print("\n  退出。")
    finally:
        node.destroy_node()
        # ⚠️ 必须判 `rclpy.ok()` —— 信号那条路上 rclpy **已经自己 shutdown 过**了,
        #    再调一次会抛 RCLError "rcl_shutdown already called on the given context"
        #    （实测), 而且它是在 finally 里抛, 会把真正的退出码盖掉。
        if rclpy.ok():
            rclpy.shutdown()
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


def cmd_probe(a):
    """**实时**打亮度和对比度 —— 调曝光用，不用靠眼睛看采集窗口。

    ⚠️ 为什么需要它（2026-09-26 加）：采集窗口里**没有数字**，而
       "看着还行"和"角点定位准不准"是两件事 —— 实测踩过：画面亮度 **5.8/255**
       （几乎全黑）时 `findChessboardCorners` **照样报"检测到角点"**
       （因为 `detect()` 带 `CALIB_CB_NORMALIZE_IMAGE`，检测前会拉对比度），
       但 `cornerSubPix` 在**原图**上做亚像素精化 ⇒ **角点位置全是噪声**,
       标出来的 RMS 1.8 px 起。**眼睛看不出这个。**

    判据（拍标定图之前先对到这里）:
        亮度 80~150   对比度(std) > 40   过曝(≥250) < 1%
    """
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("calib_probe")
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    bridge = CvBridge()
    state = {"frame": None, "recv": 0, "size_checked": False}

    def on_image(msg):
        if not state["size_checked"]:
            state["size_checked"] = True
            want = configured_resolution(a.camera)
            if want and (msg.width, msg.height) != want:
                print("  ⚠️ 来图 %dx%d ≠ 配置的 %dx%d —— 尺寸不对，先对齐再谈曝光"
                      % (msg.width, msg.height, want[0], want[1]))
        try:
            state["frame"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:      # noqa: BLE001
            return
        state["recv"] += 1

    node.create_subscription(Image, a.topic, on_image, qos)
    print("订 %s   每 0.5 秒打一行，Ctrl-C 退出。" % a.topic)
    print("  ⚠️ **先看「亮暗差」那一列**（黑白分明程度）—— 它才决定角点定位准不准；")
    print("     亮度均值低只是背景暗, 不影响亚像素精度。")
    print("  门槛: 亮暗差 > 150   过曝 < 3%   (亮度均值 25~215 即可)")
    print("  %-8s %-8s %-8s %-8s %s" % ("亮度", "对比度", "亮暗差", "过曝%", "判定"))
    last, n = 0, 0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            n += 1
            if state["recv"] == 0:
                if n % 20 == 0:
                    print("  ⚠️ 还没收到帧 —— 驱动起了吗？话题名对吗？")
                continue
            if n % 10:           # 约每 0.5 秒打一次
                continue
            img = state["frame"]
            if img is None:
                continue
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            m, s = float(g.mean()), float(g.std())
            sat = float((g >= 250).mean() * 100)
            hi, lo = np.percentile(g, 95), np.percentile(g, 5)
            rng = hi - lo
            # ⚠️⚠️ 判据以**黑白分明程度（`亮暗差`）为主**，**不是亮度均值**
            #   （2026-09-26 修）。原实现以「亮度 < 40 ⇒ 太暗」为主判据，
            #   而实测出现过 **亮度 40 / 亮暗差 230** 的图 —— 黑白黑白分明、
            #   亚像素定位完全够用，却被判成"太暗"。
            #   决定 `cornerSubPix` 精度的是**角点处的梯度**，与整幅的均值无关
            #   （均值低只是背景暗，黑格白格的落差才是信号）。
            if rng < 80:
                v = "❌ 对比度不足 —— 角点会是噪声"
            elif sat > 3.0:
                v = "⚠️ 过曝/反光 %.1f%%（白格打爆会糊掉边界）" % sat
            elif m < 25:
                v = "⚠️ 整体偏暗（亮暗差够就还能用，但别再降曝光）"
            elif m > 215:
                v = "⚠️ 整体偏亮"
            else:
                v = "✅ 可以拍"
            # \r 原地刷新，免得刷屏
            print("  %-8.1f %-8.1f %-8.1f %-8.2f %s" % (m, s, hi - lo, sat, v))
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n  退出。")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def cmd_show(a):
    import yaml
    txt = HARDWARE.read_text(encoding="utf-8")
    doc = yaml.safe_load(txt)
    p = doc          # 硬件文件是平铺的 (camera_wide / camera_tele / camera_depth)
    any_placeholder = False
    for i, cam in enumerate(CAMERAS):
        c = p.get(cam, {}) or {}
        intr = c.get("intrinsics") or {}
        K = intr.get("camera_matrix")
        d = intr.get("distortion_coeffs")
        print("── %s ──" % cam)
        print("  camera_matrix     :", K)
        print("  distortion_coeffs :", d)
        # ⚠️ 判据在 calib_common.placeholder_reasons() —— 本处原先只判 `fx == 800.0`,
        #    tele 的 2380（同样是占位值）因此**不会被报出来**。
        reasons = placeholder_reasons(c)
        if reasons:
            any_placeholder = True
            print("  ⚠ 占位值（未标定）:")
            for r in reasons:
                print("      · %s" % r)
        else:
            print("  ✓ 内参无占位特征")
        # ⚠️ 占位值不跑主点校验（占位值的主点就是照图心造的, 必然通过）⇒ 返回 None
        note = resolution_note(c)
        if note:
            print("  %s" % note)
        for line in frame_size_note(cam, c):
            print("  %s" % line)
        note = runtime_note(i)
        if note:
            print("  %s" % note)
        note = sky_block_note(cam)
        if note:
            print("  %s" % note)
        ex = c.get("extrinsics") or {}
        print("  extrinsics R      :", ex.get("rotation"))
        print("  extrinsics T      :", ex.get("translation"))
        print()
    if any_placeholder:
        print_runtime_debt(indent="  ")


def main():
    ap = argparse.ArgumentParser(description="相机内参标定 (v1.7 §8.1)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add_pattern(p):
        p.add_argument("--pattern", type=int, nargs=2, default=list(DEFAULT_PATTERN),
                       help="棋盘格内角点数 列 行 (默认 %s)" % (DEFAULT_PATTERN,))
        p.add_argument("--square", type=float, default=DEFAULT_SQUARE, help="方格边长 (米)")

    c = sub.add_parser("capture", help="实时采集棋盘格图像 (UVC, 走 /dev/video*)")
    c.add_argument("--camera", default="0")
    c.add_argument("--out", default="calib_imgs")
    # ⚠️ 默认值取自 `camera_hardware.yaml` 的 `camera_wide.resolution` —— **单一事实源**。
    #    `capture` 这条只管 UVC 那一路（= wide）。**改了要两边一起改**：
    #    内参按分辨率定，不一致时投影系统性偏且不报错。
    _w, _h = configured_resolution("camera_wide") or (1280, 720)
    c.add_argument("--width", type=int, default=_w,
                   help="采集宽度（默认取 camera_hardware.yaml 的 camera_wide.resolution[0] = %d）" % _w)
    c.add_argument("--height", type=int, default=_h,
                   help="采集高度（默认取 camera_hardware.yaml 的 camera_wide.resolution[1] = %d）" % _h)
    add_pattern(c)
    c.set_defaults(func=cmd_capture)

    ct = sub.add_parser("capture-topic",
                        help="从 ROS2 话题采集 (GigE 那路走这条; 需先起 camera_bringup)")
    ct.add_argument("--topic", default="/camera_tele/image_raw")
    ct.add_argument("--out", default="calib_imgs")
    # ⚠️ 只为**校验来图尺寸**用（对着 `camera_hardware.yaml` 的 resolution）——
    #    这条子命令**不设**尺寸，尺寸由驱动那头决定（launch 读同一个 yaml 喂给节点）。
    ct.add_argument("--camera", default="camera_tele", choices=["camera_wide", "camera_tele"],
                    help="用来查 config/camera_hardware.yaml 里该路的 resolution，校验来图尺寸")
    add_pattern(ct)
    ct.set_defaults(func=cmd_capture_topic)

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

    pb = sub.add_parser("probe", help="实时打亮度/对比度（调曝光用，不用靠眼睛看）")
    pb.add_argument("--topic", default="/camera_tele/image_raw")
    pb.add_argument("--camera", default="camera_tele",
                    choices=["camera_wide", "camera_tele"])
    pb.set_defaults(func=cmd_probe)

    s = sub.add_parser("show", help="打印当前 config 内外参")
    s.set_defaults(func=cmd_show)

    a = ap.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
