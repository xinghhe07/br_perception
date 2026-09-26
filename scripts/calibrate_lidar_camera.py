#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_lidar_camera.py — 雷达 ↔ 相机外参标定 (任务书 v1.7 §8.2)
====================================================================
用 **FAST-Calib** (hku-mars, arXiv 2507.17210) 解外参 —— 不是 Kalibr。
Kalibr 只做 camera-camera / camera-IMU, **做不了雷达-相机**。

选型理由
--------
  * 与 Point-LIO / FAST-LIO 同一实验室 (HKU MARS), 生态一致
  * **ROS 2 原生**: `OpenMind/FAST-Calib` 已在 Humble 上测过, 不需要 Docker
  * 无需初始外参, ~1 秒出结果, 点对点误差 < 6.5 mm
  * 原理: 专用靶上的圆孔 —— 雷达侧 RANSAC 分割 + 椭圆拟合取孔心,
    相机侧 4 个 ArUco 定孔心, Kabsch/SVD 解 3D-3D 对应

⚠️ 三条前置
-----------
  1. **相机内参必须先标** (§8.1) —— FAST-Calib 要 qr_params.yaml 当输入。
     本脚本 `check` 子命令会先验证 config 里的内参不是占位值。
  2. **专用靶需提前加工** —— 仓库提供 CAD, 基于 velo2cam_calibration 设计
     (4 个 ArUco + 圆孔)。**有加工周期, 别等要用才提需求。**
  3. **至少 3 个场景** —— 正对 / 右斜 / 左斜, 否则只能用单场景标定, 精度差。

子命令
------
  check    前置检查 (内参是否已标 / fast_calib 是否装 / 靶尺寸)
  record   录 ROS2 bag (雷达 + 相机)
  scenes   打印三场景采集指引
  prepare  生成 FAST-Calib 需要的配置 (qr_params.yaml 等)
  run      调 FAST-Calib 标定
  apply    把 calib_result.txt 的 Rcl/Pcl 写回 config/camera_params.yaml

用法
----
  python3 calibrate_lidar_camera.py check
  python3 calibrate_lidar_camera.py record --scene front
  python3 calibrate_lidar_camera.py run --scene front
  python3 calibrate_lidar_camera.py apply --result calib_result.txt
"""
import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

# ⚠️ 内外参已拆到 camera_hardware.yaml（不是 ROS 节点参数文件）。
#   路径与判据都从 calib_common 来 —— 那条判据原先在本脚本里**写错过一次**
#   （只判 `fx == 800.0`，于是 tele 的 2380 冒充"已标定"通过了本检查），
#   现在收在一处，见那个模块的头注释。
from calib_common import (  # noqa: E402
    CAMERAS, HARDWARE, frame_size_note, placeholder_reasons,
    print_runtime_debt, resolution_note, runtime_note, sky_block_note,
)

# FAST-Calib 的 ROS2 fork —— 已测 Humble
FASTCALIB_REPO = "https://github.com/OpenMind/FAST-Calib"
FASTCALIB_ALT = "https://github.com/Mark-tz/FAST-Calib_ROS2"

# 靶面到雷达的建议距离 (m) —— 太远点云稀疏, 圆孔拟合不出来
TARGET_DIST_RANGE = (2.0, 6.0)


def _read_config():
    import yaml
    return yaml.safe_load(HARDWARE.read_text(encoding="utf-8"))


def cmd_check(a):
    print("═══ §8.2 前置检查 ═══\n")
    ok = True

    # 1) 相机内参 —— §8.2 的硬前置
    # ⚠️ 判据在 calib_common.placeholder_reasons()。本处原先判的是 `fx == 800.0`,
    #    tele 的 2380（同样是占位值）就此**冒充通过** —— 见那个模块的头注释。
    doc = _read_config()          # 硬件文件是平铺的
    print("  ── 内参体检（§8.1 的产出，也就是本步的输入）──")
    for i, cam in enumerate(CAMERAS):
        cfg = doc.get(cam) or {}
        reasons = placeholder_reasons(cfg)
        if reasons:
            print("    ✗ %s —— **占位值，未标定**，%d 条证据:" % (cam, len(reasons)))
            for r in reasons:
                print("        · %s" % r)
            print("      ⇒ 先跑 §8.1: python3 calibrate_camera.py images "
                  "--dir <标定图> --camera %s --apply" % cam)
            ok = False
        else:
            K = ((cfg.get("intrinsics") or {}).get("camera_matrix")) or []
            print("    ✓ %s 看起来是标定值（fx=%.3f，无占位特征）" % (cam, K[0]))
            # ⚠️ 用『』而不是 ASCII 引号 —— 这是**字符串字面量内部**, 双引号会截断它
            print("      ⚠️ 本检查**只判『没标』**，不判『标得对不对』 —— "
                  "重投影 RMS 要回 §8.1 的输出看")
        # ⚠️ 占位值不跑主点校验（占位值的主点就是照图心造的, 必然通过）⇒ 返回 None
        note = resolution_note(cfg)
        if note:
            print("      %s" % note)
        for line in frame_size_note(cam, cfg):
            print("      %s" % line)
        note = runtime_note(i)
        if note:
            print("      %s" % note)
        note = sky_block_note(cam)
        if note:
            print("      %s" % note)
    if not ok:
        print_runtime_debt(indent="    ")
    print()

    # 2) fast_calib 是否装了
    # ⚠ ros2 不在 PATH 时 subprocess 抛 FileNotFoundError (不是返回非零),
    #   必须接住, 否则检查本身就崩。
    try:
        r = subprocess.run(["ros2", "pkg", "prefix", "fast_calib"],
                           capture_output=True, text=True)
        found = r.returncode == 0
    except FileNotFoundError:
        print("  ✗ 这台机器上没有 `ros2` —— 标定要在装了 ROS 2 Humble 的机器人主机上跑")
        print("     (本脚本可在任意机器上生成配置, 但 record/run 必须在那台上执行)")
        found = None
    if found is True:
        print("  ✓ fast_calib 已安装:", r.stdout.strip())
    elif found is False:
        print("  ✗ 没找到 fast_calib 包。安装:")
        print("      git clone %s" % FASTCALIB_REPO)
        print("      # 放进 colcon 工作空间, 然后 colcon build")
        print("    备选 fork: %s" % FASTCALIB_ALT)
        ok = False
    print()

    # 3) 靶
    print("  靶标: 需 FAST-Calib 专用靶 (仓库提供 CAD)")
    print("        ⚠ 加工有周期, 现在就提需求")
    print("        建议摆放距离 %.1f–%.1f m (%s)" % (
        TARGET_DIST_RANGE[0], TARGET_DIST_RANGE[1], "太远点云稀疏, 圆孔拟合不出"))
    print()

    # 4) docker? 不需要
    print("  Docker: 不需要 (ROS2 fork 原生跑)")
    print()
    print("结论:", "前置齐了, 可以开始" if ok else "有前置没满足 —— 见上面 ✗")
    return 0 if ok else 1


def cmd_record(a):
    scene = a.scene
    out = Path(a.out) / ("calib_%s" % scene)
    print("录包 → %s" % out)
    print("""
采集要点:
  · 靶放在雷达与相机**同时看得见**的位置, 距离 %.1f–%.1f m
  · 场景 %s —— %s
  · 保持静止, 录 3–5 秒即可 (FAST-Calib 只需单帧点云 + 单张图像)
""" % (TARGET_DIST_RANGE[0], TARGET_DIST_RANGE[1], scene, a.hint))
    cmd = ["ros2", "bag", "record", "-o", str(out), a.lidar_topic, a.cam_topic]
    print("执行:", " ".join(cmd))
    if a.dry_run:
        return 0
    return subprocess.run(cmd).returncode


def cmd_scenes(a):
    print("""
═══ FAST-Calib 三场景采集 ═══

至少 3 个**不同朝向**的场景, 多场景联合标定才稳。推荐:

  场景 1  front  靶面**正对**相机, 雷达与相机都能看到完整靶
  场景 2  right  靶整体**右斜** ~30–45°  (绕竖直轴)
  场景 3  left   靶整体**左斜** ~30–45°

每场景:  录 3–5 秒静止数据 → 存成独立 bag
          python3 calibrate_lidar_camera.py record --scene front

⚠ 常见坑
  · 靶太远 → 点云稀疏, 圆孔拟合不出 → 距离控在 %.1f–%.1f m
  · 靶太正 (三个场景几乎同一个朝向) → 联合标定退化成单场景, 精度掉
  · 相机曝光没锁 → ArUco 检测不稳, 先按 camera_params.yaml 锁手动曝光
""" % TARGET_DIST_RANGE)


def cmd_prepare(a):
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    src = ROOT / "config" / ("qr_params_%s.yaml" % a.camera)
    if not src.exists():
        raise SystemExit(
            "找不到 %s\n先跑: python3 calibrate_camera.py images --dir <标定图> --camera %s --apply"
            % (src, a.camera))
    dst = out / "qr_params.yaml"
    shutil.copyfile(src, dst)
    print("已就位:", dst)
    print("""
FAST-Calib 还需要的两项 (在它自己的 config 里配):
  1. 靶的物理参数 (圆孔直径 / ArUco 尺寸) —— 照你们加工的实物填
  2. 点云距离滤波器 —— 仓库给了 scripts/distance_filter_tool.py 生成
     建议范围: 靶到雷达 %.1f–%.1f m
""" % TARGET_DIST_RANGE)


def cmd_run(a):
    bag = Path(a.out) / ("calib_%s" % a.scene)
    print("执行 FAST-Calib 单场景标定:")
    cmd = ["ros2", "launch", "fast_calib", "calib.launch.py",
           "bag_path:=%s" % bag, "image_path:=%s" % a.image]
    print("  ", " ".join(cmd))
    print()
    print("多于 3 个场景都录完后, 改用多场景联合标定:")
    print("   ros2 launch fast_calib multi_calib.launch.py")
    print()
    if a.dry_run:
        return 0
    return subprocess.run(cmd).returncode


def _num_list(text):
    inner = text.strip().strip("[]")
    return [float(x) for x in re.split(r"[,\s]+", inner) if x]


def cmd_apply(a):
    p = Path(a.result)
    if not p.exists():
        raise SystemExit("找不到结果文件: %s" % p)
    txt = p.read_text(encoding="utf-8", errors="replace")

    mR = re.search(r"Rcl\s*[:=]\s*(\[[^\]]*\]|[\d\s.,+-]+)", txt)
    mP = re.search(r"Pcl\s*[:=]\s*(\[[^\]]*\]|[\d\s.,+-]+)", txt)
    if not mR or not mP:
        print("⚠ 解析不出 Rcl / Pcl。请人工核对, 原文如下:\n")
        print(txt)
        return 1

    R = _num_list(mR.group(1))
    P = _num_list(mP.group(1))
    if len(R) != 9:
        raise SystemExit("Rcl 应有 9 个数 (3×3 行主序), 实际 %d 个: %s" % (len(R), R))
    if len(P) != 3:
        raise SystemExit("Pcl 应有 3 个数 (x,y,z), 实际 %d 个: %s" % (len(P), P))

    # FAST-Calib 的 Rcl/Pcl 语义与 config 一致: P_world = R * P_cam + T
    print("解析结果:")
    print("  Rcl (3×3 行主序) =", R)
    print("  Pcl (x,y,z)      =", P)

    lines = HARDWARE.read_text(encoding="utf-8").splitlines(keepends=True)
    lo = hi = None
    for i, ln in enumerate(lines):
        if ln.strip().startswith(a.camera + ":"):
            lo = i
            break
    if lo is None:
        raise SystemExit("config 里找不到相机段: %s" % a.camera)
    ind = len(lines[lo]) - len(lines[lo].lstrip())
    for j in range(lo + 1, len(lines)):
        s = lines[j]
        if s.strip() and (len(s) - len(s.lstrip())) <= ind:
            hi = j
            break
    hi = hi or len(lines)

    def repl(lines, lo, hi, key, new):
        ks = None
        for i in range(lo, hi):
            if lines[i].strip().startswith(key + ":"):
                ks = i
                break
        if ks is None:
            raise SystemExit("段内找不到键: %s" % key)
        kd = len(lines[ks]) - len(lines[ks].lstrip())
        ke = ks + 1
        while ke < hi:
            s = lines[ke]
            if s.strip() and (len(s) - len(s.lstrip())) <= kd:
                break
            ke += 1
        while ke - 1 > ks and not lines[ke - 1].strip():
            ke -= 1
        return lines[:ks] + [new] + lines[ke:]

    rtxt = " " * 8 + "rotation: [%s,\n" % ", ".join("%.9f" % v for v in R[:3])
    rtxt += " " * 19 + "%s,\n" % ", ".join("%.9f" % v for v in R[3:6])
    rtxt += " " * 19 + "%s]\n" % ", ".join("%.9f" % v for v in R[6:9])
    ttxt = " " * 8 + "translation: [%.6f, %.6f, %.6f]\n" % tuple(P)

    lines = repl(lines, lo, hi, "rotation", rtxt)
    lo2 = next(i for i, ln in enumerate(lines) if ln.strip().startswith(a.camera + ":"))
    lines = repl(lines, lo2, len(lines), "translation", ttxt)
    HARDWARE.write_text("".join(lines), encoding="utf-8")
    print("\n已写回:", HARDWARE)
    print("验收: 打开 FAST-Calib 产出的 colored_cloud.pcd 看颜色是否对齐")
    return 0


def main():
    ap = argparse.ArgumentParser(description="雷达↔相机外参标定 — FAST-Calib (v1.7 §8.2)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("check", help="前置检查")
    c.set_defaults(func=cmd_check)

    s = sub.add_parser("scenes", help="三场景采集指引")
    s.set_defaults(func=cmd_scenes)

    r = sub.add_parser("record", help="录 ROS2 bag")
    r.add_argument("--scene", default="front", choices=["front", "right", "left"])
    r.add_argument("--hint", default="靶面正对相机")
    r.add_argument("--lidar-topic", default="/livox/lidar")
    r.add_argument("--cam-topic", default="/camera_wide/image_raw")
    r.add_argument("--out", default="calib_scenes")
    r.add_argument("--dry-run", action="store_true")
    r.set_defaults(func=cmd_record)

    p = sub.add_parser("prepare", help="生成 FAST-Calib 配置")
    p.add_argument("--camera", default="camera_wide")
    p.add_argument("--out-dir", default="calib_fastcalib")
    p.set_defaults(func=cmd_prepare)

    u = sub.add_parser("run", help="调 FAST-Calib")
    u.add_argument("--scene", default="front")
    u.add_argument("--out", default="calib_scenes")
    u.add_argument("--image", required=True, help="与 bag 对应的相机图像")
    u.add_argument("--dry-run", action="store_true")
    u.set_defaults(func=cmd_run)

    a = sub.add_parser("apply", help="把 Rcl/Pcl 写回 config")
    a.add_argument("--result", required=True, help="FAST-Calib 的 calib_result.txt")
    a.add_argument("--camera", default="camera_wide")
    a.set_defaults(func=cmd_apply)

    args = ap.parse_args()
    sys.exit(args.func(args) or 0)


if __name__ == "__main__":
    main()
