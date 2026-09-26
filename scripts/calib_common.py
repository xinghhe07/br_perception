# -*- coding: utf-8 -*-
"""
calib_common.py — 标定脚本的共用件：「这条内参标没标过」+ 跨文件一致性
=========================================================================

为什么单独一个文件
------------------
`calibrate_camera.py`（§8.1 **写**内参）与 `calibrate_lidar_camera.py`
（§8.2 前置检查**读**内参）都要回答同一个问题 —— **config 里那条内参到底标没标过**。

原先两处**各写了一份哨兵，而且两处写的是同一个错**：判据是 `fx == 800.0`。
后果（2026-09-26 发现）：
  `camera_tele` 的 `fx=2380.0` 是「与视场角自洽」的**估计值** ——
  `camera_hardware.yaml` 自己那三行注释写着「**这仍是占位值** —— 真值要标定后回填」，
  但它只是没撞上 `800` 这个数字 ⇒ 在 §8.2 的前置检查里**冒充「✓ 已标定」通过了**。
  而它喂给 FAST-Calib 的 `qr_params.yaml` 会把外参直接标歪 ——
  这正是那道前置检查**唯一存在的理由**。

⇒ 判据收进本文件，两处都调它。**以后改判据只改这一处。**

⚠️ 本模块只判「**没标**」，不判「**标得对不对**」——
   后者要重投影误差（§8.1 的 RMS 验收线，`calibrate_camera.py` 里那条）。
   两者是**互补的**：本模块拦的是"拿着占位值往下走"，它拦的是"标了但标歪了"。
"""

from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

# ⚠️ 2026-09-21: 内外参已从 camera_params.yaml **拆到** camera_hardware.yaml
#   —— 那些不是 ROS 节点参数, 混在 params 文件里会让整份加载失败。
# ⚠️ 导出成 `Path`（不是 str）—— 两个调用方原先都是 `HARDWARE.read_text()`
#   那种用法, 换成 str 会静默改成"没有这个属性"的报错。
HARDWARE = ROOT / "config" / "camera_hardware.yaml"

# 内参的**第二个落点**（见 `runtime_note()`）—— 运行期那份
FUSION = ROOT / "config" / "fusion_params.yaml"

# 内参的**第三个落点**（2026-09-26 才发现）—— 判色那条投影链那份。
# ⚠️ 还兼着**尺寸链**的两环: `image_preprocessor` 的 {wide,tele}_roi 与
#    `sky_block_detector` 的 orig_w/orig_h。
CAM_PARAMS = ROOT / "config" / "camera_params.yaml"

# 要标定的相机。⚠️ **camera_depth（D435i）不在此列** —— 它出厂免标
#   （`camera_hardware.yaml` 的 `intrinsics_source: "factory"`）。
CAMERAS = ("camera_wide", "camera_tele")

# `camera_hardware.yaml` 注释里**逐个点名**的历史占位值。
# ⚠️ 这是判据 ①（精确但只认已知的）—— 不是唯一判据, 见 `placeholder_reasons()`。
KNOWN_PLACEHOLDER_FX = {800.0, 2380.0}


def load_hardware():
    """读 `config/camera_hardware.yaml`（硬件规格文件的顶层是平铺的, 不是 ROS params）"""
    import yaml
    with open(HARDWARE, encoding="utf-8") as f:
        return yaml.safe_load(f)


def configured_resolution(cam):
    """→ (w, h) | None：`camera_hardware.yaml` 里该相机声明的 **resolution**。

    ⚠️ **这个值必须等于"标定时相机实际出的那一档"**，否则标出来的内参
    对应的不是流水线跑的那张图，而**两边都不报错**。
    2026-09-26 实测踩到：`capture` 子命令**不设分辨率** ⇒ 按相机默认档出图
    （本机那台 JR 默认 **640×480**），而声明是 1280×720。
    """
    try:
        res = load_hardware()[cam]["resolution"]
        return int(res[0]), int(res[1])
    except (KeyError, IndexError, TypeError, OSError):
        return None


# ══════════════════════════════════════════════════════════════════════════
# 判据 ①+②：这份内参是不是占位值
# ══════════════════════════════════════════════════════════════════════════
def placeholder_reasons(cam_cfg):
    """→ [str]：该相机段 `intrinsics` **未标定**的证据。空列表 = 看起来是标定过的。

    两条互补的判据，**任一条成立就算占位**：

      ① **历史占位值黑名单**（`KNOWN_PLACEHOLDER_FX`）
         精确、指名道姓，但只认已知的那几个数 —— 换个数字就失效。

      ② **「理想针孔」签名** —— 三条**同时成立**才判：
           · 焦距 fx/fy 正好是整数
           · 主点正好落在图心（= `resolution` / 2）
           · 畸变系数全 0
         真标定的四个值都是**重投影优化出来的浮点数**（如 fx=812.437 / cx=647.3），
         不可能同时正好是整数。
         ⇒ 这条能抓住「换了个数字但根本没标过」的占位值 —— 也就是 ① 漏掉的那类。

    ⚠️ ② 要求**整数组 + 畸变全 0 同时**成立。单看任何一条都会误伤：畸变全 0 在
       **理想镜头**（如仿真相机）上是合法的；某个值凑巧整数也可能发生。
       合起来才是"没动过"的签名。
    """
    reasons = []
    cfg = cam_cfg or {}
    intr = cfg.get("intrinsics") or {}
    K = intr.get("camera_matrix")
    dist = intr.get("distortion_coeffs")

    if not K or len(K) != 9:
        # 读不到就**不能当已标定用** —— 宁可拦下来问一句
        return ["camera_matrix 读不到或长度不是 9 —— 不能当已标定用"]

    fx, fy = float(K[0]), float(K[4])
    cx, cy = float(K[2]), float(K[5])

    # ① 黑名单
    if fx in KNOWN_PLACEHOLDER_FX:
        reasons.append("fx=%g 是 camera_hardware.yaml 注释里点名的占位值" % fx)

    # ② 理想针孔签名
    # ⚠️ 2026-09-26 收紧又放宽了一次, 两个教训都在这里:
    #   · **收紧**: 原判据要求「主点正好落在 `resolution` 的图心」。而 `resolution`
    #     是**相机输出**尺寸, 内参可能是按**裁剪后**那张标的 —— 两个参照系,
    #     tele 的 resolution 改成 1624×1240 后这条就**不再成立**了,
    #     于是 tele 的证据从 2 条掉到 1 条（只靠黑名单 ① 兜着）。
    #   · **放宽**: 改成「**fx/fy/cx/cy 四个都正好是整数**」—— 同样能判"没动过",
    #     而且**不再依赖 resolution**（换个参照系不会失效）, 还能抓住"手改过主点"的占位值。
    #   真标定四个值全是整数的概率 ≈ 0（连无噪声合成图回收的都是 900.001, 见
    #   `calibrate_camera.py` 头注释那条实测）。
    int_like = (fx.is_integer() and fy.is_integer()
                and cx.is_integer() and cy.is_integer()
                and float(K[1]) == 0.0 and float(K[3]) == 0.0
                and float(K[6]) == 0.0 and float(K[7]) == 0.0 and float(K[8]) == 1.0)
    dist_zero = (not dist) or all(float(d) == 0.0 for d in dist)
    if int_like and dist_zero:
        reasons.append("K 是理想针孔: fx/fy/cx/cy 四个都正好是整数 (%g, %g, %g, %g)"
                       " + 畸变全 0 —— 真标定四个值全是整数的概率 ≈ 0"
                       % (fx, fy, cx, cy))
    return reasons


# ══════════════════════════════════════════════════════════════════════════
# 判据 ③：这份内参是按 `resolution` 这一档标的吗
# ══════════════════════════════════════════════════════════════════════════
def principal_point_note(cam_cfg, tol_frac=0.05):
    """→ str：主点与**声明分辨率**图心的一致性。

    ⚠️ 这条拦的是一个**和"占位值"不同的**失败：**在 A 档标定、贴进声明 B 档的段里**。

    实例（本仓库正在踩的那个）：海康 `MV-CS020-10GC` 原生是 **1624×1240**，
    而 `camera_hardware.yaml` 的 `camera_tele` 段声明 **1280×720**。
    若拿原生分辨率标出来的 K 直接贴进来，主点会落在 (812, 620) 附近，
    而本段图心是 (640, 360) —— **差 172 / 260 px**。

    ⚠️ 这是**唯一能从 config 单独验出来**的那条。至于"驱动实际出的是哪一档"，
       config 里**没有事实源**（`mvs_camera` 的 `output_width/output_height` 默认 0
       = 不裁 = 跑原生，而它既不在 launch 也不在任何 yaml 里）⇒ 那条验不了，见文档。

    返回的是给**人看**的一句话（含判定），不是布尔 —— 因为"偏离 172px"和"偏离 3px"
    要的是完全不同的动作。
    """
    cfg = cam_cfg or {}
    K = ((cfg.get("intrinsics") or {}).get("camera_matrix")) or []
    res = cfg.get("resolution")
    if len(K) != 9 or not res or len(res) != 2:
        return "⚠ 主点一致性: 内参或 resolution 不全 —— 跳过"

    w, h = float(res[0]), float(res[1])
    cx, cy = float(K[2]), float(K[5])
    dx, dy = cx - w / 2.0, cy - h / 2.0
    tol = max(tol_frac * w, 8.0)      # 8px 兜底: 低分辨率下图心偏移的绝对量本来就不大

    if abs(dx) <= tol and abs(dy) <= tol:
        return ("主点 (%.1f, %.1f) 与声明分辨率 %d×%d 的图心相符（偏 %.1f, %.1f px）"
                % (cx, cy, int(w), int(h), dx, dy))
    return ("✗ 主点 (%.1f, %.1f) 偏离声明分辨率 %d×%d 的图心 %.1f / %.1f px"
            "（容差 %.0f px）⇒ **这份内参多半是在另一档分辨率上标的**，"
            "贴过来之前要按比例换算（或改回标定时那一档）"
            % (cx, cy, int(w), int(h), dx, dy, tol))


# ══════════════════════════════════════════════════════════════════════════
# 内参的第二个落点 —— 标定完不会自动生效的那一处
# ══════════════════════════════════════════════════════════════════════════
def runtime_note(idx):
    """→ str：`fusion_params.yaml` 里**运行期**那份内参的现状。取不到返回 None。

    ⚠️⚠️ 这是**内参的第二个落点**，也是本仓库一笔明写的债：

      · `calibrate_camera.py apply` 写的是 **camera_hardware.yaml**
      · 而 `mustika_pillar_check` **跑的时候**读的是
        **fusion_params.yaml → mustika_pillar_check.ros__parameters** 的
        `focal_px` / `cx_px` / `cy_px`

    ⇒ **标定完不会自动生效** —— 不人工同步这一份，运行期用的还是旧值，
      而且**不报错**（它只是算得不对）。

    idx: 相机在 `focal_px/cx_px/cy_px` 三个数组里的下标。
         ⚠️ 约定 `0 = wide / 1 = tele`，见 `camera_hardware.yaml` 与
         `mustika_detector.hpp` 的注释 —— 两边必须一致，反了整个融合层就错位。
    """
    import yaml
    try:
        with open(FUSION, encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        p = ((doc.get("mustika_pillar_check") or {}).get("ros__parameters")) or {}
        f_, cx, cy = p["focal_px"][idx], p["cx_px"][idx], p["cy_px"][idx]
    except (OSError, KeyError, IndexError, TypeError):
        return None
    return ("运行期副本 (fusion_params.yaml): fx=%g 主点=(%g, %g)"
            % (float(f_), float(cx), float(cy)))


def resolution_note(cam_cfg):
    """→ str|None：给人看的「这份内参是按这一档分辨率标的吗」。**占位值返回 None**。

    ⚠️ 占位值**不参与**这条校验，两个理由：
      · 它的主点本来就是照图心造的（那正是 `placeholder_reasons()` 判据②的一部分）
        ⇒ 拿它验"是不是按这一档标的"**必然通过**
      · 于是输出里会出现「主点与图心相符」这种**读着像 ✓ 的废话**，
        而那个相机明明刚被判成占位值 —— 自己打自己。
    """
    if placeholder_reasons(cam_cfg):
        return None
    return principal_point_note(cam_cfg)


def sky_block_intrinsics(cam):
    """→ dict|None：该相机在 `sky_block_detector`（camera_params.yaml）里的那份内参。

    ⚠️ **这是内参的第三处落点**（2026-09-26 才发现 —— 原先只记着两处）。
       它键名不叫 `focal_px` 而叫 `fx/fy/cx/cy`, 所以按名字 grep 会漏掉,
       而那正是**判色 40/80 分**那条投影链（`project_spot`）用的。

    ⚠️ 它的 `fx/fy/cx/cy` 是**数组**（下标 0=wide / 1=tele），但 `orig_w/orig_h`
       是**单值** —— 因为眼下两路都是 1280×720。⚠️ **一旦两路分辨率不同,
       这一组就得跟着改成数组**, 否则投影必错（见 `frame_size_note()`）。
    """
    import yaml
    idx = CAMERAS.index(cam)
    try:
        with open(CAM_PARAMS, encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        p = ((doc.get("sky_block_detector") or {}).get("ros__parameters")) or {}
        return {"fx": float(p["fx"][idx]), "fy": float(p["fy"][idx]),
                "cx": float(p["cx"][idx]), "cy": float(p["cy"][idx]),
                "orig_w": int(p["orig_w"]), "orig_h": int(p["orig_h"])}
    except (OSError, KeyError, IndexError, TypeError):
        return None


def _apply_roi_rule(cam_w, cam_h, roi):
    """复刻 `image_preprocessor.cpp` `crop_roi()` 的判定 → (w, h, x, y)。

    ⚠️⚠️ **必须逐字复刻, 包括那个「全图短路」**:

        if (x == 0 && y == 0 && w_raw >= image.cols && h_raw >= image.rows)
            return image;          // 不裁

      于是 `roi = [0, 0, 1280, 720]` 对一张 **1624×1240** 的图**不是全图**
      （`1280 >= 1624` 为假 ⇒ 短路不成立）⇒ **真的会裁**, 且 `Rect(0,0,1280,720)`
      裁的是**左上角**。2026-09-26 抓到的就是这个: 配置看着是"全图",
      实际是**偏轴裁切**, 而且裁完的尺寸恰好等于下游所有占位内参的尺寸 ⇒ 全链自洽地错。
    """
    if not roi or len(roi) < 4:
        return cam_w, cam_h, 0, 0
    x, y, w, h = (int(v) for v in list(roi)[:4])
    if x == 0 and y == 0 and w >= cam_w and h >= cam_h:
        return cam_w, cam_h, 0, 0
    cx, cy = max(0, x), max(0, y)
    cw, ch = min(w, cam_w - cx), min(h, cam_h - cy)
    if cw <= 0 or ch <= 0:
        return cam_w, cam_h, 0, 0
    return cw, ch, cx, cy


def frame_size_note(cam, hw_cfg):
    """→ [str]：这条相机链上**三处尺寸声明**说的是不是同一张图。

    链: `camera_hardware.resolution`（相机输出）
          → `image_preprocessor.{wide,tele}_roi`（裁剪）
          → `sky_block_detector.orig_w/orig_h`（检测器看到的那张）

    ⚠️ 内参必须按**检测器实际看到的那张图**标 —— 链条上任何一处说的不是同一张,
       投影就是系统性偏, 而它**不会报错**（这正是 `sky_block_detector` 那段
       "投影不做 ROI 偏移补偿"注释警告的事）。
    """
    import yaml
    msgs = []
    res = (hw_cfg or {}).get("resolution")
    if not res or len(res) != 2:
        return ["⚠ resolution 未声明 —— 尺寸链无从校验"]
    cam_w, cam_h = int(res[0]), int(res[1])

    roi_key = {"camera_wide": "wide_roi", "camera_tele": "tele_roi"}.get(cam)
    roi = None
    try:
        with open(CAM_PARAMS, encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        pp = ((doc.get("image_preprocessor") or {}).get("ros__parameters")) or {}
        roi = pp.get(roi_key)
    except OSError:
        pass

    eff_w, eff_h, off_x, off_y = _apply_roi_rule(cam_w, cam_h, roi)
    cropped = (eff_w, eff_h) != (cam_w, cam_h)

    msgs.append("尺寸链: 相机输出 %d×%d → %s.%-8s=%s → 检测器看到 %d×%d"
                % (cam_w, cam_h, "image_preprocessor", roi_key or "?", roi, eff_w, eff_h))
    if cropped:
        msgs.append("  ⚠️ **真的被裁了** —— 裁切偏移 (%d, %d), 不是全图"
                    "（`crop_roi` 的全图短路要 `w>=%d && h>=%d`, 现在是 %s）"
                    "⇒ 光轴落在画幅的偏侧, **内参必须按裁剪后这张图标**"
                    % (off_x, off_y, cam_w, cam_h, roi))
    sky = sky_block_intrinsics(cam)
    if sky is None:
        msgs.append("  ⚠️ 读不到 sky_block_detector 的内参 —— 尺寸链断了一环")
    elif (sky["orig_w"], sky["orig_h"]) != (eff_w, eff_h):
        # ⚠️⚠️ 这里**不能报成"投影会偏"** —— 2026-09-26 核实:
        #    `orig_w_` / `orig_h_` 在 `sky_block_detector` 里是**死状态**
        #    （只有 declare/get/clamp 六行, 无任何消费者; 出界兜底走的是运行期
        #      `roi & cv::Rect(0,0,cols,rows)` 求交）⇒ 值不一致**当前无害**。
        #    报成"系统性偏"是**喊狼来了**, 会让人去追一个不影响结果的东西。
        msgs.append("  ⚠️ `sky_block_detector.orig_w/orig_h` = %d×%d, 而检测器看到的是"
                    " %d×%d —— **值不一致, 但当前无害**（该键是**死状态**, 读参后无消费者）"
                    % (sky["orig_w"], sky["orig_h"], eff_w, eff_h))
        msgs.append("     ⚠️ 别『顺手改成 tele 的值』—— 它只有一对**单值**, 改了会把 wide 弄错。"
                    "⚠️ 也**别删**（`bringup_debug.launch.py:219-220` 真的在传它）。"
                    "要修就先改成数组 + 同步 SIM_INTRINSICS 再接进投影")
    else:
        msgs.append("  · `sky_block_detector.orig_w/orig_h` = %d×%d 与检测器看到的一致"
                    "（⚠️ 但该键是死状态, 一致也只是巧合, 不构成校验）"
                    % (sky["orig_w"], sky["orig_h"]))
    return msgs


def sky_block_note(cam):
    """→ str|None：`sky_block_detector`（第三处落点）那份内参的现状"""
    s = sky_block_intrinsics(cam)
    if s is None:
        return None
    return ("第三处副本 (camera_params.yaml · sky_block_detector): "
            "fx=%g cx=%g cy=%g, orig=%d×%d"
            % (s["fx"], s["cx"], s["cy"], s["orig_w"], s["orig_h"]))


def print_runtime_debt(indent="    "):
    """把「三处落点」这笔债印一次（两个脚本共用同一段话, 免得各写一遍各漂一遍）

    ⚠️ 2026-09-26: 原写「**两个**落点」—— 漏了 `camera_params.yaml` 的
       `sky_block_detector`（第三处）。它键名是 `fx/cx/cy` 而不是 `focal_px`,
       按名字 grep 抓不到, 而它正是**判色 40/80 分**那条投影链用的。
    """
    print("%s⚠️ 内参有**三处落点**, 标定脚本只写第一处 —— **标完不会自动生效**:"
          % indent)
    print("%s   ① %s （`calibrate_camera.py apply` 写的就是这里）"
          % (indent, HARDWARE.relative_to(ROOT)))
    print("%s   ② %s → `mustika_pillar_check.ros__parameters`"
          "（融合层 mustika 判「球在柱上」，250 分）" % (indent, FUSION.relative_to(ROOT)))
    print("%s   ③ %s → `sky_block_detector` 的 `fx/fy/cx/cy`"
          "（判色 40/80 分的投影链）" % (indent, CAM_PARAMS.relative_to(ROOT)))
    print("%s   ⇒ 上面每行「副本」都要人工同步。" % indent)
