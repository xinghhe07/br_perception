#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// sky_block_detector.hpp — 天空方块「顶面颜色」判定（任务书 §3.3）
///
/// 依赖: OpenCV (core + imgproc) + cv_bridge + color_utils
/// 职责:
///   1. 订阅双路预处理图像 + 一个「顶面候选区域」
///   2. 在候选区域内做红 vs 蓝 HSV 像素计数
///   3. 按阈值判 RED / BLUE / UNKNOWN
///   4. 发布 /perception/camera/sky_blocks_{wide,tele} (CameraSkyBlock)
///   5. 发布诊断
/// ═══════════════════════════════════════════════════════════════════════════════
///
/// ── 为什么它值 40/80 分（规则 3.5.6 原文，一手）──
///   "Sky Blocks are scored based on the color of their **top surface**.
///    A team only receives points for a Sky Block if its top surface color
///    matches the team's designated color."
///   "**3.5.7** The team whose Sky Block color is **facing up** on top of a tower
///    is considered to have completed that tower (Complete Tower), **even if the
///    Earth Blocks beneath it were initially placed by the opposing team**."
///   "**3.5.8** Teams are permitted to **flip** or change the position of a Sky
///    Block on any tower ... effectively 'stealing' the tower from the opponent."
///   ⇒ 判据是**朝上那一面**；而且**对手会来翻**，所以这不是一次性判定，
///     是"**归属会不会变**"的持续监视（融合层要按 keep_on_spot_frames 保持跟踪）。
///   ⇒ 雷达对颜色无能为力（`building_spot_analyzer.cpp` 明写"雷达无法判断天空方块
///     朝上颜色"，只置 `needs_visual_check`）—— **这 40/80 分只能靠视觉**。
///
/// ── ⚠️⚠️ 本模块**只做判色**，不做检测（2026-09-23 的重要切分）──
///   `doc/模型训练指南.md` §2 定死了分工（原文）：
///     "**颜色不交给网络** —— YOLO 只负责: '这里有个方块'(bbox)；
///       HSV 后处理: '它是红的还是蓝的'(在 bbox 内统计)"
///     "`sky_block` 的特殊之处：顶面颜色决定得分。所以推理时除了 bbox，
///       还要确定'哪个面是顶面' —— **靠 bbox 的几何 + 透视估计，不是靠网络**"
///   ⇒ "找方块"（YOLO26n ONNX）与"判颜色"（本文件）是两件事，**可以分开交付**。
///
///   ⚠️ **为什么现在必须先切**：ONNX 那套基础设施在这个仓库里**一样都没有** ——
///      `CMakeLists.txt` 只有 `find_package(OpenCV)`（无 onnxruntime）、
///      `package.xml` 无 ort 依赖、**全仓零个模型文件**、`camera_params.yaml` 里
///      `model_path` / `model_backend` **键都不存在**（`doc/模型训练指南.md` §6
///      写"现在是空串，要填" —— 那个键**从来没有过**，指南是按设计意图写的）。
///      而模型训练卡 P2-15（要相机 + H200S 规格）。
///   ⇒ 硬把 YOLO 写进来 = 一堆**编不过也无从测**的死代码，正是本仓库最恨的"假能力"。
///
/// ── ⚠️⚠️ 三条必须守住的边界 ──
///
///   **① "判不出来"必须与"没有方块"分开**
///   输出四档（与 `BuildingSpot.msg` 的 `top_color` **逐值一致**）：
///     `0 = NONE`    没检测到方块
///     `1 = RED`     顶面判为红
///     `2 = BLUE`    顶面判为蓝
///     `3 = UNKNOWN` **检到了方块，但顶面颜色判不出来**
///   ⚠️ `3` 与 `0` 合并成"都不是"是**最危险的一种省事**：
///     融合层会把"有方块但没看清"当成"这里没方块"，于是**整座塔的归属凭空消失**
///     ——而它可能正是我方的那座。数值上都是"不是红也不是蓝"，现场要做的事完全不同。
///
///   **② 判色的分母是"红+蓝像素"，不是"区域总像素"**
///   §3.3 原文写"某一颜色占比 > 70%""两者接近（45%-55%）"。⚠️ **"45%-55%"这个说法
///   只有在两个占比加起来≈1 时才成立**（若分母是区域总像素，红 0.2 / 蓝 0.2
///   根本不能叫"接近"）⇒ 取 `red/(red+blue)`。
///   ⚠️ 但**只看这个比值不够**：一个几乎全黑、只有 3 个红点的区域，比值是 100% 红。
///      ⇒ 另加一道 `min_color_fill`：`(红+蓝)/总像素` 低于它就判 UNKNOWN。
///        这条**规格里没写**，是补的（理由写在这里，别让它只存在于代码里）。
///
///   **③ 55%~70% 是规格的未定义段 → 保守判 UNKNOWN**
///   §3.3 只定义了两段：">70% 定色"与"45-55% 判 UNKNOWN"。中间那段没写。
///   取**最保守读法**：**只有 >70% 才定色，其余一律 UNKNOWN**。
///   理由同边界①：判错的代价是**把塔的归属报反**（比不报更糟）。
///
/// ── ✅ 几何前提（**2026-09-24 已裁定**：顶面是**纯色**。「每块红蓝各半」是误读）──
///
///   ⚠️⚠️ **前面所有"待定、定了要回来改"的话都作废了 —— 这里是结论。**
///
///   **裁定**：天空方块**正放时顶面纯红、倒放（翻转 180°）时顶面纯蓝**。
///
///   **依据**（用户 2026-09-24 引规则原文，不是推断）：
///     ① 官方定义 "colored red on **one side** and blue on **the other**" =
///        **相对的两面**一红一蓝，**不是**"每一面各半";
///     ② 规则 3.5.6 "scored based on the color of their **top surface**" +
///        3.5.8 "flip … to ensure their **color faces upward**" —— 这套判据
///        **只有在"顶面纯色、翻转即换色"时才执行得了**。若顶面恒半红半蓝，
///        **裁判自己都判不出分**，规则不可能这样写;
///     ③ 官方总装配的"每块 = 整块 + 半块"是**沿竖直方向**拼接的两个半部 ⇒
///        "红蓝各半"说的是**块由两个半部构成**，不是顶面被分成两半。
///
///   **⇒ 三条被推翻的旧推论（别再照它们办）**：
///     · ~~"判色看的是可见的那个色面"~~ ⇒ 只看**顶面**。侧面不是判据。
///     · ~~"掠射角门控暂时不需要"~~ ⇒ **门控是必须的**：顶面看不见/被压成窄条时，
///       继续判就是**把侧面的颜色当成顶面**报出去 —— 而那会**判反塔的归属**。
///       实现见 `min_face_h_px` / `max_grazing_deg`（`project_spot` 与 `build_requests`）。
///     · ~~"相机未必需要抬升机构"~~ ⇒ 这条**重新变回未定**：要看见顶面，
///       相机与塔顶的高度关系变成硬约束。⚠️ 仿真侧已于 2026-09-24 把相机抬到
///       z=0.60（提交 `1802069`，**只为让判色链在仿真里能被验**）；**实机装位仍未拍板**
///       （`camera_hardware.yaml` 的 0.35m 是占位）。
///
///   ⚠️ **仍悬一条（不影响顶面判色，但影响仿真模型的保真度）—— 块的构型 (B) vs (C)**：
///     (C) 上下两个半高件（**本仓库采用**）⇒ 块**绕竖轴旋转不变**，于是
///         `sky_block_initial` 里那个 `code`（红南/红北/红西/红东）**没有几何解释**;
///     (B) 相对的两个**竖直面**一红一蓝、顶面本来无色、"颜色朝上"靠**扳倒 90°**
///         （**不是**翻个底朝天）⇒ `code` 恰是 (B) 的自然产物。
///     ⚠️ **这是量得出来的，不该用推理定死**：跑 `scripts/parse_field_assembly.py`
///        那条链量两个实体的 **z 区间与平面跨度** —— 红 z[0.10,0.20] + 蓝 z[0,0.10] ⇒ (C)；
///        红/蓝各跨满一个竖直面 ⇒ (B)。
///     ⚠️ **若 (B) 为真，掠射门控更不可放松**：那时掠射拍到侧面会给出
///        **一个自信的错颜色**（侧面的色）⇒ 直接判反归属，比"判不出"贵得多。
///     ⇒ 这条与"初始棋盘各格朝上的颜色（Figure 4 图案）"一起，**建议照会组织方（FAQ）**。
///
/// ── 另一条已核实、且**与上面无关**的约束（任何构型都成立）──
///   · 塔顶面**相对车所在楼面**恒为 **0.90m**（完整塔 = 0.35+0.35+0.20；
///     L1/L2 台面各加 0.6/0.9 后抵消 —— 规则书明写 L2 高地面 900mm、高于 L1 300mm）
///   · ⚠️ 每件 **±5%** 公差 ⇒ 塔高实为 **900 ±45mm（855~945）**，不是标称值
///   · ⚠️ **判色链的硬规矩**：仿真里若跑出 `RATIO_AMBIGUOUS`（红蓝≈各半）⇒
///     **是模型错了，不是规则模糊** ⇒ **不许放宽 `decisive_ratio`**
///     （放宽 = 把"判不出"换成"**判反**"，判反是把塔送给对手）。见 `camera_params.yaml`。
///
/// 输入 / 输出:
///   订: `/perception/camera_wide/preprocessed`、`/perception/camera_tele/preprocessed`
///   发: `/perception/camera/sky_blocks_wide`、`/perception/camera/sky_blocks_tele`
///       `/perception/camera/sky_blocks_diagnostics`
///
/// 线程安全：单线程执行器（默认），两路订阅与动态参数回调同属一个 mutually-exclusive
///           group —— 与 `building_spot_detector` / `mustika_detector` 同口径。
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "br_perception/msg/camera_sky_block.hpp"
// ⚠️ 订阅类型是 PreprocessedImage（带 letterbox 变换五字段）—— 不是 sensor_msgs/Image。
//    与 mustika / pillar / building_spot 三个检测器同口径（2026-09-20 P0-3 改的）。
#include "br_perception/msg/preprocessed_image.hpp"
#include "br_perception/vision/color_utils.hpp"

// ⚠️ `BuildingSpot`（雷达的建筑位）—— 投影主路要读它的 `state` 挑出"有塔的位"
#include "br_perception/msg/building_spot.hpp"
#include "br_perception/msg/lidar_result.hpp"

// ⚠️ **只读 TF**（不广播）—— 用 `tf2_ros::Buffer` 而不是 `CoordinateTransformer`：
//    那个类构造时会**无条件广播** `odom→base_link` / `world→odom`，而
//    `perception_pipeline` 已经起了一个 ⇒ 再来一个就是**第三个广播同一条变换**
//    （tf2 会刷 TF_REPEATED_DATA）。**本节点是纯消费者。**
//    （同 `mustika_pillar_check` 的处理，那边有完整的踩坑记录。）
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace br_perception {
namespace camera {

/**
 * @brief 天空方块**顶面颜色**判定组件（任务书 §3.3）
 *
 * ⚠️ 本类**不做目标检测** —— 见头文件"只做判色"那一节。
 *    它消费一个「顶面候选区域」，产出颜色结论。
 */
class SkyBlockDetector : public rclcpp::Node
{
public:
  explicit SkyBlockDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief 顶面颜色 —— ⚠️ 与 `msg/CameraSkyBlock.msg` 及
  ///        `msg/BuildingSpot.msg` 的 `top_color` **逐值一致**，不可重排
  /// @brief 这一帧的 ROI **从哪来** —— ⚠️ 取值与 `msg/CameraSkyBlock.msg` 的
  ///        `roi_source` **逐值一致**，不可重排（融合层要按它决定信不信）
  ///
  /// ⚠️⚠️ **这两个来源的可信度不一样，下游必须能分辨**：
  ///    · `RADAR_PROJECTED`：位置来自**雷达 + 场地常量 + TF**，是**几何推出来的** ——
  ///      它不会"看错地方"，但**依赖定位**（TF 不可用时它会整条失效，那时报 UNKNOWN）。
  ///    · `MANUAL`：人工在配置里指的框，只在台架上成立（比赛模式该留空）。
  enum RoiSource : uint8_t {
    kRadarProjected = 0,   ///< 雷达建筑位投影（得分场景主路，2026-09-24 落地）
    kYolo           = 1,   ///< 检测网络给的 bbox（**本版未实现**，接口预留）
    kManual         = 2,   ///< 调试参数手动指定（台架/调阈值用）
  };

  enum TopColor : uint8_t {
    kNone    = 0,   ///< 没有方块
    kRed     = 1,   ///< 红
    kBlue    = 2,   ///< 蓝
    kUnknown = 3,   ///< **有方块，但顶面颜色判不出来**（≠ kNone，见头文件边界①）
  };

  /// @brief 相机路数（下标 = `msg/CameraSkyBlock.camera_id`，**不可重排**）
  static constexpr size_t kCamCount = 2;

  // ═══════════════════════════════════════════════════════════════════════════
  // 建筑位投影 —— **得分场景的 ROI 主路**
  //
  //  得分场景是"**已知建筑位上有一座塔**"：位坐标是 `field_geometry.yaml` 的常量、
  //  位状态与平台高由雷达的 `BuildingSpot` 给 ⇒ "塔顶那块在哪"是**几何题**，
  //  不需要检测网络。把方块投到图像上就是 ROI。
  //
  //  ⚠️⚠️ **掠射门控就长在这里**（这也是它被评审点名"真难点"的原因）：
  //     `top_face_roi` 用固定比例从 bbox 上方切"顶面" —— **正视时可用，
  //     掠射时必然切到侧面**，颜色直接判错。而掠射角只有**投影**才知道。
  //     ⇒ 判据用**顶面投影高度**（`face_h_px`）：它就是 `200·sin(θ)·f/d`，
  //       掠射时趋于 0。低于阈值直接 UNKNOWN(GRAZING_ANGLE)，**不去判一个必错的颜色**。
  // ═══════════════════════════════════════════════════════════════════════════
  struct SpotProjection
  {
    bool    valid{false};
    int32_t spot_id{-1};
    /// 在**收到的（letterbox 之后的）图**里的 ROI —— 直接拿去索引 `bgr`
    cv::Rect roi_lb;
    /// **顶面中心 → 相机** 的方向与**世界竖直**的夹角 [deg]（= 相机相对塔顶的天顶角）。
    ///   0° = 相机在顶面正上方（最理想）；90° = 与顶面同高；>90° = **相机在顶面下方**
    ///   （顶面物理上看不见 ⇒ 门控必须拦）。
    /// ⚠️⚠️ 方向是"**方块 → 相机**"、参考点是**顶面中心 `z_hi`** —— 两处都在
    ///   2026-09-24 复核时抓到过"写反"（原先两处都反）：反了之后门控会**把最好的
    ///   位置拦掉、把最坏的位置放行**，而症状是"判出的颜色不是顶面的颜色"。
    double  grazing_deg{0.0};
    /// 顶面**投影高度** [px] —— 掠射的硬指标（= 200·sinθ·f/d，θ 为仰角）
    double  face_h_px{0.0};
    /// 顶面**投影宽度** [px]（掠射时基本不变，用来做对照）
    double  face_w_px{0.0};
    /// 相机到**顶面中心**的距离 [m]（⚠️ 不是到方块中心 —— 与 `grazing_deg` 同源）
    double  distance_m{0.0};
    std::string note;
  };

  /// @brief 判不出来的**原因** —— ⚠️ 取值与 `msg/CameraSkyBlock.msg` 的
  ///        `unknown_reason` **逐值一致**，不可重排
  ///
  /// ⚠️ **要能驱动动作**，不是一个给人看的装饰 —— 下游据此决定"靠近重看 / 等下一帧 / 换位姿"。
  enum UnknownReason : uint8_t {
    kReasonNone            = 0,  ///< 没有"判不出来"这回事（判出红/蓝，或根本没方块）
    kReasonRatioAmbiguous  = 1,  ///< 红蓝比例接近（45~55%）→ 动作: **靠近重看**
    kReasonLowValidPixels  = 2,  ///< 有效颜色像素太少（远处/太小）→ 动作: **靠近到 X 米内**
    kReasonRoiTooSmall     = 3,  ///< 投影出的 ROI 本身就太小 → 动作: 靠近 / 检查标定
    kReasonGrazingAngle    = 4,  ///< 掠射角过大 → 动作: 换位姿/靠近
    kReasonOccluded        = 5,  ///< 被遮挡 / 边缘不可信 / **方块出画** → 动作: 换视角
    kReasonUnconfirmed     = 6,  ///< 单帧过了阈值、时序还没确认 → 动作: **再等帧**
    /// ⚠️ **投影本身没算出来**（TF 不可用 / 定不到位 / 内参与标定不符）→ 动作: **查定位与标定**
    ///
    /// ⚠️⚠️ 与 `kReasonOccluded` **必须分开**：都是"没判成"，但现场做的事完全不同 ——
    ///    `5` 是"塔在画外/被挡住" ⇒ 换视角、靠近就能看见；
    ///    `7` 是"不知道相机在哪 / 投影算不出来" ⇒ **靠近也没用**，要查 TF 与标定。
    ///    合并的后果：拿"靠近"去对付定位故障，白跑一整天。
    /// ⚠️ 本字段**不上串口** ⇒ 新增它**不涉及与下位机对齐**。
    kReasonProjectionFailed = 7,
  };

  /// @brief 判色阈值 —— 名字与 `config/camera_params.yaml` 的
  ///        `sky_block_detector:` 块**逐字一致**
  struct ColorParams
  {
    double decisive_ratio  {0.70};  ///< 某一色占比超过它才定色（§3.3 的 70%）
    double close_lo        {0.45};  ///< "两色接近"区间下界（§3.3 的 45%）
    double close_hi        {0.55};  ///< "两色接近"区间上界（§3.3 的 55%）
    /// ⚠️ **规格没写、本模块补的**，而且是**第一道闸**（先于颜色比例）：
    ///    有效颜色像素的**绝对数**下限。理由见头文件边界②。
    ///    ⚠️ "70% 的占比在只有几十个像素时毫无意义" —— 24 个红像素 / 900 个像素
    ///       算出来也是 100% 红，但它**不该**和 600/900 同等可信。
    int    min_valid_pixels{80};
    /// 有效颜色像素占 ROI 总像素的**比例**下限（与上面那条**同时**满足才算够）
    double min_valid_ratio {0.25};
  };

  /// @brief 一次判色的完整结果（**中间量全留着** —— 现场调阈值全靠它）
  struct ColorVerdict
  {
    TopColor      color{kNone};
    UnknownReason reason{kReasonNone};
    double        red_ratio{0.0};        ///< red_px / (red_px + blue_px)；无有效像素时为 0
    double        blue_ratio{0.0};       ///< 恒 = 1 − red_ratio（发出来只为下游读着方便）
    double        color_margin{0.0};     ///< |red_ratio − blue_ratio| —— **颜色分离程度**
    double        valid_ratio{0.0};      ///< (red+blue) / ROI 总像素
    int           valid_pixel_count{0};  ///< (red+blue) 的绝对数
    std::string   note;                  ///< 为什么是这个结论（人读，直接进诊断）
  };

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️⚠️ 时序确认 —— **单帧过阈值不等于事实**（规则 3.5.8：对手会来翻）
  //
  //    三档状态 + 滞回：
  //      UNCONFIRMED —(连续 enter 帧满足)→ CONFIRMED
  //      CONFIRMED   —(连续 exit 帧不满足)→ UNCONFIRMED
  //                  中间那段是 **COASTING**（还在滞回窗口里，**不要据此翻状态**）
  //
  //    ⚠️ 为什么不是 bool：一帧抖动就把"我方塔"报成"对手塔"的代价太高。
  //    ⚠️ **本状态机已在 Python 里穷举验过**（抖动 / 翻转 / 翻回来三种序列），
  //       并且**抓出过一个真 bug**：退出那条原先只在 `CONFIRMED` 时判断，
  //       一进 `COASTING` 就**永远出不来**（卡死）。所以下面实现里那两档是**一起判**的。
  //    ⚠️ 做成独立小结构体是**故意的**：与 ROS 无关、可单测。
  // ═══════════════════════════════════════════════════════════════════════════
  struct TemporalConfirmer
  {
    enum State : uint8_t { kUnconfirmed = 0, kConfirmed = 1, kCoasting = 2 };

    int enter_frames{2};
    int exit_frames {3};
    State state{kUnconfirmed};

    /// @brief 喂一帧的结论，返回**喂完这帧之后**的状态
    State push(bool satisfied);
    /// @brief 换位置/重新观察时清干净（别让上一处的滞回影响下一处）
    void reset() { state = kUnconfirmed; in_count_ = 0; out_count_ = 0; }

  private:
    int in_count_{0};    ///< 连续满足的帧数
    int out_count_{0};   ///< 连续不满足的帧数
  };

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️⚠️ 判色核心 —— **static 纯函数，不碰 ROS、不读成员、不抛异常**
  //
  //    做成 static 是**故意的下**：单测可以直接喂 `cv::Mat` 调它，
  //    不需要起节点、不需要 ROS 图、不需要相机。**这是本模块唯一能自动化验证的部分**。
  // ═══════════════════════════════════════════════════════════════════════════

  /// @brief 在 `roi` 内数红/蓝像素 → 按阈值判色
  /// @param bgr  **原图**（BGR8；非 BGR8 会被转成 BGR8 再算，不静默出错）
  /// @param roi  顶面候选区域（**调用方决定** —— 见 `top_face_roi`）。
  ///             ⚠️ 越界 / 空区域 → 返回 `kUnknown` 并在 note 里说明，不抛。
  /// @param red  `color_thresholds.yaml` 的 `red_team`（多组取并集）
  /// @param blue `color_thresholds.yaml` 的 `blue_team`
  [[nodiscard]] static ColorVerdict judge_top_color(
      const cv::Mat& bgr,
      const cv::Rect& roi,
      const vision::ColorThreshold& red,
      const vision::ColorThreshold& blue,
      const ColorParams& params);

  /// @brief **顶面候选区域**（⚠️ **只给"手动 ROI / 网络 bbox"那条兜底路用**）
  ///
  /// 当前实现（v1.1，§3.3 原文）：取 `bbox` 的**上半区域**。
  ///   §3.3: "在 bbox 的上半区域（**预期为朝上面**）提取像素"
  ///
  /// ⚠️⚠️ **这个假设的成立条件没有写在规格里，但它不是无条件成立的**：
  ///     "上半区域 = 顶面"只在**相机明显高于顶面、俯角足够大**时才对；
  ///     俯角浅时顶面在图像里是**一条窄带**，这个启发式在最需要它的时候最脆。
  ///     §3.3 的 v1.2 项正是要修它（"结合雷达顶面检测 top_surface_z 与方块倾斜角
  ///     修正'朝上面' ROI, **不只看 bbox 上半区域**"）。
  ///
  /// ✅ **v1.2 那版几何已经落地了 —— 就是投影主路**（2026-09-24）：
  ///     `project_spot()` 用**位坐标 + 平台高 + 塔高 + TF + 内参**直接把顶面四角投到图上，
  ///     **不给"上半区域"任何猜测余地**；而"射不到顶面"这件事由
  ///     `min_face_h_px` / `max_grazing_deg` 两道门控**明说**（报 `GRAZING_ANGLE`）。
  ///     ⇒ 本函数现在**只服务兜底路**：人工框（`debug_roi_ratio`，台架调阈值用）——
  ///       那种框是"整个方块"，仍需要按比例收成上半区域。
  ///
  /// ⚠️ 因此**别把它接回主路**：主路若也套一刀，就会把投影已经算准的顶面**再切掉一半**
  ///    （切掉的那半可能正是判色要用的部分）。
  ///
  /// @param bbox          方块的包围盒（原图坐标系）
  /// @param top_face_frac 上半区域占比（1.0 = 整个 bbox，0.5 = 上一半）
  [[nodiscard]] static cv::Rect top_face_roi(const cv::Rect& bbox, double top_face_frac);

  /// @brief **【纯几何】世界 → 图像 ROI** —— ⚠️ **可单测的那一半**
  ///
  /// 与 `project_spot` 的分工：本函数**只吃数学**（一个变换 + 几个标量），
  /// 不碰 TF、不碰 ROS 节点 —— 所以 `test/test_sky_block_detector.cpp` 能拿手搭的位姿
  /// 直接验它。`project_spot` 是它的薄壳（查一次 TF 然后调它）。
  ///
  /// ⚠️⚠️ **为什么非要拆出来不可**：投影里最贵的错是**坐标系约定错**
  ///    （机身系 +X 前/+Y 左/+Z 上 vs 光心系 +X 右/+Y 下/+Z 前）——
  ///    漏了那一步**整体错 90°**，而**图像上看着还有东西**（只是位置全错），
  ///    靠肉眼看 RViz 是发现不了的。拆出来之后，单测能用"相机正对塔顶往下看"这种
  ///    **手算得出答案**的位姿把它钉死。
  ///
  /// @param world_to_cam  世界 → **相机机身系** 的变换（⚠️ 不是光心系；光心系那步在本函数内部做）
  ///        ⚠️⚠️ **按 tf 约定解释**：`p_body = R·p_world + t`（= `tf2::doTransform` 的算法，
  ///        也就是 `lookupTransform(camera, world)` 给出来的那个）。
  ///        `t` 是 **`−R·p_cam`**，**不是**相机在世界系里的位置 —— 2026-09-24 真写错过一次，
  ///        症状是"所有点落在光心背后 ⇒ 投影恒失败"，靠单测的几何用例抓到的。
  /// @param block_size_m  天空方块边长（0.20）
  /// @param tower_height_m 完整塔高（0.90）—— 天空方块底面 = `platform_z + 塔高 − 方块边长`
  /// @param fx/fy/cx/cy   该路相机内参（描述**原图**）
  /// @param scale/pad_x/pad_y letterbox 正向变换（原图 → 收到的图）
  /// @return `valid=false` 时 `note` 写明原因（**不去猜**）
  [[nodiscard]] static SpotProjection project_with_transform(
      const geometry_msgs::msg::TransformStamped& world_to_cam,
      int32_t spot_id,
      const geometry_msgs::msg::Point& spot_position,
      double platform_z,
      double block_size_m, double tower_height_m,
      double fx, double fy, double cx, double cy,
      double scale, int32_t pad_x, int32_t pad_y,
      int img_w, int img_h);

  /// @brief **把某个建筑位上的天空方块投影成图像 ROI**（几何主路）
  ///
  /// 输入全是已知量：位中心的 world 坐标（常量）、平台高（雷达给）、
  /// 相机在那一帧的位姿（TF）、内参（标定）。输出见 `SpotProjection`。
  ///
  /// ── ⚠️⚠️ 坐标系：这里是**最容易静默错 90°** 的地方（与 `mustika_pillar_check` 同源）──
  ///    TF 里 `camera_*_frame` 相对 `base_link` 是 `rpy=[0,0,0]` ⇒ 那是**机身系**
  ///    （+X 前 / +Y 左 / +Z 上）。而针孔成像要的是**光心系**（+X 右 / +Y 下 / +Z 前）。
  ///    两者差一个固定旋转: 光心 = (-y机身, -z机身, +x机身)。
  ///    **不转的话图像上看着"有东西"，只是位置全错** —— 而诊断里的残差会稳定地大，
  ///    标定时怎么调外参都调不好（错的是约定，不是外参）。
  ///
  /// ── ⚠️ letterbox：投影先得原图坐标，再**正向**变换到收到的图 ──
  ///    接收到的 `PreprocessedImage.image` 是 letterbox 之后的图，
  ///    而内参描述的是**原始**图 ⇒ 必须过 `to_box_x/to_box_y` 才能拿去索引 `bgr`。
  ///    （与检测器那边"反算回原图"的方向**正好相反**，别抄错。）
  ///
  /// @param spot_position 位中心的坐标（系由 `spot_header.frame_id` 决定）
  /// @param platform_z    该位的平台高（雷达的 `building_spot_z` 兼容层同源）
  /// @param cam           相机下标（0=wide / 1=tele）—— 内参按它取
  /// @param spot_header   位坐标那条消息的 header：**只用它的 `frame_id`**（坐标在哪个系）
  /// @param query_stamp   ⚠️ **TF 查这一时刻** —— 必须传**图像的曝光时刻**，不是雷达那一帧：
  ///        要算的是"**相机在哪**"，而相机在动。用雷达的时间戳 = 用 0.1s 前的相机位姿
  ///        （雷达 10Hz）⇒ 1m/s 车速下 ≈**10cm** 的投影误差，而方块只有 20cm。
  ///        ⚠️ 与"位状态不能太旧"是**两件事**：那个是"塔还在不在"（用雷达时刻），
  ///        这个是"相机在哪"（用曝光时刻）。
  /// @param scale/pad_x/pad_y  letterbox 变换（把原图坐标**正向**变到收到的图）
  ///
  /// ⚠️ **非 const**：`tf2_ros::Buffer::transform()` 是**非 const** 成员，
  ///    从 const 成员函数里调不到（这条编译器会拦，不用猜）。
  [[nodiscard]] SpotProjection project_spot(
      int32_t spot_id,
      const geometry_msgs::msg::Point& spot_position,
      double platform_z,
      uint8_t cam,
      const std_msgs::msg::Header& spot_header,
      const builtin_interfaces::msg::Time& query_stamp,
      double scale, int32_t pad_x, int32_t pad_y,
      int img_w, int img_h);

private:
  // ── ROS 回调 ──
  void wide_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);
  void tele_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);

  /// @brief 两路共用的处理：定 ROI（投影主路/手动）→ 判色 → 时序确认 → 发布
  /// ⚠️ **一帧可能发多条**（每个"有塔"的建筑位一条），见 `RoiRequest`
  void process_and_publish(
      const br_perception::msg::PreprocessedImage::SharedPtr msg,
      uint8_t camera_id,
      const rclcpp::Publisher<br_perception::msg::CameraSkyBlock>::SharedPtr& pub);


  // ═══════════════════════════════════════════════════════════════════════════
  // 一帧的 ROI 清单 —— **一帧可以发多条消息**
  // ═══════════════════════════════════════════════════════════════════════════
  /// @brief 一条待判的 ROI：来源 + 位置 + （投影主路时才有的）几何判据
  ///
  /// ⚠️ 为什么是"清单"而不是"一个 ROI"：`CameraSkyBlock.msg` 里 `spot_id` 是**单数**的
  ///    （一条观测 = 一个目标），而一帧里**可能有不止一座塔**
  ///    （规则 3.5.6/3.5.8：多个位都可能被建塔/被翻转）。硬塞进一条消息就只能报一座，
  ///    而"漏报的那座"在计分上等于**不存在**。⇒ 一帧发多条，各带各的 `spot_id`。
  struct RoiRequest
  {
    cv::Rect    roi_lb;                 ///< **letterbox 坐标系**（下游要用 msg.scale/pad 反算）
    uint8_t     source{kManual};        ///< 与 `msg/CameraSkyBlock.msg` 的 `roi_source` 逐值一致
    int32_t     spot_id{-1};            ///< `source != RADAR_PROJECTED` 时恒 -1
    bool        have_roi{false};        ///< false ⇒ 这一条**根本没定出 ROI**（颜色必是 UNKNOWN）
    /// @brief ⚠️ **这一帧连"要看的对象"都没有**（雷达没报完整塔、且没配手动 ROI）
    ///
    ///   与"雷达说有塔、只是投影/门控/ROI 太小"**必须分开**（2026-09-25 加）：
    ///   前者这一帧**什么都没看** ⇒ 结论是 `NONE`（`detected=false`）才是实话；
    ///   后者是"有方块但没判出来" ⇒ 必须 `UNKNOWN`（报 `NONE` 会让整座塔凭空消失，
    ///   见头文件边界①）。⚠️ `detected` 是二值字段，装不下"没去看" ——
    ///   那件事只能由 `unknown_reason` / `note` 表达。
    bool        no_source{false};
    UnknownReason reason{kReasonNone};  ///< `have_roi=false` 或掠射被拦时用的原因码
    bool        grazing_blocked{false}; ///< 掠射门控拦下 ⇒ 判 `GRAZING_ANGLE`，**不去判颜色**
    std::string note;                   ///< 给人看的那句话（诊断里原样带出去）
  };

  /// @brief 组本帧的 ROI 清单：**投影主路优先，手动兜底**
  ///
  /// ⚠️ 主路只要**产出过任何一条**（哪怕是"投影失败"），就不再叠手动 ROI ——
  ///    否则同一帧里"雷达说这儿有塔"和"调试参数指的那块"会一起发出去，
  ///    下游分不清哪条该信。
  void build_requests(const br_perception::msg::PreprocessedImage& msg,
                      uint8_t camera_id,
                      const cv::Mat& bgr,
                      std::vector<RoiRequest>& out);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  /// @param out 已经组好的那条消息（诊断要报 `state` / `spot_id` / `roi_source`，
  ///            ⚠️ 与发出去的那条**同源** —— 别在两处各算一次，那会漂）
  void publish_diagnostics(uint8_t camera_id, const ColorVerdict& v,
                           const br_perception::msg::CameraSkyBlock& out,
                           double total_ms);

  // ═══════════════════════════════════════════════════════════════════════════
  // 订阅 & 发布
  // ═══════════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<br_perception::msg::PreprocessedImage>::SharedPtr sub_wide_;
  rclcpp::Subscription<br_perception::msg::PreprocessedImage>::SharedPtr sub_tele_;
  /// @brief 雷达的建筑位 —— **投影主路的唯一输入**（哪几个位上有塔）
  rclcpp::Subscription<br_perception::msg::LidarResult>::SharedPtr sub_lidar_;

  /// @brief 雷达结果回调：只做"记下来"，判定在相机帧到了才做
  ///        （⚠️ 投影要的是**同一时刻**的位姿与雷达位状态，所以在相机帧那侧组）
  void on_lidar(const br_perception::msg::LidarResult::SharedPtr msg);

  rclcpp::Publisher<br_perception::msg::CameraSkyBlock>::SharedPtr pub_wide_;
  rclcpp::Publisher<br_perception::msg::CameraSkyBlock>::SharedPtr pub_tele_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ═══════════════════════════════════════════════════════════════════════════
  // 参数 —— ⚠️ 名字与 `config/camera_params.yaml` 的 `sky_block_detector:` 块逐字一致
  // ═══════════════════════════════════════════════════════════════════════════
  std::string color_yaml_path_;

  // ── 投影主路（雷达建筑位 → 图像 ROI）──
  std::string lidar_topic_{"/perception/lidar/result"};

  /// @brief 10 个建筑位的位坐标 + 平台高 —— **与 `lidar_perception_node` 读的是同一组键**
  ///        （`field_geometry.yaml` 第 11 节兼容层）。⚠️ launch 必须把 `field_geometry.yaml`
  ///        传给本节点，否则三个数组为空 ⇒ 投影主路直接不可用（**会明确 WARN，不静默**）。
  std::vector<double> spot_x_, spot_y_, spot_z_;

  /// @brief 完整塔高 [m] —— `field_geometry.yaml` 的 `complete_tower.height` = **0.90**
  ///        （= 地球块 0.35×2 + 天空块 0.20）。⚠️ 每件有 ±5% 公差 ⇒ **实为 855~945mm**，
  ///        法向余量要按这个波动留（别按标称 0.90 卡着设计）。
  double tower_height_m_{0.90};
  /// @brief 天空方块边长 [m] = 0.20（`field_geometry.yaml` 的 `sky_block.size`）
  double block_size_m_{0.20};

  /// @brief 相机内参（下标 = camera_id）—— ⚠️ **目前是占位值**，事实源在
  ///        `config/camera_hardware.yaml`，标定后要一起改（那笔债台账有记）。
  std::array<double, kCamCount> fx_{{800.0, 2380.0}};
  std::array<double, kCamCount> fy_{{800.0, 2380.0}};
  std::array<double, kCamCount> cx_{{640.0, 640.0}};
  std::array<double, kCamCount> cy_{{360.0, 360.0}};

  /// @brief 投影用的「原图」尺寸（内参对应的那张图）—— 必须与内参同源
  int orig_w_{1280};
  int orig_h_{720};

  /// @brief 两路相机在 **TF 树里**的 frame 名（下标 = camera_id）
  ///        ⚠️ 必须与 `bringup_gazebo` 的 `SENSOR_TF` / `bringing_full` 广播的
  ///           `--child-frame-id` **逐字一致**。对不上 → TF 查不到 → 投影恒失败
  ///           （**不报错，只是永远判不出** —— 所以构造函数里会校验长度并 WARN）。
  std::array<std::string, kCamCount> camera_frames_{{"camera_wide_frame",
                                                     "camera_tele_frame"}};

  /// @brief ⚠️ **掠射门控**：顶面投影高度低于它就直接判 `GRAZING_ANGLE`，**不去判颜色**。
  ///
  /// 由 `200·sinθ·f/d ≥ min_face_h_px` 反解出可用的最小仰角。
  /// 取 **6px** 是**经验线**（再少就被单行噪声与 Canny 边缘误差支配），
  /// **不是推导出来的** —— 相机到手后要拿实拍图重定，别当已验证。
  double min_face_h_px_{6.0};

  /// @brief 掠射角的硬上限 [deg]：相机一旦落在顶面**下方**（`grazing_deg ≥ 90`）
  ///        顶面根本不可见 ⇒ 无论像素多少都判 `GRAZING_ANGLE`。
  ///        ⚠️ 90° 是**几何事实**，不是可调参数；留成参数是为了让标定时能收紧。
  double max_grazing_deg_{89.0};

  /// @brief TF 查询超时 [s] —— ⚠️ **别用 tf2 的默认值 0**（同 `mustika_pillar_check`）：
  ///        TF 按里程计时间戳广播、本节点按消息时间戳查，两边跨进程；给 0 时缓冲区
  ///        还没包住该时刻就直接抛，而症状会写成"取不到方块位置"。
  double tf_timeout_sec_{0.05};

  /// @brief 只读 TF（⚠️ 不广播 —— 理由见本文件 `tf_ros` 那段注释）
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief 最近一帧雷达结果里的建筑位（投影主路的输入）
  std::vector<br_perception::msg::BuildingSpot> spots_;
  /// @brief 上面那帧的 header（**坐标系 + 时间戳都靠它**，不能用 now()）
  std_msgs::msg::Header spots_header_;
  bool has_spots_{false};

  // ── 判色阈值（与 `ColorParams` 同名字段，但这里是"运行期当前值"）──
  ColorParams color_params_;

  /// @brief 顶面候选 ROI 的上半区域占比
  ///        ⚠️ 见 `top_face_roi` 的注释：这是 v1.1 的临时几何，
  ///           相机装位定了之后要换成基于柱位/高度的版本。
  double top_face_frac_{0.5};

  /// @brief **调试用**：整图比例默认 ROI `[x, y, w, h]`（相对宽高的 0~1）
  ///
  /// ⚠️ 为什么需要它：本模块**不做检测**（见头文件），所以拿不到 bbox。
  ///    在检测接进来之前，现场要能**手动指一个区域**把它跑起来
  ///    （对着塔顶摆好相机、量一下塔顶在画面里的位置填进来）。
  ///    ⇒ 这是**过渡手段**，检测接进来后应留空。**留空 = 没有可用 ROI → 全部判 NONE**，
  ///      不静默用整幅图（那会把整个画面当顶面，判出毫无意义的颜色）。
  std::vector<double> debug_roi_ratio_{};

  /// @brief 单路 ROI 最小像素面积 —— 太小的 ROI 判色无意义（噪声主导）
  int roi_min_area_{64};

  // ── 时序确认帧数（见 `TemporalConfirmer`）──
  /// @brief 连续多少帧满足才从 UNCONFIRMED 升到 CONFIRMED
  int enter_confirm_frames_{2};
  /// @brief 连续多少帧不满足才从 CONFIRMED 掉出去（之间是 COASTING 滞回窗口）
  int exit_confirm_frames_{3};

  /// @brief 每路相机各一个时序状态机（**必须分开** —— 两路视野不同、帧率也可能不同）
  /// ⚠️ **按位**（每座塔一个），不是"每相机一个"（2026-09-24 改）：
  ///    投影主路一帧可能同时看到**多座塔**，而每座塔顶面的颜色是**各自独立**的状态。
  ///    共用一个确认器 = A 塔的红蓝帧去喂 B 塔的滞回 —— 那是凭空造出的串扰。
  ///    槽位 = `spot_id`；`id < 0`（手动 ROI）与越界的都落到最后一槽。
  static constexpr size_t kConfirmerSlots = 16;   ///< ⚠️ 必须 > `field_geometry` 的建筑位数(10)
  std::array<std::array<TemporalConfirmer, kConfirmerSlots>, kCamCount> confirmer_;

  /// @brief 取某个 spot 的确认器槽位（`id < 0` 或越界 → 最后一槽）
  static size_t confirmer_slot(int32_t spot_id) {
    return (spot_id >= 0 && static_cast<size_t>(spot_id) < kConfirmerSlots - 1)
               ? static_cast<size_t>(spot_id)
               : kConfirmerSlots - 1;
  }

  /// @brief 本帧的 ROI 来源 —— ⚠️ 取值与 `msg/CameraSkyBlock.msg` 的 `roi_source` 一致
  ///
  /// ✅ **2026-09-24: `RADAR_PROJECTED`（得分场景主路）已落地** —— 见 `project_spot`
  ///    与 `process_and_publish` 里的 ROI 来源分支。它们**一帧可以发多条消息**
  ///    （每个"看到塔"的建筑位一条），所以 `spot_id` 是**每条消息各自的**，
  ///    不再是整个节点一个。
  /// ⚠️ 本成员只剩**兜底**用途：主路取不到（雷达没消息、没有塔）时回到 `kManual`。
  uint8_t roi_source_{kManual};
  // ⚠️ 原来这里还有一个 `roi_source_name_`（注释写着"也当诊断里那条来源名"用）—— **2026-09-25 删**：
  //    它**从不被更新**、恒是 `"manual"`，而主路实际是 `radar_projected` ⇒
  //    诊断会把现场的人指到"手动 ROI 开着"上去。诊断现在从**这条消息自己的**
  //    `roi_source` 取名（见 `publish_diagnostics`，那正是它参数注释说的"同源"）。
  /// @brief 雷达投影时的建筑位号（0~9）；手动/网络来源填 **-1**
  int32_t spot_id_{-1};

  // ── HSV 阈值（从 color_thresholds.yaml 加载）──
  vision::ColorThreshold red_;
  vision::ColorThreshold blue_;

  // ═══════════════════════════════════════════════════════════════════════════
  // 运行时状态
  // ═══════════════════════════════════════════════════════════════════════════
  int  wide_count_{0};
  int  tele_count_{0};
  std::mutex state_mutex_;

  uint64_t wide_frame_seq_{0};
  uint64_t tele_frame_seq_{0};
  std::mutex diag_mutex_;
};

}  // namespace camera
}  // namespace br_perception
