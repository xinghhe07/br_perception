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
/// ── ⚠️⚠️ 一条**未决**的几何前提（2026-09-23 暂定，定了要回来改 ROI 与门控）──
///
///   **天空方块的红蓝怎么分布？** 这一条决定了"要不要爬到塔顶上方"，进而决定
///   相机装位、要不要抬升机构、以及要不要掠射角门控。**文本上它是歧义的**：
///
///   | 说法 | 出处 |
///   |---|---|
///   | "colored red on **one side** and blue on **the other**" | 规则书英文 v1.1 术语表 |
///   | "一侧呈红色、另一侧呈蓝色" | 中文 v1.0 译本（⚠️ 该译本本仓库已判定过时且多处有错） |
///   | 官方 `天空块-红/蓝` 是**多实体零件**：每块一个"整块" + 一个"半块"，**半块那边即该颜色**<br>每块带 `分色方向`（沿y分红南 / 沿y分红北 / 沿x分红西 / 沿x分红东）<br>与规则书 Figure 4 的行奇偶一致；规则 4.1.4 那句"团队颜色的方向在南北与东西之间**交替 90°**"正好对上 | `config/field_geometry.yaml` 的 `sky_block_initial`（标 **CONFIRMED-SW**，由官方总装配 STEP 派生） |
///
///   ⚠️ "**分色方向**"这个词只在"方块被一个平面切成红半/蓝半"时才说得通 ⇒
///      装配数据指向 **红蓝各半**，而"1 面红 + 对面蓝"是我**一开始没核实就默认的**。
///
///   **⇒ 暂定按「红蓝各半」设计**（2026-09-23 用户拍板），并据此：
///     · 判色看的是**可见的那个色面**，**不预设相机必须在塔顶之上**
///     · 因此**掠射角门控暂时不需要**（侧面本来就是竖直的，不被压成窄条）
///     · 相机**未必需要抬升机构**（那条推论作废）
///
///   ⚠️ **但这条没被证实** —— 唯一的硬证据是规则书 Figure 3/4 的图（我们手上
///      只有抽出来的文字，图丢了），或直接解析 `doc/field_assembly/2027场地总装配.STEP`。
///   ⚠️ **对模块的影响是可控的**：判色核心、契约、时序确认**两种情形完全一样**，
///      只有 `top_face_roi` 这个几何函数要改。所以本文件把它做成**显式入参**，
///      不把几何假设埋进判色逻辑里。
///
/// ── 另一条已核实、且**与上面无关**的约束（无论哪种分布都成立）──
///   · 塔顶面**相对车所在楼面**恒为 **0.90m**（完整塔 = 0.35+0.35+0.20；
///     L1/L2 台面各加 0.6/0.9 后抵消 —— 规则书明写 L2 高地面 900mm、高于 L1 300mm）
///   · ⚠️ 每件 **±5%** 公差 ⇒ 塔高实为 **900 ±45mm（855~945）**，不是标称值
///   · 相机装位**尚未拍板**（配置与仿真里都是占位：config 0.35m / Gazebo 0.1669m，两处还不一致）
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
  enum TopColor : uint8_t {
    kNone    = 0,   ///< 没有方块
    kRed     = 1,   ///< 红
    kBlue    = 2,   ///< 蓝
    kUnknown = 3,   ///< **有方块，但顶面颜色判不出来**（≠ kNone，见头文件边界①）
  };

  /// @brief 相机路数（下标 = `msg/CameraSkyBlock.camera_id`，**不可重排**）
  static constexpr size_t kCamCount = 2;

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
    kReasonOccluded        = 5,  ///< 被遮挡 / 边缘不可信 → 动作: 换视角
    kReasonUnconfirmed     = 6,  ///< 单帧过了阈值、时序还没确认 → 动作: **再等帧**
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

  /// @brief **顶面候选区域** —— "哪个面是顶面"这件事的全部几何假设都在这一个函数里
  ///
  /// 当前实现（v1.1，§3.3 原文）：取 `bbox` 的**上半区域**。
  ///   §3.3: "在 bbox 的上半区域（**预期为朝上面**）提取像素"
  ///
  /// ⚠️⚠️ **这个假设的成立条件没有写在规格里，但它不是无条件成立的**：
  ///     "上半区域 = 顶面"只在**相机明显高于顶面、俯角足够大**时才对。
  ///     而核实的机械约束是：相机光心刚过塔顶 0.05~0.30m（见头文件末节），
  ///     ⇒ **俯角很浅**，顶面在图像里是**一条窄带**，这个启发式在最需要它的时候最脆。
  ///     §3.3 的 v1.2 项正是要修它（"结合雷达顶面检测 top_surface_z 与方块倾斜角
  ///     修正'朝上面' ROI, **不只看 bbox 上半区域**"）。
  ///
  /// ⇒ 所以它被**单独拎成 static 函数**：v1.2 那版几何（雷达给了柱位与高度、
  ///    方块朝向已知）落地时，**只改这一个函数**，判色核心与单测一行不动。
  ///
  /// @param bbox          方块的包围盒（原图坐标系）
  /// @param top_face_frac 上半区域占比（1.0 = 整个 bbox，0.5 = 上一半）
  [[nodiscard]] static cv::Rect top_face_roi(const cv::Rect& bbox, double top_face_frac);

private:
  // ── ROS 回调 ──
  void wide_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);
  void tele_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);

  /// @brief 两路共用的处理：裁 ROI → 判色 → 发布
  /// @param bbox_override 空 = 用 `bbox_from_ratio_` 那个整图比例默认值（见下）
  void process_and_publish(
      const br_perception::msg::PreprocessedImage::SharedPtr msg,
      uint8_t camera_id,
      const rclcpp::Publisher<br_perception::msg::CameraSkyBlock>::SharedPtr& pub);

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

  rclcpp::Publisher<br_perception::msg::CameraSkyBlock>::SharedPtr pub_wide_;
  rclcpp::Publisher<br_perception::msg::CameraSkyBlock>::SharedPtr pub_tele_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ═══════════════════════════════════════════════════════════════════════════
  // 参数 —— ⚠️ 名字与 `config/camera_params.yaml` 的 `sky_block_detector:` 块逐字一致
  // ═══════════════════════════════════════════════════════════════════════════
  std::string color_yaml_path_;

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
  std::array<TemporalConfirmer, kCamCount> confirmer_;

  /// @brief 本帧的 ROI 来源 —— ⚠️ 取值与 `msg/CameraSkyBlock.msg` 的 `roi_source` 一致
  ///
  /// ⚠️⚠️ **当前只实现了 `kManual`（调试参数手动指定）**：
  ///    `RADAR_PROJECTED`（雷达建筑位投影，得分场景的主路）**尚未实现** ——
  ///    它要拿 `LidarResult.building_spots` 的位状态 + `field_geometry` 的位坐标 +
  ///    平台高 + TF + 内参，把世界点投到图像上再收成 ROI。
  ///    ⇒ 这条是**明确的下一步**，不是遗漏；接口（本参数与 `spot_id`）已经留好，
  ///      补的时候只加一个 ROI 来源分支，判色核心一行不动。
  std::string roi_source_name_{"manual"};
  /// @brief `roi_source` 对应的枚举值（发给下游）；`kManual` = 2
  uint8_t roi_source_{2};
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
