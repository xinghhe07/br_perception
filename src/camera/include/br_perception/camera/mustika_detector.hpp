#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// mustika_detector.hpp — ROBOCON 2027 穆斯蒂卡相机检测节点
///
/// 依赖: OpenCV (core + imgproc) + cv_bridge + color_utils
/// 职责:
///   1. 订阅双路预处理图像 (/perception/camera_wide/preprocessed,
///      /perception/camera_tele/preprocessed)
///   2. HSV 金色区域提取 (阈值从 color_thresholds.yaml 加载)
///   3. 形态学闭运算填补球体表面暗斑
///   4. cv::HoughCircles 检测圆形目标
///   5. 候选验证: 半径范围检查 + 金色像素占比 > 60% + 圆形度 > 0.85
///   6. Fallback: 金色区域轮廓检测 + minEnclosingCircle
///   7. 发布 /perception/camera/mustika (CameraMustika)
///   8. 发布 /perception/camera/mustika_diagnostics
///
/// 策略: 纯传统 CV — 金色球体在 HSV 空间极其明显, 不需要深度学习.
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "br_perception/msg/camera_mustika.hpp"          // 自定义消息 (ROBOCON 专用)
// ⚠️ 2026-09-20: 订阅类型从 sensor_msgs/Image 改为 PreprocessedImage ——
//    后者把 letterbox 变换一起带来, 检测框必须用它反算回原图坐标
//    (函数 letterbox_to_orig() 在 image_preprocessor.hpp)
#include "br_perception/msg/preprocessed_image.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace br_perception {
namespace camera {

/// @brief 检测方法（原为裸 int，魔法数字 + 诊断输出不可读）
enum class DetectMethod : uint8_t {
  NONE             = 0,
  HOUGH_CIRCLES    = 1,
  CONTOUR_FALLBACK = 2,
};

/**
 * @brief 单次 Mustika 图像检测的原始结果
 *
 * 在 publish 之前聚合前后相机结果。
 *
 * ⚠️ **坐标系契约（2026-09-20 定 —— 之前没写，是缺口）**
 *   下面的 `u` / `v` / `radius` 都在 **预处理图（letterbox 之后）的像素坐标**里，
 *   **不是原图坐标**。`detect()` 内部**不反算** —— 反算由调用方
 *   （`process_and_publish`）在**发布之前**做。
 *
 *   反算（⚠️ **三个量都要换**；只换中心点、不换半径，会让下游拿到错误尺寸）：
 *   ```
 *   double ox = u, oy = v, orr = radius;
 *   letterbox_to_orig(msg->scale, msg->pad_x, msg->pad_y, ox, oy, orr);
 *   //  ox/oy = 原图坐标;  orr = 原图像素半径
 *   ```
 *   `letterbox_to_orig` 在 `image_preprocessor.hpp`；**半径只除 scale、不减填充偏移**。
 *
 *   ⚠️ 若拿相机内参（`focal_length_px_` 等）换算本结构的值，
 *      那个内参必须对应**同一空间** —— 见成员 `focal_length_px_` 的说明。
 */
struct MustikaImageResult
{
  bool   detected = false;          // ⚠️ false 时 u/v/radius 均为 0，**不代表"左上角有个半径为 0 的球"**，下游必须先判本字段

  // ── 以下为 letterbox 空间坐标，发布前必须反算回原图（见上面的契约）──
  float  u = 0.0f;
  float  v = 0.0f;
  float  radius = 0.0f;

  /// 置信度，范围 **[0, 1]**。公式：
  /// `conf = 0.5 × min(1, golden_ratio / 0.80) + 0.5 × circularity`
  /// （golden_ratio 到 0.80 即满分；见 .cpp 的 detect()）
  float  confidence = 0.0f;

  // 验证指标（诊断用）
  float  golden_ratio = 0.0f;       // 圆内黄底像素占比 [0,1]
  float  circularity = 0.0f;        // 圆形度 [0,1]
  DetectMethod method = DetectMethod::NONE;
};

/**
 * @brief 穆斯蒂卡相机检测组件
 *
 * 检测管线 (每路相机独立):
 *   RGB 图像 → BGR → HSV → 金色阈值 mask
 *   → 形态学闭运算 → HoughCircles
 *   → 候选验证 (半径 / 金色占比 / 圆形度)
 *   → [fallback] 轮廓 + minEnclosingCircle
 *   → 发布 CameraMustika + Diagnostics
 *
 * 前后相机独立回调, 共享参数和 YAML 颜色阈值。
 */
class MustikaDetector : public rclcpp::Node
{
public:
  struct HsvGroup {
    cv::Scalar lower;
    cv::Scalar upper;
  };

  // ═════════════════════════════════════════════════════════════════════════════
  // 纯函数契约（2026-09-25 加，为单测而摘）
  //
  // ⚠️ 摘成 `static + Params` 是为了让**判定逻辑能被单测直接喂**（仓库既有模式：
  //    `SkyBlockDetector::judge_top_color`、`ClusterExtractor::filter_obstacle`）。
  // ⚠️⚠️ **原成员函数保留为薄包装**（把成员变量打包成 `MustikaParams` 传进去）
  //    ⇒ **节点行为逐位不变**，调用点一处不改。
  //
  // ⚠️ **为什么这个模块最需要单测**：它出的是**那 250 分**那条链的输入
  //    （`mustika_pillar_check` 拿它检出的球去做"在柱上吗"的判定）。
  //    而它的三个关键阈值 **2026-09-20 改过值**（半径 0.065→0.105、
  //    圆形度 0.85→0.70、黄底占比 0.60→0.50）—— 台账 P1-9 原话是
  //    「**需重编重测**」。下面的用例就是"重测"。
  // ═════════════════════════════════════════════════════════════════════════════

  /// 一个 HSV 阈值组（金色是**多组取并集**）
  /// ⚠️ 定义在下面（原在 private 段，2026-09-25 挪到 public —— `MustikaParams` 是
  ///    公开结构体，它引用私有嵌套类型会让外部无法使用）。

  /// 检测参数（**名字与 `config/camera_params.yaml` 的键一致**）
  struct MustikaParams {
    /// 金色阈值组（**并集**）—— 来自 `color_thresholds.yaml` 的 `yellow_ball`
    std::vector<HsvGroup> golden_groups;

    // ── 形态学 ──
    int    morph_close_kernel{5};
    int    morph_close_iters{2};

    // ── HoughCircles ──
    double hough_dp{1.0};
    double hough_min_dist{30.0};
    double hough_param1{100.0};
    double hough_param2{30.0};
    int    hough_min_radius{10};
    int    hough_max_radius{150};

    // ── 验证 ──
    /// ⚠️ 圆内金像素占比下限（**2026-09-20 由 0.60 改到 0.50**）
    double min_golden_ratio{0.50};
    /// ⚠️ 圆形度下限（**2026-09-20 由 0.85 改到 0.70**）
    double min_circularity{0.70};

    // ── 兜底轮廓 ──
    int    contour_min_area{50};
    double contour_min_golden_ratio{0.40};

    // ── 距离 / 尺寸（用来把像素半径换算成"可能多远"）──
    double focal_length_px{600.0};
    /// ⚠️ 穆斯蒂卡真实半径 (m) —— **2026-09-20 由 0.065 改到 0.105**
    ///    （实物是 Mikasa V300W 排球，**直径 210mm** ⇒ 半径 0.105）
    double mustika_real_radius{0.105};
    double min_distance{0.2};
    double max_distance{3.0};
  };

  /// @brief 金色区域二值 mask（HSV 阈值并集 + 形态学闭运算）
  /// @param bgr  输入 BGR（**CV_8UC3**；空图/非法通道数 ⇒ 输出全零 mask，不抛）
  /// @param mask 输出 CV_8UC1，255 = 金色
  ///
  /// ⚠️ `morph_close_iters <= 0` ⇒ **跳过闭运算**（不是"跑 0 次"的等价写法 ——
  ///    OpenCV 对 `iterations = 0` 的行为是"不迭代"，两条路结果一样但显式跳过更清楚）。
  static void extract_golden_mask(const cv::Mat& bgr, cv::Mat& mask,
                                  const MustikaParams& params);

  /// @brief 验证一个候选圆是不是穆斯蒂卡球
  /// @param result ⚠️ **仅成功时填入**；任何一条判据不过就 `return false` **早退**，
  ///               `result` **一个字段都不碰**（保持调用前的值）⇒ 调用方**必须以返回值为准**。
  ///
  /// ⚠️⚠️ **半径判据是"两套来源取交集"**（头文件早就标注了这个陷阱）：
  /// ```
  ///   eff_min = max(hough_min_radius, r_min_by_dist / 2)
  ///   eff_max = min(hough_max_radius, r_max_by_dist * 2)
  /// ```
  ///    ⇒ `hough_max_radius` 若配小，会**把按距离算对的上界夹回去**，
  ///      而症状是"**明明在可见距离内的球检不出来**" —— 不报错。
  ///    ⚠️ 单测专门钉这条（见 `test_mustika_detector.cpp`）。
  static bool validate_circle(const cv::Mat& bgr, const cv::Mat& mask,
                              const cv::Vec3f& circle, MustikaImageResult& result,
                              const MustikaParams& params);

  /// @brief 把当前成员参数打包成 `MustikaParams`（**薄包装用**）
  [[nodiscard]] MustikaParams make_params() const;

  explicit MustikaDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void wide_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);
  void tele_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);
  /// 相机 ID 编码 —— 与 `msg/CameraMustika.msg` 的 `camera_id` 字段**逐值一致**
  ///   0 = 宽角 (wide, JR H200S 宽)
  ///   1 = 长焦 (tele, JR H200S 长焦)
  /// ⚠️ 新增相机时**只能往后追加**，不要插队或重排 —— 下位机与日志按数值解析。
  static constexpr uint8_t kCamWide = 0;
  static constexpr uint8_t kCamTele = 1;

  void process_and_publish(
      const br_perception::msg::PreprocessedImage::SharedPtr msg,
      uint8_t camera_id,
      const rclcpp::Publisher<br_perception::msg::CameraMustika>::SharedPtr& pub);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  // ═════════════════════════════════════════════════════════════════════════════
  // 检测管线子步骤
  // ═════════════════════════════════════════════════════════════════════════════

  /// @brief 完整检测管线: BGR 图像 → MustikaImageResult
  /// @param bgr         输入 BGR 图像 (CV_8UC3)
  /// @param cam_name    相机名 (wide/tele, 日志用)
  /// @return 检测结果
  MustikaImageResult detect(const cv::Mat& bgr,
                            const std::string& cam_name);

  /// @brief Step 1: 生成金色区域二值 mask (HSV 阈值 + 形态学闭运算)
  /// @param bgr  输入 BGR
  /// @param mask 输出二值 mask (CV_8UC1, 255=金色)
  void extract_golden_mask(const cv::Mat& bgr, cv::Mat& mask);

  /// @brief Step 2: HoughCircles 检测圆形
  /// @param bgr         BGR 图像 (用于 HoughCircles 梯度)
  /// @param mask        金色区域 mask (用于预筛选 ROI)
  /// @param circles     输出圆列表 [x, y, r] × N (CV_32FC3 或空)
  void detect_circles_hough(const cv::Mat& bgr,
                            const cv::Mat& mask,
                            std::vector<cv::Vec3f>& circles);

  /// @brief Step 3: 验证检测到的圆
  /// @param bgr     BGR 图像
  /// @param mask    金色 mask
  /// @param circle  候选圆 (x, y, r)
  /// @param result  输出验证后的结果
  /// @return true 若通过验证
  ///
  /// **输出契约（2026-09-20 定 —— 之前没写，已按实现核实）**
  ///   · **仅成功时填入**。任何一条判据不过就 **`return false` 早退**，
  ///     `result` **一个字段都不碰**（保持调用前的值）。
  ///   · 所以调用方**必须以返回值为准** —— 不能只看 `result.detected`，
  ///     也不能指望失败时能读到"为什么没过"（诊断指标不填）。
  ///
  /// ⚠️ 半径判据用的是 `eff_min/eff_max`，它把**两套来源取交集**：
  ///      `eff_max = min(hough_max_radius_, f×R/min_distance_ × 2)`
  ///    所以 `hough_max_radius_` 若偏小，会**把按距离算对的上界夹回去** ——
  ///    这正是"0.42m 以内检测不到"的根因（见成员处的说明）。
  bool validate_circle(const cv::Mat& bgr,
                       const cv::Mat& mask,
                       const cv::Vec3f& circle,
                       MustikaImageResult& result);

  /// @brief Step 4 (Fallback): 金色轮廓 + minEnclosingCircle
  /// @param mask    金色 mask
  /// @param result  输出结果
  /// @return true 若找到有效轮廓
  bool fallback_contour_detect(const cv::Mat& mask,
                               MustikaImageResult& result);

  /// @brief 计算轮廓的圆形度
  /// @param contour  输入轮廓点集
  /// @param area     轮廓面积 (传入避免重复计算)
  /// @return circularity = 4π × area / perimeter²  [0.0, 1.0]
  static float compute_circularity(const std::vector<cv::Point>& contour,
                                   double area);

  // ── 诊断 ──
  void publish_diagnostics(uint8_t camera_id,
                           double total_ms,
                           const MustikaImageResult& result,
                           int image_w, int image_h);

  // ═════════════════════════════════════════════════════════════════════════════
  // 订阅 & 发布
  // ═════════════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<br_perception::msg::PreprocessedImage>::SharedPtr sub_wide_;
  rclcpp::Subscription<br_perception::msg::PreprocessedImage>::SharedPtr sub_tele_;

  rclcpp::Publisher<br_perception::msg::CameraMustika>::SharedPtr pub_wide_;
  rclcpp::Publisher<br_perception::msg::CameraMustika>::SharedPtr pub_tele_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  // ── 参数回调句柄 ──
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ═════════════════════════════════════════════════════════════════════════════
  // HSV 金色阈值 (从 color_thresholds.yaml 加载)
  // ═════════════════════════════════════════════════════════════════════════════
  // ⚠️ 线程安全（2026-09-20 核实并写明 —— 之前是缺口）
  //    `golden_groups_` **只在构造函数里写一次**（从 color_yaml_path_ 加载），
  //    之后只被 extract_golden_mask() 读。on_parameter_change() 覆盖的 10 个
  //    参数里**没有颜色相关项** → **不存在数据竞争，无需加锁**。
  //
  //    ⚠️ 契约：**若将来把颜色阈值也做成 ROS 参数**（支持运行时改），
  //       必须给它加锁（复用 state_mutex_ 或另起一个），并在读侧先拷贝再遍历 ——
  //       `std::vector` 边改边遍历会让迭代器失效。
  std::vector<HsvGroup> golden_groups_;  // 金色阈值组 (并集); 构造期写入, 之后只读
  std::string color_yaml_path_;          // ⚠️ 改了它不会重载 —— 需重启节点

  // ═════════════════════════════════════════════════════════════════════════════
  // 检测参数 (可通过 ROS param 动态调整)
  // ═════════════════════════════════════════════════════════════════════════════
  // ── 形态学 ──
  int    morph_close_kernel_{5};       // 闭运算核大小
  int    morph_close_iters_{2};        // 闭运算迭代次数

  // ── HoughCircles ──
  double hough_dp_{1.0};              // 累加器分辨率反比
  double hough_min_dist_{30.0};       // 圆心最小间距 (pixels)
  double hough_param1_{100.0};        // Canny 高阈值
  double hough_param2_{30.0};         // 圆心累加器阈值
  //
  // ⚠️⚠️ 半径搜索窗必须与 min_distance_ / max_distance_ **一致**（2026-09-20 修）
  //
  //   针孔模型: r_px = focal_length_px_ × mustika_real_radius_ / d
  //   所以搜索窗应当**从距离派生**，而不是各写各的：
  //       hough_min_radius_ = f × R / max_distance_     ← 最远的球 = 最小像素半径
  //       hough_max_radius_ = f × R / min_distance_     ← 最近的球 = 最大像素半径
  //
  //   旧值 `min=10 / max=150` + `f=600 / R=0.105` 的等价距离是 **6.3 m ~ 0.42 m**，
  //   而 min_distance_ 声明的是 **0.2 m**（需要 315 px）——
  //   **球在 0.42 m 以内时 HoughCircles 直接框不到，静默返回 detected=false。**
  //   而那正是"把球放到柱顶凹槽"那个动作的距离。
  //
  //   ⚠️ 这两个值**不建议手工设** —— 构造函数会按上面的公式重算并覆盖。
  //      要改检测距离，改 min_distance_ / max_distance_。
  int    hough_min_radius_{10};       // 最小半径 (px) —— 由 max_distance_ 派生
  int    hough_max_radius_{150};      // 最大半径 (px) —— 由 min_distance_ 派生

  // ── 验证阈值 ──
  // ✅ 2026-09-20 按 v1.1 实物 (Mikasa V300W 排球, 黄底+蓝纹+白字) 改默认值:
  //    60% → 50%: 排球不是整片黄, 有蓝纹和白字, 卡 60% 会判掉真球
  //    (同时对应 color_thresholds.yaml 的 yellow_min_pixel_ratio)
  double min_golden_ratio_{0.50};     // 圆内黄底像素占比下限 (原 0.60)
  //    0.85 → 0.70: 球坐在 Ø180 凹槽里**只有上半可见**, 圆形度达不到 0.85
  //    (同时对应 color_thresholds.yaml 的 circularity_threshold)
  double min_circularity_{0.70};      // 圆形度下限 (原 0.85)

  // ── Fallback ──
  int    contour_min_area_{50};       // 轮廓最小面积 (pixels²)
  double contour_min_golden_ratio_{0.40}; // fallback 黄底占比放宽 (原 0.50)

  // ── 相机参数 (用于半径合理性换算) ──
  //
  // ⚠️ `focal_length_px_` 的空间归属（2026-09-20 补 —— 之前没说清）：
  //    它必须对应 **预处理图（letterbox 之后）的像素**，因为 `detect()` 的
  //    输入就是那张图，HoughCircles 也在那张图上做。
  //    标定给出的是**原图**的 f_x；letterbox 后应换算：
  //        f_此处 = f_原图 × scale      （scale 来自 PreprocessedImage 消息）
  //    ⚠️ wide 与 tele 若型号/分辨率不同，f 也不同 —— 当前是**单值**，
  //       换分辨率或换相机时必须同步改。
  //    ⚠️ 标定完成后回填（现在 600.0 是近似值，config 里也这么标的）。
  double focal_length_px_{600.0};     // 近似焦距 (px, **letterbox 空间**)
  // ✅ 2026-09-20 修正: 0.065 (Ø130, 两个版本都不是) → 0.105 (Ø210 排球半径)。
  //    该值直接决定 HoughCircles 的半径搜索窗 (f*R/d), 错 40% 会导致框不到球。
  //    v1.1 第 10 页: Mikasa V300W, Size 5, circumference 65–67 cm → Ø≈210 mm
  double mustika_real_radius_{0.105}; // 穆斯蒂卡真实半径 (m)
  //
  // ⚠️ 这两个是**距离的唯一事实源** —— hough_min/max_radius_ 由它们派生
  //    （构造函数里算），不要手工设半径窗。
  double min_distance_{0.2};          // 最近检测距离 (m) → 派生 hough_max_radius_
  double max_distance_{3.0};          // 最远检测距离 (m) → 派生 hough_min_radius_

  // ═════════════════════════════════════════════════════════════════════════════
  // 运行时状态
  // ═════════════════════════════════════════════════════════════════════════════
  // ⚠️ 这三个配对使用：**state_mutex_ 保护的就是下面两个 bool**
  //    （wide/tele 两个回调并发写、诊断线程读）。改其它成员时不要误以为它也在保护。
  bool wide_in_view_{false};
  bool tele_in_view_{false};
  std::mutex state_mutex_;

  // 帧计数器
  uint64_t wide_frame_seq_{0};
  uint64_t tele_frame_seq_{0};
  std::mutex diag_mutex_;
};

}  // namespace camera
}  // namespace br_perception
