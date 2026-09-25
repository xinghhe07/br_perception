#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// pillar_detector.hpp — ROBOCON 2027 柱子检测节点
///
/// 依赖: OpenCV + cv_bridge + color_utils
/// 职责:
///   1. 订阅双路预处理图像
///   2. 双色 HSV 提取: dark_green (穆斯蒂卡柱) + brown (核心支柱)
///   3. 竖直矩形轮廓检测 (高宽比 > 3:1)
///   4. 柱子类型判定 (按颜色 + 高度比)
///   5. 柱顶凹形容器区域检查 (是否有物体遮挡)
///   6. field_geometry 位置一致性验证
///   7. 发布 /perception/camera/pillars (CameraPillar)
///   8. 发布诊断信息
///
/// 策略: 纯传统 CV — 柱子颜色鲜明 (深绿/棕) 且几何特征明确 (细长竖直矩形),
///       不需要深度学习. 柱子是静态目标, 不需要每帧都高置信度更新.
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "br_perception/msg/camera_pillar.hpp"
// ⚠️ 2026-09-20: 订阅类型改 PreprocessedImage —— 它带 letterbox 变换,
//    检测框必须用它反算回原图坐标 (letterbox_to_orig 在 image_preprocessor.hpp)
#include "br_perception/msg/preprocessed_image.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace br_perception {
namespace camera {

/// @brief 单次柱子检测的原始结果
struct PillarImageResult
{
  bool   detected = false;
  float  confidence = 0.0f;

  // 包围盒 (像素坐标系)
  cv::Rect2f bbox;

  // 柱子类型
  enum Type { UNKNOWN = 0, MUSTIKA_PILLAR = 1, CORE_PILLAR = 2 };
  Type   pillar_type = UNKNOWN;

  // 验证指标
  float  aspect_ratio = 0.0f;       // 高宽比
  float  color_fill_ratio = 0.0f;   // 包围盒内目标颜色像素占比
  float  upright_score = 0.0f;      // 竖直度评分

  // 柱顶检测
  bool   top_has_object = false;
  float  top_region_occupancy = 0.0f;  // 柱顶区域非背景像素占比
};

/**
 * @brief 柱子检测组件
 *
 * 检测管线 (每路相机独立):
 *   RGB → HSV → 双色 mask (dark_green + brown)
 *   → 形态学闭运算 → findContours
 *   → 高宽比筛选 (> 3:1) → 竖直度验证
 *   → 颜色填充率 + 尺寸合理性
 *   → 柱顶区域检查 → 类型判定
 *   → 发布 CameraPillar + Diagnostics
 */
class PillarDetector : public rclcpp::Node
{
public:
  // ═════════════════════════════════════════════════════════════════════════════
  // 纯函数：**候选柱子的验证**（2026-09-25 摘出来，为单测）
  //
  // ⚠️ 它判的是"这团深绿/棕色**是不是一根柱子**"—— 而柱子的位置喂给
  //    `mustika_pillar_check`（**那 250 分**）当"球在不在柱上"的基准点。
  // ═════════════════════════════════════════════════════════════════════════════

  /// 验证用到的参数（名字与 `camera_params.yaml` 的键一致）
  struct PillarParams {
    double min_color_fill{0.55};        ///< 包围盒内颜色像素占比下限（**硬拒**）
    double max_upright_angle{15.0};     ///< 主轴偏离垂直的最大角度（度），越低分越少
    double pillar_diameter{0.270};      ///< 柱子直径 (m)
    double mustika_pillar_height{0.500};///< 穆斯蒂卡柱高 (m)
    double core_pillar_height{0.800};   ///< 核心支柱高 (m)
    double focal_length_px{600.0};
  };

  /// @brief 候选柱子的验证
  /// @param bgr      原图（**当前实现没用到它**，留着是为了将来加颜色复核）
  /// @param mask     颜色 mask（CV_8UC1，255 = 深绿/棕）
  /// @param bbox     候选包围盒
  /// @param contour  候选轮廓
  /// @param result   ⚠️ **仅成功时填入**；不过判据就 `return false` 早退
  /// @param params   参数
  ///
  /// **判据（按实现）**：
  ///   · **硬拒**：`fill_ratio < min_color_fill`（包围盒内颜色占比）
  ///   · **硬拒**：bbox 与图像无交集（宽或高 <= 0）
  ///   · 评分：`conf = 0.40×填充 + 0.35×竖直度 + 0.25×实心度`
  ///
  /// ⚠️⚠️ **有一段是死代码**（2026-09-25 复核发现）：3b「尺寸合理性」算了
  ///    `est_distance` 与 `expected_height_px`，注释写"不做硬拒绝，**降低后续评分**"，
  ///    **但后面没有任何降分代码** ⇒ 那几个变量算完就丢。
  ///    ⇒ **改它不会有任何效果**，而这正是"配了不生效"最难发现的一种形状。
  ///    ⚠️ 本函数**只做搬运**，没有顺手删它 —— 删/接是**产品决定**（见台账）。
  static bool validate_pillar(const cv::Mat& bgr, const cv::Mat& mask,
                              const cv::Rect& bbox,
                              const std::vector<cv::Point>& contour,
                              PillarImageResult& result,
                              const PillarParams& params);

  /// 把当前成员参数打包（**薄包装用**）
  [[nodiscard]] PillarParams make_params() const;

  explicit PillarDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void wide_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);
  void tele_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);
  void process_and_publish(
      const br_perception::msg::PreprocessedImage::SharedPtr msg,
      uint8_t camera_id,
      const rclcpp::Publisher<br_perception::msg::CameraPillar>::SharedPtr& pub);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  // ═════════════════════════════════════════════════════════════════════════════
  // 检测管线
  // ═════════════════════════════════════════════════════════════════════════════

  /// @brief 完整检测管线
  std::vector<PillarImageResult> detect(const cv::Mat& bgr,
                                         const std::string& cam_name);

  /// @brief Step 1: 双色 mask (dark_green ∪ brown)
  void extract_pillar_mask(const cv::Mat& bgr,
                           cv::Mat& green_mask,
                           cv::Mat& brown_mask);

  /// @brief Step 2: 找竖直矩形候选
  void find_pillar_candidates(const cv::Mat& mask,
                              std::vector<cv::Rect>& bboxes,
                              std::vector<std::vector<cv::Point>>& contours);

  /// @brief Step 3: 验证柱子候选
  bool validate_pillar(const cv::Mat& bgr,
                       const cv::Mat& mask,
                       const cv::Rect& bbox,
                       const std::vector<cv::Point>& contour,
                       PillarImageResult& result);

  /// @brief 判定柱子类型 (按颜色占比)
  PillarImageResult::Type classify_pillar(const cv::Mat& bgr,
                                           const cv::Rect& bbox);

  /// @brief 检查柱顶区域是否有物体
  bool check_top_region(const cv::Mat& bgr,
                        const cv::Rect& bbox,
                        float& occupancy);

  // ── 诊断 ──
  void publish_diagnostics(uint8_t camera_id,
                           double total_ms,
                           const std::vector<PillarImageResult>& results,
                           int image_w, int image_h);

  // ═════════════════════════════════════════════════════════════════════════════
  // 订阅 & 发布
  // ═════════════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<br_perception::msg::PreprocessedImage>::SharedPtr sub_wide_;
  rclcpp::Subscription<br_perception::msg::PreprocessedImage>::SharedPtr sub_tele_;

  rclcpp::Publisher<br_perception::msg::CameraPillar>::SharedPtr pub_wide_;
  rclcpp::Publisher<br_perception::msg::CameraPillar>::SharedPtr pub_tele_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ═════════════════════════════════════════════════════════════════════════════
  // HSV 颜色阈值 (从 color_thresholds.yaml 加载)
  // ═════════════════════════════════════════════════════════════════════════════
  struct HsvGroup { cv::Scalar lower; cv::Scalar upper; };
  std::vector<HsvGroup> dark_green_groups_;
  std::vector<HsvGroup> brown_groups_;
  std::string color_yaml_path_;

  // ═════════════════════════════════════════════════════════════════════════════
  // 检测参数
  // ═════════════════════════════════════════════════════════════════════════════
  int    morph_close_kernel_{7};
  int    morph_close_iters_{2};

  double min_aspect_ratio_{3.0};        // 高宽比下限
  double max_aspect_ratio_{12.0};       // 高宽比上限 (防止过细噪点)

  int    contour_min_area_{300};        // 最小面积 (pixels²)
  int    contour_max_area_{60000};

  double min_color_fill_{0.55};         // 包围盒内颜色像素占比下限
  double max_upright_angle_{15.0};      // 主轴偏离垂直方向的最大角度 (度)

  // ── 柱顶检查 ──
  double top_check_offset_{0.10};       // 柱顶区域从顶部往下占 bbox 高度的比例
  double top_object_threshold_{0.30};   // 柱顶区域非背景像素占比阈值

  // ── 柱子真实尺寸 ──
  double pillar_diameter_{0.270};       // 柱子直径 (m)
  double mustika_pillar_height_{0.500}; // 穆斯蒂卡柱高 (m)
  double core_pillar_height_{0.800};    // 核心支柱高 (m)
  double focal_length_px_{600.0};

  // ═════════════════════════════════════════════════════════════════════════════
  // 运行时状态
  // ═════════════════════════════════════════════════════════════════════════════
  int wide_pillar_count_{0};
  int tele_pillar_count_{0};
  std::mutex state_mutex_;

  uint64_t wide_frame_seq_{0};
  uint64_t tele_frame_seq_{0};
  std::mutex diag_mutex_;
};

}  // namespace camera
}  // namespace br_perception
