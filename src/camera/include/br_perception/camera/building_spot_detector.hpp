#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// building_spot_detector.hpp — ROBOCON 2027 建筑位绿色方格检测节点
///
/// 依赖: OpenCV (core + imgproc) + cv_bridge + color_utils
/// 职责:
///   1. 订阅双路预处理图像
///   2. HSV 绿色区域提取 (阈值从 color_thresholds.yaml 加载)
///   3. 轮廓检测 + 多边形近似 → 找四边形（建筑位 500×500mm 绿色方格）
///   4. 正方形验证: 内角 80°-100° + 对边平行 + 绿色填充率 > 70%
///   5. 部分遮挡容错: 3/5 顶点轮廓仍尝试匹配，置信度降级
///   6. field_geometry 一致性校验 (PnP 投影验证)
///   7. 发布 /perception/camera/building_spots (CameraBuildingSpot)
///   8. 发布诊断信息
///
/// 策略: 纯传统 CV — 绿色方格在 HSV 空间极其明显, 不需要深度学习.
///       500×500mm 正方形在透视投影下变为凸四边形，
///       因此检测四边形而非严格正方形。
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "br_perception/msg/camera_building_spot.hpp"
// ⚠️ 2026-09-20: 订阅类型改 PreprocessedImage —— 它带 letterbox 变换,
//    角点必须用它反算回原图坐标 (letterbox_to_orig_xy 在 image_preprocessor.hpp)
#include "br_perception/msg/preprocessed_image.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace br_perception {
namespace camera {

/**
 * @brief 单次建筑位检测的原始结果
 */
struct BuildingSpotImageResult
{
  bool  detected = false;
  float confidence = 0.0f;

  // 四角像素坐标 (顺序: 从最左上的顶点开始逆时针)
  cv::Point2f corners[4];

  // 中心
  cv::Point2f center;

  // 验证指标
  float green_ratio = 0.0f;       // 四边形内绿色像素占比
  float square_score = 0.0f;      // 正方形评分 (内角+对边综合)
  float convexity = 0.0f;         // 凸度 (多边形面积 / 凸包面积)

  // 匹配信息
  int   matched_spot_id = -1;     // 匹配到的 field_geometry 建筑位 ID
  float reprojection_error = 0.0f; // PnP 重投影误差
};

/**
 * @brief 建筑位绿色方格检测组件
 *
 * 检测管线 (每路相机独立):
 *   RGB → HSV → 绿色阈值 mask → 形态学闭运算
 *   → findContours → approxPolyDP → 四边形筛选
 *   → 正方形验证 (内角 / 对边 / 凸度 / 绿色占比)
 *   → [可选] field_geometry PnP 一致性验证
 *   → 发布 CameraBuildingSpot + Diagnostics
 */
class BuildingSpotDetector : public rclcpp::Node
{
public:
  // ═════════════════════════════════════════════════════════════════════════════
  // 纯函数：**四边形的几何验证**（2026-09-25 摘出来，为单测）
  //
  // ⚠️ 它们判的是"检出的四边形**是不是一个矩形**"——
  //    即"这团绿色**是不是一个建筑位**"。判错了**不报错**：
  //      · 太松 ⇒ 把绿色杂物也当成建筑位（**多报位**）
  //      · 太紧 ⇒ 真建筑位在斜视角下被拒（**漏报位**）
  //    而建筑位是那 40/80 分判色的**投影锚点**（位坐标 → 图像 ROI）。
  // ═════════════════════════════════════════════════════════════════════════════

  /// @brief 内角验证：四个角都要接近 90°
  /// @param angle_score **输出**：0(全偏 30°) ~ 1(全是直角)
  /// @param min_angle_score 评分下限
  ///
  /// ⚠️ **两层判据**：① 任一角偏离 90° 超过 **30°** ⇒ **硬拒**（直接 return false，
  ///    不看评分）；② 四角评分的**均值** >= `min_angle_score`。
  /// ⚠️ 边界**非严格**（`>=`）：评分恰好等于下限 ⇒ **通过**。
  static bool validate_angles(const std::vector<cv::Point>& quad, float& angle_score,
                              double min_angle_score);

  /// @brief 对边验证：两组对边**应平行**且**长度接近**
  /// @param side_score **输出**：`0.6×平行度 + 0.4×长度比`
  /// @param min_side_score 评分下限（**非严格 `>=`**）
  /// ⚠️ 某条边长为 0 ⇒ 直接拒（`edge_vec` 返回 (0,0) 时那条直接 return false）
  static bool validate_opposite_sides(const std::vector<cv::Point>& quad, float& side_score,
                                      double min_side_score);

  explicit BuildingSpotDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void wide_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);
  void tele_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg);
  void process_and_publish(
      const br_perception::msg::PreprocessedImage::SharedPtr msg,
      uint8_t camera_id,
      const rclcpp::Publisher<br_perception::msg::CameraBuildingSpot>::SharedPtr& pub);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  // ═════════════════════════════════════════════════════════════════════════════
  // 检测管线
  // ═════════════════════════════════════════════════════════════════════════════

  /// @brief 完整检测管线
  std::vector<BuildingSpotImageResult> detect(const cv::Mat& bgr,
                                               const std::string& cam_name);

  /// @brief Step 1: 绿色区域 mask
  void extract_green_mask(const cv::Mat& bgr, cv::Mat& mask);

  /// @brief Step 2: 找四边形候选 (轮廓 + 多边形近似)，同时输出每个四边形的凸度
  void find_quadrilaterals(const cv::Mat& mask,
                           std::vector<std::vector<cv::Point>>& quads,
                           std::vector<float>& convexities);

  /// @brief Step 3: 验证四边形是否为建筑位绿色方格
  bool validate_quad(const cv::Mat& mask,
                     const std::vector<cv::Point>& quad,
                     float convexity,
                     BuildingSpotImageResult& result);

  /// @brief 验证内角 (均应在 80°-100° 附近，容忍 ±15°)
  /// @note 非 static: 需读取 min_angle_score_ 阈值 (可动态调参)
  bool validate_angles(const std::vector<cv::Point>& quad,
                       float& angle_score);

  /// @brief 验证对边平行度
  /// @note 非 static: 需读取 min_side_score_ 阈值 (可动态调参)
  bool validate_opposite_sides(const std::vector<cv::Point>& quad,
                               float& side_score);

  /// @brief 计算四边形内绿色像素占比
  static float compute_green_fill_ratio(const cv::Mat& mask,
                                        const std::vector<cv::Point>& quad);

  /// @brief 对多边形顶点排序: 从最小 y 开始逆时针
  static void order_corners(std::vector<cv::Point>& quad);

  // ── 诊断 ──
  void publish_diagnostics(uint8_t camera_id,
                           double total_ms,
                           const std::vector<BuildingSpotImageResult>& results,
                           int image_w, int image_h);

  // ═════════════════════════════════════════════════════════════════════════════
  // 订阅 & 发布
  // ═════════════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<br_perception::msg::PreprocessedImage>::SharedPtr sub_wide_;
  rclcpp::Subscription<br_perception::msg::PreprocessedImage>::SharedPtr sub_tele_;

  rclcpp::Publisher<br_perception::msg::CameraBuildingSpot>::SharedPtr pub_wide_;
  rclcpp::Publisher<br_perception::msg::CameraBuildingSpot>::SharedPtr pub_tele_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ═════════════════════════════════════════════════════════════════════════════
  // HSV 绿色阈值
  // ═════════════════════════════════════════════════════════════════════════════
  struct HsvGroup {
    cv::Scalar lower;
    cv::Scalar upper;
  };
  std::vector<HsvGroup> green_groups_;
  std::string color_yaml_path_;

  // ═════════════════════════════════════════════════════════════════════════════
  // 检测参数
  // ═════════════════════════════════════════════════════════════════════════════
  // ── 形态学 ──
  int    morph_close_kernel_{5};
  int    morph_close_iters_{1};

  // ── 轮廓 ──
  double approx_epsilon_factor_{0.03};   // 多边形近似 epsilon = 因子 × 周长
  int    contour_min_area_{200};         // 最小轮廓面积 (pixels²)
  int    contour_max_area_{80000};       // 最大轮廓面积

  // ── 验证 ──
  double min_green_ratio_{0.70};         // 四边形内绿色像素占比下限
  double min_convexity_{0.88};           // 凸度下限
  double min_angle_score_{0.80};         // 内角评分下限 (1.0 = 完全90°)
  double min_side_score_{0.70};          // 对边平行度下限

  // ── 尺寸 ──
  double spot_real_size_{0.500};         // 建筑位真实边长 (m)
  double focal_length_px_{600.0};        // 近似焦距 (pixels)
  double min_distance_{0.3};             // 最近距离 (m)
  double max_distance_{5.0};             // 最远距离 (m)

  // ── PnP 验证 ──
  bool   pnp_verify_enabled_{false};     // 需要 camera_params 中的内参

  // ═════════════════════════════════════════════════════════════════════════════
  // 运行时状态
  // ═════════════════════════════════════════════════════════════════════════════
  int wide_spot_count_{0};
  int tele_spot_count_{0};
  std::mutex state_mutex_;

  uint64_t wide_frame_seq_{0};
  uint64_t tele_frame_seq_{0};
  std::mutex diag_mutex_;
};

}  // namespace camera
}  // namespace br_perception
