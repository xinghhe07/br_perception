/// ═══════════════════════════════════════════════════════════════════════════════
/// building_spot_detector.cpp — ROBOCON 2027 建筑位绿色方格检测实现
///
/// 检测管线 (每路相机独立):
///   RGB → HSV → 绿色阈值 mask → 形态学闭运算
///   → findContours → approxPolyDP → 四边形筛选
///   → 正方形验证 (内角 / 对边平行 / 凸度 / 绿色占比)
///   → 排序 → 发布 CameraBuildingSpot + Diagnostics
///
/// 关键设计决策:
///   - 检测四边形而非严格正方形: 500×500mm 地面方格在透视投影下为凸四边形
///   - 容忍 3 或 5 顶点轮廓: 方块堆叠会遮挡部分绿色区域
///   - 内角评分: 四个角各距 90° 的偏差归一化取均值
///   - 对边平行度: 两组对边方向向量的点积绝对值
///   - 凸度: polygon area / convex hull area, 凹形不是建筑位
///   - 绿色占比: mask 填充率, 遮挡时会降低但仍有底线
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/building_spot_detector.hpp"
// letterbox_to_orig_xy() —— 角点反算回原图坐标 (2026-09-20, P0-3)
#include "br_perception/camera/image_preprocessor.hpp"
#include "br_perception/vision/color_utils.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace br_perception {
namespace camera {

// ═══════════════════════════════════════════════════════════════════════════════
// 构造 & 参数初始化
// ═══════════════════════════════════════════════════════════════════════════════

BuildingSpotDetector::BuildingSpotDetector(const rclcpp::NodeOptions& options)
  : Node("building_spot_detector", options)
{
  // ── 颜色阈值 YAML 路径 ──
  this->declare_parameter<std::string>("color_yaml_path",
    "config/color_thresholds.yaml");
  this->get_parameter("color_yaml_path", color_yaml_path_);

  // ── 形态学 ──
  this->declare_parameter<int>("morph_close_kernel", morph_close_kernel_);
  this->declare_parameter<int>("morph_close_iters", morph_close_iters_);
  this->get_parameter("morph_close_kernel", morph_close_kernel_);
  this->get_parameter("morph_close_iters", morph_close_iters_);

  // ── 轮廓 ──
  this->declare_parameter<double>("approx_epsilon_factor", approx_epsilon_factor_);
  this->declare_parameter<int>("contour_min_area", contour_min_area_);
  this->declare_parameter<int>("contour_max_area", contour_max_area_);
  this->get_parameter("approx_epsilon_factor", approx_epsilon_factor_);
  this->get_parameter("contour_min_area", contour_min_area_);
  this->get_parameter("contour_max_area", contour_max_area_);

  // ── 验证阈值 ──
  this->declare_parameter<double>("min_green_ratio", min_green_ratio_);
  this->declare_parameter<double>("min_convexity", min_convexity_);
  this->declare_parameter<double>("min_angle_score", min_angle_score_);
  this->declare_parameter<double>("min_side_score", min_side_score_);
  this->get_parameter("min_green_ratio", min_green_ratio_);
  this->get_parameter("min_convexity", min_convexity_);
  this->get_parameter("min_angle_score", min_angle_score_);
  this->get_parameter("min_side_score", min_side_score_);

  // ── 尺寸 ──
  this->declare_parameter<double>("spot_real_size", spot_real_size_);
  this->declare_parameter<double>("focal_length_px", focal_length_px_);
  this->declare_parameter<double>("min_distance", min_distance_);
  this->declare_parameter<double>("max_distance", max_distance_);
  this->get_parameter("spot_real_size", spot_real_size_);
  this->get_parameter("focal_length_px", focal_length_px_);
  this->get_parameter("min_distance", min_distance_);
  this->get_parameter("max_distance", max_distance_);

  // ── PnP ──
  this->declare_parameter<bool>("pnp_verify_enabled", pnp_verify_enabled_);
  this->get_parameter("pnp_verify_enabled", pnp_verify_enabled_);

  // ── 参数规范化 ──
  morph_close_kernel_  = std::max(3, std::min(morph_close_kernel_, 31));
  morph_close_iters_   = std::max(0, std::min(morph_close_iters_, 5));
  approx_epsilon_factor_ = std::max(0.01, std::min(approx_epsilon_factor_, 0.10));
  contour_min_area_    = std::max(50, std::min(contour_min_area_, 50000));
  contour_max_area_    = std::max(contour_min_area_ + 1, contour_max_area_);
  min_green_ratio_     = std::max(0.2, std::min(min_green_ratio_, 1.0));
  min_convexity_       = std::max(0.5, std::min(min_convexity_, 1.0));
  min_angle_score_     = std::max(0.3, std::min(min_angle_score_, 1.0));
  min_side_score_      = std::max(0.3, std::min(min_side_score_, 1.0));
  spot_real_size_      = std::max(0.1, spot_real_size_);
  min_distance_        = std::max(0.1, min_distance_);
  max_distance_        = std::max(min_distance_ + 0.1, max_distance_);

  // ── 加载绿色 HSV 阈值 ──
  try {
    auto all_thresholds = br_perception::vision::load_color_thresholds(color_yaml_path_);
    auto it = all_thresholds.find("green_building_spot");
    if (it != all_thresholds.end()) {
      for (const auto& group : it->second.groups) {
        green_groups_.push_back({group.lower, group.upper});
      }
      RCLCPP_INFO(this->get_logger(),
        "Loaded %zu green HSV groups from %s",
        green_groups_.size(), color_yaml_path_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
        "No 'green_building_spot' in %s, using hardcoded thresholds",
        color_yaml_path_.c_str());
      green_groups_.push_back({cv::Scalar(40, 60, 30), cv::Scalar(80, 255, 180)});
      green_groups_.push_back({cv::Scalar(35, 40, 25), cv::Scalar(85, 255, 150)});
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to load color YAML (%s), using hardcoded: %s",
      color_yaml_path_.c_str(), e.what());
    green_groups_.push_back({cv::Scalar(40, 60, 30), cv::Scalar(80, 255, 180)});
    green_groups_.push_back({cv::Scalar(35, 40, 25), cv::Scalar(85, 255, 150)});
  }

  // ── 动态参数更新 ──
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&BuildingSpotDetector::on_parameter_change, this, std::placeholders::_1));

  // ── 订阅预处理图像 ──
  // ⚠ BEST_EFFORT + **队列深度 1**（任务书 §3.0 硬约束第 2 条）——
  //    完整理由写在 `image_preprocessor.cpp` 的 `stream_qos` 那一段。
  //    **上游发布端与本处必须同为 depth=1**, 只改一侧等于没改。
  const auto stream_qos = rclcpp::QoS(1).best_effort();

  sub_wide_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
    "/perception/camera_wide/preprocessed", stream_qos,
    std::bind(&BuildingSpotDetector::wide_callback, this, std::placeholders::_1));
  sub_tele_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
    "/perception/camera_tele/preprocessed", stream_qos,
    std::bind(&BuildingSpotDetector::tele_callback, this, std::placeholders::_1));

  // ── 发布 ──
  pub_wide_ = this->create_publisher<br_perception::msg::CameraBuildingSpot>(
    "/perception/camera/building_spots_wide", stream_qos);
  pub_tele_ = this->create_publisher<br_perception::msg::CameraBuildingSpot>(
    "/perception/camera/building_spots_tele", stream_qos);

  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/camera/building_spot_diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "BuildingSpotDetector ready | green_groups=%zu | morph=%dx%d | "
    "approx_eps=%.3f | area=[%d,%d] | "
    "validate(green>%.0f%% convex>%.2f angle>%.2f side>%.2f) | "
    "spot=%.3fm dist=[%.2f,%.2f]m",
    green_groups_.size(), morph_close_kernel_, morph_close_iters_,
    approx_epsilon_factor_, contour_min_area_, contour_max_area_,
    min_green_ratio_ * 100.0, min_convexity_, min_angle_score_, min_side_score_,
    spot_real_size_, min_distance_, max_distance_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 回调
// ═══════════════════════════════════════════════════════════════════════════════
void BuildingSpotDetector::wide_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg)
{
  process_and_publish(msg, 0, pub_wide_);
  wide_frame_seq_++;
}

void BuildingSpotDetector::tele_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg)
{
  process_and_publish(msg, 1, pub_tele_);
  tele_frame_seq_++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 单路相机处理
// ═══════════════════════════════════════════════════════════════════════════════
void BuildingSpotDetector::process_and_publish(
    const br_perception::msg::PreprocessedImage::SharedPtr msg,
    uint8_t camera_id,
    const rclcpp::Publisher<br_perception::msg::CameraBuildingSpot>::SharedPtr& pub)
{
  const char* cam_name = (camera_id == 0) ? "wide" : "tele";
  auto t0 = std::chrono::steady_clock::now();

  // ── 1. ROS Image → cv::Mat ──
  cv::Mat bgr;
  try {
    // msg->image 是内嵌字段, 不是独立 shared_ptr → 带 tracked_object 的重载 (零拷贝)
    auto cv_ptr = cv_bridge::toCvShare(msg->image, msg, sensor_msgs::image_encodings::BGR8);
    bgr = cv_ptr->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] cv_bridge error: %s", cam_name, e.what());
    return;
  }
  if (bgr.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] Empty image", cam_name);
    return;
  }

  // ── 2. 检测 ──
  auto results = detect(bgr, cam_name);

  // ── 3. 发布每个检测结果 ──
  //   ⚠️ 发布前必须把角点/中心从 letterbox 坐标反算回**原图坐标**（2026-09-20, P0-3）
  //   detect() 的输入是预处理后的 letterbox 图。角点和中心**全是位置量** →
  //   统一用 letterbox_to_orig_xy（减填充偏移 + 除 scale）。
  for (const auto& r : results) {
    br_perception::msg::CameraBuildingSpot out_msg;
    out_msg.camera_id  = camera_id;
    out_msg.detected   = r.detected;

    // 四个角点：逐个反算（函数签名是两个引用，所以一个一个来）
    float cu[4], cv[4];
    for (int k = 0; k < 4; ++k) {
      double x = r.corners[k].x, y = r.corners[k].y;
      letterbox_to_orig_xy(msg->scale, msg->pad_x, msg->pad_y, x, y);
      cu[k] = static_cast<float>(x);
      cv[k] = static_cast<float>(y);
    }
    out_msg.u0 = cu[0]; out_msg.v0 = cv[0];
    out_msg.u1 = cu[1]; out_msg.v1 = cv[1];
    out_msg.u2 = cu[2]; out_msg.v2 = cv[2];
    out_msg.u3 = cu[3]; out_msg.v3 = cv[3];

    // 中心点
    double cx = r.center.x, cy = r.center.y;
    letterbox_to_orig_xy(msg->scale, msg->pad_x, msg->pad_y, cx, cy);
    out_msg.center_u = static_cast<float>(cx);
    out_msg.center_v = static_cast<float>(cy);

    out_msg.confidence = r.confidence;
    out_msg.header     = msg->header;   // ⚠️ 曝光时刻, 禁止 now()
    pub->publish(out_msg);
  }

  // 如果没有检测到，发布一个空的 detected=false 消息
  if (results.empty()) {
    br_perception::msg::CameraBuildingSpot out_msg;
    out_msg.camera_id = camera_id;
    out_msg.detected  = false;
    out_msg.header    = msg->header;
    pub->publish(out_msg);
  }

  // ── 4. 更新状态 ──
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (camera_id == 0) wide_spot_count_ = static_cast<int>(results.size());
    else                tele_spot_count_  = static_cast<int>(results.size());
  }

  // ── 5. 诊断 ──
  auto t1 = std::chrono::steady_clock::now();
  double total_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
  publish_diagnostics(camera_id, total_ms, results, bgr.cols, bgr.rows);

  if (total_ms > 30.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[%s] Building spot detection slow: %.1f ms (%dx%d)",
      cam_name, total_ms, bgr.cols, bgr.rows);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 完整检测管线
// ═══════════════════════════════════════════════════════════════════════════════
std::vector<BuildingSpotImageResult>
BuildingSpotDetector::detect(const cv::Mat& bgr, const std::string& /*cam_name*/)
{
  std::vector<BuildingSpotImageResult> results;

  // ── Step 1: 绿色区域 mask ──
  cv::Mat mask;
  extract_green_mask(bgr, mask);

  const double green_ratio = static_cast<double>(cv::countNonZero(mask))
                             / static_cast<double>(mask.total());
  if (green_ratio < 0.0005) return results;  // 没有绿色

  // ── Step 2: 找四边形 ──
  std::vector<std::vector<cv::Point>> quads;
  std::vector<float> quad_convexities;
  find_quadrilaterals(mask, quads, quad_convexities);

  // ── Step 3: 验证每个候选 ──
  for (size_t i = 0; i < quads.size(); ++i) {
    BuildingSpotImageResult r;
    const float convexity = (i < quad_convexities.size()) ? quad_convexities[i] : 0.95f;
    if (validate_quad(mask, quads[i], convexity, r)) {
      results.push_back(r);
    }
  }

  // ── 按置信度降序 ──
  std::sort(results.begin(), results.end(),
    [](const BuildingSpotImageResult& a, const BuildingSpotImageResult& b) {
      return a.confidence > b.confidence;
    });

  return results;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 1: 绿色 mask 提取
// ═══════════════════════════════════════════════════════════════════════════════
void BuildingSpotDetector::extract_green_mask(const cv::Mat& bgr, cv::Mat& mask)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
  for (const auto& group : green_groups_) {
    cv::Mat group_mask;
    cv::inRange(hsv, group.lower, group.upper, group_mask);
    cv::bitwise_or(mask, group_mask, mask);
  }

  // 形态学闭运算: 填补绿色区域内的小孔洞
  if (morph_close_iters_ > 0) {
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(morph_close_kernel_, morph_close_kernel_));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel,
                     cv::Point(-1, -1), morph_close_iters_);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 2: 找四边形
// ═══════════════════════════════════════════════════════════════════════════════
void BuildingSpotDetector::find_quadrilaterals(
    const cv::Mat& mask,
    std::vector<std::vector<cv::Point>>& quads,
    std::vector<float>& convexities)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& cnt : contours) {
    const double area = cv::contourArea(cnt);
    if (area < contour_min_area_ || area > contour_max_area_) continue;

    const double perimeter = cv::arcLength(cnt, true);
    if (perimeter < 1.0) continue;

    // 多边形近似
    const double epsilon = approx_epsilon_factor_ * perimeter;
    std::vector<cv::Point> approx;
    cv::approxPolyDP(cnt, approx, epsilon, true);

    const int v = static_cast<int>(approx.size());

    // 接受 4 顶点（完整方格）或 3/5 顶点（部分遮挡）
    if (v == 4 || v == 3 || v == 5) {
      // 检查凸性
      std::vector<cv::Point> hull;
      cv::convexHull(approx, hull);
      const double hull_area = cv::contourArea(hull);
      if (hull_area < 1.0) continue;

      const double convexity = area / hull_area;
      if (convexity < min_convexity_) continue;

      // 用凸包顶点作为四边形（处理 3/5 顶点情况）
      if (v != 4) {
        if (hull.size() == 4) {
          order_corners(hull);
          // 凸包本身就是四边形，重新算凸度（应 ≈ 1.0，记录原始值供诊断）
          std::vector<cv::Point> hull_of_hull;
          cv::convexHull(hull, hull_of_hull);
          const double q_area = cv::contourArea(hull);
          const double q_hull_area = cv::contourArea(hull_of_hull);
          const float q_convexity = (q_hull_area > 0.0)
              ? static_cast<float>(q_area / q_hull_area) : 0.0f;
          quads.push_back(hull);
          convexities.push_back(q_convexity);
        }
        // 3顶点→三角形不处理, 5+顶点→取最大4个点
        else if (hull.size() > 4) {
          // 按面积贡献排序取前4个
          std::vector<std::pair<double, cv::Point>> scored;
          for (size_t hi = 0; hi < hull.size(); ++hi) {
            const auto& p = hull[hi];
            const auto& p_prev = hull[(hi + hull.size() - 1) % hull.size()];
            const auto& p_next = hull[(hi + 1) % hull.size()];
            double dx = p_next.x - p_prev.x;
            double dy = p_next.y - p_prev.y;
            double len = std::sqrt(dx*dx + dy*dy);
            double dist = 0.0;
            if (len > 1e-6) {
              dist = std::abs(dy*p.x - dx*p.y + p_next.x*p_prev.y - p_next.y*p_prev.x) / len;
            }
            scored.push_back({dist, p});
          }
          std::sort(scored.begin(), scored.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
          std::vector<cv::Point> top4;
          for (int ti = 0; ti < 4 && ti < static_cast<int>(scored.size()); ++ti) {
            top4.push_back(scored[ti].second);
          }
          if (top4.size() == 4) {
            order_corners(top4);
            // 计算 top4 的实际凸度
            std::vector<cv::Point> top4_hull;
            cv::convexHull(top4, top4_hull);
            const double t4_area = cv::contourArea(top4);
            const double t4_hull_area = cv::contourArea(top4_hull);
            const float t4_convexity = (t4_hull_area > 0.0)
                ? static_cast<float>(t4_area / t4_hull_area) : 0.0f;
            quads.push_back(top4);
            convexities.push_back(t4_convexity);
          }
        }
      } else {
        // 4 顶点，直接排序，传递原始凸度
        order_corners(approx);
        quads.push_back(approx);
        convexities.push_back(static_cast<float>(convexity));
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 3: 验证四边形
// ═══════════════════════════════════════════════════════════════════════════════
bool BuildingSpotDetector::validate_quad(
    const cv::Mat& mask,
    const std::vector<cv::Point>& quad,
    float convexity,
    BuildingSpotImageResult& result)
{
  if (quad.size() != 4) return false;

  // ── 3a. 内角检查 ──
  float angle_score = 0.0f;
  if (!validate_angles(quad, angle_score)) return false;

  // ── 3b. 对边平行度 ──
  float side_score = 0.0f;
  if (!validate_opposite_sides(quad, side_score)) return false;

  // ── 3c. 尺寸合理性 ──
  // 四边形像素边长应在合理范围
  float side_lengths[4];
  for (int i = 0; i < 4; ++i) {
    const auto& p0 = quad[i];
    const auto& p1 = quad[(i + 1) % 4];
    float dx = p1.x - p0.x;
    float dy = p1.y - p0.y;
    side_lengths[i] = std::sqrt(dx*dx + dy*dy);
  }
  const float avg_side = (side_lengths[0] + side_lengths[1] +
                          side_lengths[2] + side_lengths[3]) * 0.25f;

  // 像素边长根据距离范围换算 (使用最小边长检查)
  const float min_side_px = (focal_length_px_ * spot_real_size_) / max_distance_;
  const float max_side_px = (focal_length_px_ * spot_real_size_) / min_distance_;
  if (avg_side < min_side_px * 0.6f || avg_side > max_side_px * 1.4f) {
    return false;
  }

  // ── 3d. 绿色填充率 ──
  const float green_ratio = compute_green_fill_ratio(mask, quad);
  if (green_ratio < min_green_ratio_) return false;

  // ── 3e. 综合置信度 ──
  const float conf_green  = std::min(1.0f, green_ratio / 0.90f);
  const float conf_convex = std::min(1.0f, convexity / 0.95f);
  const float confidence  = 0.30f * conf_green + 0.30f * angle_score
                          + 0.25f * side_score  + 0.15f * conf_convex;

  // ── 填充结果 ──
  result.detected     = true;
  result.confidence   = confidence;
  result.green_ratio  = green_ratio;
  result.square_score = 0.5f * angle_score + 0.5f * side_score;
  result.convexity    = convexity;

  for (int i = 0; i < 4; ++i) {
    result.corners[i] = cv::Point2f(static_cast<float>(quad[i].x),
                                     static_cast<float>(quad[i].y));
  }

  result.center = cv::Point2f(
    (quad[0].x + quad[1].x + quad[2].x + quad[3].x) * 0.25f,
    (quad[0].y + quad[1].y + quad[2].y + quad[3].y) * 0.25f);

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 内角验证: 四角均应在 90° ± 15° 范围内
// ═══════════════════════════════════════════════════════════════════════════════
bool BuildingSpotDetector::validate_angles(
    const std::vector<cv::Point>& quad, float& angle_score)
{
  float scores[4];

  for (int i = 0; i < 4; ++i) {
    const auto& p_prev = quad[(i + 3) % 4];  // 前一个顶点
    const auto& p_curr = quad[i];
    const auto& p_next = quad[(i + 1) % 4];

    // 两边向量
    const float v1x = p_prev.x - p_curr.x;
    const float v1y = p_prev.y - p_curr.y;
    const float v2x = p_next.x - p_curr.x;
    const float v2y = p_next.y - p_curr.y;

    const float dot = v1x * v2x + v1y * v2y;
    const float mag1 = std::sqrt(v1x * v1x + v1y * v1y);
    const float mag2 = std::sqrt(v2x * v2x + v2y * v2y);

    if (mag1 < 1e-6f || mag2 < 1e-6f) return false;

    const float cos_angle = std::max(-1.0f, std::min(1.0f, dot / (mag1 * mag2)));
    const float angle_deg = std::acos(cos_angle) * 180.0f / static_cast<float>(M_PI);

    // 距离 90° 越近, 评分越高
    const float deviation = std::abs(angle_deg - 90.0f);
    if (deviation > 30.0f) return false;  // 硬阈值: < 60° 或 > 120° 直接拒绝

    // 评分: 0° 偏差 = 1.0, 30° 偏差 = 0.0
    scores[i] = 1.0f - deviation / 30.0f;
  }

  angle_score = (scores[0] + scores[1] + scores[2] + scores[3]) * 0.25f;
  return (angle_score >= min_angle_score_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 对边平行度验证
// ═══════════════════════════════════════════════════════════════════════════════
bool BuildingSpotDetector::validate_opposite_sides(
    const std::vector<cv::Point>& quad, float& side_score)
{
  // 两组对边: (0→1, 2→3) 和 (1→2, 3→0)
  auto edge_vec = [&](int i, int j) -> std::pair<float, float> {
    float dx = quad[j].x - quad[i].x;
    float dy = quad[j].y - quad[i].y;
    float len = std::sqrt(dx*dx + dy*dy);
    if (len < 1e-6f) return {0, 0};
    return {dx / len, dy / len};  // 单位方向向量
  };

  auto [e0x, e0y] = edge_vec(0, 1);   // 边 0→1
  auto [e1x, e1y] = edge_vec(2, 3);   // 边 2→3 (对边1)
  auto [e2x, e2y] = edge_vec(1, 2);   // 边 1→2
  auto [e3x, e3y] = edge_vec(3, 0);   // 边 3→0 (对边2)

  if (e0x == 0 && e0y == 0) return false;
  if (e2x == 0 && e2y == 0) return false;

  // 对边方向应相反 → 方向向量的点积应接近 -1
  const float parallel_1 = std::abs(e0x * e1x + e0y * e1y);  // |dot| → 接近1
  const float parallel_2 = std::abs(e2x * e3x + e2y * e3y);

  // 还要检查对边长度比 (应接近 1:1)
  auto side_len = [&](int i, int j) {
    float dx = quad[j].x - quad[i].x;
    float dy = quad[j].y - quad[i].y;
    return std::sqrt(dx*dx + dy*dy);
  };

  const float len0 = side_len(0, 1);
  const float len1 = side_len(2, 3);
  const float len2 = side_len(1, 2);
  const float len3 = side_len(3, 0);

  auto len_ratio = [](float a, float b) {
    if (a < 1e-6f || b < 1e-6f) return 0.0f;
    return std::min(a, b) / std::max(a, b);
  };

  const float ratio_1 = len_ratio(len0, len1);
  const float ratio_2 = len_ratio(len2, len3);

  // 综合评分: 平行度 + 长度比
  const float parallel_score = (parallel_1 + parallel_2) * 0.5f;
  const float ratio_score = (ratio_1 + ratio_2) * 0.5f;

  side_score = 0.6f * parallel_score + 0.4f * ratio_score;
  return (side_score >= min_side_score_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 绿色填充率
// ═══════════════════════════════════════════════════════════════════════════════
float BuildingSpotDetector::compute_green_fill_ratio(
    const cv::Mat& mask, const std::vector<cv::Point>& quad)
{
  // 在四边形 ROI 内统计绿色像素
  cv::Rect bbox = cv::boundingRect(quad);
  bbox &= cv::Rect(0, 0, mask.cols, mask.rows);
  if (bbox.width <= 0 || bbox.height <= 0) return 0.0f;

  cv::Mat quad_mask = cv::Mat::zeros(bbox.size(), CV_8UC1);

  // 偏移到 ROI 坐标
  std::vector<cv::Point> shifted(4);
  for (int i = 0; i < 4; ++i) {
    shifted[i] = cv::Point(quad[i].x - bbox.x, quad[i].y - bbox.y);
  }

  std::vector<std::vector<cv::Point>> polys = {shifted};
  cv::fillPoly(quad_mask, polys, cv::Scalar(255));

  cv::Mat mask_roi = mask(bbox);
  const int total_px = cv::countNonZero(quad_mask);
  const int green_px = cv::countNonZero(mask_roi & quad_mask);

  return (total_px > 0) ? static_cast<float>(green_px) / static_cast<float>(total_px) : 0.0f;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 顶点排序: 从上到下 → 从左上角开始逆时针
// ═══════════════════════════════════════════════════════════════════════════════
void BuildingSpotDetector::order_corners(std::vector<cv::Point>& quad)
{
  if (quad.size() != 4) return;

  // 按 y 坐标排序，取最小的两个为"上边"
  std::sort(quad.begin(), quad.end(),
    [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });

  // 上边两个点按 x 排序 (左上→右上)
  if (quad[0].x > quad[1].x) std::swap(quad[0], quad[1]);

  // 下边两个点按 x 反向排序 (右下→左下)
  if (quad[2].x < quad[3].x) std::swap(quad[2], quad[3]);

  // 结果: [左上, 右上, 右下, 左下]
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════════
void BuildingSpotDetector::publish_diagnostics(
    uint8_t camera_id,
    double total_ms,
    const std::vector<BuildingSpotImageResult>& results,
    int image_w, int image_h)
{
  std::lock_guard<std::mutex> lock(diag_mutex_);
  const char* cam_name = (camera_id == 0) ? "wide" : "tele";

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string("building_spot_detector_") + cam_name;
  status.hardware_id = std::string("camera_") + cam_name;

  if (total_ms < 20.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  } else if (total_ms < 35.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "SLOW";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "VERY SLOW";
  }

  auto add = [&](const std::string& key, double val) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key; kv.value = std::to_string(val);
    status.values.push_back(kv);
  };

  add("total_ms", total_ms);
  add("spot_count", static_cast<double>(results.size()));
  add("image_w", static_cast<double>(image_w));
  add("image_h", static_cast<double>(image_h));
  add("frame_seq", static_cast<double>(
      (camera_id == 0) ? wide_frame_seq_ : tele_frame_seq_));

  if (!results.empty()) {
    add("best_confidence", static_cast<double>(results[0].confidence));
    add("best_green_ratio", static_cast<double>(results[0].green_ratio));
    add("best_square_score", static_cast<double>(results[0].square_score));
  }

  diag_array.status.push_back(status);
  pub_diagnostics_->publish(diag_array);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult BuildingSpotDetector::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : params) {
    const std::string& name = param.get_name();
    try {
           if (name == "morph_close_kernel")     morph_close_kernel_ = std::max(3, std::min(static_cast<int>(param.as_int()), 31));
      else if (name == "morph_close_iters")      morph_close_iters_ = std::max(0, std::min(static_cast<int>(param.as_int()), 5));
      else if (name == "approx_epsilon_factor")  approx_epsilon_factor_ = std::max(0.01, std::min(param.as_double(), 0.10));
      else if (name == "contour_min_area")       contour_min_area_ = std::max(50, static_cast<int>(param.as_int()));
      else if (name == "contour_max_area")       contour_max_area_ = std::max(contour_min_area_ + 1, static_cast<int>(param.as_int()));
      else if (name == "min_green_ratio")        min_green_ratio_ = std::max(0.2, std::min(param.as_double(), 1.0));
      else if (name == "min_convexity")          min_convexity_ = std::max(0.5, std::min(param.as_double(), 1.0));
      else if (name == "min_angle_score")        min_angle_score_ = std::max(0.3, std::min(param.as_double(), 1.0));
      else if (name == "min_side_score")         min_side_score_ = std::max(0.3, std::min(param.as_double(), 1.0));
      else if (name == "spot_real_size")         spot_real_size_ = std::max(0.1, param.as_double());
      else if (name == "min_distance")           min_distance_ = std::max(0.1, param.as_double());
      else if (name == "max_distance")           max_distance_ = std::max(min_distance_ + 0.1, param.as_double());
      else if (name == "pnp_verify_enabled")     pnp_verify_enabled_ = param.as_bool();
      else RCLCPP_DEBUG(this->get_logger(), "Ignoring unknown param: %s", name.c_str());
    } catch (const rclcpp::ParameterTypeException& e) {
      result.successful = false;
      result.reason = std::string("Type: ") + name + " " + e.what();
      break;
    }
  }

  if (result.successful) {
    RCLCPP_INFO(this->get_logger(), "Params updated");
  }
  return result;
}

}  // namespace camera
}  // namespace br_perception

// ── ROS2 组件注册 ──
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::BuildingSpotDetector)
