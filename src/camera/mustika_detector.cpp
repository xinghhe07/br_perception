/// ═══════════════════════════════════════════════════════════════════════════════
/// mustika_detector.cpp — ROBOCON 2027 穆斯蒂卡相机检测实现
///
/// 检测管线 (每路相机独立):
///   RGB 图像 → BGR → HSV → 金色阈值 mask
///   → 形态学闭运算 → HoughCircles
///   → 候选验证 (半径 / 金色占比 > 60% / 圆形度 > 0.85)
///   → [fallback] 轮廓 + minEnclosingCircle
///   → 发布 CameraMustika + Diagnostics
///
/// 关键设计决策:
///   - 优先 HoughCircles (梯度法对圆形球体极其鲁棒, 即使部分遮挡)
///   - 金色 HSV 阈值从 color_thresholds.yaml 加载, 支持多组并集
///   - 闭运算填补球体表面因光照产生的暗斑
///   - Fallback 轮廓检测用于 HoughCircles 失效场景 (严重遮挡/低对比度)
///   - 验证三级: 半径合理性 → 金色占比 → 圆形度
///   - 前后相机独立回调, 共享 YAML 阈值和参数
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/mustika_detector.hpp"
#include "br_perception/vision/color_utils.hpp"
// letterbox_to_orig() —— 检测框反算回原图坐标 (2026-09-20, P0-3)
#include "br_perception/camera/image_preprocessor.hpp"

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

MustikaDetector::MustikaDetector(const rclcpp::NodeOptions& options)
  : Node("mustika_detector", options)
{
  // ── 彩色阈值 YAML 路径 ──
  this->declare_parameter<std::string>("color_yaml_path",
    "config/color_thresholds.yaml");
  this->get_parameter("color_yaml_path", color_yaml_path_);

  // ── 形态学参数 ──
  this->declare_parameter<int>("morph_close_kernel", morph_close_kernel_);
  this->declare_parameter<int>("morph_close_iters", morph_close_iters_);
  this->get_parameter("morph_close_kernel", morph_close_kernel_);
  this->get_parameter("morph_close_iters", morph_close_iters_);

  // ── HoughCircles ──
  this->declare_parameter<double>("hough_dp", hough_dp_);
  this->declare_parameter<double>("hough_min_dist", hough_min_dist_);
  this->declare_parameter<double>("hough_param1", hough_param1_);
  this->declare_parameter<double>("hough_param2", hough_param2_);
  this->declare_parameter<int>("hough_min_radius", hough_min_radius_);
  this->declare_parameter<int>("hough_max_radius", hough_max_radius_);
  this->get_parameter("hough_dp", hough_dp_);
  this->get_parameter("hough_min_dist", hough_min_dist_);
  this->get_parameter("hough_param1", hough_param1_);
  this->get_parameter("hough_param2", hough_param2_);
  this->get_parameter("hough_min_radius", hough_min_radius_);
  this->get_parameter("hough_max_radius", hough_max_radius_);

  // ── 验证阈值 ──
  this->declare_parameter<double>("min_golden_ratio", min_golden_ratio_);
  this->declare_parameter<double>("min_circularity", min_circularity_);
  this->get_parameter("min_golden_ratio", min_golden_ratio_);
  this->get_parameter("min_circularity", min_circularity_);

  // ── Fallback ──
  this->declare_parameter<int>("contour_min_area", contour_min_area_);
  this->declare_parameter<double>("contour_min_golden_ratio", contour_min_golden_ratio_);
  this->get_parameter("contour_min_area", contour_min_area_);
  this->get_parameter("contour_min_golden_ratio", contour_min_golden_ratio_);

  // ── 相机参数 ──
  this->declare_parameter<double>("focal_length_px", focal_length_px_);
  this->declare_parameter<double>("mustika_real_radius", mustika_real_radius_);
  this->declare_parameter<double>("min_distance", min_distance_);
  this->declare_parameter<double>("max_distance", max_distance_);
  this->get_parameter("focal_length_px", focal_length_px_);
  this->get_parameter("mustika_real_radius", mustika_real_radius_);
  this->get_parameter("min_distance", min_distance_);
  this->get_parameter("max_distance", max_distance_);

  // ── 参数规范化 ──
  morph_close_kernel_ = std::max(3, std::min(morph_close_kernel_, 31));
  morph_close_iters_  = std::max(1, std::min(morph_close_iters_, 10));
  hough_dp_           = std::max(0.5, std::min(hough_dp_, 4.0));
  hough_min_dist_     = std::max(5.0, std::min(hough_min_dist_, 500.0));
  hough_param1_       = std::max(10.0, std::min(hough_param1_, 500.0));
  hough_param2_       = std::max(5.0, std::min(hough_param2_, 300.0));
  min_golden_ratio_   = std::max(0.1, std::min(min_golden_ratio_, 1.0));
  min_circularity_    = std::max(0.1, std::min(min_circularity_, 1.0));
  contour_min_area_   = std::max(10, contour_min_area_);
  focal_length_px_    = std::max(100.0, focal_length_px_);
  mustika_real_radius_= std::max(0.01, mustika_real_radius_);
  min_distance_       = std::max(0.05, min_distance_);
  max_distance_       = std::max(min_distance_ + 0.1, max_distance_);

  // ── 半径搜索窗：**从距离派生**（2026-09-20 修）──
  //   针孔模型 r_px = focal_length_px_ × mustika_real_radius_ / d
  //     · 最近的球 (min_distance_) → 最大像素半径 → hough_max_radius_
  //     · 最远的球 (max_distance_) → 最小像素半径 → hough_min_radius_
  //
  //   ⚠️ 旧实现让这两个值**各写各的**：min=10 / max=150 等价于 6.3m ~ 0.42m，
  //      而 min_distance_ 声明的是 0.2m（需要 315px）——
  //      **球在 0.42m 以内时 HoughCircles 直接框不到，静默返回 detected=false。**
  //      而那正是"把球放到柱顶凹槽"那个动作的距离。
  //
  //   ⚠️ 放在 min_distance_ / max_distance_ **规范化之后** —— 否则会拿脏值派生。
  //   ⚠️ 用 ceil/floor 把区间**向外取整**，宁可窗略宽（多几个候选给验证层筛），
  //      也不能因为取整把真球的半径挤出窗外。
  hough_max_radius_ = static_cast<int>(std::ceil(
      focal_length_px_ * mustika_real_radius_ / min_distance_));
  hough_min_radius_ = static_cast<int>(std::floor(
      focal_length_px_ * mustika_real_radius_ / max_distance_));
  // 兜底：下界至少 3px、上界至少比下界大 1（OpenCV 会拒绝退化窗口）
  hough_min_radius_ = std::max(3, hough_min_radius_);
  hough_max_radius_ = std::max(hough_min_radius_ + 1, hough_max_radius_);

  // ── 加载黄底 HSV 阈值 ──
  // ✅ 2026-09-20: 键名 golden_mustika → yellow_ball。
  //    官方 v1.1 第 10 页写明实物是 Mikasa V300W **排球**(黄底+蓝纹+白字),
  //    不是金色球; 相应配置项见 color_thresholds.yaml 的 yellow_ball /
  //    blue_mark / white_mark + yellow_min_pixel_ratio / circularity_threshold。
  try {
    auto all_thresholds = br_perception::vision::load_color_thresholds(color_yaml_path_);
    auto it = all_thresholds.find("yellow_ball");
    if (it != all_thresholds.end()) {
      for (const auto& group : it->second.groups) {
        HsvGroup hg;
        hg.lower = group.lower;
        hg.upper = group.upper;
        golden_groups_.push_back(hg);
      }
      RCLCPP_INFO(this->get_logger(),
        "Loaded %zu yellow HSV groups from %s (v1.1 volleyball model)",
        golden_groups_.size(), color_yaml_path_.c_str());
    } else {
      // ── 硬编码 fallback: 排球黄底 H: 20-50, S: 60-255, V: 100-255 ──
      RCLCPP_WARN(this->get_logger(),
        "No 'yellow_ball' key in %s, using hardcoded thresholds",
        color_yaml_path_.c_str());
      golden_groups_.push_back({
        cv::Scalar(20, 60,  100),
        cv::Scalar(50, 255, 255)
      });
      golden_groups_.push_back({
        cv::Scalar(18, 50,  80),
        cv::Scalar(55, 255, 230)
      });
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to load color YAML (%s), using hardcoded thresholds: %s",
      color_yaml_path_.c_str(), e.what());
    golden_groups_.push_back({
      cv::Scalar(20, 60,  100),
      cv::Scalar(50, 255, 255)
    });
    golden_groups_.push_back({
      cv::Scalar(18, 50,  80),
      cv::Scalar(55, 255, 230)
    });
  }

  // ── 动态参数更新 ──
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MustikaDetector::on_parameter_change, this, std::placeholders::_1));

  // ── 订阅预处理图像 (宽/长焦双路) ──
  // ⚠ BEST_EFFORT + **队列深度 1**（任务书 §3.0 硬约束第 2 条）——
  //    完整理由写在 `image_preprocessor.cpp` 的 `stream_qos` 那一段。
  //    **上游发布端与本处必须同为 depth=1**, 只改一侧等于没改。
  const auto stream_qos = rclcpp::QoS(1).best_effort();

  sub_wide_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
    "/perception/camera_wide/preprocessed", stream_qos,
    std::bind(&MustikaDetector::wide_callback, this, std::placeholders::_1));

  sub_tele_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
    "/perception/camera_tele/preprocessed", stream_qos,
    std::bind(&MustikaDetector::tele_callback, this, std::placeholders::_1));

  // ── 发布穆斯蒂卡检测结果 ──
  pub_wide_ = this->create_publisher<br_perception::msg::CameraMustika>(
    "/perception/camera/mustika_wide", stream_qos);
  pub_tele_ = this->create_publisher<br_perception::msg::CameraMustika>(
    "/perception/camera/mustika_tele", stream_qos);

  // ── 诊断 ──
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/camera/mustika_diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "MustikaDetector ready | "
    "golden_groups=%zu | morph=%dx%d | "
    "hough(dp=%.1f minDist=%.0f p1=%.0f p2=%.0f r=[%d,%d]) | "
    "validate(golden>%.0f%% circ>%.2f) | "
    "distance=[%.2f, %.2f]m",
    golden_groups_.size(),
    morph_close_kernel_, morph_close_iters_,
    hough_dp_, hough_min_dist_, hough_param1_, hough_param2_,
    hough_min_radius_, hough_max_radius_,
    min_golden_ratio_ * 100.0, min_circularity_,
    min_distance_, max_distance_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 前向相机回调
// ═══════════════════════════════════════════════════════════════════════════════
void MustikaDetector::wide_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg)
{
  process_and_publish(msg, 0, pub_wide_);
  wide_frame_seq_++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 后向相机回调
// ═══════════════════════════════════════════════════════════════════════════════
void MustikaDetector::tele_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg)
{
  process_and_publish(msg, 1, pub_tele_);
  tele_frame_seq_++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 单路相机通用处理
// ═══════════════════════════════════════════════════════════════════════════════
void MustikaDetector::process_and_publish(
    const br_perception::msg::PreprocessedImage::SharedPtr msg,
    uint8_t camera_id,
    const rclcpp::Publisher<br_perception::msg::CameraMustika>::SharedPtr& pub)
{
  const char* cam_name = (camera_id == 0) ? "wide" : "tele";

  // ── 计时起点 ──
  auto t0 = std::chrono::steady_clock::now();

  // ── 1. ROS Image → cv::Mat (BGR8) ──
  cv::Mat bgr;
  try {
    // msg->image 是 PreprocessedImage 的**内嵌字段**, 不是独立的 shared_ptr
    //   → 用带 tracked_object 的重载 (cv_bridge.h 注明就是为"消息里包着 Image"准备的)。
    //   数据生命周期由 msg 保证 (回调期间一直活着), 且**仍是零拷贝**。
    //   ⚠️ 别图省事改成 toCvCopy —— 那会多一次整图 memcpy, 白扔预处理的一次拷贝。
    auto cv_ptr = cv_bridge::toCvShare(msg->image, msg, sensor_msgs::image_encodings::BGR8);
    bgr = cv_ptr->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] cv_bridge error: %s", cam_name, e.what());
    return;
  }

  if (bgr.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] Empty image received", cam_name);
    return;
  }

  // ── 2. 检测管线 ──
  MustikaImageResult result = detect(bgr, cam_name);

  // ── 3. 发布 CameraMustika 消息 ──
  // ── 反算: letterbox 坐标 → 原图坐标（2026-09-20 新增, P0-3）──
  //   detect() 输入的是预处理后的 letterbox 图, 所以 result.u/v/radius 都在
  //   letterbox 坐标系里。**发出去之前必须反算回原图** —— 下游的 2D→3D 投影
  //   用的是原图内参 K, 坐标不换会整体偏移 (近处看不出、远处错得离谱)。
  //
  //   ⚠️ **三个量都要换**。只换中心点、不换半径, 下游会拿到错误的尺寸
  //      （半径只除 scale、不减填充偏移 —— 见 letterbox_to_orig 的实现）。
  double u_orig = result.u;
  double v_orig = result.v;
  double r_orig = result.radius;
  letterbox_to_orig(msg->scale, msg->pad_x, msg->pad_y, u_orig, v_orig, r_orig);

  br_perception::msg::CameraMustika out_msg;
  out_msg.camera_id  = camera_id;
  out_msg.u          = static_cast<float>(u_orig);
  out_msg.v          = static_cast<float>(v_orig);
  out_msg.radius     = static_cast<float>(r_orig);
  out_msg.confidence = result.confidence;
  out_msg.detected   = result.detected;
  // ⚠️ 时间戳 = 曝光时刻, **从输入图继承**, 禁止用 now()（见任务书 §四 时间戳约定）
  out_msg.header = msg->header;
  pub->publish(out_msg);

  // ── 4. 更新 "在视野中" 状态 ──
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (camera_id == 0) wide_in_view_ = result.detected;
    else                tele_in_view_  = result.detected;
  }

  // ── 5. 诊断 ──
  auto t1 = std::chrono::steady_clock::now();
  double total_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
  publish_diagnostics(camera_id, total_ms, result, bgr.cols, bgr.rows);

  // ── 延迟超标告警 ──
  if (total_ms > 30.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[%s] Mustika detection slow: %.1f ms (%dx%d)",
      cam_name, total_ms, bgr.cols, bgr.rows);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 完整检测管线
// ═══════════════════════════════════════════════════════════════════════════════
MustikaImageResult
MustikaDetector::detect(const cv::Mat& bgr, const std::string& cam_name)
{
  MustikaImageResult result;

  // ── Step 1: 金色区域 mask ──
  cv::Mat mask;
  extract_golden_mask(bgr, mask);

  // 快速检查: 金色区域是否太少
  const double golden_area_ratio = static_cast<double>(cv::countNonZero(mask))
                                   / static_cast<double>(mask.total());
  if (golden_area_ratio < 0.001) {
    // 画面中几乎没有金色, 直接返回
    return result;
  }

  // ── Step 2: HoughCircles 检测 ──
  std::vector<cv::Vec3f> circles;
  detect_circles_hough(bgr, mask, circles);

  // ── Step 3: 验证候选圆, 选最优 ──
  if (!circles.empty()) {
    MustikaImageResult best;
    float best_conf = -1.0f;

    for (const auto& c : circles) {
      MustikaImageResult candidate;
      if (validate_circle(bgr, mask, c, candidate)) {
        if (candidate.confidence > best_conf) {
          best = candidate;
          best_conf = candidate.confidence;
        }
      }
    }

    if (best.detected) {
      best.method = DetectMethod::HOUGH_CIRCLES;
      return best;
    }
  }

  // ── Step 4: Fallback — 轮廓 + minEnclosingCircle ──
  if (fallback_contour_detect(mask, result)) {
    result.method = DetectMethod::CONTOUR_FALLBACK;
    return result;
  }

  // 未检测到
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 1: 金色区域 mask 生成
// ═══════════════════════════════════════════════════════════════════════════════
void MustikaDetector::extract_golden_mask(const cv::Mat& bgr, cv::Mat& mask)
{
  // BGR → HSV
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  // ── 对每组金色阈值做 inRange, 并集 ──
  mask = cv::Mat::zeros(hsv.size(), CV_8UC1);

  for (const auto& group : golden_groups_) {
    cv::Mat group_mask;
    cv::inRange(hsv, group.lower, group.upper, group_mask);
    cv::bitwise_or(mask, group_mask, mask);
  }

  // ── 形态学闭运算: 填补球体表面暗斑 ──
  if (morph_close_iters_ > 0) {
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(morph_close_kernel_, morph_close_kernel_));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel,
                     cv::Point(-1, -1), morph_close_iters_);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 2: HoughCircles 圆形检测
// ═══════════════════════════════════════════════════════════════════════════════
void MustikaDetector::detect_circles_hough(
    const cv::Mat& bgr,
    const cv::Mat& /*mask*/,
    std::vector<cv::Vec3f>& circles)
{
  // ── 转为灰度 ──
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  // ── 轻微高斯模糊抑制噪声 (HoughCircles 对噪声敏感) ──
  cv::Mat blurred;
  cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.0);

  // ── HoughCircles (梯度法) ──
  // HOUGH_GRADIENT 分为两步:
  //   1. Canny 边缘检测 (param1 = Canny 高阈值, 低阈值 = param1/2)
  //   2. 圆心累加器投票 (param2 = 圆心检测阈值, 越小越多候选)
  cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT,
                   hough_dp_,                // dp = 累加器分辨率反比
                   hough_min_dist_,          // minDist = 圆心最小间距
                   hough_param1_,            // param1 = Canny 高阈值
                   hough_param2_,            // param2 = 累加器阈值
                   hough_min_radius_,        // minRadius
                   hough_max_radius_);       // maxRadius
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 3: 验证候选圆
// ═══════════════════════════════════════════════════════════════════════════════
bool MustikaDetector::validate_circle(
    const cv::Mat& bgr,
    const cv::Mat& mask,
    const cv::Vec3f& circle,
    MustikaImageResult& result)
{
  const float cx = circle[0];
  const float cy = circle[1];
  const float r  = circle[2];

  // ── 3a. 半径合理性检查 ──
  // 根据真实半径、焦距和距离范围, 估算像素半径有效区间
  const int r_min_by_dist = static_cast<int>(
      (focal_length_px_ * mustika_real_radius_) / max_distance_);
  const int r_max_by_dist = static_cast<int>(
      (focal_length_px_ * mustika_real_radius_) / min_distance_);

  const int eff_min = std::max(hough_min_radius_, r_min_by_dist / 2);
  const int eff_max = std::min(hough_max_radius_, r_max_by_dist * 2);

  if (static_cast<int>(r) < eff_min || static_cast<int>(r) > eff_max) {
    return false;
  }

  // ── 3b. 构建圆形 ROI mask ──
  const int ix = static_cast<int>(cx);
  const int iy = static_cast<int>(cy);
  const int ir = static_cast<int>(r);

  // 边界检查
  if (ix - ir < 0 || ix + ir >= bgr.cols ||
      iy - ir < 0 || iy + ir >= bgr.rows) {
    return false;
  }

  cv::Mat circle_mask = cv::Mat::zeros(bgr.size(), CV_8UC1);
  cv::circle(circle_mask, cv::Point(ix, iy), ir, cv::Scalar(255), cv::FILLED);

  // ── 3c. 圆内金色像素占比 ──
  // 只统计圆形 mask 内的像素
  cv::Mat golden_in_circle;
  cv::bitwise_and(mask, circle_mask, golden_in_circle);

  const int total_pixels = cv::countNonZero(circle_mask);
  const int golden_pixels = cv::countNonZero(golden_in_circle);
  const float golden_ratio = (total_pixels > 0)
      ? static_cast<float>(golden_pixels) / static_cast<float>(total_pixels)
      : 0.0f;

  if (golden_ratio < min_golden_ratio_) {
    return false;
  }

  // ── 3d. 圆形度验证 ──
  // 在圆内提取金色区域的轮廓, 计算圆形度
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(golden_in_circle, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  float best_circularity = 0.0f;
  for (const auto& cnt : contours) {
    const double area = cv::contourArea(cnt);
    if (area < contour_min_area_) continue;

    const float circ = compute_circularity(cnt, area);
    if (circ > best_circularity) {
      best_circularity = circ;
    }
  }

  if (best_circularity < min_circularity_) {
    return false;
  }

  // ── 3e. 综合置信度 ──
  // conf = 0.5 * normalized_golden_ratio + 0.5 * circularity
  // golden_ratio 归一化: 实际值 / 理想值(1.0), clip to [0, 1]
  const float conf_golden = std::min(1.0f, golden_ratio / 0.80f);  // 80% 即满分
  const float conf_circ   = best_circularity;                       // 已经在 [0,1]
  const float confidence  = 0.5f * conf_golden + 0.5f * conf_circ;

  // ── 填充结果 ──
  result.detected     = true;
  result.u            = cx;
  result.v            = cy;
  result.radius       = r;
  result.confidence   = confidence;
  result.golden_ratio = golden_ratio;
  result.circularity  = best_circularity;
  result.method       = DetectMethod::NONE;  // caller fills

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 4 (Fallback): 金色轮廓 + minEnclosingCircle
// ═══════════════════════════════════════════════════════════════════════════════
bool MustikaDetector::fallback_contour_detect(
    const cv::Mat& mask,
    MustikaImageResult& result)
{
  // ── 提取所有轮廓 ──
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) return false;

  // ── 筛选 & 评分 ──
  struct Candidate {
    cv::Point2f center;
    float radius;
    float golden_ratio;
    float circularity;
    float confidence;
  };

  std::vector<Candidate> candidates;

  for (const auto& cnt : contours) {
    const double area = cv::contourArea(cnt);
    if (area < contour_min_area_) continue;

    // minEnclosingCircle
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(cnt, center, radius);

    // 半径合理性
    const int r_min_by_dist = static_cast<int>(
        (focal_length_px_ * mustika_real_radius_) / max_distance_);
    const int r_max_by_dist = static_cast<int>(
        (focal_length_px_ * mustika_real_radius_) / min_distance_);
    if (static_cast<int>(radius) < r_min_by_dist / 2 ||
        static_cast<int>(radius) > r_max_by_dist * 2) {
      continue;
    }

    // 构建圆形 mask 并计算金色像素占比
    cv::Mat circle_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::circle(circle_mask, center, static_cast<int>(radius),
               cv::Scalar(255), cv::FILLED);

    cv::Mat golden_in_circle;
    cv::bitwise_and(mask, circle_mask, golden_in_circle);

    const int total_px = cv::countNonZero(circle_mask);
    const int gold_px = cv::countNonZero(golden_in_circle);
    const float golden_ratio = (total_px > 0)
        ? static_cast<float>(gold_px) / static_cast<float>(total_px)
        : 0.0f;

    // Fallback 放宽金色占比阈值
    if (golden_ratio < contour_min_golden_ratio_) continue;

    // 圆形度
    const float circ = compute_circularity(cnt, area);
    if (circ < min_circularity_) continue;

    // 置信度
    const float conf_golden = std::min(1.0f, golden_ratio / 0.80f);
    const float confidence = 0.5f * conf_golden + 0.5f * circ;

    candidates.push_back({center, radius, golden_ratio, circ, confidence});
  }

  if (candidates.empty()) return false;

  // ── 选置信度最高的 ──
  const auto& best = *std::max_element(candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& b) {
        return a.confidence < b.confidence;
      });

  result.detected     = true;
  result.u            = best.center.x;
  result.v            = best.center.y;
  result.radius       = best.radius;
  result.confidence   = best.confidence;
  result.golden_ratio = best.golden_ratio;
  result.circularity  = best.circularity;

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 工具: 计算圆形度
// ═══════════════════════════════════════════════════════════════════════════════
float MustikaDetector::compute_circularity(
    const std::vector<cv::Point>& contour, double area)
{
  if (area <= 0.0) return 0.0f;

  const double perimeter = cv::arcLength(contour, true);
  if (perimeter <= 0.0) return 0.0f;

  // circularity = 4π × area / perimeter²
  // 正圆 = 1.0, 偏离圆形则 < 1.0
  const double circ = (4.0 * M_PI * area) / (perimeter * perimeter);

  // 数值修正: 由于离散化, 正圆可能 > 1.0 (极少见), clamp
  return static_cast<float>(std::min(circ, 1.0));
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断发布
// ═══════════════════════════════════════════════════════════════════════════════
void MustikaDetector::publish_diagnostics(
    uint8_t camera_id,
    double total_ms,
    const MustikaImageResult& result,
    int image_w, int image_h)
{
  std::lock_guard<std::mutex> lock(diag_mutex_);

  const char* cam_name = (camera_id == 0) ? "wide" : "tele";

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string("mustika_detector_") + cam_name;
  status.hardware_id = std::string("camera_") + cam_name;

  // 总体健康判定
  if (total_ms < 20.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  } else if (total_ms < 40.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "SLOW";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "VERY SLOW";
  }

  auto add_kv = [&](const std::string& key, double val) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = std::to_string(val);
    status.values.push_back(kv);
  };

  add_kv("total_ms",     total_ms);
  add_kv("detected",     result.detected ? 1.0 : 0.0);
  add_kv("u",            static_cast<double>(result.u));
  add_kv("v",            static_cast<double>(result.v));
  add_kv("radius",       static_cast<double>(result.radius));
  add_kv("confidence",   static_cast<double>(result.confidence));
  add_kv("golden_ratio", static_cast<double>(result.golden_ratio));
  add_kv("circularity",  static_cast<double>(result.circularity));
  add_kv("method",       static_cast<double>(static_cast<uint8_t>(result.method)));
  add_kv("image_w",      static_cast<double>(image_w));
  add_kv("image_h",      static_cast<double>(image_h));
  add_kv("frame_seq",    static_cast<double>(
      (camera_id == 0) ? wide_frame_seq_ : tele_frame_seq_));

  // 在视野中的状态 (前后任一)
  bool any_in_view = false;
  {
    std::lock_guard<std::mutex> slock(state_mutex_);
    any_in_view = wide_in_view_ || tele_in_view_;
  }
  add_kv("any_in_view", any_in_view ? 1.0 : 0.0);

  diag_array.status.push_back(status);
  pub_diagnostics_->publish(diag_array);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult MustikaDetector::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto& param : params) {
    const std::string& name = param.get_name();

    try {
      if (name == "morph_close_kernel") {
        int val = param.as_int();
        if (val < 3 || val > 31) {
          result.successful = false;
          result.reason = "morph_close_kernel must be in [3, 31]";
          break;
        }
        morph_close_kernel_ = val;
      }
      else if (name == "morph_close_iters") {
        int val = param.as_int();
        if (val < 0 || val > 10) {
          result.successful = false;
          result.reason = "morph_close_iters must be in [0, 10]";
          break;
        }
        morph_close_iters_ = val;
      }
      else if (name == "hough_dp") {
        double val = param.as_double();
        if (val < 0.5 || val > 4.0) {
          result.successful = false;
          result.reason = "hough_dp must be in [0.5, 4.0]";
          break;
        }
        hough_dp_ = val;
      }
      else if (name == "hough_min_dist") {
        double val = param.as_double();
        if (val < 5.0 || val > 500.0) {
          result.successful = false;
          result.reason = "hough_min_dist must be in [5, 500]";
          break;
        }
        hough_min_dist_ = val;
      }
      else if (name == "hough_param1") {
        double val = param.as_double();
        if (val < 10.0 || val > 500.0) {
          result.successful = false;
          result.reason = "hough_param1 must be in [10, 500]";
          break;
        }
        hough_param1_ = val;
      }
      else if (name == "hough_param2") {
        double val = param.as_double();
        if (val < 5.0 || val > 300.0) {
          result.successful = false;
          result.reason = "hough_param2 must be in [5, 300]";
          break;
        }
        hough_param2_ = val;
      }
      else if (name == "hough_min_radius") {
        int val = param.as_int();
        if (val < 3 || val > 500) {
          result.successful = false;
          result.reason = "hough_min_radius must be in [3, 500]";
          break;
        }
        hough_min_radius_ = val;
        hough_max_radius_ = std::max(hough_max_radius_, hough_min_radius_ + 1);
      }
      else if (name == "hough_max_radius") {
        int val = param.as_int();
        if (val <= hough_min_radius_ || val > 1000) {
          result.successful = false;
          result.reason = "hough_max_radius must be > min_radius and <= 1000";
          break;
        }
        hough_max_radius_ = val;
      }
      else if (name == "min_golden_ratio") {
        double val = param.as_double();
        if (val < 0.1 || val > 1.0) {
          result.successful = false;
          result.reason = "min_golden_ratio must be in [0.1, 1.0]";
          break;
        }
        min_golden_ratio_ = val;
      }
      else if (name == "min_circularity") {
        double val = param.as_double();
        if (val < 0.1 || val > 1.0) {
          result.successful = false;
          result.reason = "min_circularity must be in [0.1, 1.0]";
          break;
        }
        min_circularity_ = val;
      }
      else if (name == "contour_min_area") {
        int val = param.as_int();
        if (val < 10) {
          result.successful = false;
          result.reason = "contour_min_area must be >= 10";
          break;
        }
        contour_min_area_ = val;
      }
      else if (name == "contour_min_golden_ratio") {
        double val = param.as_double();
        if (val < 0.1 || val > 1.0) {
          result.successful = false;
          result.reason = "contour_min_golden_ratio must be in [0.1, 1.0]";
          break;
        }
        contour_min_golden_ratio_ = val;
      }
      else if (name == "focal_length_px") {
        double val = param.as_double();
        if (val < 100.0) {
          result.successful = false;
          result.reason = "focal_length_px must be >= 100";
          break;
        }
        focal_length_px_ = val;
      }
      else if (name == "min_distance") {
        double val = param.as_double();
        if (val < 0.05 || val >= max_distance_) {
          result.successful = false;
          result.reason = "min_distance must be >= 0.05 and < max_distance";
          break;
        }
        min_distance_ = val;
      }
      else if (name == "max_distance") {
        double val = param.as_double();
        if (val <= min_distance_) {
          result.successful = false;
          result.reason = "max_distance must be > min_distance";
          break;
        }
        max_distance_ = val;
      }
      else {
        RCLCPP_DEBUG(this->get_logger(), "Ignoring unknown param: %s", name.c_str());
      }
    }
    catch (const rclcpp::ParameterTypeException& e) {
      result.successful = false;
      result.reason = std::string("Type mismatch for ") + name + ": " + e.what();
      break;
    }
  }

  if (result.successful) {
    RCLCPP_INFO(this->get_logger(),
      "Params updated | morph=%dx%d | hough(dp=%.1f,%.0f,p1=%.0f,p2=%.0f,r=[%d,%d]) | "
      "validate(gold>%.2f,circ>%.2f) | dist=[%.2f,%.2f]",
      morph_close_kernel_, morph_close_iters_,
      hough_dp_, hough_min_dist_, hough_param1_, hough_param2_,
      hough_min_radius_, hough_max_radius_,
      min_golden_ratio_, min_circularity_,
      min_distance_, max_distance_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

}  // namespace camera
}  // namespace br_perception

// ── ROS2 组件注册 ──
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::MustikaDetector)
