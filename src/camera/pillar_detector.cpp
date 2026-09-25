/// ═══════════════════════════════════════════════════════════════════════════════
/// pillar_detector.cpp — ROBOCON 2027 柱子检测实现
///
/// 检测管线 (每路相机独立):
///   RGB → HSV → 双色 mask (dark_green + brown)
///   → 形态学闭运算 → findContours
///   → 高宽比筛选 (3:1 ~ 12:1) → 竖直度验证 (minAreaRect 角度)
///   → 颜色填充率 + 尺寸合理性 → 类型判定 → 柱顶检查
///   → 发布 CameraPillar + Diagnostics
///
/// 关键设计决策:
///   - 双色独立检测: dark_green → 穆斯蒂卡柱, brown → 核心支柱
///   - 高宽比为主筛选: 柱子是细长竖直矩形, 与方形建筑位自然区分
///   - minAreaRect 角度验证竖直度: OpenCV 的 angle 属性直接给主轴偏离
///   - 柱顶区域分析: 取 bbox 顶部 15% 区域, 检查是否有非背景物体
///   - 尺寸合理性: 根据已知柱子直径/高度 + 焦距换算像素尺寸范围
///   - 静态目标的低帧率处理: 柱子不动, 偶发漏检不影响跟踪
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/pillar_detector.hpp"
// letterbox_to_orig() —— 检测框反算回原图坐标 (2026-09-20, P0-3)
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
// 构造
// ═══════════════════════════════════════════════════════════════════════════════

PillarDetector::PillarDetector(const rclcpp::NodeOptions& options)
  : Node("pillar_detector", options)
{
  // YAML 路径
  this->declare_parameter<std::string>("color_yaml_path", "config/color_thresholds.yaml");
  this->get_parameter("color_yaml_path", color_yaml_path_);

  // 形态学
  this->declare_parameter<int>("morph_close_kernel", morph_close_kernel_);
  this->declare_parameter<int>("morph_close_iters", morph_close_iters_);
  this->get_parameter("morph_close_kernel", morph_close_kernel_);
  this->get_parameter("morph_close_iters", morph_close_iters_);

  // 高宽比
  this->declare_parameter<double>("min_aspect_ratio", min_aspect_ratio_);
  this->declare_parameter<double>("max_aspect_ratio", max_aspect_ratio_);
  this->get_parameter("min_aspect_ratio", min_aspect_ratio_);
  this->get_parameter("max_aspect_ratio", max_aspect_ratio_);

  // 轮廓
  this->declare_parameter<int>("contour_min_area", contour_min_area_);
  this->declare_parameter<int>("contour_max_area", contour_max_area_);
  this->get_parameter("contour_min_area", contour_min_area_);
  this->get_parameter("contour_max_area", contour_max_area_);

  // 验证
  this->declare_parameter<double>("min_color_fill", min_color_fill_);
  this->declare_parameter<double>("max_upright_angle", max_upright_angle_);
  this->get_parameter("min_color_fill", min_color_fill_);
  this->get_parameter("max_upright_angle", max_upright_angle_);

  // 柱顶
  this->declare_parameter<double>("top_check_offset", top_check_offset_);
  this->declare_parameter<double>("top_object_threshold", top_object_threshold_);
  this->get_parameter("top_check_offset", top_check_offset_);
  this->get_parameter("top_object_threshold", top_object_threshold_);

  // 尺寸
  this->declare_parameter<double>("pillar_diameter", pillar_diameter_);
  this->declare_parameter<double>("mustika_pillar_height", mustika_pillar_height_);
  this->declare_parameter<double>("core_pillar_height", core_pillar_height_);
  this->declare_parameter<double>("focal_length_px", focal_length_px_);
  this->get_parameter("pillar_diameter", pillar_diameter_);
  this->get_parameter("mustika_pillar_height", mustika_pillar_height_);
  this->get_parameter("core_pillar_height", core_pillar_height_);
  this->get_parameter("focal_length_px", focal_length_px_);

  // 参数规范化
  morph_close_kernel_ = std::max(3, std::min(morph_close_kernel_, 31));
  morph_close_iters_  = std::max(0, std::min(morph_close_iters_, 5));
  min_aspect_ratio_   = std::max(1.5, std::min(min_aspect_ratio_, 20.0));
  max_aspect_ratio_   = std::max(min_aspect_ratio_ + 0.5, max_aspect_ratio_);
  contour_min_area_   = std::max(100, std::min(contour_min_area_, 50000));
  contour_max_area_   = std::max(contour_min_area_ + 1, contour_max_area_);
  min_color_fill_     = std::max(0.2, std::min(min_color_fill_, 1.0));
  max_upright_angle_  = std::max(3.0, std::min(max_upright_angle_, 45.0));
  top_check_offset_   = std::max(0.05, std::min(top_check_offset_, 0.40));
  top_object_threshold_ = std::max(0.10, std::min(top_object_threshold_, 0.80));

  // 加载颜色阈值
  auto load_groups = [&](const std::string& key, std::vector<HsvGroup>& dst,
                          const std::vector<HsvGroup>& fallback) {
    try {
      auto all = br_perception::vision::load_color_thresholds(color_yaml_path_);
      auto it = all.find(key);
      if (it != all.end()) {
        for (const auto& g : it->second.groups) dst.push_back({g.lower, g.upper});
        RCLCPP_INFO(this->get_logger(), "Loaded %zu '%s' HSV groups", dst.size(), key.c_str());
      } else {
        dst = fallback;
        RCLCPP_WARN(this->get_logger(), "No '%s' in YAML, using hardcoded", key.c_str());
      }
    } catch (const std::exception& e) {
      dst = fallback;
      RCLCPP_WARN(this->get_logger(), "YAML load fail (%s), hardcoded '%s'", e.what(), key.c_str());
    }
  };

  load_groups("dark_green_pillar", dark_green_groups_, {
    {cv::Scalar(40, 60, 30), cv::Scalar(80, 255, 180)},
    {cv::Scalar(35, 40, 25), cv::Scalar(85, 255, 150)}
  });

  load_groups("brown_core_pillar", brown_groups_, {
    {cv::Scalar(10, 60, 25), cv::Scalar(28, 255, 180)},
    {cv::Scalar(8,  40, 20), cv::Scalar(30, 255, 150)}
  });

  // ROS
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PillarDetector::on_parameter_change, this, std::placeholders::_1));

  // ⚠ BEST_EFFORT + **队列深度 1**（任务书 §3.0 硬约束第 2 条）——
  //    完整理由写在 `image_preprocessor.cpp` 的 `stream_qos` 那一段。
  //    **上游发布端与本处必须同为 depth=1**, 只改一侧等于没改。
  const auto stream_qos = rclcpp::QoS(1).best_effort();

  sub_wide_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
    "/perception/camera_wide/preprocessed", stream_qos,
    std::bind(&PillarDetector::wide_callback, this, std::placeholders::_1));
  sub_tele_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
    "/perception/camera_tele/preprocessed", stream_qos,
    std::bind(&PillarDetector::tele_callback, this, std::placeholders::_1));

  pub_wide_ = this->create_publisher<br_perception::msg::CameraPillar>(
    "/perception/camera/pillars_wide", stream_qos);
  pub_tele_ = this->create_publisher<br_perception::msg::CameraPillar>(
    "/perception/camera/pillars_tele", stream_qos);

  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/camera/pillar_diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "PillarDetector ready | green_groups=%zu brown_groups=%zu | "
    "morph=%dx%d | aspect=[%.1f,%.1f] | area=[%d,%d] | "
    "fill>%.0f%% upright<%.0f°",
    dark_green_groups_.size(), brown_groups_.size(),
    morph_close_kernel_, morph_close_iters_,
    min_aspect_ratio_, max_aspect_ratio_,
    contour_min_area_, contour_max_area_,
    min_color_fill_ * 100.0, max_upright_angle_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 回调
// ═══════════════════════════════════════════════════════════════════════════════
void PillarDetector::wide_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg)
{ process_and_publish(msg, 0, pub_wide_); wide_frame_seq_++; }

void PillarDetector::tele_callback(const br_perception::msg::PreprocessedImage::SharedPtr msg)
{ process_and_publish(msg, 1, pub_tele_); tele_frame_seq_++; }

// ═══════════════════════════════════════════════════════════════════════════════
// 单路处理
// ═══════════════════════════════════════════════════════════════════════════════
void PillarDetector::process_and_publish(
    const br_perception::msg::PreprocessedImage::SharedPtr msg,
    uint8_t camera_id,
    const rclcpp::Publisher<br_perception::msg::CameraPillar>::SharedPtr& pub)
{
  const char* cam_name = (camera_id == 0) ? "wide" : "tele";
  auto t0 = std::chrono::steady_clock::now();

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
  if (bgr.empty()) return;

  auto results = detect(bgr, cam_name);

  // 发布 —— ⚠️ 发布前必须把 bbox 从 letterbox 坐标反算回原图坐标（2026-09-20, P0-3）
  //   detect() 的输入是预处理后的 letterbox 图, 所以 results 里的 bbox 都在那个坐标系。
  //   下游的 2D→3D 投影用原图内参 K, 不反算会整体偏移。
  for (const auto& r : results) {
    // 中心点: 既要减填充偏移、又要除 scale
    double u = r.bbox.x + r.bbox.width * 0.5f;
    double v = r.bbox.y + r.bbox.height * 0.5f;
    // 宽高: **尺度量** —— 只除 scale, **不减填充偏移**（减了会算出负宽度）
    double w = r.bbox.width;
    double h = r.bbox.height;
    letterbox_to_orig_xy(msg->scale, msg->pad_x, msg->pad_y, u, v);
    letterbox_to_orig_size(msg->scale, w, h);

    br_perception::msg::CameraPillar out;
    out.camera_id     = camera_id;
    out.detected      = r.detected;
    out.pillar_type   = static_cast<uint8_t>(r.pillar_type);
    out.u             = static_cast<float>(u);
    out.v             = static_cast<float>(v);
    out.width         = static_cast<float>(w);
    out.height        = static_cast<float>(h);
    out.confidence    = r.confidence;
    out.has_top_object = r.top_has_object;
    out.header        = msg->header;   // ⚠️ 曝光时刻, 禁止 now()
    pub->publish(out);
  }

  if (results.empty()) {
    br_perception::msg::CameraPillar out;
    out.camera_id = camera_id;
    out.detected  = false;
    out.header    = msg->header;
    pub->publish(out);
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (camera_id == 0) wide_pillar_count_ = static_cast<int>(results.size());
    else                tele_pillar_count_  = static_cast<int>(results.size());
  }

  auto t1 = std::chrono::steady_clock::now();
  double total_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
  publish_diagnostics(camera_id, total_ms, results, bgr.cols, bgr.rows);

  if (total_ms > 35.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[%s] Pillar detection slow: %.1f ms", cam_name, total_ms);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 完整检测管线
// ═══════════════════════════════════════════════════════════════════════════════
std::vector<PillarImageResult>
PillarDetector::detect(const cv::Mat& bgr, const std::string& /*cam_name*/)
{
  std::vector<PillarImageResult> results;

  // Step 1: 双色 mask
  cv::Mat green_mask, brown_mask;
  extract_pillar_mask(bgr, green_mask, brown_mask);

  // 处理每种颜色 mask
  auto process_mask = [&](const cv::Mat& mask,
                           PillarImageResult::Type color_type) {
    std::vector<cv::Rect> bboxes;
    std::vector<std::vector<cv::Point>> contours;
    find_pillar_candidates(mask, bboxes, contours);

    for (size_t i = 0; i < bboxes.size(); ++i) {
      PillarImageResult r;
      if (validate_pillar(bgr, mask, bboxes[i], contours[i], r)) {
        // 主分类: 由颜色 mask 来源决定
        r.pillar_type = color_type;

        // 二次验证: classify_pillar 在 bbox 内直接统计双色像素占比，
        //          与 mask 来源交叉校验，检测"绿色 mask 误匹配棕色柱子"等异常
        auto verify_type = classify_pillar(bgr, bboxes[i]);
        if (verify_type == PillarImageResult::UNKNOWN) {
          // classify 无法判定 → mask 类型保留，降置信度
          r.confidence *= 0.7f;
        } else if (verify_type != color_type) {
          // classify 与 mask 来源矛盾 → 以 classify 为准（bbox 内直接统计更可靠）
          r.pillar_type = verify_type;
          r.confidence *= 0.5f;
        }
        // else: 一致 → 不降置信度

        // 柱顶检查
        float occupancy = 0.0f;
        r.top_has_object = check_top_region(bgr, bboxes[i], occupancy);
        r.top_region_occupancy = occupancy;
        results.push_back(r);
      }
    }
  };

  process_mask(green_mask, PillarImageResult::MUSTIKA_PILLAR);
  process_mask(brown_mask, PillarImageResult::CORE_PILLAR);

  // 按置信度降序
  std::sort(results.begin(), results.end(),
    [](const PillarImageResult& a, const PillarImageResult& b) {
      return a.confidence > b.confidence;
    });

  return results;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 1: 双色 mask
// ═══════════════════════════════════════════════════════════════════════════════
void PillarDetector::extract_pillar_mask(const cv::Mat& bgr,
                                          cv::Mat& green_mask,
                                          cv::Mat& brown_mask)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  auto build_mask = [&](const std::vector<HsvGroup>& groups) {
    cv::Mat m = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto& g : groups) {
      cv::Mat gm;
      cv::inRange(hsv, g.lower, g.upper, gm);
      cv::bitwise_or(m, gm, m);
    }
    if (morph_close_iters_ > 0) {
      cv::Mat k = cv::getStructuringElement(
          cv::MORPH_RECT,
          cv::Size(morph_close_kernel_, morph_close_kernel_));
      cv::morphologyEx(m, m, cv::MORPH_CLOSE, k,
                       cv::Point(-1, -1), morph_close_iters_);
    }
    return m;
  };

  green_mask = build_mask(dark_green_groups_);
  brown_mask = build_mask(brown_groups_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 2: 找柱子候选 — 高宽比筛选
// ═══════════════════════════════════════════════════════════════════════════════
void PillarDetector::find_pillar_candidates(
    const cv::Mat& mask,
    std::vector<cv::Rect>& bboxes,
    std::vector<std::vector<cv::Point>>& contours_out)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& cnt : contours) {
    const double area = cv::contourArea(cnt);
    if (area < contour_min_area_ || area > contour_max_area_) continue;

    cv::Rect bbox = cv::boundingRect(cnt);

    // 高宽比
    if (bbox.height < 1 || bbox.width < 1) continue;
    const float ar = static_cast<float>(bbox.height) / static_cast<float>(bbox.width);
    if (ar < min_aspect_ratio_ || ar > max_aspect_ratio_) continue;

    // 用 minAreaRect 检查竖直度
    cv::RotatedRect rrect = cv::minAreaRect(cnt);
    // OpenCV: angle ∈ [-90, 0), 0° = 竖直 (矩形高 > 宽时)
    float angle = rrect.angle;
    // 规范化到 [0, 90)
    if (angle < -45.0f) angle += 90.0f;
    if (angle < 0.0f)   angle += 90.0f;
    // 现在 angle = 偏离水平的角度, 竖直 = 接近 90°
    const float deviation = std::abs(angle - 90.0f);
    if (deviation > max_upright_angle_ && deviation < (180.0f - max_upright_angle_)) continue;

    bboxes.push_back(bbox);
    contours_out.push_back(cnt);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 3: 验证柱子
// ═══════════════════════════════════════════════════════════════════════════════
bool PillarDetector::validate_pillar(
    const cv::Mat& bgr,
    const cv::Mat& mask,
    const cv::Rect& bbox,
    const std::vector<cv::Point>& contour,
    PillarImageResult& result,
    const PillarParams& params)
{
  // ── 3a. 包围盒内颜色像素填充率 ──
  cv::Rect valid_bbox = bbox & cv::Rect(0, 0, mask.cols, mask.rows);
  if (valid_bbox.width <= 0 || valid_bbox.height <= 0) return false;

  cv::Mat mask_roi = mask(valid_bbox);
  const int total_px = valid_bbox.area();
  const int fill_px  = cv::countNonZero(mask_roi);
  const float fill_ratio = static_cast<float>(fill_px) / static_cast<float>(total_px);

  if (fill_ratio < params.min_color_fill) return false;

  // ── 3b. 尺寸合理性 ──
  // 根据真实高度和焦距估算预期的像素高度范围
  // 使用 mustika_pillar (0.5m) 和 core_pillar (0.8m) 的平均
  const float avg_height_m = (params.mustika_pillar_height + params.core_pillar_height) * 0.5f;
  // 距离未知，用柱子像素宽度估算距离
  // distance ≈ (focal * diameter) / pixel_width
  const float est_distance = (params.focal_length_px * params.pillar_diameter)
                             / std::max(static_cast<float>(bbox.width), 1.0f);
  const float expected_height_px = (params.focal_length_px * avg_height_m) / est_distance;

  // 允许 ±50% 偏差 (透视和检测误差)
  const float h = static_cast<float>(bbox.height);
  // ⚠️⚠️ **2026-09-25 修：原来这里是死代码** —— 上面的 `est_distance` /
  //    `expected_height_px` 算完之后**什么都不做**，而注释承诺"不做硬拒绝，
  //    **降低后续评分**" ⇒ 那句承诺**没有任何实现**（"配了不生效"里最难发现的一种）。
  //    ⇒ 现在把它**接成一个乘子**（不是硬拒 —— 按注释与设计意图）：
  const float size_ratio = (expected_height_px > 1e-3f) ? (h / expected_height_px) : 1.0f;
  float size_penalty = 1.0f;   // 1.0 = 完全不罚
  if (size_ratio < 0.5f) {
    // 比预期**矮**得多 ⇒ 可能不是柱子（是方块的侧面/噪点）。越矮罚得越多，封底 0.5。
    size_penalty = std::max(0.5f, 0.5f + size_ratio);
  } else if (size_ratio > 1.5f) {
    // 比预期**高**得多 ⇒ 可能是近处大柱子（合理）或误并了两根。同样封底 0.5。
    size_penalty = std::max(0.5f, 2.5f - size_ratio);
  }
  // ⚠️ **封底 0.5**：近/远两端都"可能是对的"（注释原话），所以**不给 0 分** ——
  //    0 分等于硬拒，那与"不做硬拒绝"矛盾。

  // ── 3c. 竖直度评分 ──
  cv::RotatedRect rrect = cv::minAreaRect(contour);
  float angle = rrect.angle;
  if (angle < -45.0f) angle += 90.0f;
  if (angle < 0.0f)   angle += 90.0f;
  const float deviation = std::abs(angle - 90.0f);
  // 0° 偏差 = 1.0, params.max_upright_angle 偏差 = 0.0
  const float upright_score = 1.0f - std::min(deviation / static_cast<float>(params.max_upright_angle), 1.0f);

  // ── 3d. 轮廓面积 vs 包围盒面积 ──
  const double contour_area = cv::contourArea(contour);
  const double bbox_area = static_cast<double>(bbox.width) * bbox.height;
  const float solidity = (bbox_area > 0.0)
      ? static_cast<float>(contour_area / bbox_area) : 0.0f;

  // ── 综合置信度 ──
  const float conf_fill     = std::min(1.0f, fill_ratio / 0.80f);
  const float conf_upright  = upright_score;
  const float conf_solid    = std::min(1.0f, solidity / 0.60f);

  // ⚠️ `size_penalty` 乘在**总分**上（三项权重的比例不变，整体按尺寸合理性打折）
  const float confidence =
      size_penalty * (0.40f * conf_fill + 0.35f * conf_upright + 0.25f * conf_solid);

  // ── 填充结果 ──
  result.detected     = true;
  result.confidence   = confidence;
  result.bbox         = cv::Rect2f(static_cast<float>(bbox.x),
                                    static_cast<float>(bbox.y),
                                    static_cast<float>(bbox.width),
                                    static_cast<float>(bbox.height));
  result.aspect_ratio  = static_cast<float>(bbox.height)
                         / std::max(static_cast<float>(bbox.width), 1.0f);
  result.color_fill_ratio = fill_ratio;
  result.upright_score    = upright_score;

  return true;
}
// ── 成员薄包装：行为逐位不变 ──
bool PillarDetector::validate_pillar(
    const cv::Mat& bgr, const cv::Mat& mask, const cv::Rect& bbox,
    const std::vector<cv::Point>& contour, PillarImageResult& result)
{
  return validate_pillar(bgr, mask, bbox, contour, result, make_params());
}

PillarDetector::PillarParams PillarDetector::make_params() const
{
  PillarParams p;
  p.min_color_fill         = min_color_fill_;
  p.max_upright_angle      = max_upright_angle_;
  p.pillar_diameter        = pillar_diameter_;
  p.mustika_pillar_height  = mustika_pillar_height_;
  p.core_pillar_height     = core_pillar_height_;
  p.focal_length_px        = focal_length_px_;
  return p;
}


// ═══════════════════════════════════════════════════════════════════════════════
// 类型判定: 已在 detect() 中由颜色 mask 决定，这里做二次确认
// ═══════════════════════════════════════════════════════════════════════════════
PillarImageResult::Type
PillarDetector::classify_pillar(const cv::Mat& bgr, const cv::Rect& bbox)
{
  // 在 bbox 内分别统计深绿色和棕色像素占比，选占比高的
  cv::Mat roi = bgr(bbox & cv::Rect(0, 0, bgr.cols, bgr.rows));
  if (roi.empty()) return PillarImageResult::UNKNOWN;

  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

  auto count = [&](const std::vector<HsvGroup>& groups) -> int {
    int c = 0;
    for (const auto& g : groups) {
      cv::Mat gm;
      cv::inRange(hsv, g.lower, g.upper, gm);
      c += cv::countNonZero(gm);
    }
    return c;
  };

  const int green_cnt = count(dark_green_groups_);
  const int brown_cnt = count(brown_groups_);

  if (green_cnt > brown_cnt * 2.0f) return PillarImageResult::MUSTIKA_PILLAR;
  if (brown_cnt > green_cnt * 2.0f) return PillarImageResult::CORE_PILLAR;
  return PillarImageResult::UNKNOWN;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 柱顶区域检查
// ═══════════════════════════════════════════════════════════════════════════════
bool PillarDetector::check_top_region(
    const cv::Mat& bgr, const cv::Rect& bbox, float& occupancy)
{
  // 取 bbox 顶部 top_check_offset_ 比例的区域
  const int top_h = static_cast<int>(bbox.height * top_check_offset_);
  if (top_h < 5) { occupancy = 0.0f; return false; }

  cv::Rect top_region(bbox.x, bbox.y, bbox.width, top_h);
  top_region &= cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (top_region.width <= 0 || top_region.height <= 0) { occupancy = 0.0f; return false; }

  cv::Mat top_roi = bgr(top_region);

  // 转灰度，用边缘密度判断是否有物体
  cv::Mat gray;
  cv::cvtColor(top_roi, gray, cv::COLOR_BGR2GRAY);

  // Canny 边缘检测
  cv::Mat edges;
  cv::Canny(gray, edges, 50, 150);

  const int edge_px = cv::countNonZero(edges);
  const int total_px = top_region.area();

  // 边缘像素占比作为 "有物体" 的指标
  // 空柱顶 → 基本是纯色 (边缘少); 有物体 → 边缘多
  occupancy = static_cast<float>(edge_px) / static_cast<float>(std::max(total_px, 1));

  return (occupancy > top_object_threshold_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════════
void PillarDetector::publish_diagnostics(
    uint8_t camera_id, double total_ms,
    const std::vector<PillarImageResult>& results,
    int image_w, int image_h)
{
  std::lock_guard<std::mutex> lock(diag_mutex_);
  const char* cam_name = (camera_id == 0) ? "wide" : "tele";

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = std::string("pillar_detector_") + cam_name;
  st.hardware_id = std::string("camera_") + cam_name;

  if (total_ms < 20.0)       { st.level = diagnostic_msgs::msg::DiagnosticStatus::OK; st.message = "OK"; }
  else if (total_ms < 35.0)  { st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN; st.message = "SLOW"; }
  else                       { st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR; st.message = "VERY SLOW"; }

  auto add = [&](const std::string& k, double v) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = k; kv.value = std::to_string(v);
    st.values.push_back(kv);
  };

  add("total_ms", total_ms);
  add("pillar_count", static_cast<double>(results.size()));
  add("image_w", static_cast<double>(image_w));
  add("image_h", static_cast<double>(image_h));
  add("frame_seq", static_cast<double>(
      (camera_id == 0) ? wide_frame_seq_ : tele_frame_seq_));

  int mc = 0, cc = 0, tc = 0;
  for (const auto& r : results) {
    if (r.pillar_type == PillarImageResult::MUSTIKA_PILLAR) mc++;
    if (r.pillar_type == PillarImageResult::CORE_PILLAR)    cc++;
    if (r.top_has_object) tc++;
  }
  add("mustika_pillar", mc);
  add("core_pillar", cc);
  add("top_object", tc);

  if (!results.empty()) {
    add("best_conf", static_cast<double>(results[0].confidence));
  }

  arr.status.push_back(st);
  pub_diagnostics_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 动态参数
// ═══════════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult PillarDetector::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult r; r.successful = true;
  for (const auto& p : params) {
    try {
      const std::string& n = p.get_name();
           if (n == "morph_close_kernel")      morph_close_kernel_ = std::max(3, std::min(static_cast<int>(p.as_int()), 31));
      else if (n == "morph_close_iters")       morph_close_iters_  = std::max(0, std::min(static_cast<int>(p.as_int()), 5));
      else if (n == "min_aspect_ratio")        min_aspect_ratio_   = std::max(1.5, std::min(p.as_double(), 20.0));
      else if (n == "max_aspect_ratio")        max_aspect_ratio_   = std::max(min_aspect_ratio_+0.5, p.as_double());
      else if (n == "contour_min_area")        contour_min_area_   = std::max(100, static_cast<int>(p.as_int()));
      else if (n == "contour_max_area")        contour_max_area_   = std::max(contour_min_area_+1, static_cast<int>(p.as_int()));
      else if (n == "min_color_fill")          min_color_fill_     = std::max(0.2, std::min(p.as_double(), 1.0));
      else if (n == "max_upright_angle")       max_upright_angle_  = std::max(3.0, std::min(p.as_double(), 45.0));
      else if (n == "top_check_offset")        top_check_offset_   = std::max(0.05, std::min(p.as_double(), 0.40));
      else if (n == "top_object_threshold")    top_object_threshold_ = std::max(0.10, std::min(p.as_double(), 0.80));
      else if (n == "focal_length_px")         focal_length_px_    = std::max(100.0, p.as_double());
      else RCLCPP_DEBUG(this->get_logger(), "Unknown param: %s", n.c_str());
    } catch (const rclcpp::ParameterTypeException& e) {
      r.successful = false; r.reason = std::string("Type: ") + p.get_name();
    }
  }
  return r;
}

}  // namespace camera
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::PillarDetector)
