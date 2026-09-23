/// ═══════════════════════════════════════════════════════════════════════════════
/// image_preprocessor.cpp — ROBOCON 2027 双相机图像预处理管线实现
///
/// 管线 (每路独立):
///   ROS Image → CV::Mat(BGR) → ROI 裁剪 → CLAHE 增强(LAB L通道)
///   → 曝光补偿(V通道直方图拉伸) → Resize(640×384) → BGR→RGB
///   → [可选归一化 mean/std] → ROS Image → publish
///
/// 关键设计决策:
///   - CLAHE 应用于 LAB-L 通道 (避免 RGB 三通道独立均衡导致偏色)
///   - 曝光补偿用 V 通道直方图 clip 两端 0.5% → min-max 拉伸
///   - 前后相机回调独立, 共享 diag_mutex_ 保护诊断发布
///   - cv_bridge 只在回调入口/出口各调用一次 (零拷贝 cv_bridge::toCvShare)
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/image_preprocessor.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <string>
#include <algorithm>
#include <stdexcept>

namespace br_perception {
namespace camera {

// ═══════════════════════════════════════════════════════════════════════════════
// 构造 & 参数初始化
// ═══════════════════════════════════════════════════════════════════════════════

ImagePreprocessor::ImagePreprocessor(const rclcpp::NodeOptions& options)
  : Node("image_preprocessor", options)
{
  // ── 全局模型参数 ──
  this->declare_parameter<int>("model_input_w", model_input_w_);
  this->declare_parameter<int>("model_input_h", model_input_h_);
  this->declare_parameter<bool>("enable_normalize", enable_normalize_);
  this->declare_parameter<std::vector<double>>("norm_mean", norm_mean_);
  this->declare_parameter<std::vector<double>>("norm_std", norm_std_);
  // ── 曝光补偿开关 (2026-09-20 新增) ──
  //   默认 false: Ultralytics 训练时不做这一步, 推理侧开着会造成分布错配。
  //   要开的话, **训练数据也必须过同一套 compensate_exposure**。
  this->declare_parameter<bool>("exposure_compensation_enabled",
                                exposure_compensation_enabled_);

  // ── 前向相机参数 ──
  this->declare_parameter<bool>("wide_clahe_enabled", wide_clahe_enabled_);
  this->declare_parameter<double>("wide_clahe_clip", wide_clahe_clip_);
  this->declare_parameter<int>("wide_clahe_tile", wide_clahe_tile_);
  this->declare_parameter<std::vector<int64_t>>("wide_roi", wide_roi_);

  // ── 后向相机参数 ──
  this->declare_parameter<bool>("tele_clahe_enabled", tele_clahe_enabled_);
  this->declare_parameter<double>("tele_clahe_clip", tele_clahe_clip_);
  this->declare_parameter<int>("tele_clahe_tile", tele_clahe_tile_);
  this->declare_parameter<std::vector<int64_t>>("tele_roi", tele_roi_);

  // ── 读取所有参数 ──
  this->get_parameter("model_input_w", model_input_w_);
  this->get_parameter("model_input_h", model_input_h_);
  this->get_parameter("enable_normalize", enable_normalize_);
  this->get_parameter("norm_mean", norm_mean_);
  this->get_parameter("norm_std", norm_std_);
  this->get_parameter("exposure_compensation_enabled", exposure_compensation_enabled_);

  this->get_parameter("wide_clahe_enabled", wide_clahe_enabled_);
  this->get_parameter("wide_clahe_clip", wide_clahe_clip_);
  this->get_parameter("wide_clahe_tile", wide_clahe_tile_);
  this->get_parameter("wide_roi", wide_roi_);

  this->get_parameter("tele_clahe_enabled", tele_clahe_enabled_);
  this->get_parameter("tele_clahe_clip", tele_clahe_clip_);
  this->get_parameter("tele_clahe_tile", tele_clahe_tile_);
  this->get_parameter("tele_roi", tele_roi_);

  // ── 规范化参数 ──
  model_input_w_ = std::max(16, std::min(model_input_w_, 4096));
  model_input_h_ = std::max(16, std::min(model_input_h_, 4096));
  wide_clahe_tile_ = std::max(2, std::min(wide_clahe_tile_, 64));
  tele_clahe_tile_  = std::max(2, std::min(tele_clahe_tile_,  64));
  wide_clahe_clip_ = std::max(0.0, std::min(wide_clahe_clip_, 40.0));
  tele_clahe_clip_  = std::max(0.0, std::min(tele_clahe_clip_,  40.0));

  if (norm_mean_.size() != 3) norm_mean_ = {0.485, 0.456, 0.406};
  if (norm_std_.size()  != 3) norm_std_  = {0.229, 0.224, 0.225};

  // ⚠️ 归一化是一颗雷（2026-09-21 VM 实测踩过）:
  //   打开后本节点输出 **32FC3 (float32)**，而订阅这条话题的 4 个传统 CV 检测器
  //   做的是 `toCvShare(msg->image, BGR8)` —— **float32 转不了, 直接抛异常**，
  //   而且不报错日志，表现是"整条相机链路一帧都收不到"。
  //   归一化是**模型专属**的事，应由 ONNX 检测器自己 convertTo(1/255)。
  if (enable_normalize_) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ enable_normalize=true → 输出编码变 32FC3(float32)。"
      "订阅本话题的传统 CV 检测器要 uint8 BGR8, 它们会 cv_bridge 抛异常、"
      "一帧都收不到。确认**所有订阅者**都能吃 float32 再开。");
  }

  // ── 参数动态更新 ──
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ImagePreprocessor::on_parameter_change, this, std::placeholders::_1));

  // ── 订阅原始图像 ──
  // ⚠ QoS 必须 BEST_EFFORT: 相机驱动 (usb_cam / realsense2_camera / Gazebo 相机插件)
  //   一律按 sensor 数据发 BEST_EFFORT。RELIABLE 订阅者 ↔ BEST_EFFORT 发布者
  //   = QoS 不兼容, 一帧都收不到, 且不报错 (静默无数据)。
  //
  // ⚠️⚠️ **队列深度 = 1**（任务书 §3.0 三相机并行的**硬约束第 2 条**，
  //    2026-09-22 落地）: "晚了就丢, 不堆积 —— 否则一次卡顿会让延迟**永久滚大**"。
  //    原先是 `SensorDataQoS()`，它的 depth 是 **5**。差别是实打实的:
  //      · depth=5 → 回调慢下来时队列攒 5 帧, 之后按序消费,
  //        回调**永远落后约 5 帧的处理时间**, 而且**卡顿过去也不会自己恢复**;
  //      · depth=1 → 只留最新, 旧帧直接丢, 延迟不累积。
  //    对"实时看世界"的视觉链, 迟到的帧不只是没用, 是**有害**的。
  //
  // ⚠️ 本链**六条话题**（原始 ×2 / 预处理 ×2 / 检测结果 ×2 ×3 个检测器）**两侧都要 depth=1**:
  //    发布端 depth 大照样会在发布侧攒帧, 只改订阅端等于没改。
  //    全仓库的落点见 `grep -n "QoS(1).best_effort" src/camera/*.cpp`。
  //    ⚠️ `camera_depth` 那一路**本来就是** depth=1（它是独立后加的），别改回去。
  const auto stream_qos = rclcpp::QoS(1).best_effort();

  sub_wide_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_wide/image_raw", stream_qos,
    std::bind(&ImagePreprocessor::wide_callback, this, std::placeholders::_1));

  sub_tele_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera_tele/image_raw", stream_qos,
    std::bind(&ImagePreprocessor::tele_callback, this, std::placeholders::_1));

  // ── 发布预处理图像 (与下游检测器保持同为 depth=1 + BEST_EFFORT, 两侧必须一起改) ──
  // ⚠️ 2026-09-20: 类型改为 PreprocessedImage —— 它把 letterbox 变换 (scale/pad/orig)
  //    一起带走, 检测器才能把 bbox 从「letterbox 坐标」反算回「原图坐标」。
  //    不反算的话, 所有 2D→3D 投影会整体偏移 (近处看不出、远处错得离谱)。
  pub_wide_ = this->create_publisher<br_perception::msg::PreprocessedImage>(
    "/perception/camera_wide/preprocessed", stream_qos);

  pub_tele_ = this->create_publisher<br_perception::msg::PreprocessedImage>(
    "/perception/camera_tele/preprocessed", stream_qos);

  // ── 诊断 ──
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/camera/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "ImagePreprocessor ready | model=%dx%d | "
    "wide(clahe=%s clip=%.1f tile=%d roi=%ld×%ld+%ld+%ld) | "
    "tele(clahe=%s clip=%.1f tile=%d roi=%ld×%ld+%ld+%ld) | "
    "normalize=%s (mean=[%.3f,%.3f,%.3f] std=[%.3f,%.3f,%.3f])",
    model_input_w_, model_input_h_,
    wide_clahe_enabled_ ? "on" : "off", wide_clahe_clip_, wide_clahe_tile_,
    wide_roi_.size() >= 4 ? wide_roi_[2] : 0, wide_roi_.size() >= 4 ? wide_roi_[3] : 0,
    wide_roi_.size() >= 2 ? wide_roi_[0] : 0, wide_roi_.size() >= 2 ? wide_roi_[1] : 0,
    tele_clahe_enabled_  ? "on" : "off", tele_clahe_clip_,  tele_clahe_tile_,
    tele_roi_.size()  >= 4 ? tele_roi_[2]  : 0, tele_roi_.size()  >= 4 ? tele_roi_[3]  : 0,
    tele_roi_.size()  >= 2 ? tele_roi_[0]  : 0, tele_roi_.size()  >= 2 ? tele_roi_[1]  : 0,
    enable_normalize_ ? "on" : "off",
    norm_mean_[0], norm_mean_[1], norm_mean_[2],
    norm_std_[0],  norm_std_[1],  norm_std_[2]);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 前向相机回调
// ═══════════════════════════════════════════════════════════════════════════════
void ImagePreprocessor::wide_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  process_and_publish(msg, "wide",
                      wide_clahe_enabled_, wide_clahe_clip_, wide_clahe_tile_,
                      wide_roi_, pub_wide_);
  wide_frame_seq_++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 后向相机回调
// ═══════════════════════════════════════════════════════════════════════════════
void ImagePreprocessor::tele_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  process_and_publish(msg, "tele",
                      tele_clahe_enabled_, tele_clahe_clip_, tele_clahe_tile_,
                      tele_roi_, pub_tele_);
  tele_frame_seq_++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 单路相机通用处理
// ═══════════════════════════════════════════════════════════════════════════════
void ImagePreprocessor::process_and_publish(
    const sensor_msgs::msg::Image::SharedPtr msg,
    const std::string& cam_name,
    bool clahe_enabled, double clahe_clip, int clahe_tile,
    const std::vector<int64_t>& roi,
    const rclcpp::Publisher<br_perception::msg::PreprocessedImage>::SharedPtr& pub)
{
  // ── 计时起点 ──
  auto t0 = std::chrono::steady_clock::now();

  // ── 1. ROS Image → cv::Mat (BGR8) ──
  cv::Mat bgr;
  try {
    // 使用 toCvShare 零拷贝 (shared_ptr 保证数据生命周期)
    auto cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    bgr = cv_ptr->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] cv_bridge error: %s", cam_name.c_str(), e.what());
    return;
  }

  if (bgr.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] Empty image received", cam_name.c_str());
    return;
  }

  const int orig_w = bgr.cols;
  const int orig_h = bgr.rows;

  // ── 2. 预处理管线 ──
  //    前后相机各存一份 letterbox 变换 (回调可并发, 各写各的成员) ——
  //    **下游检测器反算检测框坐标时必须用它**, 否则 2D→3D 投影全偏。
  LetterboxTransform* tf = (cam_name == "wide") ? &wide_letterbox_ : &tele_letterbox_;
  std::string out_encoding;
  cv::Mat processed = preprocess(bgr, roi,
                                 clahe_enabled, clahe_clip, clahe_tile,
                                 model_input_w_, model_input_h_,
                                 enable_normalize_, norm_mean_, norm_std_,
                                 out_encoding, tf);

  if (processed.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[%s] Preprocessing returned empty image", cam_name.c_str());
    return;
  }

  // ── 3. cv::Mat → PreprocessedImage (图 + letterbox 变换) + 发布 ──
  //    ⚠️ 变换必须随图一起发: 检测器拿到 640×384 的 letterbox 图之后,
  //       **无法从图像本身推出** orig_w/orig_h 和填充量。
  br_perception::msg::PreprocessedImage out_msg;
  out_msg.header = msg->header;                       // ⚠ 继承曝光时刻, 禁止用 now()
  out_msg.image  = *cv_bridge::CvImage(msg->header, out_encoding, processed).toImageMsg();
  out_msg.scale  = static_cast<float>(tf->scale);
  out_msg.pad_x  = tf->pad_x;
  out_msg.pad_y  = tf->pad_y;
  out_msg.orig_w = tf->orig_w;                        // letterbox 之前 (ROI 之后) 的尺寸
  out_msg.orig_h = tf->orig_h;
  pub->publish(out_msg);

  // ── 4. 耗时统计 ──
  auto t1 = std::chrono::steady_clock::now();
  double total_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

  // ── 5. 诊断 ──
  publish_diagnostics(cam_name, total_ms,
                      (cam_name == "wide") ? wide_frame_seq_ : tele_frame_seq_,
                      orig_w, orig_h,
                      processed.cols, processed.rows,
                      out_encoding);

  // ── 延迟超标告警 ──
  if (total_ms > 20.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[%s] Preprocessing slow: %.1f ms (%dx%d → %dx%d)",
      cam_name.c_str(), total_ms, orig_w, orig_h,
      processed.cols, processed.rows);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 完整预处理管线
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::preprocess(
    const cv::Mat& bgr,
    const std::vector<int64_t>& roi,
    bool clahe_enabled, double clahe_clip, int clahe_tile,
    int target_w, int target_h,
    bool normalize_enabled,
    const std::vector<double>& norm_mean,
    const std::vector<double>& norm_std,
    std::string& out_encoding,
    LetterboxTransform* tf_out)
{
  // ── Step 1: ROI 裁剪 ──
  cv::Mat working = crop_roi(bgr, roi);
  if (working.empty()) return {};

  // ── Step 2: CLAHE 增强 (LAB L 通道) ──
  if (clahe_enabled) {
    working = apply_clahe(working, clahe_clip, clahe_tile);
  }

  // ── Step 3: 曝光补偿 (可选; 默认关, 见 exposure_compensation_enabled_) ──
  if (exposure_compensation_enabled_) {
    working = compensate_exposure(working);
  }

  // ── Step 4: **letterbox** 到模型输入尺寸 ──
  //    ⚠️ 2026-09-20 改: 原为 resize_to_model 的**硬拉伸**, 会把 16:9 的相机画面
  //    在 5:3 的输入上纵向拉伸 6.7%, 且与 Ultralytics 训练时的 letterbox 不一致。
  working = letterbox_to_model(working, target_w, target_h, tf_out);

  // ── Step 5: BGR → RGB ──
  working = convert_bgr_to_rgb(working);

  // ── Step 6: [可选] 归一化 ──
  if (normalize_enabled && norm_mean.size() >= 3 && norm_std.size() >= 3) {
    working = apply_normalization(working, norm_mean, norm_std);
    out_encoding = sensor_msgs::image_encodings::TYPE_32FC3;
  } else {
    // 确保输出为连续内存 (cv_bridge 发布要求)
    if (!working.isContinuous()) {
      working = working.clone();
    }
    out_encoding = sensor_msgs::image_encodings::RGB8;
  }

  return working;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 1: ROI 裁剪
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::crop_roi(const cv::Mat& image,
                                    const std::vector<int64_t>& roi)
{
  // ROI 无效或为全图 → 不裁剪
  if (roi.size() < 4) {
    return image;
  }

  const int x      = static_cast<int>(roi[0]);
  const int y      = static_cast<int>(roi[1]);
  const int w_raw  = static_cast<int>(roi[2]);
  const int h_raw  = static_cast<int>(roi[3]);

  // 全零 ROI 或覆盖全图 → 不裁剪
  if (x == 0 && y == 0 && w_raw >= image.cols && h_raw >= image.rows) {
    return image;
  }

  // 修正边界
  const int img_w = image.cols;
  const int img_h = image.rows;

  int cx = std::max(0, x);
  int cy = std::max(0, y);
  int cw = std::min(w_raw, img_w - cx);
  int ch = std::min(h_raw, img_h - cy);

  if (cw <= 0 || ch <= 0) {
    // ROI 完全在图像外, 返回原图
    return image;
  }

  cv::Rect rect(cx, cy, cw, ch);
  return image(rect).clone();  // clone necessary: ROI Mat data not contiguous
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 2: CLAHE 增强 — LAB 色彩空间 L 通道
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::apply_clahe(const cv::Mat& bgr,
                                       double clip_limit, int tile_size)
{
  // BGR → LAB
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

  // 分离通道: L, a, b
  std::vector<cv::Mat> channels(3);
  cv::split(lab, channels);

  // 仅对 L 通道做 CLAHE (避免 a/b 通道产生颜色失真)
  auto clahe = cv::createCLAHE(clip_limit,
                                cv::Size(tile_size, tile_size));
  clahe->apply(channels[0], channels[0]);

  // 合并通道
  cv::merge(channels, lab);

  // LAB → BGR
  cv::Mat result;
  cv::cvtColor(lab, result, cv::COLOR_Lab2BGR);
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 3: 曝光补偿 — V 通道直方图拉伸
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::compensate_exposure(const cv::Mat& bgr)
{
  // BGR → HSV
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  std::vector<cv::Mat> channels(3);
  cv::split(hsv, channels);
  cv::Mat& v_channel = channels[2];  // V = value / brightness

  // ── 计算 V 通道直方图 ──
  int hist_size = 256;
  float range[] = {0, 256};
  const float* hist_range = {range};
  cv::Mat hist;
  cv::calcHist(&v_channel, 1, nullptr, cv::Mat(), hist, 1, &hist_size, &hist_range);

  // ── Clip 两端各 0.5% ──
  const double total = v_channel.total();
  const double clip_count = total * 0.005;  // 0.5%

  // 从低端累加
  int v_min = 0;
  {
    double accum = 0;
    for (int i = 0; i < 256; ++i) {
      accum += hist.at<float>(i);
      if (accum > clip_count) { v_min = i; break; }
    }
  }

  // 从高端累加
  int v_max = 255;
  {
    double accum = 0;
    for (int i = 255; i >= 0; --i) {
      accum += hist.at<float>(i);
      if (accum > clip_count) { v_max = i; break; }
    }
  }

  // ── Min-max 拉伸 ──
  if (v_max > v_min) {
    const double scale = 255.0 / static_cast<double>(v_max - v_min);
    for (int y = 0; y < v_channel.rows; ++y) {
      uint8_t* row = v_channel.ptr<uint8_t>(y);
      for (int x = 0; x < v_channel.cols; ++x) {
        int val = row[x];
        val = cv::saturate_cast<uint8_t>((val - v_min) * scale);
        row[x] = static_cast<uint8_t>(val);
      }
    }
  }

  // ── 合并回 BGR ──
  cv::merge(channels, hsv);
  cv::Mat result;
  cv::cvtColor(hsv, result, cv::COLOR_HSV2BGR);
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 4: letterbox 到模型输入尺寸  (2026-09-20: 取代原"硬拉伸"版 resize_to_model)
//
//   等比缩放到能完整放进 target, 四周居中补 kLetterboxPadValue(114) 灰边。
//   这与 Ultralytics 训练时的预处理一致 —— 原实现(直接 cv::resize 拉伸)
//   会让 1280×720 → 640×384 纵向拉伸 6.7%, 训练/推理分布不一致。
//
//   ⚠️ 填充值用 **114 灰** 而不是 0 黑: Ultralytics 的 letterbox 就是 114。
//      用 0 会在边框处造出一条纯黑带, 网络没见过。
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::letterbox_to_model(const cv::Mat& image,
                                              int target_w, int target_h,
                                              LetterboxTransform* tf_out)
{
  const LetterboxTransform tf = compute_letterbox(image.cols, image.rows,
                                                   target_w, target_h);
  if (tf_out != nullptr) {
    *tf_out = tf;
  }

  // 尺寸已完全匹配 → 不缩放不填充
  if (image.cols == target_w && image.rows == target_h) {
    return image;
  }

  // ── 等比缩放 ──
  cv::Mat scaled;
  // INTER_LINEAR: 速度与质量的平衡; 如需更高精度可换 INTER_CUBIC
  cv::resize(image, scaled, cv::Size(tf.content_w(), tf.content_h()),
             0, 0, cv::INTER_LINEAR);

  // ── 居中填充 ──
  cv::Mat out(target_h, target_w, scaled.type(),
              cv::Scalar::all(kLetterboxPadValue));
  const int x0 = std::max(0, std::min(tf.pad_x, target_w - scaled.cols));
  const int y0 = std::max(0, std::min(tf.pad_y, target_h - scaled.rows));
  scaled.copyTo(out(cv::Rect(x0, y0, scaled.cols, scaled.rows)));
  return out;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 5: BGR → RGB
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::convert_bgr_to_rgb(const cv::Mat& bgr)
{
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  return rgb;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Step 6: 归一化 — (pixel/255.0 - mean) / std → float32
// ═══════════════════════════════════════════════════════════════════════════════
cv::Mat ImagePreprocessor::apply_normalization(const cv::Mat& rgb,
                                               const std::vector<double>& mean,
                                               const std::vector<double>& stddev)
{
  CV_Assert(rgb.type() == CV_8UC3);
  CV_Assert(mean.size() >= 3 && stddev.size() >= 3);

  // 转为 float32 [0, 1]
  cv::Mat rgb_float;
  rgb.convertTo(rgb_float, CV_32FC3, 1.0 / 255.0);

  // (x - mean) / std → 逐通道
  // OpenCV 的 subtract + divide 对多通道自动按 Scalar 广播
  cv::Mat normalized;
  cv::subtract(rgb_float,
               cv::Scalar(static_cast<float>(mean[0]),
                          static_cast<float>(mean[1]),
                          static_cast<float>(mean[2])),
               normalized);
  cv::divide(normalized,
             cv::Scalar(static_cast<float>(stddev[0]),
                        static_cast<float>(stddev[1]),
                        static_cast<float>(stddev[2])),
             normalized);

  return normalized;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断发布
// ═══════════════════════════════════════════════════════════════════════════════
void ImagePreprocessor::publish_diagnostics(
    const std::string& cam_name,
    double preprocess_ms,
    size_t frame_seq,
    int orig_w, int orig_h,
    int out_w, int out_h,
    const std::string& out_encoding)
{
  std::lock_guard<std::mutex> lock(diag_mutex_);

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "image_preprocessor_" + cam_name;
  status.hardware_id = "camera_" + cam_name;

  // 总体健康判定
  if (preprocess_ms < 15.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  } else if (preprocess_ms < 25.0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "SLOW";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "VERY SLOW";
  }

  // Key-value 对
  auto add_kv = [&](const std::string& key, double val) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = std::to_string(val);
    status.values.push_back(kv);
  };

  add_kv("preprocess_ms", preprocess_ms);
  add_kv("frame_seq",      static_cast<double>(frame_seq));
  add_kv("input_width",    static_cast<double>(orig_w));
  add_kv("input_height",   static_cast<double>(orig_h));
  add_kv("output_width",   static_cast<double>(out_w));
  add_kv("output_height",  static_cast<double>(out_h));

  diag_array.status.push_back(status);
  pub_diagnostics_->publish(diag_array);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult ImagePreprocessor::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto& param : params) {
    const std::string& name = param.get_name();

    try {
      // ── 全局模型参数 ──
      if (name == "model_input_w") {
        int val = param.as_int();
        if (val < 16 || val > 4096) {
          result.successful = false;
          result.reason = "model_input_w must be in [16, 4096]";
          break;
        }
        model_input_w_ = val;
      }
      else if (name == "model_input_h") {
        int val = param.as_int();
        if (val < 16 || val > 4096) {
          result.successful = false;
          result.reason = "model_input_h must be in [16, 4096]";
          break;
        }
        model_input_h_ = val;
      }
      else if (name == "enable_normalize") {
        enable_normalize_ = param.as_bool();
      }
      else if (name == "norm_mean") {
        auto val = param.as_double_array();
        if (val.size() != 3) {
          result.successful = false;
          result.reason = "norm_mean must have exactly 3 elements";
          break;
        }
        norm_mean_ = val;
      }
      else if (name == "norm_std") {
        auto val = param.as_double_array();
        if (val.size() != 3) {
          result.successful = false;
          result.reason = "norm_std must have exactly 3 elements";
          break;
        }
        // std 不能有零值, 否则除零
        for (size_t i = 0; i < val.size(); ++i) {
          if (std::abs(val[i]) < 1e-9) {
            result.successful = false;
            result.reason = "norm_std elements must be non-zero";
            break;
          }
        }
        if (!result.successful) break;
        norm_std_ = val;
      }

      // ── 前向相机参数 ──
      else if (name == "wide_clahe_enabled") {
        wide_clahe_enabled_ = param.as_bool();
      }
      else if (name == "wide_clahe_clip") {
        double val = param.as_double();
        if (val < 0.0 || val > 40.0) {
          result.successful = false;
          result.reason = "wide_clahe_clip must be in [0, 40]";
          break;
        }
        wide_clahe_clip_ = val;
      }
      else if (name == "wide_clahe_tile") {
        int val = param.as_int();
        if (val < 2 || val > 64) {
          result.successful = false;
          result.reason = "wide_clahe_tile must be in [2, 64]";
          break;
        }
        wide_clahe_tile_ = val;
      }
      else if (name == "wide_roi") {
        auto val = param.as_integer_array();
        if (val.size() != 4) {
          result.successful = false;
          result.reason = "wide_roi must have exactly 4 elements [x, y, w, h]";
          break;
        }
        wide_roi_ = val;
      }

      // ── 后向相机参数 ──
      else if (name == "tele_clahe_enabled") {
        tele_clahe_enabled_ = param.as_bool();
      }
      else if (name == "tele_clahe_clip") {
        double val = param.as_double();
        if (val < 0.0 || val > 40.0) {
          result.successful = false;
          result.reason = "tele_clahe_clip must be in [0, 40]";
          break;
        }
        tele_clahe_clip_ = val;
      }
      else if (name == "tele_clahe_tile") {
        int val = param.as_int();
        if (val < 2 || val > 64) {
          result.successful = false;
          result.reason = "tele_clahe_tile must be in [2, 64]";
          break;
        }
        tele_clahe_tile_ = val;
      }
      else if (name == "tele_roi") {
        auto val = param.as_integer_array();
        if (val.size() != 4) {
          result.successful = false;
          result.reason = "tele_roi must have exactly 4 elements [x, y, w, h]";
          break;
        }
        tele_roi_ = val;
      }

      else {
        // 未知参数, 忽略 (可能属于其他节点)
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
      "Params updated | model=%dx%d | wide_clahe=%s(%.1f,%d) tele_clahe=%s(%.1f,%d) | normalize=%s",
      model_input_w_, model_input_h_,
      wide_clahe_enabled_ ? "on" : "off", wide_clahe_clip_, wide_clahe_tile_,
      tele_clahe_enabled_  ? "on" : "off", tele_clahe_clip_,  tele_clahe_tile_,
      enable_normalize_    ? "on" : "off");
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

}  // namespace camera
}  // namespace br_perception

// ── ROS2 组件注册 ──
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::ImagePreprocessor)
