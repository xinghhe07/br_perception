/// ═══════════════════════════════════════════════════════════════════════════════
/// camera_depth_node.cpp — D435i 深度辅助节点实现（任务书 §3.0）
///
/// 单一职责：给一个目标 ROI → 出该处**鲁棒深度**。
///
/// 核心是 nearest_cluster()：**不用最小深度**（会被飞点/反光劫持），
/// 改成"最近的显著深度簇"的中值 + 质心。见头文件说明。
///
/// ⚠️ 三条不能违背的约束：QoS depth=1 · 独立回调不阻塞主链 · 时间戳继承曝光时刻
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/camera_depth_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace br_perception {
namespace camera {

CameraDepthNode::CameraDepthNode(const rclcpp::NodeOptions& options)
  : Node("camera_depth", options)
{
  // ── 参数 ──
  this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_rect_raw");
  this->declare_parameter<std::string>("roi_topic",   "/camera/depth/target_roi");
  this->declare_parameter<double>("depth_scale_m",    depth_scale_m_);
  this->declare_parameter<double>("valid_ratio_min",  valid_ratio_min_);
  this->declare_parameter<double>("depth_min_m",      depth_min_m_);
  this->declare_parameter<double>("depth_max_m",      depth_max_m_);
  this->declare_parameter<double>("hist_bin_m",       hist_bin_m_);
  this->declare_parameter<int>   ("min_cluster_px",   min_cluster_px_);
  this->declare_parameter<double>("roi_timeout_sec",  roi_timeout_sec_);
  this->declare_parameter<double>("diag_rate_hz",     diag_rate_hz_);
  this->declare_parameter<double>("stream_timeout_sec", stream_timeout_sec_);

  this->get_parameter("depth_topic",   depth_topic_);
  this->get_parameter("roi_topic",     roi_topic_);
  this->get_parameter("depth_scale_m", depth_scale_m_);
  this->get_parameter("valid_ratio_min", valid_ratio_min_);
  this->get_parameter("depth_min_m",   depth_min_m_);
  this->get_parameter("depth_max_m",   depth_max_m_);
  this->get_parameter("hist_bin_m",    hist_bin_m_);
  this->get_parameter("min_cluster_px", min_cluster_px_);
  this->get_parameter("roi_timeout_sec", roi_timeout_sec_);
  this->get_parameter("diag_rate_hz",  diag_rate_hz_);
  this->get_parameter("stream_timeout_sec", stream_timeout_sec_);

  // ── 规范化 ──
  depth_scale_m_   = std::max(1e-6, depth_scale_m_);
  valid_ratio_min_ = std::max(0.0, std::min(valid_ratio_min_, 1.0));
  depth_min_m_     = std::max(0.0, depth_min_m_);
  depth_max_m_     = std::max(depth_min_m_ + 0.05, depth_max_m_);
  hist_bin_m_      = std::max(0.005, std::min(hist_bin_m_, 1.0));
  min_cluster_px_  = std::max(1, min_cluster_px_);
  roi_timeout_sec_ = std::max(0.05, roi_timeout_sec_);
  diag_rate_hz_    = std::max(0.1, std::min(diag_rate_hz_, 10.0));
  // ⚠️ 下限不能小于一个诊断周期 —— 否则兜底心跳会和正常心跳互相插队抢发
  stream_timeout_sec_ = std::max(1.0 / diag_rate_hz_, stream_timeout_sec_);

  // ── 订阅深度图（QoS depth=1，晚了就丢）──
  sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, rclcpp::QoS(1).best_effort(),
    std::bind(&CameraDepthNode::depth_callback, this, std::placeholders::_1));

  // ── 订阅目标 ROI（上层给；这是进 MEASURE 的唯一入口）──
  sub_roi_ = this->create_subscription<sensor_msgs::msg::RegionOfInterest>(
    roi_topic_, rclcpp::QoS(1).best_effort(),
    std::bind(&CameraDepthNode::roi_callback, this, std::placeholders::_1));

  // ── 发布 ──
  pub_depth_ = this->create_publisher<br_perception::msg::CameraDepth>(
    "/perception/camera/depth", rclcpp::QoS(1).best_effort());
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/camera/depth_diagnostics", 10);

  last_diag_time_  = this->now();
  // ⚠️ 从**节点启动时刻**起算, 不是从 0 起算 —— 否则刚起来那一瞬间
  //    (now - 0) 是个天文数字, 兜底心跳会在第一帧到达前就误报"流断了"。
  last_frame_time_ = this->now();

  // ── 兜底心跳（唯一在没有深度帧时还活着的东西）──
  diag_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / diag_rate_hz_),
    std::bind(&CameraDepthNode::idle_diag_tick, this));

  RCLCPP_INFO(this->get_logger(),
    "CameraDepthNode ready | depth=%s roi=%s | scale=%.6f | range=[%.2f,%.2f]m | "
    "bin=%.3fm minCluster=%dpx | roiTimeout=%.2fs | diag=%.1fHz | streamTimeout=%.2fs",
    depth_topic_.c_str(), roi_topic_.c_str(), depth_scale_m_,
    depth_min_m_, depth_max_m_, hist_bin_m_, min_cluster_px_,
    roi_timeout_sec_, diag_rate_hz_, stream_timeout_sec_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 单像素取深度（米）
// ═══════════════════════════════════════════════════════════════════════════════
bool CameraDepthNode::depth_at(const cv::Mat& depth, int u, int v,
                               double depth_scale_m, double& meters)
{
  if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows) return false;

  if (depth.type() == CV_16UC1) {
    const std::uint16_t raw = depth.at<std::uint16_t>(v, u);
    if (raw == 0) return false;                        // 0 = 无测量（RealSense 惯用）
    meters = static_cast<double>(raw) * depth_scale_m;
    return true;
  }
  if (depth.type() == CV_32FC1) {
    const float f = depth.at<float>(v, u);
    if (!std::isfinite(f) || f <= 0.0f) return false;   // NaN / Inf = 无测量
    meters = static_cast<double>(f);                    // 32FC1 已经是米
    return true;
  }
  return false;                                         // 其它类型不支持
}

// ═══════════════════════════════════════════════════════════════════════════════
// 最近的显著深度簇
// ═══════════════════════════════════════════════════════════════════════════════
CameraDepthNode::ClusterStats
CameraDepthNode::nearest_cluster(const cv::Mat& depth,
                                 int x0, int y0, int x1, int y1) const
{
  ClusterStats st;

  // ── Pass 1: 深度直方图（**只统计个数，不存点** —— ROI 大时省内存）──
  const double bin_w = hist_bin_m_;
  const int nbins = static_cast<int>((depth_max_m_ - depth_min_m_) / bin_w) + 1;
  if (nbins <= 0) return st;
  std::vector<int> hist(static_cast<std::size_t>(nbins), 0);

  int total = 0, valid_cnt = 0;
  for (int v = y0; v < y1; ++v) {
    for (int u = x0; u < x1; ++u) {
      ++total;
      double d = 0.0;
      if (!depth_at(depth, u, v, depth_scale_m_, d)) continue;
      if (d < depth_min_m_ || d > depth_max_m_) continue;
      const int b = static_cast<int>((d - depth_min_m_) / bin_w);
      if (b < 0 || b >= nbins) continue;
      ++hist[static_cast<std::size_t>(b)];
      ++valid_cnt;
    }
  }
  st.total_px = total;
  st.valid_px = valid_cnt;
  if (total == 0 || valid_cnt == 0) return st;

  // ── 找**从近到远**第一个像素数达标的 bin ──
  //   ⚠️ 这一步就是"不用最小深度"的落点：单个飞点凑不满 min_cluster_px_，
  //      会被跳过；只有真正成片的表面才算"一个目标"。
  int first_bin = -1;
  for (int b = 0; b < nbins; ++b) {
    if (hist[static_cast<std::size_t>(b)] >= min_cluster_px_) { first_bin = b; break; }
  }
  if (first_bin < 0) return st;                       // 没有成簇的 → 无目标

  // ── Pass 2: 收集该簇带（bin ± 1，容忍表面起伏）内像素 ──
  const double lo = depth_min_m_ + static_cast<double>(first_bin - 1) * bin_w;
  const double hi = depth_min_m_ + static_cast<double>(first_bin + 2) * bin_w;

  std::vector<double> ds;
  double su = 0.0, sv = 0.0;
  for (int v = y0; v < y1; ++v) {
    for (int u = x0; u < x1; ++u) {
      double d = 0.0;
      if (!depth_at(depth, u, v, depth_scale_m_, d)) continue;
      if (d < lo || d >= hi) continue;
      ds.push_back(d);
      su += u;
      sv += v;
    }
  }
  if (ds.empty()) return st;

  // ── 统计（排序取分位；n 通常几百~几万，代价可接受）──
  std::sort(ds.begin(), ds.end());
  const std::size_t n = ds.size();
  auto q = [&ds, n](double f) {
    const std::size_t i = std::min(n - 1, static_cast<std::size_t>(f * static_cast<double>(n)));
    return ds[i];
  };

  st.found        = true;
  st.p25          = q(0.25);
  st.depth_median = q(0.50);
  st.p75          = q(0.75);
  st.count        = static_cast<int>(n);

  double s2 = 0.0;
  for (double d : ds) { const double e = d - st.depth_median; s2 += e * e; }
  st.stddev = std::sqrt(s2 / static_cast<double>(n));

  // **质心**而不是 ROI 中心 —— 目标未必填满 ROI，中心可能落在背景上
  st.u = su / static_cast<double>(n);
  st.v = sv / static_cast<double>(n);
  return st;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ROI 回调 —— 唯一能进 MEASURE 的入口
// ═══════════════════════════════════════════════════════════════════════════════
void CameraDepthNode::roi_callback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(roi_mutex_);
  roi_       = *msg;
  roi_stamp_ = this->now();
  has_roi_   = (msg->width > 0 && msg->height > 0);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 深度回调
// ═══════════════════════════════════════════════════════════════════════════════
void CameraDepthNode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  const auto t0 = std::chrono::steady_clock::now();
  ++frame_seq_;

  // 年龄仪器（2026-09-26 加）—— 语义与键名**与雷达/工业相机那两路逐字对齐**。
  // ⚠️ 用 `msg->header.stamp`（RealSense 给的曝光/采集时刻），不是到达时刻。
  last_frame_age_sec_ = (this->now() - rclcpp::Time(msg->header.stamp)).seconds();

  // 帧到了 → 刷新"流还活着"的时间戳（兜底心跳 idle_diag_tick 据此判断）
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    last_frame_time_ = this->now();
  }

  // ── 判断模式：有 ROI 且没过期 → MEASURE，否则 IDLE ──
  sensor_msgs::msg::RegionOfInterest roi;
  bool measuring = false;
  {
    std::lock_guard<std::mutex> lock(roi_mutex_);
    if (has_roi_) {
      const double age = (this->now() - roi_stamp_).seconds();
      if (age <= roi_timeout_sec_) { roi = roi_; measuring = true; }
    }
  }
  mode_ = measuring ? kModeMeasure : kModeIdle;

  // ── IDLE：**不做任何逐像素工作**，只按诊断频率发一条心跳 ──
  //   D435i 是动作阶段辅助传感器；巡航时没必要每帧扫图。
  if (!measuring) {
    const double since = (this->now() - last_diag_time_).seconds();
    if (since >= 1.0 / diag_rate_hz_) {
      const auto t1 = std::chrono::steady_clock::now();
      const double ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
      publish_diagnostics(ms, true, false, 0, 0.0, 0.0);
      last_diag_time_ = this->now();
    }
    return;
  }

  // ── MEASURE：解码 + 只处理这个 ROI ──
  cv::Mat depth;
  try {
    // ⚠️ 深度图编码是 16UC1 / 32FC1，**不是 BGR8** —— 不传编码，保留原样
    depth = cv_bridge::toCvShare(msg)->image;
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "cv_bridge error (encoding=%s): %s", msg->encoding.c_str(), e.what());
    return;
  }
  if (depth.empty()) return;
  if (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Unsupported depth type %d (expect CV_16UC1 / CV_32FC1); encoding=%s",
      depth.type(), msg->encoding.c_str());
    return;
  }

  // ROI 钳到图内
  const int x0 = std::max(0, static_cast<int>(roi.x_offset));
  const int y0 = std::max(0, static_cast<int>(roi.y_offset));
  const int x1 = std::min(depth.cols, x0 + static_cast<int>(roi.width));
  const int y1 = std::min(depth.rows, y0 + static_cast<int>(roi.height));
  if (x1 <= x0 || y1 <= y0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Target ROI empty after clamping to %dx%d", depth.cols, depth.rows);
    return;
  }

  const ClusterStats st = nearest_cluster(depth, x0, y0, x1, y1);

  // valid_ratio 直接用 Pass 1 顺带数出来的计数（**不再扫第三遍**）
  //   分母是 ROI 面积, 与"有没有簇"无关 —— 它反映的是"这一片深度流好不好"。
  const double valid_ratio = (st.total_px > 0)
      ? static_cast<double>(st.valid_px) / static_cast<double>(st.total_px) : 0.0;
  const bool valid = (valid_ratio >= valid_ratio_min_);

  // ── 可信度：三维取小，**每维都 clamp 到 [0,1]** ──
  //   任一维差 → confidence 低, 下游动作控制据此判断"这个深度能不能用"。
  //
  //   ⚠️ clamp 不能省：valid_ratio/valid_ratio_min_ 可以是 1.9（ratio=0.95, min=0.5），
  //      不 clamp 那就不再是概率意义上的 confidence。
  auto clamp01 = [](double v) { return std::max(0.0, std::min(1.0, v)); };

  double conf = 0.0;
  if (valid && st.found) {
    // 深度流健康度
    const double c_valid = clamp01(valid_ratio / std::max(1e-6, valid_ratio_min_));
    // 样本量
    const double c_count = clamp01(double(st.count) / double(min_cluster_px_ * 4));
    // ⚠️ **深度一致性 / 单一目标可信度** —— 不是"平整度"！
    //    std 大可能是：斜面 / **球体曲面** / 多物体混合 / 深度边缘 / 深度噪声 / ROI 含背景。
    //    ⚠️ 别把它当成"表面必须平"的约束 —— 球是曲面, 按平整度读会把真目标判掉。
    const double c_consistency = clamp01(1.0 - st.stddev / 0.05);
    conf = std::min(std::min(c_valid, c_count), c_consistency);
  }

  // ── 发布 ──
  br_perception::msg::CameraDepth out;
  // ⚠️ 时间戳 = 曝光时刻，从输入**继承**；禁止 now()（任务书 §四）
  out.header      = msg->header;
  out.valid       = valid;
  out.valid_ratio = static_cast<float>(valid_ratio);
  out.mode        = mode_;

  out.target_depth_m     = st.found ? static_cast<float>(st.depth_median) : 0.0f;
  out.target_depth_p25_m       = st.found ? static_cast<float>(st.p25)          : 0.0f;
  out.target_depth_p75_m       = st.found ? static_cast<float>(st.p75)          : 0.0f;
  out.target_depth_std_m       = st.found ? static_cast<float>(st.stddev)       : 0.0f;
  out.target_pixel_count = st.found ? static_cast<uint32_t>(st.count)     : 0u;
  out.target_u           = st.found ? static_cast<float>(st.u)            : 0.0f;
  out.target_v           = st.found ? static_cast<float>(st.v)            : 0.0f;
  out.confidence         = static_cast<float>(conf);
  last_confidence_       = conf;   // 供 1Hz 诊断读

  pub_depth_->publish(out);

  // ── 诊断（**限频**，不是每帧）──
  const double since = (this->now() - last_diag_time_).seconds();
  if (since >= 1.0 / diag_rate_hz_) {
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<float, std::milli>(t1 - t0).count();
    publish_diagnostics(ms, valid, st.found, st.count, st.depth_median, st.stddev);
    last_diag_time_ = this->now();
  }

  if (!valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Depth stream unhealthy: valid_ratio=%.2f < %.2f", valid_ratio, valid_ratio_min_);
  } else if (!st.found) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "No significant depth cluster in ROI (min_cluster_px=%d)", min_cluster_px_);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断（限频发布）
// ═══════════════════════════════════════════════════════════════════════════════
void CameraDepthNode::publish_diagnostics(double proc_ms, bool valid, bool found,
                                          int count, double median_m, double stddev_m)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus s;
  s.name = "camera_depth";
  s.hardware_id = "D435i";
  s.level = valid ? diagnostic_msgs::msg::DiagnosticStatus::OK
                  : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  s.message = !valid ? "depth stream unhealthy"
            : (found ? "OK" : "no significant cluster in ROI");

  auto kv = [&s](const char* k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue e; e.key = k; e.value = v; s.values.push_back(e);
  };
  kv("mode", std::to_string(static_cast<int>(mode_)));
  kv("valid", valid ? "1" : "0");
  kv("cluster_found", found ? "1" : "0");
  kv("cluster_px", std::to_string(count));
  kv("median_m", std::to_string(median_m));
  kv("std_m", std::to_string(stddev_m));
  kv("last_confidence", std::to_string(last_confidence_));
  kv("proc_ms", std::to_string(proc_ms));
  kv("frame_seq", std::to_string(frame_seq_));

  arr.status.push_back(s);

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️ 端到端**年龄**（2026-09-26 加）—— **与雷达、工业相机两路同口径同键名**。
  //
  //   上面那条 `camera_depth` 报的是 `proc_ms`（**本节点处理花了多久**）；
  //   这条回答的是**另一个问题**：「这帧深度图到我手上时已经旧了多少」。
  //   延迟预算按后者算，而本节点此前**根本没有这个量** ⇒ 「三路总延迟」
  //   里 D435i 这一路无从计入。
  //
  //   ⚠️ D435i 在**判色（40/80 分）**的关键路径上，不该是盲区。
  //
  //   ⚠️⚠️ 键名 `age_newest_ms` **必须与另外两路逐字一致** ——
  //      名字不一致的"对齐"本身就在制造歧义。
  //   ⚠️ D435i 整帧在同一瞬间曝光 ⇒ 只有"最新=最旧"一个 age，
  //      不像雷达那样要分两个（那是雷达一帧横跨 100ms 才有的问题）。
  //
  //   ⚠️ 阈值 56/80 与 `config/perception_params.yaml` 的
  //      `latency_warn_ms` / `latency_error_ms` 同源，改那边要一起改。
  // ═══════════════════════════════════════════════════════════════════════════
  diagnostic_msgs::msg::DiagnosticStatus lx;
  lx.name = "end_to_end_latency";
  lx.hardware_id = "D435i";

  const double age_ms = last_frame_age_sec_ * 1000.0;
  if (age_ms < 56.0) {
    lx.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    lx.message = "Latency OK";
  } else if (age_ms < 80.0) {
    lx.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    lx.message = "Latency over warn (56ms)";
  } else {
    lx.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    lx.message = "Latency over error (80ms)";
  }
  auto kv2 = [&lx](const char* k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue e; e.key = k; e.value = v; lx.values.push_back(e);
  };
  kv2("age_newest_ms", std::to_string(age_ms));
  // ⚠️ `frame_seq_ == 0`（一帧都没来过）时 age 是 0，会被读成"延迟极好" ——
  //    把序号一并报出来，免得那个 0 被当成好数。真正判"有没有数据"看上面那条的 `valid`。
  kv2("frame_seq", std::to_string(frame_seq_));
  arr.status.push_back(lx);

  std::lock_guard<std::mutex> lock(diag_mutex_);
  pub_diagnostics_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 兜底心跳（有帧时什么都不做）
// ═══════════════════════════════════════════════════════════════════════════════
void CameraDepthNode::idle_diag_tick()
{
  // ⚠️ 为什么需要这条路（2026-09-21 VM 实测发现）:
  //   depth_callback 里那条 IDLE 心跳是**按帧节流**的 —— 没有深度帧就永不触发。
  //   于是 "D435i 掉线" 和 "节点整个死了" 在诊断上**完全一样**: 都是彻底静默。
  //   而区分这两件事, 正是诊断存在的唯一理由。
  //
  //   这里只负责"报丧": 有帧时**直接返回, 绝不插手**,
  //   正常的 IDLE 心跳仍由 depth_callback 发（那边还能顺带报处理耗时）。
  const rclcpp::Time now = this->now();

  double stale = 0.0;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    stale = (now - last_frame_time_).seconds();
  }
  if (stale <= stream_timeout_sec_) return;   // 流是活的 → 不插手

  // 与正常心跳共用 last_diag_time_ 限频 → 两条路不会重复发
  {
    std::lock_guard<std::mutex> lock(diag_mutex_);
    if ((now - last_diag_time_).seconds() < 1.0 / diag_rate_hz_) return;
    last_diag_time_ = now;
  }

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "No depth frame for %.2fs (> stream_timeout_sec=%.2fs) — depth stream lost?",
    stale, stream_timeout_sec_);

  publish_no_stream_diagnostics(stale);
}

// ═══════════════════════════════════════════════════════════════════════════════
// "流断了" 诊断（ERROR 级）
// ═══════════════════════════════════════════════════════════════════════════════
void CameraDepthNode::publish_no_stream_diagnostics(double stale_sec)
{
  // 与 publish_diagnostics 分开写: 那边 valid=false 的语义是
  // "帧到了, 但有效像素不够" —— 和"压根没帧"是两回事, 混在一起会误导现场排障。
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus s;
  s.name        = "camera_depth";
  s.hardware_id = "D435i";
  s.level       = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  s.message     = "no depth stream";

  auto kv = [&s](const char* k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue e; e.key = k; e.value = v; s.values.push_back(e);
  };
  kv("mode",        std::to_string(static_cast<int>(mode_)));
  kv("valid",       "0");
  kv("stream_lost", "1");
  kv("stale_sec",   std::to_string(stale_sec));
  kv("timeout_sec", std::to_string(stream_timeout_sec_));
  kv("frame_seq",   std::to_string(frame_seq_));

  arr.status.push_back(s);

  std::lock_guard<std::mutex> lock(diag_mutex_);
  pub_diagnostics_->publish(arr);
}

}  // namespace camera
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::CameraDepthNode)
