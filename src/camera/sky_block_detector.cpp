/// ═══════════════════════════════════════════════════════════════════════════════
/// sky_block_detector.cpp — 天空方块「顶面颜色」判定（任务书 §3.3）
///
/// **设计理由、三条边界、两个规格未定义段的取舍、以及那条"红蓝怎么分布"的
/// 未决前提，全部写在头文件里。** 本文件只实现，不重复论证。
///
/// ⚠️ 本文件**一行都没编译过**（本机无 ROS、无 OpenCV C++）。
///    ⚠️ 但**判色核心与时序状态机已在 Python 里穷举验过**（见下），
///       而且那轮验证抓出过一个真 bug（时序状态机卡死）。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/sky_block_detector.hpp"

// ⚠️ 这个头**必须显式 include**：漏了的话报错长得像语法错
//    （`expected constructor, destructor, or type conversion at end of input`，
//     指向文件最后一行）—— 2026-09-23 在另外两个节点上撞过，记在这里。
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

// ⚠️ letterbox 反算的**唯一实现**在这里，别在本文件里重写一份。
#include "br_perception/camera/image_preprocessor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace br_perception {
namespace camera {

namespace {

inline std::chrono::steady_clock::time_point mono_now()
{
  return std::chrono::steady_clock::now();
}

inline double ms_since(const std::chrono::steady_clock::time_point& t0)
{
  return std::chrono::duration<double, std::milli>(mono_now() - t0).count();
}

const char* color_name(uint8_t c)
{
  switch (c) {
    case 0: return "NONE";
    case 1: return "RED";
    case 2: return "BLUE";
    case 3: return "UNKNOWN";
    default: return "?";
  }
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// 时序确认状态机
//
// ⚠️⚠️ 已在 Python 里穷举验过（抖动 / 翻转 / 翻回来），并抓出过一个真 bug：
//     退出那条**原先只在 `state == kConfirmed` 时判断** ⇒ 一旦进了 `kCoasting`
//     就**永远出不来**（状态卡死，再也回不到 UNCONFIRMED）。
//     ⇒ 下面把 `kConfirmed` 与 `kCoasting` **放在一起判**，就是修这个。
//     改这段之前先回去跑那三个序列，别凭读代码判断它对不对。
// ═══════════════════════════════════════════════════════════════════════════════
SkyBlockDetector::TemporalConfirmer::State
SkyBlockDetector::TemporalConfirmer::push(bool satisfied)
{
  if (satisfied) {
    in_count_++;
    out_count_ = 0;
    if (state != kConfirmed && in_count_ >= std::max(1, enter_frames)) {
      state = kConfirmed;
    }
  } else {
    out_count_++;
    in_count_ = 0;
    // ⚠️ **两档一起判** —— 只判 kConfirmed 会让状态卡在 kCoasting 出不来（见上）。
    if (state == kConfirmed || state == kCoasting) {
      state = (out_count_ < std::max(1, exit_frames)) ? kCoasting : kUnconfirmed;
    }
  }
  return state;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 判色核心（纯函数，可单测）
// ═══════════════════════════════════════════════════════════════════════════════
SkyBlockDetector::ColorVerdict SkyBlockDetector::judge_top_color(
    const cv::Mat& bgr,
    const cv::Rect& roi,
    const vision::ColorThreshold& red,
    const vision::ColorThreshold& blue,
    const ColorParams& p)
{
  ColorVerdict v;

  if (bgr.empty()) {
    v.color = kUnknown; v.reason = kReasonRoiTooSmall; v.note = "空图";
    return v;
  }
  if (bgr.type() != CV_8UC3) {
    // ⚠️ 不静默转：本模块的契约是 BGR8（`cv_bridge` 那侧已确保）。
    //    到这里说明上游改了编码 —— 正是 2026-09-20 踩过的那类坑
    //    （`enable_normalize` 一开，检测器 `toCvShare(BGR8)` 直接抛异常）。
    v.color = kUnknown; v.reason = kReasonOccluded;
    v.note = "图像不是 CV_8UC3（BGR8）—— 去查 image_preprocessor 的编码设置";
    return v;
  }

  const cv::Rect r = roi & cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (r.width <= 0 || r.height <= 0) {
    v.color = kUnknown; v.reason = kReasonRoiTooSmall;
    v.note = "ROI 与原图无交集（越界？）—— 没算";
    return v;
  }
  std::string pre;
  if (r != roi) {
    // ⚠️ 部分越界**要说出来**：不报的话，"位置填偏了一点"会表现成"颜色判错"，
    //    而现场会去调 HSV 阈值 —— 查错方向。
    pre = "⚠️ ROI 有部分越界，已按交集算 | ";
  }

  const cv::Mat sub = bgr(r);
  cv::Mat hsv;
  cv::cvtColor(sub, hsv, cv::COLOR_BGR2HSV);

  auto count_hits = [&hsv](const vision::ColorThreshold& th) -> int {
    if (th.groups.empty()) return 0;
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto& g : th.groups) {
      cv::Mat m;
      cv::inRange(hsv, g.lower, g.upper, m);
      mask |= m;
    }
    return cv::countNonZero(mask);
  };
  const int red_px  = count_hits(red);
  const int blue_px = count_hits(blue);

  const int total = r.width * r.height;
  const int valid = red_px + blue_px;

  v.valid_pixel_count = valid;
  v.valid_ratio = (total > 0) ? static_cast<double>(valid) / total : 0.0;
  v.red_ratio   = (valid > 0) ? static_cast<double>(red_px) / valid : 0.0;
  v.blue_ratio  = (valid > 0) ? 1.0 - v.red_ratio : 0.0;
  v.color_margin = std::abs(v.red_ratio - v.blue_ratio);

  char buf[320];

  // ── ① 一个有效像素都没有 → **NONE**（头文件边界①）──
  //    "六个面都贴了贴纸" ⇒ 连一个红/蓝像素都没有，说明这里**没有方块**，
  //    而不是"没看清"。这与"看到了但判不出来"（UNKNOWN）必须是两个值。
  if (valid == 0) {
    v.color = kNone; v.reason = kReasonNone;
    std::snprintf(buf, sizeof(buf), "%s区域内一个红/蓝像素都没有 → NONE（这里没有方块）",
                  pre.c_str());
    v.note = buf;
    return v;
  }

  // ── ② **有效像素闸**（第一道，先于颜色比例）──
  //    ⚠️ 这一条是本模块补的（规格没写），理由见头文件边界②：
  //       24 个红像素 / 900 算出来也是 100% 红，但它**不该**与 600/900 同等可信。
  if (valid < std::max(1, p.min_valid_pixels) || v.valid_ratio < p.min_valid_ratio) {
    v.color = kUnknown; v.reason = kReasonLowValidPixels;
    std::snprintf(buf, sizeof(buf),
                  "%s有效像素 %d 个（占比 %.0f%%）没过闸（需 ≥%d 个 且 ≥%.0f%%）"
                  "→ UNKNOWN：**看到了东西，但太少，不敢说它是方块**。"
                  "动作: 靠近到能看清为止",
                  pre.c_str(), valid, v.valid_ratio * 100.0,
                  p.min_valid_pixels, p.min_valid_ratio * 100.0);
    v.note = buf;
    return v;
  }

  // ── ③ 颜色比例 ──
  const double decisive = std::max(0.5, std::min(p.decisive_ratio, 0.999));

  if (v.red_ratio >= decisive) {
    v.color = kRed; v.reason = kReasonNone;
    std::snprintf(buf, sizeof(buf), "%s红占 %.0f%%（> %.0f%%）→ RED | margin=%.2f 有效像素 %d",
                  pre.c_str(), v.red_ratio * 100.0, decisive * 100.0, v.color_margin, valid);
    v.note = buf;
    return v;
  }
  if (v.blue_ratio >= decisive) {
    v.color = kBlue; v.reason = kReasonNone;
    std::snprintf(buf, sizeof(buf), "%s蓝占 %.0f%%（> %.0f%%）→ BLUE | margin=%.2f 有效像素 %d",
                  pre.c_str(), v.blue_ratio * 100.0, decisive * 100.0, v.color_margin, valid);
    v.note = buf;
    return v;
  }

  // ── ④ 剩下的都是 UNKNOWN，但**原因分开写**（下游采取的动作不同）──
  v.color = kUnknown;
  if (v.red_ratio >= p.close_lo && v.red_ratio <= p.close_hi) {
    v.reason = kReasonRatioAmbiguous;
    std::snprintf(buf, sizeof(buf),
                  "%s红蓝接近（红 %.0f%% / 蓝 %.0f%%，margin=%.2f）→ UNKNOWN："
                  "**需靠近重新观察**（§3.3 原话）—— 别停在这",
                  pre.c_str(), v.red_ratio * 100.0, v.blue_ratio * 100.0, v.color_margin);
  } else {
    v.reason = kReasonRatioAmbiguous;
    std::snprintf(buf, sizeof(buf),
                  "%s红 %.0f%% 未过 %.0f%% 门槛 → UNKNOWN。"
                  "⚠️ 55~70%% 是**规格未定义段**（§3.3 只定义了 >70%% 与 45-55%%），"
                  "本模块取最保守读法：**只有 >70%% 才定色**",
                  pre.c_str(), v.red_ratio * 100.0, decisive * 100.0);
  }
  v.note = buf;
  return v;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 候选区域 —— "色面在哪"的全部几何假设都在这一个函数里
// ═══════════════════════════════════════════════════════════════════════════════
cv::Rect SkyBlockDetector::top_face_roi(const cv::Rect& bbox, double top_face_frac)
{
  if (bbox.width <= 0 || bbox.height <= 0) return cv::Rect();
  // ⚠️ 夹到 (0,1]：给了 0 或负数会让高度变 0 → 后面判成"无交集"，
  //    而症状是"恒判 UNKNOWN"，与"没配 ROI"长得一样 —— 别让它静默。
  const double f = std::max(0.01, std::min(top_face_frac, 1.0));
  const int h = std::max(1, static_cast<int>(bbox.height * f));
  return cv::Rect(bbox.x, bbox.y, bbox.width, h);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════════════════
SkyBlockDetector::SkyBlockDetector(const rclcpp::NodeOptions& options)
  : Node("sky_block_detector", options)
{
  // ── 颜色阈值文件路径 ──
  // ⚠️ `camera_params.yaml` 里写的是**相对路径**，而 `color_utils.cpp` 是
  //    `std::ifstream(yaml_path)` —— 相对的是**启动时的 $PWD**，节点里没有回退解析。
  //    ⇒ launch **必须**用绝对路径覆盖它（见 launch/bringup_full.launch.py 的 color_yaml）。
  this->declare_parameter<std::string>("color_yaml_path", "config/color_thresholds.yaml");

  // ── 判色阈值 ──
  this->declare_parameter<double>("decisive_ratio",   color_params_.decisive_ratio);
  this->declare_parameter<double>("close_lo",         color_params_.close_lo);
  this->declare_parameter<double>("close_hi",         color_params_.close_hi);
  this->declare_parameter<int>("min_valid_pixels",    color_params_.min_valid_pixels);
  this->declare_parameter<double>("min_valid_ratio",  color_params_.min_valid_ratio);

  // ── 几何 ──
  this->declare_parameter<double>("top_face_frac",  top_face_frac_);
  this->declare_parameter<std::vector<double>>("debug_roi_ratio", debug_roi_ratio_);
  this->declare_parameter<int>("roi_min_area", roi_min_area_);

  // ── 时序确认 ──
  this->declare_parameter<int>("enter_confirm_frames", enter_confirm_frames_);
  this->declare_parameter<int>("exit_confirm_frames",  exit_confirm_frames_);

  this->get_parameter("color_yaml_path", color_yaml_path_);
  this->get_parameter("decisive_ratio",   color_params_.decisive_ratio);
  this->get_parameter("close_lo",         color_params_.close_lo);
  this->get_parameter("close_hi",         color_params_.close_hi);
  this->get_parameter("min_valid_pixels", color_params_.min_valid_pixels);
  this->get_parameter("min_valid_ratio",  color_params_.min_valid_ratio);
  this->get_parameter("top_face_frac",    top_face_frac_);
  this->get_parameter("debug_roi_ratio",  debug_roi_ratio_);
  this->get_parameter("roi_min_area",     roi_min_area_);
  this->get_parameter("enter_confirm_frames", enter_confirm_frames_);
  this->get_parameter("exit_confirm_frames",  exit_confirm_frames_);

  // ── 规范化 ──
  color_params_.decisive_ratio = std::max(0.5, std::min(color_params_.decisive_ratio, 0.999));
  color_params_.close_lo = std::max(0.0, std::min(color_params_.close_lo, 0.5));
  color_params_.close_hi = std::max(0.5, std::min(color_params_.close_hi, 1.0));
  if (color_params_.close_hi < color_params_.close_lo) {
    std::swap(color_params_.close_hi, color_params_.close_lo);
    RCLCPP_WARN(this->get_logger(),
      "close_lo/close_hi 反了（%.2f / %.2f）—— 已交换。去查 yaml",
      color_params_.close_lo, color_params_.close_hi);
  }
  color_params_.min_valid_pixels = std::max(1, color_params_.min_valid_pixels);
  color_params_.min_valid_ratio  = std::max(0.0, std::min(color_params_.min_valid_ratio, 1.0));
  top_face_frac_ = std::max(0.01, std::min(top_face_frac_, 1.0));
  roi_min_area_  = std::max(1, roi_min_area_);
  // ⚠️ 退出帧数**必须 ≥ 进入帧数**：反了的话"刚确认就马上掉出去"，
  //    滞回窗口形同虚设（规则 3.5.8 对手会翻，那种抖动最贵）。
  enter_confirm_frames_ = std::max(1, enter_confirm_frames_);
  exit_confirm_frames_  = std::max(enter_confirm_frames_, exit_confirm_frames_);
  for (auto& c : confirmer_) {
    c.enter_frames = enter_confirm_frames_;
    c.exit_frames  = exit_confirm_frames_;
    c.reset();
  }

  // ── 加载 HSV 阈值 ──
  // ⚠️ **加载失败要让它响**（抛）。`load_color_thresholds` 在文件打不开/格式错时
  //    抛 `cv::Exception` —— 这里**不接**，让它冒上去变成节点启动失败。
  //    理由：静默回落成"空阈值组"的话，节点会**正常跑、每帧都判 NONE**，
  //    而 NONE 的语义是"这里没有方块" ⇒ 融合层以为塔是空的。
  //    这正是本仓库最恨的"看着配好了，实际没接上"。
  {
    const auto all = vision::load_color_thresholds(color_yaml_path_);
    auto need = [&](const char* key) -> vision::ColorThreshold {
      const auto it = all.find(key);
      if (it == all.end()) {
        throw std::runtime_error(
            std::string("color_thresholds.yaml 里没有 '") + key +
            "' 这一节 —— 判色无法进行。文件: " + color_yaml_path_);
      }
      return it->second;
    };
    red_  = need("red_team");
    blue_ = need("blue_team");
  }

  // ── 订阅 ──
  // ⚠️ 与三个已有检测器**同口径**：BEST_EFFORT + 队列深度 1（任务书 §3.0 硬约束第 2 条）。
  //    发布端是 `image_preprocessor`，也是 depth=1 best_effort。
  //    **两侧必须同为 best_effort** —— RELIABLE 订阅接 BEST_EFFORT 发布 = 静默收不到。
  const auto stream_qos = rclcpp::QoS(1).best_effort();

  sub_wide_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
      "/perception/camera_wide/preprocessed", stream_qos,
      std::bind(&SkyBlockDetector::wide_callback, this, std::placeholders::_1));
  sub_tele_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
      "/perception/camera_tele/preprocessed", stream_qos,
      std::bind(&SkyBlockDetector::tele_callback, this, std::placeholders::_1));

  // ── 发布 ──
  // ⚠️ 话题名带 `_wide` / `_tele` 后缀 —— 与 `mustika_wide` / `pillars_wide` /
  //    `building_spots_wide` 同体例。
  pub_wide_ = this->create_publisher<br_perception::msg::CameraSkyBlock>(
      "/perception/camera/sky_blocks_wide", stream_qos);
  pub_tele_ = this->create_publisher<br_perception::msg::CameraSkyBlock>(
      "/perception/camera/sky_blocks_tele", stream_qos);
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/perception/camera/sky_blocks_diagnostics", 10);

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SkyBlockDetector::on_parameter_change, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "SkyBlockDetector ready | 阈值=%s (红 %zu 组 / 蓝 %zu 组) | "
    "判据: 红/蓝占比>%.0f%% 定色 · 有效像素≥%d 且≥%.0f%% · 45~55%% 判未知 | "
    "时序: 连续 %d 帧确认 / %d 帧掉出 | top_face_frac=%.2f | ROI 来源=%s",
    color_yaml_path_.c_str(), red_.groups.size(), blue_.groups.size(),
    color_params_.decisive_ratio * 100.0, color_params_.min_valid_pixels,
    color_params_.min_valid_ratio * 100.0,
    enter_confirm_frames_, exit_confirm_frames_, top_face_frac_, roi_source_name_.c_str());

  // ⚠️⚠️ **没有 ROI 来源时，把它说在最显眼的地方**。
  //    当前**只实现了手动 ROI**（`debug_roi_ratio`）—— 雷达建筑位投影那条主路
  //    **尚未实现**（见头文件里 `roi_source_name_` 的注释）。
  //    不配的话：节点**照常起、照常发消息**，但每帧都是 NONE ——
  //    而 NONE 的意思是"这里没有方块"，**症状与"塔真的是空的"完全一样**。
  if (debug_roi_ratio_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "**没有配置 debug_roi_ratio** → 本节点不会产出有效的判色结果（恒发 NONE）。"
      "⚠️ 这不是故障：本模块**只做判色，不做检测**，而"
      "「雷达建筑位投影」那条 ROI 主路**尚未实现**。"
      "现在先用 `debug_roi_ratio: [x, y, w, h]`（相对宽高的 0~1）手动指定一个区域"
      "把它跑起来（台架调阈值用）；补上投影后这个参数就该留空。");
  } else if (debug_roi_ratio_.size() != 4) {
    RCLCPP_ERROR(this->get_logger(),
      "debug_roi_ratio 长度是 %zu，**必须是 4**（x, y, w, h 相对比例）→ 整组忽略，恒发 NONE",
      debug_roi_ratio_.size());
    debug_roi_ratio_.clear();
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 回调
// ═══════════════════════════════════════════════════════════════════════════════
void SkyBlockDetector::wide_callback(
    const br_perception::msg::PreprocessedImage::SharedPtr msg)
{
  process_and_publish(msg, 0, pub_wide_);
}

void SkyBlockDetector::tele_callback(
    const br_perception::msg::PreprocessedImage::SharedPtr msg)
{
  process_and_publish(msg, 1, pub_tele_);
}

void SkyBlockDetector::process_and_publish(
    const br_perception::msg::PreprocessedImage::SharedPtr msg,
    uint8_t camera_id,
    const rclcpp::Publisher<br_perception::msg::CameraSkyBlock>::SharedPtr& pub)
{
  const auto t0 = mono_now();

  br_perception::msg::CameraSkyBlock out;
  out.camera_id  = camera_id;
  out.roi_source = roi_source_;
  out.spot_id    = spot_id_;

  if (msg) {
    // ⚠️ **原样继承曝光时刻**，不是 now()。任务书 §四「时间戳约定」：
    //    取发出时刻会多算 ≈43ms 的位移，污染下游的外推。
    out.header = msg->header;
  } else {
    out.header.stamp = this->get_clock()->now();
    out.detected = false;
    out.top_color = kNone;
    out.state = TemporalConfirmer::kUnconfirmed;
    pub->publish(out);
    return;
  }

  // ── 取图（零拷贝：msg->image 是内嵌字段，用带 tracked_object 的重载）──
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg->image, msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "cv_bridge 转换失败（cam%d）: %s —— ⚠️ 常见原因: 上游把编码改成了 float32",
      static_cast<int>(camera_id), e.what());
    out.detected = false;
    out.top_color = kNone;
    out.state = TemporalConfirmer::kUnconfirmed;
    pub->publish(out);
    return;
  }

  const cv::Mat& bgr = cv_ptr->image;

  // ── 决定 ROI（当前只有"手动"这一条来源）──
  ColorVerdict v;
  cv::Rect roi;
  bool have_roi = false;

  if (debug_roi_ratio_.size() == 4 && bgr.cols > 0 && bgr.rows > 0) {
    const int x = static_cast<int>(std::lround(debug_roi_ratio_[0] * bgr.cols));
    const int y = static_cast<int>(std::lround(debug_roi_ratio_[1] * bgr.rows));
    const int w = static_cast<int>(std::lround(debug_roi_ratio_[2] * bgr.cols));
    const int h = static_cast<int>(std::lround(debug_roi_ratio_[3] * bgr.rows));

    // ⚠️ 先用 `top_face_roi` 收成"上半区域" —— 这一步就是头文件里那条**未决几何**
    //    （红蓝怎么分布）的落点。暂定"红蓝各半"；若最终证实是"一面红+对面蓝"，
    //    **只改 `top_face_roi` 这一个函数**，判色核心与本文件的其它部分一行不动。
    roi = top_face_roi(cv::Rect(x, y, w, h), top_face_frac_);

    if (roi.width * roi.height >= roi_min_area_) {
      have_roi = true;
    } else {
      v.color = kUnknown; v.reason = kReasonRoiTooSmall;
      v.note  = "ROI 太小（" + std::to_string(roi.width * roi.height) +
                " px² < roi_min_area " + std::to_string(roi_min_area_) + "）—— 没算";
    }
  } else {
    v.color = kUnknown; v.reason = kReasonRoiTooSmall;
    v.note  = "没有 ROI 来源（本模块不做检测，投影主路尚未实现）";
  }

  if (have_roi) {
    v = judge_top_color(bgr, roi, red_, blue_, color_params_);
  }

  // ── 时序确认 ──
  // ⚠️ 喂给状态机的是"**这一帧判出了红或蓝**"（satisfied），不是"判色成功"：
  //    UNKNOWN 与 NONE 都算**不满足** —— 它们都不是可以拿去改塔归属的结论。
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    auto& c = confirmer_[camera_id < kCamCount ? camera_id : 0];
    out.state = static_cast<uint8_t>(c.push(v.color == kRed || v.color == kBlue));
  }

  // ⚠️ `detected` 与 `top_color` 的关系（头文件边界①）：
  //    `NONE` ⇒ `detected=false`（这里没有方块）
  //    其余三档 ⇒ `detected=true` —— **`UNKNOWN` 也是"检测到了"**，
  //    只是颜色判不出来。把 UNKNOWN 也算成"没检测到"会让整座塔凭空消失。
  out.detected  = (v.color != kNone);
  out.top_color = static_cast<uint8_t>(v.color);
  out.unknown_reason = static_cast<uint8_t>(v.reason);

  out.red_ratio        = static_cast<float>(v.red_ratio);
  out.blue_ratio       = static_cast<float>(v.blue_ratio);
  out.color_margin     = static_cast<float>(v.color_margin);
  out.valid_ratio      = static_cast<float>(v.valid_ratio);
  out.valid_pixel_count = static_cast<uint32_t>(std::max(0, v.valid_pixel_count));

  // 置信度：**只反映"这次判色有多可信"，不是"这里有方块的概率"**。
  // ⚠️ 本模块不做检测，所以这个数只能这么定；定义写在这里，别当下游的检测概率用。
  //    判出颜色时 = 该颜色的占比；其余一律 0（结论不可信就该是 0）。
  out.confidence = (v.color == kRed)  ? static_cast<float>(v.red_ratio)
                 : (v.color == kBlue) ? static_cast<float>(v.blue_ratio)
                                      : 0.0f;

  if (have_roi) {
    // ── 坐标反算回**原图** ──
    // ⚠️ 位置类与尺度类**分开调**（`letterbox_to_orig_xy` vs `letterbox_to_orig_size`）：
    //    角点要**减**填充偏移，宽高**不能减** —— 减了会出负宽度。
    //    这是整套反算最易错的一处（P0-3 当时踩过）。
    double x1 = roi.x,             y1 = roi.y;
    double x2 = roi.x + roi.width, y2 = roi.y + roi.height;
    letterbox_to_orig_xy(msg->scale, msg->pad_x, msg->pad_y, x1, y1);
    letterbox_to_orig_xy(msg->scale, msg->pad_x, msg->pad_y, x2, y2);

    out.target_u  = static_cast<float>((x1 + x2) * 0.5);
    out.target_v  = static_cast<float>((y1 + y2) * 0.5);
    out.roi_width  = static_cast<float>(x2 - x1);
    out.roi_height = static_cast<float>(y2 - y1);
  } else {
    out.target_u = out.target_v = out.roi_width = out.roi_height = 0.0f;
  }

  pub->publish(out);

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    (camera_id == 0 ? wide_count_ : tele_count_)++;
    (camera_id == 0 ? wide_frame_seq_ : tele_frame_seq_)++;
  }

  publish_diagnostics(camera_id, v, out, ms_since(t0));
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════════
void SkyBlockDetector::publish_diagnostics(
    uint8_t camera_id, const ColorVerdict& v,
    const br_perception::msg::CameraSkyBlock& out, double total_ms)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name        = std::string("sky_block_detector/cam") + std::to_string(camera_id);
  st.hardware_id = camera_id == 0 ? "camera_wide" : "camera_tele";

  // ⚠️ 等级只反映"这一帧判得顺不顺"，**不是节点健康**。
  //    UNKNOWN 不是故障 —— 它可能是"相机没看清"（正常，靠近重看即可）。
  st.level = (v.color == kRed || v.color == kBlue)
                 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  st.message = v.note.empty() ? color_name(v.color) : v.note;

  auto kv = [](const std::string& k, const std::string& val) {
    diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = val; return p;
  };
  char buf[192];
  st.values.push_back(kv("top_color", color_name(v.color)));
  st.values.push_back(kv("unknown_reason", std::to_string(static_cast<int>(v.reason))));
  st.values.push_back(kv("roi_source", roi_source_name_));
  std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(out.spot_id));
  st.values.push_back(kv("spot_id", buf));

  std::snprintf(buf, sizeof(buf), "%.3f / %.3f (margin %.3f)",
                v.red_ratio, v.blue_ratio, v.color_margin);
  st.values.push_back(kv("red/blue ratio", buf));
  std::snprintf(buf, sizeof(buf), "%d px (%.1f%%)",
                v.valid_pixel_count, v.valid_ratio * 100.0);
  st.values.push_back(kv("valid pixels", buf));

  const char* st_name[] = {"UNCONFIRMED", "CONFIRMED", "COASTING"};
  const int si = static_cast<int>(out.state);
  st.values.push_back(kv("state", (si >= 0 && si < 3) ? st_name[si] : "?"));
  std::snprintf(buf, sizeof(buf), "%d / %d", enter_confirm_frames_, exit_confirm_frames_);
  st.values.push_back(kv("enter/exit frames", buf));

  std::snprintf(buf, sizeof(buf), "%.2f / [%.2f, %.2f]",
                color_params_.decisive_ratio, color_params_.close_lo, color_params_.close_hi);
  st.values.push_back(kv("decisive / close_band", buf));
  st.values.push_back(kv("color_yaml", color_yaml_path_));
  std::snprintf(buf, sizeof(buf), "%.2f", total_ms);
  st.values.push_back(kv("total_ms", buf));

  arr.status.push_back(st);
  pub_diagnostics_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 动态参数
// ═══════════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult SkyBlockDetector::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  for (const auto& p : params) {
    const auto& n = p.get_name();
    if (n == "decisive_ratio") {
      const double x = p.as_double();
      if (x < 0.5 || x > 0.999) {
        res.successful = false;
        res.reason = "decisive_ratio 必须在 [0.5, 0.999]";
        return res;
      }
      color_params_.decisive_ratio = x;
    } else if (n == "close_lo") {
      color_params_.close_lo = std::max(0.0, std::min(p.as_double(), 0.5));
    } else if (n == "close_hi") {
      color_params_.close_hi = std::max(0.5, std::min(p.as_double(), 1.0));
    } else if (n == "min_valid_pixels") {
      color_params_.min_valid_pixels = std::max(1, static_cast<int>(p.as_int()));
    } else if (n == "min_valid_ratio") {
      color_params_.min_valid_ratio = std::max(0.0, std::min(p.as_double(), 1.0));
    } else if (n == "top_face_frac") {
      top_face_frac_ = std::max(0.01, std::min(p.as_double(), 1.0));
    } else if (n == "debug_roi_ratio") {
      const auto arr = p.as_double_array();
      if (arr.size() == 4) {
        debug_roi_ratio_ = arr;
      } else {
        res.successful = false;
        res.reason = "debug_roi_ratio 必须是 4 个数（x, y, w, h 相对比例）";
        return res;
      }
    } else if (n == "roi_min_area") {
      roi_min_area_ = std::max(1, static_cast<int>(p.as_int()));
    } else if (n == "enter_confirm_frames") {
      enter_confirm_frames_ = std::max(1, static_cast<int>(p.as_int()));
      exit_confirm_frames_  = std::max(enter_confirm_frames_, exit_confirm_frames_);
      for (auto& c : confirmer_) {
        c.enter_frames = enter_confirm_frames_;
        c.exit_frames  = exit_confirm_frames_;
      }
    } else if (n == "exit_confirm_frames") {
      exit_confirm_frames_ = std::max(enter_confirm_frames_, static_cast<int>(p.as_int()));
      for (auto& c : confirmer_) {
        c.exit_frames = exit_confirm_frames_;
      }
    }
    // ⚠️ `color_yaml_path` **热改无效**：阈值是构造时加载的，改路径不会重新加载。
    //    这条的处置是**不改**（要换阈值文件就重启）—— 静默忽略比假装支持好。
  }
  return res;
}

}  // namespace camera
}  // namespace br_perception

// ═══════════════════════════════════════════════════════════════════════════════
// 组件注册（容器化时被容器加载; 也生成同名独立可执行）
// ═══════════════════════════════════════════════════════════════════════════════
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::SkyBlockDetector)
