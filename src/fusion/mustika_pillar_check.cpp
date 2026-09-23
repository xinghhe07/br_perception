/// ═══════════════════════════════════════════════════════════════════════════════
/// mustika_pillar_check.cpp — 「穆斯蒂卡在柱上吗」的相机-雷达交叉确认（任务书 §15.5）
///
/// **设计理由、三条必须守住的边界、两路合并规则全部写在头文件里** ——
/// 本文件只实现，不重复论证。动手改之前先读 `mustika_pillar_check.hpp`，特别是：
///   · "⚠️⚠️ 三条必须守住的边界"：判不了说未知 / **未标定不许发可信结论** / 外参只认 TF
///   · `decide()` 的"两路是**互相印证**，不是任一路通过"
///
/// ⚠️ 本文件**一行都没编译过**（本机无 ROS；见 doc/模块进度.md 顶部那条更正）。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/fusion/mustika_pillar_check.hpp"

#include <rclcpp_components/register_node_macro.hpp>
// ⚠️ 这个头**必须显式 include**：漏了的话报错长得像语法错
//    （`expected constructor, destructor, or type conversion at end of input`，
//     指向文件最后一行）—— 2026-09-23 首次编译在另外两个节点上撞过，记在这里。

#include <tf2/exceptions.h>   // ⚠️ 显式 include —— `tf2::TransformException` 别指望传递包含
#include <tf2/time.h>         // ⚠️ 同上 —— `tf2::durationFromSec`（查 TF 时传超时用）

#include <algorithm>
#include <cmath>
#include <cstdio>      // std::snprintf —— 诊断文案拼字符串（⚠️ 只用它，不用 std::string 拼接爆表）
#include <functional>
#include <utility>
#include <vector>

namespace br_perception {
namespace fusion {

namespace {

/// @brief 单调时钟当前值（**降级/新鲜度判定一律用它**；ROS 时间在仿真里会暂停/跳变）
inline std::chrono::steady_clock::time_point mono_now()
{
  return std::chrono::steady_clock::now();
}

/// @brief `time_point` 的"从未设置"哨兵判定（默认构造的 epoch 计数为 0）
inline bool tp_unset(const std::chrono::steady_clock::time_point& tp)
{
  return tp.time_since_epoch().count() == 0;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// 几何：单目测距 + 反投影（**纯函数，可单独推理**）
// ═══════════════════════════════════════════════════════════════════════════════
std::optional<double> MustikaPillarCheck::range_from_pixel_radius(
    double radius_px, double focal_px, double ball_diameter_m)
{
  // 球在图像里的像素半径 r ≈ f · (D/2) / d  ⇒  d = f · D / (2r)
  if (radius_px <= 0.0 || focal_px <= 0.0 || ball_diameter_m <= 0.0) {
    return std::nullopt;
  }
  const double d = focal_px * ball_diameter_m / (2.0 * radius_px);
  if (!std::isfinite(d) || d <= 0.0) return std::nullopt;
  return d;
}

void MustikaPillarCheck::back_project(double u, double v, double depth, double fx, double fy,
                                      double cx, double cy, double* xyz)
{
  // ⚠️ 针孔反投影，得到**光心坐标系**下的点：+X 右 / +Y 下 / +Z 前。
  //
  // ⚠️⚠️ **本函数返回的是光心系，不是本节点用来查 TF 的那个系**（2026-09-23 更正）：
  //    原先这里写"这与 ROS 的 `camera_*_frame` 约定一致（那些 frame 通常挂在光学系上）"
  //    —— **在本仓库里不成立**：`camera_wide_frame` / `camera_tele_frame` 在
  //    `config/fusion_params.yaml` 的 `static_tf:` 块里 `rotation_rpy = [0,0,0]`，
  //    即与 `base_link` 同向 = **机身系**（REP-103: +X 前 / +Y 左 / +Z 上）。
  //    ⇒ **调用方必须做一次光心系→机身系的固定旋转**（`judge_one` 里已做，别删）。
  //    ⚠️ 全仓库 grep `optical` 零命中 —— 没有任何地方建过光学系 frame，
  //       所以这个转换只能由"产生三维点再查 TF"的调用方负责。
  xyz[0] = (u - cx) * depth / fx;
  xyz[1] = (v - cy) * depth / fy;
  xyz[2] = depth;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════════════════
MustikaPillarCheck::MustikaPillarCheck(const rclcpp::NodeOptions& options)
  : Node("mustika_pillar_check", options)
{
  // ── 话题 ──
  this->declare_parameter<std::string>("wide_topic",  wide_topic_);
  this->declare_parameter<std::string>("tele_topic",  tele_topic_);
  this->declare_parameter<std::string>("lidar_topic", lidar_topic_);
  this->declare_parameter<std::string>("state_topic", state_topic_);
  this->declare_parameter<std::string>("diag_topic",  diag_topic_);

  // ── ⚠️⚠️ 人工闸 ──
  this->declare_parameter<bool>("extrinsics_calibrated", extrinsics_calibrated_);

  // ── 几何 / 标定 ──
  this->declare_parameter<double>("ball_diameter_m", ball_diameter_m_);
  this->declare_parameter<double>("ball_center_above_pillar_top_m",
                                  ball_center_above_pillar_top_m_);
  this->declare_parameter<std::vector<double>>(
      "focal_px", {focal_px_[0], focal_px_[1]});
  this->declare_parameter<std::vector<double>>("cx_px", {cx_px_[0], cx_px_[1]});
  this->declare_parameter<std::vector<double>>("cy_px", {cy_px_[0], cy_px_[1]});
  this->declare_parameter<std::string>("world_frame", world_frame_);
  // ⚠️ **frame 名必须能配**（2026-09-23 自查发现：第一版只给了默认值、没 declare，
  //    于是它与 TF 树对不上时**改都改不了**，而症状是"恒判未知、不报错"）。
  this->declare_parameter<std::vector<std::string>>(
      "camera_frames", {camera_frames_[0], camera_frames_[1]});

  // ── 判据 ──
  this->declare_parameter<double>("pos_tolerance_m",      pos_tolerance_m_);
  this->declare_parameter<double>("height_tolerance_m",   height_tolerance_m_);
  this->declare_parameter<double>("residual_max_m",       residual_max_m_);
  this->declare_parameter<double>("min_confidence",       min_confidence_);
  this->declare_parameter<double>("min_radius_px",        min_radius_px_);
  this->declare_parameter<double>("radius_noise_px",      radius_noise_px_);
  this->declare_parameter<double>("fallen_dist_m",        fallen_dist_m_);
  this->declare_parameter<double>("fallen_height_max_m",  fallen_height_max_m_);
  this->declare_parameter<double>("cross_check_rel",      cross_check_rel_);
  this->declare_parameter<double>("stale_sec",            stale_sec_);
  this->declare_parameter<double>("tf_timeout_sec",       tf_timeout_sec_);
  this->declare_parameter<double>("publish_rate_hz",      publish_rate_hz_);

  this->get_parameter("wide_topic",  wide_topic_);
  this->get_parameter("tele_topic",  tele_topic_);
  this->get_parameter("lidar_topic", lidar_topic_);
  this->get_parameter("state_topic", state_topic_);
  this->get_parameter("diag_topic",  diag_topic_);
  this->get_parameter("extrinsics_calibrated", extrinsics_calibrated_);
  this->get_parameter("ball_diameter_m", ball_diameter_m_);
  this->get_parameter("ball_center_above_pillar_top_m", ball_center_above_pillar_top_m_);
  this->get_parameter("world_frame", world_frame_);
  this->get_parameter("pos_tolerance_m",     pos_tolerance_m_);
  this->get_parameter("height_tolerance_m",  height_tolerance_m_);
  this->get_parameter("residual_max_m",      residual_max_m_);
  this->get_parameter("min_confidence",      min_confidence_);
  this->get_parameter("min_radius_px",       min_radius_px_);
  this->get_parameter("radius_noise_px",     radius_noise_px_);
  this->get_parameter("fallen_dist_m",       fallen_dist_m_);
  this->get_parameter("fallen_height_max_m", fallen_height_max_m_);
  this->get_parameter("cross_check_rel",     cross_check_rel_);
  this->get_parameter("stale_sec",           stale_sec_);
  this->get_parameter("tf_timeout_sec",      tf_timeout_sec_);
  this->get_parameter("publish_rate_hz",     publish_rate_hz_);

  // ⚠️ 三个数组型参数**长度必须与相机路数一致**，否则整组忽略并回落默认 ——
  //    这是本仓库 `building_spot_analyzer` 用过的同一个口径（长度不符不猜）。
  auto read_array = [&](const char* name, std::array<double, kCamCount>& dst) {
    std::vector<double> v;
    this->get_parameter(name, v);
    if (v.size() == kCamCount) {
      std::copy(v.begin(), v.end(), dst.begin());
    } else {
      RCLCPP_WARN(this->get_logger(),
        "参数 %s 长度 %zu ≠ 相机路数 %zu → **整组忽略**，沿用默认值（不猜）",
        name, v.size(), kCamCount);
    }
  };
  read_array("focal_px", focal_px_);
  read_array("cx_px",    cx_px_);
  read_array("cy_px",    cy_px_);

  {
    std::vector<std::string> fr;
    this->get_parameter("camera_frames", fr);
    if (fr.size() == kCamCount) {
      camera_frames_[0] = fr[0];
      camera_frames_[1] = fr[1];
    } else {
      RCLCPP_WARN(this->get_logger(),
        "camera_frames 长度 %zu ≠ 相机路数 %zu → 整组忽略，沿用默认", fr.size(), kCamCount);
    }
  }

  // ── 规范化 ──
  // ⚠️ 焦距/主点必须为正，否则反投影会算出无意义的点（除以 0 / 负数）
  for (size_t i = 0; i < kCamCount; ++i) {
    focal_px_[i] = std::max(1.0, focal_px_[i]);
  }
  ball_diameter_m_ = std::max(0.0, ball_diameter_m_);
  pos_tolerance_m_ = std::max(0.0, pos_tolerance_m_);
  height_tolerance_m_ = std::max(0.0, height_tolerance_m_);
  // ⚠️ `residual_max_m_` 是"明确反对"的门槛，**必须 ≥ 通过门槛** ——
  //    反了的话会出现"既算通过、又算反对"的重叠区间（判据自相矛盾）。
  residual_max_m_ = std::max(pos_tolerance_m_, residual_max_m_);
  stale_sec_ = std::max(0.01, stale_sec_);
  // ⚠️ 超时**不给 0**（理由见 hpp 成员注释）；上限 1s —— 比这更长等于让回调卡住
  tf_timeout_sec_ = std::max(0.001, std::min(tf_timeout_sec_, 1.0));
  publish_rate_hz_ = std::max(0.1, std::min(publish_rate_hz_, 20.0));
  cross_check_rel_ = std::max(0.0, cross_check_rel_);

  // ── TF：**只读**（不广播，见头文件成员 `tf_buffer_` 的注释）──
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── 订阅 ──
  // ⚠️ 相机检测结果用 `SensorDataQoS`（BEST_EFFORT）—— 发布端是检测器，
  //    它们发 BEST_EFFORT；**RELIABLE 订阅接 BEST_EFFORT 发布 = 静默收不到**。
  const auto cam_qos = rclcpp::SensorDataQoS();
  camera_subs_[kCamWide] = this->create_subscription<br_perception::msg::CameraMustika>(
      wide_topic_, cam_qos,
      [this](const br_perception::msg::CameraMustika::SharedPtr m) {
        on_camera(static_cast<uint8_t>(kCamWide), m);
      });
  camera_subs_[kCamTele] = this->create_subscription<br_perception::msg::CameraMustika>(
      tele_topic_, cam_qos,
      [this](const br_perception::msg::CameraMustika::SharedPtr m) {
        on_camera(static_cast<uint8_t>(kCamTele), m);
      });

  // 雷达结果用 RELIABLE（发布端是 `create_publisher<LidarResult>(topic, 10)`）
  lidar_sub_ = this->create_subscription<br_perception::msg::LidarResult>(
      lidar_topic_, rclcpp::QoS(10),
      std::bind(&MustikaPillarCheck::on_lidar, this, std::placeholders::_1));

  // ── 发布 ──
  // ⚠️ `mustika_state` 的消费者是 `perception_pipeline`（它订阅后用 RELIABLE）——
  //    这里也发 RELIABLE，两侧一致且对 BEST_EFFORT 订阅者同样兼容。
  state_pub_ = this->create_publisher<std_msgs::msg::UInt8>(state_topic_, rclcpp::QoS(10));
  diag_pub_  = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diag_topic_, 10);

  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&MustikaPillarCheck::tick, this));

  RCLCPP_INFO(this->get_logger(),
    "MustikaPillarCheck ready | in: %s %s %s | out: %s | %.1fHz | 外参标定=%s",
    wide_topic_.c_str(), tele_topic_.c_str(), lidar_topic_.c_str(),
    state_topic_.c_str(), publish_rate_hz_,
    extrinsics_calibrated_ ? "YES" : "**NO（只发未知）**");

  if (!extrinsics_calibrated_) {
    // ⚠️ 把"这个节点现在不会给可信结论"说在最显眼的地方 —— 否则会有人看到
    //    它一直在发 `0`，以为节点坏了，而去"修"它（实际是那道人工闸在工作）。
    RCLCPP_WARN(this->get_logger(),
      "**外参未标定** → 本节点只计算并发布**残差诊断**，`mustika_state` **恒发 0=未知**。"
      "这是**故意的**：用占位外参算出的'位置一致'是巧合，而本节点最危险的地方"
      "就是它可能碰巧对、然后被当成标定过了。"
      "标定完成后把 `extrinsics_calibrated` 显式改成 true —— 让'开始信任它'是个明确动作。");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 回调
// ═══════════════════════════════════════════════════════════════════════════════
void MustikaPillarCheck::on_camera(uint8_t cam,
                                   const br_perception::msg::CameraMustika::SharedPtr msg)
{
  if (!msg || cam >= kCamCount) return;
  last_ball_[cam] = msg;
  ball_at_[cam] = mono_now();
}

void MustikaPillarCheck::on_lidar(const br_perception::msg::LidarResult::SharedPtr msg)
{
  if (!msg) return;
  lidar_at_ = mono_now();
  lidar_says_object_ = msg->mustika_pillar.has_object_on_top;

  // ⚠️ **未检出的哨兵**：全零（height==0 && diameter==0，见 `PillarStatus.msg`）。
  //    这时**不能**拿 (0,0,0) 去 TF —— 那会得到一个"世界系原点附近的假柱子"。
  const auto& p = msg->mustika_pillar;
  if (p.height == 0.0f && p.diameter == 0.0f) {
    pillar_top_.reset();
    return;
  }

  // ⚠️ 上游没填坐标系 → 无从变换，别猜
  if (msg->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "雷达结果的 `header.frame_id` 为空 → 无法 TF，柱位不可得。"
      "上游 `lidar_perception_node` 应继承输入点云的 frame_id");
    pillar_top_.reset();
    return;
  }

  // ⚠️⚠️ **必须 TF 到 world**（头文件边界③ / 成员 `pillar_top_` 的注释）：
  //    `/perception/lidar/result` 的坐标在**传感器系**里。直接拿它当 world 去和
  //    相机的 world 球心比距离，就是**两边错着同一个外参** ⇒ 残差可能碰巧落进容差。
  geometry_msgs::msg::PointStamped in, out;
  in.header = msg->header;          // 坐标系 + 时间戳**都用上游的**，不是 now()
  in.point  = p.position;           // ⚠️ `position.z` 是**柱脚**（地面投影点）
  try {
    // ⚠️ 用**有返回值**的那个重载 —— 与 `coordinate_transformer.cpp:324` 逐字同型，
    //    那是本仓库**已编译通过**的写法（另有 `void transform(in, out, ...)` 那个重载，
    //    但它在本仓库从未被用过 ⇒ 不拿它当"应该能编"的依据）。
    // ⚠️ 超时**显式传**，不用默认的 0 —— 理由见 hpp 的 `tf_timeout_sec_`。
    out = tf_buffer_->transform(in, world_frame_, tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException& ex) {
    // ⚠️ TF 查不到 → **柱位不可得** → 一路判"未知"。**不静默用传感器系顶上。**
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      // ⚠️⚠️ **两行字面量必须各自带引号**（2026-09-23 编译复核抓出）：
      //    普通字符串字面量**不允许含裸换行**；写成"上一行开引号、下一行才闭合"
      //    会在词法阶段就报 `missing terminating " character`，
      //    而后面的实参被一起吞掉 ⇒ 报错位置看着离真正的错很远。
      "柱位 TF 到 %s 失败: %s —— 本次按'柱位不可得'处理（判未知）。"
      "⚠️ 常见原因: 里程计/Point-LIO 没起、TF 树没通、或 %s 这个 frame 没人发",
      world_frame_.c_str(), ex.what(), msg->header.frame_id.c_str());
    pillar_top_.reset();
    return;
  }

  // 柱顶 = 变换后的柱脚 + 柱高（沿 **world 的 Z** 抬 —— 柱子是垂直的）
  out.point.z += static_cast<double>(p.height);
  pillar_top_ = out.point;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 单路判定
// ═══════════════════════════════════════════════════════════════════════════════
MustikaPillarCheck::CameraVerdict MustikaPillarCheck::judge_one(
    size_t cam, const std::optional<geometry_msgs::msg::Point>& pillar_top)
{
  CameraVerdict v;
  if (cam >= kCamCount) { v.note = "相机下标非法"; return v; }

  // ⚠️⚠️ **雷达那一路也要判新鲜度**（2026-09-23 编译复核抓出）：
  //    原实现只判相机（下面 `stale_sec_`），`lidar_at_` **写了从不读**（零次）。
  //    后果：雷达节点一旦停发（或它那条 TF 链路只断一段），`pillar_top_` 会被
  //    **永久保留**，本节点照旧判"在柱上"并 2Hz 一直发 —— 而**消费者那侧的兜底
  //    救不了它**：`perception_pipeline` 的超时量的是"本节点消息的到达时刻"，
  //    本节点一直在发新消息 ⇒ 超时永不触发，`mustika_state = 1` 被一直保持。
  //    ⚠️ hpp 里 `stale_sec_` 的注释本来就写着"相机**/雷达**的消息超过它就当没有"
  //       —— 是**实现**漏了，不是注释写错。这里补上。
  if (tp_unset(lidar_at_)) {
    v.note = "还没收到过雷达结果";
    return v;
  }
  if (std::chrono::duration<double>(mono_now() - lidar_at_).count() > stale_sec_) {
    v.note = "雷达结果不新鲜（超 stale_sec）—— 按'柱位不可得'处理，不拿旧柱位判";
    return v;
  }

  if (!pillar_top) { v.note = "雷达未给出柱位（未检出/未收到/TF 失败）"; return v; }

  const auto& m = last_ball_[cam];
  if (!m) { v.note = "这一路还没有数据"; return v; }

  const double age = std::chrono::duration<double>(mono_now() - ball_at_[cam]).count();
  if (tp_unset(ball_at_[cam]) || age > stale_sec_) {
    v.note = "数据不新鲜（" + std::to_string(age) + "s）";
    return v;
  }
  if (!m->detected) { v.note = "本路未检出球"; return v; }
  if (m->confidence < min_confidence_) { v.note = "置信度不足"; return v; }
  if (m->radius <= 0.0f) { v.note = "像素半径非正"; return v; }

  // ⚠️ **该路能用的最小像素半径** —— 由容差与噪声**推出来**，不是另拍一个数
  //    （头文件 `min_radius_px_` 的注释：`r_min = √(f·D·δr / (2·tol))`）。
  //    取两者较大值：配置里那个是"绝对下限"，推出来的是"这一路在这组容差下的下限"。
  const double r_min_derived = std::sqrt(
      focal_px_[cam] * ball_diameter_m_ * radius_noise_px_ / (2.0 * std::max(1e-6, pos_tolerance_m_)));
  const double r_min = std::max(min_radius_px_, r_min_derived);
  if (static_cast<double>(m->radius) < r_min) {
    v.note = "像素半径 " + std::to_string(m->radius) + "px < 该路可用下限 " +
             std::to_string(r_min) + "px（距离估计误差超出容差）";
    return v;
  }

  const auto rng = range_from_pixel_radius(m->radius, focal_px_[cam], ball_diameter_m_);
  if (!rng) { v.note = "距离反推失败（参数非法）"; return v; }
  v.range_m = *rng;

  double xyz[3];
  back_project(m->u, m->v, v.range_m, focal_px_[cam], focal_px_[cam],
               cx_px_[cam], cy_px_[cam], xyz);

  // ⚠️⚠️ **光学系 → 机身系，这一步不能省**（2026-09-23 复核抓出；原实现是错的）
  //
  //    `back_project` 给的是**光心系**：+X 右 / +Y 下 / +Z 前。
  //    而 `camera_frames_[cam]` 在 TF 树里是**机身系** ——
  //    `config/fusion_params.yaml` 的 `static_tf:` 块里它相对 `base_link` 的
  //    `rotation_rpy = [0,0,0]`，即与 base_link 同向 = **REP-103 的 +X 前 / +Y 左 / +Z 上**。
  //    直接把光心系的三个数填进机身系的 PointStamped ⇒ **整体绕 X 轴差 90°，且不报错**。
  //
  //    ⚠️ 为什么"标定时盯残差"救不了它：这 90° 是**约定**错配，不是外参误差 ——
  //       残差会稳定地大，而调外参**永远调不好**，会把标定引向错误方向。
  //    ⚠️ 全仓库 grep `optical` **零命中**，没有任何地方建立过光学系约定 ⇒
  //       这个转换只能在本节点做（它是唯一一个"产生三维点再查 TF"的节点）。
  //
  //    固定旋转（与安装位姿无关，标定只给安装位姿）：
  //        x_机身 =  z_光心    （前 = 前）
  //        y_机身 = -x_光心    （左 = -右）
  //        z_机身 = -y_光心    （上 = -下）
  const double x_body =  xyz[2];
  const double y_body = -xyz[0];
  const double z_body = -xyz[1];

  geometry_msgs::msg::PointStamped pin, pout;
  pin.header.frame_id = camera_frames_[cam];
  pin.header.stamp    = m->header.stamp;      // ⚠️ 曝光时刻，不是 now()
  pin.point.x = x_body; pin.point.y = y_body; pin.point.z = z_body;
  try {
    // ⚠️ 同上：与 `coordinate_transformer.cpp:324` 同型的写法 + 显式超时
    pout = tf_buffer_->transform(pin, world_frame_, tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException& ex) {
    v.note = std::string("球心 TF 失败: ") + ex.what();
    return v;
  }

  // ── 位置：**水平**残差（两条判据要互相独立，别把高度也混进来）──
  const double dx = pout.point.x - pillar_top->x;
  const double dy = pout.point.y - pillar_top->y;
  v.residual_m = std::sqrt(dx * dx + dy * dy);

  // ── 高度：球心应在「柱顶 + center_above_pillar_top」附近 ──
  const double expected_z = pillar_top->z + ball_center_above_pillar_top_m_;

  v.used = true;
  v.ball_z_world = pout.point.z;
  v.position_ok = (v.residual_m <= pos_tolerance_m_);
  v.height_ok   = (std::abs(pout.point.z - expected_z) <= height_tolerance_m_);

  char buf[192];
  std::snprintf(buf, sizeof(buf),
                "距离 %.2fm | 水平残差 %.2fm(限 %.2f) | 球心 z %.3f(期望 %.3f±%.2f)",
                v.range_m, v.residual_m, pos_tolerance_m_,
                pout.point.z, expected_z, height_tolerance_m_);
  v.note = buf;
  return v;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 汇总（合并规则写在头文件 `decide()` 的注释里）
// ═══════════════════════════════════════════════════════════════════════════════
uint8_t MustikaPillarCheck::decide(const std::array<CameraVerdict, kCamCount>& vs,
                                   std::string& why) const
{
  // ⚠️⚠️ **人工闸**（头文件边界②）：未标定 → **只发未知**。
  //    注意这里**不是"少发一个 1"**：残差诊断照发（那是标定要盯的数），
  //    只是**不允许产出可信结论**。
  if (!extrinsics_calibrated_) {
    why = "外参未标定 → 只发未知（诊断里有残差，标定完再翻 `extrinsics_calibrated`）";
    return kUnknown;
  }
  if (!pillar_top_) {
    why = "雷达未给出柱位 → 无从比对";
    return kUnknown;
  }

  const CameraVerdict& vw = vs[kCamWide];
  const CameraVerdict& vt = vs[kCamTele];

  // 「明确反对」= 算出了残差、且超出**反对门槛**。
  // ⚠️ 与「通过」（残差 ≤ `pos_tolerance_m_`）是**两个不同的门槛**，
  //    中间那段**既不通过也不反对** —— 那是滞回带，别把它当成任意一边。
  auto opposes = [&](const CameraVerdict& v) {
    return v.used && (!v.position_ok && v.residual_m > residual_max_m_);
  };
  auto passes = [](const CameraVerdict& v) { return v.used && v.position_ok && v.height_ok; };
  /// 「**正观测到球掉在地上**」—— 两条同时满足才算（头文件边界①）
  auto observes_fallen = [&](const CameraVerdict& v) {
    return v.used && v.residual_m > fallen_dist_m_ && v.ball_z_world < fallen_height_max_m_;
  };

  const bool any_pass   = passes(vw) || passes(vt);
  const bool any_fallen = observes_fallen(vw) || observes_fallen(vt);

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️⚠️ 掉落判定**必须排在规则①之前**（2026-09-23 复核抓出的真 bug）
  //
  //    `fallen_dist_m_`(0.8) **大于** `residual_max_m_`(0.5)，而"球离柱够远"
  //    这个前提本身就满足 `opposes()`（残差 > 0.5）⇒ **照原顺序，规则①会把
  //    这类输入先判成"未知"，掉落分支永远走不到**。
  //    （默认参数下穷举验过：12 组"球明明掉在地上"的输入**全部**返回未知。）
  //    `State::kFallen` 声明了却做不到 —— 就是**假能力**，与 :62 那句自己写的
  //    警告同源（上一版修过另一个原因：`CameraVerdict` 没带球心高度）。
  //
  //    ⚠️ 反过来**不能**简单地把掉落提到最前：头文件规则⑤要求
  //       "一路说'在柱上'、另一路说'在地上' ⇒ **矛盾**，判未知"。
  //       所以先判矛盾，再判掉落。
  // ═══════════════════════════════════════════════════════════════════════════
  if (any_fallen && any_pass) {
    why = "一路判'在柱上'、另一路判'在地上' → **矛盾**，判未知（规则⑤：那是矛盾，不是结论）";
    return kUnknown;
  }
  if (any_fallen) {
    for (const auto& v : {vw, vt}) {
      if (observes_fallen(v)) {
        why = std::string("**掉落**（正观测）：球心离柱 ") + std::to_string(v.residual_m) +
              "m 且高度仅 " + std::to_string(v.ball_z_world) + "m —— 看到了，不是没看到";
        return kFallen;
      }
    }
  }

  // 规则①：**任一路明确反对 → 不能判"在柱上"**
  if (opposes(vw) || opposes(vt)) {
    // ⚠️ 细分文案：残差大**且球心不低**时最可能是"球被 TR/BR 拿走了"——
    //    但结论**仍是未知**：本节点只看得见相机，**没看到它在地上**，
    //    不能把"没看到"升级成"被拿走了"（那是拿不确定当结论）。
    for (const auto& v : {vw, vt}) {
      if (opposes(v) && v.residual_m > fallen_dist_m_) {
        why = "球离柱 " + std::to_string(v.residual_m) + "m 且高度 " +
              std::to_string(v.ball_z_world) + "m 不低 → 判未知：**没看到它在地上**，"
              "可能是被 TR/BR 拿走了（本节点看不到那种情况）";
        return kUnknown;
      }
    }
    why = "有相机路明确反对（残差超 " + std::to_string(residual_max_m_) + "m）→ 判未知";
    return kUnknown;
  }

  // 规则③：两路都算出距离 → 一致性检查（**零成本的独立验证**）
  if (vw.used && vt.used && vw.range_m > 0.0 && vt.range_m > 0.0) {
    const double lo = std::min(vw.range_m, vt.range_m);
    const double rel = std::abs(vw.range_m - vt.range_m) / std::max(1e-6, lo);
    if (rel > cross_check_rel_) {
      // ⚠️ 相机之间都打架 → 标定或检测有问题 → **别硬判**
      why = "两路距离差 " + std::to_string(rel * 100.0) + "% 超一致性门槛 " +
            std::to_string(cross_check_rel_ * 100.0) + "%（相机之间都打架）→ 判未知";
      return kUnknown;
    }
  }

  // 规则②：至少一路通过（且已确认没有一路反对）
  if (passes(vw) || passes(vt)) {
    why = std::string("在柱上：") +
          (passes(vw) ? "宽角通过" : "") + (passes(vw) && passes(vt) ? " / " : "") +
          (passes(vt) ? "长焦通过" : "");
    return kOnPillar;
  }

  // ── 走到这里 = 没人通过、没人反对、也没人正观测到"掉在地上" ──
  //
  // ⚠️ 掉落（`kFallen`）**已在上面判过**，不在这里 —— 它必须抢在规则①之前，
  //    否则永远走不到（理由见上）。
  why = "没有任何一路通过，也没有明确反对 → 判未知"
        "（**不是「不在柱上」**：没看到的原因太多，一律判「不在」是拿不确定当结论）";
  return kUnknown;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 心跳：判定 → 发布状态 + 诊断
// ═══════════════════════════════════════════════════════════════════════════════
void MustikaPillarCheck::tick()
{
  // ⚠️ **两路判定只算一次**，判定与诊断共用同一份 —— 分开算的话，
  //    中间若来了新消息，诊断说的和判定用的就对不上了（同一次心跳里两套事实）。
  const std::array<CameraVerdict, kCamCount> vs = {
      judge_one(kCamWide, pillar_top_), judge_one(kCamTele, pillar_top_)};

  std::string why;
  const uint8_t state = decide(vs, why);

  std_msgs::msg::UInt8 out;
  out.data = state;
  state_pub_->publish(out);

  // ── 诊断：**把两路的中间量都摆出来** —— 标定时盯的就是这里 ──
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "mustika_pillar_check";
  st.hardware_id = "camera+mid360";

  // ⚠️ 等级：
  //    · 未标定 → WARN（**不是 ERROR**：这是"已知的、有意的"状态，不是故障）
  //    · 判出"在柱上"/"掉落" → OK
  //    · 其余（未知）→ OK —— ⚠️ **未知不是故障**：没看到球是常态
  st.level = extrinsics_calibrated_ ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                    : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  st.message = (state == kOnPillar) ? "在柱上"
             : (state == kFallen)   ? "**掉落**（正观测）"
                                    : "未知";

  auto kv = [](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; return p;
  };

  char buf[256];
  st.values.push_back(kv("extrinsics_calibrated", extrinsics_calibrated_ ? "1" : "0"));
  std::snprintf(buf, sizeof(buf), "%u (%s)", static_cast<unsigned>(state), why.c_str());
  st.values.push_back(kv("state", buf));
  // ⚠️ 雷达说"柱顶有东西"——**本节点不拿它做门限**（它是弱信号，漏检会让明明在柱上的球判不出来），
  //    但两路**不一致时要能看见**：相机说"在柱上"而雷达说"柱顶没东西"，
  //    说明其中一个错了（更可能是雷达的顶物检测漏了）。
  {
    const bool disagree = (state == kOnPillar) && !lidar_says_object_;
    st.values.push_back(kv("lidar_says_object_on_top",
        std::string(lidar_says_object_ ? "1" : "0") +
        (disagree ? "  [!] 与相机的'在柱上'结论冲突 —— 查雷达顶物检测" : "")));
  }
  st.values.push_back(kv("pillar_top_ok",
      pillar_top_ ? "1" : "0（未检出/未收到/TF 失败）"));

  // ⚠️ **把两边用到的坐标系与 TF 成败打出来**（头文件边界③的要求）——
  //    否则"一边 world 一边传感器系"这类错配在数字上**看不出来**。
  st.values.push_back(kv("frames", std::string("world=") + world_frame_ +
                                  " cam=" + camera_frames_[0] + "/" + camera_frames_[1] +
                                  " lidar=" + lidar_topic_));

  // 逐路：残差 / 距离 / 判定 —— **标定时盯 residual_m**
  // （`vs` 是上面算好的那一份，不再重算）
  const char* cam_name[kCamCount] = {"wide", "tele"};
  for (size_t i = 0; i < kCamCount; ++i) {
    const auto& v = vs[i];
    if (v.used) {
      std::snprintf(buf, sizeof(buf),
                    "used=1 range=%.2fm residual=%.3fm z=%.3fm pos_ok=%d h_ok=%d",
                    v.range_m, v.residual_m, v.ball_z_world,
                    v.position_ok ? 1 : 0, v.height_ok ? 1 : 0);
    } else {
      std::snprintf(buf, sizeof(buf), "used=0 (%s)", v.note.c_str());
    }
    st.values.push_back(kv(std::string("cam_") + cam_name[i], buf));
  }
  // ⚠️ 两路残差都列出来 —— **标定就是看这两个数往下掉**
  st.values.push_back(kv("residual_max_m", std::to_string(residual_max_m_)));

  arr.status.push_back(st);
  diag_pub_->publish(arr);
}

}  // namespace fusion
}  // namespace br_perception

// ═══════════════════════════════════════════════════════════════════════════════
// 组件注册（容器化时被容器加载; 也生成同名独立可执行）
// ═══════════════════════════════════════════════════════════════════════════════
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::fusion::MustikaPillarCheck)
