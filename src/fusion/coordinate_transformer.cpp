#include "br_perception/fusion/coordinate_transformer.hpp"

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <stdexcept>

namespace br_perception {
namespace fusion {

const char* to_string(OdometryState state)
{
  switch (state) {
    case OdometryState::OK:            return "OK";
    case OdometryState::DEGENERATE:    return "DEGENERATE";
    case OdometryState::IMU_VIBRATION: return "IMU_VIBRATION";
    case OdometryState::TIMEOUT:       return "TIMEOUT";
  }
  return "UNKNOWN";
}

// ═══════════════════════════════════════════════════════════════════════════
// 参数加载 (config/fusion_params.yaml §1.5)
// ═══════════════════════════════════════════════════════════════════════════

namespace {
/// 已声明则读取, 否则按默认值声明 — 便于本模块独立跑测试节点
template <typename T>
T get_or_declare(rclcpp::Node* node, const std::string& name, const T& default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  return node->declare_parameter<T>(name, default_value);
}
}  // namespace

TransformerParams TransformerParams::load_from_node(rclcpp::Node* node)
{
  TransformerParams p;
  // 坐标系名称
  p.world_frame        = get_or_declare(node, "world_frame", p.world_frame);
  p.odom_frame         = get_or_declare(node, "odom_frame", p.odom_frame);
  p.base_frame         = get_or_declare(node, "robot_base_frame", p.base_frame);
  p.lidar_frame        = get_or_declare(node, "lidar_frame", p.lidar_frame);
  p.camera_wide_frame = get_or_declare(node, "camera_wide_frame", p.camera_wide_frame);
  p.camera_tele_frame  = get_or_declare(node, "camera_tele_frame", p.camera_tele_frame);
  // 里程计输入
  p.odometry_topic   = get_or_declare(node, "odometry.topic", p.odometry_topic);
  p.imu_topic        = get_or_declare(node, "odometry.imu_topic", p.imu_topic);
  p.degenerate_topic = get_or_declare(node, "odometry.degenerate_topic", p.degenerate_topic);
  // 可靠性判定
  p.odom_timeout_sec = get_or_declare(node, "odometry.timeout_sec", p.odom_timeout_sec);
  p.tf_timeout_sec   = get_or_declare(node, "odometry.tf_timeout_sec", p.tf_timeout_sec);
  p.imu_variance_window_sec = get_or_declare(node, "odometry.imu_variance_window_sec",
                                             p.imu_variance_window_sec);
  p.imu_acc_variance_thresh = get_or_declare(node, "odometry.imu_acc_variance_thresh",
                                             p.imu_acc_variance_thresh);
  p.confidence_decay = get_or_declare(node, "odometry.confidence_decay", p.confidence_decay);
  p.relative_mode_on_degraded = get_or_declare(node, "odometry.relative_mode_on_degraded",
                                               p.relative_mode_on_degraded);
  p.tf_publish_rate_hz = get_or_declare(node, "odometry.tf_publish_rate_hz",
                                        p.tf_publish_rate_hz);
  // BR_RESET 位姿注入
  p.pose_prior_window_sec = get_or_declare(node, "odometry.pose_prior_window_sec",
                                           p.pose_prior_window_sec);
  return p;
}

// ═══════════════════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════════════════

CoordinateTransformer::CoordinateTransformer(rclcpp::Node* node,
                                             const TransformerParams& params)
    : node_(node), params_(params)
{
  if (node_ == nullptr) {
    throw std::invalid_argument("CoordinateTransformer: node 不能为空");
  }

  tf_buffer_      = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_    = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  // 里程计 — Point-LIO (ROS2 移植版) 输出; 该移植版默认发 /aft_mapped_to_init
  // ⚠ QoS 取 BEST_EFFORT: best_effort 订阅者同时兼容 RELIABLE 与 BEST_EFFORT 发布者,
  //   是唯一"两种发布端都不会漏"的选择 (反向 RELIABLE 订阅 ↔ BEST_EFFORT 发布 = 收不到)。
  //   保留原来的队列深度 (keep_last), 只是把可靠性放宽。
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      params_.odometry_topic, rclcpp::SensorDataQoS().keep_last(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { odometry_callback(msg); });

  // IMU — 用于加速度方差判定 (坡道振动 / 碰撞)
  sub_imu_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      params_.imu_topic, rclcpp::SensorDataQoS().keep_last(50),
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); });

  // Point-LIO 退化标志 (可选)
  if (!params_.degenerate_topic.empty()) {
    sub_degenerate_ = node_->create_subscription<std_msgs::msg::Bool>(
        params_.degenerate_topic, rclcpp::SensorDataQoS().keep_last(10),
        [this](std_msgs::msg::Bool::SharedPtr msg) { degenerate_callback(msg); });
  }

  RCLCPP_INFO(node_->get_logger(),
              "坐标统一器就绪: %s ← %s(里程计 %s) ← %s ← {%s, %s, %s}",
              params_.world_frame.c_str(), params_.odom_frame.c_str(),
              params_.odometry_topic.c_str(), params_.base_frame.c_str(),
              params_.lidar_frame.c_str(), params_.camera_wide_frame.c_str(),
              params_.camera_tele_frame.c_str());
}

CoordinateTransformer::~CoordinateTransformer() = default;

// ═══════════════════════════════════════════════════════════════════════════
// 回调
// ═══════════════════════════════════════════════════════════════════════════

void CoordinateTransformer::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // 只取位姿; Point-LIO 的 header.frame_id (camera_init) 与 child_frame_id
  // (aft_mapped) 不直接采用 — 感知侧统一用 world/odom/base_link 命名 (§4.1)
  const auto& p = msg->pose.pose.position;
  const auto& q = msg->pose.pose.orientation;

  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  quat.normalize();
  last_odom_pose_ = tf2::Transform(quat, tf2::Vector3(p.x, p.y, p.z));

  last_odom_stamp_ = node_->now();
  has_odom_        = true;

  // 记录"最后可靠位姿" — 相对定位模式的局部坐标原点 (§4.1 v1.5)
  if (odometry_reliable()) {
    geometry_msgs::msg::Pose2D pose;
    pose.x     = p.x;
    pose.y     = p.y;
    pose.theta = tf2::getYaw(quat);
    last_reliable_pose_ = pose;
  }

  log_state_transition();
  publish_tf();
}

void CoordinateTransformer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  update_imu_variance(*msg);
}

void CoordinateTransformer::degenerate_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  degenerate_flag_ = msg->data;
  log_state_transition();
}

void CoordinateTransformer::log_state_transition()
{
  const OdometryState state = odometry_state();
  if (state == last_logged_state_) {
    return;
  }
  if (state == OdometryState::OK) {
    RCLCPP_INFO(node_->get_logger(), "里程计恢复可靠 (state: OK)");
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "里程计不可靠 (state: %s) → 世界坐标置信度 ×%.2f%s",
                to_string(state), params_.confidence_decay,
                relative_mode() ? ", 进入相对定位模式" : "");
  }
  last_logged_state_ = state;
}

// ═══════════════════════════════════════════════════════════════════════════
// 里程计可靠性 (§4.1 v1.5)
// ═══════════════════════════════════════════════════════════════════════════

void CoordinateTransformer::update_imu_variance(const sensor_msgs::msg::Imu& msg)
{
  const rclcpp::Time now = node_->now();
  const double acc_norm = std::sqrt(
      msg.linear_acceleration.x * msg.linear_acceleration.x +
      msg.linear_acceleration.y * msg.linear_acceleration.y +
      msg.linear_acceleration.z * msg.linear_acceleration.z);

  imu_acc_norm_window_.emplace_back(now, acc_norm);

  // 掉出窗口的样本
  while (!imu_acc_norm_window_.empty() &&
         (now - imu_acc_norm_window_.front().first).seconds() >
             params_.imu_variance_window_sec) {
    imu_acc_norm_window_.pop_front();
  }

  // 滑窗方差 (样本数 < 3 时置 0, 避免冷启动误判)
  const size_t n = imu_acc_norm_window_.size();
  if (n < 3) {
    imu_acc_variance_ = 0.0;
    return;
  }
  double mean = 0.0;
  for (const auto& s : imu_acc_norm_window_) {
    mean += s.second;
  }
  mean /= static_cast<double>(n);

  double var = 0.0;
  for (const auto& s : imu_acc_norm_window_) {
    const double d = s.second - mean;
    var += d * d;
  }
  imu_acc_variance_ = var / static_cast<double>(n);
}

OdometryState CoordinateTransformer::odometry_state() const
{
  if (!has_odom_) {
    return OdometryState::TIMEOUT;   // 里程计尚未就绪
  }
  if ((node_->now() - last_odom_stamp_).seconds() > params_.odom_timeout_sec) {
    return OdometryState::TIMEOUT;   // 数据链路断 / TF 断连
  }
  if (degenerate_flag_) {
    return OdometryState::DEGENERATE;
  }
  if (imu_acc_variance_ > params_.imu_acc_variance_thresh) {
    return OdometryState::IMU_VIBRATION;   // 坡道振动大
  }
  return OdometryState::OK;
}

bool CoordinateTransformer::odometry_reliable() const
{
  return odometry_state() == OdometryState::OK;
}

float CoordinateTransformer::confidence_scale() const
{
  return odometry_reliable() ? 1.0f : static_cast<float>(params_.confidence_decay);
}

bool CoordinateTransformer::relative_mode() const
{
  return params_.relative_mode_on_degraded && !odometry_reliable();
}

float CoordinateTransformer::apply_reliability(float confidence) const
{
  return confidence * confidence_scale();
}

// ═══════════════════════════════════════════════════════════════════════════
// 位姿查询
// ═══════════════════════════════════════════════════════════════════════════

tf2::Transform CoordinateTransformer::world_correction() const
{
  // world←odom: BR_RESET 位姿注入的校正量; 未注入时为单位阵
  // (纯里程计模式: world 与 odom 只在 BR_RESET 时刻对齐, 其余靠里程计递推)
  return pose_prior_valid_ ? pose_prior_correction_ : tf2::Transform::getIdentity();
}

std::optional<geometry_msgs::msg::Pose2D> CoordinateTransformer::robot_pose_world() const
{
  if (odometry_state() == OdometryState::TIMEOUT) {
    return std::nullopt;
  }
  const tf2::Transform world_base = world_correction() * last_odom_pose_;

  geometry_msgs::msg::Pose2D pose;
  pose.x     = world_base.getOrigin().x();
  pose.y     = world_base.getOrigin().y();
  pose.theta = tf2::getYaw(world_base.getRotation());
  return pose;
}

std::optional<geometry_msgs::msg::Pose2D> CoordinateTransformer::relative_pose_delta() const
{
  if (!has_odom_) {
    return std::nullopt;
  }
  // 当前 odom 位姿 − 最后可靠位姿: 局部坐标差, 代替绝对 world 坐标 (§4.1 v1.5)
  const double base_x   = last_reliable_pose_ ? last_reliable_pose_->x     : 0.0;
  const double base_y   = last_reliable_pose_ ? last_reliable_pose_->y     : 0.0;
  const double base_yaw = last_reliable_pose_ ? last_reliable_pose_->theta : 0.0;

  geometry_msgs::msg::Pose2D delta;
  delta.x     = last_odom_pose_.getOrigin().x() - base_x;
  delta.y     = last_odom_pose_.getOrigin().y() - base_y;
  delta.theta = tf2::getYaw(last_odom_pose_.getRotation()) - base_yaw;
  return delta;
}

// ═══════════════════════════════════════════════════════════════════════════
// 坐标变换 (§4.1)
// ═══════════════════════════════════════════════════════════════════════════

void CoordinateTransformer::mark_error(const std::string& what)
{
  last_error_    = what;
  tf_unreliable_ = true;
}

bool CoordinateTransformer::transform_to_world(const geometry_msgs::msg::PointStamped& in,
                                               geometry_msgs::msg::PointStamped& out)
{
  try {
    out = tf_buffer_->transform(in, params_.world_frame,
                                tf2::durationFromSec(params_.tf_timeout_sec));
    tf_unreliable_ = false;
    return true;
  } catch (const tf2::TransformException& ex) {
    // TF 树断连或延迟 → 标记坐标变换不可靠 (§4.1)
    mark_error(std::string("TF ") + in.header.frame_id + " → " +
               params_.world_frame + " 失败: " + ex.what());
    return false;
  }
}

bool CoordinateTransformer::lidar_to_world(const geometry_msgs::msg::PointStamped& in,
                                           geometry_msgs::msg::PointStamped& out)
{
  geometry_msgs::msg::PointStamped lidar_point = in;
  if (lidar_point.header.frame_id.empty()) {
    lidar_point.header.frame_id = params_.lidar_frame;
  }
  return transform_to_world(lidar_point, out);
}

bool CoordinateTransformer::tf_tree_ok()
{
  if (!has_odom_) {
    return false;
  }
  try {
    tf_buffer_->lookupTransform(params_.world_frame, params_.base_frame,
                                tf2::TimePointZero,
                                tf2::durationFromSec(params_.tf_timeout_sec));
    tf_unreliable_ = false;
    return true;
  } catch (const tf2::TransformException& ex) {
    mark_error(std::string("TF 树不完整: ") + ex.what());
    return false;
  }
}

bool CoordinateTransformer::tf_unreliable() const
{
  return tf_unreliable_;
}

std::string CoordinateTransformer::last_error() const
{
  return last_error_;
}

// ═══════════════════════════════════════════════════════════════════════════
// 统一戳记
// ═══════════════════════════════════════════════════════════════════════════

void CoordinateTransformer::stamp_world(std_msgs::msg::Header& header) const
{
  header.frame_id = params_.world_frame;
  header.stamp    = node_->now();
}

// ═══════════════════════════════════════════════════════════════════════════
// BR_RESET 位姿注入 (§4.1 v1.6)
// ═══════════════════════════════════════════════════════════════════════════

bool CoordinateTransformer::inject_pose_prior(double zone_x, double zone_y, double yaw_prior)
{
  if (!has_odom_) {
    RCLCPP_WARN(node_->get_logger(),
                "BR_RESET 位姿注入被拒: 里程计尚未就绪, 无法求 odom→world 校正");
    return false;
  }

  // 已知世界位姿 T_world_base_prior = (zone_center, yaw_prior)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_prior);
  const tf2::Transform world_base_prior(q, tf2::Vector3(zone_x, zone_y, 0.0));

  // 校正量: T_world_odom = T_world_base_prior · (T_odom_base)⁻¹
  pose_prior_correction_ = world_base_prior * last_odom_pose_.inverse();
  pose_prior_stamp_      = node_->now();
  pose_prior_valid_      = true;

  RCLCPP_INFO(node_->get_logger(),
              "BR_RESET 位姿注入: 场地先验 (%.3f, %.3f, %.1f°) → odom→world 校正已建立",
              zone_x, zone_y, yaw_prior * 180.0 / M_PI);
  publish_tf(/*force=*/true);   // 校正变化立即生效, 不受限频影响
  return true;
}

bool CoordinateTransformer::pose_prior_active() const
{
  return pose_prior_valid_ &&
         (node_->now() - pose_prior_stamp_).seconds() <= params_.pose_prior_window_sec;
}

// ═══════════════════════════════════════════════════════════════════════════
// TF 广播
// ═══════════════════════════════════════════════════════════════════════════

void CoordinateTransformer::publish_tf(bool force)
{
  if (!has_odom_) {
    return;
  }

  // 限频: 里程计高带宽输出 (可达 kHz) 时, 感知侧 TF 无需同频 (§4.1)
  const rclcpp::Time now = node_->now();
  if (!force && params_.tf_publish_rate_hz > 0.0) {
    const double min_interval = 1.0 / params_.tf_publish_rate_hz;
    if ((now - last_tf_publish_).seconds() < min_interval) {
      return;
    }
  }
  last_tf_publish_ = now;

  // world ← odom (校正量; 无注入时为单位阵)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = last_odom_stamp_;
    tf_msg.header.frame_id = params_.world_frame;
    tf_msg.child_frame_id  = params_.odom_frame;
    tf_msg.transform       = tf2::toMsg(world_correction());
    tf_broadcaster_->sendTransform(tf_msg);
  }

  // odom ← base_link (Point-LIO 里程计转播)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = last_odom_stamp_;
    tf_msg.header.frame_id = params_.odom_frame;
    tf_msg.child_frame_id  = params_.base_frame;
    tf_msg.transform       = tf2::toMsg(last_odom_pose_);
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

}  // namespace fusion
}  // namespace br_perception
