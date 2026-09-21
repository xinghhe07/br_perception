#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace br_perception {
namespace fusion {

// ═══════════════════════════════════════════════════════════════════════════
// 里程计健康状态 (§4.1 v1.5)
// ═══════════════════════════════════════════════════════════════════════════
enum class OdometryState : uint8_t {
  OK            = 0,   // 里程计正常
  DEGENERATE    = 1,   // Point-LIO 报告退化 (退化标志话题)
  IMU_VIBRATION = 2,   // IMU 加速度方差过大 (坡道振动 / 碰撞)
  TIMEOUT       = 3,   // 里程计超时 (数据链路断 / TF 断连)
};

/// 状态 → 字符串 (诊断输出 / 日志)
const char* to_string(OdometryState state);

// ═══════════════════════════════════════════════════════════════════════════
// 坐标变换参数 (config/fusion_params.yaml §1.5)
// ═══════════════════════════════════════════════════════════════════════════
struct TransformerParams {
  // ── 坐标系名称 (§1.5) ──
  std::string world_frame{"world"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_link"};
  std::string lidar_frame{"livox_frame"};
  std::string camera_wide_frame{"camera_wide_frame"};
  std::string camera_tele_frame{"camera_tele_frame"};

  // ── 输入话题 ──
  /// Point-LIO (ROS2 移植版) 里程计话题 — 注意该移植版发的是 /aft_mapped_to_init, 不是 /Odometry
  std::string odometry_topic{"/aft_mapped_to_init"};
  std::string imu_topic{"/livox/imu"};
  /// Point-LIO 退化标志话题 (std_msgs/Bool, true=退化); 空字符串 = 不订阅
  std::string degenerate_topic{""};

  // ── 可靠性判定 ──
  double odom_timeout_sec{0.5};        // 里程计超时阈值 (s)
  double tf_timeout_sec{0.05};         // TF lookup 超时 (s)
  double imu_variance_window_sec{1.0}; // IMU 方差统计窗口 (s)
  /// 加速度模长方差阈值 — 单位 = IMU 加速度原始单位².
  /// ⚠ Mid-360 原始输出为 g (实测静止 |a|≈0.97), 故单位为 g²;
  ///   0.25 g² ≈ 0.5g 抖动 — 现场按坡道实测标定 (该项写入 odometry_reliable)
  double imu_acc_variance_thresh{0.25};
  double confidence_decay{0.5};        // 退化时世界坐标置信度系数 (§1.5)
  bool   relative_mode_on_degraded{true};  // 退化时切相对定位模式 (§1.5)
  /// TF 广播限频 (Hz) — 里程计高带宽输出可达 kHz, 感知侧 TF 无需同频; ≤0 = 不限频
  double tf_publish_rate_hz{100.0};

  // ── BR_RESET 位姿注入 (§4.1 v1.6) ──
  double pose_prior_window_sec{5.0};   // 收到 BR_RESET 后 N 秒内注入 odom→world 初值

  /// 从节点参数加载 (config/fusion_params.yaml 的 frame / odometry.* 键)
  static TransformerParams load_from_node(rclcpp::Node* node);
};

// ═══════════════════════════════════════════════════════════════════════════
// 坐标统一器 — TF 树 + 里程计可靠性 (§4.1, 模块 17)
//
// TF 树 (§4.1):
//   world ← odom            (位姿校正 / BR_RESET 注入, 本模块广播)
//   odom  ← base_link       (Point-LIO 里程计, 本模块转播)
//   base_link ← livox_frame / camera_wide_frame / camera_tele_frame
//                           (静态安装外参, 由 bringup 的 static_transform_publisher 提供)
//
// 职责:
//   1. 把 Point-LIO 的里程计转成 odom←base_link TF, 并据此给出 world 位姿
//   2. 里程计可靠性判定 (§4.1 v1.5): 退化标志 + IMU 加速度方差 + 超时 → odometry_reliable
//   3. 不可靠时的降权 (× confidence_decay) 与相对定位模式
//   4. BR_RESET 位姿注入 (§4.1 v1.6): 启动区/重试区已知位置作为 odom→world 初值
//   5. 所有检测结果的统一戳记: frame_id = world + timestamp (§4.1)
//
// 用法 (在 fusion_node 中):
//   CoordinateTransformer transformer(this, TransformerParams::load_from_node(this));
//   ... 每个检测结果: transformer.stamp_world(header);
//       conf = transformer.apply_reliability(conf);
//       if (transformer.relative_mode()) { /* 用 relative_pose_delta() */ }
// ═══════════════════════════════════════════════════════════════════════════
class CoordinateTransformer
{
public:
  /// @param node 宿主节点 — 用于建订阅/广播器 (生命周期由调用方保证)
  explicit CoordinateTransformer(rclcpp::Node* node,
                                 const TransformerParams& params = TransformerParams());
  ~CoordinateTransformer();

  CoordinateTransformer(const CoordinateTransformer&) = delete;
  CoordinateTransformer& operator=(const CoordinateTransformer&) = delete;

  // ── 状态查询 ──────────────────────────────────────────────────────────

  /// 里程计是否可靠 (§4.1 v1.5; 输出到 PerceptionFrame.odometry_reliable)
  bool odometry_reliable() const;

  /// 当前健康状态 (OK / DEGENERATE / IMU_VIBRATION / TIMEOUT)
  OdometryState odometry_state() const;

  /// 世界坐标置信度系数: 可靠 = 1.0, 退化 = confidence_decay (0.5)
  float confidence_scale() const;

  /// 是否处于相对定位模式 (§4.1 v1.5: 退化时以帧间局部坐标差代替绝对 world)
  bool relative_mode() const;

  // ── 位姿 ──────────────────────────────────────────────────────────────

  /// 机器人当前 world 位姿 (2D), 供 PerceptionFrame.ego_pose 使用;
  /// 里程计未就绪 / 超时 → nullopt
  std::optional<geometry_msgs::msg::Pose2D> robot_pose_world() const;

  /// 相对定位模式下: 相对"最后可靠位姿"的局部位姿增量 (§4.1 v1.5)
  std::optional<geometry_msgs::msg::Pose2D> relative_pose_delta() const;

  // ── 坐标变换 ──────────────────────────────────────────────────────────

  /// 任意坐标系 → world (TF 查询, 失败返回 false 并置 TF 不可靠)
  bool transform_to_world(const geometry_msgs::msg::PointStamped& in,
                          geometry_msgs::msg::PointStamped& out);

  /// livox_frame 3D → world 3D (§4.1: 雷达检测结果直接 TF 变换)
  /// in.header.frame_id 为空时按 lidar_frame 处理
  bool lidar_to_world(const geometry_msgs::msg::PointStamped& in,
                      geometry_msgs::msg::PointStamped& out);

  /// TF 树完整性自检 (world→odom→base_link 是否连通)
  bool tf_tree_ok();

  /// TF 查询失败/超时 → true (§4.1: 处理 TF 树断连或延迟 → 标记坐标变换不可靠)
  bool tf_unreliable() const;

  // ── 结果统一戳记与降权 ────────────────────────────────────────────────

  /// 给检测结果打上 frame_id = world + 当前时间戳 (§4.1)
  void stamp_world(std_msgs::msg::Header& header) const;

  /// 按里程计可靠性降权置信度: 退化时 × confidence_decay (§4.1 v1.5)
  float apply_reliability(float confidence) const;

  // ── BR_RESET 位姿注入 (§4.1 v1.6) ─────────────────────────────────────

  /// 收到 BR_RESET 后, 把启动区/重试区已知位置 (zone_center, yaw_prior) 注入为
  /// odom→world 初值, 此后 world 位姿以此为基准递推
  /// @return 注入是否被接受 (里程计未就绪 → false)
  bool inject_pose_prior(double zone_x, double zone_y, double yaw_prior);

  /// 是否在位姿注入的有效窗口内 (pose_prior_window_sec)
  bool pose_prior_active() const;

  // ── 诊断 ──────────────────────────────────────────────────────────────
  std::string last_error() const;

private:
  // ── 回调 ──
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void degenerate_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // ── 内部 ──
  void  update_imu_variance(const sensor_msgs::msg::Imu& msg);
  void  publish_tf(bool force = false);
  void  log_state_transition();
  tf2::Transform world_correction() const;   // world←odom 校正 (含位姿注入)
  void  mark_error(const std::string& what);

  rclcpp::Node* node_{nullptr};
  TransformerParams params_;

  // TF
  std::unique_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener>    tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 订阅
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    sub_imu_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr      sub_degenerate_;

  // 里程计状态
  tf2::Transform last_odom_pose_{tf2::Transform::getIdentity()};
  rclcpp::Time   last_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool           has_odom_{false};
  bool           degenerate_flag_{false};

  // 相对定位模式: 进入退化前的最后可靠位姿 (odom 系, 作为局部坐标原点)
  std::optional<geometry_msgs::msg::Pose2D> last_reliable_pose_;

  // IMU 加速度方差 (滑动窗口)
  std::deque<std::pair<rclcpp::Time, double>> imu_acc_norm_window_;
  double imu_acc_variance_{0.0};

  // BR_RESET 位姿注入
  bool           pose_prior_valid_{false};
  tf2::Transform pose_prior_correction_{tf2::Transform::getIdentity()};
  rclcpp::Time   pose_prior_stamp_{0, 0, RCL_ROS_TIME};

  // TF 健康与限频
  bool          tf_unreliable_{false};
  rclcpp::Time  last_tf_publish_{0, 0, RCL_ROS_TIME};
  OdometryState last_logged_state_{OdometryState::TIMEOUT};
  std::string   last_error_;
};

}  // namespace fusion
}  // namespace br_perception
