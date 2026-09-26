#include "br_perception/lidar/lidar_perception_node.hpp"

// PCL
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>

// ⚠️ TF：把输入点云从雷达系搬到场地系（见头文件「坐标系纪律」那段）。
//    ⚠️ 只用 `lookupTransform` + 自己组 `Eigen::Affine3f` —— **不需要 `tf2_geometry_msgs`**
//       （那个包是为 `Buffer::transform<PointStamped>` 的重载准备的，这里不走那条路）。
//       旋转直接用四元数构造，省一层转换、也少一个编译期依赖。
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <random>
#include <string>
#include <tuple>
#include <unordered_set>
#include <thread>

namespace br_perception {
namespace lidar {

using ms_dur = std::chrono::duration<float, std::milli>;

// ═══════════════════════════════════════════════════════════════════════════
// 辅助函数
// ═══════════════════════════════════════════════════════════════════════════
namespace {

const char* spot_status_name(BuildingSpotStatus s) {
  switch (s) {
    case BuildingSpotStatus::EMPTY:          return "Empty";
    case BuildingSpotStatus::ONE_EARTH:      return "One Earth";
    case BuildingSpotStatus::TWO_EARTH:      return "Two Earth";
    case BuildingSpotStatus::COMPLETE_TOWER: return "Complete Tower";
    // ⚠️ **不能落进 `default` 印成 "Empty"** —— 那正是"用 0 冒充未知"的同一个错。
    case BuildingSpotStatus::UNKNOWN:        return "**UNKNOWN(没量成)**";
    default:                                  return "??(未定义的 status)";
  }
}

const char* obstacle_type_name(ObstacleType t) {
  switch (t) {
    case ObstacleType::EARTH_CUBE:   return "Earth Cube";
    case ObstacleType::SKY_CUBE:     return "Sky Cube";
    case ObstacleType::ENEMY_ROBOT:  return "Enemy Robot";
    case ObstacleType::MUSTIKA:      return "Mustika";
    default:                         return "Unknown";
  }
}

}  // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════
// 构造 & 参数初始化
// ═══════════════════════════════════════════════════════════════════════════
LidarPerceptionNode::LidarPerceptionNode(const rclcpp::NodeOptions& options)
  : Node("lidar_perception_node", options)
{
  // ── 预处理参数 ──
  this->declare_parameter<double>("leaf_size", leaf_size_);
  this->declare_parameter<bool>("enable_outlier_filter", enable_outlier_filter_);
  this->declare_parameter<int>("outlier_mean_k", outlier_mean_k_);
  this->declare_parameter<double>("outlier_std_thresh", outlier_std_thresh_);
  this->declare_parameter<double>("roi_min_x", roi_min_x_);
  this->declare_parameter<double>("roi_max_x", roi_max_x_);
  this->declare_parameter<double>("roi_min_y", roi_min_y_);
  this->declare_parameter<double>("roi_max_y", roi_max_y_);
  this->declare_parameter<double>("roi_min_z", roi_min_z_);
  this->declare_parameter<double>("roi_max_z", roi_max_z_);

  // ── 地面分割参数 ──
  this->declare_parameter<double>("ransac_distance_threshold", ransac_distance_threshold_);
  this->declare_parameter<int>("ransac_max_iterations", ransac_max_iterations_);
  this->declare_parameter<double>("ransac_eps_angle", ransac_eps_angle_);
  this->declare_parameter<bool>("enable_normal_constraint", enable_normal_constraint_);
  this->declare_parameter<double>("normal_angle_threshold", normal_angle_threshold_);
  this->declare_parameter<double>("l0_height", l0_height_);
  this->declare_parameter<double>("l1_height", l1_height_);
  this->declare_parameter<double>("l2_height", l2_height_);
  this->declare_parameter<double>("layer_tolerance", layer_tolerance_);

  // ── 圆柱检测参数 ──
  this->declare_parameter<int>("normal_k_search", normal_k_search_);
  this->declare_parameter<double>("slice_thickness", slice_thickness_);
  this->declare_parameter<int>("slice_min_points", slice_min_points_);
  this->declare_parameter<double>("circle_fit_thresh", circle_fit_thresh_);
  this->declare_parameter<double>("radius_min", radius_min_);
  this->declare_parameter<double>("radius_max", radius_max_);
  this->declare_parameter<double>("merge_center_tol", merge_center_tol_);
  this->declare_parameter<double>("merge_radius_tol", merge_radius_tol_);
  this->declare_parameter<int>("merge_min_slices", merge_min_slices_);
  this->declare_parameter<double>("min_cyl_height", min_cyl_height_);
  this->declare_parameter<double>("cyl3d_dist_thresh", cyl3d_dist_thresh_);
  this->declare_parameter<int>("cyl3d_max_iters", cyl3d_max_iters_);
  this->declare_parameter<double>("cyl3d_normal_weight", cyl3d_normal_weight_);
  this->declare_parameter<double>("cyl3d_axis_eps", cyl3d_axis_eps_);
  this->declare_parameter<double>("mustika_h_min", mustika_h_min_);
  this->declare_parameter<double>("mustika_h_max", mustika_h_max_);
  this->declare_parameter<double>("core_h_min", core_h_min_);
  this->declare_parameter<double>("core_h_max", core_h_max_);

  // ── 预期柱子几何 (从 field_geometry.yaml 读取) ──
  this->declare_parameter<std::vector<double>>("mustika_pillar.position", mustika_pillar_pos_);
  this->declare_parameter<double>("mustika_pillar.height", mustika_pillar_height_expected_);
  this->declare_parameter<double>("mustika_pillar.radius", mustika_pillar_radius_expected_);
  this->declare_parameter<std::vector<double>>("core_pillar.position", core_pillar_pos_);
  this->declare_parameter<double>("core_pillar.height", core_pillar_height_expected_);
  this->declare_parameter<double>("core_pillar.radius", core_pillar_radius_expected_);

  // ── 障碍物聚类参数 ──
  this->declare_parameter<double>("cluster_tolerance", cluster_tolerance_);
  this->declare_parameter<int>("min_cluster_size", min_cluster_size_);
  this->declare_parameter<int>("max_cluster_size", max_cluster_size_);
  this->declare_parameter<double>("max_object_size", max_object_size_);
  this->declare_parameter<double>("max_object_height", max_object_height_);
  this->declare_parameter<int>("min_object_points", min_object_points_);
  this->declare_parameter<double>("floating_z_min", floating_z_min_);
  this->declare_parameter<double>("size_tolerance", size_tolerance_);
  this->declare_parameter<double>("earth_cube_size", earth_cube_size_);
  this->declare_parameter<double>("sky_cube_size", sky_cube_size_);
  this->declare_parameter<double>("enemy_robot_size", enemy_robot_size_);
  this->declare_parameter<double>("mustika_size", mustika_size_);
  this->declare_parameter<double>("elongation_ratio", elongation_ratio_);
  this->declare_parameter<double>("sphere_eigen_tol", sphere_eigen_tol_);

  // ── 建筑位分析参数 ──
  this->declare_parameter<std::vector<double>>("building_spot_x");
  this->declare_parameter<std::vector<double>>("building_spot_y");
  this->declare_parameter<std::vector<double>>("building_spot_z");   // 可选: per-spot 平台高
  this->declare_parameter<double>("platform_z", platform_z_);
  this->declare_parameter<double>("spot_half_size", spot_half_size_);
  this->declare_parameter<double>("column_z_min", column_z_min_);
  this->declare_parameter<double>("column_z_max", column_z_max_);
  this->declare_parameter<int>("min_spot_points", min_spot_points_);
  this->declare_parameter<double>("top_surface_tolerance", top_surface_tolerance_);
  this->declare_parameter<int>("top_surface_min_points", top_surface_min_points_);
  this->declare_parameter<double>("empty_max_height", empty_max_height_);
  this->declare_parameter<double>("one_earth_min", one_earth_min_);
  this->declare_parameter<double>("one_earth_max", one_earth_max_);
  this->declare_parameter<double>("two_earth_min", two_earth_min_);
  this->declare_parameter<double>("two_earth_max", two_earth_max_);
  this->declare_parameter<double>("complete_tower_min", complete_tower_min_);
  this->declare_parameter<double>("complete_tower_max", complete_tower_max_);

  // ── 坐标系（2026-09-24 新增）──
  // ⚠️ 这两个键**必须与融合层的 `fusion_params.yaml` 一致**：名字对不上 = 查不到 TF
  //    = 全帧 UNKNOWN（**不报错、只是什么都检测不到**，见头文件「坐标系纪律」）。
  this->declare_parameter<std::string>("world_frame", world_frame_);
  this->declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);

  // ── 中间结果话题开关（2026-09-24 新增；语义见调用处那段）──
  //   ⚠️ 与 `debug_enabled` **不是一回事**：那个管"给人看的东西"，
  //      这个管"组件化拓扑的数据接口有没有人消费"。默认 **true**（发）。
  this->declare_parameter<bool>("publish_intermediate_topics", publish_intermediate_topics_);

  // ── 调试 & 看门狗参数 ──
  this->declare_parameter<bool>("debug_enabled", debug_enabled_);
  this->declare_parameter<double>("watchdog_timeout", watchdog_timeout_);
  this->declare_parameter<int>("max_skip_frames", max_skip_frames_);

  // ── 读取参数 ──
  #define GET(n, v) this->get_parameter(n, v)
  GET("world_frame", world_frame_);
  GET("tf_timeout_sec", tf_timeout_sec_);
  GET("publish_intermediate_topics", publish_intermediate_topics_);
  GET("leaf_size", leaf_size_);
  GET("enable_outlier_filter", enable_outlier_filter_);
  GET("outlier_mean_k", outlier_mean_k_);
  GET("outlier_std_thresh", outlier_std_thresh_);
  GET("roi_min_x", roi_min_x_); GET("roi_max_x", roi_max_x_);
  GET("roi_min_y", roi_min_y_); GET("roi_max_y", roi_max_y_);
  GET("roi_min_z", roi_min_z_); GET("roi_max_z", roi_max_z_);
  GET("ransac_distance_threshold", ransac_distance_threshold_);
  GET("ransac_max_iterations", ransac_max_iterations_);
  GET("ransac_eps_angle", ransac_eps_angle_);
  GET("enable_normal_constraint", enable_normal_constraint_);
  GET("normal_angle_threshold", normal_angle_threshold_);
  GET("l0_height", l0_height_); GET("l1_height", l1_height_);
  GET("l2_height", l2_height_); GET("layer_tolerance", layer_tolerance_);
  GET("normal_k_search", normal_k_search_);
  GET("slice_thickness", slice_thickness_);
  GET("slice_min_points", slice_min_points_);
  GET("circle_fit_thresh", circle_fit_thresh_);
  GET("radius_min", radius_min_); GET("radius_max", radius_max_);
  GET("merge_center_tol", merge_center_tol_);
  GET("merge_radius_tol", merge_radius_tol_);
  GET("merge_min_slices", merge_min_slices_);
  GET("min_cyl_height", min_cyl_height_);
  GET("cyl3d_dist_thresh", cyl3d_dist_thresh_);
  GET("cyl3d_max_iters", cyl3d_max_iters_);
  GET("cyl3d_normal_weight", cyl3d_normal_weight_);
  GET("cyl3d_axis_eps", cyl3d_axis_eps_);
  GET("mustika_h_min", mustika_h_min_); GET("mustika_h_max", mustika_h_max_);
  GET("core_h_min", core_h_min_); GET("core_h_max", core_h_max_);

  // ── 预期柱子几何 ──
  this->get_parameter("mustika_pillar.position", mustika_pillar_pos_);
  this->get_parameter("mustika_pillar.height", mustika_pillar_height_expected_);
  this->get_parameter("mustika_pillar.radius", mustika_pillar_radius_expected_);
  this->get_parameter("core_pillar.position", core_pillar_pos_);
  this->get_parameter("core_pillar.height", core_pillar_height_expected_);
  this->get_parameter("core_pillar.radius", core_pillar_radius_expected_);

  GET("cluster_tolerance", cluster_tolerance_);
  GET("min_cluster_size", min_cluster_size_);
  GET("max_cluster_size", max_cluster_size_);
  GET("max_object_size", max_object_size_);
  GET("max_object_height", max_object_height_);
  GET("min_object_points", min_object_points_);
  GET("floating_z_min", floating_z_min_);
  GET("size_tolerance", size_tolerance_);
  GET("earth_cube_size", earth_cube_size_);
  GET("sky_cube_size", sky_cube_size_);
  GET("enemy_robot_size", enemy_robot_size_);
  GET("mustika_size", mustika_size_);
  GET("elongation_ratio", elongation_ratio_);
  GET("sphere_eigen_tol", sphere_eigen_tol_);
  GET("platform_z", platform_z_);
  GET("spot_half_size", spot_half_size_);
  GET("column_z_min", column_z_min_); GET("column_z_max", column_z_max_);
  GET("min_spot_points", min_spot_points_);
  GET("top_surface_tolerance", top_surface_tolerance_);
  GET("top_surface_min_points", top_surface_min_points_);
  GET("empty_max_height", empty_max_height_);
  GET("one_earth_min", one_earth_min_); GET("one_earth_max", one_earth_max_);
  GET("two_earth_min", two_earth_min_); GET("two_earth_max", two_earth_max_);
  GET("complete_tower_min", complete_tower_min_);
  GET("complete_tower_max", complete_tower_max_);
  GET("debug_enabled", debug_enabled_);
  GET("watchdog_timeout", watchdog_timeout_);
  GET("max_skip_frames", max_skip_frames_);
  #undef GET

  // 建筑位坐标
  {
    std::vector<double> bx, by;
    this->get_parameter("building_spot_x", bx);
    this->get_parameter("building_spot_y", by);
    if (bx.size() == by.size()) {
      building_positions_.reserve(bx.size());
      for (size_t i = 0; i < bx.size(); ++i)
        building_positions_.emplace_back(bx[i], by[i]);
    } else if (!bx.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "building_spot_x/y length mismatch (%zu vs %zu)", bx.size(), by.size());
    }

    // 可选: per-spot 平台高度 (L1=0.6 / L2=0.9)。长度不符则整组忽略, 回落 platform_z
    std::vector<double> bz;
    this->get_parameter("building_spot_z", bz);
    if (bz.size() == building_positions_.size()) {
      building_spot_zs_ = std::move(bz);
    } else if (!bz.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "building_spot_z length (%zu) != spots (%zu); falling back to platform_z=%.2f",
        bz.size(), building_positions_.size(), platform_z_);
    }
  }

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&LidarPerceptionNode::on_param_change, this, std::placeholders::_1));

  // ═══════════════════════════════════════════════════════════════════════
  // TF：**只读**（把点云从雷达系搬到场地系；见头文件「坐标系纪律」）
  // ⚠️ 用 `tf2_ros` 而不是 `fusion::CoordinateTransformer`：后者会让雷达层反向依赖
  //    融合层，而且它构造时会**无条件广播** TF（多一个广播者就撞 `TF_REPEATED_DATA`）。
  // ⚠️ 建在这里（订阅之前）—— 第一帧点云到达时监听器就已经在收 TF 了。
  // ═══════════════════════════════════════════════════════════════════════
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── 订阅: 原始雷达点云 ──
  // ⚠ QoS 必须 BEST_EFFORT: 仿真里这个话题由 Gazebo ray 插件直接发布 (BEST_EFFORT),
  //   RELIABLE 订阅者 ↔ BEST_EFFORT 发布者 = 不兼容, 静默收不到。best_effort 订阅
  //   同时兼容实机 (驱动/桥接的 RELIABLE 发布), 故两头都不会漏。
  sub_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/livox/lidar/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&LidarPerceptionNode::cloud_callback, this, std::placeholders::_1));

  // ── 中间结果发布 (使用 RELIABLE QoS 确保与 rviz2 兼容) ──
  pub_filtered_   = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                      "/perception/lidar/filtered", rclcpp::SystemDefaultsQoS());
  pub_ground_     = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                      "/perception/lidar/ground", rclcpp::SystemDefaultsQoS());
  pub_non_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                      "/perception/lidar/non_ground", rclcpp::SystemDefaultsQoS());
  pub_l0_surface_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                      "/perception/lidar/l0_surface", rclcpp::SystemDefaultsQoS());
  pub_l1_surface_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                      "/perception/lidar/l1_surface", rclcpp::SystemDefaultsQoS());
  pub_l2_surface_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                      "/perception/lidar/l2_surface", rclcpp::SystemDefaultsQoS());
  pub_plane_      = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                      "/perception/lidar/ground_plane", 10);
  pub_pillars_    = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                      "/perception/lidar/pillars", 10);
  pub_obstacles_  = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                      "/perception/lidar/obstacles", 10);
  pub_spots_      = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                      "/perception/lidar/building_spots", 10);

  // ── 统一输出 ──
  // ⚠️ 2026-09-22: 此前这里发的是 `DiagnosticArray`（给人看的计数文本），
  //    而 `LidarResult.msg` 从 2026-09-18 建好起**一直零引用** —— 机器可读的
  //    雷达接口实际上不存在。现改为发真正的数据接口（`LidarResult.msg` 头注释
  //    写明它就是"替代拿 DiagnosticArray 当数据接口的做法"）。
  //    原 DiagnosticArray 的内容一项没丢，全部搬到了 `/perception/lidar/diagnostics`。
  //    ⚠️ QoS 保持 RELIABLE(10)：它是包内数据接口（容器内/同机），不是传感器原始流，
  //       丢一帧雷达结果会让下游等下一次，比传感器图像严重。
  pub_result_      = this->create_publisher<br_perception::msg::LidarResult>(
                      "/perception/lidar/result", 10);
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                      "/perception/lidar/diagnostics", 10);
  pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                      "/perception/lidar/debug_cloud", rclcpp::SystemDefaultsQoS());

  // ── 看门狗定时器 (雷达断连检测) ──
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(watchdog_timeout_),
    std::bind(&LidarPerceptionNode::radar_watchdog_timer_callback, this));

  // ── 坐标系：**说在最显眼的地方**（它是"整个模块有没有在工作"的总开关）──
  tf_timeout_sec_ = std::max(0.001, std::min(tf_timeout_sec_, 1.0));
  if (world_frame_.empty()) {
    // ⚠️ 空串 = 明确"不换系"。那时本模块在**车不在原点时恒报错值**（见头文件那段），
    //    所以这条必须响 —— 它是配置错误，不是模式选择。
    RCLCPP_WARN(this->get_logger(),
      "**world_frame 为空 ⇒ 本节点不做坐标变换** —— 而本模块的常量"
      "(l0/l1/l2_height、roi、building_spot_*) **全是场地系的值**。"
      "后果: 地面分层全空 + 建筑位裁到空气 + 对外坐标与 frame_id 不符。"
      "⚠️ 只有在「输入点云本来就在场地系」时才允许这么配。");
  } else {
    RCLCPP_INFO(this->get_logger(),
      "坐标系: 点云将变换到 **%s**（TF 超时 %.3fs）。⚠️ 若该系无人广播"
      "（`perception_pipeline` 未起），本节点会**全帧报 UNKNOWN**，且不报错。",
      world_frame_.c_str(), tf_timeout_sec_);
  }

  RCLCPP_INFO(this->get_logger(),
    "LidarPerceptionNode ready | "
    "leaf=%.3f roi=%+.1f..%+.1f | ransac_th=%.3f iters=%d | "
    "cyl: slice=%.2f radius=[%.2f,%.2f] | "
    "cluster: tol=%.3f min=%d max=%d | "
    "spots=%zu | 中间话题=%s 调试输出=%s watchdog=%.1fs | "
    "mustika_pillar: pos=[%.1f,%.1f] h=%.2f r=%.3f | "
    "core_pillar: pos=[%.1f,%.1f] h=%.2f r=%.3f",
    leaf_size_, roi_min_x_, roi_max_x_,
    ransac_distance_threshold_, ransac_max_iterations_,
    slice_thickness_, radius_min_, radius_max_,
    cluster_tolerance_, min_cluster_size_, max_cluster_size_,
    building_positions_.size(),
    publish_intermediate_topics_ ? "ON(默认)" : "**OFF**",
    debug_enabled_ ? "ON" : "OFF(比赛默认)", watchdog_timeout_,
    mustika_pillar_pos_.size()>=2 ? mustika_pillar_pos_[0] : -1.0,
    mustika_pillar_pos_.size()>=2 ? mustika_pillar_pos_[1] : -1.0,
    mustika_pillar_height_expected_, mustika_pillar_radius_expected_,
    core_pillar_pos_.size()>=2 ? core_pillar_pos_[0] : -1.0,
    core_pillar_pos_.size()>=2 ? core_pillar_pos_[1] : -1.0,
    core_pillar_height_expected_, core_pillar_radius_expected_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 主回调 — 全管线编排
// ═══════════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════════
// [0] 点云 → 场地系（理由与后果见头文件「坐标系纪律」那段）
//
// ⚠️ 三个"不"：
//    · 读不到 TF 时**不猜**（不回落身份变换 —— 那正好把坑重新踩一遍）
//    · 已经在场地系时**不拷贝**（`out = in`，让常见情形零成本）
//    · 时间戳**用点云那一帧的**，不用 `now()`（TF 是按时戳插值的）
// ═══════════════════════════════════════════════════════════════════════════
bool LidarPerceptionNode::cloud_to_world(const PointCloudPtr& in,
                                         const std_msgs::msg::Header& hdr,
                                         PointCloudPtr& out)
{
  // 没配场地系（或配成空串）= 明确"不做变换"模式 ⇒ 原样透传，与旧行为一致
  if (world_frame_.empty()) { out = in; return true; }

  if (hdr.frame_id.empty()) {
    // ⚠️ 无从变换就**别猜**：`frame_id` 为空既可能"已在场系"也可能"根本没标"，
    //    而两者的后果天差地别 —— 按"已在场系"处理就是拿整条链去赌。
    //    （同 `BuildingSpot.msg` / `LidarResult.header` 里对空 frame_id 的处置。）
    return false;
  }

  // 已经在场地系里 —— 直接过（仿真里若上游已换好就是这条路）
  if (hdr.frame_id == world_frame_) { out = in; return true; }

  if (!tf_buffer_) return false;

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
        world_frame_, hdr.frame_id, hdr.stamp,
        tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException&) {
    return false;                      // 不回落、不猜 —— 交给调用方报 UNKNOWN
  }

  // ── 组装 Eigen 变换 ──
  //    ⚠️ `Eigen::Quaternionf(w, x, y, z)`，**w 在前**（ROS 消息里也是 w 在前）
  const auto& t = tf_msg.transform.translation;
  const auto& q = tf_msg.transform.rotation;
  Eigen::Affine3f T = Eigen::Affine3f::Identity();
  T.translate(Eigen::Vector3f(static_cast<float>(t.x),
                              static_cast<float>(t.y),
                              static_cast<float>(t.z)));
  T.rotate(Eigen::Quaternionf(static_cast<float>(q.w), static_cast<float>(q.x),
                              static_cast<float>(q.y), static_cast<float>(q.z)));

  out = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*in, *out, T);
  // ⚠️ 变换**不改变** header 的语义字段（时间戳、序号）—— 只改了系，
  //    而系的声明由调用方的 `world_header` 负责，这里不动它。
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// 一帧全报 UNKNOWN
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::publish_all_unknown(const std_msgs::msg::Header& header,
                                              const char* reason)
{
  LidarPerceptionResult result;
  result.stamp = this->now();
  result.ground_found    = false;
  result.radar_connected = radar_connected_;
  result.building_spots.reserve(building_positions_.size());

  for (size_t i = 0; i < building_positions_.size(); ++i) {
    BuildingSpot spot;
    spot.id         = static_cast<int>(i);
    spot.spot_x     = static_cast<float>(building_positions_[i].first);
    spot.spot_y     = static_cast<float>(building_positions_[i].second);
    spot.platform_z = spot_platform_z(i);
    // ⚠️⚠️ **UNKNOWN，不是 EMPTY** —— 这一行就是本文件最要紧的一个字。
    spot.status     = BuildingSpotStatus::UNKNOWN;
    spot.height     = 0.0f;                // 没量 → 0；⚠️ 别拿它当"高度是 0"
    spot.point_count = 0;
    result.building_spots.push_back(spot);
  }

  RCLCPP_DEBUG(this->get_logger(),
    "publish_all_unknown: %zu 个位报 UNKNOWN | 原因: %s",
    result.building_spots.size(), reason ? reason : "?");

  publish_unified_result(result, world_frame_.empty() ? header.frame_id : world_frame_);
}

void LidarPerceptionNode::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t0 = std::chrono::steady_clock::now();

  // ── 雷达在线标记（⚠️⚠️ **位置有讲究，别往下面挪**）──
  //   "收到过点云"是**雷达硬件活着**的证据，与 TF / 坐标系**无关** ——
  //   所以它必须排在 `cloud_to_world` 那条早退分支**之前**：换系失败是**感知层**的事，
  //   若因此不刷新 `last_cloud_time_`，看门狗会报"雷达断连" ——
  //   那是**把定位故障诊断成硬件故障**（同 §零「准」那条）。
  last_cloud_time_ = this->now();
  if (!radar_connected_) {
    radar_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Radar reconnected");
  }

  // ── 预备: 解包 (防御性 try-catch: pcl::fromROSMsg 可能因格式不匹配抛异常) ──
  //   ⚠️⚠️ **本步必须排在 [0] `cloud_to_world` 之前**（2026-09-24 复核修）：
  //      先前的写法是 `cloud_to_world(raw_cloud, ...)` 在前、解包在后 ——
  //      那时 `raw_cloud` **还没声明**（编译期 `use of undeclared identifier`），
  //      而且就算侥幸编过，搬去场地系的也是**一份空点云**。
  //      ⇒ 顺序定死: **解包 → 空帧判 → 搬场地系 → 之后全流程只认场地系**。
  //      （同一条链在 `building_spot_analyzer.cpp:231` 已是这个顺序，两处必须一致。）
  PointCloudPtr raw_cloud = std::make_shared<PointCloud>();
  try {
    pcl::fromROSMsg(*msg, *raw_cloud);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "pcl::fromROSMsg failed: %s", e.what());
    return;
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "pcl::fromROSMsg unknown exception");
    return;
  }

  // ── 空帧判（⚠️ 放在换系之前：空点云没有"系"可言，也不该因此报 UNKNOWN）──
  if (raw_cloud->empty()) {
    consecutive_empty_frames_++;
    if (consecutive_empty_frames_ >= max_skip_frames_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Empty point cloud (consecutive=%d)", consecutive_empty_frames_);
    }
    return;
  }
  consecutive_empty_frames_ = 0;

  // ── 年龄仪器（2026-09-26 加；语义见 `PipelineTiming` 的那段注释）──
  //    ⚠️ 放在**这里**（空帧判之后、换系之前）是有意的：
  //      · 空帧不算 —— 它没有"数据有多旧"可言，也不该推进 `prev_input_stamp_`；
  //      · 换系失败那条早退路径**要**带上年龄（那时更该知道数据有多旧），
  //        所以必须排在它**之前**。
  const rclcpp::Time in_stamp(msg->header.stamp);
  double in_period_ms = 0.0;
  if (prev_input_stamp_.nanoseconds() > 0) {
    in_period_ms = (in_stamp - prev_input_stamp_).seconds() * 1000.0;
  }
  prev_input_stamp_ = in_stamp;

  // ⚠️ `timing` 必须在**换系之前**就建好并填上年龄 —— 换系失败那条早退路径
  //    也要能发诊断（见下面 `publish_diagnostics(timing, ...)` 那段注释）。
  PipelineTiming timing;
  timing.input_points = static_cast<int>(raw_cloud->size());
  timing.input_stamp  = in_stamp;
  timing.period_ms    = in_period_ms;

  // ═══════════════════════════════════════════════════════════════════════
  // [0] **搬到场地系**（2026-09-24 立；理由见头文件「坐标系纪律」那段）
  //
  //   ⚠️⚠️ 这一步是**整条雷达链的前提**，不是优化：本节点所有常量
  //      （l0/l1/l2_height、roi_min_z、building_spot_x/y/z）都是**场地系**的值，
  //      而进来的是雷达系点云。不换系 ⇒ 地面分层全空 + 建筑位裁到空气 + 对外坐标说谎。
  //      它唯一的症状是"什么都没检测到"，**不报任何错**。
  //
  //   ⚠️ 失败时**不许继续**：换不成就是"这一帧无从判断"，报 UNKNOWN 让主控知道
  //      （"没量到"与"量到是空的"是两件事，见 `BuildingSpotStatus::UNKNOWN`）。
  // ═══════════════════════════════════════════════════════════════════════
  PointCloudPtr world_cloud;
  if (!cloud_to_world(raw_cloud, msg->header, world_cloud)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "**点云换不到场地系 '%s'**（输入 frame_id='%s'）⇒ 本帧全部建筑位报 UNKNOWN。"
      "⚠️ 常见原因: ① `perception_pipeline` 没起 ⇒ 没有人广播 world→odom；"
      "② 里程计还没出第一帧；③ `world_frame` 参数与实际 TF 树上的名字不符",
      world_frame_.c_str(), msg->header.frame_id.c_str());
    publish_all_unknown(msg->header, "point cloud not in world frame");
    // ⚠️⚠️ 2026-09-26 加：**早退路径也要发诊断**。
    //   原实现在这里直接 `return` ⇒ 只要换系一直失败（比如 `perception_pipeline`
    //   没起、没人广播 `world→odom`），`/perception/lidar/diagnostics`
    //   **一个发布者、一帧都不发** —— 而"每帧都换不过去"正是最该被看见的情况。
    //   实测踩到（本机 2026-09-26）：`bringup_lidar_only` 不起 `perception_pipeline`
    //   ⇒ 雷达链 100% 走这条早退 ⇒ 诊断话题**完全静默**，连"雷达在不在"都看不出来。
    //   ⇒ 至少把 `input_points` 与**年龄**报出去（那正是"数据有多旧"最要紧的时候）。
    publish_diagnostics(timing, LidarPerceptionResult{});
    return;
  }

  // 输出用 header：**时间戳照旧继承曝光/扫描时刻**，只把 frame_id 改成场地系。
  // ⚠️ 本帧之后所有发布（结果、诊断、调试点云、Marker）**都用它** ——
  //    点云与坐标都已经在场地系，标错系比不标更毒。
  // ⚠️ `world_frame_` 为空 = 本节点被配成"不换系"，那时沿用输入系（与旧行为一致）。
  std_msgs::msg::Header world_header = msg->header;
  if (!world_frame_.empty()) {
    world_header.frame_id = world_frame_;
  }
  const std::string& output_frame = world_header.frame_id;

  // ⚠️ `timing` 已在换系之前建好（那里要填年龄）—— 这里**不要再声明一次**。

  // ═══════════════════════════════════════════════════════════════════════
  // [1] 预处理（⚠️ 喂**场地系**那份 —— ROI 的 z 上下限是绝对高度）
  // ═══════════════════════════════════════════════════════════════════════
  auto [filtered, preproc_ms] = preprocess_stage(world_cloud);
  timing.preprocess_ms = preproc_ms;

  if (filtered->size() < 50) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Too few points after preprocess: %zu", filtered->size());
    publish_diagnostics(timing, LidarPerceptionResult{});
    return;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // [2] 地面分割
  // ═══════════════════════════════════════════════════════════════════════
  GroundResult gr = ground_segment_stage(filtered);
  timing.ground_segment_ms = gr.elapsed_ms;
  timing.ground_points     = static_cast<int>(gr.ground->size());
  timing.non_ground_points = static_cast<int>(gr.non_ground->size());

  if (!gr.success) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Ground segmentation failed — using last valid or default plane");
    // 使用上一次有效地面平面或默认水平面作为回退
    if (has_last_coeff_) {
      gr.coeff = last_valid_coeff_;
    }
  } else {
    last_valid_coeff_ = gr.coeff;
    has_last_coeff_   = true;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // [3] 圆柱检测 (需要法向量 → 转换 PointNormal)
  // ═══════════════════════════════════════════════════════════════════════
  PointNormalCloudPtr non_ground_normals = estimate_normals(gr.non_ground);
  auto [cylinders, cyl_ms] = cylinder_detect_stage(non_ground_normals);
  timing.cylinder_detect_ms = cyl_ms;

  // ═══════════════════════════════════════════════════════════════════════
  // [4] 障碍物聚类
  // ═══════════════════════════════════════════════════════════════════════
  auto [obstacles, cluster_ms] = cluster_extract_stage(gr.non_ground, cylinders);
  timing.cluster_extract_ms = cluster_ms;

  // ═══════════════════════════════════════════════════════════════════════
  // [5] 建筑位分析
  // ═══════════════════════════════════════════════════════════════════════
  auto [spots, bld_ms] = building_spot_stage(gr.non_ground);
  timing.building_spot_ms = bld_ms;

  // ═══════════════════════════════════════════════════════════════════════
  // 总耗时
  // ═══════════════════════════════════════════════════════════════════════
  auto t_end = std::chrono::steady_clock::now();
  timing.total_ms = ms_dur(t_end - t0).count();

  // ═══════════════════════════════════════════════════════════════════════
  // 发布中间结果 —— ⚠️ 2026-09-24 **按"有没有消费者"拆成两半**
  // ═══════════════════════════════════════════════════════════════════════
  // ⚠️⚠️ 2026-09-23 曾把这 10 条**整体**挂到 `debug_enabled` 下，依据是一张
  //     写"代码订阅者 0"的表。**2026-09-24 逐条实测后更正**——那张表**对了一半**：
  //
  //   话题                              真实代码订阅者（`create_subscription` 数出来的）
  //   /perception/lidar/filtered        **1** ground_segmenter
  //   /perception/lidar/non_ground      **3** building_spot_analyzer / cluster_extractor /
  //                                            cylinder_detector
  //   /perception/lidar/ground          0
  //   /perception/lidar/l0/l1/l2_surface 0
  //   /perception/lidar/ground_plane    0
  //   /perception/lidar/pillars         0
  //   /perception/lidar/obstacles       0
  //   /perception/lidar/building_spots  0
  //   /perception/lidar/debug_cloud     0
  //
  //   ⚠️ 实测方法也有教训：中间有一版用 `grep -l "<话题>" src/` 数，得出"9/10 条有订阅者"
  //      —— 那是错的，`grep -l` 把**发布者**也数进去了。**必须看 `create_subscription`
  //      那个调用块**。这条已写进 `scripts/check_topic_consumers.py` 的文件头。
  //
  //   ⇒ 结论：**2 条是数据接口**（组件的输入），**8 条是纯可视化**。两者的开关不是一回事：
  //      · 数据 ⇒ `publish_intermediate_topics_`（默认 true）：关掉会让**组件链饿死**，
  //        症状是"组件全都不出结果"，与"代码坏了"长得一样（而跑单体时**完全看不出来**）。
  //      · 可视化 ⇒ `debug_enabled_`（比赛 false）：关掉没人饿死，只是 RViz 里看不见。
  //   ⇒ 比赛模式因此只做 **2 次**整份点云拷贝（原来 6 次）—— 组件喂饱 + CPU 省下，两头都要。
  //
  //   ⇒ 纪律（2026-09-24 立，台账 §八 有账）:
  //      **把一个话题挂到任何开关下之前，先列出它的订阅者** ——
  //      `python3 scripts/check_topic_consumers.py`（它就是这条动作的自动化）。
  //      判据一句话: **数据话题挂拓扑开关，纯可视化才挂调试开关。**
  if (publish_intermediate_topics_) {
    publish_intermediate_data(filtered, gr, world_header);
  }
  if (debug_enabled_) {
    publish_intermediate_viz(gr, cylinders, obstacles, spots, world_header);
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 构建 & 发布统一结果
  // ═══════════════════════════════════════════════════════════════════════
  LidarPerceptionResult result;
  result.stamp = this->now();
  result.cylinders      = cylinders;
  result.obstacles       = obstacles;
  result.building_spots  = spots;
  result.l0_points = static_cast<int>(gr.l0_surface->size());
  result.l1_points = static_cast<int>(gr.l1_surface->size());
  result.l2_points = static_cast<int>(gr.l2_surface->size());
  result.ground_found    = gr.success;
  result.radar_connected = radar_connected_;
  result.timing          = timing;

  // 归一化平面系数
  normalize_coefficients(gr.coeff);
  result.ground_plane.resize(4);
  result.ground_plane[0] = static_cast<float>(gr.coeff.values[0]);
  result.ground_plane[1] = static_cast<float>(gr.coeff.values[1]);
  result.ground_plane[2] = static_cast<float>(gr.coeff.values[2]);
  result.ground_plane[3] = static_cast<float>(gr.coeff.values[3]);

  // ⚠️ frame_id = **场地系**（2026-09-24 改）。
  //    上面已经把点云搬到场地系了，所有结果（建筑位/柱/障碍的坐标）**真在那个系里** ——
  //    这正是第一宗旨「准」那条的原话："坐标真在 `header.frame_id` 声明的那个系里"。
  //    ⚠️ 老写法是继承输入点云的 frame_id，而坐标填的是**场地常量** ⇒ **消息在说谎**
  //       （表现为：下游拿 `livox_frame` 去变换，塔被投影到车前方 8m 的空气里）。
  // ⚠️ `world_frame_` 为空 = 本节点被配成"不做变换"，那时才回落到输入系（与旧行为一致）。
  publish_unified_result(result, output_frame);

  // ═══════════════════════════════════════════════════════════════════════
  // 调试点云
  // ═══════════════════════════════════════════════════════════════════════
  if (debug_enabled_) {
    publish_debug_cloud(gr.ground, gr.non_ground, cylinders,
                        obstacles, spots, world_header);
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 诊断
  // ═══════════════════════════════════════════════════════════════════════
  publish_diagnostics(timing, result);

  // ── 日志摘要 ──
  RCLCPP_DEBUG(this->get_logger(),
    "Pipeline: pre=%.1f gnd=%.1f cyl=%.1f clu=%.1f bld=%.1f | total=%.1f ms | "
    "cyl=%zu obs=%zu spots=%zu",
    timing.preprocess_ms, timing.ground_segment_ms,
    timing.cylinder_detect_ms, timing.cluster_extract_ms, timing.building_spot_ms,
    timing.total_ms,
    cylinders.size(), obstacles.size(), spots.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// [1] 预处理阶段
// ═══════════════════════════════════════════════════════════════════════════
std::pair<LidarPerceptionNode::PointCloudPtr, double>
LidarPerceptionNode::preprocess_stage(const PointCloudPtr& input)
{
  auto t0 = std::chrono::steady_clock::now();

  // 降采样
  auto cloud = std::make_shared<PointCloud>();
  {
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(static_cast<float>(leaf_size_),
                      static_cast<float>(leaf_size_),
                      static_cast<float>(leaf_size_));
    voxel.filter(*cloud);
  }

  // 离群点过滤
  if (enable_outlier_filter_ && cloud->size() >= static_cast<size_t>(outlier_mean_k_)) {
    auto filtered = std::make_shared<PointCloud>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(outlier_mean_k_);
    sor.setStddevMulThresh(outlier_std_thresh_);
    sor.setNegative(false);
    sor.filter(*filtered);
    cloud = filtered;
  }

  // ROI 裁剪
  {
    auto cropped = std::make_shared<PointCloud>();
    pcl::CropBox<pcl::PointXYZ> box;
    box.setInputCloud(cloud);
    box.setMin(Eigen::Vector4f(
      static_cast<float>(roi_min_x_), static_cast<float>(roi_min_y_),
      static_cast<float>(roi_min_z_), 1.0f));
    box.setMax(Eigen::Vector4f(
      static_cast<float>(roi_max_x_), static_cast<float>(roi_max_y_),
      static_cast<float>(roi_max_z_), 1.0f));
    box.setNegative(false);
    box.filter(*cropped);
    cloud = cropped;
  }

  auto t1 = std::chrono::steady_clock::now();
  return {cloud, ms_dur(t1 - t0).count()};
}

// ═══════════════════════════════════════════════════════════════════════════
// [2] 地面分割阶段
// ═══════════════════════════════════════════════════════════════════════════
LidarPerceptionNode::GroundResult
LidarPerceptionNode::ground_segment_stage(const PointCloudPtr& input)
{
  auto t0 = std::chrono::steady_clock::now();
  GroundResult gr;
  gr.ground     = std::make_shared<PointCloud>();
  gr.non_ground = std::make_shared<PointCloud>();
  gr.l0_surface = std::make_shared<PointCloud>();
  gr.l1_surface = std::make_shared<PointCloud>();
  gr.l2_surface = std::make_shared<PointCloud>();

  // ── RANSAC 平面拟合 ──
  pcl::PointIndices inliers;
  try {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_distance_threshold_);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setInputCloud(input);

    if (enable_normal_constraint_) {
      seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
      seg.setEpsAngle(ransac_eps_angle_);
    }

    seg.segment(inliers, gr.coeff);
    if (inliers.indices.empty()) {
      gr.elapsed_ms = ms_dur(std::chrono::steady_clock::now() - t0).count();
      return gr;
    }
  }
  catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "RANSAC segmentation exception: %s", e.what());
    gr.elapsed_ms = ms_dur(std::chrono::steady_clock::now() - t0).count();
    return gr;
  }

  // ── 法向量验证 ──
  if (!validate_plane_normal(gr.coeff)) {
    RCLCPP_DEBUG(this->get_logger(), "Ground plane tilted — using RANSAC result anyway");
  }

  // ── 动态阈值精筛 ──
  pcl::PointIndices refined = refine_inliers_adaptive(input, gr.coeff);

  // ── 分离地面 / 非地面 ──
  {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input);
    auto inliers_ptr = std::make_shared<pcl::PointIndices>(refined);
    extract.setIndices(inliers_ptr);
    extract.setNegative(false);
    extract.filter(*gr.ground);
    extract.setNegative(true);
    extract.filter(*gr.non_ground);
  }

  // ── 分层 ──
  classify_by_platform(gr.ground, gr.l0_surface, gr.l1_surface, gr.l2_surface);

  gr.success = true;
  gr.elapsed_ms = ms_dur(std::chrono::steady_clock::now() - t0).count();
  return gr;
}

// ═══════════════════════════════════════════════════════════════════════════
// [3] 圆柱检测阶段 (PointNormal 输入)
// ═══════════════════════════════════════════════════════════════════════════
std::pair<std::vector<CylinderDetection>, double>
LidarPerceptionNode::cylinder_detect_stage(const PointNormalCloudPtr& cloud)
{
  auto t0 = std::chrono::steady_clock::now();
  std::vector<CylinderDetection> out;

  if (cloud->size() < 20) {
    return {out, ms_dur(std::chrono::steady_clock::now() - t0).count()};
  }

  // ── 方法 A: 水平切片 + 2D 圆拟合 ──
  struct Slice { float z_center; std::vector<size_t> indices; };

  float z_min = cloud->points[0].z, z_max = cloud->points[0].z;
  for (const auto& p : cloud->points) {
    if (p.z < z_min) z_min = p.z;
    if (p.z > z_max) z_max = p.z;
  }
  const float dz = static_cast<float>(slice_thickness_);
  const int N = std::max(1, static_cast<int>((z_max - z_min) / dz + 1));
  std::vector<Slice> slices(N);
  for (int i = 0; i < N; ++i)
    slices[i].z_center = z_min + dz * i + dz * 0.5f;
  for (size_t pi = 0; pi < cloud->size(); ++pi) {
    int idx = static_cast<int>((cloud->points[pi].z - z_min) / dz);
    idx = std::clamp(idx, 0, N - 1);
    slices[idx].indices.push_back(pi);
  }

  // 圆拟合 + 合并
  std::mt19937 rng(42);
  struct Group { float sum_cx=0,sum_cy=0,sum_r=0,z_bot=0,z_top=0; int n=0; };
  std::vector<Group> groups;
  int cur_idx = -1;

  for (size_t i = 0; i < slices.size(); ++i) {
    const auto& sl = slices[i];
    if (static_cast<int>(sl.indices.size()) < slice_min_points_) {
      cur_idx = -1; continue;
    }

    // RANSAC 2D 圆拟合
    const size_t n = sl.indices.size();
    std::vector<float> xs(n), ys(n);
    for (size_t j = 0; j < n; ++j) {
      xs[j] = cloud->points[sl.indices[j]].x;
      ys[j] = cloud->points[sl.indices[j]].y;
    }

    float best_cx=0, best_cy=0, best_r=0;
    size_t best_cnt = 0;
    const float th_sq = circle_fit_thresh_ * circle_fit_thresh_;
    const float rmin_sq = radius_min_ * radius_min_;
    const float rmax_sq = radius_max_ * radius_max_;
    std::uniform_int_distribution<size_t> dist(0, n-1);

    for (int iter = 0; iter < 60; ++iter) {
      size_t i1=dist(rng), i2=dist(rng), i3=dist(rng);
      while (i2==i1) i2=dist(rng);
      while (i3==i1||i3==i2) i3=dist(rng);
      float d = 2.f*(xs[i1]*(ys[i2]-ys[i3])+xs[i2]*(ys[i3]-ys[i1])+xs[i3]*(ys[i1]-ys[i2]));
      if (std::abs(d) < 1e-9f) continue;
      float s1=xs[i1]*xs[i1]+ys[i1]*ys[i1], s2=xs[i2]*xs[i2]+ys[i2]*ys[i2], s3=xs[i3]*xs[i3]+ys[i3]*ys[i3];
      float cx=(s1*(ys[i2]-ys[i3])+s2*(ys[i3]-ys[i1])+s3*(ys[i1]-ys[i2]))/d;
      float cy=(s1*(xs[i3]-xs[i2])+s2*(xs[i1]-xs[i3])+s3*(xs[i2]-xs[i1]))/d;
      float rsq=(cx-xs[i1])*(cx-xs[i1])+(cy-ys[i1])*(cy-ys[i1]);
      if (rsq<rmin_sq||rsq>rmax_sq) continue;
      size_t cnt=0;
      for (size_t j=0;j<n;++j){float dx=xs[j]-cx,dy=ys[j]-cy; if(std::abs(std::sqrt(dx*dx+dy*dy)-std::sqrt(rsq))*std::abs(std::sqrt(dx*dx+dy*dy)-std::sqrt(rsq))<th_sq) cnt++;}
      if(cnt>best_cnt){best_cnt=cnt;best_cx=cx;best_cy=cy;best_r=std::sqrt(rsq);}
    }

    if (best_cnt < 4) { cur_idx = -1; continue; }

    // 合并同心切片
    if (cur_idx < 0) {
      groups.push_back({best_cx, best_cy, best_r,
                        sl.z_center - dz*0.5f, sl.z_center + dz*0.5f, 1});
      cur_idx = static_cast<int>(groups.size()) - 1;
    } else {
      auto& g = groups[cur_idx];
      float acx=g.sum_cx/g.n, acy=g.sum_cy/g.n, ar=g.sum_r/g.n;
      float dc=std::sqrt((best_cx-acx)*(best_cx-acx)+(best_cy-acy)*(best_cy-acy));
      if (dc < merge_center_tol_ && std::abs(best_r-ar) < merge_radius_tol_) {
        g.sum_cx+=best_cx; g.sum_cy+=best_cy; g.sum_r+=best_r;
        g.z_top = sl.z_center + dz*0.5f; g.n++;
      } else {
        groups.push_back({best_cx, best_cy, best_r,
                          sl.z_center - dz*0.5f, sl.z_center + dz*0.5f, 1});
        cur_idx = static_cast<int>(groups.size()) - 1;
      }
    }
  }

  static int gid = 0;
  for (const auto& g : groups) {
    if (g.n < merge_min_slices_) continue;
    float h = g.z_top - g.z_bot;
    if (h < min_cyl_height_) continue;

    CylinderDetection c;
    c.center_x = g.sum_cx/g.n; c.center_y = g.sum_cy/g.n;
    c.radius = g.sum_r/g.n;
    c.bottom_z = g.z_bot; c.top_z = g.z_top;
    c.height = h;
    c.confidence = std::min(1.f, g.n / 10.f);
    c.id = gid++;

    // 高度分类 + 预期几何验证
    if (h >= mustika_h_min_ && h <= mustika_h_max_) {
      c.type = CylinderDetection::MUSTIKA_PILLAR;
      // 半径验证: 必须在预期半径 ±30% 以内
      if (std::abs(c.radius - mustika_pillar_radius_expected_) < mustika_pillar_radius_expected_ * 0.3f) {
        c.matches_expected = true;
        c.confidence = std::min(1.f, c.confidence + 0.3f);
      }
      // 位置验证 (在无 TF 时用宽松阈值 1.0m)
      if (mustika_pillar_pos_.size() >= 2) {
        float dx = c.center_x - static_cast<float>(mustika_pillar_pos_[0]);
        float dy = c.center_y - static_cast<float>(mustika_pillar_pos_[1]);
        c.position_error = std::sqrt(dx*dx + dy*dy);
        if (c.position_error < 1.0f && c.matches_expected) {
          c.confidence = std::min(1.f, c.confidence + 0.2f);
        }
      }
    } else if (h >= core_h_min_ && h <= core_h_max_) {
      c.type = CylinderDetection::CORE_PILLAR;
      if (std::abs(c.radius - core_pillar_radius_expected_) < core_pillar_radius_expected_ * 0.3f) {
        c.matches_expected = true;
        c.confidence = std::min(1.f, c.confidence + 0.3f);
      }
      if (core_pillar_pos_.size() >= 2) {
        float dx = c.center_x - static_cast<float>(core_pillar_pos_[0]);
        float dy = c.center_y - static_cast<float>(core_pillar_pos_[1]);
        c.position_error = std::sqrt(dx*dx + dy*dy);
        if (c.position_error < 1.0f && c.matches_expected) {
          c.confidence = std::min(1.f, c.confidence + 0.2f);
        }
      }
    }
    out.push_back(c);
  }

  auto t1 = std::chrono::steady_clock::now();
  return {out, ms_dur(t1 - t0).count()};
}

// ═══════════════════════════════════════════════════════════════════════════
// [4] 障碍物聚类阶段
// ═══════════════════════════════════════════════════════════════════════════
std::pair<std::vector<Obstacle>, double>
LidarPerceptionNode::cluster_extract_stage(
    const PointCloudPtr& non_ground,
    const std::vector<CylinderDetection>& cylinders)
{
  auto t0 = std::chrono::steady_clock::now();

  // ── 圆柱掩膜 ──
  PointCloudPtr work = std::make_shared<PointCloud>();
  if (!cylinders.empty()) {
    const float mask_r_sq = 0.25f * 0.25f;  // 25cm 半径
    work->reserve(non_ground->size());
    for (const auto& pt : non_ground->points) {
      bool inside = false;
      for (const auto& cyl : cylinders) {
        float dx = pt.x - cyl.center_x;
        float dy = pt.y - cyl.center_y;
        if (dx*dx + dy*dy < mask_r_sq) { inside = true; break; }
      }
      if (!inside) work->push_back(pt);
    }
  } else {
    *work = *non_ground;
  }

  // ── 欧式聚类 ──
  std::vector<pcl::PointIndices> clusters;
  {
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(work);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(work);
    ec.extract(clusters);
  }

  // ── 构建 Obstacle + 过滤 + 分类 ──
  std::vector<Obstacle> obstacles;
  obstacles.reserve(clusters.size());
  for (size_t i = 0; i < clusters.size(); ++i) {
    Obstacle obs = build_obstacle(work, clusters[i], static_cast<int>(i));

    // 过滤
    if (obs.length > max_object_size_ || obs.width > max_object_size_) continue;
    if (obs.height > max_object_height_) continue;
    if (obs.point_count < min_object_points_) continue;
    if (obs.min_z > floating_z_min_) continue;

    classify_obstacle(obs);
    obstacles.push_back(obs);
  }

  // 重排 ID
  for (size_t i = 0; i < obstacles.size(); ++i)
    obstacles[i].id = static_cast<int>(i);

  auto t1 = std::chrono::steady_clock::now();
  return {obstacles, ms_dur(t1 - t0).count()};
}

// ═══════════════════════════════════════════════════════════════════════════
// [5] 建筑位分析阶段
// ═══════════════════════════════════════════════════════════════════════════
std::pair<std::vector<BuildingSpot>, double>
LidarPerceptionNode::building_spot_stage(const PointCloudPtr& non_ground)
{
  auto t0 = std::chrono::steady_clock::now();
  std::vector<BuildingSpot> spots;
  spots.reserve(building_positions_.size());

  for (size_t i = 0; i < building_positions_.size(); ++i) {
    BuildingSpot spot = analyze_single_spot(
        non_ground, static_cast<int>(i),
        static_cast<float>(building_positions_[i].first),
        static_cast<float>(building_positions_[i].second),
        spot_platform_z(i));
    spots.push_back(spot);
  }

  auto t1 = std::chrono::steady_clock::now();
  return {spots, ms_dur(t1 - t0).count()};
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: XYZ → PointNormal (法向量估计)
// ═══════════════════════════════════════════════════════════════════════════
LidarPerceptionNode::PointNormalCloudPtr
LidarPerceptionNode::estimate_normals(const PointCloudPtr& cloud)
{
  auto pn = std::make_shared<PointNormalCloud>();
  pn->reserve(cloud->size());
  for (const auto& p : cloud->points) {
    pcl::PointNormal pt;
    pt.x = p.x; pt.y = p.y; pt.z = p.z;
    pt.normal_x = 0; pt.normal_y = 0; pt.normal_z = 0;
    pt.curvature = 0;
    pn->push_back(pt);
  }

  if (pn->size() < 3) return pn;

  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
  tree->setInputCloud(pn);

  pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> ne;
  ne.setInputCloud(pn);
  ne.setSearchMethod(tree);
  ne.setKSearch(normal_k_search_);
  const int max_threads = static_cast<int>(std::thread::hardware_concurrency());
  ne.setNumberOfThreads(std::min(4, max_threads > 0 ? max_threads : 4));
  ne.compute(*pn);

  return pn;
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: 法向量验证
// ═══════════════════════════════════════════════════════════════════════════
bool LidarPerceptionNode::validate_plane_normal(
    const pcl::ModelCoefficients& coeff) const
{
  const double norm = std::sqrt(coeff.values[0]*coeff.values[0] + coeff.values[1]*coeff.values[1] + coeff.values[2]*coeff.values[2]);
  if (norm < 1e-9) return false;
  const double cos_angle = std::abs(coeff.values[2]) / norm;
  const double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));
  return angle < normal_angle_threshold_;
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: 动态阈值精筛
// ═══════════════════════════════════════════════════════════════════════════
pcl::PointIndices LidarPerceptionNode::refine_inliers_adaptive(
    const PointCloudPtr& cloud,
    const pcl::ModelCoefficients& coeff) const
{
  pcl::PointIndices refined;
  refined.indices.reserve(cloud->size() / 2);

  const double norm = std::sqrt(coeff.values[0]*coeff.values[0] + coeff.values[1]*coeff.values[1] + coeff.values[2]*coeff.values[2]);
  if (norm < 1e-9) return refined;

  const auto& hmap = height_threshold_map_;
  const double fallback = ransac_distance_threshold_;

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& pt = cloud->points[i];
    double dist = std::abs(coeff.values[0]*pt.x + coeff.values[1]*pt.y + coeff.values[2]*pt.z + coeff.values[3]) / norm;
    double thr = fallback;
    double z = static_cast<double>(pt.z);

    if (!hmap.empty()) {
      if (z <= hmap.front().first) thr = hmap.front().second;
      else if (z >= hmap.back().first) thr = hmap.back().second;
      else {
        for (size_t k = 0; k < hmap.size()-1; ++k) {
          if (z >= hmap[k].first && z < hmap[k+1].first) {
            double t = (z - hmap[k].first) / (hmap[k+1].first - hmap[k].first);
            thr = hmap[k].second + t * (hmap[k+1].second - hmap[k].second);
            break;
          }
        }
      }
    }
    if (dist < thr) refined.indices.push_back(i);
  }
  return refined;
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: 按平台高度分层
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::classify_by_platform(
    const PointCloudPtr& ground,
    PointCloudPtr& l0, PointCloudPtr& l1, PointCloudPtr& l2) const
{
  l0->clear(); l1->clear(); l2->clear();
  const double tol = layer_tolerance_;
  for (const auto& pt : ground->points) {
    if (std::abs(pt.z - l0_height_) < tol) l0->push_back(pt);
    else if (std::abs(pt.z - l1_height_) < tol) l1->push_back(pt);
    else if (std::abs(pt.z - l2_height_) < tol) l2->push_back(pt);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: 归一化平面系数
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::normalize_coefficients(
    pcl::ModelCoefficients& coeff) const
{
  double n = std::sqrt(coeff.values[0]*coeff.values[0] + coeff.values[1]*coeff.values[1] + coeff.values[2]*coeff.values[2]);
  if (n < 1e-9) return;
  coeff.values[0] /= n; coeff.values[1] /= n; coeff.values[2] /= n; coeff.values[3] /= n;
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: 构建 Obstacle (包围盒 + 质心 + PCA)
// ═══════════════════════════════════════════════════════════════════════════
Obstacle LidarPerceptionNode::build_obstacle(
    const PointCloudPtr& cloud,
    const pcl::PointIndices& cluster, int id)
{
  Obstacle obs; obs.id = id;
  const auto& idx = cluster.indices;
  if (idx.empty()) return obs;

  float xmin=1e9,ymin=1e9,zmin=1e9, xmax=-1e9,ymax=-1e9,zmax=-1e9;
  float sx=0,sy=0,sz=0;
  for (int i : idx) {
    const auto& p = cloud->points[i];
    if(p.x<xmin)xmin=p.x; if(p.y<ymin)ymin=p.y; if(p.z<zmin)zmin=p.z;
    if(p.x>xmax)xmax=p.x; if(p.y>ymax)ymax=p.y; if(p.z>zmax)zmax=p.z;
    sx+=p.x; sy+=p.y; sz+=p.z;
  }
  float n = static_cast<float>(idx.size());
  obs.min_x=xmin; obs.max_x=xmax; obs.min_y=ymin; obs.max_y=ymax; obs.min_z=zmin; obs.max_z=zmax;
  obs.length=xmax-xmin; obs.width=ymax-ymin; obs.height=zmax-zmin;
  obs.centroid_x=sx/n; obs.centroid_y=sy/n; obs.centroid_z=sz/n;
  obs.point_count = static_cast<int>(idx.size());

  // PCA
  if (idx.size() >= 3) {
    double cov[3][3]={{0}};
    double cx=obs.centroid_x, cy=obs.centroid_y, cz=obs.centroid_z;
    for (int i : idx) {
      double dx=cloud->points[i].x-cx, dy=cloud->points[i].y-cy, dz=cloud->points[i].z-cz;
      cov[0][0]+=dx*dx; cov[0][1]+=dx*dy; cov[0][2]+=dx*dz;
      cov[1][0]+=dy*dx; cov[1][1]+=dy*dy; cov[1][2]+=dy*dz;
      cov[2][0]+=dz*dx; cov[2][1]+=dz*dy; cov[2][2]+=dz*dz;
    }
    double inv=1.0/n;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)cov[i][j]*=inv;
    Eigen::Matrix3d C;
    C<<cov[0][0],cov[0][1],cov[0][2],
       cov[1][0],cov[1][1],cov[1][2],
       cov[2][0],cov[2][1],cov[2][2];
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);
    if(solver.info()==Eigen::Success){
      double l1=solver.eigenvalues()[2],l2=solver.eigenvalues()[1],l3=solver.eigenvalues()[0];
      obs.eigen_ratio_1 = (l2>1e-9) ? static_cast<float>(l1/l2) : 1.f;
      obs.eigen_ratio_2 = (l3>1e-9) ? static_cast<float>(l2/l3) : 1.f;
      obs.is_elongated = (obs.eigen_ratio_1 > elongation_ratio_);
      Eigen::Vector3d v1 = solver.eigenvectors().col(2);
      obs.yaw = std::atan2(static_cast<float>(v1.y()), static_cast<float>(v1.x()));
    }
  }
  return obs;
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: 尺寸分类
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::classify_obstacle(Obstacle& obs)
{
  const float tol = static_cast<float>(size_tolerance_);
  auto in_range = [tol](float v, float e) -> float {
    float d = std::abs(v-e);
    return (d > tol) ? 0.f : (1.f - d/tol);
  };

  // Enemy robot (700mm)
  { float s=(in_range(obs.length,enemy_robot_size_)+in_range(obs.width,enemy_robot_size_)+in_range(obs.height,enemy_robot_size_))/3.f;
    if(s>0.5f){obs.type=ObstacleType::ENEMY_ROBOT; obs.confidence=s; return;}}

  // Earth cube (350mm)
  { float s=(in_range(obs.length,earth_cube_size_)+in_range(obs.width,earth_cube_size_)+in_range(obs.height,earth_cube_size_))/3.f;
    if(s>0.5f){obs.type=ObstacleType::EARTH_CUBE; obs.confidence=s; return;}}

  // Sky cube vs Mustika (200mm)
  // ⚠️ 2026-09-25：与组件版 `cluster_extractor.cpp` **同步修**（§六 的"两份实现都要改"）——
  //    原实现只用 `sky_cube_size_` 算尺寸分，`mustika_size` 这个键声明了却**没进任何计算**
  //    ⇒ 改 yaml 里 `mustika_size` 静默不生效。现在两个键各算各的分。
  //    ⚠️ 今天两值都是 0.20 ⇒ **数值零变化**；但两份实现必须同时改，
  //       漏一份就会"跑单体时对、跑组件时又变回去"。
  { float sc=(in_range(obs.length,sky_cube_size_)+in_range(obs.width,sky_cube_size_)+in_range(obs.height,sky_cube_size_))/3.f;
    float ms=(in_range(obs.length,mustika_size_)+in_range(obs.width,mustika_size_)+in_range(obs.height,mustika_size_))/3.f;
    bool spherical = obs.eigen_ratio_1>0 && obs.eigen_ratio_2>0 &&
      std::abs(obs.eigen_ratio_1-1.f)<sphere_eigen_tol_ &&
      std::abs(obs.eigen_ratio_2-1.f)<sphere_eigen_tol_;
    if(spherical && ms>0.5f){obs.type=ObstacleType::MUSTIKA; obs.confidence=ms*0.85f; return;}
    if(sc>0.5f){obs.type=ObstacleType::SKY_CUBE; obs.confidence=sc; return;}}

  obs.type = ObstacleType::UNKNOWN;
  obs.confidence = 0.f;
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: 分析单建筑位
// ═══════════════════════════════════════════════════════════════════════════
BuildingSpot LidarPerceptionNode::analyze_single_spot(
    const PointCloudPtr& cloud, int id, float spot_x, float spot_y, float spot_z)
{
  BuildingSpot spot;
  spot.id = id; spot.spot_x = spot_x; spot.spot_y = spot_y;
  spot.platform_z = spot_z;   // per-spot: L1=0.6 / L2=0.9

  const float hs = static_cast<float>(spot_half_size_);
  const float z_min = spot.platform_z + static_cast<float>(column_z_min_);
  const float z_max = spot.platform_z + static_cast<float>(column_z_max_);

  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(spot_x-hs, spot_y-hs, z_min, 1.0f));
  crop.setMax(Eigen::Vector4f(spot_x+hs, spot_y+hs, z_max, 1.0f));
  crop.setNegative(false);

  auto spot_cloud = std::make_shared<PointCloud>();
  crop.filter(*spot_cloud);
  spot.point_count = static_cast<int>(spot_cloud->size());

  if (spot.point_count < min_spot_points_) {
    spot.status = BuildingSpotStatus::EMPTY;
    return spot;
  }

  float z_max_pt = -1e9f;
  for (const auto& pt : spot_cloud->points)
    if (pt.z > z_max_pt) z_max_pt = pt.z;
  spot.highest_z = z_max_pt;

  auto [has_surf, surf_z, surf_pts] = detect_top_surface(spot_cloud, z_max_pt);
  spot.has_top_surface   = has_surf;
  spot.top_surface_z     = surf_z;
  spot.top_surface_points = surf_pts;

  float h = (has_surf ? surf_z : z_max_pt) - spot.platform_z;
  spot.height = h;

  if (h < static_cast<float>(empty_max_height_)) {
    spot.status = BuildingSpotStatus::EMPTY;
  } else if (h >= static_cast<float>(one_earth_min_) && h < static_cast<float>(one_earth_max_)) {
    spot.status = BuildingSpotStatus::ONE_EARTH;
  } else if (h >= static_cast<float>(two_earth_min_) && h < static_cast<float>(two_earth_max_)) {
    spot.status = BuildingSpotStatus::TWO_EARTH;
  } else if (h >= static_cast<float>(complete_tower_min_) && h < static_cast<float>(complete_tower_max_)) {
    spot.status = BuildingSpotStatus::COMPLETE_TOWER;
    spot.needs_visual_check = true;
  } else {
    spot.status = BuildingSpotStatus::EMPTY;
  }
  return spot;
}

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: 顶面检测
// ═══════════════════════════════════════════════════════════════════════════
std::tuple<bool, float, int> LidarPerceptionNode::detect_top_surface(
    const PointCloudPtr& spot_cloud, float z_max)
{
  const float tol = static_cast<float>(top_surface_tolerance_);
  const float z_lo = z_max - tol;

  std::vector<float> z_vals;
  z_vals.reserve(spot_cloud->size());
  for (const auto& pt : spot_cloud->points)
    if (pt.z >= z_lo) z_vals.push_back(pt.z);

  int cnt = static_cast<int>(z_vals.size());
  if (cnt < top_surface_min_points_) return {false, z_max, cnt};

  float sum=0;
  for(float z:z_vals) sum+=z;
  float mean=sum/cnt;

  float var=0;
  for(float z:z_vals){float dz=z-mean; var+=dz*dz;}
  var/=cnt;

  bool flat = (std::sqrt(var) < tol * 0.5f);
  return {flat, mean, cnt};
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: 中间结果 (所有话题)
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::publish_intermediate_data(
    const PointCloudPtr& filtered,
    const GroundResult& gr,
    const std_msgs::msg::Header& header)
{
  auto pub_cloud = [&](const PointCloudPtr& cloud, auto& pub) {
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud, ros_msg);
    ros_msg.header = header;
    pub->publish(ros_msg);
  };

  pub_cloud(filtered,      pub_filtered_);     // ← ground_segmenter 要它
  pub_cloud(gr.non_ground, pub_non_ground_);   // ← 3 个组件要它
}

// ═══════════════════════════════════════════════════════════════════════════
// [纯可视化] 8 条零订阅者的话题 —— 只给 RViz 看
//
// ⚠️ 它们**不是数据接口**（`python3 scripts/check_topic_consumers.py` 逐条复核过），
//    所以挂在 `debug_enabled` 下：比赛模式关掉它们**不饿死任何人**，
//    只是 RViz 里看不到地面/分层/柱子/障碍/建筑位 —— 那是预期的。
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::publish_intermediate_viz(
    const GroundResult& gr,
    const std::vector<CylinderDetection>& cylinders,
    const std::vector<Obstacle>& obstacles,
    const std::vector<BuildingSpot>& spots,
    const std_msgs::msg::Header& header)
{
  auto pub_cloud = [&](const PointCloudPtr& cloud, auto& pub) {
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud, ros_msg);
    ros_msg.header = header;
    pub->publish(ros_msg);
  };

  pub_cloud(gr.ground,       pub_ground_);
  pub_cloud(gr.l0_surface,   pub_l0_surface_);
  pub_cloud(gr.l1_surface,   pub_l1_surface_);
  pub_cloud(gr.l2_surface,   pub_l2_surface_);

  // 平面系数
  auto plane_msg = std_msgs::msg::Float32MultiArray();
  plane_msg.data.resize(4);
  plane_msg.data[0] = static_cast<float>(gr.coeff.values[0]);
  plane_msg.data[1] = static_cast<float>(gr.coeff.values[1]);
  plane_msg.data[2] = static_cast<float>(gr.coeff.values[2]);
  plane_msg.data[3] = static_cast<float>(gr.coeff.values[3]);
  pub_plane_->publish(plane_msg);

  // 圆柱 MarkerArray
  {
    visualization_msgs::msg::MarkerArray arr;
    for (const auto& c : cylinders) {
      visualization_msgs::msg::Marker m;
      m.header = header; m.ns = "pillars"; m.id = c.id;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = c.center_x; m.pose.position.y = c.center_y;
      m.pose.position.z = (c.bottom_z + c.top_z)*0.5f;
      m.pose.orientation.w = 1.0;
      m.scale.x = c.radius*2.f; m.scale.y = c.radius*2.f; m.scale.z = c.height;
      m.color.r = (c.type==CylinderDetection::MUSTIKA_PILLAR) ? 0.16f :
                  (c.type==CylinderDetection::CORE_PILLAR) ? 0.39f : 0.5f;
      m.color.g = (c.type==CylinderDetection::MUSTIKA_PILLAR) ? 0.39f :
                  (c.type==CylinderDetection::CORE_PILLAR) ? 0.24f : 0.5f;
      m.color.b = (c.type==CylinderDetection::MUSTIKA_PILLAR) ? 0.20f :
                  (c.type==CylinderDetection::CORE_PILLAR) ? 0.00f : 0.5f;
      m.color.a = 0.7f;
      arr.markers.push_back(m);
    }
    pub_pillars_->publish(arr);
  }

  // 障碍物 MarkerArray
  {
    visualization_msgs::msg::MarkerArray arr;
    for (const auto& obs : obstacles) {
      visualization_msgs::msg::Marker m;
      m.header = header; m.ns = "obstacles"; m.id = obs.id;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = obs.centroid_x; m.pose.position.y = obs.centroid_y;
      m.pose.position.z = obs.centroid_z;
      m.pose.orientation.w = std::cos(obs.yaw*0.5f);
      m.pose.orientation.z = std::sin(obs.yaw*0.5f);
      m.scale.x = std::max(obs.length, 0.02f);
      m.scale.y = std::max(obs.width, 0.02f);
      m.scale.z = std::max(obs.height, 0.02f);
      switch(obs.type){
        case ObstacleType::EARTH_CUBE:  m.color.r=0.16f;m.color.g=0.39f;m.color.b=0.20f;break;
        case ObstacleType::SKY_CUBE:    m.color.r=0.20f;m.color.g=0.60f;m.color.b=0.86f;break;
        case ObstacleType::ENEMY_ROBOT: m.color.r=0.85f;m.color.g=0.20f;m.color.b=0.20f;break;
        case ObstacleType::MUSTIKA:     m.color.r=0.85f;m.color.g=0.65f;m.color.b=0.13f;break;
        default: m.color.r=m.color.g=m.color.b=0.5f;
      }
      m.color.a = 0.5f;
      m.lifetime = rclcpp::Duration::from_seconds(1.0);
      arr.markers.push_back(m);
    }
    if (obstacles.empty()) {
      visualization_msgs::msg::Marker del;
      del.header = header; del.ns = "obstacles"; del.id = 0;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
    }
    pub_obstacles_->publish(arr);
  }

  // 建筑位 MarkerArray
  {
    visualization_msgs::msg::MarkerArray arr;
    for (const auto& spot : spots) {
      visualization_msgs::msg::Marker m;
      m.header = header; m.ns = "building_spots"; m.id = spot.id;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = spot.spot_x; m.pose.position.y = spot.spot_y;
      m.pose.position.z = spot.platform_z + spot.height * 0.5f;
      m.pose.orientation.w = 1.0;
      m.scale.x = spot_half_size_ * 2.0; m.scale.y = spot_half_size_ * 2.0;
      m.scale.z = std::max(static_cast<double>(spot.height), 0.01);
      switch(spot.status){
        case BuildingSpotStatus::ONE_EARTH:      m.color.r=0.16f;m.color.g=0.39f;m.color.b=0.20f;m.color.a=0.8f;break;
        case BuildingSpotStatus::TWO_EARTH:      m.color.r=0.27f;m.color.g=0.55f;m.color.b=0.27f;m.color.a=0.8f;break;
        case BuildingSpotStatus::COMPLETE_TOWER: m.color.r=0.85f;m.color.g=0.65f;m.color.b=0.13f;m.color.a=0.9f;break;
        default: m.color.r=0.6f;m.color.g=0.6f;m.color.b=0.6f;m.color.a=0.3f;
      }
      m.lifetime = rclcpp::Duration::from_seconds(1.0);
      arr.markers.push_back(m);
    }
    if (spots.empty()) {
      visualization_msgs::msg::Marker del;
      del.header = header; del.ns = "building_spots"; del.id = 0;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
    }
    pub_spots_->publish(arr);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: 统一感知结果 (/perception/lidar/result)
// ═══════════════════════════════════════════════════════════════════════════
// ── 对外消息里各类枚举的**取值**（照 msg/*.msg 的注释，**不照内部枚举**）──
//   单独列出而不是就地写魔数：这里每一条都与内部枚举**编号不同**，散在代码里必看错。
namespace {
// Obstacle.msg 的 type
constexpr uint8_t kMsgObsUnknown     = 0;   // UNKNOWN
constexpr uint8_t kMsgObsEnemyRobot  = 1;   // ENEMY_ROBOT
constexpr uint8_t kMsgObsDroppedBlock= 2;   // DROPPED_BLOCK
constexpr uint8_t kMsgObsOther       = 3;   // OTHER
// Obstacle.msg / BuildingSpot.msg 的 source
constexpr uint8_t kMsgSourceLidar    = 0;   // LIDAR
// BuildingSpot.msg 的 state
constexpr uint8_t kMsgSpotEmpty      = 0;
constexpr uint8_t kMsgSpotOneEarth   = 1;
constexpr uint8_t kMsgSpotTwoEarth   = 2;
constexpr uint8_t kMsgSpotFullTower  = 3;
// ⚠️ **没量成**（2026-09-24 新增）—— 与 `msg/BuildingSpot.msg` 的 `state` 逐值一致，
//    且**走串口**（`protocol_encoder.cpp` 原样写 u8）⇒ 已记入 `comm_params.yaml`
//    头部「未冻结参数」清单**第 6 项**（单方提案，待与主控对齐）。
//    ⚠️ **它绝不能落进 `kMsgSpotEmpty`**（见下面那个函数）。
constexpr uint8_t kMsgSpotUnknown    = 4;
// BuildingSpot.msg 的 top_color
constexpr uint8_t kMsgTopColorNone    = 0;
constexpr uint8_t kMsgTopColorUnknown = 3;
// BuildingSpot.msg 的 ownership
constexpr uint8_t kMsgOwnerNone      = 0;
// PillarStatus.msg 的 type
constexpr uint8_t kMsgPillarMustika  = 0;
constexpr uint8_t kMsgPillarCore     = 1;

// ══════════════════════════════════════════════════════════════════════════
// 内部枚举 → 对外消息枚举 的**显式映射**
//
// ⚠️⚠️ 为什么必须显式写、**绝不能 `static_cast`**（2026-09-22 接线时发现）：
//     两套枚举的编号根本不一致，直接转会让分类**整体错位**，
//     而且这是**编译期完全查不出**的静默错误 —— 现场表现是"感知报了但全错"。
//
//   内部 `ObstacleType`     值   `Obstacle.msg` 的 type
//   ─────────────────────── ───  ─────────────────────
//   UNKNOWN                  0   0 = UNKNOWN
//   EARTH_CUBE               1   1 = ENEMY_ROBOT     ← 地球方块会**被当成敌方机器人**
//   SKY_CUBE                 2   2 = DROPPED_BLOCK
//   ENEMY_ROBOT              3   3 = OTHER           ← 敌方机器人会**被当成"其他"**
//   MUSTIKA                  4   （越界，无对应值）
//
//   内部 `CylinderDetection::Type` 值  `PillarStatus.msg` 的 type
//   ────────────────────────────── ───  ──────────────────────
//   UNKNOWN                        0   （无对应值）
//   MUSTIKA_PILLAR                 1   0 = MUSTIKA_PILLAR    ← **差 1**
//   CORE_PILLAR                    2   1 = CORE_PILLAR       ← **差 1**
//
//   内部 `BuildingSpotStatus` 与 `BuildingSpot.msg.state` **恰好同序**（0..3），
//   但仍走显式 switch —— 将来任一边加值，这里会**漏掉分支被编译器抓到**，
//   而不是继续静默错位。
//
//   ⚠️ 三个映射里**只有建筑位那个是安全的巧合**，另外两个是**真错位**。
//      要改这些映射，先回去读上面对应那张表，别看枚举名想当然。
// ══════════════════════════════════════════════════════════════════════════
uint8_t to_msg_obstacle_type(ObstacleType t)
{
  switch (t) {
    case ObstacleType::UNKNOWN:     return kMsgObsUnknown;
    case ObstacleType::ENEMY_ROBOT: return kMsgObsEnemyRobot;
    // 地球方块与天空方块都是"掉落在地面的方块" —— 协议里就是这么归类的。
    // ⚠️ 代价：**地球/天空之分在 Obstacle.type 里丢失**。若下游需要区分
    //    （计分要用），得扩 Obstacle.msg 的枚举 + 协议 —— 与"协议冻结"一起谈，
    //    别在这里就地编一个值。
    case ObstacleType::EARTH_CUBE:
    case ObstacleType::SKY_CUBE:    return kMsgObsDroppedBlock;
    // 穆斯蒂卡不是方块，协议里没有它的位置 → OTHER。
    // 它"在不在柱上"由 `PillarStatus.has_object_on_top` 表达，那才是正路。
    case ObstacleType::MUSTIKA:     return kMsgObsOther;
  }
  return kMsgObsUnknown;   // 将来新增枚举值时的兜底：宁可报未知，不可错位
}

uint8_t to_msg_spot_state(BuildingSpotStatus s)
{
  switch (s) {
    case BuildingSpotStatus::EMPTY:          return kMsgSpotEmpty;
    case BuildingSpotStatus::ONE_EARTH:      return kMsgSpotOneEarth;
    case BuildingSpotStatus::TWO_EARTH:      return kMsgSpotTwoEarth;
    case BuildingSpotStatus::COMPLETE_TOWER: return kMsgSpotFullTower;
    // ⚠️⚠️ **这一行是本文件最要紧的一行**（2026-09-24）—— 漏了它，
    //    UNKNOWN 会落进下面的兜底变成 **0 = EMPTY**，于是主控把"我没量到"
    //    读成"这个位是空的"：**塔明明建着，它以为没建**。
    case BuildingSpotStatus::UNKNOWN:        return kMsgSpotUnknown;
  }
  // ⚠️ 兜底也**不许是 EMPTY** —— 将来再加枚举值时，宁可报未知也不许冒充"空"。
  return kMsgSpotUnknown;
}

/// @brief 从圆柱列表里挑"最像目标柱"的那一个；挑不出返回 nullptr
///
/// ⚠️ 判据要求 `matches_expected`（高度落在该柱的量程内**且**半径在预期值 ±30% 内，见
///    `detect_cylinders` 的 707-737 行）。**不满足就返回 nullptr，不退而求其次**：
///    场上还有别的圆柱（例如两块叠起来的塔正好落在柱高量程内），挑错比不报更糟 ——
///    下游会拿一个错误位置的"柱子"去做规划。
///
/// ⚠️ 返回的是**指向 `result.cylinders` 元素的裸指针**，只在本次发布内有效，不要存。
const CylinderDetection* pick_pillar(const std::vector<CylinderDetection>& cyls,
                                     CylinderDetection::Type want)
{
  const CylinderDetection* best = nullptr;
  for (const auto& c : cyls) {
    if (c.type != want || !c.matches_expected) continue;
    if (best == nullptr ||
        c.position_error < best->position_error ||
        (c.position_error == best->position_error && c.confidence > best->confidence)) {
      best = &c;
    }
  }
  return best;
}

/// @brief 内部圆柱 → `PillarStatus`；`c == nullptr` 表示**这根柱子没检出**
///
/// ⚠️ 没检出时返回**全零**（position 0,0,0 / height 0 / diameter 0 / 无顶物）——
///    这是本节点与下游约定的"未检出"哨兵。**`PillarStatus.msg` 没有 `detected` 字段**，
///    所以"没检出"和"检出了一个 height=0 的柱子"在消息层面**长得一样**；
///    下游请按 `height == 0 && diameter == 0` 判定未检出。
///    ⚠️ 这是**接口的缺陷不是本节点的**：真要干净，该给 `PillarStatus.msg` 加
///       `bool detected`（加法改动，不影响线上协议 —— encoder 只写 occupied 位）。
///       已记入 doc/待编译验证清单，等用户定。
br_perception::msg::PillarStatus to_msg_pillar(const CylinderDetection* c)
{
  br_perception::msg::PillarStatus out;   // 默认构造即全零 = "未检出"
  if (c == nullptr) return out;

  out.type = (c->type == CylinderDetection::MUSTIKA_PILLAR) ? kMsgPillarMustika
                                                            : kMsgPillarCore;
  // 位置 = **地面投影点**（msg 的约定）。bottom_z 就是柱脚高度，不是 0 ——
  // 柱子立在 L1/L2 上时 bottom_z 会明显大于 0，别丢掉它。
  out.position.x = c->center_x;
  out.position.y = c->center_y;
  out.position.z = c->bottom_z;
  out.height   = c->height;
  out.diameter = c->radius * 2.0f;   // ⚠️ 内部存的是**半径**，消息要的是**直径**
  out.has_object_on_top = c->has_top_object;
  out.confidence = c->confidence;
  return out;
}
}  // namespace

void LidarPerceptionNode::publish_unified_result(
    const LidarPerceptionResult& result,
    const std::string& frame_id)
{
  br_perception::msg::LidarResult out;
  out.header.stamp = result.stamp;

  // ⚠️ frame_id **原样继承输入点云的**，不写死 "world"。
  //    本节点**没有任何坐标变换** —— 进来的点云是什么系, 出去的结果就是什么系。
  //    2026-09-22 核实: 全文件搜不到 transform / world 相关代码。
  //    硬写 "world" 就是**虚构一个本节点并没有产出的坐标系**,
  //    下游照它做 TF 变换会失败或算错, 且**错得没有提示**
  //    （同 `msg/CameraFrame.msg` 里对 frame_id 留空的那段理由）。
  //    转世界系是融合层的活（它有 TF、有外参、有里程计）。
  out.header.frame_id = frame_id;

  // ── 检测结果 ──
  out.obstacles.reserve(result.obstacles.size());
  for (const auto& o : result.obstacles) {
    br_perception::msg::Obstacle m;
    m.id = static_cast<uint32_t>(std::max(0, o.id));
    m.position.x = o.centroid_x;
    m.position.y = o.centroid_y;
    m.position.z = o.centroid_z;
    m.bbox_size.x = o.length;
    m.bbox_size.y = o.width;
    m.bbox_size.z = o.height;
    // ⚠️ 速度**恒为 0**, 这是如实报告: 雷达层只出单帧几何, 不做运动估计。
    //    速度是 `kalman_tracker`（模块 18）的活 —— 它拿到连续帧后给。
    //    这里填 0 的意思是"本节点不提供速度", 不是"该物体静止"。
    m.velocity.x = 0.0; m.velocity.y = 0.0; m.velocity.z = 0.0;
    m.type = to_msg_obstacle_type(o.type);
    m.confidence = o.confidence;
    m.source = kMsgSourceLidar;
    out.obstacles.push_back(m);
  }

  out.building_spots.reserve(result.building_spots.size());
  for (const auto& s : result.building_spots) {
    br_perception::msg::BuildingSpot m;
    m.spot_id = static_cast<uint32_t>(std::max(0, s.id));
    m.position.x = s.spot_x;
    m.position.y = s.spot_y;
    m.position.z = s.platform_z;
    m.state = to_msg_spot_state(s.status);
    // ⚠️⚠️ **雷达判不了塔顶颜色** —— 这是本层的硬事实, 不是没做:
    //    `building_spot_analyzer.cpp:296` 原文"完整塔: 雷达无法判断天空方块朝上颜色
    //    → 标记需要视觉辅助"。
    //    所以这里填 **UNKNOWN(3)**, 而**不是 NONE(0)**:
    //      NONE 的意思是"没有塔顶", 而这里明明有塔 —— 填 NONE 就是**撒谎**,
    //      下游会以为那个建筑位是空的。
    //    顶面颜色归 `sky_block_detector`（模块 9, 视觉）, 融合时覆盖这个值。
    m.top_color = kMsgTopColorUnknown;
    // 雷达没有"归属"概念（那是按队伍立场推断的, 需要融合层知道红蓝方与自机颜色）
    m.ownership = kMsgOwnerNone;
    m.source = kMsgSourceLidar;
    // ⚠️ 雷达层**不计算逐建筑位置信度**（没有那个算法）。这里给的是**由可观测证据
    //    导出的粗值**, 语义只有三档, **别当连续量用**:
    //      · 有点云且有顶面证据 → 1.0（"确实在那儿"）
    //      · 有点云但没形成顶面  → 0.6（"有回波, 但不像塔/方块"）
    //      · 完全没点            → 0.0（"**没观测**", 不是"看见了但不可信"）
    //    ⚠️ 第三档与"置信度为 0"字面相同 —— 要区分得看 `point_count`,
    //       故本节点把 point_count 放进了诊断话题。
    //       融合层若要更精细的置信度, 那是 building_spot_analyzer 要补的算法。
    if (s.has_top_surface)      m.confidence = 1.0f;
    else if (s.point_count > 0) m.confidence = 0.6f;
    else                        m.confidence = 0.0f;
    out.building_spots.push_back(m);
  }

  // ── 两根柱子（一根都没有时是全零哨兵，见 to_msg_pillar）──
  out.mustika_pillar = to_msg_pillar(
      pick_pillar(result.cylinders, CylinderDetection::MUSTIKA_PILLAR));
  out.core_pillar = to_msg_pillar(
      pick_pillar(result.cylinders, CylinderDetection::CORE_PILLAR));

  // ── 各阶段耗时 ──
  // ⚠️ 下标顺序与 `StageProfiler::declare` 的顺序对应（msg/LidarResult.msg 的约定）。
  //    本节点用的是 `PipelineTiming` 结构体而不是 StageProfiler, 故这里**显式按
  //    管线阶段顺序**填, 顺序写在下面这段注释里, 改管线顺序时**两边一起改**:
  //      [0] 预处理  [1] 地面分割  [2] 圆柱检测  [3] 障碍聚类  [4] 建筑位分析
  out.stage_elapsed_ms = {
      static_cast<float>(result.timing.preprocess_ms),
      static_cast<float>(result.timing.ground_segment_ms),
      static_cast<float>(result.timing.cylinder_detect_ms),
      static_cast<float>(result.timing.cluster_extract_ms),
      static_cast<float>(result.timing.building_spot_ms)};

  // watchdog_ok = "雷达流水线是否在线"。注意**只看雷达连没连**，不看这一帧处理得好不好
  // （地面没找到 / 处理慢 是另外的诊断项，混进来会让下游分不清"断流"与"这帧质量差"）。
  out.watchdog_ok = result.radar_connected;

  pub_result_->publish(out);
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: 调试点云 (不同颜色标注各层)
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::publish_debug_cloud(
    const PointCloudPtr& ground,
    const PointCloudPtr& non_ground,
    const std::vector<CylinderDetection>& cylinders,
    const std::vector<Obstacle>& obstacles,
    const std::vector<BuildingSpot>& spots,
    const std_msgs::msg::Header& header)
{
  // 使用 pcl::PointXYZRGB 发布带颜色标注的点云
  pcl::PointCloud<pcl::PointXYZRGB> debug_cloud;

  // 地面点 → 绿色
  for (const auto& pt : ground->points) {
    pcl::PointXYZRGB cp;
    cp.x = pt.x; cp.y = pt.y; cp.z = pt.z;
    cp.r = 0; cp.g = 180; cp.b = 0;
    debug_cloud.push_back(cp);
  }

  // 非地面点 → 默认灰色
  for (const auto& pt : non_ground->points) {
    pcl::PointXYZRGB cp;
    cp.x = pt.x; cp.y = pt.y; cp.z = pt.z;
    cp.r = 160; cp.g = 160; cp.b = 160;

    // 圆柱区域的点 → 橙色
    for (const auto& cyl : cylinders) {
      float dx = pt.x - cyl.center_x;
      float dy = pt.y - cyl.center_y;
      float r_sq = (cyl.radius + 0.05f) * (cyl.radius + 0.05f);
      if (dx*dx + dy*dy < r_sq) {
        cp.r = 255; cp.g = 140; cp.b = 0; break;
      }
    }

    // 障碍物包围盒内的点 → 红色
    for (const auto& obs : obstacles) {
      if (pt.x >= obs.min_x && pt.x <= obs.max_x &&
          pt.y >= obs.min_y && pt.y <= obs.max_y &&
          pt.z >= obs.min_z && pt.z <= obs.max_z) {
        cp.r = 220; cp.g = 50; cp.b = 50; break;
      }
    }

    // 建筑位区域内的点 → 金色
    for (const auto& spot : spots) {
      if (spot.status == BuildingSpotStatus::EMPTY) continue;
      float hs = static_cast<float>(spot_half_size_);
      if (std::abs(pt.x - spot.spot_x) < hs &&
          std::abs(pt.y - spot.spot_y) < hs) {
        cp.r = 218; cp.g = 165; cp.b = 32; break;
      }
    }

    debug_cloud.push_back(cp);
  }

  sensor_msgs::msg::PointCloud2 ros_msg;
  pcl::toROSMsg(debug_cloud, ros_msg);
  ros_msg.header = header;
  pub_debug_cloud_->publish(ros_msg);
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: 诊断 (各阶段耗时)
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::publish_diagnostics(
    const PipelineTiming& timing,
    const LidarPerceptionResult& result)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "pipeline_timing";
  st.hardware_id = "mid360";

  if (timing.total_ms < 60.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "Pipeline OK";
  } else if (timing.total_ms < 100.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "Pipeline SLOW";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "Pipeline VERY SLOW";
  }

  auto add = [&](const std::string& k, double v) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = k;
    kv.value = std::to_string(v); st.values.push_back(kv);
  };

  add("total_ms",          timing.total_ms);
  add("preprocess_ms",     timing.preprocess_ms);
  add("ground_segment_ms", timing.ground_segment_ms);
  add("cylinder_detect_ms",timing.cylinder_detect_ms);
  add("cluster_extract_ms",timing.cluster_extract_ms);
  add("building_spot_ms",  timing.building_spot_ms);
  add("input_points",      static_cast<double>(timing.input_points));
  add("ground_points",     static_cast<double>(timing.ground_points));
  add("non_ground_points", static_cast<double>(timing.non_ground_points));

  // ═══════════════════════════════════════════════════════════════════════════
  // 以下这些**原先塞在 `/perception/lidar/result` 里**（那时那边发的是
  // `DiagnosticArray`）。2026-09-22 那条话题改成发真正的数据接口 `LidarResult`
  // 之后，这些**给人看的**内容搬到这里 —— **一项都没丢，只是换了话题**。
  // 它们本来就属于诊断：下游要的是 `LidarResult` 里的结构化字段，不是这些计数文本。
  // ═══════════════════════════════════════════════════════════════════════════

  // ── 健康与质量 ──
  // ⚠️ 这三项与 `LidarResult.watchdog_ok`（只表示雷达连没连）**不是一回事**：
  //    这里说的是一帧算得好不好（地面找没找到、处理快不快）。分开表达，
  //    下游才能分清"断流了"和"这帧质量差"。
  add("ground_found",    result.ground_found ? 1.0 : 0.0);
  add("radar_connected", result.radar_connected ? 1.0 : 0.0);

  // ── 地面平面系数（归一化 a,b,c,d）──
  if (result.ground_plane.size() >= 4) {
    add("ground_a", result.ground_plane[0]);
    add("ground_b", result.ground_plane[1]);
    add("ground_c", result.ground_plane[2]);
    add("ground_d", result.ground_plane[3]);
  }

  // ── 检测计数 ──
  add("cylinder_count",      static_cast<double>(result.cylinders.size()));
  add("obstacle_count",      static_cast<double>(result.obstacles.size()));
  add("building_spot_count", static_cast<double>(result.building_spots.size()));

  int empty_c = 0, one_c = 0, two_c = 0, tower_c = 0, unknown_c = 0;
  for (const auto& s : result.building_spots) {
    switch (s.status) {
      case BuildingSpotStatus::EMPTY:          empty_c++;   break;
      case BuildingSpotStatus::ONE_EARTH:      one_c++;     break;
      case BuildingSpotStatus::TWO_EARTH:      two_c++;     break;
      case BuildingSpotStatus::COMPLETE_TOWER: tower_c++;   break;
      // ⚠️ **单独一档**：`unknown` 涨 = "没量成"（TF 不可用等），
      //    与 `empty` 涨（真量到是空的）**是两件事** —— 现场的读法完全不同。
      case BuildingSpotStatus::UNKNOWN:        unknown_c++; break;
    }
  }
  add("spots_empty",     static_cast<double>(empty_c));
  add("spots_one_earth", static_cast<double>(one_c));
  add("spots_two_earth", static_cast<double>(two_c));
  add("spots_tower",     static_cast<double>(tower_c));
  // ⚠️ 与 `spots_empty` **必须分开读**：这一项涨说明**这一帧没量成**，
  //    不是"这些位是空的"（见 `BuildingSpotStatus::UNKNOWN`）。
  add("spots_unknown",   static_cast<double>(unknown_c));

  // ⚠️ 这两个计数是**内部**分类（地球/天空方块分得开），而发出去的
  //    `Obstacle.type` 把两者都归成 DROPPED_BLOCK（协议里没有更细的档）。
  //    所以**这里是对外接口的信息损失的一个观测窗口**：现场若发现"两种方块分不开了"，
  //    看这两个计数还是分开的，说明是**消息/协议的表达力不够**，不是检测器坏了。
  int earth = 0, sky = 0, enemy = 0, mustika = 0, unknown = 0;
  for (const auto& o : result.obstacles) {
    switch (o.type) {
      case ObstacleType::EARTH_CUBE:  earth++;   break;
      case ObstacleType::SKY_CUBE:    sky++;     break;
      case ObstacleType::ENEMY_ROBOT: enemy++;   break;
      case ObstacleType::MUSTIKA:     mustika++; break;
      case ObstacleType::UNKNOWN:     unknown++; break;
    }
  }
  add("obs_earth",   static_cast<double>(earth));
  add("obs_sky",     static_cast<double>(sky));
  add("obs_enemy",   static_cast<double>(enemy));
  add("obs_mustika", static_cast<double>(mustika));
  add("obs_unknown", static_cast<double>(unknown));

  // ── 分层地面点云规模 ──
  add("l0_points", static_cast<double>(result.l0_points));
  add("l1_points", static_cast<double>(result.l1_points));
  add("l2_points", static_cast<double>(result.l2_points));

  arr.status.push_back(st);

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️⚠️ 端到端**年龄**（2026-09-26 加）—— **单开一条**，不塞进 `pipeline_timing`。
  //
  //    上面那条里头**全是处理耗时**（各段花了多久）；这一条回答的是**另一个问题**：
  //    「这帧数据到我手上时已经旧了多少」—— **延迟预算是按后者算的**。
  //
  //    ⚠️ 为什么单开一条而不是并进 `pipeline_timing`：
  //       那条的 `level` 语义是「本帧算得快不快」，下游和人已经按那个读；
  //       把年龄混进去会**改掉它的语义**（同一个 `level` 两个含义）。
  //       分开表达才能分清「算得慢」和「数据旧」—— 那是两件事、两个动作。
  //
  //    怎么读（**这里踩过一次坑，记下来**）：
  //      · `header.stamp` 是**扫描开始**（驱动 `lddc.cpp:289` 用 `pkg.base_time`）
  //        ⇒ `now − stamp` 是**最旧那个点**的年龄；
  //      · 雷达与相机**结构不同**：相机整帧在同一瞬间曝光（最旧=最新），
  //        而 Mid-360 一帧横跨 **100ms** ⇒ 两端差一整个帧周期。
  //      ⇒ 报**两个**：`age_newest_ms`（与相机 `frame_age_sec` 同口径，**主看它**）
  //        和 `age_oldest_ms`。**不要**写成 `age + period` —— 那是算两遍，
  //        本诊断第一版就这么写的，实测虚报了一整个帧周期（256ms vs 实际 ~56ms）。
  //
  //    ⚠️ 阈值 56/80 与 `config/perception_params.yaml` 的
  //       `latency_warn_ms` / `latency_error_ms` **同源**（那里写着
  //       "warn 56 = §15.3 的 max(雷达 34, 视觉 46) + 10"）。
  //       ⚠️ 改那边要一起改这里 —— 本节点没有那两个参数，只能硬编码。
  // ═══════════════════════════════════════════════════════════════════════════
  if (timing.input_stamp.nanoseconds() > 0) {
    diagnostic_msgs::msg::DiagnosticStatus lx;
    lx.name = "end_to_end_latency";
    lx.hardware_id = "mid360";

    // ⚠️⚠️ **`header.stamp` 是"扫描开始"，不是"扫描结束"**（2026-09-26 核实）：
    //   驱动里是 `timestamp = pkg.base_time;`（`lddc.cpp:289/331`）——
    //   `base_time` 是该帧**第一个点**的时刻。
    //   ⇒ `now - input_stamp` = **最旧那个点**的年龄；最新那个点比它新一整个帧周期。
    //
    //   ⚠️ 所以**不能**写成 `age + period`（那是把帧周期算两遍）——
    //      本诊断的第一版就是这么写的，实测报出 256ms，而按同口径只有 ~56ms，
    //      **虚报了一整个帧周期**。看数的人会据此去做错误的取舍。
    const double age_oldest_ms = (this->now() - timing.input_stamp).seconds() * 1000.0;
    const double age_newest_ms = std::max(0.0, age_oldest_ms - timing.period_ms);

    // 告警看**最新那个点**（与相机侧 `frame_age_sec` 同口径：都是"最新数据有多旧"）
    const double watch_ms = age_newest_ms;
    if (watch_ms < 56.0) {
      lx.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      lx.message = "Latency OK";
    } else if (watch_ms < 80.0) {
      lx.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      lx.message = "Latency over warn (56ms)";
    } else {
      lx.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      lx.message = "Latency over error (80ms)";
    }

    auto addl = [&lx](const std::string& k, double v) {
      diagnostic_msgs::msg::KeyValue kv; kv.key = k;
      kv.value = std::to_string(v); lx.values.push_back(kv);
    };
    // ⚠️ 两项**都要报**，而且名字要说清是哪一端 —— 一帧点云横跨一整个帧周期，
    //    "最旧"和"最新"差 100ms，只报一个必然被误读（本诊断第一版就栽在这）。
    addl("age_newest_ms", age_newest_ms);   // ⭐ 与相机同口径，**看这个**
    addl("age_oldest_ms", age_oldest_ms);   // 该帧最早那个点有多旧
    addl("period_ms",     timing.period_ms); // 传感器帧周期（Mid-360 ≈ 100ms）

    arr.status.push_back(lx);
  }

  pub_diagnostics_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 看门狗: 雷达断连检测
// ═══════════════════════════════════════════════════════════════════════════
void LidarPerceptionNode::radar_watchdog_timer_callback()
{
  if (!radar_connected_) return;  // 已经断连, 不重复告警

  // 如果还没有收到过点云, 跳过看门狗检查
  if (last_cloud_time_.nanoseconds() == 0) return;

  auto now = this->now();
  auto elapsed = (now - last_cloud_time_).seconds();

  if (elapsed > watchdog_timeout_) {
    radar_connected_ = false;
    RCLCPP_WARN(this->get_logger(),
      "Radar disconnected! Last cloud received %.1fs ago (timeout=%.1fs)",
      elapsed, watchdog_timeout_);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult LidarPerceptionNode::on_param_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& p : params) {
    const std::string& n = p.get_name();
    try {
      #define S(name,var) if(n==name) var=p.as_double()
      #define SI(name,var) if(n==name) var=p.as_int()
      #define SB(name,var) if(n==name) var=p.as_bool()

      S("leaf_size", leaf_size_);
      SB("enable_outlier_filter", enable_outlier_filter_);
      SI("outlier_mean_k", outlier_mean_k_);
      S("outlier_std_thresh", outlier_std_thresh_);
      S("roi_min_x", roi_min_x_); S("roi_max_x", roi_max_x_);
      S("roi_min_y", roi_min_y_); S("roi_max_y", roi_max_y_);
      S("roi_min_z", roi_min_z_); S("roi_max_z", roi_max_z_);
      S("ransac_distance_threshold", ransac_distance_threshold_);
      SI("ransac_max_iterations", ransac_max_iterations_);
      S("ransac_eps_angle", ransac_eps_angle_);
      SB("enable_normal_constraint", enable_normal_constraint_);
      S("normal_angle_threshold", normal_angle_threshold_);
      S("l0_height", l0_height_); S("l1_height", l1_height_);
      S("l2_height", l2_height_); S("layer_tolerance", layer_tolerance_);
      SI("normal_k_search", normal_k_search_);
      S("slice_thickness", slice_thickness_);
      SI("slice_min_points", slice_min_points_);
      S("circle_fit_thresh", circle_fit_thresh_);
      S("radius_min", radius_min_); S("radius_max", radius_max_);
      S("merge_center_tol", merge_center_tol_);
      S("merge_radius_tol", merge_radius_tol_);
      SI("merge_min_slices", merge_min_slices_);
      S("min_cyl_height", min_cyl_height_);
      S("cyl3d_dist_thresh", cyl3d_dist_thresh_);
      SI("cyl3d_max_iters", cyl3d_max_iters_);
      S("cyl3d_normal_weight", cyl3d_normal_weight_);
      S("cyl3d_axis_eps", cyl3d_axis_eps_);
      S("mustika_h_min", mustika_h_min_); S("mustika_h_max", mustika_h_max_);
      S("core_h_min", core_h_min_); S("core_h_max", core_h_max_);

      // 预期柱子几何 — 支持 vector<double> 的嵌套参数
      if (n == "mustika_pillar.position") { mustika_pillar_pos_ = p.as_double_array(); }
      if (n == "mustika_pillar.height")   { mustika_pillar_height_expected_ = p.as_double(); }
      if (n == "mustika_pillar.radius")   { mustika_pillar_radius_expected_ = p.as_double(); }
      if (n == "core_pillar.position")    { core_pillar_pos_ = p.as_double_array(); }
      if (n == "core_pillar.height")      { core_pillar_height_expected_ = p.as_double(); }
      if (n == "core_pillar.radius")      { core_pillar_radius_expected_ = p.as_double(); }

      S("cluster_tolerance", cluster_tolerance_);
      SI("min_cluster_size", min_cluster_size_);
      SI("max_cluster_size", max_cluster_size_);
      S("max_object_size", max_object_size_);
      S("max_object_height", max_object_height_);
      SI("min_object_points", min_object_points_);
      S("floating_z_min", floating_z_min_);
      S("size_tolerance", size_tolerance_);
      S("earth_cube_size", earth_cube_size_);
      S("sky_cube_size", sky_cube_size_);
      S("enemy_robot_size", enemy_robot_size_);
      S("mustika_size", mustika_size_);
      S("elongation_ratio", elongation_ratio_);
      S("sphere_eigen_tol", sphere_eigen_tol_);
      S("platform_z", platform_z_);
      S("spot_half_size", spot_half_size_);
      S("column_z_min", column_z_min_); S("column_z_max", column_z_max_);
      SI("min_spot_points", min_spot_points_);
      S("top_surface_tolerance", top_surface_tolerance_);
      SI("top_surface_min_points", top_surface_min_points_);
      S("empty_max_height", empty_max_height_);
      S("one_earth_min", one_earth_min_); S("one_earth_max", one_earth_max_);
      S("two_earth_min", two_earth_min_); S("two_earth_max", two_earth_max_);
      S("complete_tower_min", complete_tower_min_);
      S("complete_tower_max", complete_tower_max_);
      SB("debug_enabled", debug_enabled_);
      S("watchdog_timeout", watchdog_timeout_);
      SI("max_skip_frames", max_skip_frames_);

      #undef S
      #undef SI
      #undef SB
    }
    catch (const rclcpp::ParameterTypeException& e) {
      result.successful = false;
      result.reason = std::string("Type mismatch: ") + n;
      break;
    }
  }

  if (result.successful) {
    RCLCPP_DEBUG(this->get_logger(),
      "Params updated | debug=%s watchdog=%.1fs",
      debug_enabled_ ? "ON" : "OFF", watchdog_timeout_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

}  // namespace lidar
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::lidar::LidarPerceptionNode)
