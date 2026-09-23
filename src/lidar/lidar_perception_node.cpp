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
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>

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
    case BuildingSpotStatus::ONE_EARTH:      return "One Earth";
    case BuildingSpotStatus::TWO_EARTH:      return "Two Earth";
    case BuildingSpotStatus::COMPLETE_TOWER: return "Complete Tower";
    default:                                  return "Empty";
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

  // ── 调试 & 看门狗参数 ──
  this->declare_parameter<bool>("debug_enabled", debug_enabled_);
  this->declare_parameter<double>("watchdog_timeout", watchdog_timeout_);
  this->declare_parameter<int>("max_skip_frames", max_skip_frames_);

  // ── 读取参数 ──
  #define GET(n, v) this->get_parameter(n, v)
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

  RCLCPP_INFO(this->get_logger(),
    "LidarPerceptionNode ready | "
    "leaf=%.3f roi=%+.1f..%+.1f | ransac_th=%.3f iters=%d | "
    "cyl: slice=%.2f radius=[%.2f,%.2f] | "
    "cluster: tol=%.3f min=%d max=%d | "
    "spots=%zu | debug=%s watchdog=%.1fs | "
    "mustika_pillar: pos=[%.1f,%.1f] h=%.2f r=%.3f | "
    "core_pillar: pos=[%.1f,%.1f] h=%.2f r=%.3f",
    leaf_size_, roi_min_x_, roi_max_x_,
    ransac_distance_threshold_, ransac_max_iterations_,
    slice_thickness_, radius_min_, radius_max_,
    cluster_tolerance_, min_cluster_size_, max_cluster_size_,
    building_positions_.size(),
    debug_enabled_ ? "ON" : "OFF", watchdog_timeout_,
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
void LidarPerceptionNode::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t0 = std::chrono::steady_clock::now();

  // ── 雷达在线标记 ──
  last_cloud_time_ = this->now();
  if (!radar_connected_) {
    radar_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Radar reconnected");
  }

  // ── 解包 (try-catch 防御) ──
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

  if (raw_cloud->empty()) {
    consecutive_empty_frames_++;
    if (consecutive_empty_frames_ >= max_skip_frames_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "Empty point cloud (consecutive=%d)", consecutive_empty_frames_);
    }
    return;
  }
  consecutive_empty_frames_ = 0;

  PipelineTiming timing;
  timing.input_points = static_cast<int>(raw_cloud->size());

  // ═══════════════════════════════════════════════════════════════════════
  // [1] 预处理
  // ═══════════════════════════════════════════════════════════════════════
  auto [filtered, preproc_ms] = preprocess_stage(raw_cloud);
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
  // 发布中间结果 —— ⚠️ 2026-09-23 起**挂到 `debug_enabled` 下**
  // ═══════════════════════════════════════════════════════════════════════
  // 原先**无条件每帧执行**，而它发的是**纯可视化**内容（2026-09-23 逐条核实）：
  //
  //   话题                                     代码订阅者  RViz 显示项
  //   /perception/lidar/filtered                    0          0
  //   /perception/lidar/ground                      0          1
  //   /perception/lidar/non_ground                  0          1
  //   /perception/lidar/l0_surface                  0          0   ← 孤儿
  //   /perception/lidar/l1_surface                  0          0   ← 孤儿
  //   /perception/lidar/l2_surface                  0          0   ← 孤儿
  //   /perception/lidar/ground_plane                0          0   ← 孤儿
  //   /perception/lidar/pillars  (MarkerArray)      0          1
  //   /perception/lidar/obstacles(MarkerArray)      0          1
  //   /perception/lidar/building_spots(MarkerArray) 0          1
  //
  // ⇒ **10 条话题、6 次整份点云 `pcl::toROSMsg` 拷贝**，每帧都做，而**代码里零订阅者**。
  //    `LidarResult.msg` 的头注释早已定性："MarkerArray / debug_cloud 保留，
  //    但从此**只作可视化**，不再承担数据职责" —— 数据走 `LidarResult`。
  //    所以它们**全部属于"调试输出"**，比赛模式就该关（任务书 §1.7）。
  //
  // ⚠️ **代价（要知道，别当成 bug）**：`debug_enabled: false` 时 RViz 里
  //    **柱子/障碍/建筑位/地面/非地面都看不到**（它们是这 10 条话题画的）。
  //    台架要看就改 `config/debug_flags.yaml` 的 `debug_enabled: true` ——
  //    ⚠️ **别去改回这里**：那等于把比赛模式的 CPU 又还回去了。
  //
  // ⚠️ 注意与上面"调试点云"的区别：`publish_debug_cloud`（`:463` 附近）发的是
  //    **另一条** `debug_cloud`，它**本来就在这个开关下**。本条漏了，现在补齐。
  if (debug_enabled_) {
    publish_intermediate(filtered, gr, cylinders, obstacles, spots, msg->header);
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

  // ⚠️ frame_id 取自**输入点云的 header**（`msg` 是本回调的入参点云）。
  //    本节点没有坐标变换，输出坐标系 = 输入坐标系，继承是最诚实的做法。
  publish_unified_result(result, msg->header.frame_id);

  // ═══════════════════════════════════════════════════════════════════════
  // 调试点云
  // ═══════════════════════════════════════════════════════════════════════
  if (debug_enabled_) {
    publish_debug_cloud(gr.ground, gr.non_ground, cylinders,
                        obstacles, spots, msg->header);
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
  { float s=(in_range(obs.length,sky_cube_size_)+in_range(obs.width,sky_cube_size_)+in_range(obs.height,sky_cube_size_))/3.f;
    if(s>0.5f){
      bool spherical = obs.eigen_ratio_1>0 && obs.eigen_ratio_2>0 &&
        std::abs(obs.eigen_ratio_1-1.f)<sphere_eigen_tol_ &&
        std::abs(obs.eigen_ratio_2-1.f)<sphere_eigen_tol_;
      obs.type = spherical ? ObstacleType::MUSTIKA : ObstacleType::SKY_CUBE;
      obs.confidence = spherical ? s*0.85f : s; return;}}

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
void LidarPerceptionNode::publish_intermediate(
    const PointCloudPtr& filtered,
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

  pub_cloud(filtered,        pub_filtered_);
  pub_cloud(gr.ground,       pub_ground_);
  pub_cloud(gr.non_ground,   pub_non_ground_);
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
  }
  return kMsgSpotEmpty;
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

  int empty_c = 0, one_c = 0, two_c = 0, tower_c = 0;
  for (const auto& s : result.building_spots) {
    switch (s.status) {
      case BuildingSpotStatus::EMPTY:          empty_c++; break;
      case BuildingSpotStatus::ONE_EARTH:      one_c++;   break;
      case BuildingSpotStatus::TWO_EARTH:      two_c++;   break;
      case BuildingSpotStatus::COMPLETE_TOWER: tower_c++; break;
    }
  }
  add("spots_empty",     static_cast<double>(empty_c));
  add("spots_one_earth", static_cast<double>(one_c));
  add("spots_two_earth", static_cast<double>(two_c));
  add("spots_tower",     static_cast<double>(tower_c));

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
