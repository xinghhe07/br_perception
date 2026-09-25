#include "br_perception/lidar/cluster_extractor.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>      // std::pair

namespace br_perception {
namespace lidar {

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: ObstacleType → 字符串
// ═══════════════════════════════════════════════════════════════════════════
namespace {
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
ClusterExtractor::ClusterExtractor(const rclcpp::NodeOptions& options)
  : Node("cluster_extractor", options)
{
  // ⚠️ 声明与读取都走 `params_` —— 它是本模块参数的**唯一事实源**。
  //    三方一致（yaml 键名 = Params 字段名 = 这里的字面量）由同一行的两处引用保证。
  // ── 声明聚类参数 ──
  this->declare_parameter<double>("cluster_tolerance",    params_.cluster_tolerance);
  this->declare_parameter<int>("min_cluster_size",        params_.min_cluster_size);
  this->declare_parameter<int>("max_cluster_size",        params_.max_cluster_size);

  // ── 声明过滤参数 ──
  this->declare_parameter<double>("max_object_size",      params_.max_object_size);
  this->declare_parameter<double>("max_object_height",    params_.max_object_height);
  this->declare_parameter<int>("min_object_points",       params_.min_object_points);
  this->declare_parameter<double>("floating_z_min",       params_.floating_z_min);

  // ── 声明圆柱掩膜参数 ──
  this->declare_parameter<bool>("enable_cylinder_mask",   params_.enable_cylinder_mask);
  this->declare_parameter<double>("cylinder_mask_radius", params_.cylinder_mask_radius);
  this->declare_parameter<std::vector<double>>("cylinder_mask_x");
  this->declare_parameter<std::vector<double>>("cylinder_mask_y");

  // ── 声明分类参数 ──
  this->declare_parameter<double>("size_tolerance",       params_.size_tolerance);
  this->declare_parameter<double>("earth_cube_size",      params_.earth_cube_size);
  this->declare_parameter<double>("sky_cube_size",        params_.sky_cube_size);
  this->declare_parameter<double>("enemy_robot_size",     params_.enemy_robot_size);
  this->declare_parameter<double>("mustika_size",         params_.mustika_size);
  this->declare_parameter<double>("elongation_ratio",     params_.elongation_ratio);
  this->declare_parameter<double>("sphere_eigen_tol",     params_.sphere_eigen_tol);

  // ── 读取参数 ──
  #define GET(name, var) this->get_parameter(name, var)
  GET("cluster_tolerance",    params_.cluster_tolerance);
  GET("min_cluster_size",     params_.min_cluster_size);
  GET("max_cluster_size",     params_.max_cluster_size);
  GET("max_object_size",      params_.max_object_size);
  GET("max_object_height",    params_.max_object_height);
  GET("min_object_points",    params_.min_object_points);
  GET("floating_z_min",       params_.floating_z_min);
  GET("enable_cylinder_mask", params_.enable_cylinder_mask);
  GET("cylinder_mask_radius", params_.cylinder_mask_radius);
  GET("size_tolerance",       params_.size_tolerance);
  GET("earth_cube_size",      params_.earth_cube_size);
  GET("sky_cube_size",        params_.sky_cube_size);
  GET("enemy_robot_size",     params_.enemy_robot_size);
  GET("mustika_size",         params_.mustika_size);
  GET("elongation_ratio",     params_.elongation_ratio);
  GET("sphere_eigen_tol",     params_.sphere_eigen_tol);
  #undef GET

  // 圆柱中心列表: 从两个独立数组重建为 pair 向量 (避免越界)
  {
    std::vector<double> cx, cy;
    this->get_parameter("cylinder_mask_x", cx);
    this->get_parameter("cylinder_mask_y", cy);
    if (cx.size() != cy.size()) {
      RCLCPP_WARN(this->get_logger(),
        "cylinder_mask_x (%zu) and cylinder_mask_y (%zu) length mismatch; "
        "cylinder mask disabled",
        cx.size(), cy.size());
      params_.enable_cylinder_mask = false;
    } else {
      params_.cylinder_centers.reserve(cx.size());
      for (size_t i = 0; i < cx.size(); ++i) {
        params_.cylinder_centers.emplace_back(cx[i], cy[i]);
      }
    }
  }

  // ── 动态参数回调 ──
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ClusterExtractor::on_param_change, this, std::placeholders::_1));

  // ── 订阅 ──
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/lidar/non_ground",
    rclcpp::SensorDataQoS(),
    std::bind(&ClusterExtractor::cloud_callback, this, std::placeholders::_1));

  // ── 发布 ──
  pub_obstacles_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/lidar/obstacles", 10);
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/lidar/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "ClusterExtractor ready | "
    "cluster: tol=%.3fm min=%d max=%d | "
    "filter: max_size=%.2fm max_h=%.2fm min_pts=%d floating_z=%.2fm | "
    "cylinder_mask: %s radius=%.2fm centers=%zu | "
    "classify: tol=%.3fm earth=%.2f sky=%.2f robot=%.2f mustika=%.2f",
    params_.cluster_tolerance, params_.min_cluster_size, params_.max_cluster_size,
    params_.max_object_size, params_.max_object_height, params_.min_object_points,
    params_.floating_z_min,
    params_.enable_cylinder_mask ? "ON" : "OFF",
    params_.cylinder_mask_radius, params_.cylinder_centers.size(),
    params_.size_tolerance, params_.earth_cube_size, params_.sky_cube_size,
    params_.enemy_robot_size, params_.mustika_size);
}

// ═══════════════════════════════════════════════════════════════════════════
// 主回调
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t0 = std::chrono::steady_clock::now();

  // ── 解包 ──
  PointCloudPtr cloud_in = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *cloud_in);

  if (cloud_in->size() < static_cast<size_t>(params_.min_cluster_size)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                         3000, "Too few non-ground points: %zu", cloud_in->size());
    // 即使无有效数据也发布空结果, 避免下游等待超时
    std::vector<Obstacle> empty;
    publish_obstacles(empty, msg->header);
    publish_diagnostics(0.0, 0, 0, empty);
    return;
  }

  // ── 1. 圆柱掩膜 (可选) ──
  PointCloudPtr cloud_work = cloud_in;
  if (params_.enable_cylinder_mask && !params_.cylinder_centers.empty()) {
    cloud_work = mask_cylinders(cloud_in);
  }

  // ── 2. 欧式聚类 ──
  auto clusters = extract_clusters(cloud_work);
  const size_t cluster_count = clusters.size();

  // ── 3. 对每个聚类计算特征 ──
  std::vector<Obstacle> raw_obstacles;
  raw_obstacles.reserve(cluster_count);
  for (size_t i = 0; i < cluster_count; ++i) {
    auto obs = compute_obstacle(cloud_work, clusters[i], static_cast<int>(i));
    raw_obstacles.push_back(obs);
  }

  // ── 4. 规则过滤 + 5. 分类猜测 ──
  std::vector<Obstacle> passed;
  passed.reserve(raw_obstacles.size());

  for (auto& obs : raw_obstacles) {
    if (filter_obstacle(obs)) continue;
    classify_obstacle(obs);
    passed.push_back(obs);
  }

  // ── 6. 重新编排 ID (按通过顺序) ──
  for (size_t i = 0; i < passed.size(); ++i) {
    passed[i].id = static_cast<int>(i);
  }

  // ── 耗时 ──
  auto t_end = std::chrono::steady_clock::now();
  using ms = std::chrono::duration<float, std::milli>;
  double total_ms = ms(t_end - t0).count();

  // ── 日志摘要 ──
  RCLCPP_DEBUG(this->get_logger(),
    "Clusters: raw=%zu passed=%zu | %.1f ms",
    cluster_count, passed.size(), total_ms);

  // ── 7. 发布 ──
  publish_obstacles(passed, msg->header);
  publish_diagnostics(total_ms, cluster_count, passed.size(), passed);
}

// ═══════════════════════════════════════════════════════════════════════════
// 欧式聚类
// ═══════════════════════════════════════════════════════════════════════════
std::vector<pcl::PointIndices>
ClusterExtractor::extract_clusters(const PointCloudPtr& cloud,
                                   const Params& params)
{
  std::vector<pcl::PointIndices> clusters;

  if (cloud->size() < static_cast<size_t>(params.min_cluster_size)) {
    return clusters;
  }

  // KdTree
  auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  // ⚠️ PCL 的这三个 setter 收的是 `double` / `int`：`min/max_cluster_size` 在
  //    `Params` 里就是 `int`，别顺手改成 `size_t`（会多一次窄化警告）。
  ec.setClusterTolerance(params.cluster_tolerance);
  ec.setMinClusterSize(params.min_cluster_size);
  ec.setMaxClusterSize(params.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);

  // ⚠️ static 函数拿不到 `this->get_logger()` ⇒ 用同名静态 logger。
  //    这里是 ERROR 级，单测里**会**打到 stderr —— 但只有真抛异常才会走到。
  static const rclcpp::Logger kLogger = rclcpp::get_logger("cluster_extractor");
  try {
    ec.extract(clusters);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(kLogger,
      "EuclideanClusterExtraction exception: %s", e.what());
  }
  catch (...) {
    RCLCPP_ERROR(kLogger,
      "EuclideanClusterExtraction unknown exception");
  }

  return clusters;
}

// ── 成员薄包装：把唯一事实源 `params_` 传进去（回调走这条）──
std::vector<pcl::PointIndices>
ClusterExtractor::extract_clusters(const PointCloudPtr& cloud)
{
  return extract_clusters(cloud, params_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 单聚类 → Obstacle
// ═══════════════════════════════════════════════════════════════════════════
Obstacle ClusterExtractor::compute_obstacle(
    const PointCloudPtr& cloud,
    const pcl::PointIndices& cluster,
    int id,
    const Params& params)
{
  Obstacle obs;
  obs.id = id;

  const auto& indices = cluster.indices;
  if (indices.empty()) return obs;

  // ── 3D 包围盒 (min_pt, max_pt) ──
  float xmin = 1e9, ymin = 1e9, zmin = 1e9;
  float xmax = -1e9, ymax = -1e9, zmax = -1e9;
  float sx = 0, sy = 0, sz = 0;

  for (int idx : indices) {
    const auto& pt = cloud->points[idx];
    if (pt.x < xmin) xmin = pt.x;
    if (pt.y < ymin) ymin = pt.y;
    if (pt.z < zmin) zmin = pt.z;
    if (pt.x > xmax) xmax = pt.x;
    if (pt.y > ymax) ymax = pt.y;
    if (pt.z > zmax) zmax = pt.z;
    sx += pt.x;
    sy += pt.y;
    sz += pt.z;
  }

  obs.min_x = xmin; obs.max_x = xmax;
  obs.min_y = ymin; obs.max_y = ymax;
  obs.min_z = zmin; obs.max_z = zmax;

  obs.length = xmax - xmin;
  obs.width  = ymax - ymin;
  obs.height = zmax - zmin;

  // ── 质心 ──
  const float n = static_cast<float>(indices.size());
  obs.centroid_x = sx / n;
  obs.centroid_y = sy / n;
  obs.centroid_z = sz / n;

  // ── 点数 ──
  obs.point_count = static_cast<int>(indices.size());

  // ── PCA 方向 ──
  compute_pca_orientation(cloud, cluster, obs, params);

  return obs;
}

// ── 成员薄包装 ──
Obstacle ClusterExtractor::compute_obstacle(
    const PointCloudPtr& cloud,
    const pcl::PointIndices& cluster,
    int id)
{
  return compute_obstacle(cloud, cluster, id, params_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 3D PCA: 主轴方向 + 长宽比判断
//
// 协方差矩阵 (3×3):
//   C_ij = Σ (p_i - μ_i)(p_j - μ_j) / N
//
// 特征值 λ1≥λ2≥λ3:
//   λ1/λ2 > elongation_ratio → 长条形, 主轴 = v1
//   λ1≈λ2≈λ3             → 近似球体
//
// 偏航角: yaw = atan2(v1.y, v1.x)  (水平投影)
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::compute_pca_orientation(
    const PointCloudPtr& cloud,
    const pcl::PointIndices& cluster,
    Obstacle& obs,
    const Params& params)
{
  const auto& indices = cluster.indices;
  const size_t n = indices.size();

  // ⚠️ `n < 3` 时**不是**把比值填 0 —— 填 0 会让 `classify_obstacle` 的球体判据
  //    `ratio > 0` 直接判假（"没算出来"被当成"不是球"）。填 1.0 才是"各向同性"。
  if (n < 3) {
    obs.yaw = 0;
    obs.is_elongated = false;
    obs.eigen_ratio_1 = 1.0f;
    obs.eigen_ratio_2 = 1.0f;
    return;
  }

  // ── 质心：**自己算**，不依赖调用方先填 `obs.centroid_*` ──
  // ⚠️ 2026-09-25 修：原实现直接取 `obs.centroid_x/y/z` ⇒ **调用方不先填质心，
  //    PCA 就会静默地绕原点求协方差**，算出一组"看着合理"的错特征值与错主轴
  //    （不报错、不抛异常）。`compute_obstacle` 恰好先填了质心，所以节点路径一直是对的；
  //    但本函数现在是 `public static`、可以被单独调用 ⇒ 这个隐含前提就是个陷阱。
  //    改成自给自足。用 double 累加（比 `compute_obstacle` 的 float 质心更准，差 ~1e-7）。
  double cx = 0.0, cy = 0.0, cz = 0.0;
  for (int idx : indices) {
    const auto& pt = cloud->points[idx];
    cx += static_cast<double>(pt.x);
    cy += static_cast<double>(pt.y);
    cz += static_cast<double>(pt.z);
  }
  const double inv_n = 1.0 / static_cast<double>(n);
  cx *= inv_n;
  cy *= inv_n;
  cz *= inv_n;

  // ── 协方差矩阵 (3×3) ──
  double cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  for (int idx : indices) {
    const auto& pt = cloud->points[idx];
    double dx = pt.x - cx;
    double dy = pt.y - cy;
    double dz = pt.z - cz;
    cov[0][0] += dx * dx;  cov[0][1] += dx * dy;  cov[0][2] += dx * dz;
    cov[1][0] += dy * dx;  cov[1][1] += dy * dy;  cov[1][2] += dy * dz;
    cov[2][0] += dz * dx;  cov[2][1] += dz * dy;  cov[2][2] += dz * dz;
  }

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      cov[i][j] *= inv_n;

  // ── Eigen 分解 ──
  Eigen::Matrix3d C;
  C << cov[0][0], cov[0][1], cov[0][2],
       cov[1][0], cov[1][1], cov[1][2],
       cov[2][0], cov[2][1], cov[2][2];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);
  if (solver.info() != Eigen::Success) {
    obs.yaw = 0;
    obs.is_elongated = false;
    obs.eigen_ratio_1 = 1.0f;
    obs.eigen_ratio_2 = 1.0f;
    return;
  }

  // 特征值: 升序排列 (SelfAdjointEigenSolver 默认升序)
  // eigenvalues()[0] = λ3 (最小), eigenvalues()[2] = λ1 (最大)
  double lambda1 = solver.eigenvalues()[2];  // 最大
  double lambda2 = solver.eigenvalues()[1];
  double lambda3 = solver.eigenvalues()[0];  // 最小

  // 特征向量: 第 2 列 = 最大特征值对应的向量 (主轴)
  Eigen::Vector3d v1 = solver.eigenvectors().col(2);

  // ── 记录 ──
  obs.eigen_ratio_1 = (lambda2 > 1e-9)
    ? static_cast<float>(lambda1 / lambda2) : 1.0f;
  obs.eigen_ratio_2 = (lambda3 > 1e-9)
    ? static_cast<float>(lambda2 / lambda3) : 1.0f;

  // ── 判断是否长条形 ──
  // ⚠️ 严格大于：恰好等于 `elongation_ratio` 时**不算** elongated。
  obs.is_elongated = (obs.eigen_ratio_1 > params.elongation_ratio);

  // ── 偏航角 (主轴水平分量) ──
  // ⚠️ 主轴的**正负号有二义性**：`v1` 与 `−v1` 都是合法特征向量，Eigen 取哪个
  //    取决于内部 LAPACK 的符号约定。⇒ `yaw` 只在**模 π** 意义下有意义
  //    （差 180° 的两条直线方向相同，对外框、对分类都没有影响）。
  //    单测断言必须按"模 π 相等"写，否则会写出一个随 Eigen 版本飘的用例。
  obs.yaw = std::atan2(static_cast<float>(v1.y()),
                       static_cast<float>(v1.x()));
}

// ── 成员薄包装 ──
void ClusterExtractor::compute_pca_orientation(
    const PointCloudPtr& cloud,
    const pcl::PointIndices& cluster,
    Obstacle& obs)
{
  compute_pca_orientation(cloud, cluster, obs, params_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 规则过滤 (ROBOCON 2027 场景)
//
// 过滤条件:
//   - 任一维度 > max_object_size   → 围栏/墙面 (过大)
//   - 高度 > max_object_height     → 柱子 (过高)
//   - 点数 < min_object_points     → 噪点 (过小)
//   - 底部 z > floating_z_min      → 悬空物 (底部无支撑)
//
// @return true = 应剔除
// ═══════════════════════════════════════════════════════════════════════════
bool ClusterExtractor::filter_obstacle(const Obstacle& obs, const Params& params)
{
  // ⚠️ 本函数是 static 纯函数 ⇒ 拿不到 `this->get_logger()`。用**同名静态 logger**
  //    （节点名就是 "cluster_extractor"，日志标签不变）。级别是 DEBUG，
  //    默认控制台级别 INFO ⇒ 平时不打印，单测里也不会刷屏。
  static const rclcpp::Logger kLogger = rclcpp::get_logger("cluster_extractor");

  // 过大物体 (围栏 / 墙面)
  if (obs.length > params.max_object_size || obs.width > params.max_object_size) {
    RCLCPP_DEBUG(kLogger,
      "Filtered LARGE: %.2f x %.2f x %.2f at (%.2f, %.2f)",
      obs.length, obs.width, obs.height, obs.centroid_x, obs.centroid_y);
    return true;
  }

  // 过高物体 (柱子)
  if (obs.height > params.max_object_height) {
    RCLCPP_DEBUG(kLogger,
      "Filtered TALL: h=%.2f at (%.2f, %.2f)",
      obs.height, obs.centroid_x, obs.centroid_y);
    return true;
  }

  // 过小物体 (噪点)
  if (obs.point_count < params.min_object_points) {
    return true;
  }

  // 悬空物 (底部无支撑)
  // ⚠️ `min_z` 的**参考系**由喂进来的点云决定，本函数不做任何变换 ——
  //    见头文件 `floating_z_min` 那条 ⚠️⚠️（雷达系 vs 场地系，两份实现不一致）。
  if (obs.min_z > params.floating_z_min) {
    RCLCPP_DEBUG(kLogger,
      "Filtered FLOATING: z_min=%.2f at (%.2f, %.2f)",
      obs.min_z, obs.centroid_x, obs.centroid_y);
    return true;
  }

  return false;
}

// ── 成员薄包装 ──
bool ClusterExtractor::filter_obstacle(const Obstacle& obs) const
{
  return filter_obstacle(obs, params_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 分类猜测
//
// 基于包围盒尺寸和 PCA 特征值:
//   - 三轴 ≈ earth_cube_size    → EARTH_CUBE   (350mm 立方体)
//   - 三轴 ≈ sky_cube_size      → SKY_CUBE     (200mm 立方体)
//   - 三轴 ≈ enemy_robot_size   → ENEMY_ROBOT  (700mm 立方体)
//   - 三轴 ≈ mustika_size 且特征值接近 → MUSTIKA (200mm 球体)
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::classify_obstacle(Obstacle& obs, const Params& params)
{
  const float tol = static_cast<float>(params.size_tolerance);

  auto in_range = [tol](float val, float expected) -> float {
    float diff = std::abs(val - expected);
    if (diff > tol) return 0.0f;  // 超出容差
    return 1.0f - (diff / tol);   // [0, 1] 越接近越高的匹配度
  };

  // ⚠️ **分支顺序敏感**：从大到小（敌方 0.70 → 地球 0.35 → 天空/穆斯蒂卡 0.20），
  //    第一个 `score > 0.5` 的胜出。改顺序会**静默**改变标签。
  // ⚠️ 2026-09-25 删掉了原地的 `avg_dim` / `max_dev` 两个局部量 ——
  //    它们算了但**从头到尾没被用过**（死代码），注释却写着"三个维度的平均值和最大偏差"
  //    暗示它们参与判定。删掉**不改任何输出**。见台账 §八。

  // ── 敌方机器人 (700mm 立方体) ──
  {
    float score = (in_range(obs.length, static_cast<float>(params.enemy_robot_size)) +
                   in_range(obs.width,  static_cast<float>(params.enemy_robot_size)) +
                   in_range(obs.height, static_cast<float>(params.enemy_robot_size))) / 3.0f;
    if (score > 0.5f) {
      obs.type = ObstacleType::ENEMY_ROBOT;
      obs.confidence = score;
      return;
    }
  }

  // ── 地球方块 (350mm 立方体) ──
  {
    float score = (in_range(obs.length, static_cast<float>(params.earth_cube_size)) +
                   in_range(obs.width,  static_cast<float>(params.earth_cube_size)) +
                   in_range(obs.height, static_cast<float>(params.earth_cube_size))) / 3.0f;
    if (score > 0.5f) {
      obs.type = ObstacleType::EARTH_CUBE;
      obs.confidence = score;
      return;
    }
  }

  // ── 天空方块 (200mm 立方体) vs 穆斯蒂卡 (200mm 球体) ──
  // ⚠️ 两者**尺寸同档（都是 200mm 级）** ⇒ 光看尺寸分不开，必须靠 PCA 特征值：
  //   球体: 三特征值接近 (λ1≈λ2≈λ3) ⇒ 两个比值都≈1
  //   立方体: 特征值有差异 ⇒ 比值偏离 1
  //
  // ⚠️⚠️ **2026-09-25 修**：原实现**只用 `sky_cube_size_` 算 size_score**，
  //    `mustika_size` 这个键**声明了、读了、日志里打了、还能回设，但任何计算都没碰它**
  //    ⇒ **改 yaml 里 `mustika_size` 静默不生效**（§零「稳」轴点名的那一类）。
  //    现在两个尺寸键**各自算自己的匹配度**：`mustika_size` 判"像不像球"，
  //    `sky_cube_size` 判"像不像立方体"。
  //    ⚠️ 今天两值都是 0.20 ⇒ 本次改动**数值零变化**（见单测「两者相等时结论不变」那条）。
  {
    const float cube_score =
        (in_range(obs.length, static_cast<float>(params.sky_cube_size)) +
         in_range(obs.width,  static_cast<float>(params.sky_cube_size)) +
         in_range(obs.height, static_cast<float>(params.sky_cube_size))) / 3.0f;

    const float ball_score =
        (in_range(obs.length, static_cast<float>(params.mustika_size)) +
         in_range(obs.width,  static_cast<float>(params.mustika_size)) +
         in_range(obs.height, static_cast<float>(params.mustika_size))) / 3.0f;

    // 球体判断: 特征值比值都在 [1-tol, 1+tol] 范围内
    // ⚠️ `ratio > 0` 这两条是**防"没算出来"**：`compute_pca_orientation` 在
    //    n<3 / 分解失败时填的是 1.0（各向同性）而不是 0，所以这里挡的是真正的 0。
    const bool is_spherical =
      obs.eigen_ratio_1 > 0.0f &&
      obs.eigen_ratio_2 > 0.0f &&
      std::abs(obs.eigen_ratio_1 - 1.0f) < static_cast<float>(params.sphere_eigen_tol) &&
      std::abs(obs.eigen_ratio_2 - 1.0f) < static_cast<float>(params.sphere_eigen_tol);

    if (is_spherical && ball_score > 0.5f) {
      obs.type = ObstacleType::MUSTIKA;
      obs.confidence = ball_score * 0.85f;  // 略低于立方体, 因为球体判断更不确定
      return;
    }
    if (cube_score > 0.5f) {
      obs.type = ObstacleType::SKY_CUBE;
      obs.confidence = cube_score;
      return;
    }
  }

  // ── 兜底: UNKNOWN ──
  obs.type = ObstacleType::UNKNOWN;
  obs.confidence = 0.0f;
}

// ── 成员薄包装 ──
void ClusterExtractor::classify_obstacle(Obstacle& obs)
{
  classify_obstacle(obs, params_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 圆柱掩膜
//
// 将圆柱中心 (参数配置) 周围 `cylinder_mask_radius` 内的点移除.
// 用于避免圆柱残留点被误聚类为障碍物.
// ═══════════════════════════════════════════════════════════════════════════
ClusterExtractor::PointCloudPtr
ClusterExtractor::mask_cylinders(const PointCloudPtr& cloud, const Params& params)
{
  const size_t n_cylinders = params.cylinder_centers.size();
  // ⚠️ 没配圆柱 ⇒ **原样返回入参指针**（不是复制一份空壳）：调用方拿到的是同一块内存，
  //    下游 `extract_clusters` 只读它，安全；但别在这里改成"返回新云"。
  if (n_cylinders == 0) return cloud;

  const float r_sq = static_cast<float>(
      params.cylinder_mask_radius * params.cylinder_mask_radius);

  auto filtered = std::make_shared<PointCloud>();
  filtered->reserve(cloud->size());

  size_t removed = 0;
  for (const auto& pt : cloud->points) {
    bool inside = false;
    for (size_t i = 0; i < n_cylinders; ++i) {
      float dx = pt.x - static_cast<float>(params.cylinder_centers[i].first);
      float dy = pt.y - static_cast<float>(params.cylinder_centers[i].second);
      // ⚠️ 只判**水平**距离（不判 z）⇒ 圆柱正上方的点也会被删掉；
      //    边界是**严格小于** ⇒ 恰好落在半径上的点**保留**。
      if (dx * dx + dy * dy < r_sq) {
        inside = true;
        break;
      }
    }
    if (inside) {
      removed++;
    } else {
      filtered->push_back(pt);
    }
  }

  if (removed > 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("cluster_extractor"),
      "Cylinder mask removed %zu/%zu points", removed, cloud->size());
  }

  return filtered;
}

// ── 成员薄包装 ──
ClusterExtractor::PointCloudPtr
ClusterExtractor::mask_cylinders(const PointCloudPtr& cloud)
{
  return mask_cylinders(cloud, params_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: MarkerArray (包围盒 + 文本标签)
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::publish_obstacles(
    const std::vector<Obstacle>& obstacles,
    const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray arr;

  for (const auto& obs : obstacles) {
    // ── 包围盒 CUBE ──
    {
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.ns     = "obstacles";
      m.id     = obs.id;
      m.type   = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;

      // 中心位置
      m.pose.position.x = obs.centroid_x;
      m.pose.position.y = obs.centroid_y;
      m.pose.position.z = obs.centroid_z;

      // 方向 (绕 Z 轴旋转 yaw)
      m.pose.orientation.w = std::cos(obs.yaw * 0.5f);
      m.pose.orientation.x = 0.0;
      m.pose.orientation.y = 0.0;
      m.pose.orientation.z = std::sin(obs.yaw * 0.5f);

      // 尺寸 (使用包围盒尺寸)
      m.scale.x = std::max(obs.length, 0.02f);
      m.scale.y = std::max(obs.width,  0.02f);
      m.scale.z = std::max(obs.height, 0.02f);

      // 颜色 (按分类)
      switch (obs.type) {
        case ObstacleType::EARTH_CUBE:
          m.color.r = 0.16f; m.color.g = 0.39f; m.color.b = 0.20f; break;  // 深绿
        case ObstacleType::SKY_CUBE:
          m.color.r = 0.20f; m.color.g = 0.60f; m.color.b = 0.86f; break;  // 天蓝
        case ObstacleType::ENEMY_ROBOT:
          m.color.r = 0.85f; m.color.g = 0.20f; m.color.b = 0.20f; break;  // 红
        case ObstacleType::MUSTIKA:
          m.color.r = 0.85f; m.color.g = 0.65f; m.color.b = 0.13f; break;  // 金
        default:
          m.color.r = 0.5f; m.color.g = 0.5f; m.color.b = 0.5f; break;     // 灰
      }
      m.color.a = 0.5f;  // 半透明
      m.lifetime = rclcpp::Duration::from_seconds(1.0);
      arr.markers.push_back(m);
    }

    // ── 分类标签 TEXT ──
    {
      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns     = "obstacle_labels";
      text.id     = obs.id + 10000;
      text.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;

      text.pose.position.x = obs.centroid_x;
      text.pose.position.y = obs.centroid_y;
      text.pose.position.z = obs.max_z + 0.10f;  // 浮在顶部上方
      text.pose.orientation.w = 1.0;

      text.scale.z = 0.08f;  // 文字高度 8cm

      // 文本: 类型 + 置信度
      char buf[64];
      snprintf(buf, sizeof(buf), "%s (%.0f%%)",
               obstacle_type_name(obs.type), obs.confidence * 100.0f);
      text.text = buf;

      text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.lifetime = rclcpp::Duration::from_seconds(1.0);
      arr.markers.push_back(text);
    }
  }

  // ── 清理过期 marker (DELETEALL 在无障碍物时清除) ──
  if (obstacles.empty()) {
    visualization_msgs::msg::Marker del;
    del.header = header;
    del.ns     = "obstacles";
    del.id     = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    visualization_msgs::msg::Marker del2;
    del2.header = header;
    del2.ns     = "obstacle_labels";
    del2.id     = 0;
    del2.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del2);
  }

  pub_obstacles_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: Diagnostics
// ═══════════════════════════════════════════════════════════════════════════
void ClusterExtractor::publish_diagnostics(
    double total_ms,
    size_t cluster_count,
    size_t passed_count,
    const std::vector<Obstacle>& obstacles)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "cluster_extractor";
  st.hardware_id = "mid360";

  if (total_ms < 15.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "OK";
  } else if (total_ms < 25.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "SLOW";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "VERY SLOW";
  }

  auto add = [&](const std::string& k, double v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k; kv.value = std::to_string(v);
    st.values.push_back(kv);
  };

  add("total_ms",       total_ms);
  add("cluster_count",  static_cast<double>(cluster_count));
  add("passed_count",   static_cast<double>(passed_count));
  add("filtered_count", static_cast<double>(cluster_count - passed_count));

  // 各类别计数
  std::unordered_map<ObstacleType, int> type_counts;
  for (const auto& obs : obstacles) {
    type_counts[obs.type]++;
  }
  add("earth_cube",   static_cast<double>(type_counts[ObstacleType::EARTH_CUBE]));
  add("sky_cube",     static_cast<double>(type_counts[ObstacleType::SKY_CUBE]));
  add("enemy_robot",  static_cast<double>(type_counts[ObstacleType::ENEMY_ROBOT]));
  add("mustika",      static_cast<double>(type_counts[ObstacleType::MUSTIKA]));
  add("unknown",      static_cast<double>(type_counts[ObstacleType::UNKNOWN]));

  arr.status.push_back(st);
  pub_diagnostics_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult ClusterExtractor::on_param_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool cylinder_centers_changed = false;

  for (const auto& p : params) {
    const std::string& name = p.get_name();

    try {
      #define SET(n, v)              if (name == n) v = p.as_double()
      #define SET_I(n, v)            if (name == n) v = p.as_int()
      #define SET_B(n, v)            if (name == n) v = p.as_bool()
      #define ARRAY_CHANGED(n)       if (name == n) cylinder_centers_changed = true

      SET("cluster_tolerance",    params_.cluster_tolerance);
      SET_I("min_cluster_size",   params_.min_cluster_size);
      SET_I("max_cluster_size",   params_.max_cluster_size);
      SET("max_object_size",      params_.max_object_size);
      SET("max_object_height",    params_.max_object_height);
      SET_I("min_object_points",  params_.min_object_points);
      SET("floating_z_min",       params_.floating_z_min);
      SET_B("enable_cylinder_mask", params_.enable_cylinder_mask);
      SET("cylinder_mask_radius", params_.cylinder_mask_radius);
      SET("size_tolerance",       params_.size_tolerance);
      SET("earth_cube_size",      params_.earth_cube_size);
      SET("sky_cube_size",        params_.sky_cube_size);
      SET("enemy_robot_size",     params_.enemy_robot_size);
      SET("mustika_size",         params_.mustika_size);
      SET("elongation_ratio",     params_.elongation_ratio);
      SET("sphere_eigen_tol",     params_.sphere_eigen_tol);

      ARRAY_CHANGED("cylinder_mask_x");
      ARRAY_CHANGED("cylinder_mask_y");

      #undef SET
      #undef SET_I
      #undef SET_B
      #undef ARRAY_CHANGED
    }
    catch (const rclcpp::ParameterTypeException& e) {
      result.successful = false;
      result.reason = std::string("Type mismatch: ") + name + " — " + e.what();
      break;
    }
  }

  // 圆柱中心数组变更 → 重新读取并重建 pair 向量
  if (cylinder_centers_changed && result.successful) {
    std::vector<double> cx, cy;
    this->get_parameter("cylinder_mask_x", cx);
    this->get_parameter("cylinder_mask_y", cy);
    if (cx.size() == cy.size()) {
      params_.cylinder_centers.clear();
      params_.cylinder_centers.reserve(cx.size());
      for (size_t i = 0; i < cx.size(); ++i) {
        params_.cylinder_centers.emplace_back(cx[i], cy[i]);
      }
    } else {
      result.successful = false;
      result.reason = "cylinder_mask_x and cylinder_mask_y must have same length";
    }
  }

  if (result.successful) {
    RCLCPP_DEBUG(this->get_logger(),
      "Params updated | tol=%.3f min=%d max=%d | "
      "filter: size=%.2f h=%.2f pts=%d floating=%.2f | "
      "classify: tol=%.3f earth=%.2f sky=%.2f robot=%.2f mustika=%.2f",
      params_.cluster_tolerance, params_.min_cluster_size, params_.max_cluster_size,
      params_.max_object_size, params_.max_object_height, params_.min_object_points,
      params_.floating_z_min,
      params_.size_tolerance, params_.earth_cube_size, params_.sky_cube_size,
      params_.enemy_robot_size, params_.mustika_size);
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

}  // namespace lidar
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::lidar::ClusterExtractor)
