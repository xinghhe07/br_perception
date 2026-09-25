#include "br_perception/lidar/cylinder_detector.hpp"

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <random>
#include <thread>
#include <unordered_set>

namespace br_perception {
namespace lidar {

// ═══════════════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════════════
CylinderDetector::CylinderDetector(const rclcpp::NodeOptions& options)
  : Node("cylinder_detector", options)
{
  #define DECL(name, var) this->declare_parameter<double>(name, var)
  #define DECL_I(name, var) this->declare_parameter<int>(name, var)
  #define DECL_B(name, var) this->declare_parameter<bool>(name, var)

  DECL_I("normal_k_search",       normal_k_search_);
  DECL_I("normal_omp_threads",    normal_omp_threads_);
  DECL("slice_thickness",         slice_thickness_);
  DECL_I("slice_min_points",      slice_min_points_);
  DECL("circle_fit_thresh",       circle_fit_thresh_);
  DECL_I("circle_fit_iters",      circle_fit_iters_);
  DECL("radius_min",              radius_min_);
  DECL("radius_max",              radius_max_);
  DECL("merge_center_tol",        merge_center_tol_);
  DECL("merge_radius_tol",        merge_radius_tol_);
  DECL_I("merge_min_slices",      merge_min_slices_);
  DECL("min_height",              min_height_);
  DECL("cyl3d_dist_thresh",       cyl3d_dist_thresh_);
  DECL_I("cyl3d_max_iters",       cyl3d_max_iters_);
  DECL("cyl3d_normal_weight",     cyl3d_normal_weight_);
  DECL("cyl3d_axis_eps",          cyl3d_axis_eps_);
  DECL("mustika_h_min",           mustika_h_min_);
  DECL("mustika_h_max",           mustika_h_max_);
  DECL("core_h_min",              core_h_min_);
  DECL("core_h_max",              core_h_max_);
  DECL("top_z_offset",            top_z_offset_);
  DECL("top_z_range",             top_z_range_);
  DECL_I("top_min_pts",           top_min_pts_);
  DECL("top_cluster_tol",         top_cluster_tol_);
  DECL_B("geo_check_enabled",     geo_check_enabled_);
  DECL("geo_match_tol",           geo_match_tol_);
  #undef DECL
  #undef DECL_I
  #undef DECL_B

  #define GET(name, var) this->get_parameter(name, var)
  GET("normal_k_search",    normal_k_search_);
  GET("normal_omp_threads", normal_omp_threads_);
  GET("slice_thickness",    slice_thickness_);
  GET("slice_min_points",   slice_min_points_);
  GET("circle_fit_thresh",  circle_fit_thresh_);
  GET("circle_fit_iters",   circle_fit_iters_);
  GET("radius_min",         radius_min_);
  GET("radius_max",         radius_max_);
  GET("merge_center_tol",   merge_center_tol_);
  GET("merge_radius_tol",   merge_radius_tol_);
  GET("merge_min_slices",   merge_min_slices_);
  GET("min_height",         min_height_);
  GET("cyl3d_dist_thresh",  cyl3d_dist_thresh_);
  GET("cyl3d_max_iters",    cyl3d_max_iters_);
  GET("cyl3d_normal_weight",cyl3d_normal_weight_);
  GET("cyl3d_axis_eps",     cyl3d_axis_eps_);
  GET("mustika_h_min",      mustika_h_min_);
  GET("mustika_h_max",      mustika_h_max_);
  GET("core_h_min",         core_h_min_);
  GET("core_h_max",         core_h_max_);
  GET("top_z_offset",       top_z_offset_);
  GET("top_z_range",        top_z_range_);
  GET("top_min_pts",        top_min_pts_);
  GET("top_cluster_tol",    top_cluster_tol_);
  GET("geo_check_enabled",  geo_check_enabled_);
  GET("geo_match_tol",      geo_match_tol_);
  #undef GET

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CylinderDetector::on_param_change, this, std::placeholders::_1));

  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/lidar/non_ground", rclcpp::SensorDataQoS(),
    std::bind(&CylinderDetector::cloud_callback, this, std::placeholders::_1));

  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/lidar/pillars", 10);
  pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/lidar/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "CylinderDetector ready | PointNormal pipeline | "
    "slice=%.2fm | 2D RANSAC thresh=%.2fm iters=%d | "
    "3D RANSAC thresh=%.2fm iters=%d norm_w=%.2f | "
    "OMP threads=%d",
    slice_thickness_, circle_fit_thresh_, circle_fit_iters_,
    cyl3d_dist_thresh_, cyl3d_max_iters_, cyl3d_normal_weight_,
    normal_omp_threads_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 主回调
// ═══════════════════════════════════════════════════════════════════════════
void CylinderDetector::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t0 = std::chrono::steady_clock::now();

  // ── 解包 XYZ → 转换 PointNormal ──
  CloudXYZPtr xyz_in = std::make_shared<CloudXYZ>();
  pcl::fromROSMsg(*msg, *xyz_in);

  if (xyz_in->size() < 50) return;

  PointCloudPtr cloud = std::make_shared<PointCloud>();
  cloud->reserve(xyz_in->size());
  for (const auto& p : xyz_in->points) {
    PointT pn;
    pn.x = p.x; pn.y = p.y; pn.z = p.z;
    pn.normal_x = 0; pn.normal_y = 0; pn.normal_z = 0;
    pn.curvature = 0;
    cloud->push_back(pn);
  }

  // ── 法向量 (OMP 并行, 两个方法共用) ──
  estimate_normals(cloud);
  auto t_normals = std::chrono::steady_clock::now();

  // ── 方法 A: 水平切片 + 2D 圆拟合 ──
  auto slices = slice_horizontally(cloud);

  std::vector<std::tuple<float,float,float,bool>> circle_results;
  circle_results.reserve(slices.size());

  for (auto& sl : slices) {
    if (static_cast<int>(sl.point_indices.size()) < slice_min_points_) {
      circle_results.emplace_back(0.f, 0.f, 0.f, false);
      continue;
    }
    float cx, cy, r;
    std::vector<size_t> inliers;
    bool ok = fit_circle_ransac_2d(cloud, sl.point_indices, cx, cy, r, inliers);
    circle_results.emplace_back(cx, cy, r, ok);
  }

  auto cylinders_a = merge_slices(cloud, slices, circle_results);

  // ── 方法 B: RANSAC 3D 圆柱拟合 ──
  auto cylinders_b = detect_cylinders_ransac_3d(cloud);

  // ── 合并去重 ──
  auto cylinders = deduplicate(std::move(cylinders_a), std::move(cylinders_b));

  // ── 分类 ──
  classify_by_height(cylinders);

  // ── 顶部检测 ──
  check_top_objects(cylinders, cloud);

  // ── 几何一致性 ──
  geometry_consistency_check(cylinders);

  // ── 发布 ──
  publish_markers(cylinders, msg->header);

  auto t_end = std::chrono::steady_clock::now();
  using ms = std::chrono::duration<float, std::milli>;
  double total_ms   = ms(t_end - t0).count();
  double normals_ms = ms(t_normals - t0).count();
  publish_diagnostics(total_ms, normals_ms, cylinders);
}

// ═══════════════════════════════════════════════════════════════════════════
// 法向量估计 (OMP 并行)
// ═══════════════════════════════════════════════════════════════════════════
void CylinderDetector::estimate_normals(PointCloudPtr& cloud)
{
  if (cloud->size() < 3) return;

  // 用 pcl::NormalEstimationOMP 替代单线程版
  // PointNormal 的 normal_* 字段会被直接填充为 pcl::Normal
  auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
  tree->setInputCloud(cloud);

  // NormalEstimationOMP 的模板: <PointInT, PointOutT>
  // 这里 PointInT=PointT(pcl::PointNormal), PointOutT=PointT
  // pcl::NormalEstimationOMP 内部会操作 normal_x/normal_y/normal_z/curvature 字段
  pcl::NormalEstimationOMP<PointT, PointT> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(normal_k_search_);
  // OMP 线程数不超过硬件核心数
  const int max_threads = static_cast<int>(std::thread::hardware_concurrency());
  const int threads = std::min(normal_omp_threads_, max_threads > 0 ? max_threads : 4);
  ne.setNumberOfThreads(threads);
  ne.compute(*cloud);  // 原地写入法向量到 PointNormal 字段
}

// ═══════════════════════════════════════════════════════════════════════════
// 方法 A: 水平切片
// ═══════════════════════════════════════════════════════════════════════════
std::vector<CylinderDetector::SliceInfo>
CylinderDetector::slice_horizontally(const PointCloudPtr& cloud)
{
  std::vector<SliceInfo> result;
  if (cloud->empty()) return result;

  // Z 范围
  float z_min = cloud->points[0].z, z_max = cloud->points[0].z;
  for (const auto& p : cloud->points) {
    if (p.z < z_min) z_min = p.z;
    if (p.z > z_max) z_max = p.z;
  }

  const float dz = static_cast<float>(slice_thickness_);
  const int N = std::max(1, static_cast<int>((z_max - z_min) / dz + 1));
  result.resize(N);

  for (int i = 0; i < N; ++i) {
    result[i].z_center = z_min + dz * i + dz * 0.5f;
  }

  for (size_t pi = 0; pi < cloud->size(); ++pi) {
    int idx = static_cast<int>((cloud->points[pi].z - z_min) / dz);
    idx = std::clamp(idx, 0, N - 1);
    result[idx].point_indices.push_back(pi);
  }

  return result;
}

// ═══════════════════════════════════════════════════════════════════════════
// 方法 A: RANSAC 2D 圆拟合
// ═══════════════════════════════════════════════════════════════════════════
bool CylinderDetector::fit_circle_ransac_2d(
    const PointCloudPtr& cloud,
    const std::vector<size_t>& indices,
    float& best_cx, float& best_cy, float& best_r,
    std::vector<size_t>& best_inliers) const
{
  const size_t n = indices.size();
  if (n < 3) return false;

  // 预取 2D 坐标
  std::vector<float> xs(n), ys(n);
  for (size_t i = 0; i < n; ++i) {
    xs[i] = cloud->points[indices[i]].x;
    ys[i] = cloud->points[indices[i]].y;
  }

  const float thresh_sq = circle_fit_thresh_ * circle_fit_thresh_;
  const float r_min_sq  = radius_min_ * radius_min_;
  const float r_max_sq  = radius_max_ * radius_max_;

  size_t best_cnt = 0;
  best_cx = best_cy = best_r = 0.f;

  std::mt19937 rng(42);
  std::uniform_int_distribution<size_t> dist(0, n - 1);
  // n 较大时 n*n/6 会溢出 int, 直接用参数值并限制不超过 n*3
  const int iters = std::min({circle_fit_iters_, static_cast<int>(n * 3), 200});

  for (int iter = 0; iter < iters; ++iter) {
    size_t i1 = dist(rng);
    size_t i2 = dist(rng); while (i2 == i1) i2 = dist(rng);
    size_t i3 = dist(rng); while (i3 == i1 || i3 == i2) i3 = dist(rng);

    const float x1 = xs[i1], y1 = ys[i1];
    const float x2 = xs[i2], y2 = ys[i2];
    const float x3 = xs[i3], y3 = ys[i3];

    const float d = 2.f * (x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2));
    if (std::abs(d) < 1e-9f) continue;

    const float s1 = x1*x1 + y1*y1;
    const float s2 = x2*x2 + y2*y2;
    const float s3 = x3*x3 + y3*y3;

    const float cx = (s1*(y2-y3) + s2*(y3-y1) + s3*(y1-y2)) / d;
    const float cy = (s1*(x3-x2) + s2*(x1-x3) + s3*(x2-x1)) / d;
    const float r_sq = (cx-x1)*(cx-x1) + (cy-y1)*(cy-y1);

    if (r_sq < r_min_sq || r_sq > r_max_sq) continue;

    const float r = std::sqrt(r_sq);

    // 计内点
    size_t cnt = 0;
    for (size_t j = 0; j < n; ++j) {
      const float dx = xs[j] - cx, dy = ys[j] - cy;
      const float dist_to_circle = std::abs(std::sqrt(dx*dx + dy*dy) - r);
      if (dist_to_circle * dist_to_circle < thresh_sq) cnt++;
    }

    if (cnt > best_cnt) {
      best_cnt = cnt;
      best_cx = cx; best_cy = cy; best_r = r;
    }
  }

  if (best_cnt < 4) return false;

  // 重新收集最优内点索引
  best_inliers.clear();
  best_inliers.reserve(best_cnt);
  for (size_t j = 0; j < n; ++j) {
    const float dx = xs[j] - best_cx, dy = ys[j] - best_cy;
    const float dist = std::abs(std::sqrt(dx*dx + dy*dy) - best_r);
    if (dist * dist < thresh_sq) {
      best_inliers.push_back(indices[j]);
    }
  }

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// 方法 A: 合并同心切片 → 圆柱
// ═══════════════════════════════════════════════════════════════════════════
std::vector<CylinderDetection>
CylinderDetector::merge_slices(
    const PointCloudPtr& cloud,
    const std::vector<SliceInfo>& slices,
    const std::vector<std::tuple<float,float,float,bool>>& circle_results)
{
  std::vector<CylinderDetection> out;

  struct Group { float sum_cx=0,sum_cy=0,sum_r=0,z_bot=0,z_top=0; int n=0; };
  std::vector<Group> groups;
  int cur_idx = -1;  // groups 中当前活跃组的索引, -1 = 无

  for (size_t i = 0; i < slices.size(); ++i) {
    const auto& [cx, cy, r, ok] = circle_results[i];
    if (!ok) { cur_idx = -1; continue; }

    const float half_dz = static_cast<float>(slice_thickness_ * 0.5);

    if (cur_idx < 0) {
      groups.push_back({cx, cy, r,
                        slices[i].z_center - half_dz,
                        slices[i].z_center + half_dz,
                        1});
      cur_idx = static_cast<int>(groups.size()) - 1;
    } else {
      auto& g = groups[cur_idx];
      float acx = g.sum_cx/g.n, acy = g.sum_cy/g.n, ar = g.sum_r/g.n;
      float dc = std::sqrt((cx-acx)*(cx-acx) + (cy-acy)*(cy-acy));
      if (dc < merge_center_tol_ && std::abs(r-ar) < merge_radius_tol_) {
        g.sum_cx+=cx; g.sum_cy+=cy; g.sum_r+=r;
        g.z_top = slices[i].z_center + half_dz;
        g.n++;
      } else {
        groups.push_back({cx, cy, r,
                          slices[i].z_center - half_dz,
                          slices[i].z_center + half_dz,
                          1});
        cur_idx = static_cast<int>(groups.size()) - 1;
      }
    }
  }

  for (const auto& g : groups) {
    if (g.n < merge_min_slices_) continue;
    float h = g.z_top - g.z_bot;
    if (h < min_height_) continue;

    CylinderDetection c;
    c.center_x = g.sum_cx/g.n; c.center_y = g.sum_cy/g.n;
    c.radius   = g.sum_r/g.n;
    c.bottom_z = g.z_bot; c.top_z = g.z_top;
    c.height   = h;
    c.confidence = std::min(1.f, g.n / 10.f);
    c.type     = CylinderDetection::UNKNOWN;

    // 用切片内点的平均法向量计算轴方向
    c.axis_nx = 0; c.axis_ny = 0; c.axis_nz = 1;  // 默认竖直
    static int gid = 0; c.id = gid++;
    out.push_back(c);
  }

  return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// 方法 B: RANSAC 3D 圆柱拟合 (pcl::SACMODEL_CYLINDER + 法向量)
// ═══════════════════════════════════════════════════════════════════════════
std::vector<CylinderDetection>
CylinderDetector::detect_cylinders_ransac_3d(const PointCloudPtr& cloud)
{
  std::vector<CylinderDetection> out;
  if (cloud->size() < 20) return out;

  // 注意: pcl::SACSegmentationFromNormals 的模板参数是 <PointT, pcl::Normal>
  // PointT 是 pcl::PointNormal (同时含 xyz + normal)
  pcl::SACSegmentationFromNormals<PointT, PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(cyl3d_dist_thresh_);
  seg.setMaxIterations(cyl3d_max_iters_);
  seg.setRadiusLimits(radius_min_, radius_max_);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(cyl3d_axis_eps_);
  seg.setNormalDistanceWeight(cyl3d_normal_weight_);

  // 工作副本 (SACSegmentation 会修改输入标志)
  PointCloudPtr work = std::make_shared<PointCloud>(*cloud);
  seg.setInputCloud(work);
  seg.setInputNormals(work);  // PointNormal = 自带法向量的点

  // 统一 ID 分配 (跨方法 A/B 不冲突)
  static int global_id = 0;

  // 迭代提取多个圆柱 (每次提取一个, 从点云中移除内点)
  const int max_cylinders = 3;
  for (int cyl_idx = 0; cyl_idx < max_cylinders && work->size() > 20; ++cyl_idx) {
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coeff;
    seg.setInputCloud(work);
    seg.setInputNormals(work);
    seg.segment(inliers, coeff);

    if (inliers.indices.empty() || coeff.values.size() < 7) break;

    // ── 验证阶段: 每一项失败都移除内点后 continue, 不做 goto ──
    bool accept = true;

    // coeff.values: point_on_axis[3], axis_direction[3], radius
    float ax = coeff.values[3], ay = coeff.values[4], az = coeff.values[5];
    float axis_norm = std::sqrt(ax*ax + ay*ay + az*az);
    if (axis_norm < 1e-6f) { accept = false; }
    else {
      ax /= axis_norm; ay /= axis_norm; az /= axis_norm;
      // 轴方向检查: 是否接近竖直
      if (std::abs(az) < 0.80f) { accept = false; }
    }

    // 从内点计算 Z 范围 & 高度
    float z_min = 1e9f, z_max = -1e9f;
    if (accept) {
      for (int idx : inliers.indices) {
        float z = work->points[idx].z;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
      }
      if (z_max - z_min < min_height_) { accept = false; }
    }

    if (accept) {
      CylinderDetection c;
      c.center_x = coeff.values[0];
      c.center_y = coeff.values[1];
      c.bottom_z = z_min; c.top_z = z_max;
      c.height   = z_max - z_min;
      c.radius   = coeff.values[6];
      c.axis_nx  = ax; c.axis_ny = ay; c.axis_nz = az;
      c.confidence = 0.8f;
      c.type     = CylinderDetection::UNKNOWN;
      c.id       = global_id++;
      out.push_back(c);
    }

    // 无论接受与否, 移除本次内点, 继续搜索下一个圆柱
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(work);
    extract.setIndices(std::make_shared<pcl::PointIndices>(inliers));
    extract.setNegative(true);
    PointCloudPtr remaining = std::make_shared<PointCloud>();
    extract.filter(*remaining);
    work = remaining;
  }

  return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// 去重: 两方法的结果合并, 5cm 内视为同一圆柱, 保留置信度更高的
// ═══════════════════════════════════════════════════════════════════════════
std::vector<CylinderDetection>
CylinderDetector::deduplicate(
    std::vector<CylinderDetection>&& a,
    std::vector<CylinderDetection>&& b)
{
  std::vector<CylinderDetection> merged;
  merged.reserve(a.size() + b.size());
  for (auto& c : a) merged.push_back(std::move(c));
  for (auto& c : b) merged.push_back(std::move(c));

  if (merged.size() <= 1) return merged;

  // 按置信度降序
  std::sort(merged.begin(), merged.end(),
    [](const auto& x, const auto& y) { return x.confidence > y.confidence; });

  std::vector<bool> keep(merged.size(), true);
  for (size_t i = 0; i < merged.size(); ++i) {
    if (!keep[i]) continue;
    for (size_t j = i + 1; j < merged.size(); ++j) {
      if (!keep[j]) continue;
      float dx = merged[i].center_x - merged[j].center_x;
      float dy = merged[i].center_y - merged[j].center_y;
      float dr = std::abs(merged[i].radius - merged[j].radius);
      // 中心靠近 AND 半径接近 → 同一圆柱, 保留高置信度的
      if (std::sqrt(dx*dx + dy*dy) < merge_center_tol_ &&
          dr < merge_radius_tol_) {
        keep[j] = false;
      }
    }
  }

  std::vector<CylinderDetection> deduped;
  for (size_t i = 0; i < merged.size(); ++i) {
    if (keep[i]) deduped.push_back(std::move(merged[i]));
  }
  return deduped;
}

// ═══════════════════════════════════════════════════════════════════════════
// 按高度分类
// ═══════════════════════════════════════════════════════════════════════════
void CylinderDetector::classify_by_height(std::vector<CylinderDetection>& cyls,
                                          const HeightClassParams& params)
{
  for (auto& c : cyls) {
    // ⚠️ 闭区间（`>=` / `<=`）：恰好等于端点算在内。
    if (c.height >= params.mustika_h_min && c.height <= params.mustika_h_max) {
      c.type = CylinderDetection::MUSTIKA_PILLAR;
      // ⚠️ `std::max` ⇒ **只抬不降**。写成直接赋值会把上游更可靠的置信度**压下来**。
      c.confidence = std::max(c.confidence, params.classified_confidence);
    } else if (c.height >= params.core_h_min && c.height <= params.core_h_max) {
      c.type = CylinderDetection::CORE_PILLAR;
      c.confidence = std::max(c.confidence, params.classified_confidence);
    }
    // ⚠️ **落在两段之外 ⇒ 什么都不做**（`type` 保持原值，通常是 UNKNOWN）。
    //    中间的 0.60~0.70 空档是**刻意**的（见头文件）：填上它 = 给每个噪声团贴标签。
  }
}

// ── 成员薄包装：行为逐位不变 ──
void CylinderDetector::classify_by_height(std::vector<CylinderDetection>& cyls)
{
  HeightClassParams p;
  p.mustika_h_min = mustika_h_min_;
  p.mustika_h_max = mustika_h_max_;
  p.core_h_min    = core_h_min_;
  p.core_h_max    = core_h_max_;
  classify_by_height(cyls, p);
}

// ═══════════════════════════════════════════════════════════════════════════
// 顶部检测
// ═══════════════════════════════════════════════════════════════════════════
void CylinderDetector::check_top_objects(
    std::vector<CylinderDetection>& cyls, const PointCloudPtr& cloud)
{
  for (auto& c : cyls) {
    const float z_lo = c.top_z + top_z_offset_;
    const float z_hi = z_lo + top_z_range_;
    const float r_sq_ext = (c.radius + 0.08f) * (c.radius + 0.08f);

    // 收集顶部区域点
    CloudXYZPtr top_pts = std::make_shared<CloudXYZ>();
    for (const auto& p : cloud->points) {
      if (p.z < z_lo || p.z > z_hi) continue;
      float dx = p.x - c.center_x, dy = p.y - c.center_y;
      if (dx*dx + dy*dy < r_sq_ext) {
        top_pts->push_back(pcl::PointXYZ{p.x, p.y, p.z});
      }
    }

    if (static_cast<int>(top_pts->size()) < top_min_pts_) {
      c.has_top_object = false; continue;
    }

    // 欧式聚类
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    tree->setInputCloud(top_pts);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(top_cluster_tol_);
    ec.setMinClusterSize(top_min_pts_);
    ec.setMaxClusterSize(top_pts->size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(top_pts);
    ec.extract(clusters);

    if (!clusters.empty()) {
      c.has_top_object = true;
      c.top_object_points = clusters[0].indices.size();

      // 估算顶部物体半径
      float sr = 0;
      for (int idx : clusters[0].indices) {
        float dx = top_pts->points[idx].x - c.center_x;
        float dy = top_pts->points[idx].y - c.center_y;
        sr += std::sqrt(dx*dx + dy*dy);
      }
      c.top_object_radius = sr / clusters[0].indices.size();
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 几何一致性校验
// ═══════════════════════════════════════════════════════════════════════════
void CylinderDetector::geometry_consistency_check(
    std::vector<CylinderDetection>& cyls)
{
  if (!geo_check_enabled_) return;

  for (auto& c : cyls) {
    c.matches_expected = false;
    for (const auto& ep : expected_) {
      if (c.type != CylinderDetection::UNKNOWN && c.type != ep.type) continue;
      float dist = std::sqrt((c.center_x-ep.x)*(c.center_x-ep.x) +
                             (c.center_y-ep.y)*(c.center_y-ep.y));
      if (dist < geo_match_tol_) {
        c.matches_expected = true;
        c.position_error = dist;
        c.confidence = std::min(1.f, c.confidence + 0.2f);
        break;
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: MarkerArray
// ═══════════════════════════════════════════════════════════════════════════
void CylinderDetector::publish_markers(
    const std::vector<CylinderDetection>& cyls,
    const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray arr;

  for (const auto& c : cyls) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns     = "pillars";
    m.id     = c.id;
    m.type   = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = c.center_x;
    m.pose.position.y = c.center_y;
    m.pose.position.z = (c.bottom_z + c.top_z) * 0.5f;
    m.pose.orientation.w = 1.0;
    m.scale.x = c.radius * 2.f;
    m.scale.y = c.radius * 2.f;
    m.scale.z = c.height;

    if (c.type == CylinderDetection::MUSTIKA_PILLAR) {
      m.color.r = 0.16f; m.color.g = 0.39f; m.color.b = 0.20f;  // 深绿
    } else if (c.type == CylinderDetection::CORE_PILLAR) {
      m.color.r = 0.39f; m.color.g = 0.24f; m.color.b = 0.00f;  // 棕
    } else {
      m.color.r = 0.5f; m.color.g = 0.5f; m.color.b = 0.5f;
    }
    m.color.a = c.has_top_object ? 1.0f : 0.6f;
    arr.markers.push_back(m);

    // 顶部物体球
    if (c.has_top_object) {
      visualization_msgs::msg::Marker ball;
      ball.header = header;
      ball.ns     = "top_objects";
      ball.id     = c.id + 10000;
      ball.type   = visualization_msgs::msg::Marker::SPHERE;
      ball.action = visualization_msgs::msg::Marker::ADD;
      ball.pose.position.x = c.center_x;
      ball.pose.position.y = c.center_y;
      ball.pose.position.z = c.top_z + 0.10f;
      ball.scale.x = c.top_object_radius * 2.f;
      ball.scale.y = c.top_object_radius * 2.f;
      ball.scale.z = c.top_object_radius * 2.f;
      ball.color.r = 0.85f; ball.color.g = 0.65f; ball.color.b = 0.13f;
      ball.color.a = 1.f;
      arr.markers.push_back(ball);
    }
  }

  pub_markers_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: Diagnostics
// ═══════════════════════════════════════════════════════════════════════════
void CylinderDetector::publish_diagnostics(
    double total_ms, double normals_ms,
    const std::vector<CylinderDetection>& cyls)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "cylinder_detector";
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
    diagnostic_msgs::msg::KeyValue kv; kv.key = k;
    kv.value = std::to_string(v); st.values.push_back(kv);
  };

  add("total_ms",    total_ms);
  add("normals_ms",  normals_ms);

  int mc = 0, cc = 0, tc = 0;
  for (auto& c : cyls) {
    if (c.type == CylinderDetection::MUSTIKA_PILLAR) mc++;
    if (c.type == CylinderDetection::CORE_PILLAR)    cc++;
    if (c.has_top_object) tc++;
  }
  add("mustika_pillar", mc);
  add("core_pillar",    cc);
  add("top_object",     tc);
  add("total_cylinders",cyls.size());

  arr.status.push_back(st);
  pub_diag_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 动态参数
// ═══════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult CylinderDetector::on_param_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult r; r.successful = true;
  for (auto& p : params) {
    try {
      #define SET(n,v) if (p.get_name()==n) v=p.as_double()
      #define SET_I(n,v) if (p.get_name()==n) v=p.as_int()
      #define SET_B(n,v) if (p.get_name()==n) v=p.as_bool()
      SET_I("normal_k_search",    normal_k_search_);
      SET_I("normal_omp_threads", normal_omp_threads_);
      SET("slice_thickness",      slice_thickness_);
      SET_I("slice_min_points",   slice_min_points_);
      SET("circle_fit_thresh",    circle_fit_thresh_);
      SET_I("circle_fit_iters",   circle_fit_iters_);
      SET("radius_min",           radius_min_);
      SET("radius_max",           radius_max_);
      SET("merge_center_tol",     merge_center_tol_);
      SET("merge_radius_tol",     merge_radius_tol_);
      SET_I("merge_min_slices",   merge_min_slices_);
      SET("min_height",           min_height_);
      SET("cyl3d_dist_thresh",    cyl3d_dist_thresh_);
      SET_I("cyl3d_max_iters",    cyl3d_max_iters_);
      SET("cyl3d_normal_weight",  cyl3d_normal_weight_);
      SET("cyl3d_axis_eps",       cyl3d_axis_eps_);
      SET("mustika_h_min",        mustika_h_min_);
      SET("mustika_h_max",        mustika_h_max_);
      SET("core_h_min",           core_h_min_);
      SET("core_h_max",           core_h_max_);
      SET("top_z_offset",         top_z_offset_);
      SET("top_z_range",          top_z_range_);
      SET_I("top_min_pts",        top_min_pts_);
      SET("top_cluster_tol",      top_cluster_tol_);
      SET_B("geo_check_enabled",  geo_check_enabled_);
      SET("geo_match_tol",        geo_match_tol_);
      #undef SET
      #undef SET_I
      #undef SET_B
    } catch (const rclcpp::ParameterTypeException&) {
      r.successful = false; r.reason = "type: " + p.get_name();
    }
  }
  return r;
}

}  // namespace lidar
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::lidar::CylinderDetector)
