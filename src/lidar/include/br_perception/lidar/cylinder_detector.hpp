#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>      // pcl::PointXYZ, pcl::PointNormal, pcl::Normal
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <tuple>

namespace br_perception {
namespace lidar {

// ── 圆柱检测结果 ──
struct CylinderDetection {
  int    id = 0;
  float  center_x = 0, center_y = 0;
  float  bottom_z = 0, top_z = 0;
  float  height = 0;
  float  radius = 0;
  float  confidence = 0;       // [0-1]

  // 法向量 (RANSAC 圆柱法可给出可靠轴方向)
  float  axis_nx = 0, axis_ny = 0, axis_nz = 1;

  bool   has_top_object = false;
  float  top_object_radius = 0;
  int    top_object_points = 0;

  enum Type { UNKNOWN = 0, MUSTIKA_PILLAR = 1, CORE_PILLAR = 2 };
  Type   type = UNKNOWN;

  // 几何一致性
  bool   matches_expected = false;
  float  position_error = 0;
};

/**
 * @brief 圆柱检测节点 — pcl::PointNormal 全管线版
 *
 * 双方法并行:
 *   A. 水平切片 + 2D 圆拟合 (对部分弧段鲁棒, Mid-360 主力)
 *   B. RANSAC 3D 圆柱拟合 + 法向量约束 (检测完整圆柱, 兜底)
 *
 * 双结果合并去重, 输出统一 CylinderDetection 列表.
 *
 * 输入: /perception/lidar/non_ground  (pcl::PointXYZ → 内部转 PointNormal)
 * 输出: /perception/lidar/pillars      (MarkerArray)
 *       /perception/lidar/pillar_info  (DiagnosticArray, 含 mustika_on_pillar 等关键布尔)
 */
class CylinderDetector : public rclcpp::Node
{
public:
  // ══════════════════════════════════════════════════════════════════════
  // 纯函数 —— **static + public**，可被 `test/test_cylinder_detector.cpp` 直接调
  // （仓库既有模式：`ClusterExtractor::filter_obstacle`、`GroundSegmenter::validate_plane_normal`）
  //
  // ⚠️⚠️ **按高度分两段区间，中间那条空档是刻意的**：
  // ```
  //   穆斯蒂卡柱  0.40 ~ 0.60 m
  //   ─── 空档 0.60 ~ 0.70（0.1m）───   ← 这里**不分类**（留 UNKNOWN）
  //   核心支柱    0.70 ~ 0.90 m
  // ```
  //    场上这两种柱的高度本来就分得开（500mm vs 800mm 级）—— 空档是留给
  //    **测量误差与台阶边缘**的：一个"半截柱子"落在空档里，**报未知比报错好**。
  //    ⇒ 若把空档填上（改成 `else`），**每个噪声团都会被贴上标签**。
  //
  // ⚠️ **区间是闭区间**（`>=` / `<=`）⇒ 恰好 0.40 / 0.60 / 0.70 / 0.90 都算在内。
  // ⚠️ 判定顺序是 `if (穆斯蒂卡) … else if (核心)` ⇒ **两段区间重叠时穆斯蒂卡优先**
  //    （当前默认不重叠，但参数可改 —— 顺序被单测钉住）。
  struct HeightClassParams
  {
    double mustika_h_min{0.40}, mustika_h_max{0.60};
    double core_h_min{0.70},    core_h_max{0.90};
    /// 分类成立时把置信度**抬到**这个下界（`std::max`，**不降级**已有的更高值）
    float  classified_confidence{0.75f};
  };

  /// @brief 按柱高贴语义标签（就地改写 `type` 与 `confidence`）
  /// ⚠️ **落在两段区间之外的一律不动** —— `type` 保持调用前的值（通常是 `UNKNOWN`）。
  ///    这比"猜一个最近的"安全：**报未知比报错好**。
  [[nodiscard]] static void classify_by_height(std::vector<CylinderDetection>& cyls,
                                               const HeightClassParams& params);

  using PointT      = pcl::PointNormal;       // 统一点类型: xyz + normal
  using PointCloud  = pcl::PointCloud<PointT>;
  using PointCloudPtr = PointCloud::Ptr;
  using CloudXYZ    = pcl::PointCloud<pcl::PointXYZ>;
  using CloudXYZPtr = CloudXYZ::Ptr;

  explicit CylinderDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_param_change(
      const std::vector<rclcpp::Parameter>& params);

  // ── 法向量 ──
  /// 为点云计算法向量 (pcl::NormalEstimation OMP 并行)
  void estimate_normals(PointCloudPtr& cloud);

  // ── 方法 A: 水平切片 + 2D 圆拟合 ──
  struct SliceInfo {
    float z_center;
    std::vector<size_t> point_indices;  // 指向原始点云
  };
  std::vector<SliceInfo> slice_horizontally(const PointCloudPtr& cloud);
  bool fit_circle_ransac_2d(const PointCloudPtr& cloud,
                            const std::vector<size_t>& indices,
                            float& cx, float& cy, float& r,
                            std::vector<size_t>& inlier_indices) const;
  std::vector<CylinderDetection> merge_slices(
      const PointCloudPtr& cloud,
      const std::vector<SliceInfo>& slices,
      const std::vector<std::tuple<float,float,float,bool>>& circle_results);

  // ── 方法 B: RANSAC 3D 圆柱拟合 ──
  std::vector<CylinderDetection> detect_cylinders_ransac_3d(
      const PointCloudPtr& cloud);

  // ── 去重 ──
  std::vector<CylinderDetection> deduplicate(
      std::vector<CylinderDetection>&& a,
      std::vector<CylinderDetection>&& b);

  // ── 分类 & 验证 ──
  void classify_by_height(std::vector<CylinderDetection>& cylinders);
  void check_top_objects(std::vector<CylinderDetection>& cylinders,
                         const PointCloudPtr& cloud);
  void geometry_consistency_check(std::vector<CylinderDetection>& cylinders);

  // ── 发布 ──
  void publish_markers(const std::vector<CylinderDetection>& cylinders,
                       const std_msgs::msg::Header& header);
  void publish_diagnostics(double total_ms, double normals_ms,
                           const std::vector<CylinderDetection>& cylinders);

  // ── 话题 ──
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ── 参数 ──
  // 法向量
  int    normal_k_search_{20};
  int    normal_omp_threads_{4};

  // 切片法
  double slice_thickness_{0.03};
  int    slice_min_points_{8};
  double circle_fit_thresh_{0.02};
  int    circle_fit_iters_{60};
  double radius_min_{0.08};
  double radius_max_{0.20};
  double merge_center_tol_{0.05};
  double merge_radius_tol_{0.03};
  int    merge_min_slices_{3};
  double min_height_{0.10};

  // RANSAC 3D
  double cyl3d_dist_thresh_{0.025};
  int    cyl3d_max_iters_{200};
  double cyl3d_normal_weight_{0.1};
  double cyl3d_axis_eps_{0.35};       // ~20° 竖直容差

  // 分类
  double mustika_h_min_{0.40}, mustika_h_max_{0.60};
  double core_h_min_{0.70},    core_h_max_{0.90};

  // 顶部检测
  double top_z_offset_{-0.05};
  double top_z_range_{0.20};
  int    top_min_pts_{5};
  double top_cluster_tol_{0.05};

  // 几何校验
  bool   geo_check_enabled_{true};
  double geo_match_tol_{0.30};

  struct ExpectedPillar { std::string name; float x, y; CylinderDetection::Type type; };
  std::vector<ExpectedPillar> expected_{
    {"mustika_pillar", 0.f, 0.f, CylinderDetection::MUSTIKA_PILLAR},
    {"core_pillar",    0.f, 0.f, CylinderDetection::CORE_PILLAR},
  };
};

}  // namespace lidar
}  // namespace br_perception
