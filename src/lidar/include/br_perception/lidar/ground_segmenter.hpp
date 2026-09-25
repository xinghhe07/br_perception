#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <memory>
#include <vector>
#include <utility>    // std::pair
#include <string>

namespace br_perception {
namespace lidar {

/**
 * @brief 地面分割节点 — ROBOCON 2027 多层场地适配版
 *
 * 管线:
 *   滤波后点云 → [可选: 降采样] → RANSAC平面拟合 → 法向量验证 →
 *   动态阈值精筛 → 分离地面/非地面 → 高度分层(L0/L1/L2) → 归一化发布
 *
 * 发布话题:
 *   /perception/lidar/ground         — 地面点云 (BR 当前站立平面)
 *   /perception/lidar/non_ground     — 非地面点云 (障碍物、柱子、方块等)
 *   /perception/lidar/l0_surface     — L0 地面层表面
 *   /perception/lidar/l1_surface     — L1 平台表面 (绝对 z≈0.6m)
 *   /perception/lidar/l2_surface     — L2 平台表面 (绝对 z≈0.9m)
 *   /perception/lidar/ground_plane   — 归一化地平面系数 [a,b,c,d]
 *   /perception/lidar/diagnostics    — 各步骤耗时 + 点数统计
 */
class GroundSegmenter : public rclcpp::Node
{
public:

  // ══════════════════════════════════════════════════════════════════════
  // 纯函数 —— **static + public**，可被 `test/test_ground_segmenter.cpp` 直接调
  // （仓库既有模式：`ClusterExtractor::filter_obstacle`、`SkyBlockDetector::judge_top_color`）
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 平面法向量是不是"**像地面**"（与 Z 轴的夹角小于阈值）
  /// @param coeff            PCL 平面系数 `ax+by+cz+d=0`（**不要求已归一化**）
  /// @param angle_threshold  允许的最大夹角（**弧度**）
  ///
  /// ⚠️ **取 `|c|` 再算夹角** ⇒ 法向量朝向不影响结论（`(0,0,1)` 与 `(0,0,-1)` 同判）。
  ///    RANSAC 给的系数**朝向是任意的**，不取绝对值的话**一半的地面会被否掉**。
  /// ⚠️ 零法向量（`norm < 1e-9`）⇒ **返回 false（不是 NaN）**。
  ///    ⚠️ 不防的话 `acos(0/0)` = NaN，而 `NaN < threshold` **恒为 false** ——
  ///    侥幸结果一样，但**上游一旦把 NaN 传出去就是灾难**（它污染一切比较）。
  /// ⚠️ 边界是**严格 `<`**：夹角恰好等于阈值 ⇒ **不算地面**。
  [[nodiscard]] static bool validate_plane_normal(const pcl::ModelCoefficients& coeff,
                                                  double angle_threshold);

  using PointCloud    = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;

  explicit GroundSegmenter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── 回调 ──
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  // ── 核心管线 ──
  bool fit_ground_plane(const PointCloudPtr& cloud,
                        pcl::ModelCoefficients& coeff,
                        pcl::PointIndices& inliers);

  /// 成员薄包装：把 `normal_angle_threshold_` 传进去（节点行为逐位不变）
  bool validate_plane_normal(const pcl::ModelCoefficients& coeff) const;

  pcl::PointIndices refine_inliers_adaptive(const PointCloudPtr& cloud,
                                            const pcl::ModelCoefficients& coeff) const;

  void extract_ground_and_non_ground(const PointCloudPtr& cloud,
                                     const pcl::PointIndices& inliers,
                                     PointCloudPtr& ground,
                                     PointCloudPtr& non_ground) const;

  /// 将地面点按绝对高度分为 L0 / L1 / L2 三层
  void classify_by_platform(const PointCloudPtr& ground,
                            PointCloudPtr& l0_surface,
                            PointCloudPtr& l1_surface,
                            PointCloudPtr& l2_surface) const;

  /// 归一化平面系数 (a²+b²+c²=1)
  void normalize_coefficients(pcl::ModelCoefficients& coeff) const;

  void publish_results(const PointCloudPtr& ground,
                       const PointCloudPtr& non_ground,
                       const PointCloudPtr& l0_surface,
                       const PointCloudPtr& l1_surface,
                       const PointCloudPtr& l2_surface,
                       const pcl::ModelCoefficients& coeff,
                       const std_msgs::msg::Header& header);

  void publish_diagnostics(double total_ms, double ransac_ms, double refine_ms,
                           size_t input_pts, size_t ground_pts, size_t non_ground_pts,
                           size_t l0_pts, size_t l1_pts, size_t l2_pts);

  // ── 话题 ──
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_l0_surface_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_l1_surface_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_l2_surface_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_plane_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ── 参数 ──
  // RANSAC
  double ransac_distance_threshold_{0.04};
  int    ransac_max_iterations_{100};
  double ransac_eps_angle_{0.45};          // 法向量约束角 (rad), ~26°
  bool   enable_normal_constraint_{true};  // 是否启用 RANSAC 法向量约束

  // 法向量验证
  double normal_angle_threshold_{0.15};    // 最大允许偏角 (rad), ~8.6°
                                           // 注意: 这是验证阈值(严), ransac_eps_angle 是搜索约束(宽)

  // 动态阈值: (高度(m), 阈值(m)) 对, 按高度升序
  // 可从 YAML 覆盖: height_threshold_map.z_keys / height_threshold_map.threshold_values
  std::vector<std::pair<double, double>> height_threshold_map_{
    {0.0, 0.04},
    {1.0, 0.06},
    {3.0, 0.10},
  };

  // 平台绝对高度 (场地坐标系, m)
  double l0_height_{0.0};
  double l1_height_{0.6};
  double l2_height_{0.9};
  double layer_tolerance_{0.15};
};

}  // namespace lidar
}  // namespace br_perception
