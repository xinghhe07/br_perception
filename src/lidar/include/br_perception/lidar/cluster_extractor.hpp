#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <memory>
#include <vector>
#include <string>
#include <utility>      // std::pair
#include <cstdint>

namespace br_perception {
namespace lidar {

// ── 障碍物分类 ──
enum class ObstacleType : uint8_t {
  UNKNOWN      = 0,
  EARTH_CUBE   = 1,   // ≈350mm 立方体 — 地球方块
  SKY_CUBE     = 2,   // ≈200mm 立方体 — 天空方块
  ENEMY_ROBOT  = 3,   // ≈700×700×700mm — 敌方机器人
  MUSTIKA      = 4    // ≈200mm 球体 — 穆斯蒂卡
};

// ── 障碍物检测结果 ──
struct Obstacle {
  int id = 0;

  // 3D 包围盒
  float min_x = 0, min_y = 0, min_z = 0;
  float max_x = 0, max_y = 0, max_z = 0;

  // 质心
  float centroid_x = 0, centroid_y = 0, centroid_z = 0;

  // 尺寸 (长/宽/高)
  float length = 0, width = 0, height = 0;

  // 统计
  int   point_count = 0;

  // 方向 (PCA 主轴水平投影偏航角, rad)
  float yaw = 0;
  bool  is_elongated = false;    // 长宽比 > threshold 时为 true

  // 分类
  ObstacleType type = ObstacleType::UNKNOWN;
  float confidence = 0;          // [0-1]

  // PCA 特征值 (用于球体/立方体区分)
  float eigen_ratio_1 = 0;       // λ1 / λ2
  float eigen_ratio_2 = 0;       // λ2 / λ3
};

/**
 * @brief 障碍物聚类与分类节点 — ROBOCON 2027 适配版
 *
 * 管线:
 *   非地面点云(去圆柱) → [圆柱掩膜] → 欧式聚类 →
 *   特征计算(包围盒/PCA/质心) → 规则过滤 → 尺寸分类 → 发布
 *
 * 聚类参数从 lidar_params.yaml 读取 (ROS2 parameter 机制).
 *
 * 输入:
 *   /perception/lidar/non_ground     — 非地面点云 (sensor_msgs::PointCloud2)
 *
 * 输出:
 *   /perception/lidar/obstacles      — 3D 包围盒 + 分类标签 (visualization_msgs::MarkerArray)
 *   /perception/lidar/diagnostics     — 耗时 + 统计 (diagnostic_msgs::DiagnosticArray)
 *
 * ⚠️ **本模块的输出去向（2026-09-25 实测，别当成"在得分链上"）**：
 *    `python3 scripts/check_topic_consumers.py` 实测 `/perception/lidar/obstacles` 是
 *    **2 发 / 0 订**（单体 + 本组件两支发布者，零订阅者）⇒ **纯可视化**。
 *    而且**没有任何 launch 启动本组件** —— 实际跑的是单体 `lidar_perception_node.cpp`，
 *    它把本文件的过滤/分类逻辑**另写了一份**（§六 的重复实现台账，等你说的容器化重构合并）。
 *    ⚠️ ⇒ 本文件的单测**保护不了实际在跑的那份代码**，这一点在 §八 里如实记着。
 */
class ClusterExtractor : public rclcpp::Node
{
public:
  using PointCloud    = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;

  // ══════════════════════════════════════════════════════════════════════
  // 参数 —— 名字与 `config/lidar_params.yaml` 的键**逐字一致**
  // (§三 第 2 步的"三方一致"：yaml 键名 = 本结构字段名 = 头文件里的默认值)
  //
  // ⚠️ **为什么摘成一个结构体而不是散落在成员里**：这些是**纯判定输入**，
  //    摘出来之后判定逻辑才能是 `static` 纯函数、才能被单测直接喂进去
  //    （仓库既有模式：`SkyBlockDetector::judge_top_color`、
  //     `PerceptionPipeline::choose_sky_color` 都是 static + public + Params）。
  // ══════════════════════════════════════════════════════════════════════
  struct Params
  {
    // ── 欧式聚类 ──
    double cluster_tolerance {0.05};       ///< 欧式聚类距离阈值 (m)
    int    min_cluster_size  {5};          ///< 最小聚类点数
    int    max_cluster_size  {5000};       ///< 最大聚类点数

    // ── 规则过滤 (ROBOCON 2027 场景) ──
    double max_object_size   {2.0};        ///< 任一维度超此值 → 围栏/墙面 (m)
    double max_object_height {2.0};        ///< 高度超此值 → 柱子 (m)
    int    min_object_points {5};          ///< 点数少于此 → 噪点
    /// 底部 z 高于此值 → 悬空物 (m)
    /// ⚠️⚠️ **它是"绝对高度"还是"雷达系高度"，取决于喂进来的点云在哪个系**
    ///    —— 本模块**不做任何坐标变换**（`publish_obstacles` 用输入 header 原样发）。
    ///    上游 `ground_segmenter` 也是原样透传 header ⇒ 输入是**雷达系**，
    ///    而 0.8 这个值是按**场地系绝对高度**定的。模块 15/16 已于 2026-09-24 改成
    ///    发 `world_frame`，本模块**没跟着改** ⇒ **两份实现现在不一致**。
    ///    详见 §八 "仍未验过" 与 §六 的重复实现台账。
    double floating_z_min    {0.8};

    // ── 圆柱掩膜 ──
    bool   enable_cylinder_mask {true};    ///< 圆柱检测时屏蔽自身
    double cylinder_mask_radius {0.25};    ///< 圆柱外扩半径 (m)
    /// 圆柱中心列表 (从 YAML 的 `cylinder_mask_x` / `cylinder_mask_y` 两个数组配对重建),
    /// 存成 pair 向量**避免两个独立数组越界**
    std::vector<std::pair<double, double>> cylinder_centers;

    // ── 分类 ──
    double size_tolerance   {0.08};        ///< 分类尺寸容差 (m)
    double earth_cube_size  {0.35};        ///< 地球方块预期尺寸 (m)
    double sky_cube_size    {0.20};        ///< 天空方块预期尺寸 (m)
    double enemy_robot_size {0.70};        ///< 敌方机器人预期尺寸 (m)
    /// 穆斯蒂卡预期直径 (m)
    /// ⚠️ 场地真值是 **Ø210mm**（任务书 §14），这里的 0.20 是占位；
    ///    2026-09-25 之前它**根本没被任何计算用过**（死参数），现已接进 MUSTIKA 分支。
    double mustika_size     {0.20};
    double elongation_ratio {2.0};         ///< PCA λ1/λ2 超过此值 → elongated
    double sphere_eigen_tol {0.3};         ///< 球体判断: 特征值差异容差
  };

  explicit ClusterExtractor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // ══════════════════════════════════════════════════════════════════════
  // 纯函数 —— **static + public**，可被 `test/test_cluster_extractor.cpp` 直接调用
  //
  // ⚠️ **不起 ROS 节点、不碰 TF、不进回调** ⇒ 单测只链组件库即可。
  // ⚠️ 签名里**不放 Eigen 类型**（用本文件自有的 `Obstacle` POD）⇒ 测试不需要 Eigen。
  // ⚠️ 成员函数里同名的那几个只是**薄包装**（把 `params_` 传进去），节点行为逐位不变。
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 欧式聚类 → 每个簇的点索引
  /// @param cloud 非地面点云（**已在调用方去除圆柱**）
  /// ⚠️ `cloud->size() < min_cluster_size` 时直接返回空 —— 这是**点数不足**，
  ///    不是"没有簇"；两者在下游看起来一样，但耗时不同。
  /// ⚠️ PCL 抛异常时**吞掉并返回已提取的部分**（不向上抛，避免拖垮回调）。
  [[nodiscard]] static std::vector<pcl::PointIndices> extract_clusters(
      const PointCloudPtr& cloud, const Params& params);

  /// @brief 单个聚类 → 包围盒 / 质心 / 点数 / PCA 特征
  static Obstacle compute_obstacle(
      const PointCloudPtr& cloud, const pcl::PointIndices& cluster, int id,
      const Params& params);

  /// @brief 3D PCA：主轴方向 + `is_elongated` + 两个特征值比值
  ///
  /// ⚠️ **只写 `obs.yaw` / `obs.is_elongated` / `obs.eigen_ratio_1` / `obs.eigen_ratio_2`
  ///    四个字段**，不碰 `obs.centroid_*`。
  /// ✅ **质心由本函数自己算**（2026-09-25 起）：原先它读 `obs.centroid_*`，
  ///    等于要求"调用方必须先填质心"——**不填就静默绕原点求协方差**，算出一组
  ///    看着合理的错主轴（不报错）。`compute_obstacle` 恰好先填了所以一直没露头，
  ///    但本函数已是 `public static`、可被单独调用 ⇒ 那个隐含前提就是个陷阱，已摘掉。
  ///
  /// ⚠️ `n < 3` 或特征分解失败 ⇒ 回落成 `yaw=0 / is_elongated=false / 比值=1`
  ///    （**不是** `0` —— 比值 1 表示"各向同性"，与"没算出来"是两回事）。
  /// ⚠️ 主轴**有正负号二义性**（`v1` 与 `−v1` 都是解）⇒ `yaw` 只在模 π 意义下有意义。
  /// ⚠️ 近乎对称的点云（如完整立方体）**特征值本就接近简并** ⇒ 主轴方向数值上敏感，
  ///    不要对 `yaw` 做紧断言。见 `test/test_cluster_extractor.cpp` 的同名组。
  static void compute_pca_orientation(
      const PointCloudPtr& cloud, const pcl::PointIndices& cluster, Obstacle& obs,
      const Params& params);

  /// @brief 规则过滤（4 条：过大 / 过高 / 点数过少 / 悬空）
  /// @return true = **应剔除**
  /// ⚠️ 四条判据全是**严格大于/小于**：恰好等于阈值**不**剔除。
  [[nodiscard]] static bool filter_obstacle(const Obstacle& obs, const Params& params);

  /// @brief 按包围盒尺寸 + PCA 特征分类（改写 `obs.type` 与 `obs.confidence`）
  /// ⚠️ **顺序敏感**：先大后小（敌方 0.70 → 地球 0.35 → 天空/穆斯蒂卡 0.20），
  ///    第一个 `score > 0.5` 的分支胜出。改顺序会静默改变标签。
  /// ⚠️ 天空方块与穆斯蒂卡**尺寸相同**，只靠特征值比值分叉 ⇒ 见实现里的 ⚠️。
  static void classify_obstacle(Obstacle& obs, const Params& params);

  /// @brief 圆柱掩膜：删掉每个已知圆柱中心 `cylinder_mask_radius` 内的点
  /// ⚠️ 只看**水平距离** `dx²+dy²`（不判 z）⇒ 圆柱正上方的点也会被删。
  /// ⚠️ 边界是**严格小于**：恰好等于半径的点**保留**。
  /// ⚠️ `cylinder_centers` 为空 ⇒ **原样返回入参指针**（不复制）。
  [[nodiscard]] static PointCloudPtr mask_cylinders(
      const PointCloudPtr& cloud, const Params& params);

private:
  // ── ROS 回调 ──
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_param_change(
      const std::vector<rclcpp::Parameter>& params);

  // ── 成员薄包装（把 params_ 传给上面的 static 纯函数）──
  std::vector<pcl::PointIndices> extract_clusters(const PointCloudPtr& cloud);
  Obstacle compute_obstacle(const PointCloudPtr& cloud,
                            const pcl::PointIndices& cluster,
                            int id);
  void compute_pca_orientation(const PointCloudPtr& cloud,
                               const pcl::PointIndices& cluster,
                               Obstacle& obs);
  bool filter_obstacle(const Obstacle& obs) const;
  void classify_obstacle(Obstacle& obs);
  PointCloudPtr mask_cylinders(const PointCloudPtr& cloud);

  // ── 发布 ──
  void publish_obstacles(const std::vector<Obstacle>& obstacles,
                          const std_msgs::msg::Header& header);
  void publish_diagnostics(double total_ms,
                           size_t cluster_count,
                           size_t passed_count,
                           const std::vector<Obstacle>& obstacles);

  // ── 话题 ──
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  /// 全部可调参数的**唯一事实源**（发布/回调/纯函数都从它取值）
  Params params_;
};

}  // namespace lidar
}  // namespace br_perception
