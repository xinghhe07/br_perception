#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// ⚠️ **只读 TF**：把输入点云从雷达系搬到场地系（见下面「坐标系纪律」那段）。
//    ⚠️ 不 include/链接 `coordinate_transformer.hpp` —— 那会让雷达层反向依赖融合层。
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// ── 对外数据接口 (设计文档 §5.1) ──
// ⚠️ 2026-09-22 接上: `LidarResult.msg` 建于 2026-09-18, 但**一直没有被任何代码引用** ——
//    本节点当时发的仍是 `diagnostic_msgs/DiagnosticArray`（"给人看的", 见该 msg 头注释）。
//    机器可读的雷达数据接口因此实际上不存在, 融合层无从对接。
#include "br_perception/msg/lidar_result.hpp"
#include "br_perception/msg/obstacle.hpp"
#include "br_perception/msg/building_spot.hpp"
#include "br_perception/msg/pillar_status.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>

#include <memory>
#include <vector>
#include <string>
#include <utility>      // std::pair
#include <cstdint>
#include <chrono>

namespace br_perception {
namespace lidar {

// ═══════════════════════════════════════════════════════════════════════════
// 前向声明 — 复用子模块数据结构
// ═══════════════════════════════════════════════════════════════════════════

// 建筑位状态 (与 building_spot_analyzer.hpp 一致)
enum class BuildingSpotStatus : uint8_t {
  EMPTY          = 0,
  ONE_EARTH      = 1,
  TWO_EARTH      = 2,
  COMPLETE_TOWER = 3,
  /// ⚠️⚠️ **没量成**（2026-09-24 新增，与组件那份 `building_spot_analyzer.hpp` 同一口径）——
  /// TF 不可用 / 定不到位 ⇒ 这一位**无从判断**。
  ///
  /// ⚠️ **别用 `EMPTY` 冒充它**：主控会把"我没量到"读成"这个位是空的"，
  ///    于是**塔明明建着、它以为没建**。两者动作完全不同。
  /// ⚠️ 这份与组件那份是**同一段代码的两个拷贝**（台账 §六 记为技术债）——
  ///    **改一处必须改两处**，否则跑单体时又变回去。
  UNKNOWN        = 4,
};

struct BuildingSpot {
  int   id = 0;
  float spot_x = 0, spot_y = 0;
  float platform_z = 0;
  BuildingSpotStatus status = BuildingSpotStatus::EMPTY;
  float highest_z = 0;
  float height = 0;
  int   point_count = 0;
  bool  has_top_surface = false;
  float top_surface_z = 0;
  int   top_surface_points = 0;
  bool  needs_visual_check = false;
};

// 圆柱检测结果 (与 cylinder_detector.hpp 一致)
struct CylinderDetection {
  int   id = 0;
  float center_x = 0, center_y = 0;
  float bottom_z = 0, top_z = 0;
  float height = 0;
  float radius = 0;
  float confidence = 0;
  float axis_nx = 0, axis_ny = 0, axis_nz = 1;
  bool  has_top_object = false;
  float top_object_radius = 0;
  int   top_object_points = 0;
  enum Type { UNKNOWN = 0, MUSTIKA_PILLAR = 1, CORE_PILLAR = 2 };
  Type  type = UNKNOWN;
  bool  matches_expected = false;
  float position_error = 0;
};

// 障碍物分类 (与 cluster_extractor.hpp 一致)
enum class ObstacleType : uint8_t {
  UNKNOWN      = 0,
  EARTH_CUBE   = 1,
  SKY_CUBE     = 2,
  ENEMY_ROBOT  = 3,
  MUSTIKA      = 4,
};

struct Obstacle {
  int   id = 0;
  float min_x = 0, min_y = 0, min_z = 0;
  float max_x = 0, max_y = 0, max_z = 0;
  float centroid_x = 0, centroid_y = 0, centroid_z = 0;
  float length = 0, width = 0, height = 0;
  int   point_count = 0;
  float yaw = 0;
  bool  is_elongated = false;
  ObstacleType type = ObstacleType::UNKNOWN;
  float confidence = 0;
  float eigen_ratio_1 = 0;
  float eigen_ratio_2 = 0;
};

// ═══════════════════════════════════════════════════════════════════════════
// 管线各阶段耗时
// ═══════════════════════════════════════════════════════════════════════════
struct PipelineTiming {
  double preprocess_ms       = 0;
  double ground_segment_ms   = 0;
  double cylinder_detect_ms  = 0;
  double cluster_extract_ms  = 0;
  double building_spot_ms    = 0;
  double total_ms            = 0;
  int    input_points        = 0;
  int    ground_points       = 0;
  int    non_ground_points   = 0;
};

// ═══════════════════════════════════════════════════════════════════════════
// 统一感知结果
// ═══════════════════════════════════════════════════════════════════════════
struct LidarPerceptionResult {
  // 时间戳
  rclcpp::Time stamp;

  // 地面平面系数 [a, b, c, d] (归一化)
  std::vector<float> ground_plane;

  // 检测结果
  std::vector<CylinderDetection> cylinders;
  std::vector<Obstacle>          obstacles;
  std::vector<BuildingSpot>      building_spots;

  // 分层地面点云 (L0/L1/L2)
  int l0_points = 0;
  int l1_points = 0;
  int l2_points = 0;

  // 各阶段耗时
  PipelineTiming timing;

  // 健康状态
  bool radar_connected = true;
  bool ground_found    = false;
};

/**
 * @brief 雷达感知主节点 — ROBOCON 2027 全管线编排
 *
 * 管线顺序:
 *   /livox/lidar (原始点云)
 *     → [1] 预处理 (降采样/离群点/ROI/自过滤)
 *     → [2] 地面分割 (RANSAC + 法向量验证 + 动态阈值)
 *     → [3] 圆柱检测 (水平切片2D圆拟合 + RANSAC 3D圆柱)
 *     → [4] 障碍物聚类 (欧式聚类 + 尺寸分类)
 *     → [5] 建筑位分析 (柱状裁剪 + 顶面检测 + 高度分类)
 *
 * 输出话题 (中间结果, 供调试和独立运行):
 *   /perception/lidar/filtered
 *   /perception/lidar/ground
 *   /perception/lidar/non_ground
 *   /perception/lidar/l0_surface / l1_surface / l2_surface
 *   /perception/lidar/ground_plane
 *   /perception/lidar/pillars          (MarkerArray)
 *   /perception/lidar/obstacles        (MarkerArray)
 *   /perception/lidar/building_spots   (MarkerArray)
 *
 * 统一输出:
 *   /perception/lidar/result           (**LidarResult**, 对外数据接口, 设计文档 §5.1)
 *                                      ⚠️ 2026-09-22 由 DiagnosticArray 改过来 ——
 *                                         此前发的是给人看的计数文本, 融合层没法用。
 *                                         ⚠️ header.frame_id **继承输入点云**, 不是 "world"
 *   /perception/lidar/debug_cloud      (PointCloud2, debug_flags 开启时)
 *   /perception/lidar/diagnostics      (DiagnosticArray, 各阶段耗时 + 平面系数 + 分类计数)
 *
 * 异常处理:
 *   - 雷达断连: 超时检测 → RCLCPP_WARN + radar_connected=false, 不崩溃
 *   - 地面拟合失败: 继续处理 (使用上一次有效平面或默认水平面)
 *   - 点云格式异常: pcl::fromROSMsg try-catch, 跳过当前帧
 */
class LidarPerceptionNode : public rclcpp::Node
{
public:
  using PointCloud    = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointNormalCloud    = pcl::PointCloud<pcl::PointNormal>;
  using PointNormalCloudPtr = PointNormalCloud::Ptr;

  explicit LidarPerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ═══════════════════════════════════════════════════════════════════════
  // ROS 回调
  // ═══════════════════════════════════════════════════════════════════════
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void radar_watchdog_timer_callback();
  rcl_interfaces::msg::SetParametersResult on_param_change(
      const std::vector<rclcpp::Parameter>& params);

  // ═══════════════════════════════════════════════════════════════════════
  // 管线阶段 (每个返回结果 + 耗时)
  // ═══════════════════════════════════════════════════════════════════════

  /// [0] **把输入点云搬到场地系**（2026-09-24 新增，见下面「坐标系纪律」那段）
  ///
  /// @param in   输入点云（系由 `hdr.frame_id` 声明）
  /// @param hdr  该帧的 header（⚠️ **时间戳用它的**，TF 要查那一时刻的变换）
  /// @param out  变换结果；已在场系或 `world_frame` 为空时**原样返回 `in`**（不拷贝）
  /// @return false = **变换不成**（TF 不可用 / frame_id 为空）
  ///         ⇒ 调用方**必须报 UNKNOWN，不许当成"空位"**
  bool cloud_to_world(const PointCloudPtr& in, const std_msgs::msg::Header& hdr,
                      PointCloudPtr& out);

  /// [1] 预处理: 降采样 → 离群点 → ROI裁剪 → 自过滤
  /// ⚠️ **要求入参已在场地系**（ROI 的 z 上下限是绝对高度，见「坐标系纪律」）
  /// @return {过滤后点云, 耗时_ms}
  std::pair<PointCloudPtr, double> preprocess_stage(const PointCloudPtr& input);

  /// [2] 地面分割: RANSAC 平面拟合 → 法向量验证 → 动态阈值精筛
  /// @return {ground_cloud, non_ground_cloud, l0, l1, l2, coeff, 耗时_ms}
  struct GroundResult {
    PointCloudPtr ground;
    PointCloudPtr non_ground;
    PointCloudPtr l0_surface;
    PointCloudPtr l1_surface;
    PointCloudPtr l2_surface;
    pcl::ModelCoefficients coeff;
    double elapsed_ms = 0;
    bool   success = false;
  };
  GroundResult ground_segment_stage(const PointCloudPtr& input);

  /// [3] 圆柱检测: 双方法 (2D切片圆拟合 + RANSAC 3D圆柱)
  /// @return {cylinders, 耗时_ms}
  std::pair<std::vector<CylinderDetection>, double>
  cylinder_detect_stage(const PointNormalCloudPtr& cloud);

  /// [4] 障碍物聚类: 圆柱掩膜 → 欧式聚类 → 特征计算 → 分类
  /// @return {obstacles, 耗时_ms}
  std::pair<std::vector<Obstacle>, double>
  cluster_extract_stage(const PointCloudPtr& non_ground,
                        const std::vector<CylinderDetection>& cylinders);

  /// [5] 建筑位分析: 逐建筑位裁剪柱状区域 → 高度分类
  /// @return {spots, 耗时_ms}
  std::pair<std::vector<BuildingSpot>, double>
  building_spot_stage(const PointCloudPtr& non_ground);

  // ═══════════════════════════════════════════════════════════════════════
  // 子阶段辅助方法
  // ═══════════════════════════════════════════════════════════════════════

  /// XYZ → PointNormal (法向量估计, OMP 并行)
  PointNormalCloudPtr estimate_normals(const PointCloudPtr& cloud);

  /// 法向量验证: 检查平面法向量是否接近竖直
  bool validate_plane_normal(const pcl::ModelCoefficients& coeff) const;

  /// 动态阈值精筛地面点
  pcl::PointIndices refine_inliers_adaptive(const PointCloudPtr& cloud,
                                            const pcl::ModelCoefficients& coeff) const;

  /// 按平台高度分层
  void classify_by_platform(const PointCloudPtr& ground,
                            PointCloudPtr& l0, PointCloudPtr& l1, PointCloudPtr& l2) const;

  /// 归一化平面系数
  void normalize_coefficients(pcl::ModelCoefficients& coeff) const;

  /// 构建 Obstacle (从聚类索引)
  Obstacle build_obstacle(const PointCloudPtr& cloud,
                          const pcl::PointIndices& cluster, int id);

  /// 尺寸分类
  void classify_obstacle(Obstacle& obs);

  /// 构建 BuildingSpot (裁剪+顶面检测+高度分类)
  /// @param spot_z 该位所在平台的表面绝对高度 (L1=0.6 / L2=0.9)
  BuildingSpot analyze_single_spot(const PointCloudPtr& cloud,
                                   int id, float spot_x, float spot_y, float spot_z);

  /// 取第 i 个建筑位的平台高度: 有 building_spot_z[i] 用之, 否则回落到 platform_z
  float spot_platform_z(size_t i) const {
    return static_cast<float>(i < building_spot_zs_.size() ? building_spot_zs_[i]
                                                            : platform_z_);
  }

  /// 顶面检测
  std::tuple<bool, float, int> detect_top_surface(
      const PointCloudPtr& spot_cloud, float z_max);

  // ═══════════════════════════════════════════════════════════════════════
  // 发布
  // ═══════════════════════════════════════════════════════════════════════
  /// @brief **中间结果 —— 拆成两半，因为它们的"该不该发"由两件不同的事决定**
  ///
  /// ⚠️⚠️ 2026-09-24 拆的，理由是一张错表咬过一次：
  ///    · **数据接口**（有代码订阅者）⇒ 挂 `publish_intermediate_topics_`
  ///      `filtered`（1 个订阅者：`ground_segmenter`）、
  ///      `non_ground`（**3 个**：`building_spot_analyzer` / `cluster_extractor` /
  ///       `cylinder_detector`）—— 它们是**容器化拓扑里组件的输入**。
  ///      ⚠️ 把它们关掉 = **组件那条链饿死**（症状与"代码坏了"一样）。
  ///    · **纯可视化**（**零**订阅者，`python3 scripts/check_topic_consumers.py` 可复核）
  ///      ⇒ 挂 `debug_enabled_`：ground / l0..l2_surface / ground_plane /
  ///      pillars / obstacles / building_spots —— 给 RViz 看的。
  ///    ⇒ 比赛模式（`debug_enabled_ = false`）下只剩 **2 次**整份点云拷贝，
  ///      而不是原来那 6 次 —— 组件照样喂饱，CPU 也省下来。
  ///   ⚠️ 复核命令写在上面了：**下次再挂开关之前先跑它**（台账 §八 的纪律）。
  void publish_intermediate_data(const PointCloudPtr& filtered,
                                 const GroundResult& gr,
                                 const std_msgs::msg::Header& header);

  void publish_intermediate_viz(const GroundResult& gr,
                                const std::vector<CylinderDetection>& cylinders,
                                const std::vector<Obstacle>& obstacles,
                                const std::vector<BuildingSpot>& spots,
                                const std_msgs::msg::Header& header);

  /// @brief 发布对外数据接口 `LidarResult`（话题 `/perception/lidar/result`）
  /// @param result   本帧感知结果
  /// @param frame_id **`result` 里那些坐标真正所在的系**。
  ///                 ⚠️ 2026-09-24 起是**场地系**（`world_frame_`）—— 本节点现在真的
  ///                 把点云换到那个系里算了，所以这里写的必须是它，不是输入点云的系。
  ///                 ⚠️ 老注释说"不要写死 world（那是融合层的产出）"—— 那句话的**前提
  ///                 （本节点不做变换、输出系 = 输入系）已经不成立了**，别再照它办。
  void publish_unified_result(const LidarPerceptionResult& result,
                              const std::string& frame_id);

  /// @brief 一帧**全报 UNKNOWN** —— 换不到场地系时的唯一诚实做法
  ///
  /// ⚠️⚠️ **不许回落成"空位"**：拿不到坐标时若报 `EMPTY`，主控会把"**我没量到**"
  ///    读成"**这个位是空的**" —— 塔明明建着，它以为没建（两件事的动作完全不同）。
  ///    ⚠️ 也**不许干脆不发**：主控分不清"没有塔"和"感知挂了"。
  void publish_all_unknown(const std_msgs::msg::Header& header, const char* reason);

  void publish_debug_cloud(const PointCloudPtr& ground,
                           const PointCloudPtr& non_ground,
                           const std::vector<CylinderDetection>& cylinders,
                           const std::vector<Obstacle>& obstacles,
                           const std::vector<BuildingSpot>& spots,
                           const std_msgs::msg::Header& header);

  void publish_diagnostics(const PipelineTiming& timing,
                           const LidarPerceptionResult& result);

  // ═══════════════════════════════════════════════════════════════════════
  // 话题
  // ═══════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_raw_;

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️⚠️ 坐标系纪律（2026-09-24 立）—— **本模块最毒的一个坑，且一直没被发现**
  //
  //   本节点的**每一个常量都是在场地系里量的**：
  //     · `l0/l1/l2_height` = 0.0 / 0.6 / 0.9  —— 平台面的**绝对**高度
  //       （`ground_segmenter.cpp:327` 就是 `|pt.z − l1_height_| < tol` 的绝对比较）
  //     · `roi_min_z/-0.5_roi_max_z/2.0`      —— 地面以下到 2m 的**绝对**高度
  //     · `building_spot_x/y/z`               —— `field_geometry.yaml` 的场地常量
  //   而输入点云是**雷达系**（`livox_frame`，随车走）——
  //   `livox_custom_to_pcl2` 桥接发出来时写死的就是它。
  //
  //   ⇒ 不换系就三个后果，**一个都不会报错**：
  //     ① 地面分层全空: 雷达离地 ~0.24m ⇒ `pt.z − 0.0` 恒 ≈ −0.24，L0/L1/L2 **一个都匹配不上**
  //     ② 建筑位 CropBox 裁到空气: 场地常量 (2.80, 2.80) 被当成"车前方 2.8m" ⇒ 裁不到点 ⇒ EMPTY
  //     ③ 所有对外坐标都**不在 `header.frame_id` 声明的那个系里**（第一宗旨「准」那条的原话）
  //
  //   ⚠️ 它在"车停在原点、朝向一致"时**完全看不出来**（两个系恰好重合）——
  //      这正是台架联调时它一直没露头的原因。**必须动着测。**
  //
  //   ⇒ 纪律: **点云进回调第一件事就是搬到场地系**，之后全流程只认场地系常量。
  //   ⇒ 定位不到时**报 UNKNOWN，不许回落成 EMPTY**（`BuildingSpotStatus::UNKNOWN`）。
  // ═══════════════════════════════════════════════════════════════════════════
  /// @brief 场地系名字（`building_spot_*` 与各高度常量所在的那个系）
  ///        ⚠️ 融合层的 `CoordinateTransformer` 广播的就是这个名字（`fusion_params.yaml`）
  std::string world_frame_{"world"};

  /// @brief TF 查询超时 [s] —— ⚠️ **别用 tf2 的默认值 0**（同 `mustika_pillar_check` 的教训）：
  ///        点云时间戳由雷达打、TF 由里程计/融合层打，两边跨进程；给 0 时缓冲区
  ///        还没包住该时刻就直接抛，症状会写成"取不到建筑位"。
  double tf_timeout_sec_{0.05};

  /// @brief 只读 TF（⚠️ **不链接 `fusion::CoordinateTransformer`** —— 那会让雷达层
  ///        反向依赖融合层；TF 是基础设施，直接用 `tf2_ros`。本节点**只读不广播**，
  ///        多一个广播者就会撞 `TF_REPEATED_DATA`。）
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 中间结果发布
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_non_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_l0_surface_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_l1_surface_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_l2_surface_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_plane_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pillars_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_spots_;

  // 统一输出
  /// @brief 对外**数据**接口 (LidarResult)。
  ///        ⚠️ 2026-09-22 从 `DiagnosticArray` 改过来 —— 那条话题此前发的是给人看的计数文本,
  ///           下游（融合/编排）没法拿它当数据用。诊断内容已移到 `pub_diagnostics_`。
  rclcpp::Publisher<br_perception::msg::LidarResult>::SharedPtr        pub_result_;
  /// @brief 诊断接口 (DiagnosticArray) —— 2026-09-22 起**同时**承载原来塞在 result 里的
  ///        平面系数/各层点数/分类计数（那些是给人看的, 本来就该在这）
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_cloud_;

  // 看门狗 (雷达断连检测)
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // 动态参数回调句柄
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ═══════════════════════════════════════════════════════════════════════
  // 参数 — 预处理
  // ═══════════════════════════════════════════════════════════════════════
  double leaf_size_{0.05};
  bool   enable_outlier_filter_{true};
  int    outlier_mean_k_{10};
  double outlier_std_thresh_{1.0};
  double roi_min_x_{-10.0}, roi_max_x_{30.0};
  double roi_min_y_{-10.0}, roi_max_y_{10.0};
  double roi_min_z_{-0.5},  roi_max_z_{2.0};

  // ═══════════════════════════════════════════════════════════════════════
  // 参数 — 地面分割
  // ═══════════════════════════════════════════════════════════════════════
  double ransac_distance_threshold_{0.04};
  int    ransac_max_iterations_{100};
  double ransac_eps_angle_{0.45};
  bool   enable_normal_constraint_{true};
  double normal_angle_threshold_{0.15};
  std::vector<std::pair<double, double>> height_threshold_map_{
    {0.0, 0.04}, {1.0, 0.06}, {3.0, 0.10}};
  double l0_height_{0.0};
  double l1_height_{0.6};
  double l2_height_{0.9};
  double layer_tolerance_{0.15};

  // ═══════════════════════════════════════════════════════════════════════
  // 参数 — 圆柱检测
  // ═══════════════════════════════════════════════════════════════════════
  int    normal_k_search_{20};
  double slice_thickness_{0.03};
  int    slice_min_points_{8};
  double circle_fit_thresh_{0.02};
  double radius_min_{0.08};
  double radius_max_{0.20};
  double merge_center_tol_{0.05};
  double merge_radius_tol_{0.03};
  int    merge_min_slices_{3};
  double min_cyl_height_{0.10};
  double cyl3d_dist_thresh_{0.025};
  int    cyl3d_max_iters_{200};
  double cyl3d_normal_weight_{0.1};
  double cyl3d_axis_eps_{0.35};
  double mustika_h_min_{0.40}, mustika_h_max_{0.60};
  double core_h_min_{0.70}, core_h_max_{0.90};

  // ═══════════════════════════════════════════════════════════════════════
  // 参数 — 障碍物聚类
  // ═══════════════════════════════════════════════════════════════════════
  double cluster_tolerance_{0.05};
  int    min_cluster_size_{5};
  int    max_cluster_size_{5000};
  double max_object_size_{2.0};
  double max_object_height_{2.0};
  int    min_object_points_{5};
  double floating_z_min_{0.8};
  double size_tolerance_{0.08};
  double earth_cube_size_{0.35};
  double sky_cube_size_{0.20};
  double enemy_robot_size_{0.70};
  double mustika_size_{0.20};
  double elongation_ratio_{2.0};
  double sphere_eigen_tol_{0.3};

  // ═══════════════════════════════════════════════════════════════════════
  // 参数 — 建筑位分析
  // ═══════════════════════════════════════════════════════════════════════
  std::vector<std::pair<double, double>> building_positions_;
  /// 与 building_positions_ 一一对应的平台高度; 为空或长度不符时全部回落到 platform_z_
  /// (用于 L1=0.6 与 L2=0.9 的建筑位共存: 单一 platform_z 会把 L2 空位误判成"有 1 块")
  std::vector<double> building_spot_zs_;
  double platform_z_{0.0};
  double spot_half_size_{0.25};
  double column_z_min_{0.0};
  double column_z_max_{1.5};
  int    min_spot_points_{5};
  double top_surface_tolerance_{0.03};
  int    top_surface_min_points_{3};
  double empty_max_height_{0.05};
  double one_earth_min_{0.20},  one_earth_max_{0.45};
  double two_earth_min_{0.55},  two_earth_max_{0.80};
  double complete_tower_min_{0.80}, complete_tower_max_{1.10};

  // ═══════════════════════════════════════════════════════════════════════
  // 参数 — 预期柱子几何 (从 field_geometry.yaml 读取)
  // ═══════════════════════════════════════════════════════════════════════
  std::vector<double> mustika_pillar_pos_;
  double mustika_pillar_height_expected_{0.5};
  double mustika_pillar_radius_expected_{0.135};
  std::vector<double> core_pillar_pos_;
  double core_pillar_height_expected_{0.8};
  double core_pillar_radius_expected_{0.135};

  // ═══════════════════════════════════════════════════════════════════════
  // 参数 — 调试 & 看门狗
  // ═══════════════════════════════════════════════════════════════════════
  /// @brief **数据接口**话题发不发 —— 只指 `filtered` 与 `non_ground` 两条
  ///        （它们的订阅者见 `publish_intermediate_data` 的注释）。
  ///
  /// ⚠️ 默认 **true** —— 它们**不是"给人看的装饰"**，而是**容器化拓扑里组件的输入**：
  ///    关掉 = 3 个组件没有输入 ⇒ **整条容器化链饿死**（症状与"代码坏了"一样）。
  /// ⚠️ 与 `debug_enabled_` **不是一回事**：那个管"给人看的东西"（8 条零订阅者的话题
  ///    + `debug_cloud`）。**混用早晚咬一次** —— 2026-09-23 就咬了（台账 §八）。
  /// ⇒ 只在确认"**单体独占、组件不在场**"时才置 false（省下 2 次整份点云拷贝）。
  bool   publish_intermediate_topics_{true};

  bool   debug_enabled_{false};
  double watchdog_timeout_{1.0};     // 雷达超时阈值 (秒)
  int    max_skip_frames_{3};        // 连续跳过帧数上限

  // ═══════════════════════════════════════════════════════════════════════
  // 状态
  // ═══════════════════════════════════════════════════════════════════════
  rclcpp::Time last_cloud_time_;
  bool   radar_connected_{true};
  int    consecutive_empty_frames_{0};

  // 上一次有效的地面平面 (用于地面拟合失败时的回退)
  pcl::ModelCoefficients last_valid_coeff_;
  bool   has_last_coeff_{false};
};

}  // namespace lidar
}  // namespace br_perception
