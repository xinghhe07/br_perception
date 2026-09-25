#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/header.hpp>

// ⚠️ **只读 TF**（把场地系常量变换到点云那个系）。见下面"坐标系纪律"那段。
//    ⚠️ **不 include/链接 `coordinate_transformer.hpp`** —— 那会让雷达层反向依赖
//       融合层（分层就破了），而且那个类构造时会**无条件广播 TF**，多一个广播者
//       就会撞 `TF_REPEATED_DATA`。
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>

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

// ── 建筑位状态 ──
enum class BuildingSpotStatus : uint8_t {
  EMPTY          = 0,   // 空位 — **量到了**，这个位上没有方块
  ONE_EARTH      = 1,   // 一块地球方块 (≈350mm)
  TWO_EARTH      = 2,   // 两块地球方块 (≈700mm)
  COMPLETE_TOWER = 3,   // 完整塔: 两块地球 + 一块天空 (≈900mm)
  /// ⚠️⚠️ **没量成**（2026-09-24 新增）—— TF 不可用 / 定不到位 ⇒ 这一位**无从判断**。
  ///
  /// **它与 `EMPTY` 必须分开**，这是本模块最毒的一处：
  ///   拿不到坐标时若报 `EMPTY`，主控会把"**我没量到**"读成"**这个位是空的**"
  ///   —— 塔明明建着，它以为没建。而反过来（真有塔报空）主控会去重放/重试，
  ///   也是白白丢时间。两者**动作完全不同**。
  ///
  /// ⚠️ 取值 **4** 与 `msg/BuildingSpot.msg` 的 `state` 逐值一致，且**走串口**
  ///    （`protocol_encoder.cpp:320` 原样写）⇒ 已记入 `config/comm_params.yaml`
  ///    头部「未冻结参数」清单**第 6 项**（单方提案，待与主控对齐）。
  UNKNOWN        = 4,
};

// ── 单个建筑位检测结果 ──
struct BuildingSpot {
  int   id = 0;
  float spot_x = 0;            // 建筑位中心 X (场地坐标系)
  float spot_y = 0;            // 建筑位中心 Y
  float platform_z = 0;        // 平台表面高度

  BuildingSpotStatus status = BuildingSpotStatus::EMPTY;

  float highest_z = 0;         // 柱状区域内最高点 Z
  float height = 0;            // 最高点到平台表面的高度差

  int   point_count = 0;       // 区域内非地面点数

  bool  has_top_surface = false;   // 是否检测到近水平顶面
  float top_surface_z = 0;         // 顶面平均高度 (如有)
  int   top_surface_points = 0;    // 顶面点数

  bool  needs_visual_check = false; // 完整塔需要视觉确认天空方块颜色
};

/**
 * @brief 建筑位分析节点 — ROBOCON 2027 场地建筑塔状态检测
 *
 * 管线:
 *   非地面点云 + field_geometry.yaml 建筑位坐标
 *   → 逐建筑位裁剪垂直柱状点云
 *   → 检测顶面 & 计算高度
 *   → 按高度阈值分类 (EMPTY / ONE_EARTH / TWO_EARTH / COMPLETE_TOWER)
 *
 * 输入:
 *   /perception/lidar/non_ground  — 非地面点云 (sensor_msgs::PointCloud2)
 *
 * 输出:
 *   /perception/lidar/building_spots  — 建筑位占用状态 (MarkerArray, 彩色方块)
 *   /perception/lidar/diagnostics      — 耗时 + 各状态计数 (DiagnosticArray)
 *
 * 注意:
 *   雷达无法判断天空方块的朝上颜色 → 完整塔标记 needs_visual_check = true
 */
class BuildingSpotAnalyzer : public rclcpp::Node
{
public:
  using PointCloud    = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;

  explicit BuildingSpotAnalyzer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_param_change(
      const std::vector<rclcpp::Parameter>& params);

  // ── 核心管线 ──
  /// 分析单个建筑位: 裁剪柱状区域 → 点数检查 → 顶面检测 → 高度分类
  BuildingSpot analyze_spot(const PointCloudPtr& cloud, int id,
                            float spot_x, float spot_y, float platform_z);

  /// 检测点云中是否存在近水平顶面 (最高点附近的点聚类)
  /// @return {has_surface, surface_z, surface_points}
  std::tuple<bool, float, int> detect_top_surface(
      const PointCloudPtr& spot_cloud, float z_max);

  // ── 发布 ──
  void publish_spots(const std::vector<BuildingSpot>& spots,
                     const std_msgs::msg::Header& header);
  void publish_diagnostics(double total_ms,
                           const std::vector<BuildingSpot>& spots);

  // ── 话题 ──
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_spots_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ══════════════════════════════════════════════════════════════
  // ⚠️⚠️ 坐标系纪律（2026-09-24 立）——**本模块最毒的一个坑**
  //
  //   `building_positions_` 是 **场地系**常量（`field_geometry.yaml`），
  //   而进来的点云在**雷达系**（`livox_frame`，随车走）。
  //   `pcl::CropBox` 拿场地系坐标去裁雷达系的云 ⇒ **机器人不在场原点时裁到错地方**
  //   ⇒ 裁不到点 ⇒ 报 EMPTY ⇒ **主控以为塔没建**。
  //
  //   ⚠️ 它在"台架原点静止 + 朝向一致"时**完全看不出来**（两个系恰好重合）
  //      —— 这就是它一直没被发现的原因。**必须动着测。**
  //
  //   ⚠️ **本模块的每个常量都是场地系的值**：位坐标、平台高、`empty_max_height`
  //      这类高度阈值（`analyze_spot` 里 `h = top_z − platform_z` 是**绝对高度差**）。
  //      所以不是"把常量拆到点云系"，而是**把点云整份搬到场地系** ——
  //      一次变换，全流程只在场地系里算，出错面最小。
  //
  //   ⇒ 纪律一:**测量搬去常量那边，别把常量拆到测量这边**。
  //      点云进回调第一件事就是 `cloud_to_world`。
  //   ⇒ 纪律二:**定位不到 ≠ 空**。TF 不可用时报 `UNKNOWN`，**不是 `EMPTY`**。
  // ══════════════════════════════════════════════════════════════
  /// @brief 只读 TF（⚠️ **不链接 `fusion::CoordinateTransformer`** —— 那会让雷达层
  ///        反向依赖融合层。TF 是基础设施，直接用 `tf2_ros`。）
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// @brief 场地系的名字（`building_positions_` 所在的那个系）
  std::string world_frame_{"world"};

  /// @brief TF 超时 [s] —— ⚠️ 别用 tf2 默认的 0（同 `mustika_pillar_check` 的教训）
  double tf_timeout_sec_{0.05};

  /// @brief 把**输入点云整份搬到场地系**（`world_frame_`）。
  ///
  /// @param in  输入点云（系由 `hdr.frame_id` 声明）
  /// @param out 变换结果；已在场地系 / `world_frame_` 为空时**原样返回 `in`**（不拷贝）
  /// @return false = 换不成（TF 不可用 / frame_id 为空）
  ///         ⇒ 调用方**必须报 UNKNOWN**（`publish_all_unknown`），**不许当成"空位"**
  ///
  /// ⚠️ 用 `lookupTransform` + `pcl::transformPointCloud`，**不用**
  ///    `tf2_geometry_msgs` 的 `Buffer::transform<PointStamped>`（那是逐点的路子，
  ///    而且那个包必须显式列进 CMake —— 少它会在编译期报 `doTransform` 找不到）。
  bool cloud_to_world(const PointCloudPtr& in, const std_msgs::msg::Header& hdr,
                      PointCloudPtr& out);

  /// @brief 一帧**全报 UNKNOWN** —— 换不到场地系时的唯一诚实做法
  /// ⚠️ 不许回落成"空位"，也不许干脆不发（主控分不清"没有塔"和"感知挂了"）
  void publish_all_unknown(const std_msgs::msg::Header& header);

  // ══════════════════════════════════════════════════════════════
  // 建筑位坐标参数 (从 field_geometry.yaml 通过 ROS params 传入)
  // ══════════════════════════════════════════════════════════════
  std::vector<std::pair<double, double>> building_positions_;
  /// 与 building_positions_ 一一对应的平台高度; 为空或长度不符时全部回落到 platform_z_
  /// (用于 L1=0.6 与 L2=0.9 的建筑位共存)
  std::vector<double> building_spot_zs_;

  /// 取第 i 个建筑位的平台高度: 有 building_spot_z[i] 用之, 否则回落到 platform_z
  float spot_platform_z(size_t i) const {
    return static_cast<float>(i < building_spot_zs_.size() ? building_spot_zs_[i]
                                                            : platform_z_);
  }

  // ══════════════════════════════════════════════════════════════
  // 几何参数
  // ══════════════════════════════════════════════════════════════
  double platform_z_{0.0};           // 平台表面绝对高度 (场地 Z 坐标, m)
  double spot_half_size_{0.25};      // 建筑位半宽 (500×500mm → 250mm 半宽)
  double column_z_min_{0.0};         // 柱状区域 Z 下界偏移 (相对于 platform_z)
  double column_z_max_{1.5};         // 柱状区域 Z 上界偏移

  // ══════════════════════════════════════════════════════════════
  // 检测阈值
  // ══════════════════════════════════════════════════════════════
  int    min_points_threshold_{5};   // 低于此值 → EMPTY
  double top_surface_tolerance_{0.03}; // 顶面点 Z 容差 (m)
  int    top_surface_min_points_{3};   // 顶面最小点数

  // ══════════════════════════════════════════════════════════════
  // 高度分类阈值 (相对于 platform_z, 单位 m)
  // ══════════════════════════════════════════════════════════════
  double empty_max_height_{0.05};      // 高度 < 此值 → EMPTY
  double one_earth_min_{0.20};         // ONE_EARTH 下界
  double one_earth_max_{0.45};         // ONE_EARTH 上界
  double two_earth_min_{0.55};         // TWO_EARTH 下界
  double two_earth_max_{0.80};         // TWO_EARTH 上界
  double complete_tower_min_{0.80};    // COMPLETE_TOWER 下界
  double complete_tower_max_{1.10};    // COMPLETE_TOWER 上界
};

}  // namespace lidar
}  // namespace br_perception
