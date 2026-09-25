#include "br_perception/lidar/building_spot_analyzer.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <cmath>

// ⚠️ TF：把输入点云整份搬到**场地系**（坐标系纪律见头文件）。
//    ⚠️ 只用 `lookupTransform` + 自己组 `Eigen::Affine3f` ⇒ **不需要 `tf2_geometry_msgs`**
//       （那个包是为 `Buffer::transform<PointStamped>` 的重载准备的，这里不走那条路）。
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <pcl/common/transforms.h>


namespace br_perception {                                            //有助于组织代码，避免与其他库与或模块发生冲突
namespace lidar {

// ═══════════════════════════════════════════════════════════════════════════
// 辅助: BuildingSpotStatus → 字符串
// ═══════════════════════════════════════════════════════════════════════════#匿名命名空间 子在，  BuildingSpotStatus（枚举类型）转换为可读字符串
namespace {
const char* spot_status_name(BuildingSpotStatus s) {
  switch (s) {                                                               // 根据传入状态s  返回对应字符串描述
    case BuildingSpotStatus::EMPTY:          return "Empty";
    case BuildingSpotStatus::ONE_EARTH:      return "One Earth";
    case BuildingSpotStatus::TWO_EARTH:      return "Two Earth";
    case BuildingSpotStatus::COMPLETE_TOWER: return "Complete Tower";
    // ⚠️ **不能落进 `default` 印成 "Empty"** —— 那正是"用 0 冒充未知"的同一个错：
    //    现场看诊断的人会把"没量成"读成"这个位是空的"。
    case BuildingSpotStatus::UNKNOWN:        return "**UNKNOWN(没量成)**";
    default:                                  return "??(未定义的 status)";
  }
}
}  // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════
// 构造 & 参数初始化
// ═══════════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════════
// 点云 → 场地系（坐标系纪律见头文件；2026-09-24 立）
//
// ⚠️ 方向是**整份点云一次性搬过来**，不是"每个位各变换一次"：
//    本模块的常量全是场地系的值（位坐标、平台高、各高度阈值），
//    把**测量**搬到常量的系里做，比把常量拆到测量系里更少出错面。
// ⚠️⚠️ **不做这一步的后果**：`CropBox` 拿场地系坐标去裁雷达系的点云 ⇒
//    机器人不在场原点/朝向不一时**裁到错地方** ⇒ 裁不到点 ⇒ 报 EMPTY
//    ⇒ **主控以为塔没建**。而它在"台架原点静止"时**完全看不出**（两个系恰好重合）。
// ═══════════════════════════════════════════════════════════════════════════
bool BuildingSpotAnalyzer::cloud_to_world(const PointCloudPtr& in,
                                         const std_msgs::msg::Header& hdr,
                                         PointCloudPtr& out)
{
  // 没配场地系（或配成空串）= 明确"不做变换"模式 ⇒ 原样透传
  if (world_frame_.empty()) { out = in; return true; }
  if (hdr.frame_id.empty()) return false;          // ⚠️ 无从变换就**别猜**
  if (hdr.frame_id == world_frame_) { out = in; return true; }   // 上游已换好
  if (!tf_buffer_) return false;

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
        world_frame_, hdr.frame_id, hdr.stamp,
        tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException&) {
    return false;                                   // 不回落、不猜 → 调用方报 UNKNOWN
  }

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
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// 一帧全报 UNKNOWN
// ═══════════════════════════════════════════════════════════════════════════
void BuildingSpotAnalyzer::publish_all_unknown(const std_msgs::msg::Header& header)
{
  std::vector<BuildingSpot> spots;
  spots.reserve(building_positions_.size());
  for (size_t i = 0; i < building_positions_.size(); ++i) {
    BuildingSpot spot;
    spot.id         = static_cast<int>(i);
    spot.spot_x     = static_cast<float>(building_positions_[i].first);
    spot.spot_y     = static_cast<float>(building_positions_[i].second);
    spot.platform_z = spot_platform_z(i);
    // ⚠️⚠️ **UNKNOWN，不是 EMPTY** —— 这一行就是本文件最要紧的一个字。
    spot.status     = BuildingSpotStatus::UNKNOWN;
    spot.height     = 0.0f;      // 没量 → 0；⚠️ 别拿它当"高度是 0"
    spot.point_count = 0;
    spots.push_back(spot);
  }
  publish_spots(spots, header);
  publish_diagnostics(0.0, spots);
}

BuildingSpotAnalyzer::BuildingSpotAnalyzer(const rclcpp::NodeOptions& options)
  : Node("building_spot_analyzer", options)                                           //create_publisher<MessageT>(topic, qos, options) QoS（服务质量）——“通信协议/合同  选项（PublisherOptions）——“发布者的系统配置/管理”
{
  // ── 声明建筑位坐标参数 (从 field_geometry.yaml 通过 ROS params 传入) ──
  this->declare_parameter<std::vector<double>>("building_spot_x");
  this->declare_parameter<std::vector<double>>("building_spot_y");
  this->declare_parameter<std::vector<double>>("building_spot_z");   // 可选: per-spot 平台高

  // ── 声明几何参数 ──
  this->declare_parameter<double>("platform_z",       platform_z_);
  this->declare_parameter<double>("spot_half_size",   spot_half_size_);
  this->declare_parameter<double>("column_z_min",     column_z_min_);
  this->declare_parameter<double>("column_z_max",     column_z_max_);

  // ── 声明检测阈值 ──
  this->declare_parameter<int>("min_points_threshold",   min_points_threshold_);
  this->declare_parameter<double>("top_surface_tolerance", top_surface_tolerance_);
  this->declare_parameter<int>("top_surface_min_points",  top_surface_min_points_);

  // ── 声明分类阈值 ──
  this->declare_parameter<double>("empty_max_height",     empty_max_height_);
  this->declare_parameter<double>("one_earth_min",        one_earth_min_);
  this->declare_parameter<double>("one_earth_max",        one_earth_max_);
  this->declare_parameter<double>("two_earth_min",        two_earth_min_);
  this->declare_parameter<double>("two_earth_max",        two_earth_max_);
  this->declare_parameter<double>("complete_tower_min",   complete_tower_min_);
  this->declare_parameter<double>("complete_tower_max",   complete_tower_max_);

  // ── 坐标系（2026-09-24 新增；⚠️ 名字必须与融合层的 fusion_params.yaml 一致）──
  this->declare_parameter<std::string>("world_frame", world_frame_);
  this->declare_parameter<double>("tf_timeout_sec",   tf_timeout_sec_);

  // ── 读取参数 ──
  #define GET(name, var) this->get_parameter(name, var)                   //C 预处理器宏，在编译前直接把代码里的 GET(name, var) 原封不动地替换为 this->get_parameter(name, var)
  GET("platform_z",            platform_z_);
  GET("spot_half_size",        spot_half_size_);
  GET("column_z_min",          column_z_min_);
  GET("column_z_max",          column_z_max_);
  GET("min_points_threshold",  min_points_threshold_);
  GET("top_surface_tolerance", top_surface_tolerance_);
  GET("top_surface_min_points",top_surface_min_points_);
  GET("empty_max_height",      empty_max_height_);
  GET("one_earth_min",         one_earth_min_);
  GET("one_earth_max",         one_earth_max_);
  GET("two_earth_min",         two_earth_min_);
  GET("two_earth_max",         two_earth_max_);
  GET("complete_tower_min",    complete_tower_min_);
  GET("complete_tower_max",    complete_tower_max_);
  GET("world_frame",           world_frame_);
  GET("tf_timeout_sec",        tf_timeout_sec_);
  #undef GET                                                             // #undef 是 C/C++ 预处理器指令，用于取消定义一个宏 C++ 程序员讲究“命名空间隔离”和“避免污染

  // 建筑位坐标: 从两个独立数组重建为 pair 向量 (避免越界)    #参数服务器读取成对的 X/Y 坐标点，检查错误，并填充类成员容器
  {
    std::vector<double> bx, by;
    this->get_parameter("building_spot_x", bx);    //从 ROS 2 参数服务器按名称检索参数，并将其存储到向量 bx 中。
    this->get_parameter("building_spot_y", by);
    if (bx.size() != by.size()) {                    //检查两个向量的大小是否匹配
      RCLCPP_WARN(this->get_logger(),
        "building_spot_x (%zu) and building_spot_y (%zu) length mismatch; "
        "no building spots configured",
        bx.size(), by.size());
    } else {                                      //为 building_positions_ 预分配内存（假设它是 std::vector<std::pair<double, double>> 或类似结构）。
      building_positions_.reserve(bx.size());
      for (size_t i = 0; i < bx.size(); ++i) {      //遍历索引
        building_positions_.emplace_back(bx[i], by[i]);   //使用来自 bx 和 by 的相应元素就地构造一个点（对/结构体），并将其添加到 building_positions_ 中。
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
  }   //结束外部作用域  C++ 内存优化的好习惯
                                                                                      //this->add_on_set_parameters_callback是 ROS 2 节点提供的接口。作用是注册一个回调函数，
  // ── 动态参数回调 ──                                                                #std::bind(...)：将类成员函数 on_param_change 绑定到当前对象（this）
  param_cb_handle_ = this->add_on_set_parameters_callback(                             //std::placeholders::_1：占位符，代表回调函数将会收到的第一个参数（即 std::vector<rclcpp::Parameter> 变更列表）。
    std::bind(&BuildingSpotAnalyzer::on_param_change, this, std::placeholders::_1));  //param_cb_handle_：返回的句柄（Handle）。用于后续可以随时注销该回调（比如在析构函数中 reset() 掉），防止节点销毁后回调还在触发导致崩溃

  // ── TF：**只读**（把点云搬到场地系；见头文件「坐标系纪律」）──
  //    ⚠️ 不链接 `fusion::CoordinateTransformer`：那会让雷达层反向依赖融合层，
  //       而且它构造时会**无条件广播** TF（多一个广播者就撞 TF_REPEATED_DATA）。
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── 订阅: 非地面点云 ──
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/lidar/non_ground",
    rclcpp::SensorDataQoS(),     //专为传感器数据设计的 QoS（深度默认 5，可靠性为 BEST_EFFORT）。因为激光雷达帧率高，偶尔丢一两帧无关紧要，追求低延迟。
    std::bind(&BuildingSpotAnalyzer::cloud_callback, this, std::placeholders::_1));

  // ── 发布: 建筑位状态 (MarkerArray) ──
  pub_spots_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/perception/lidar/building_spots", 10);
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/perception/lidar/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),                                     //RCLCPL_INFO：ROS 2 的标准信息日志宏，会输出到终端并写入 rosbag 日志文件
    "BuildingSpotAnalyzer ready | "
    "spots=%zu | platform_z=%.2fm | spot_half=%.2fm | "
    "column_z=[%.2f, %.2f] | min_pts=%d | "
    "thresholds: empty<%.2f one_earth[%.2f,%.2f] two_earth[%.2f,%.2f] complete[%.2f,%.2f]",
    building_positions_.size(),
    platform_z_, spot_half_size_,
    column_z_min_, column_z_max_,
    min_points_threshold_,
    empty_max_height_,
    one_earth_min_, one_earth_max_,
    two_earth_min_, two_earth_max_,
    complete_tower_min_, complete_tower_max_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 主回调
// ═══════════════════════════════════════════════════════════════════════════
void BuildingSpotAnalyzer::cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t0 = std::chrono::steady_clock::now();

  // ── 解包 (防御性 try-catch: pcl::fromROSMsg 可能因格式不匹配抛异常) ──
  PointCloudPtr cloud_in = std::make_shared<PointCloud>();
  try {
    pcl::fromROSMsg(*msg, *cloud_in);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "pcl::fromROSMsg failed: %s", e.what());
    return;
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "pcl::fromROSMsg unknown exception");
    return;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // [0] **搬到场地系** —— 本模块所有常量（位坐标/平台高/高度阈值）都是场地系的值
  //     ⚠️ 换不成 ⇒ **报 UNKNOWN，不许当成"空位"**（两件事的动作完全不同）
  //     ⚠️ 注意: 本节点吃的是 `/perception/lidar/non_ground` —— 若上游是
  //        `lidar_perception_node`，那份点云**已经换好了**，这里会走透传分支。
  // ═══════════════════════════════════════════════════════════════════════
  PointCloudPtr world_cloud;
  if (!cloud_to_world(cloud_in, msg->header, world_cloud)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "**点云换不到场地系 '%s'**（输入 frame_id='%s'）⇒ 本帧全部建筑位报 UNKNOWN。"
      "⚠️ 常见原因: ① 没人广播 world→odom（`perception_pipeline` 没起）；"
      "② 上游还没换系；③ `world_frame` 参数与 TF 树上的名字不符",
      world_frame_.c_str(), msg->header.frame_id.c_str());
    publish_all_unknown(msg->header);
    return;
  }
  cloud_in = world_cloud;      // 之后全流程只认场地系

  const size_t input_pts = cloud_in->size();

  // 无建筑位配置或点云过小 → 发布空结果 (但仍记录实际耗时)
  if (building_positions_.empty() || input_pts < static_cast<size_t>(min_points_threshold_)) {
    auto t_now = std::chrono::steady_clock::now();
    using ms = std::chrono::duration<float, std::milli>;
    double elapsed_ms = ms(t_now - t0).count();
    std::vector<BuildingSpot> empty;
    publish_spots(empty, msg->header);
    publish_diagnostics(elapsed_ms, empty);
    return;
  }

  // ── 逐建筑位分析 ──
  std::vector<BuildingSpot> spots;
  spots.reserve(building_positions_.size());

  for (size_t i = 0; i < building_positions_.size(); ++i) {
    BuildingSpot spot = analyze_spot(
        cloud_in,
        static_cast<int>(i),
        static_cast<float>(building_positions_[i].first),
        static_cast<float>(building_positions_[i].second),
        spot_platform_z(i));
    spots.push_back(spot);
  }

  // ── 耗时 ──
  auto t_end = std::chrono::steady_clock::now();
  using ms = std::chrono::duration<float, std::milli>;
  double total_ms = ms(t_end - t0).count();

  // ── 日志摘要 ──
  int empty_cnt = 0, one_cnt = 0, two_cnt = 0, complete_cnt = 0;
  for (const auto& s : spots) {
    switch (s.status) {
      case BuildingSpotStatus::EMPTY:          empty_cnt++;    break;
      case BuildingSpotStatus::ONE_EARTH:      one_cnt++;      break;
      case BuildingSpotStatus::TWO_EARTH:      two_cnt++;      break;
      case BuildingSpotStatus::COMPLETE_TOWER: complete_cnt++; break;
    }
    // 非空建筑位输出详细日志
    if (s.status != BuildingSpotStatus::EMPTY) {
      RCLCPP_DEBUG(this->get_logger(),
        "  spot[%d] (%5.2f, %5.2f) → %s | h=%.3fm | top_surf=%s pts=%d",
        s.id, s.spot_x, s.spot_y, spot_status_name(s.status),
        s.height, s.has_top_surface ? "Y" : "N", s.point_count);
    }
  }
  RCLCPP_DEBUG(this->get_logger(),
    "Spots: %zu total | empty=%d one=%d two=%d complete=%d | %.1f ms",
    spots.size(), empty_cnt, one_cnt, two_cnt, complete_cnt, total_ms);

  // ── 发布 ──
  publish_spots(spots, msg->header);
  publish_diagnostics(total_ms, spots);
}

// ═══════════════════════════════════════════════════════════════════════════
// 单建筑位分析
//
// 步骤:
//   1. CropBox 裁剪垂直柱状区域
//   2. 点数 < 阈值 → EMPTY
//   3. 找最高点 → 顶面检测
//   4. 按高度差分类
// ═══════════════════════════════════════════════════════════════════════════
BuildingSpot BuildingSpotAnalyzer::analyze_spot(
    const PointCloudPtr& cloud,
    int id,
    float spot_x, float spot_y, float platform_z)
{
  BuildingSpot spot;
  spot.id        = id;
  spot.spot_x    = spot_x;
  spot.spot_y    = spot_y;
  spot.platform_z = platform_z;

  // ── 1. CropBox 裁剪垂直柱状区域 ──
  // x: spot_x ± spot_half_size_
  // y: spot_y ± spot_half_size_
  // z: platform_z + column_z_min_ ~ platform_z + column_z_max_
  const float hs = static_cast<float>(spot_half_size_);
  const float z_min = platform_z + static_cast<float>(column_z_min_);
  const float z_max = platform_z + static_cast<float>(column_z_max_);

  pcl::CropBox<pcl::PointXYZ> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(spot_x - hs, spot_y - hs, z_min, 1.0f));
  crop.setMax(Eigen::Vector4f(spot_x + hs, spot_y + hs, z_max, 1.0f));
  crop.setNegative(false);

  PointCloudPtr spot_cloud = std::make_shared<PointCloud>();
  crop.filter(*spot_cloud);

  spot.point_count = static_cast<int>(spot_cloud->size());

  // ── 2. 点数检查 ──
  if (spot.point_count < min_points_threshold_) {
    spot.status = BuildingSpotStatus::EMPTY;
    return spot;
  }

  // ── 3. 找最高点 ──
  float z_global_max = -1e9f;
  for (const auto& pt : spot_cloud->points) {
    if (pt.z > z_global_max) {
      z_global_max = pt.z;
    }
  }

  spot.highest_z = z_global_max;

  // ── 4. 顶面检测 ──
  auto [has_surface, surface_z, surface_pts] = detect_top_surface(spot_cloud, z_global_max);
  spot.has_top_surface   = has_surface;
  spot.top_surface_z     = surface_z;
  spot.top_surface_points = surface_pts;

  // 用于分类的高度: 如果有顶面则使用顶面高度, 否则使用最高点
  const float effective_top_z = has_surface ? surface_z : z_global_max;
  spot.height = effective_top_z - platform_z;

  // ── 5. 按高度分类 ──
  const float h = spot.height;

  if (h < static_cast<float>(empty_max_height_)) {
    spot.status = BuildingSpotStatus::EMPTY;
  }
  else if (h >= static_cast<float>(one_earth_min_) &&
           h <  static_cast<float>(one_earth_max_)) {
    spot.status = BuildingSpotStatus::ONE_EARTH;
  }
  else if (h >= static_cast<float>(two_earth_min_) &&
           h <  static_cast<float>(two_earth_max_)) {
    spot.status = BuildingSpotStatus::TWO_EARTH;
  }
  else if (h >= static_cast<float>(complete_tower_min_) &&
           h <  static_cast<float>(complete_tower_max_)) {
    spot.status = BuildingSpotStatus::COMPLETE_TOWER;
    // 完整塔: 雷达无法判断天空方块朝上颜色 → 标记需要视觉辅助
    spot.needs_visual_check = true;
  }
  else {
    // 高度超出所有分类范围 → 保持 EMPTY (未知高度, 可能是噪点或异常)
    spot.status = BuildingSpotStatus::EMPTY;
  }

  return spot;
}

// ═══════════════════════════════════════════════════════════════════════════
// 顶面检测
//
// 在柱状点云中找到最高点附近、近似水平的点聚类.
//
// 算法:
//   1. 收集 z ∈ [z_max - tolerance, z_max] 范围内的所有点
//   2. 检查这些点的 Z 方差: 如果足够小 → 近水平表面
//   3. 返回 {是否检测到, 顶面平均 Z, 顶面点数}
// ═══════════════════════════════════════════════════════════════════════════
std::tuple<bool, float, int> BuildingSpotAnalyzer::detect_top_surface(
    const PointCloudPtr& spot_cloud, float z_max)
{
  const float tol = static_cast<float>(top_surface_tolerance_);
  const float z_lo = z_max - tol;

  // 收集最高点附近的点
  std::vector<float> top_z_values;
  top_z_values.reserve(spot_cloud->size());

  for (const auto& pt : spot_cloud->points) {
    if (pt.z >= z_lo) {
      top_z_values.push_back(pt.z);
    }
  }

  const int top_count = static_cast<int>(top_z_values.size());
  if (top_count < top_surface_min_points_) {
    return {false, z_max, top_count};
  }

  // 计算平均 Z 和方差
  float sum_z = 0.0f;
  for (float z : top_z_values) {
    sum_z += z;
  }
  const float mean_z = sum_z / static_cast<float>(top_count);

  float var_z = 0.0f;
  for (float z : top_z_values) {
    const float dz = z - mean_z;
    var_z += dz * dz;
  }
  var_z /= static_cast<float>(top_count);

  // 标准差小于 tolerance 的一半 → 认为是近水平表面
  const float std_z = std::sqrt(var_z);
  const bool is_flat = (std_z < tol * 0.5f);

  return {is_flat, mean_z, top_count};
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: MarkerArray (建筑位状态可视化)
//
// 每个建筑位用彩色方块标记:
//   EMPTY          → 浅灰, 半透明
//   ONE_EARTH      → 深绿 (地球方块色)
//   TWO_EARTH      → 浅绿
//   COMPLETE_TOWER → 金色, 附文字标签 "需要视觉确认"
// ═══════════════════════════════════════════════════════════════════════════
void BuildingSpotAnalyzer::publish_spots(
    const std::vector<BuildingSpot>& spots,
    const std_msgs::msg::Header& header)
{
  visualization_msgs::msg::MarkerArray arr;

  for (const auto& spot : spots) {
    visualization_msgs::msg::Marker m;
    m.header = header;
    m.ns     = "building_spots";
    m.id     = spot.id;
    m.type   = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    // 中心位置: XY 在建筑位中心, Z 在最高点一半处
    m.pose.position.x = spot.spot_x;
    m.pose.position.y = spot.spot_y;
    m.pose.position.z = spot.platform_z + spot.height * 0.5f;
    m.pose.orientation.w = 1.0;

    // 尺寸: 500×500mm 底面积 × 检测高度
    m.scale.x = static_cast<double>(spot_half_size_) * 2.0;
    m.scale.y = static_cast<double>(spot_half_size_) * 2.0;
    m.scale.z = std::max(static_cast<double>(spot.height), 0.01);

    // 颜色按状态
    switch (spot.status) {
      case BuildingSpotStatus::ONE_EARTH:
        m.color.r = 0.16f; m.color.g = 0.39f; m.color.b = 0.20f;  // 深绿
        m.color.a = 0.8f;
        break;
      case BuildingSpotStatus::TWO_EARTH:
        m.color.r = 0.27f; m.color.g = 0.55f; m.color.b = 0.27f;  // 浅绿
        m.color.a = 0.8f;
        break;
      case BuildingSpotStatus::COMPLETE_TOWER:
        m.color.r = 0.85f; m.color.g = 0.65f; m.color.b = 0.13f;  // 金
        m.color.a = 0.9f;
        break;
      case BuildingSpotStatus::EMPTY:
      default:
        m.color.r = 0.6f; m.color.g = 0.6f; m.color.b = 0.6f;     // 浅灰
        m.color.a = 0.3f;
        break;
    }

    m.lifetime = rclcpp::Duration::from_seconds(1.0);
    arr.markers.push_back(m);

    // ── 完整塔: 文字标签 (提示需要视觉确认天空方块颜色) ──
    if (spot.needs_visual_check) {
      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns     = "spot_labels";
      text.id     = spot.id + 10000;
      text.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;

      text.pose.position.x = spot.spot_x;
      text.pose.position.y = spot.spot_y;
      text.pose.position.z = spot.platform_z + spot.height + 0.08f;
      text.pose.orientation.w = 1.0;

      text.scale.z = 0.07f;  // 文字高度 7cm

      char buf[64];
      snprintf(buf, sizeof(buf), "Tower h=%.2f | check sky", static_cast<double>(spot.height));
      text.text = buf;

      text.color.r = 1.0f; text.color.g = 0.84f; text.color.b = 0.0f;  // 金色文字
      text.color.a = 1.0f;
      text.lifetime = rclcpp::Duration::from_seconds(1.0);
      arr.markers.push_back(text);
    }
  }

  // ── 清理过期 marker ──
  if (spots.empty()) {
    visualization_msgs::msg::Marker del;
    del.header = header;
    del.ns     = "building_spots";
    del.id     = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    visualization_msgs::msg::Marker del2;
    del2.header = header;
    del2.ns     = "spot_labels";
    del2.id     = 0;
    del2.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del2);
  }

  pub_spots_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布: Diagnostics
// ═══════════════════════════════════════════════════════════════════════════
void BuildingSpotAnalyzer::publish_diagnostics(
    double total_ms,
    const std::vector<BuildingSpot>& spots)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "building_spot_analyzer";
  st.hardware_id = "mid360";

  if (total_ms < 10.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "OK";
  } else if (total_ms < 20.0) {
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

  add("total_ms",      total_ms);
  add("spot_count",    static_cast<double>(spots.size()));

  int empty_cnt = 0, one_cnt = 0, two_cnt = 0, complete_cnt = 0, unknown_cnt = 0;
  int visual_check_needed = 0;
  for (const auto& s : spots) {
    switch (s.status) {
      case BuildingSpotStatus::EMPTY:          empty_cnt++;    break;
      case BuildingSpotStatus::ONE_EARTH:      one_cnt++;      break;
      case BuildingSpotStatus::TWO_EARTH:      two_cnt++;      break;
      case BuildingSpotStatus::COMPLETE_TOWER: complete_cnt++; break;
      // ⚠️ **单独一档**：`unknown` 涨 = "没量成"（TF 不可用等），与"真量到是空的"
      //    是两件事 —— 现场的读法完全不同（见 `BuildingSpotStatus::UNKNOWN`）。
      case BuildingSpotStatus::UNKNOWN:        unknown_cnt++;  break;
    }
    if (s.needs_visual_check) visual_check_needed++;
  }

  add("empty_spots",    static_cast<double>(empty_cnt));
  add("one_earth",      static_cast<double>(one_cnt));
  add("two_earth",      static_cast<double>(two_cnt));
  add("complete_tower", static_cast<double>(complete_cnt));
  // ⚠️ 与 `empty_spots` **必须分开读**（见 `BuildingSpotStatus::UNKNOWN`）
  add("unknown_spots",  static_cast<double>(unknown_cnt));
  add("visual_check_needed", static_cast<double>(visual_check_needed));

  arr.status.push_back(st);
  pub_diagnostics_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult BuildingSpotAnalyzer::on_param_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool positions_changed = false;

  for (const auto& p : params) {
    const std::string& name = p.get_name();

    try {
      #define SET(n, v)     if (name == n) v = p.as_double()
      #define SET_I(n, v)   if (name == n) v = p.as_int()
      #define ARRAY_CHANGED(n) if (name == n) positions_changed = true

      SET("platform_z",             platform_z_);
      SET("spot_half_size",         spot_half_size_);
      SET("column_z_min",           column_z_min_);
      SET("column_z_max",           column_z_max_);
      SET_I("min_points_threshold", min_points_threshold_);
      SET("top_surface_tolerance",  top_surface_tolerance_);
      SET_I("top_surface_min_points", top_surface_min_points_);
      SET("empty_max_height",       empty_max_height_);
      SET("one_earth_min",          one_earth_min_);
      SET("one_earth_max",          one_earth_max_);
      SET("two_earth_min",          two_earth_min_);
      SET("two_earth_max",          two_earth_max_);
      SET("complete_tower_min",     complete_tower_min_);
      SET("complete_tower_max",     complete_tower_max_);

      ARRAY_CHANGED("building_spot_x");
      ARRAY_CHANGED("building_spot_y");

      #undef SET
      #undef SET_I
      #undef ARRAY_CHANGED
    }
    catch (const rclcpp::ParameterTypeException& e) {
      result.successful = false;
      result.reason = std::string("Type mismatch: ") + name + " — " + e.what();
      break;
    }
  }

  // 建筑位坐标变更 → 重新读取并重建 pair 向量
  if (positions_changed && result.successful) {
    std::vector<double> bx, by;
    this->get_parameter("building_spot_x", bx);
    this->get_parameter("building_spot_y", by);
    if (bx.size() == by.size()) {
      building_positions_.clear();
      building_positions_.reserve(bx.size());
      for (size_t i = 0; i < bx.size(); ++i) {
        building_positions_.emplace_back(bx[i], by[i]);
      }
      RCLCPP_INFO(this->get_logger(),
        "Building positions updated: %zu spots", building_positions_.size());
    } else {
      result.successful = false;
      result.reason = "building_spot_x and building_spot_y must have same length";
    }
  }

  // ── 交叉验证: 分类阈值区间合法性 ──
  if (result.successful) {
    std::string reject_reason;
    auto reject = [&](const std::string& r) { result.successful = false; reject_reason = r; };

    // 各区间必须 min < max
    if (one_earth_min_ >= one_earth_max_)
      reject("one_earth_min >= one_earth_max");
    else if (two_earth_min_ >= two_earth_max_)
      reject("two_earth_min >= two_earth_max");
    else if (complete_tower_min_ >= complete_tower_max_)
      reject("complete_tower_min >= complete_tower_max");
    // 区间不可重叠: EMPTY < ONE_EARTH < TWO_EARTH < COMPLETE_TOWER
    else if (empty_max_height_ > one_earth_min_)
      reject("empty_max_height > one_earth_min (overlap)");
    else if (one_earth_max_ > two_earth_min_)
      reject("one_earth_max > two_earth_min (overlap)");
    else if (two_earth_max_ > complete_tower_min_)
      reject("two_earth_max > complete_tower_min (overlap)");
    // 基本范围检查
    else if (spot_half_size_ <= 0.0)
      reject("spot_half_size must be > 0");
    else if (min_points_threshold_ < 1)
      reject("min_points_threshold must be >= 1");
    else if (column_z_min_ >= column_z_max_)
      reject("column_z_min >= column_z_max");

    if (!result.successful) {
      result.reason = reject_reason;
    }
  }

  if (result.successful) {
    RCLCPP_DEBUG(this->get_logger(),
      "Params updated | platform_z=%.2f spot_half=%.2f column=[%.2f,%.2f] "
      "min_pts=%d empty<%.2f one[%.2f,%.2f] two[%.2f,%.2f] complete[%.2f,%.2f]",
      platform_z_, spot_half_size_,
      column_z_min_, column_z_max_,
      min_points_threshold_,
      empty_max_height_,
      one_earth_min_, one_earth_max_,
      two_earth_min_, two_earth_max_,
      complete_tower_min_, complete_tower_max_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

}  // namespace lidar
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::lidar::BuildingSpotAnalyzer)
