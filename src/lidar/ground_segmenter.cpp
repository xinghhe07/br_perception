#include "br_perception/lidar/ground_segmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <string>
#include <stdexcept>

namespace br_perception {
namespace lidar {

// ═══════════════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════════════
GroundSegmenter::GroundSegmenter(const rclcpp::NodeOptions& options)
  : Node("ground_segmenter", options)
{
  // ── 声明参数 ──
  this->declare_parameter<double>("ransac_distance_threshold", ransac_distance_threshold_);
  this->declare_parameter<int>("ransac_max_iterations", ransac_max_iterations_);
  this->declare_parameter<double>("ransac_eps_angle", ransac_eps_angle_);
  this->declare_parameter<bool>("enable_normal_constraint", enable_normal_constraint_);
  this->declare_parameter<double>("normal_angle_threshold", normal_angle_threshold_);
  this->declare_parameter<double>("l0_height", l0_height_);
  this->declare_parameter<double>("l1_height", l1_height_);
  this->declare_parameter<double>("l2_height", l2_height_);
  this->declare_parameter<double>("layer_tolerance", layer_tolerance_);
  // 动态阈值映射: 从 YAML 读取两个等长数组
  this->declare_parameter<std::vector<double>>(
    "height_threshold_z_keys", {0.0, 1.0, 3.0});
  this->declare_parameter<std::vector<double>>(
    "height_threshold_values", {0.04, 0.06, 0.10});

  // ── 读取参数 ──
  #define GET(name, var) this->get_parameter(name, var)
  GET("ransac_distance_threshold", ransac_distance_threshold_);
  GET("ransac_max_iterations",     ransac_max_iterations_);
  GET("ransac_eps_angle",          ransac_eps_angle_);
  GET("enable_normal_constraint",  enable_normal_constraint_);
  GET("normal_angle_threshold",    normal_angle_threshold_);
  GET("l0_height",                 l0_height_);
  GET("l1_height",                 l1_height_);
  GET("l2_height",                 l2_height_);
  GET("layer_tolerance",           layer_tolerance_);
  #undef GET

  // 从两个等长数组重建 height_threshold_map_
  {
    std::vector<double> z_keys, thresh_vals;
    this->get_parameter("height_threshold_z_keys", z_keys);
    this->get_parameter("height_threshold_values", thresh_vals);
    if (!z_keys.empty() && z_keys.size() == thresh_vals.size()) {
      height_threshold_map_.clear();
      for (size_t i = 0; i < z_keys.size(); ++i) {
        height_threshold_map_.emplace_back(z_keys[i], thresh_vals[i]);
      }
    }
    // 不等长 → 保留头文件中的默认值, 打印警告
    else if (!z_keys.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "height_threshold_z_keys (%zu) and values (%zu) length mismatch; using defaults",
        z_keys.size(), thresh_vals.size());
    }
  }

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&GroundSegmenter::on_parameter_change, this, std::placeholders::_1));

  // ── 订阅 ──
  sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/perception/lidar/filtered",
    rclcpp::SensorDataQoS(),
    std::bind(&GroundSegmenter::pointcloud_callback, this, std::placeholders::_1));

  // ── 发布 ──
  pub_ground_      = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                       "/perception/lidar/ground", rclcpp::SensorDataQoS());
  pub_non_ground_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                       "/perception/lidar/non_ground", rclcpp::SensorDataQoS());
  pub_l0_surface_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                       "/perception/lidar/l0_surface", rclcpp::SensorDataQoS());
  pub_l1_surface_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                       "/perception/lidar/l1_surface", rclcpp::SensorDataQoS());
  pub_l2_surface_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                       "/perception/lidar/l2_surface", rclcpp::SensorDataQoS());
  pub_plane_       = this->create_publisher<std_msgs::msg::Float32MultiArray>(
                       "/perception/lidar/ground_plane", 10);
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                       "/perception/lidar/diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "GroundSegmenter ready | thresh=%.3fm iter=%d eps_angle=%.2f normal_const=%s | "
    "validate_angle=%.2f | map_z=[%.1f,%.1f,%.1f] map_th=[%.3f,%.3f,%.3f] | "
    "layers L0=%.1f L1=%.1f L2=%.1f",
    ransac_distance_threshold_, ransac_max_iterations_,
    ransac_eps_angle_, enable_normal_constraint_ ? "ON" : "OFF",
    normal_angle_threshold_,
    height_threshold_map_[0].first, height_threshold_map_[1].first,
    height_threshold_map_[2].first,
    height_threshold_map_[0].second, height_threshold_map_[1].second,
    height_threshold_map_[2].second,
    l0_height_, l1_height_, l2_height_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 主回调
// ═══════════════════════════════════════════════════════════════════════════
void GroundSegmenter::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto t0 = std::chrono::steady_clock::now();

  // ── 解包 ──
  PointCloudPtr cloud_in = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *cloud_in);

  const size_t input_pts = cloud_in->size();
  if (input_pts < 50) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                         3000, "Too few points (%zu)", input_pts);
    return;
  }

  // ── 1. RANSAC 地面拟合 (带 try-catch) ──
  pcl::ModelCoefficients coeff;
  pcl::PointIndices inliers;
  if (!fit_ground_plane(cloud_in, coeff, inliers)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                         3000, "RANSAC failed to find ground plane");
    return;
  }
  auto t1 = std::chrono::steady_clock::now();

  // ── 2. 法向量验证 ──
  if (!validate_plane_normal(coeff)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Ground plane tilted: normal=[%.3f,%.3f,%.3f] angle=%.1f° — using result anyway",
      coeff.values[0], coeff.values[1], coeff.values[2],
      std::acos(std::clamp(std::abs(coeff.values[2]) /
        std::sqrt(coeff.values[0]*coeff.values[0] + coeff.values[1]*coeff.values[1] + coeff.values[2]*coeff.values[2]), -1.0f, 1.0f))
        * 180.0 / M_PI);
  }

  // ── 3. 动态阈值精筛 ──
  pcl::PointIndices refined = refine_inliers_adaptive(cloud_in, coeff);
  auto t2 = std::chrono::steady_clock::now();

  // ── 4. 分离地面 & 非地面 ──
  PointCloudPtr ground     = std::make_shared<PointCloud>();
  PointCloudPtr non_ground = std::make_shared<PointCloud>();
  extract_ground_and_non_ground(cloud_in, refined, ground, non_ground);

  // ── 5. 地面点分层 (L0/L1/L2) ──
  PointCloudPtr l0_surface = std::make_shared<PointCloud>();
  PointCloudPtr l1_surface = std::make_shared<PointCloud>();
  PointCloudPtr l2_surface = std::make_shared<PointCloud>();
  classify_by_platform(ground, l0_surface, l1_surface, l2_surface);

  // ── 6. 归一化平面系数 ──
  normalize_coefficients(coeff);

  // ── 耗时 ──
  using ms = std::chrono::duration<float, std::milli>;
  double total_ms  = ms(t2 - t0).count();
  double ransac_ms = ms(t1 - t0).count();
  double refine_ms = ms(t2 - t1).count();

  // ── 7. 发布 ──
  publish_results(ground, non_ground, l0_surface, l1_surface, l2_surface,
                  coeff, msg->header);
  publish_diagnostics(total_ms, ransac_ms, refine_ms,
                      input_pts, ground->size(), non_ground->size(),
                      l0_surface->size(), l1_surface->size(), l2_surface->size());
}

// ═══════════════════════════════════════════════════════════════════════════
// RANSAC 平面拟合 (带 try-catch 防御)
// ═══════════════════════════════════════════════════════════════════════════
bool GroundSegmenter::fit_ground_plane(const PointCloudPtr& cloud,
                                       pcl::ModelCoefficients& coeff,
                                       pcl::PointIndices& inliers)
{
  try {
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ransac_distance_threshold_);
    seg.setMaxIterations(ransac_max_iterations_);
    seg.setInputCloud(cloud);

    if (enable_normal_constraint_) {
      // 约束法向量大致朝上, 加速搜索并减少竖直面误匹配
      seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
      seg.setEpsAngle(ransac_eps_angle_);
    }

    seg.segment(inliers, coeff);
    return !inliers.indices.empty();
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "RANSAC exception: %s", e.what());
    return false;
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "RANSAC unknown exception");
    return false;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 法向量验证
// ═══════════════════════════════════════════════════════════════════════════
bool GroundSegmenter::validate_plane_normal(const pcl::ModelCoefficients& coeff,
                                            double angle_threshold)
{
  // ⚠️ `coeff.values` 的**长度不保证是 4**（PCL 的 `ModelCoefficients` 是个裸 `vector<float>`）
  //    ⇒ 不检查就索引是**越界读**。正常 RANSAC 会给 4 个，但这是公开函数，要自己兜住。
  if (coeff.values.size() < 4) return false;

  const double a = coeff.values[0];
  const double b = coeff.values[1];
  const double c = coeff.values[2];
  const double norm = std::sqrt(a * a + b * b + c * c);
  // ⚠️ 零法向量 ⇒ **直接 false**（不是让它去算 acos(0/0)=NaN）。
  //    侥幸的是 `NaN < threshold` 恒为 false、结果一样 ——
  //    但**一旦 NaN 从这里流出去就会污染上游的一切比较**。
  if (norm < 1e-9) return false;

  // 与 Z 轴 [0,0,1] 的夹角
  // ⚠️ **取 `|c|`**：RANSAC 给的系数朝向是任意的，不取绝对值的话**一半的地面会被否掉**。
  const double cos_angle = std::abs(c) / norm;
  // ⚠️ `std::clamp` 防浮点越界（`|c|/norm` 差一点点超过 1 会让 `acos` 出 NaN）
  const double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));
  // ⚠️ **严格 `<`**：恰好等于阈值不算地面
  return angle < angle_threshold;
}

// ── 成员薄包装：行为逐位不变 ──
bool GroundSegmenter::validate_plane_normal(const pcl::ModelCoefficients& coeff) const
{
  return validate_plane_normal(coeff, normal_angle_threshold_);
}

// ═══════════════════════════════════════════════════════════════════════════
// 动态阈值精筛
// 设计: 雷达噪声随距离增大 → 远处用更宽松的阈值
//       阈值按点云相对高度(z)线性插值查表
// ═══════════════════════════════════════════════════════════════════════════
pcl::PointIndices GroundSegmenter::refine_inliers_adaptive(
    const PointCloudPtr& cloud,
    const pcl::ModelCoefficients& coeff) const
{
  pcl::PointIndices refined;
  refined.indices.reserve(cloud->size() / 2);

  const double norm = std::sqrt(coeff.values[0] * coeff.values[0] +
                                coeff.values[1] * coeff.values[1] +
                                coeff.values[2] * coeff.values[2]);
  if (norm < 1e-9) return refined;

  const auto& hmap = height_threshold_map_;
  const double fallback_thr = ransac_distance_threshold_;

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& pt = cloud->points[i];

    const double dist = std::abs(coeff.values[0] * pt.x +
                                 coeff.values[1] * pt.y +
                                 coeff.values[2] * pt.z + coeff.values[3]) / norm;

    // 按照 pt.z 查动态阈值表 (线性插值)
    double thr = fallback_thr;
    const double z = static_cast<double>(pt.z);

    if (!hmap.empty()) {
      if (z <= hmap.front().first) {
        thr = hmap.front().second;
      } else if (z >= hmap.back().first) {
        thr = hmap.back().second;
      } else {
        for (size_t k = 0; k < hmap.size() - 1; ++k) {
          if (z >= hmap[k].first && z < hmap[k + 1].first) {
            const double t = (z - hmap[k].first) /
                             (hmap[k + 1].first - hmap[k].first);
            thr = hmap[k].second + t * (hmap[k + 1].second - hmap[k].second);
            break;
          }
        }
      }
    }

    if (dist < thr) {
      refined.indices.push_back(i);
    }
  }

  return refined;
}

// ═══════════════════════════════════════════════════════════════════════════
// 分离地面 / 非地面
// ═══════════════════════════════════════════════════════════════════════════
void GroundSegmenter::extract_ground_and_non_ground(
    const PointCloudPtr& cloud,
    const pcl::PointIndices& inliers,
    PointCloudPtr& ground,
    PointCloudPtr& non_ground) const
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);

  auto inliers_ptr = std::make_shared<pcl::PointIndices>(inliers);

  extract.setIndices(inliers_ptr);
  extract.setNegative(false);
  extract.filter(*ground);

  extract.setNegative(true);
  extract.filter(*non_ground);
}

// ═══════════════════════════════════════════════════════════════════════════
// 地面点按绝对高度分层
// 只在地面点中分类: 匹配 L0/L1/L2 平台高度的点标记为对应表面
// ═══════════════════════════════════════════════════════════════════════════
void GroundSegmenter::classify_by_platform(
    const PointCloudPtr& ground,
    PointCloudPtr& l0_surface,
    PointCloudPtr& l1_surface,
    PointCloudPtr& l2_surface) const
{
  l0_surface->clear();
  l1_surface->clear();
  l2_surface->clear();

  const double tol = layer_tolerance_;

  for (const auto& pt : ground->points) {
    if (std::abs(pt.z - l0_height_) < tol) {
      l0_surface->push_back(pt);
    } else if (std::abs(pt.z - l1_height_) < tol) {
      l1_surface->push_back(pt);
    } else if (std::abs(pt.z - l2_height_) < tol) {
      l2_surface->push_back(pt);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 归一化平面系数 → a²+b²+c² = 1
// ═══════════════════════════════════════════════════════════════════════════
void GroundSegmenter::normalize_coefficients(pcl::ModelCoefficients& coeff) const
{
  const double n = std::sqrt(coeff.values[0] * coeff.values[0] +
                             coeff.values[1] * coeff.values[1] +
                             coeff.values[2] * coeff.values[2]);
  if (n < 1e-9) return;
  coeff.values[0] /= n;
  coeff.values[1] /= n;
  coeff.values[2] /= n;
  coeff.values[3] /= n;
}

// ═══════════════════════════════════════════════════════════════════════════
// 发布结果
// 即使是空点云也发布 (带有效 header), 避免下游等待超时
// ═══════════════════════════════════════════════════════════════════════════
void GroundSegmenter::publish_results(
    const PointCloudPtr& ground,
    const PointCloudPtr& non_ground,
    const PointCloudPtr& l0_surface,
    const PointCloudPtr& l1_surface,
    const PointCloudPtr& l2_surface,
    const pcl::ModelCoefficients& coeff,
    const std_msgs::msg::Header& header)
{
  // 辅助: 无条件发布 (空点云也发, 带有效 header)
  auto publish_always = [&](
      const PointCloudPtr& cloud,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud, ros_msg);
    ros_msg.header = header;
    pub->publish(ros_msg);
  };

  publish_always(ground,      pub_ground_);
  publish_always(non_ground,  pub_non_ground_);
  publish_always(l0_surface,  pub_l0_surface_);
  publish_always(l1_surface,  pub_l1_surface_);
  publish_always(l2_surface,  pub_l2_surface_);

  // 平面系数 (已归一化)
  auto plane_msg = std_msgs::msg::Float32MultiArray();
  plane_msg.data.resize(4);
  plane_msg.data[0] = static_cast<float>(coeff.values[0]);
  plane_msg.data[1] = static_cast<float>(coeff.values[1]);
  plane_msg.data[2] = static_cast<float>(coeff.values[2]);
  plane_msg.data[3] = static_cast<float>(coeff.values[3]);
  pub_plane_->publish(plane_msg);
}

// ═══════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════
void GroundSegmenter::publish_diagnostics(
    double total_ms, double ransac_ms, double refine_ms,
    size_t input_pts, size_t ground_pts, size_t non_ground_pts,
    size_t l0_pts, size_t l1_pts, size_t l2_pts)
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name       = "ground_segmenter";
  status.hardware_id = "mid360";

  if (total_ms < 12.0 && ground_pts > 0) {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  } else if (total_ms < 20.0) {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "SLOW";
  } else {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "VERY SLOW or NO GROUND";
  }

  auto add = [&](const std::string& k, double v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k; kv.value = std::to_string(v);
    status.values.push_back(kv);
  };

  add("total_ms",       total_ms);
  add("ransac_ms",      ransac_ms);
  add("refine_ms",      refine_ms);
  add("input_points",   static_cast<double>(input_pts));
  add("ground_points",  static_cast<double>(ground_pts));
  add("non_ground_pts", static_cast<double>(non_ground_pts));
  add("l0_pts",         static_cast<double>(l0_pts));
  add("l1_pts",         static_cast<double>(l1_pts));
  add("l2_pts",         static_cast<double>(l2_pts));
  add("ground_ratio",   (input_pts > 0) ? static_cast<double>(ground_pts) / input_pts : 0.0);

  diag_array.status.push_back(status);
  pub_diagnostics_->publish(diag_array);
}

// ═══════════════════════════════════════════════════════════════════════════
// 动态参数更新
// ═══════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult GroundSegmenter::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool thresholds_changed = false;

  for (const auto& p : params) {
    const std::string& name = p.get_name();

    try {
      if (name == "ransac_distance_threshold") {
        double v = p.as_double();
        if (v <= 0.0 || v > 0.5) {
          result.successful = false;
          result.reason = "ransac_distance_threshold must be in (0, 0.5]";
          break;
        }
        ransac_distance_threshold_ = v;
      }
      else if (name == "ransac_max_iterations") {
        int v = p.as_int();
        if (v < 10 || v > 5000) {
          result.successful = false;
          result.reason = "ransac_max_iterations must be in [10, 5000]";
          break;
        }
        ransac_max_iterations_ = v;
      }
      else if (name == "ransac_eps_angle") {
        double v = p.as_double();
        if (v < 0.0 || v > 1.5) {
          result.successful = false;
          result.reason = "ransac_eps_angle must be in [0, 1.5] rad";
          break;
        }
        ransac_eps_angle_ = v;
      }
      else if (name == "enable_normal_constraint") {
        enable_normal_constraint_ = p.as_bool();
      }
      else if (name == "normal_angle_threshold") {
        double v = p.as_double();
        if (v <= 0.0 || v > 1.0) {
          result.successful = false;
          result.reason = "normal_angle_threshold must be in (0, 1.0] rad";
          break;
        }
        normal_angle_threshold_ = v;
      }
      else if (name == "l0_height")       { l0_height_       = p.as_double(); }
      else if (name == "l1_height")       { l1_height_       = p.as_double(); }
      else if (name == "l2_height")       { l2_height_       = p.as_double(); }
      else if (name == "layer_tolerance") { layer_tolerance_ = p.as_double(); }
      else if (name == "height_threshold_z_keys" ||
               name == "height_threshold_values") {
        thresholds_changed = true;
      }
    }
    catch (const rclcpp::ParameterTypeException& e) {
      result.successful = false;
      result.reason = std::string("Type mismatch: ") + name;
      break;
    }
  }

  // 如果高度阈值映射的两个数组中有任一变更, 整表重建
  if (thresholds_changed && result.successful) {
    std::vector<double> z_keys, thresh_vals;
    this->get_parameter("height_threshold_z_keys", z_keys);
    this->get_parameter("height_threshold_values", thresh_vals);
    if (z_keys.size() == thresh_vals.size() && !z_keys.empty()) {
      height_threshold_map_.clear();
      for (size_t i = 0; i < z_keys.size(); ++i) {
        height_threshold_map_.emplace_back(z_keys[i], thresh_vals[i]);
      }
    }
  }

  if (result.successful) {
    RCLCPP_DEBUG(this->get_logger(),
      "Params updated | thresh=%.3f iter=%d eps_angle=%.2f normal_const=%s | "
      "L0=%.1f L1=%.1f L2=%.1f",
      ransac_distance_threshold_, ransac_max_iterations_,
      ransac_eps_angle_, enable_normal_constraint_ ? "ON" : "OFF",
      l0_height_, l1_height_, l2_height_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Param update rejected: %s", result.reason.c_str());
  }

  return result;
}

}  // namespace lidar
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::lidar::GroundSegmenter)
