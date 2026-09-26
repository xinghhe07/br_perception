#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// camera_depth_node.hpp — D435i 深度辅助节点（任务书 §3.0 三相机并行）
///
/// **单一职责**：给我一个目标 ROI，我告诉你那里的**鲁棒深度**。
///   不做识别、不跑 YOLO、**不做塔高交叉验证**（那是 building_height_verifier 的事）。
///
/// 两种模式（2026-09-20 二版）：
///   ① **IDLE**（巡航）—— **不做任何逐像素工作**，只按 `diag_rate_hz` 发低频健康检查。
///      D435i 是动作阶段辅助传感器，巡航时没必要每帧扫全图。
///   ② **MEASURE**（动作）—— 收到 `/camera/depth/target_roi` 后进入，只处理那个 ROI。
///      `roi_timeout_sec` 内没收到新的 ROI 就退回 IDLE。
///
/// ⚠️ IDLE 心跳有**两条路**（2026-09-21 VM 实测后补的，缺一不可）：
///    · 有帧时 —— `depth_callback` 里按 `diag_rate_hz` 节流发（顺带报处理耗时）。
///    · 没帧时 —— `diag_timer_` 兜底。**为什么必须有兜底**：只靠回调那条的话，
///      "D435i 掉线"和"节点死了"在诊断上**长得一模一样**（都是彻底静默），
///      而区分这两件事正是诊断存在的唯一理由。超过 `stream_timeout_sec`
///      没帧就发一条 ERROR 诊断，把"流断了"明说出来。
///      有帧时兜底那条什么都不做，**不会重复发**。
///
/// ⚠️ **为什么不用"最小深度"**（一版的错）：
///    最小值会被噪声点 / 边缘飞点 / 反光 / 目标外的近处物体**劫持**。
///    现在改成「**最近的显著深度簇**」——从近到远扫深度直方图，取第一个
///    像素数达标的 bin 区间，再算它的**中值**与**质心**。成簇要求把飞点挡掉。
///
/// ⚠️ 三条不能违背的约束：
///    · QoS **depth = 1** —— 晚了就丢，绝不堆积
///    · 独立回调，**不阻塞主感知链**
///    · 时间戳 = **曝光时刻**（继承输入），禁止 now()
///
/// ⚠️ 本节点**不依赖任何外参标定** —— 它只输出相机坐标系下的深度与像素坐标，
///    转世界坐标是下游（有 TF 的那一层）的事。
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <opencv2/core.hpp>

#include "br_perception/msg/camera_depth.hpp"

#include <cstdint>
#include <string>
#include <mutex>

namespace br_perception {
namespace camera {

/**
 * @brief D435i 深度辅助节点
 */
class CameraDepthNode : public rclcpp::Node
{
public:
  explicit CameraDepthNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief 模式（与 msg/CameraDepth.msg 的 mode 字段逐值一致）
  static constexpr uint8_t kModeIdle    = br_perception::msg::CameraDepth::MODE_IDLE;
  static constexpr uint8_t kModeMeasure = br_perception::msg::CameraDepth::MODE_MEASURE;

private:
  // ── ROS 回调 ──
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  /// @brief 收到上层给的目标 ROI → 进 MEASURE 模式。
  /// 上层（有 TF 与跨相机外参的那一层）负责把检测框投影成"在 D435i 图里的 ROI"。
  void roi_callback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg);

  // ── 核心算法 ──
  /// @brief ROI 内「最近的显著深度簇」的统计量
  struct ClusterStats {
    bool   found        = false;
    double depth_median = 0.0;   // 主用值
    double p25          = 0.0;
    double p75          = 0.0;
    // ⚠️ **深度一致性**，不是"平整度" —— std 大可能是斜面 / 球体曲面 /
    //    多物体混合 / 深度边缘 / 噪声 / ROI 含背景。别当"表面必须平"用。
    double stddev       = 0.0;
    double u            = 0.0;   // 簇的**质心**（原图像素坐标）—— 不是 ROI 中心
    double v            = 0.0;
    int    count        = 0;     // 簇内像素数

    // Pass 1 顺带数出来的，供调用方算 valid_ratio —— **不要再扫第三遍**
    int    total_px     = 0;
    int    valid_px     = 0;
  };

  /// @brief 从近到远扫深度直方图，取第一个**成簇像素数达标**的区间，再算其统计量。
  /// @return found=true 才表示找到了有效簇；否则各字段无意义。
  ClusterStats nearest_cluster(const cv::Mat& depth,
                               int x0, int y0, int x1, int y1) const;

  static bool depth_at(const cv::Mat& depth, int u, int v,
                       double depth_scale_m, double& meters);

  void publish_diagnostics(double proc_ms, bool valid, bool found, int count,
                           double median_m, double stddev_m);

  /// @brief 兜底心跳：有帧时**什么都不做**；超过 stream_timeout_sec_ 没帧才发 ERROR 诊断。
  ///        与 publish_diagnostics 共用 last_diag_time_ 限频，两条路不会重复发。
  void idle_diag_tick();

  /// @brief "深度流断了" 的诊断（ERROR 级）—— 与 publish_diagnostics 分开，
  ///        因为 valid=false 在那边的语义是"帧到了但有效像素不够"，是两回事。
  void publish_no_stream_diagnostics(double stale_sec);

  // ── 参数 ──
  std::string depth_topic_;
  std::string roi_topic_;
  double depth_scale_m_{0.001};     // 16UC1 的换算（默认 mm → m）
  double valid_ratio_min_{0.10};
  double depth_min_m_{0.10};
  double depth_max_m_{10.0};
  double hist_bin_m_{0.05};         // 深度直方图 bin 宽
  int    min_cluster_px_{30};       // 成簇最小像素数（挡飞点）
  double roi_timeout_sec_{0.5};     // 多久没收到 ROI 就回 IDLE
  double diag_rate_hz_{1.0};        // 诊断发布频率（**不是每帧**）
  double stream_timeout_sec_{1.0};  // 多久没帧就判定"深度流断了"并发 ERROR 诊断

  // ── 订阅 / 发布 ──
  // ⚠️ 全部 QoS depth=1：晚了就丢
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr sub_roi_;
  rclcpp::Publisher<br_perception::msg::CameraDepth>::SharedPtr pub_depth_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  // ⚠️ 兜底心跳定时器：没有深度帧时唯一还活着的东西（见类头说明）
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // ── 运行时状态 ──
  //   ⚠️ roi_ / has_roi_ 被两个回调并发访问（depth 回调读、roi 回调写）→ 必须加锁
  std::mutex roi_mutex_;
  sensor_msgs::msg::RegionOfInterest roi_;
  rclcpp::Time roi_stamp_;
  bool has_roi_{false};

  uint8_t  mode_{kModeIdle};
  uint64_t frame_seq_{0};

  /// 最近一帧深度图的**年龄** [s] = `now − 深度图 header.stamp`。
  ///
  /// ⚠️ 2026-09-26 加：本节点此前**只测"处理耗时"（`proc_ms`），从不测"数据有多旧"**
  ///    —— 与 `lidar_perception_node` 原来那个毛病一模一样。而延迟预算按后者算，
  ///    所以「三路总延迟」里 **D435i 这一路一直是无从计入的**。
  ///   ⚠️ D435i 在**判色（40/80 分）**的关键路径上 ⇒ 它不该是个盲区。
  double last_frame_age_sec_{0.0};
  double   last_confidence_{0.0};   // 最近一次的 confidence（诊断用，1Hz 才读）
  rclcpp::Time last_diag_time_;

  // ⚠️ last_frame_time_ 被 depth_callback 写、被 diag_timer_ 读 → **两侧都要加锁**。
  //    单独一把锁：与 diag_mutex_ 从不嵌套, 没有死锁风险。
  rclcpp::Time last_frame_time_;
  std::mutex   frame_mutex_;

  std::mutex diag_mutex_;
};

}  // namespace camera
}  // namespace br_perception
