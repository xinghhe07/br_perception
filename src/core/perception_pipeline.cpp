/// ═══════════════════════════════════════════════════════════════════════════════
/// perception_pipeline.cpp — 感知编排与降级薄壳（任务书 §6.1 模块 27）
///
/// **设计理由、字段策略、坐标系硬约束、发布契约全部写在头文件里** ——
/// 本文件只实现，不重复论证。动手改之前先读 `perception_pipeline.hpp` 的类文档，
/// 特别是这两节：
///   · "⚠️⚠️ 本节点**填不满** `PerceptionFrame`，而且**不许假装填满**"
///   · "⚠️⚠️ 坐标系：**算不出 world 就绝不发坐标** —— 但绝不等于'静默'"
///
/// ⚠️ 本文件**一行都没编译过**（本机无 ROS；见 doc/模块进度.md 顶部那条更正）。
///    按 doc/待编译验证清单_20260920.md 的做法做 grep 自检。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/core/perception_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace br_perception {
namespace core {

namespace {

/// @brief 消息**没填** header.stamp 的判据（ROS 时间 0 点 = 未初始化）
inline bool stamp_is_unset(const rclcpp::Time& t)
{
  return t.nanoseconds() == 0;
}

/// @brief `steady_clock::time_point` 的"从未设置"哨兵判定
///        （默认构造的 time_point 其 epoch 计数为 0）
inline bool tp_unset(const std::chrono::steady_clock::time_point& tp)
{
  return tp.time_since_epoch().count() == 0;
}

/// @brief 单调时钟的"距上次已过 ws 秒"节流（**不用 ROS 时间** —— 仿真里会暂停/跳变）
inline bool throttled(std::chrono::steady_clock::time_point& last, double ws)
{
  const auto now_tp = std::chrono::steady_clock::now();
  if (!tp_unset(last) &&
      std::chrono::duration<double>(now_tp - last).count() < ws) {
    return true;
  }
  last = now_tp;
  return false;
}

/// @brief 降级模式 → 诊断等级（**唯一口径**）
///
/// ⚠️⚠️ 抽成一个函数是**故意的**（2026-09-22 用户复核 P1-4）：同一个模式原先在
///    "帧内诊断"与"降级诊断"两处**各写了一套三元表达式**，而其中一套退化成
///    "只有 EMERGENCY 是 ERROR、其余全 OK" —— 同一个 LIDAR_ONLY 在两边等级不同，
///    现场对着两个面板看会以为其中一个是错的。
///    以后**只改这里**，两处自动一致。
[[nodiscard]] uint8_t diag_level_of(uint8_t mode) noexcept
{
  using DS = diagnostic_msgs::msg::DiagnosticStatus;
  // 全断 / 出不了坐标 → ERROR（后者意味着主控**一帧都收不到**）
  if (mode == PerceptionPipeline::kEmergency || mode == PerceptionPipeline::kNoPose) {
    return DS::ERROR;
  }
  // 正常 / 启动初期 → OK。⚠️ STARTING **不是**故障：话题还没来、Point-LIO 还没起
  //   都是常态，报黄灯会让"习惯性无视"提前发生（同 NEVER 的处理，见头文件）
  if (mode == PerceptionPipeline::kNormal || mode == PerceptionPipeline::kStarting) {
    return DS::OK;
  }
  // 少了一路环境传感器 → WARN
  return DS::WARN;
}

/// @brief 降级模式的**给人读的说明** —— 与 `PerceptionPipeline::mode_name()` 不同：
///        这里会把"**这个模式当前有没有输出**"一并说清
///
/// ⚠️⚠️ 为什么不能只印 `mode_name`（2026-09-23 复核指出）：
///    `kVisionOnly` 听起来像"降级后仍可用"，而**实际是一帧都不发**（雷达停 = 没有
///    3D 内容，相机是 2D、要融合层才有用，而融合层没写）。
///    只印 `VISION_ONLY` ⇒ **诊断在撒谎**，而主控侧看到的是"完全收不到" ——
///    现场会往"链路断了"去查，查错方向。
///    ⚠️ **两处诊断（帧内 + 降级）共用本函数**，别各写一份（同 `diag_level_of` 的教训）。
[[nodiscard]] std::string mode_message(uint8_t mode)
{
  std::string s = PerceptionPipeline::mode_name(mode);
  if (mode == PerceptionPipeline::kVisionOnly) {
    s += " —— ⚠️ 本模式当前**不发帧**"
         "（雷达停 = 没有 3D 内容；相机是 2D，要融合层(模块21,未写)才有用）。"
         "主控侧看到的是「完全收不到」，别去查链路";
    // ⚠️⚠️ 2026-09-24 补一条**这条链自己造成的**后果（不是实现缺陷，是设计事实）：
    s += "。⚠️ **注意：天空方块判色链也停了** —— `sky_block_detector` 的投影 ROI"
         "**由雷达触发**（只有雷达报 `state == 完整塔` 的位才会被投影）⇒ "
         "**雷达停 = 相机不知道该看哪几个位** ⇒ 那 40/80 分在降级模式下**拿不到**。"
         "这不是 bug，是「相机不做检测、只做判色」这条切分的必然代价；"
         "要改就得让相机自己找方块（YOLO 那一路，本版未实现）";
  }
  return s;
}

// ═══════════════════════════════════════════════════════════════════════════
// `CameraSkyBlock.msg` 的取值（⚠️ **与消息头注释逐值一致，不可重排**）
// ═══════════════════════════════════════════════════════════════════════════
// 本文件只读这几个字段，所以就地定义 —— 与 `lidar_perception_node.cpp` 里
// `kMsgSpotEmpty` 那组是同一个做法（见那边的注释：定常量比散落魔数好查）。
constexpr uint8_t kSkyRoiRadarProjected = 0;   ///< roi_source: 雷达建筑位投影（主路）
constexpr uint8_t kSkyStateConfirmed    = 1;   ///< state: 已确认（连续 N 帧满足）
constexpr uint8_t kSkyColorNone         = 0;   ///< top_color: 没有方块
constexpr uint8_t kSkyColorUnknown      = 3;   ///< top_color: 有方块但判不出色

/// `BuildingSpot.msg` 的 `source`（⚠️ 同样逐值一致）
constexpr uint8_t kMsgSourceFusion      = 2;   ///< 融合（状态来自雷达 + 颜色来自相机）

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// 名字表
// ═══════════════════════════════════════════════════════════════════════════════
const char* PerceptionPipeline::source_name(size_t src)
{
  switch (src) {
    case kSrcLidar:  return "lidar";
    case kSrcCamera: return "camera";
    case kSrcFusion: return "fusion";
    case kSrcOdom:   return "odometry";
    case kSrcComm:   return "comm";
    default:         return "?";
  }
}

const char* PerceptionPipeline::state_name(uint8_t st)
{
  switch (st) {
    case kNever:  return "NEVER";    // ⚠️ 是"未接入", **不是故障**（见头文件）
    case kOk:     return "OK";
    case kStale:  return "STALE";
    case kFailed: return "FAILED";
    default:      return "?";
  }
}

const char* PerceptionPipeline::mode_name(uint8_t mode)
{
  switch (mode) {
    case kStarting:   return "STARTING";
    case kNormal:     return "NORMAL";
    case kLidarOnly:  return "LIDAR_ONLY";
    case kVisionOnly: return "VISION_ONLY";
    case kEmergency:  return "EMERGENCY";
    case kNoPose:     return "NO_POSE";
    default:          return "?";
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════════════════
PerceptionPipeline::PerceptionPipeline(const rclcpp::NodeOptions& options)
  : Node("perception_pipeline", options)
{
  // ── 话题 ──
  this->declare_parameter<std::string>("lidar_topic",       lidar_topic_);
  this->declare_parameter<std::string>("camera_topic",      camera_topic_);
  this->declare_parameter<std::string>("comm_diag_topic",   comm_diag_topic_);
  // ── 天空方块判色（模块 9）──
  this->declare_parameter<std::string>("sky_block_wide_topic", sky_block_wide_topic_);
  this->declare_parameter<std::string>("sky_block_tele_topic", sky_block_tele_topic_);
  this->declare_parameter<double>("sky_block_timeout_sec", sky_block_timeout_sec_);
  this->declare_parameter<std::string>("fusion_state_topic", fusion_state_topic_);
  this->declare_parameter<std::string>("output_topic",      output_topic_);
  this->declare_parameter<std::string>("emergency_stop_topic", emergency_stop_topic_);
  this->declare_parameter<std::string>("degradation_topic", degradation_topic_);

  // ── 频率 ──
  this->declare_parameter<double>("publish_rate_hz",    publish_rate_hz_);
  this->declare_parameter<double>("degradation_rate_hz", degradation_rate_hz_);

  // ── 各路超时 ──
  this->declare_parameter<double>("lidar_timeout_sec",  lidar_timeout_sec_);
  this->declare_parameter<double>("camera_timeout_sec", camera_timeout_sec_);
  this->declare_parameter<double>("fusion_timeout_sec", fusion_timeout_sec_);
  this->declare_parameter<double>("odom_timeout_sec",   odom_timeout_sec_);
  this->declare_parameter<double>("comm_timeout_sec",   comm_timeout_sec_);

  // ── 比赛计时 / 告警 / 事件补帧 ──
  this->declare_parameter<double>("match_duration_sec", match_duration_sec_);
  this->declare_parameter<double>("assembly_warn_ms",   assembly_warn_ms_);
  this->declare_parameter<double>("emergency_repeat_sec", emergency_repeat_sec_);
  this->declare_parameter<bool>("event_trigger_enabled",  event_trigger_enabled_);
  this->declare_parameter<double>("event_publish_min_interval_sec",
                                  event_publish_min_interval_sec_);
  this->declare_parameter<double>("event_obstacle_near_m", event_obstacle_near_m_);
  this->declare_parameter<double>("latency_warn_ms",  latency_thresh_.warn_ms);
  this->declare_parameter<double>("latency_error_ms", latency_thresh_.error_ms);

  this->get_parameter("lidar_topic",         lidar_topic_);
  this->get_parameter("camera_topic",        camera_topic_);
  this->get_parameter("comm_diag_topic",     comm_diag_topic_);
  this->get_parameter("sky_block_wide_topic", sky_block_wide_topic_);
  this->get_parameter("sky_block_tele_topic", sky_block_tele_topic_);
  this->get_parameter("sky_block_timeout_sec", sky_block_timeout_sec_);
  this->get_parameter("fusion_state_topic",  fusion_state_topic_);
  this->get_parameter("output_topic",        output_topic_);
  this->get_parameter("emergency_stop_topic", emergency_stop_topic_);
  this->get_parameter("degradation_topic",   degradation_topic_);
  this->get_parameter("publish_rate_hz",     publish_rate_hz_);
  this->get_parameter("degradation_rate_hz", degradation_rate_hz_);
  this->get_parameter("lidar_timeout_sec",   lidar_timeout_sec_);
  this->get_parameter("camera_timeout_sec",  camera_timeout_sec_);
  this->get_parameter("fusion_timeout_sec",  fusion_timeout_sec_);
  this->get_parameter("odom_timeout_sec",    odom_timeout_sec_);
  this->get_parameter("comm_timeout_sec",    comm_timeout_sec_);
  this->get_parameter("match_duration_sec",  match_duration_sec_);
  this->get_parameter("assembly_warn_ms",    assembly_warn_ms_);
  this->get_parameter("emergency_repeat_sec", emergency_repeat_sec_);
  this->get_parameter("event_trigger_enabled", event_trigger_enabled_);
  this->get_parameter("event_publish_min_interval_sec", event_publish_min_interval_sec_);
  this->get_parameter("event_obstacle_near_m", event_obstacle_near_m_);
  this->get_parameter("latency_warn_ms",  latency_thresh_.warn_ms);
  this->get_parameter("latency_error_ms", latency_thresh_.error_ms);

  // ── 规范化 ──
  // ⚠️ 上限 50Hz：这是**发给下位机的协议帧**，再高只是在重复发送同一批没变的状态
  //    （见头文件"感知帧不参与底层控制闭环"那条契约）。
  publish_rate_hz_    = std::max(1.0, std::min(publish_rate_hz_, 50.0));
  degradation_rate_hz_= std::max(0.1, std::min(degradation_rate_hz_, 10.0));
  // 各路超时下限 0.05s：填 0 会让该路**一启动就是 STALE**（永远不 OK）
  lidar_timeout_sec_  = std::max(0.05, lidar_timeout_sec_);
  camera_timeout_sec_ = std::max(0.05, camera_timeout_sec_);
  fusion_timeout_sec_ = std::max(0.05, fusion_timeout_sec_);
  odom_timeout_sec_   = std::max(0.05, odom_timeout_sec_);
  comm_timeout_sec_   = std::max(0.05, comm_timeout_sec_);
  match_duration_sec_ = std::max(1.0, match_duration_sec_);
  assembly_warn_ms_   = std::max(0.0, assembly_warn_ms_);
  emergency_repeat_sec_ = std::max(0.0, emergency_repeat_sec_);
  // ⚠️ 事件补帧的最小间隔下限 0.02s ⇒ **最坏 50Hz**。
  //    填 0 会让"每个输入回调都补发一帧"，等于把帧率悄悄提到传感器频率，
  //    **正好绕过 10Hz 那个职责边界**（见头文件发布契约 ②）。
  event_publish_min_interval_sec_ = std::max(0.02, event_publish_min_interval_sec_);
  event_obstacle_near_m_ = std::max(0.0, event_obstacle_near_m_);

  // ── 统计（复用 timer_utils，不重复造）──
  idx_end_to_end_ = profiler_.declare("end_to_end");

  // ── 订阅 ──
  // QoS 的**兼容方向是单向的**：RELIABLE 订阅 ← BEST_EFFORT 发布 = 不兼容（静默收不到）；
  // 反过来兼容。所以订阅端一律取**发布端实际用的那档**，别凭"想更可靠"往上加。
  //
  // 雷达结果：发布端是 `create_publisher<LidarResult>(topic, 10)` = RELIABLE depth 10
  sub_lidar_ = this->create_subscription<br_perception::msg::LidarResult>(
    lidar_topic_, rclcpp::QoS(10),
    std::bind(&PerceptionPipeline::on_lidar_result, this, std::placeholders::_1));

  // 相机聚合帧：发布端是 `rclcpp::SensorDataQoS()` = BEST_EFFORT depth 5
  sub_camera_ = this->create_subscription<br_perception::msg::CameraFrame>(
    camera_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PerceptionPipeline::on_camera_frame, this, std::placeholders::_1));

  // 通信诊断：发布端默认 RELIABLE depth 10
  // ── 天空方块判色（模块 9）——
  //  ⚠️⚠️ 为什么本节点要订它：**判色是相机独有的能力，雷达对颜色无能为力**
  //     （`building_spot_analyzer` 原文："完整塔: 雷达无法判断天空方块朝上颜色"）。
  //     而规则 3.5.6/3.5.7 里那 40/80 分的归属**只由顶面颜色决定** ——
  //     不接这一路，主控拿到的 `BuildingSpot.top_color` 就永远是 `UNKNOWN(3)`。
  //  ⚠️ QoS 必须与发布端一致：`sky_block_detector` 用的是
  //     `rclcpp::QoS(1).best_effort()`（BEST_EFFORT 订阅接 BEST_EFFORT 发布）。
  //  ⚠️⚠️ **这两路必须用 lambda，不能用 `std::bind` 绑第二个实参**。
  //     `create_subscription` 走的是 `AnySubscriptionCallback::set()`，它靠
  //     **回调的实参个数**去匹配那几种合法签名（`(msg)` 或 `(msg, MessageInfo&)`）。
  //     而 `std::bind(&on_sky_block, this, _1, false)` 产生的可调用体原型是
  //     `void(SharedPtr<CameraSkyBlock>, bool)` —— **两参且第二参不是 MessageInfo**，
  //     一种都对不上 ⇒ 编译期死在 `any_subscription_callback.hpp` 里那句
  //     `no match for operator=`，**报错指向 rclcpp 内部、看不出是本行引起的**
  //     （2026-09-25 首次编到本文件时栽在这）。
  //     lambda 把它收成 `(msg)` 一参，`is_tele` 作为**捕获**而不是实参进闭包。
  sub_sky_wide_ = this->create_subscription<br_perception::msg::CameraSkyBlock>(
      sky_block_wide_topic_, rclcpp::QoS(1).best_effort(),
      [this](br_perception::msg::CameraSkyBlock::SharedPtr msg) {
        on_sky_block(msg, /*is_tele=*/false);
      });
  sub_sky_tele_ = this->create_subscription<br_perception::msg::CameraSkyBlock>(
      sky_block_tele_topic_, rclcpp::QoS(1).best_effort(),
      [this](br_perception::msg::CameraSkyBlock::SharedPtr msg) {
        on_sky_block(msg, /*is_tele=*/true);
      });

  sub_comm_diag_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    comm_diag_topic_, rclcpp::QoS(10),
    std::bind(&PerceptionPipeline::on_comm_diag, this, std::placeholders::_1));

  // 融合层的穆斯蒂卡状态（发布端 = mustika_pillar_check，模块 20）。
  // ⚠️ 现在还没有发布者 → 这一路恒不触发 → `mustika_state` 仍是 0=未知，
  //    **与补这条之前的行为完全一致**（补的是链路，不是行为）。
  // ⚠️ 发布端用默认 QoS(RELIABLE depth 10) 还是 best_effort？—— 见那个节点的实现。
  //    这里取 `rclcpp::QoS(10)`（RELIABLE）是**安全侧**：它兼容 RELIABLE 与
  //    BEST_EFFORT 两种发布者，而 RELIABLE 订阅遇上 BEST_EFFORT 发布才是那个静默坑。
  sub_fusion_state_ = this->create_subscription<std_msgs::msg::UInt8>(
    fusion_state_topic_, rclcpp::QoS(10),
    std::bind(&PerceptionPipeline::on_fusion_state, this, std::placeholders::_1));

  // ── 发布 ──
  // ⚠️ 输出帧用 **RELIABLE**（`rclcpp::QoS(10)`）：它是**控制相关**的包内接口，
  //    丢一帧会让主控少一次状态更新。且 RELIABLE 发布对 BEST_EFFORT 订阅者
  //    **同样兼容**（`comm_params.yaml` 里 comm_node 就是按 BEST_EFFORT 订的），
  //    所以这是两侧都吃得下的选择 —— 反过来改成 BEST_EFFORT 才会踢掉 RELIABLE 订阅者。
  pub_frame_ = this->create_publisher<br_perception::msg::PerceptionFrame>(
    output_topic_, rclcpp::QoS(10));

  // ⚠️ 紧急停止用 RELIABLE：它**必须到**。丢一条 = 主控少一次停车机会。
  pub_emergency_ = this->create_publisher<std_msgs::msg::Bool>(
    emergency_stop_topic_, rclcpp::QoS(10));

  pub_degradation_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    degradation_topic_, 10);

  // ⚠️ **本节点加载 `config/fusion_params.yaml` 的坐标系/里程计键**（同一个事实源）——
  //    因为本节点要借 `CoordinateTransformer` 做 TF，而 frame 名必须和融合层一致，
  //    否则两边各写一份迟早漂移。**launch 里要把 fusion_params.yaml 也传给本节点**。
  //    ⚠️ 必须放在**本节点自己那批 declare_parameter 之后**：它会 declare 一批
  //       frame/odometry 键，顺序反了会先声明后覆盖（值仍是默认的）。
  tf_ = std::make_unique<br_perception::fusion::CoordinateTransformer>(
      this, br_perception::fusion::TransformerParams::load_from_node(this));

  // ── 心跳定时器 ──
  // ⚠️ 周期取 `1/publish_rate_hz`。**事件补帧不走这里** —— 它由输入回调直接触发
  //    （变化是在回调那一刻观测到的，等下一个 tick 就又是 ≤100ms 延迟，等于没做）。
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&PerceptionPipeline::tick, this));

  last_degradation_pub_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(),
    "PerceptionPipeline ready | in: lidar=%s camera=%s comm_diag=%s "
    "sky_blocks(wide/tele)=%s/%s | out: %s | "
    "publish=%.1fHz | timeouts(lidar/cam/fusion/odom/comm)=%.2f/%.2f/%.2f/%.2f/%.2fs",
    lidar_topic_.c_str(), camera_topic_.c_str(), comm_diag_topic_.c_str(),
    sky_block_wide_topic_.c_str(), sky_block_tele_topic_.c_str(),
    output_topic_.c_str(), publish_rate_hz_,
    lidar_timeout_sec_, camera_timeout_sec_, fusion_timeout_sec_,
    odom_timeout_sec_, comm_timeout_sec_);

  // ⚠️ 把"融合层还没接"这件事**在最显眼的地方说一次**：它不是 bug，
  //    但现场看到 `fusion_bypass` 恒为 true 时会怀疑是不是坏了。
  RCLCPP_INFO(this->get_logger(),
    "融合层（模块 21）尚未实现 → 本节点工作在「透传雷达结果」模式（任务书 §6.1 的"
    "融合异常降级），这是预期行为，不是故障");
}

// ═══════════════════════════════════════════════════════════════════════════════
// 回调
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::on_lidar_result(
    const br_perception::msg::LidarResult::SharedPtr msg)
{
  // `watchdog_ok` 是**该路自报的健康位**：false = 雷达流水线自己说它断了
  // （见 lidar_perception_node 里 watchdog_ok 的填法）→ 记 kFailed 而不是 kOk。
  const bool healthy = msg->watchdog_ok;
  mark_seen(kSrcLidar, healthy);

  last_lidar_ = msg;

  // 比赛计时：**首次收到环境传感器数据**时起算（近似，见头文件发布契约 ④）
  if (!match_started_) {
    match_started_ = true;
    match_start_   = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(),
      "首次收到环境传感器数据 → 比赛自计时启动（%.0fs）。"
      "⚠️ 这只是近似：真正的开始由裁判蜂鸣器决定，感知无从得知",
      match_duration_sec_);
  }

  // ⚠️ 事件补帧**从回调直接触发**（不等下一个 tick）—— 见头文件发布契约 ②
  try_publish(/*is_event=*/true);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 天空方块判色（模块 9）—— 只做"记下来"
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::on_sky_block(
    const br_perception::msg::CameraSkyBlock::SharedPtr msg, bool is_tele)
{
  if (!msg) return;

  // ⚠️ 判色不是"相机链活没活"的证据 —— 它只在**雷达说有完整塔**时才发消息。
  //    所以这里**不调 `mark_seen(kSrcCamera, ...)`**：拿它当心跳会让
  //    "相机挂了但雷达也没塔"这段时间看起来像相机活着（那正是最坏的情况）。
  // ⚠️⚠️ **只收投影主路的结果**（`roi_source == 0`）。
  //    手动 ROI（台架调阈值用）与将来的 YOLO 通道的 `spot_id` 契约是 **-1** ——
  //    若让它们进表，`-1` 会被 clamp 成 0 而**把结论错挂到 0 号建筑位上**。
  //    这类"负数穿进无符号"的错在本仓库已经咬过（见 `BuildingSpot.msg` 的 state 注释）。
  if (msg->roi_source != kSkyRoiRadarProjected) return;
  if (msg->spot_id < 0) return;

  // ⚠️ 每个位只留**最近一条**（按收到顺序覆盖）。不做排序/累加：
  //    时序滞回是相机节点自己的活（它按位各有一套确认器）。
  auto& store = is_tele ? sky_tele_ : sky_wide_;
  store[static_cast<uint32_t>(msg->spot_id)] = *msg;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 相机聚合帧（模块 10 的输出）—— 本节点只**借它判"相机链活没活"**
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::on_camera_frame(
    const br_perception::msg::CameraFrame::SharedPtr msg)
{
  if (!msg) return;

  // ⚠️⚠️ 本回调**不能**写成 `mark_seen(kSrcCamera, true)`。理由：
  //
  //    `camera_perception_node` 是**聚合薄壳** —— 它的职责是"订阅已有话题、按槽汇总"，
  //    所以**一个检测器都没起时它照样按自己的频率发帧**，只是每槽 `state == S_NEVER`。
  //    拿"聚合帧到了"当相机链的活证据，等于让一个空壳自证：
  //    诊断报 `camera OK`，而主控拿到的 `building_spots.top_color` 恒是 UNKNOWN ——
  //    正是本文件反复强调的"不许让诊断撒谎"。
  //    `mark_seen` 那个 `healthy` 参数存在的全部意义就在这里（`kFailed` 的定义是
  //    "**有数据但自报不健康**"，见 `publish_degradation_state` 的文案）。
  //
  //    ⇒ 健康判据取 **`slot_status`**：`msg/SlotStatus.msg` 自己写明
  //      "机器可读，下游按这个分支，**不要按 `stale_sec` 猜**"。
  //      ⚠️ 也不用 `frame_age_sec`：那是**把 8 个槽塌缩成一个标量**，而槽状态
  //         已经按 `camera_perception_node` 的 `fresh_tolerance_sec_` 逐槽判好了 ——
  //         再拿一个自己的阈值去重判同一个东西，只会多出一处会漂移的判据。
  //
  //  三档 → `kOk` / `kFailed` / 不置位，逐档理由：
  //    · 有槽 FRESH           → 相机链确实在产出            → `kOk`
  //    · 无 FRESH 但有 STALE   → 跑过、现在慢了或停了        → `kFailed`
  //    · 全部 NEVER           → **整个回调不 `mark_seen`**（见下）
  //
  //  ⚠️⚠️ 第三档**刻意不 mark_seen**，而不是 `mark_seen(..., false)`：
  //     `kNever` 只在 `last_seen` 未设置时出现，它的语义是"**从没见过** →
  //     配置问题（还没起 / 话题名错），**不是故障**"（头文件"降级状态机"那节，
  //     以及 `refresh_states()` 里那句"诊断里也不许报 WARN"）。
  //     若这里一律 mark_seen，检测器还没起的那段时间就会报 `kFailed`
  //     ⇒ 诊断从启动起就挂黄，**而 `decide_mode()` 也会由 `kStarting` 提前跳到
  //     `kLidarOnly`** —— 把一个"还没起"说成"坏了"。现场这两件事要做的事完全不同。
  bool any_fresh = false;
  bool any_stale = false;
  for (const auto& s : msg->slot_status) {
    if (s.state == br_perception::msg::SlotStatus::S_FRESH) {
      any_fresh = true;
      break;                       // FRESH 优先：找到了就不必再扫 STALE
    }
    if (s.state == br_perception::msg::SlotStatus::S_STALE) any_stale = true;
  }

  if (any_fresh)      mark_seen(kSrcCamera, /*healthy=*/true);
  else if (any_stale) mark_seen(kSrcCamera, /*healthy=*/false);
  // else: 全 NEVER → 不 mark_seen（留 `kNever`，理由见上）

  // ⚠️⚠️ 原来这里有一行 `last_camera_stamp_ = rclcpp::Time(...)`，**2026-09-25 删**：
  //    那个成员**只写不读** —— 每帧算一次 `rclcpp::Time` 然后丢掉（**死写**）。
  //    健康判定**本来就该用槽状态**（上面那段 ⚠️⚠️ 的理由），时间戳不参与 ⇒
  //    这行没有存在意义。见头文件里那条说明。

  // ⚠️ 本回调**不调 `try_publish()`**（与 `on_lidar_result` 不同），**这是有意的**：
  //    `make_signature()` 的入参只有 `LidarResult`；其中唯一来自相机的量是它内部
  //    `camera_color_for()` 查的那两张表（`sky_wide_` / `sky_tele_`，由
  //    `on_sky_block` 填）—— **`CameraFrame` 本身不参与签名**。
  //    ⇒ 在这里补帧**一帧都不会真的发出去**，却会在相机 30Hz 下白白重算 30 次签名。
  //    （签名里那个相机颜色由**下一次雷达回调**带出去，10Hz ⇒ ≤100ms —— 见
  //      `make_signature` ④。所以"塔被翻了"这条事件仍然进得来，只是入口在雷达侧。）
}

// ═══════════════════════════════════════════════════════════════════════════════
// 通信诊断（comm_node 的输出）—— 本节点只**观测**，不自理（见头文件降级那节）
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::on_comm_diag(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  if (!msg) return;

  // 契约：`/communication/diagnostics` 里有一条状态带 `connected` 键
  //（发布端 `comm_node::publish_diagnostics` 的 `kv("connected", connected ? "1" : "0")`）。
  //
  // ⚠️ 按 **键** 找，**不是**按 `status.name`（"communication/serial"）找：
  //    头文件 `Source::kSrcComm` 那条写的判据就是"`connected` 键"；
  //    而 `name` 是 comm_node 的内部命名，它改名不该让本节点**静默失聪**
  //    ——"看着配好了，实际没接上"正是 §零 第一宗旨要防的。
  //
  // ⚠️ 找不到 `connected` 键 ⇒ **本次不 `mark_seen`**（同相机那条的 NEVER 处理）：
  //    宁可留 `kNever`（"未接入，不是故障"），也不替上游断言一个它没说的结论。
  for (const auto& st : msg->status) {
    for (const auto& kvp : st.values) {
      if (kvp.key != "connected") continue;
      // ⚠️ **只认 `"1"` 是健康**：`"0"` 以及任何别的取值都按不健康算。
      //    上游将来若加档（如 `"2"`=重连中），默认落到**保守侧**而不是乐观侧 ——
      //    链路的事，把"不知道"当"通"的代价是主控以为下位机在听。
      mark_seen(kSrcComm, /*healthy=*/kvp.value == "1");
      return;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 融合层的穆斯蒂卡状态（模块 20）
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::on_fusion_state(const std_msgs::msg::UInt8::SharedPtr msg)
{
  if (!msg) return;

  // ⚠️⚠️ **`mark_seen` 必须排在下面那道定义域过滤之前**（2026-09-23 复核登记过这条缺陷）。
  //    本回调要回答两个**不同**的问题：
  //      ① "融合层这根链现在活没活" → 看 `health_[kSrcFusion]`，
  //         判据是"**有没有消息来**"，与消息里的值合不合法**无关**；
  //      ② "它给的穆斯蒂卡状态是什么" → 看 `last_fusion_state_`，这档才要过滤。
  //    合并成一处（值不认识就整个 return、连 `mark_seen` 都不做）会让诊断自相矛盾：
  //    同一张表里既写 "fusion NEVER（未接入）"、又写 "mustika_state 来自融合层"——
  //    现场会据此**不信任一个真结论**。
  mark_seen(kSrcFusion, /*healthy=*/true);

  // ── 定义域过滤：**只认 0 / 1 / 5** ──
  //  头文件契约：`mustika_pillar_check` 只会发这三档
  //  （见 `msg/PerceptionFrame.msg` 的取值表：0=未知 1=柱上 5=掉落）。
  //  其余值（2=TR转运中 / 3=BR持有 / 4=已供奉 / 6=重试归还中）**它无从判断** ——
  //  那些要看到 TR/BR 的动作。真收到就说明上游**超出了它自己的定义域**，
  //  而 `PerceptionFrame.mustika_state` 是主控侧 decode 的依据 ⇒
  //  **丢弃并降级为 0**，绝不把不认识的值透传过去（透传 = 替上游断言一个它没说的事实）。
  constexpr uint8_t kUnknown  = 0;   // 未知 —— "这条信息不可用"，不是"球不在"
  constexpr uint8_t kOnPillar = 1;   // 在柱上
  constexpr uint8_t kFallen   = 5;   // 球掉在地上
  switch (msg->data) {
    case kUnknown:
    case kOnPillar:
    case kFallen:
      last_fusion_state_ = msg->data;
      break;
    default:
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "融合层给的 mustika_state=%u 超出定义域(0/1/5) → 按未知(0)处理",
        static_cast<unsigned>(msg->data));
      last_fusion_state_ = kUnknown;
      break;
  }

  // 单调时钟（**不是** ROS 时间）：仿真里 ROS 时间会暂停/跳变，
  // 用它算"多久没来"会得到负数或天文数字 —— 见头文件"时钟"那节。
  fusion_state_at_ = std::chrono::steady_clock::now();

  // ⚠️ 本回调也**不调 `try_publish()`**：理由同相机那条 ——
  //    `mustika_state` 不在 `make_signature()` 读的字段里（它是签名里的
  //    `mustika_pillar_occupied`，来自**雷达**，不是这个融合状态）。
}

// ═══════════════════════════════════════════════════════════════════════════════
// 取某个建筑位的"相机确认色"
// ═══════════════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════════════
// 【纯决策】两路相机 → 用哪个颜色（**可单测**：只吃数据，不碰 ROS）
// ═══════════════════════════════════════════════════════════════════════════════
bool PerceptionPipeline::choose_sky_color(const SkyColorInput& wide,
                                         const SkyColorInput& tele,
                                         double timeout_sec,
                                         uint8_t& color_out,
                                         std::string& why)
{
  // 一路"可不可用"的判据（①②）—— 两路共用同一套，别写成两份
  const auto usable = [&](const SkyColorInput& in, const char* name, uint8_t& col) -> bool {
    if (!in.present) return false;
    if (in.state != kSkyStateConfirmed) return false;      // ② COASTING 不许翻状态
    if (in.color == kSkyColorNone || in.color == kSkyColorUnknown) return false;
    if (!in.age_known) return false;                       // 没时间戳 ⇒ 判不了新鲜度 ⇒ 不采信
    if (in.age_sec > timeout_sec) return false;            // ① 太旧
    col = in.color;
    (void)name;
    return true;
  };

  uint8_t wide_col = 0, tele_col = 0;
  const bool has_wide = usable(wide, "wide", wide_col);
  const bool has_tele = usable(tele, "tele", tele_col);

  if (has_tele && has_wide && tele_col != wide_col) {
    // ⚠️⚠️ **两路打架 ⇒ 不下结论**。宁可让主控看到"颜色未知"，
    //    也不拿一个可能就是错色去覆盖 —— 错色的代价是**把塔的归属判反**。
    why = "宽角/长焦结论不一致（" + std::to_string(wide_col) + " vs " +
          std::to_string(tele_col) + "）—— 保守报未知";
    return false;
  }
  if (has_tele) { color_out = tele_col; why = "长焦 CONFIRMED"; return true; }
  if (has_wide) { color_out = wide_col; why = "宽角 CONFIRMED"; return true; }
  why = "无 CONFIRMED 的相机颜色（或已过期）";
  return false;
}

bool PerceptionPipeline::camera_color_for(uint32_t spot_id, uint8_t& color_out,
                                          std::string& why) const
{
  // 从"每位最近一条"的表里取出这条位的两路消息，转成**纯数据**再交给纯决策函数。
  // ⚠️ 新鲜度在这里算（要用节点的时钟 —— 那是 ROS 的东西，所以留在这一层）。
  const auto pack = [&](const std::unordered_map<uint32_t,
                                                br_perception::msg::CameraSkyBlock>& m) {
    SkyColorInput in;
    auto it = m.find(spot_id);
    if (it == m.end()) return in;
    in.present = true;
    in.color   = it->second.top_color;
    in.state   = it->second.state;
    const rclcpp::Time t(it->second.header.stamp, this->get_clock()->get_clock_type());
    if (!stamp_is_unset(t)) {
      // ⚠️ 用 `this->now()`（`Node::now() const`），**不是** `this->get_clock()->now()`：
      //    本函数是 **const 成员**，const 重载的 `get_clock()` 返回
      //    `Clock::ConstSharedPtr`，而 `rclcpp::Clock::now()` 是**非 const** 的
      //    （ROS_TIME 下它要更新内部状态）⇒ `discards qualifiers` 编译错。
      //    `Node::now() const` 内部对 `clock_`（成员 shared_ptr）解引用调用，
      //    constness 不穿透指针 ⇒ 恰好绕开这点，语义与上面 `get_clock_type()` 同一口钟。
      in.age_known = true;
      in.age_sec   = (this->now() - t).seconds();
    }
    return in;
  };

  return choose_sky_color(pack(sky_wide_), pack(sky_tele_),
                          sky_block_timeout_sec_, color_out, why);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 健康判定
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::mark_seen(size_t src, bool healthy)
{
  if (src >= kSrcCount) return;
  auto& h = health_[src];
  h.last_seen = std::chrono::steady_clock::now();
  h.healthy   = healthy;
  // 只有"健康地见过"才算 ever_ok —— 一路从来只报不健康，不构成"曾经能用"
  if (healthy) h.ever_ok = true;
  h.state     = healthy ? kOk : kFailed;
}

void PerceptionPipeline::refresh_states()
{
  const auto now_tp = std::chrono::steady_clock::now();
  const std::array<double, kSrcCount> timeouts = {
      lidar_timeout_sec_, camera_timeout_sec_, fusion_timeout_sec_,
      odom_timeout_sec_, comm_timeout_sec_};

  // 里程计：**不靠订阅**，直接问 CoordinateTransformer（它是里程计状态的权威）。
  // ⚠️ 它内部用的是 ROS 时间判超时，与本节点其它路的单调时钟**不是一套** ——
  //    这是有意的：里程计的超时语义属于模块 17，本节点只**读结论**，不重算。
  {
    const bool ok = tf_ && tf_->odometry_reliable();
    auto& h = health_[kSrcOdom];
    if (ok) {
      h.last_seen = now_tp;
      h.ever_ok   = true;
    }
    // ⚠️ 不可靠时**不更新时间戳**，让它自然走向 STALE；但也**不直接置 FAILED** ——
    //    "退化/振动"（DEGENERATE / IMU_VIBRATION）与"断了"（TIMEOUT）是两回事，
    //    混成一个档会让现场分不清该查链路还是该查场地。
  }

  for (size_t i = 0; i < kSrcCount; ++i) {
    auto& h = health_[i];
    if (tp_unset(h.last_seen)) {
      h.state = kNever;        // ⚠️ **不是故障**（见头文件）；诊断里也不许报 WARN
      continue;
    }
    // ⚠️ 自报不健康**也要看超时**（2026-09-22 用户复核 P2-11）：
    //    原先只要 `!healthy` 就永远停在 kFailed、文案一直是"有数据但自报不健康"——
    //    可那一路**可能早就一条消息都不来了**，现场会以为数据还在来。
    //    现在：不健康且**还在来** → kFailed；不健康且**已经超时** → kStale
    //    （"最后自报不健康"这件事由诊断文案带上，见 `publish_degradation_state`）。
    const double age = std::chrono::duration<double>(now_tp - h.last_seen).count();
    if (age > timeouts[i]) {
      h.state = kStale;
    } else {
      h.state = h.healthy ? kOk : kFailed;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 降级模式判定（**纯函数** —— 只看 health_，不改任何状态）
// ═══════════════════════════════════════════════════════════════════════════════
uint8_t PerceptionPipeline::decide_mode() const
{
  const auto& lidar  = health_[kSrcLidar];
  const auto& camera = health_[kSrcCamera];

  // "可用" = 状态是 OK。⚠️ NEVER 与 STALE/FAILED **在这里一视同仁**（都不可用），
  // 区别只体现在**要不要报紧急**（下面）与诊断文案上。
  const bool lidar_ok  = (lidar.state  == kOk);
  const bool camera_ok = (camera.state == kOk);

  if (lidar_ok && camera_ok) return kNormal;

  // ⚠️⚠️ **"从未见过"不算故障** —— 节点刚起来、话题还没来时，两路都是 NEVER。
  //     若把它当故障，本节点**一启动就会发紧急停止**（主控当场停车），
  //     而那时多半只是"相机话题名配错了"。所以紧急停止的条件是
  //     "**曾经能用、现在两个环境传感器都没了**"。
  const bool ever_had_any = lidar.ever_ok || camera.ever_ok;
  const bool both_never   = (lidar.state == kNever) && (camera.state == kNever);

  if (both_never) return kStarting;

  if (!lidar_ok && !camera_ok) {
    // 两个都没了。只有"曾经有过"才算紧急；否则是"从没接上"，归 STARTING。
    return ever_had_any ? kEmergency : kStarting;
  }

  // ⚠️ **能不能出 world 坐标**是**另一维**的判断（2026-09-22 用户复核 P0-3）：
  //    它不参与上面那套"哪路传感器没了"，但**对主控同样致命** ——
  //    出不了坐标就**一帧都发不出去**。
  //    放在传感器之后判，是因为传感器全断是**根因**（坐标出不来往往是它的后果）。
  //    ⚠️ 与 STARTING 同一套逻辑：**从没出过** = 启动初期（Point-LIO 还没起来是常态），
  //       不是故障；**出过又没了**才是 kNoPose。
  if (ever_had_pose_ && !pose_available_) return kNoPose;

  // 只剩一路：**哪一路没了**决定模式（任务书 §6.1 的两条降级）
  if (!lidar_ok)  return kVisionOnly;   // 雷达不可用 → 纯视觉（建筑位降级，靠相机）
  return kLidarOnly;                    // 相机不可用 → 纯雷达
}

// ═══════════════════════════════════════════════════════════════════════════════
// 心跳
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::tick()
{
  refresh_states();

  // 紧急状态与诊断**每 tick 都更新**（低频发布在函数内部自己节流），
  // ⚠️ 即使这一帧因为 TF 不可用发不出去，紧急停止也必须照发 —— 见头文件降级那节。
  const uint8_t mode = decide_mode();
  const bool stop = (mode == kEmergency);
  publish_emergency_stop(stop);

  try_publish(/*is_event=*/false);

  // ── 降级诊断（低频）──
  const auto now_tp = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now_tp - last_degradation_pub_).count() >=
      1.0 / degradation_rate_hz_) {
    publish_degradation_state();
    last_degradation_pub_ = now_tp;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 组帧与发布
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::try_publish(bool is_event)
{
  // ⚠️ 回调路径（on_lidar_result / on_camera_frame）也走这里，所以健康状态
  //    **必须在这里刷新** —— 不能假设只有 tick() 会调用本函数。
  refresh_states();

  const auto now_tp = std::chrono::steady_clock::now();

  if (is_event) {
    if (!event_trigger_enabled_) return;

    // ⚠️⚠️ **先判要不要发，再组帧**（2026-09-22 用户复核 P1-6）。
    //    组帧含**逐点 TF**，不便宜；而输入回调是 10Hz(雷达)+30Hz(相机) ——
    //    若每次都白组装一次，每秒要白付约 40 次逐点 TF 查询。
    //    判据只用**原始雷达数据**（`make_signature` 一个 TF 都不做）。
    if (!last_lidar_) return;                       // 还没有可比的基准
    if (last_sig_.valid && !scene_changed(make_signature(*last_lidar_))) return;
    if (!tp_unset(last_event_pub_) &&
        std::chrono::duration<double>(now_tp - last_event_pub_).count() <
            event_publish_min_interval_sec_) {
      return;
    }
  }

  const auto t0 = std::chrono::steady_clock::now();

  // ── 到这里才真的组帧（含 TF）──
  uint8_t reason = kFailNone;
  auto frame = assemble_frame(reason);
  if (!frame) {
    // ⚠️ **按原因分开计数、分开出文案** —— 合并成一个数就只剩猜（P0-1）
    ++fail_counts_[reason];
    warn_frame_failure(reason);
    return;
  }

  if (is_event) {
    last_event_pub_ = now_tp;
    ++event_frames_;
  }

  // ── 组帧耗时（**只算组帧本身**：拷小消息 + TF；不含发布、不含检测）──
  const double assembly_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

  // ── 端到端延迟（**复用 timer_utils**，不自己算 min/max/mean）──
  // 定义：本帧时间戳（上游曝光/采集时刻）到**现在**。含上游处理 + 本节点组帧。
  // ⚠️ 必须在 `publish(std::move(...))` **之前**读 `header.stamp` —— move 之后
  //    那个消息对象就空了，再去读它是未定义行为。
  {
    // ⚠️ **要带 clock type**（同文件 :279 / :584 的写法）。裸构造的 `rclcpp::Time`
    //    clock_type 是默认的，`use_sim_time` 下与 `now()` 相减会**抛异常**：
    //    "can't subtract times with different time sources"。
    const rclcpp::Time stamp(frame->header.stamp, this->get_clock()->get_clock_type());
    const double e2e_ms = (this->get_clock()->now() - stamp).seconds() * 1000.0;
    // ⚠️ `StageProfiler::record` 自己会丢掉 NaN/负值/非法下标（见其头注释），
    //    这里不用再判一遍 —— 重复判反而会在将来它放宽时变成两套标准。
    profiler_.record(idx_end_to_end_, e2e_ms);

    const auto level = br_perception::utils::classify_latency(e2e_ms, latency_thresh_);
    // ⚠️ 节流走本文件自己的 `throttled()`（**单调时钟**），不用
    //    `RCLCPP_WARN_THROTTLE(*get_clock())` —— 见头文件"两套时钟"那节：
    //    仿真里 ROS 时钟会暂停，基于它的节流窗口**永远不会到期**。
    if (level != br_perception::utils::LatencyLevel::OK &&
        !throttled(last_latency_warn_, 5.0)) {
      RCLCPP_WARN(this->get_logger(),
        "端到端延迟 %.1fms 达 %s 级（阈值 warn=%.0f / error=%.0f ms）。"
        "⚠️ 这是「曝光→发布」的整条通路，单看本节点查不出来 —— "
        "先看上游检测耗时与串口侧是否积压",
        e2e_ms, br_perception::utils::to_string(level),
        latency_thresh_.warn_ms, latency_thresh_.error_ms);
    }
  }

  // ⚠️⚠️ **必须更新比对基准，且必须在 move 之前** ——
  //    move 之后 `frame` 已空，从它算签名会读到空/零值；
  //    而基准不更新的话，事件补帧会拿第一帧的签名一直比（每次都触发，
  //    帧率被悄悄提到传感器频率），或反过来再也触发不了。
  last_sig_ = make_signature(*last_lidar_);

  // ⚠️ move 发布，不再整帧拷贝（2026-09-22 用户复核 P1-7）。
  //    ⚠️ 这之后**不许再碰 `frame`** —— 上面那两段都已在它之前完成。
  pub_frame_->publish(std::move(*frame));
  ++frame_seq_;
  ++assembled_frames_;
  fps_.tick();

  if (assembly_ms > assembly_warn_ms_ && !throttled(last_assembly_warn_, 5.0)) {
    RCLCPP_WARN(this->get_logger(),
      "组帧耗时 %.2fms 超过阈值 %.2fms（本节点不等待任何上游 —— "
      "检查消息体是否变大 / TF 是否变慢）", assembly_ms, assembly_warn_ms_);
  }
}

PerceptionPipeline::PublishSignature PerceptionPipeline::make_signature(
    const br_perception::msg::LidarResult& lidar) const
{
  PublishSignature s;
  s.valid = true;

  // ① 障碍物**个数** —— "多/少了一个障碍"本身就是事件，且比逐点比对便宜得多
  s.obstacle_count = lidar.obstacles.size();

  // ② 最近障碍物距离 —— **在传感器系里算，不做 TF**
  //    ⚠️ 这样算与"在 world 系里减去 ego_pose"**结果等价**：机器人就在传感器原点
  //       附近（差一个安装外参，量级 0.1~0.4m），而阈值是 2m 且这里只用于
  //       **判变没变**，那点偏差不影响结论。换来的是**一个 TF 查询都不用**
  //       —— 这正是"先廉价判、再组帧"能成立的原因（P1-6）。
  s.nearest_obstacle_m = -1.0;
  for (const auto& o : lidar.obstacles) {
    const double d = std::hypot(o.position.x, o.position.y);
    if (s.nearest_obstacle_m < 0.0 || d < s.nearest_obstacle_m) {
      s.nearest_obstacle_m = d;
    }
  }

  // ③ 降级模式
  s.mode = decide_mode();

  // ④ 建筑位状态（**排序后比**，避免只因为顺序变化就误判成"变了"）
  //   ⚠️ 第三项是**相机判出的顶面颜色**（不是雷达的 —— 雷达恒填 UNKNOWN）。
  //      带上它是因为**翻转只改颜色、不改高度**（规则 3.5.8）：塔被翻时
  //      `state` 仍是 COMPLETE_TOWER，不把颜色放进签名，这件事**一个事件都不触发**。
  //   ⚠️ 这里**不做 TF**（`make_signature` 的取舍没变）：颜色是查表来的。
  s.spot_states.reserve(lidar.building_spots.size());
  for (const auto& sp : lidar.building_spots) {
    uint8_t cam_color = 0;
    std::string why;
    const uint8_t color = camera_color_for(sp.spot_id, cam_color, why)
                              ? cam_color : sp.top_color;
    s.spot_states.emplace_back(sp.spot_id, sp.state, color);
  }
  std::sort(s.spot_states.begin(), s.spot_states.end());

  // ⑤ 柱顶有无物体 —— 穆斯蒂卡"被取走 / 被放回"的信号
  s.mustika_pillar_occupied = lidar.mustika_pillar.has_object_on_top;
  s.core_pillar_occupied    = lidar.core_pillar.has_object_on_top;

  return s;
}

bool PerceptionPipeline::scene_changed(const PublishSignature& s) const
{
  if (!last_sig_.valid) return true;      // 第一帧永远值得发

  const PublishSignature& prev = last_sig_;

  // ① 降级模式翻转（含 kNormal ↔ kLidarOnly / kVisionOnly / kEmergency / kNoPose）
  if (s.mode != prev.mode) return true;

  // ② 障碍物个数变化
  if (s.obstacle_count != prev.obstacle_count) return true;

  // ③ 有障碍物**进入近距**（默认 2m）—— 这是避障最需要提前知道的
  //    只有"从远处进到近处"或"近处的位置又变了"才触发；单纯远离不触发
  {
    const bool was_near = (prev.nearest_obstacle_m >= 0.0 &&
                           prev.nearest_obstacle_m <= event_obstacle_near_m_);
    const bool is_near  = (s.nearest_obstacle_m >= 0.0 &&
                           s.nearest_obstacle_m <= event_obstacle_near_m_);
    if (is_near && (!was_near ||
                    std::abs(s.nearest_obstacle_m - prev.nearest_obstacle_m) > 0.2)) {
      return true;
    }
  }

  // ④ 建筑位状态翻转（EMPTY ↔ 有塔）—— **比赛里最要紧的事件之一**
  if (s.spot_states != prev.spot_states) return true;

  // ⑤ 柱顶有无物体变化
  if (s.mustika_pillar_occupied != prev.mustika_pillar_occupied ||
      s.core_pillar_occupied    != prev.core_pillar_occupied) {
    return true;
  }

  return false;
}

void PerceptionPipeline::warn_frame_failure(uint8_t reason)
{
  if (reason >= fail_warn_time_.size()) return;
  if (throttled(fail_warn_time_[reason], 5.0)) return;   // 每条原因**独立**节流

  const unsigned long long n =
      static_cast<unsigned long long>(fail_counts_[reason]);

  // ⚠️⚠️ 每条原因的**排查方向完全不同**，合并成一条就会指错路（P0-1）。
  //    尤其："还没有雷达结果"（最常见）会被 TF 的文案带去看里程计。
  switch (reason) {
    case kFailNoLidar:
      RCLCPP_WARN(this->get_logger(),
        "还没有收到任何雷达结果 → 本帧不发（已累计 %llu 帧）。"
        "⚠️ 排查方向：**雷达链** —— lidar_perception_node 起了没、"
        "话题 `%s` 名对不对。**不要去查里程计或串口**",
        n, lidar_topic_.c_str());
      break;

    case kFailLidarNotOk:
      RCLCPP_ERROR(this->get_logger(),
        "雷达结果**已经不新鲜**（该路状态 STALE/FAILED）→ **本帧不发**（已累计 %llu 帧）。"
        "⚠️ 这是**故意的**：拿旧位置当当前状态发出去会让主控按过期障碍物避障。"
        "排查方向：雷达链为什么停了（`%s` 的 watchdog_ok / 点云断流）",
        n, lidar_topic_.c_str());
      break;

    case kFailEmptyFrameId:
      RCLCPP_WARN(this->get_logger(),
        "雷达结果的 `header.frame_id` 为空 → 无法 TF 到 world，本帧不发（已累计 %llu 帧）。"
        "排查方向：**上游 lidar_perception_node 要继承输入点云的 frame_id**"
        "（它自己不做坐标变换，见该节点实现）",
        n);
      break;

    case kFailTfFailed:
      RCLCPP_WARN(this->get_logger(),
        "TF 查询失败（逐点变换有失败）→ **本帧不发**（已累计 %llu 帧）。"
        "⚠️ 这与「通信链路断开」是**两件事**：链路是好的，是**算不出 world 坐标**。"
        "排查方向：里程计/Point-LIO 起了没、TF 树通不通 —— **不要去看串口**",
        n);
      break;

    default:
      break;
  }
}

std::optional<br_perception::msg::PerceptionFrame> PerceptionPipeline::assemble_frame(
    uint8_t& reason)
{
  reason = kFailNone;
  br_perception::msg::PerceptionFrame frame;

  // ═══════════════════════════════════════════════════════════════════════════
  // 前置门槛：**凑不齐就不组帧**。每一道都带**自己的原因码**（P0-1），
  // 调用方按原因分开计数、分开出文案 —— 合并成一种就会把排查方向指错。
  // ═══════════════════════════════════════════════════════════════════════════

  // ① 还没有过任何雷达结果
  //    ⚠️ 这时**不发**一个"空帧"：空帧在下游看来与"真的什么都没检测到"一样，
  //       分不出来（这正是 msg/CameraFrame.msg 警告过的坑）。
  if (!last_lidar_) {
    reason = kFailNoLidar;
    return std::nullopt;
  }

  // ② ⚠️⚠️ **雷达数据必须新鲜**（2026-09-22 用户复核 P0-2 —— 这条会撞车）
  //
  //    原实现只判 `last_lidar_` 是不是空，**收到后永不清空** → 雷达 STALE/FAILED
  //    之后照样拿那份**旧** LidarResult 组帧：障碍物/建筑位是旧位置，
  //    而 match_remaining_sec 是新的，帧还按 10Hz 发出去 ——
  //    **主控会把过期障碍位置当当前状态用**，这正是"会撞车"的那类错误。
  //
  //    ⚠️ 而且**不许拿相机顶上**：`kVisionOnly` 听起来像"退到纯视觉"，
  //       但相机给的是 **2D 检测结果**，要经融合层投影成 3D 才有用，
  //       而融合层（模块 21）还没实现。所以雷达一停，本节点**就是没有 3D 内容**，
  //       如实**不发**，比发一份旧的好。
  //    ⚠️ 这条与"帧率掉到 0"是同一个现象，但诊断里会明说是 `kFailLidarNotOk`
  //       （见 `warn_frame_failure`），排查方向指向**雷达链**，不是本节点。
  if (health_[kSrcLidar].state != kOk) {
    reason = kFailLidarNotOk;
    return std::nullopt;
  }

  // ③ 上游没填坐标系 → 无从 TF
  //    ⚠️ 不能"默认当成 livox_frame"去猜 —— 猜错就是把坐标算错，且**错得没提示**
  if (last_lidar_->header.frame_id.empty()) {
    reason = kFailEmptyFrameId;
    return std::nullopt;
  }

  // ── 时间戳：**继承上游的曝光/采集时刻**（任务书 §四），不用 now() ──
  {
    const rclcpp::Time s(last_lidar_->header.stamp,
                         this->get_clock()->get_clock_type());
    frame.header.stamp = stamp_is_unset(s) ? this->get_clock()->now() : s;
  }
  // ⚠️ 只有在**下面 TF 全部成功**之后才敢写 "world"（写早了就是撒谎）。
  //    先留空，成功后再赋值。
  frame.header.frame_id = "";

  // ── 自身位姿 / 里程计可靠性（模块 17 是权威）──
  // ⚠️ **取不到位姿就不发帧**（2026-09-22 用户复核 P2-10）：
  //    原先取不到就留零值 (0,0,0)，靠"反正 TF 失败也不会发"兜着 ——
  //    可那是**两个独立的判断**，将来只要有人放宽发布条件，一个 (0,0,0) 的
  //    `ego_pose` 就会静静地发出去（主控会以为自己在西南角）。
  //    这里直接返回，**不留任何"将来会漏出去"的零值**。
  if (tf_ == nullptr) {
    pose_available_ = false;
    reason = kFailTfFailed;
    return std::nullopt;
  }
  const auto pose = tf_->robot_pose_world();
  if (!pose) {
    pose_available_ = false;
    reason = kFailTfFailed;   // 位姿拿不到 = TF 树没通（同一个根因）
    return std::nullopt;
  }
  frame.ego_pose = *pose;
  frame.odometry_reliable = tf_->odometry_reliable();

  // ── 雷达结果 → world（**失败即整帧不发**，见头文件）──
  if (!transform_lidar_to_world(*last_lidar_, frame)) {
    pose_available_ = false;
    reason = kFailTfFailed;
    return std::nullopt;
  }
  frame.header.frame_id = "world";   // ✅ 到这里才敢写

  // ✅ 出得了 world 坐标 → 记上（P0-3：这是独立于"传感器在不在"的一维状态）
  pose_available_  = true;
  ever_had_pose_   = true;

  // ── 比赛计时（**近似**；契约：以主控计时为准，感知只做 release 预警）──
  {
    double remaining = 0.0;
    if (match_started_) {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - match_start_).count();
      remaining = std::max(0.0, match_duration_sec_ - elapsed);
    } else {
      remaining = match_duration_sec_;   // 还没起算 → 按满时长报
    }
    frame.match_remaining_sec = static_cast<uint8_t>(std::min(255.0, remaining));
    // 到点即置 1：**保守置位**（宁可多报一次提醒释放，也不要漏报导致终场持握扣分）
    frame.match_end_release_warning = (remaining <= 0.0) ? 1 : 0;
  }

  // ── ⚠️⚠️ 本节点**填不了**的字段（逐条理由见头文件那张表）──
  //
  // `mustika_state`：**来自融合层**（`mustika_pillar_check`，模块 20）。
  // ⚠️ 2026-09-23 起**不再写死 0** —— 改订阅 `/perception/fusion/mustika_state`。
  //    在补这条订阅之前，那个节点的结论**再对也到不了这里**（它零消费者）。
  //
  // ⚠️ **三档回退规则**（和它去未知的逻辑一致）：
  //    · 收到过、且没过 `fusion_timeout_sec_` → 用它的值（0/1/5）
  //    · **没收到过 / 已超时** → `0 = 未知`
  //    ⚠️ 超时**退回未知，而不是"保持上一次的结论"** —— 旧结论比没有结论危险
  //       （球可能早被拿走了，而主控会照"在柱上"去派 TR）。
  {
    const bool fresh =
        !tp_unset(fusion_state_at_) &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - fusion_state_at_).count()
            <= fusion_timeout_sec_;
    // ⚠️ **0 是消息里明确定义的"未知"档**，是真哨兵，不是"没填" —— 可以放心用
    frame.mustika_state = fresh ? last_fusion_state_ : 0u;
  }
  //
  // `mustika_pose`：⚠️ **零值 PoseStamped 在协议里是一个合法的世界坐标 (0,0,0)
  //   = 场地西南角**，编码器会照发 int16 cm、主控会照用。这不是"不填"，是**伪造位置**。
  //   当前靠**契约**兜底：「`mustika_state == 0` 时主控必须忽略 `mustika_pose`」，
  //   并已列入必须落地的协议改动（头文件 ③ ①）。**在协议加 `pose_valid` 之前，
  //   这条契约是唯一的防线** —— 不要删掉这句注释。
  //   （零值即默认构造，这里不额外赋值。）
  //
  // `sanctuary_mandate_fulfilled`：❌ **本节点不填**。该字段**没有未知档**（只有 0/1）：
  //   填 0 = 断言"使命未达成" → **TR 永远不去取穆斯蒂卡，白丢 250 分**；
  //   填 1 更糟（TR 会违规去取）。正确做法是**移出上行帧**（归融合层或主控），
  //   已列入头文件 ③ ②。这里保持默认 0，并在 diagnostics 里**点名**
  //   "NOT PROVIDED BY PERCEPTION"，让主控侧读错时能立刻查到原因。
  frame.sanctuary_mandate_fulfilled = 0;
  //
  // `held_object_count` / `holding_over_limit`：❌ 夹爪上有几块**只有主控知道**，
  //   感知链路根本无从观测。已列入头文件 ③ ③（应移出上行帧，由主控填）。
  frame.held_object_count  = 0;
  // ⚠️ `holding_over_limit` 保留 0 是**可接受的**（与上面两条不同）：风险不对称 ——
  //    恒 0 是"安全默认"（不会误报超限去打断主控的正常动作）。
  frame.holding_over_limit = 0;

  // ── 诊断 ──
  {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "perception_pipeline";
    st.hardware_id = "camera+mid360";
    const uint8_t mode = decide_mode();
    // ⚠️ 等级走**唯一的** `diag_level_of()`，与降级诊断**同一个口径**（P1-4）
    st.level = diag_level_of(mode);
    st.message = mode_message(mode);   // ⚠️ 不是 mode_name —— 见该函数的注释

    auto kv = [](const std::string& k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; return p;
    };
    st.values.push_back(kv("mode", mode_message(mode)));
    // 逐路点名 —— 现场排查全靠这个，**不要只报一个"降级了"**
    for (size_t i = 0; i < kSrcCount; ++i) {
      st.values.push_back(kv(std::string("src_") + source_name(i),
                             state_name(health_[i].state)));
    }
    // ⚠️ 这两项**必须点名**：它们不是"值就是 0"，是"感知根本没提供"（见头文件那张表）
    st.values.push_back(kv("sanctuary_mandate_fulfilled", "NOT PROVIDED BY PERCEPTION"));
    st.values.push_back(kv("held_object_count",           "NOT PROVIDED BY PERCEPTION"));
    // ⚠️ 融合层没实现 → 恒 true，是**预期**（任务书 §6.1 的"融合异常→透传雷达"）
    st.values.push_back(kv("fusion_bypass", "1 (模块21未实现, 预期行为)"));
    // ⚠️ mustika_state 来自融合层，**必须能看出它是"真值"还是"回退的未知"** ——
    //    否则现场看到 0 会分不清"球不在"与"融合层没给出结论"（而 0 的定义是后者）。
    st.values.push_back(kv("mustika_state",
                           std::to_string(frame.mustika_state) +
                               (tp_unset(fusion_state_at_) ? " (融合层从未给过 → 回退未知)"
                                                           : " (来自融合层)")));
    // ⚠️ 帧内诊断也给一份**失败原因分类**（与降级诊断同源，见 P0-1）：
    //    主控收到帧时若发现"帧突然不来了"，这一条能立刻告诉它该往哪查。
    st.values.push_back(kv("fail_no_lidar",       std::to_string(fail_counts_[kFailNoLidar])));
    st.values.push_back(kv("fail_lidar_not_ok",   std::to_string(fail_counts_[kFailLidarNotOk])));
    st.values.push_back(kv("fail_empty_frame_id", std::to_string(fail_counts_[kFailEmptyFrameId])));
    st.values.push_back(kv("fail_tf_failed",      std::to_string(fail_counts_[kFailTfFailed])));
    st.values.push_back(kv("event_frames",  std::to_string(event_frames_)));

    frame.diagnostics.push_back(st);
  }

  // ── 帧序号（递增）──
  frame.seq = frame_seq_;

  return frame;
}

bool PerceptionPipeline::transform_lidar_to_world(
    const br_perception::msg::LidarResult& in,
    br_perception::msg::PerceptionFrame& out)
{
  if (!tf_) return false;

  // ⚠️ `in.header.frame_id` **可能是空的**（上游还没设）→ 这种帧不能转，
  //    直接失败。别用"默认当成 livox_frame"去猜 —— 猜错就是把坐标算错，
  //    而且**错得没有提示**（同 LidarResult.msg 里那段 frame_id 的理由）。
  if (in.header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "雷达结果的 header.frame_id 为空 → 无法 TF 到 world，本帧不发。"
      "上游 lidar_perception_node 应继承输入点云的 frame_id");
    return false;
  }

  const rclcpp::Time stamp(in.header.stamp, this->get_clock()->get_clock_type());

  auto to_world = [&](double x, double y, double z,
                      geometry_msgs::msg::Point& dst) -> bool {
    geometry_msgs::msg::PointStamped pin, pout;
    pin.header = in.header;       // frame_id + stamp 都用上游的（**不是 now()**）
    pin.point.x = x; pin.point.y = y; pin.point.z = z;
    if (!tf_->transform_to_world(pin, pout)) return false;
    dst = pout.point;
    return true;
  };

  // ── 障碍物 ──
  out.obstacles.reserve(in.obstacles.size());
  for (const auto& o : in.obstacles) {
    br_perception::msg::Obstacle m = o;      // 其余字段直接透传
    if (!to_world(o.position.x, o.position.y, o.position.z, m.position)) return false;
    out.obstacles.push_back(m);
  }

  // ── 建筑位 ──
  //  ⚠️⚠️ 这里做**本节点最要紧的一次融合**（2026-09-24 加）：
  //     雷达给"这个位上有没有塔/几块"（它对颜色无能为力，`top_color` 恒填 UNKNOWN=3），
  //     相机给"**顶面是什么颜色**" —— 而规则 3.5.6/3.5.7 里那 40/80 分的归属
  //     **只由顶面颜色决定**。不在这里合，主控拿到的就永远是"颜色未知"。
  out.building_spots.reserve(in.building_spots.size());
  for (const auto& s : in.building_spots) {
    br_perception::msg::BuildingSpot m = s;
    if (!to_world(s.position.x, s.position.y, s.position.z, m.position)) return false;

    // ── 相机颜色覆盖（只在"相机敢下结论"时）──
    //   ⚠️ 三条判据都在 `camera_color_for` 里（新鲜度 / CONFIRMED / 两路不打架）。
    //   ⚠️ 覆盖的是**颜色**，不是状态：状态的来源仍是雷达。
    uint8_t cam_color = 0;
    std::string why;
    if (camera_color_for(s.spot_id, cam_color, why)) {
      m.top_color = cam_color;
      // ⚠️ `source` 填 **FUSION(2)** 而不是 CAMERA(1)：
      //    这个字段描述的是**这条建筑位记录**的来源，而它现在是
      //    "状态来自雷达 + 颜色来自相机" —— 单一来源都不是实话。
      //    （契约里 `source` 的三个取值正是为这种情况准备的。）
      m.source = kMsgSourceFusion;
    }
    // ⚠️ 相机**判不出来**时，**不动** `top_color`：它保持雷达填的 UNKNOWN(3)
    //    —— "有方块但颜色未知"是对的诚实值（`NONE` 才是"这里没有方块"）。

    out.building_spots.push_back(m);
  }

  // ── 两根柱子 ──
  // ⚠️ 没检出的柱子是**全零哨兵**（见 lidar_perception_node 的 to_msg_pillar）：
  //    height==0 && diameter==0。这种**不参与 TF** —— 拿 (0,0,0) 去变换会得到
  //    一个"世界系里的原点附近"的假柱子位置，比不报更糟。
  auto pillar = [&](const br_perception::msg::PillarStatus& src,
                    br_perception::msg::PillarStatus& dst) -> bool {
    dst = src;
    if (src.height == 0.0f && src.diameter == 0.0f) return true;   // 未检出 → 原样透传
    geometry_msgs::msg::Point p;
    if (!to_world(src.position.x, src.position.y, src.position.z, p)) return false;
    dst.position = p;
    return true;
  };
  if (!pillar(in.mustika_pillar, out.mustika_pillar)) return false;
  if (!pillar(in.core_pillar,    out.core_pillar))    return false;

  return true;   // ✅ **全部成功**才算成功（一半世界系一半传感器系的帧比不发还糟）
}

// ═══════════════════════════════════════════════════════════════════════════════
// 降级诊断
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::publish_degradation_state()
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->get_clock()->now();

  const uint8_t mode = decide_mode();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name        = "perception_pipeline/degradation";
  st.hardware_id = "camera+mid360";
  // ⚠️ **必须走 `mode_message` 而不是 `mode_name`** —— 后者只印 `VISION_ONLY`，
  //    会让人读成"降级后仍可用"，而那个模式**实际一帧都不发**。
  //    理由与两处共用的原因见 `mode_message` 的注释。
  st.message = mode_message(mode);

  // ⚠️⚠️ 等级**不能**因为某路是 NEVER 就报 WARN/ERROR（2026-09-22 用户要求）：
  //    融合层（模块 21）**今天就是不存在的**，`fusion` 会长期是 NEVER。
  //    若把它当故障挂黄灯，从今天到模块 21 落地这段时间面板**一直黄**，
  //    大家很快学会无视它 —— **真出事时就看不见了**。
  //    NEVER 只作为**信息**列出（"未接入"），等级按模式给。
  // ⚠️⚠️ 等级与 `assemble_frame()` 里那条**必须一致**（2026-09-22 用户复核 P1-4）：
  //    原来这里退化成"只有 EMERGENCY 是 ERROR、其余全 OK"，而同一个 LIDAR_ONLY
  //    在帧内诊断里是 WARN —— **同一个状态两套等级**，两边看对不上。
  //    统一口径：NORMAL/STARTING = OK，LIDAR_ONLY/VISION_ONLY = WARN，
  //              EMERGENCY/NO_POSE = ERROR。
  //    ⚠️ kNoPose 判 ERROR 的理由：它意味着**一帧都发不出去**（主控彻底收不到），
  //       从主控视角看比"传感器少一路"严重得多。
  // ⚠️ 走**唯一的** `diag_level_of()` —— 与帧内诊断**同一个口径**，见该函数注释。
  st.level = diag_level_of(mode);

  auto kv = [](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; return p;
  };
  // ⚠️ 与 `st.message` 同源（`mode_message`）—— 两处印同一个模式名却一个有说明、
  //    一个没有，正是"同一状态两套说法"（同 P1-4 修过的那个坑）。字符串仍以
  //    `VISION_ONLY` 开头，**grep 照样能搜到**。
  st.values.push_back(kv("mode", mode_message(mode)));
  for (size_t i = 0; i < kSrcCount; ++i) {
    std::string note = state_name(health_[i].state);
    // 把"未接入"与"死了"在**文字上**分开 —— 两者在数值上都不可用，
    // 但现场要做的事完全不同（一个去查话题名，一个去查硬件）
    if (health_[i].state == kNever)   note += " (未接入, 不是故障)";
    if (health_[i].state == kFailed)  note += " (有数据但自报不健康)";
    // ⚠️ 超时转 STALE 后**要说清"最后自报是不健康的"**（P2-11）：
    //    否则现场看到 STALE 会以为是"数据还在来只是慢"，而实际上那路
    //    可能早就没消息了 —— 两件事的排查方向不一样
    if (health_[i].state == kStale && !health_[i].healthy) {
      note += " (最后一条自报不健康)";
    }
    st.values.push_back(kv(std::string("src_") + source_name(i), note));
  }
  // ⚠️ **"能否出 world 坐标"必须单列**（2026-09-22 用户复核 P0-3）：
  //    出不了坐标 = **一帧都发不出去**，而它原先只藏在模式里看不出来。
  st.values.push_back(kv("pose_available",
                         pose_available_ ? "1" : (ever_had_pose_ ? "0 (曾经可用!)" : "0 (从未可用)")));
  st.values.push_back(kv("assembled_frames", std::to_string(assembled_frames_)));
  st.values.push_back(kv("event_frames",     std::to_string(event_frames_)));
  // ⚠️ 组帧失败**按原因分开列**（P0-1）—— 合成一个数就只能靠猜，
  //    而"还没雷达"与"TF 失败"现场要做的事完全不同
  st.values.push_back(kv("fail_no_lidar",      std::to_string(fail_counts_[kFailNoLidar])));
  st.values.push_back(kv("fail_lidar_not_ok",  std::to_string(fail_counts_[kFailLidarNotOk])));
  st.values.push_back(kv("fail_empty_frame_id",std::to_string(fail_counts_[kFailEmptyFrameId])));
  st.values.push_back(kv("fail_tf_failed",     std::to_string(fail_counts_[kFailTfFailed])));
  st.values.push_back(kv("fps",              std::to_string(fps_.fps())));
  {
    const auto s = profiler_.stats(idx_end_to_end_);
    st.values.push_back(kv("end_to_end_ms_mean", std::to_string(s.mean_ms)));
    st.values.push_back(kv("end_to_end_ms_max",  std::to_string(s.max_ms)));
  }

  arr.status.push_back(st);
  pub_degradation_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 紧急停止
// ═══════════════════════════════════════════════════════════════════════════════
void PerceptionPipeline::publish_emergency_stop(bool stop)
{
  const auto now_tp = std::chrono::steady_clock::now();

  // 只在**状态变化**时发；仍处于紧急状态则按 `emergency_repeat_sec_` 重复。
  // ⚠️ 为什么允许重复：下游（comm_node）可能**中途才连上** —— 完全不重发
  //    会让它永远不知道当前处于紧急状态，而那正是最需要它知道的时刻。
  const bool changed = (stop != emergency_active_);
  if (!changed) {
    if (!stop) return;                      // 状态没变且不是紧急 → 什么都不发
    if (emergency_repeat_sec_ <= 0.0) return;
    if (!tp_unset(last_emergency_pub_) &&
        std::chrono::duration<double>(now_tp - last_emergency_pub_).count() <
            emergency_repeat_sec_) {
      return;
    }
  }

  emergency_active_ = stop;
  last_emergency_pub_ = now_tp;

  std_msgs::msg::Bool msg;
  msg.data = stop;
  pub_emergency_->publish(msg);

  if (changed) {
    if (stop) {
      RCLCPP_ERROR(this->get_logger(),
        "进入紧急状态：两个环境传感器（雷达/相机）**曾经可用，现在都没了** → "
        "发 %s。⚠️ 主控要真收到停车指令，还需要 comm_node 订阅本话题并落到线上"
        "（0x03 或心跳位）—— 见 perception_pipeline.hpp 的降级那节",
        emergency_stop_topic_.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "退出紧急状态");
    }
  }
}

}  // namespace core
}  // namespace br_perception

// ⚠️ `RCLCPP_COMPONENTS_REGISTER_NODE` 由这个头文件定义 —— 漏了它不是"未定义标识符"
//    那么直白，而是 "expected constructor, destructor, or type conversion at end of input"
//    （2026-09-23 首次编译实测，浪费一轮）。lidar_perception_node.cpp 也是这个写法。
#include <rclcpp_components/register_node_macro.hpp>

// ═══════════════════════════════════════════════════════════════════════════════
// 组件注册（容器化时被容器加载; 也生成同名独立可执行）
// ═══════════════════════════════════════════════════════════════════════════════
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::core::PerceptionPipeline)
