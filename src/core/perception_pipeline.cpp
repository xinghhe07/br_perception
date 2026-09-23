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
  sub_comm_diag_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    comm_diag_topic_, rclcpp::QoS(10),
    std::bind(&PerceptionPipeline::on_comm_diag, this, std::placeholders::_1));

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
    "PerceptionPipeline ready | in: lidar=%s camera=%s comm_diag=%s | out: %s | "
    "publish=%.1fHz | timeouts(lidar/cam/fusion/odom/comm)=%.2f/%.2f/%.2f/%.2f/%.2fs",
    lidar_topic_.c_str(), camera_topic_.c_str(), comm_diag_topic_.c_str(),
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

void PerceptionPipeline::on_camera_frame(
    const br_perception::msg::CameraFrame::SharedPtr msg)
{
  mark_seen(kSrcCamera, true);

  // ⚠️ 本节点**只关心相机链活没活**，不用它的 2D 检测结果 ——
  //    那些要经融合层投影到 3D 才有用，而融合层（模块 21）还没实现。
  //    存时间戳是为了"多久没来了"的判定，也为将来接融合留个抓手。
  last_camera_stamp_ = rclcpp::Time(msg->header.stamp,
                                    this->get_clock()->get_clock_type());
  if (stamp_is_unset(*last_camera_stamp_)) {
    // 发布方没填时间戳 → 退回本节点收到它的时刻（不理想，但比按 1970 年算强）。
    // ⚠️ 按**来源**只告警一次，且用**成员**而不是函数级 `static`（2026-09-22 用户复核 P2-9）：
    //    函数级 static 在多线程执行器下是竞态，同进程多实例（容器里跑两份）还会互相影响
    //    —— 而它本来要解决的恰恰是"别漏报每个来源"，用 static 正好违背它。
    if (!warned_camera_stamp_) {
      warned_camera_stamp_ = true;
      RCLCPP_WARN(this->get_logger(),
        "相机聚合帧的 header.stamp 为 0（发布方未填时间戳）—— 本节点退回用到达时刻");
    }
    last_camera_stamp_ = this->get_clock()->now();
  }

  try_publish(/*is_event=*/true);
}

void PerceptionPipeline::on_comm_diag(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  // 从 `/communication/diagnostics` 的 `communication/serial` 里取 `connected` 键。
  // ⚠️ 通信链路的健康**只有 comm_node 知道**（它持有串口），本节点只是**观测**：
  //    "通信断开 → 本地缓存最新帧，重连后补发"是**它的活**（见头文件偏差表），
  //    在感知侧再缓存一份是重复，而且两份队列的丢弃策略必然不一致。
  for (const auto& st : msg->status) {
    if (st.name != "communication/serial") continue;
    for (const auto& kv : st.values) {
      if (kv.key != "connected") continue;
      mark_seen(kSrcComm, kv.value == "1");
      return;   // 找到就走：一个诊断数组里这个 status 只该有一个
    }
  }
  // 话题来了但没有这两个键 → 说明 comm_node 的诊断格式变了。
  // **不 mark_seen**：宁可让 comm 因超时变 STALE，也不要拿一个读不懂的诊断
  // 去"证明"链路是好的（那会把真故障盖掉）。
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
  s.spot_states.reserve(lidar.building_spots.size());
  for (const auto& sp : lidar.building_spots) {
    s.spot_states.emplace_back(sp.spot_id, sp.state);
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
  // `mustika_state = 0`：**0 是消息里明确定义的"未知"档**，是**真哨兵**，可以用。
  frame.mustika_state = 0;   // 0 = 未知（归融合层，模块 20/21）
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
    st.message = mode_name(mode);

    auto kv = [](const std::string& k, const std::string& v) {
      diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = v; return p;
    };
    st.values.push_back(kv("mode", mode_name(mode)));
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
  out.building_spots.reserve(in.building_spots.size());
  for (const auto& s : in.building_spots) {
    br_perception::msg::BuildingSpot m = s;
    if (!to_world(s.position.x, s.position.y, s.position.z, m.position)) return false;
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
  st.message     = mode_name(mode);

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
  st.values.push_back(kv("mode", mode_name(mode)));
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
