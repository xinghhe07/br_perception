#include "br_perception/communication/comm_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>      // std::snprintf —— 不显式包含可能被其它头间接带进来, 但不可依赖
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace br_perception {
namespace communication {

namespace {

diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue out;
  out.key = key;
  out.value = value;
  return out;
}

std::string to_str(std::uint64_t v) { return std::to_string(v); }

/// 16 位量按十六进制写成 "0x1021" —— 协议表上就是这么写的,
/// 现场跟主控那边的配置比对时不该让人做十进制心算。
std::string to_hex16(std::uint16_t v)
{
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%04X", static_cast<unsigned>(v));
  return std::string(buf);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 配置加载
// ═══════════════════════════════════════════════════════════════════════════

SerialConfig CommNode::load_serial_config(rclcpp::Node& node)
{
  SerialConfig c;

  // ── 物理层 (⚠ 未冻结项: 没有下位机可确认) ──
  c.device = node.declare_parameter<std::string>("serial_device", c.device);
  c.baud_rate = node.declare_parameter<int>("serial_baud_rate", c.baud_rate);
  c.data_bits = node.declare_parameter<int>("serial_data_bits", c.data_bits);
  c.stop_bits = node.declare_parameter<int>("serial_stop_bits", c.stop_bits);
  const std::string parity = node.declare_parameter<std::string>("serial_parity", "N");
  c.parity = parity.empty() ? 'N' : parity[0];
  c.flow_control = node.declare_parameter<bool>("serial_flow_control", c.flow_control);

  // ── 协议参数 (⚠ 未冻结项) ──
  // 必须同时喂给 encoder 和 decoder (后者由 SerialConfig 带进 SerialManager) ——
  // 只改一边的症状是"串口正常、却一帧都收不到"。
  c.protocol.crc16_polynomial = static_cast<std::uint16_t>(
      node.declare_parameter<int>("crc16_polynomial", c.protocol.crc16_polynomial));
  c.protocol.crc16_initial = static_cast<std::uint16_t>(
      node.declare_parameter<int>("crc16_initial", c.protocol.crc16_initial));
  c.protocol.crc16_xor_out = static_cast<std::uint16_t>(
      node.declare_parameter<int>("crc16_xor_out", c.protocol.crc16_xor_out));
  c.protocol.crc16_reflect_in =
      node.declare_parameter<bool>("crc16_reflect_in", c.protocol.crc16_reflect_in);
  c.protocol.crc16_reflect_out =
      node.declare_parameter<bool>("crc16_reflect_out", c.protocol.crc16_reflect_out);
  c.protocol.max_building_spots = static_cast<std::uint8_t>(
      node.declare_parameter<int>("max_building_spots", c.protocol.max_building_spots));
  c.protocol.max_obstacles = static_cast<std::uint8_t>(
      node.declare_parameter<int>("max_obstacles", c.protocol.max_obstacles));

  // ── 时序 / 告警 ──
  c.send_rate_hz = node.declare_parameter<int>("send_rate_hz", c.send_rate_hz);
  c.heartbeat_interval_ms =
      node.declare_parameter<int>("heartbeat_interval_ms", c.heartbeat_interval_ms);
  c.reconnect_interval_ms =
      node.declare_parameter<int>("reconnect_interval_ms", c.reconnect_interval_ms);
  c.receive_poll_timeout_ms =
      node.declare_parameter<int>("receive_poll_timeout_ms", c.receive_poll_timeout_ms);
  c.crc_alarm_threshold =
      node.declare_parameter<int>("crc_alarm_threshold", c.crc_alarm_threshold);

  return c;   // SerialManager 构造时会再 sanitize 一遍非法取值
}

// ═══════════════════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════════════════

CommNode::CommNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("comm_node", options)
  // ⚠ 顺序要紧: Node 基类已构造完 → 可以读参数; 而 serial_ 是值成员,
  //   放到构造函数体里再配就晚了 (它早拿着默认配置构造完了)。
  , serial_config_(load_serial_config(*this))
  , encoder_(serial_config_.protocol)      // 不能用默认 ProtocolConfig
  , serial_(serial_config_)
{
  // ── 纯 ROS 侧参数 (与串口无关的那些) ──
  output_topic_ = declare_parameter<std::string>("output_topic", output_topic_);
  from_master_topic_ = declare_parameter<std::string>("from_master_topic", from_master_topic_);
  diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", diagnostics_topic_);
  // ⚠️ 必须与 perception_pipeline 的 `emergency_stop_topic`（config/perception_params.yaml）
  //    逐字一致 —— 对不上就是"上游在报紧急、线上一个 0x03 都没有"，且**不报任何错**。
  emergency_stop_topic_ =
      declare_parameter<std::string>("emergency_stop_topic", emergency_stop_topic_);

  // 夹取: 0 或负值会让 wall timer 变成忙循环
  diagnostics_interval_sec_ =
      std::max(0.1, declare_parameter<double>("diagnostics_interval_sec", 1.0));
  debug_mode_ = declare_parameter<bool>("debug_mode", false);
  debug_interval_sec_ = std::max(0.1, declare_parameter<double>("debug_interval_sec", 2.0));

  // ── 发布者 ──
  // QoS 用默认 (RELIABLE, depth 10)。⚠ 若将来消费者用 BEST_EFFORT 订阅, 这里没问题;
  //   反过来才有问题: 本节点**不能**用 RELIABLE 去订阅别人的 BEST_EFFORT 发布。
  from_master_pub_ = create_publisher<msg::MasterCommand>(from_master_topic_, rclcpp::QoS(10));
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));

  // ── 订阅 ──
  // ⚠ 用 SensorDataQoS (BEST_EFFORT): 它**同时兼容** RELIABLE 与 BEST_EFFORT 的发布者,
  //   而 RELIABLE 订阅遇上 BEST_EFFORT 发布是静默收不到。本仓库踩过一次这个坑,
  //   规则写死在 doc/容器化重构_任务清单_20260914.md 的"已知坑"第 1 条。
  perception_sub_ = create_subscription<msg::PerceptionFrame>(
      output_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CommNode::on_perception, this, std::placeholders::_1));

  // 紧急停止：发布端是 perception_pipeline（`std_msgs/Bool`）。
  // ⚠️ 用默认 QoS(RELIABLE, depth 10) —— 与发布端一致（那边也是 `rclcpp::QoS(10)`）。
  //    这条**不能**图省事用 SensorDataQoS：安全信号丢一条就是少一次停车机会。
  //    （方向也正好相反：这里若用 BEST_EFFORT 订 RELIABLE 发布**是兼容的**，
  //      但那就放弃了重传，没有必要。）
  emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(10),
      std::bind(&CommNode::on_emergency_stop, this, std::placeholders::_1));

  // ── 把 SerialManager 的两个回调接上 ──
  serial_.set_frame_callback([this](const DecodedFrame& frame) { on_master_frame(frame); });

  // 心跳 provider: 在**发送线程**上执行。只碰 encoder_ —— 它的帧序号是原子的,
  // 故无需加锁 (这正是当初把序号做成 atomic 的回报)。
  serial_.set_heartbeat_provider([this]() { return encoder_.encode_heartbeat(); });

  // ── 启动串口 ──
  if (serial_.start()) {
    RCLCPP_INFO(get_logger(), "串口已打开: %s @ %d", serial_config_.device.c_str(),
                serial_config_.baud_rate);
  } else {
    // ⚠ 刻意**不抛异常** —— 开发机上没插硬件是常态, 节点仍应能起来,
    //   方便调话题/参数。诊断里会报 device_unavailable, 监控层看得见。
    device_unavailable_.store(true);
    RCLCPP_WARN(get_logger(),
                "串口打开失败 (%s @ %d) —— 节点继续运行但不通信, 诊断会报 device_unavailable",
                serial_config_.device.c_str(), serial_config_.baud_rate);
  }

  // ── 定时器 ──
  diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(diagnostics_interval_sec_),
      std::bind(&CommNode::publish_diagnostics, this));

  if (debug_mode_) {
    debug_timer_ = create_wall_timer(
        std::chrono::duration<double>(debug_interval_sec_),
        std::bind(&CommNode::send_debug_text, this));
    RCLCPP_WARN(get_logger(),
                "调试模式已开启: 会额外发送 0x7F ASCII 帧 —— 比赛时务必关闭");
  }
}

CommNode::~CommNode()
{
  // 显式停: 让收发线程在 ROS 基类析构**之前**退出, 不依赖"成员声明顺序恰好对"
  serial_.stop();
}

// ═══════════════════════════════════════════════════════════════════════════
// 上行: 融合结果 → 串口
// ═══════════════════════════════════════════════════════════════════════════

void CommNode::on_perception(const msg::PerceptionFrame::SharedPtr msg)
{
  EncodeResult result = encoder_.encode(*msg);

  // 截断是重要事件: 主控会以为收到的是全部建筑位/障碍物, 实际不是。
  // 只计数不刷日志 —— 这个条件一旦成立会每帧都成立, 打日志会淹掉别的信息。
  if (result.spots_written < msg->building_spots.size() ||
      result.obstacles_written < msg->obstacles.size()) {
    truncated_frames_.fetch_add(1);
  }

  serial_.post_frame(std::move(result.bytes));
}

void CommNode::on_emergency_stop(const std_msgs::msg::Bool::SharedPtr msg)
{
  // ⚠️ **两个方向都发**：`false` 是"可以恢复行驶"。只发停车不发解除，
  //    主控会永远停在那个状态（这正是 `encode_estop` 头注释里写的那条）。
  const bool active = msg->data;

  // ⚠️ 时间戳用 `now()`（ROS 时间，随 `use_sim_time` 走）——
  //    与感知帧同源。编码器自己不取时钟，就是为了不让这里出现两个来源。
  serial_.post_frame(encoder_.encode_estop(active, now()));

  if (active) {
    ++estop_frames_;
    ++estop_entries_;
  } else {
    ++estop_clear_frames_;
  }

  // ⚠️ 这条**必须打日志**（与感知帧的截断计数刻意不同）：
  //    紧急停止是**低频且要人立刻知道**的事件。现场日志里没有这一行，
  //    就没法判断"到底发出去没有" —— 而那正是这个包存在的理由。
  RCLCPP_WARN(get_logger(), "紧急停止%s → 已编码 0x03 入发送队列（累计 触发 %llu / 解除 %llu）",
              active ? "**触发**" : "解除",
              static_cast<unsigned long long>(estop_frames_.load()),
              static_cast<unsigned long long>(estop_clear_frames_.load()));
}

// ═══════════════════════════════════════════════════════════════════════════
// 下行: 串口 → /communication/from_master
// ═══════════════════════════════════════════════════════════════════════════

void CommNode::on_master_frame(const DecodedFrame& frame)
{
  msg::MasterCommand out;
  out.header.stamp = now();
  // 主控帧不属于任何坐标系 —— 留空而不是硬填一个 "world" 假装它有
  out.header.frame_id = "";
  out.seq = frame.seq;
  out.type = frame.type;
  out.payload = frame.payload;

  // 一律原样发布: 不解析、不丢弃。主控侧除 0x10 外都没有字节级定义,
  // "看不懂就丢"会把将来才定义的指令提前扔掉。
  from_master_pub_->publish(out);
}

// ═══════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════

void CommNode::publish_diagnostics()
{
  const SerialStats s = serial_.stats();
  const bool open = serial_.device_open();
  const bool connected = serial_.connected();

  if (!open) {
    device_unavailable_.store(true);
  } else if (connected) {
    device_unavailable_.store(false);
  }

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "communication/serial";
  st.hardware_id = serial_config_.device;

  if (!open) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "串口未打开 (设备不在或权限不足)";
  } else if (!connected) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "链路断开, 重连中";
  } else if (serial_.crc_alarm()) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "CRC 连续错误超阈值 —— 怀疑硬件干扰/线缆/波特率";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "正常";
  }

  st.values = {
      kv("device", serial_config_.device),
      kv("device_open", open ? "1" : "0"),
      kv("connected", connected ? "1" : "0"),

      // ── 线格式参数: 两端必须逐项一致 ──
      // 单独列在最前面, 是为了**现场比对**: 参数不一致的症状是"串口打开正常、
      // 各项计数都正常、就是一帧都收不到", 而原来的 22 项里没有一项说明
      // 当前用的是哪组参数 —— 只能看到 crc_errors 涨, 却不知道拿什么去对。
      // 用法: ros2 topic echo /communication/diagnostics --once, 对着主控配置逐行核。
      kv("serial_frame", std::to_string(serial_config_.data_bits) +
                             std::string(1, serial_config_.parity) +
                             std::to_string(serial_config_.stop_bits)),   // 如 "8N1"
      kv("serial_baud_rate", to_str(static_cast<std::uint64_t>(serial_config_.baud_rate))),
      kv("serial_flow_control", serial_config_.flow_control ? "1" : "0"),
      kv("crc16_polynomial", to_hex16(serial_config_.protocol.crc16_polynomial)),
      kv("crc16_initial", to_hex16(serial_config_.protocol.crc16_initial)),
      kv("crc16_reflect_in", serial_config_.protocol.crc16_reflect_in ? "1" : "0"),
      kv("crc16_reflect_out", serial_config_.protocol.crc16_reflect_out ? "1" : "0"),
      kv("crc16_xor_out", to_hex16(serial_config_.protocol.crc16_xor_out)),

      kv("uptime_ms", to_str(s.uptime_ms)),
      kv("bytes_sent", to_str(s.bytes_sent)),
      kv("bytes_received", to_str(s.bytes_received)),
      kv("frames_sent", to_str(s.frames_sent)),
      kv("heartbeats_sent", to_str(s.heartbeats_sent)),
      kv("frames_received", to_str(s.frames_received)),
      // ⚠ 这两项是"收到了但没给上层"的唯一证据: frames_received 不动时,
      //   stale_frames 在涨 = 被序号门控丢了 (重传/重复/主控重启),
      //   而不是链路没数据 (那要看 bytes_received 是否也不动)。
      kv("stale_frames", to_str(s.stale_frames)),
      kv("ambiguous_seq_frames", to_str(s.ambiguous_seq_frames)),
      // 非零 = 主控重启过 (或判据误触)。它和 stale_frames 一起看:
      // stale 涨而这里一直是 0 → 只是重传/重复; 这里涨 → 对面真的重新编过号。
      kv("renumber_recoveries", to_str(s.renumber_recoveries)),
      kv("crc_errors", to_str(s.crc_errors)),
      // 两个丢帧计数器分开报: 一个高说明上层投太快, 另一个高说明串口发不动
      kv("frames_dropped_on_post", to_str(s.frames_dropped_on_post)),
      kv("frames_dropped_on_send", to_str(s.frames_dropped_on_send)),
      kv("callback_errors", to_str(s.callback_errors)),
      kv("write_errors", to_str(s.write_errors)),
      kv("reconnect_attempts", to_str(s.reconnect_attempts)),
      kv("reconnects_succeeded", to_str(s.reconnects_succeeded)),
      kv("decoder_pending_bytes", to_str(s.decoder_pending_bytes)),
      kv("queued_frames", to_str(serial_.pending_frames())),
      // 通信层才知道协议上界, 所以由它上报"融合层给多了"
      kv("truncated_frames", to_str(truncated_frames_.load())),

      // ── 紧急停止（0x03）──
      // ⚠️ 这三项是**"停车指令到底发出去没有"的唯一观测窗口**。
      //    判读方式（写在这里免得现场去翻代码）：
      //      · estop_frames / estop_clear **都为 0**，而上游在报紧急
      //          → 本节点的订阅**没接上**（话题名对不上 / 上游没发）
      //      · estop_entries 一直涨
      //          → 上游在**紧急/正常之间来回跳**，那是上游判定抖动，不是通信问题
      //      · estop_frames 稳定增长 = 上游**一直停在紧急**，本节点在按
      //        `emergency_repeat_sec` 重发（正常行为，防下游中途才连上）
      kv("estop_frames",       to_str(estop_frames_.load())),
      kv("estop_clear_frames", to_str(estop_clear_frames_.load())),
      kv("estop_entries",      to_str(estop_entries_.load())),
  };

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now();
  arr.status.push_back(st);
  diagnostics_pub_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════
// 调试模式 (任务书 §5.4)
// ═══════════════════════════════════════════════════════════════════════════

void CommNode::send_debug_text()
{
  // ⚠ 队列非空就**不投** —— 调试帧和感知帧共用发送队列 (只有 2 个位),
  //   调试信息挤掉真实感知数据是本末倒置。
  if (serial_.pending_frames() != 0) {
    return;
  }

  const SerialStats s = serial_.stats();
  char line[256];
  std::snprintf(line, sizeof(line),
                "STAT up=%llums tx=%llu rx=%llu crc=%llu drop=%llu/%llu trunc=%llu\n",
                static_cast<unsigned long long>(s.uptime_ms),
                static_cast<unsigned long long>(s.frames_sent),
                static_cast<unsigned long long>(s.frames_received),
                static_cast<unsigned long long>(s.crc_errors),
                static_cast<unsigned long long>(s.frames_dropped_on_post),
                static_cast<unsigned long long>(s.frames_dropped_on_send),
                static_cast<unsigned long long>(truncated_frames_.load()));

  serial_.post_frame(encoder_.encode_debug_text(std::string(line)));
}

}  // namespace communication
}  // namespace br_perception

// 注册为可加载组件 —— 容器化时直接挂进容器 C, 不必改代码
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::communication::CommNode)
