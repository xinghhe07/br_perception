#pragma once

/// ═══════════════════════════════════════════════════════════════════════════
/// serial_manager.hpp — 串口 I/O + 收发线程 + 发送队列 (任务书 §5.3)
///
/// 平台: **Linux / POSIX termios**。Windows 下编不过 —— 这是刻意的, 目标平台是 Ubuntu。
///
/// ══ 两条职责边界 (都写死在这里, 免得后来人"顺手"再实现一遍) ══
///
///   1. **拼帧交给 ProtocolDecoder, 本类不自己拆包。**
///      任务书 §5.3 与本类职责里都写了"拼帧", §5.2 又把它列给 protocol_decoder ——
///      同一件事在两处出现。本仓库吃过"同一套代码两份"的亏, 所以只实现一处:
///      本类内部持有 ProtocolDecoder 实例, 解出的帧经回调交给上层。
///
///   2. **心跳字节由上层用回调提供, 本类不认识协议内容。**
///      生成心跳要帧序号 (protocol_encoder 的可变状态), 本类不该持有它。
///      本类只负责"到点了就调一下 provider, 把它给的字节发出去"。
///
/// ══ 线程模型 (三条线程) ══
///   · 调用方线程: post_frame() 投递待发帧 (**非阻塞**)
///   · 发送线程:   按 send_rate_hz 轮询, 取**最新**一帧发出 + 到点发心跳
///   · 接收线程:   read() → ProtocolDecoder::feed() → 回调
///   两个回调都在**工作线程**上执行, 实现不应阻塞、**不应抛异常** (见下)。
///   set_* 系列可随时调用 (内部加锁) —— 先 start() 再装配回调是合法用法。
///
/// ══ 实时性取向 (任务书 §5.3 明确要求) ══
///   发送队列**最多留 2 帧**, 且发送时**只取最新一帧, 中间的直接丢**。
///   理由: 串口拥塞时, 一帧 200ms 前的感知结果比没有更糟 —— 主控会照着过期
///   的世界模型动作。丢弃分两处发生, 分别计入两个计数器 (见 SerialStats),
///   因为"上层投太快"和"串口发不动"是两种故障, 修法不同。
/// ═══════════════════════════════════════════════════════════════════════════

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "br_perception/communication/protocol_decoder.hpp"

namespace br_perception {
namespace communication {

/// @brief 串口参数。
///
/// ⚠ **波特率等物理层参数仍是未冻结项** (没有下位机可确认)。改这里就够了, 不必动 .cpp。
struct SerialConfig {
  std::string device{"/dev/ttyUSB0"};

  // ── 物理层 (⚠ 待与下位机确认) ──
  // ✅ 2026-09-20: 115200 → 460800, 与 config/comm_params.yaml 保持一致
  //   (本文件头警告: 两处不一致会导致"传/不传 --params-file 行为不同")
  //   依据: 感知帧 280B, 8N1 → 115200 传输 24.3ms / 460800 传输 6.1ms;
  //        端到端 68-76ms → 50-58ms, 落进指标 55-60ms 内。
  //   termios 映射 (to_speed) 已支持 B460800, 无需另改。
  int baud_rate{460800};
  int data_bits{8};              ///< 5..8
  int stop_bits{1};              ///< 1 或 2
  char parity{'N'};              ///< 'N' 无 / 'E' 偶 / 'O' 奇
  bool flow_control{false};      ///< true = 启用 RTS/CTS 硬件流控

  // ── 协议参数 ──
  /// ⚠ **收发两侧必须用同一份**。非默认时 (例如协议冻结后改成 XMODEM 多项式),
  ///   encoder 与 decoder 都要拿到它 —— 否则症状是"串口正常打开、connected() 为 true、
  ///   却一帧都收不到", 与"链路断了"完全一样, 极难定位。
  ///   本类内部按它构造 ProtocolDecoder。
  ProtocolConfig protocol{};

  /// 连续 CRC 错误达到这个数就置 crc_alarm (任务书 §5.3 给的 10)。
  /// 放在 SerialConfig 而不是 ProtocolConfig: 它是**运行参数**, 不是线格式的一部分。
  int crc_alarm_threshold{10};

  // ── 时序 ──
  /// 发送线程轮询频率 [Hz]。**必须 > 0** —— 0 会除零, 负值会让 wait_for 立刻返回
  /// 变成忙等 (CPU 100%)。构造时会夹到 [1, 1000] (1000 以上串口本身发不动, 无意义)。
  int send_rate_hz{50};

  /// 心跳间隔 [ms]; ≤0 = 不发心跳。
  /// ⚠ **精度受 send_rate_hz 限制**: 发送线程每 1000/send_rate_hz ms 才醒一次,
  ///   所以心跳时刻会在这个周期内抖动, 且**不可能快于该周期**。
  ///   例如 send_rate_hz=50 时最短心跳 20ms —— 设成 5ms 只会得到 20ms 的实际间隔。
  int heartbeat_interval_ms{1000};

  int reconnect_interval_ms{1000};   ///< 断线后重试间隔 (任务书 §5.3: 1s)

  /// 接收线程单次 poll() 的等待上限 [ms]。**必须 ≥ 1** —— 0 或负值会让 poll
  /// 立刻超时返回, 接收线程退化成忙等 (CPU 100%)。构造时会夹到 ≥1。
  /// 链路上没有下行数据时这个超时是**正常路径** (不是错误): 醒来后 close 掉 dup 的
  /// fd 再重取, 顺带让断线能被及时察觉。
  int receive_poll_timeout_ms{100};
};

/// @brief 收发统计。纯数据 —— 派生量 (如频率) 由调用方自己按 uptime_ms 算。
struct SerialStats {
  std::uint64_t bytes_sent{0};
  std::uint64_t bytes_received{0};
  std::uint64_t frames_sent{0};        ///< 真正写进串口的感知帧数
  std::uint64_t heartbeats_sent{0};

  /// 投递时队列已满 (2 帧) 而挤掉的旧帧 —— 高 = **上层投递过快**
  std::uint64_t frames_dropped_on_post{0};
  /// 发送时为了只发最新帧而跳过的中间帧 —— 高 = **发送速率跟不上投递速率**
  std::uint64_t frames_dropped_on_send{0};

  std::uint64_t frames_received{0};    ///< 解出的下行帧数
  std::uint64_t crc_errors{0};         ///< 累计 CRC 错误 (来自 ProtocolDecoder)
  std::uint64_t callback_errors{0};    ///< 上层回调抛异常的次数 (见 FrameCallback)
  std::uint64_t write_errors{0};       ///< 写失败次数

  /// 因序号不新被丢弃的下行帧 (迟到的重传 / 重复帧)。
  /// ⚠ **这些帧既不会进 callback, 也不计入 frames_received** —— 症状是
  ///   "bytes_received 在涨、frames_received 不动、crc_errors 为 0"。
  ///   没有这一项时那种丢弃是完全静默的, 只能靠 bytes/frames 对不上来猜。
  /// ⚠ 主控重启后从 0 重新编号也会落到这里 (见 ProtocolDecoder::reset 的说明)。
  std::uint64_t stale_frames{0};
  /// 上述 stale 帧里序号跨度有歧义的部分 (差值落在 [128,255])
  std::uint64_t ambiguous_seq_frames{0};
  /// 判定"主控重新编号"并自动恢复的次数 (见 ProtocolDecoder::renumber_recoveries)
  std::uint64_t renumber_recoveries{0};

  /// 重连尝试次数。与下一项一起看才有意义:
  ///   attempts 高 + succeeded 低 → 硬件/驱动问题 (设备根本没回来)
  ///   attempts 高 + succeeded 也高 → 链路不稳定 (握手/线缆/供电)
  /// 合成一个"重连次数"会让这两种故障看起来一样, 所以拆开。
  std::uint64_t reconnect_attempts{0};
  std::uint64_t reconnects_succeeded{0};

  /// 接收缓冲区里还没成帧的字节数 (接收链路是否卡住的直接指标)
  std::uint64_t decoder_pending_bytes{0};

  /// start() 成功至今的毫秒数。频率这类派生量请用它自己算 ——
  /// Stats 里放一个"算好的频率"会引入"这是哪一刻算的"的歧义。
  std::uint64_t uptime_ms{0};
};

/// @brief 串口管理器: 收发线程 + 最新帧优先的发送队列 + 断线重连。
class SerialManager
{
public:
  explicit SerialManager(const SerialConfig& config = SerialConfig());
  ~SerialManager();

  SerialManager(const SerialManager&) = delete;
  SerialManager& operator=(const SerialManager&) = delete;
  SerialManager(SerialManager&&) = delete;
  SerialManager& operator=(SerialManager&&) = delete;

  /// @brief 打开串口并启动收发线程。
  /// @return false = **首次**打开失败。此时不启动任何线程, 调用方自行决定:
  ///         修好设备后重新 start(), 或者放弃。
  ///         一旦成功返回, 后续断线由后台线程按 reconnect_interval_ms 自动重连。
  /// ⚠ 本类**不可重启**: stop() 之后再调 start() 是错误用法 (会 assert)。
  ///   要重启请构造新实例。
  bool start();

  /// @brief 停止收发线程并关闭串口。析构也会调。
  /// ⚠ **线程安全、可重复调用**: 多线程同时调会串行执行, 第一个做实事, 其余立即返回。
  /// ⚠ stop() 之后**不能再次 start()** (见上)。
  void stop();

  /// 是否已停止
  bool stopped() const noexcept { return !running_.load(); }

  /// 串口 fd 当前是否打开。**不代表链路可用** —— USB 适配器插着但对面没通电时它仍为 true。
  bool device_open() const noexcept { return fd_.load() >= 0; }

  /// 链路是否判定为可用: 最后一次 I/O 成功, 且未触发重连。
  /// 打开 fd 不等于可用, 所以诊断请按需在 device_open() 与本函数之间挑。
  ///
  /// ⚠ **重连期间会短暂抖动为 false** —— 发送线程 close+reopen 时, 接收线程可能
  ///   正好 poll 到一个刚被关掉的 fd 而把 connected 置回 false。这是正常的过渡态,
  ///   诊断代码不该依赖它的瞬时值; 要看链路健康与否请用
  ///   stats().reconnect_attempts / reconnects_succeeded。
  bool connected() const noexcept { return connected_.load(); }

  // ── 发送侧 ──

  /// @brief 投递一帧待发字节。**非阻塞**, 感知线程可直接调用。
  ///
  /// **会唤醒发送线程** —— 从投递到发出的延迟在毫秒级, 不受 send_rate_hz 轮询周期约束。
  /// 队列已满 (2 帧) → 丢弃最旧的一帧, 计入 frames_dropped_on_post。
  void post_frame(std::vector<std::uint8_t> bytes);

  /// @brief 心跳字节的提供者。发送线程每隔 heartbeat_interval_ms 调一次。
  /// 返回空 vector = 这次不发。
  /// ⚠ 在**发送线程**上执行, 不应阻塞、**不应抛异常**。
  /// ⚠ 可随时调用 (内部加锁); 但 callable 捕获的状态需自己保证线程安全。
  using HeartbeatProvider = std::function<std::vector<std::uint8_t>()>;
  void set_heartbeat_provider(HeartbeatProvider provider);

  // ── 接收侧 ──

  /// @brief 解出帧的回调。在**接收线程**上执行。
  /// ⚠ **实现不应阻塞、不应抛异常** —— 抛出会被本类捕获并计入
  ///   `Stats::callback_errors` (否则接收线程会直接死掉, 而 connected() 仍为 true,
  ///   症状是"一切正常但永远收不到新帧")。
  using FrameCallback = std::function<void(const DecodedFrame&)>;
  void set_frame_callback(FrameCallback callback);

  // ── 诊断 ──

  /// 取一份统计快照。会加锁复制数据, 故**不是 noexcept** (与只读原子量的那几个不同)。
  SerialStats stats() const;

  /// @brief 连续 CRC 错误是否已达告警线 (任务书 §5.3: 连续 10 次 → 告警, 疑硬件干扰)。
  /// 一旦置位就**保持**, 直到 reset_crc_alarm() —— 硬件干扰的特征是时好时坏,
  /// 告警跟着抖就没用了。
  bool crc_alarm() const noexcept { return crc_alarm_.load(); }
  void reset_crc_alarm() noexcept { crc_alarm_.store(false); }

  /// 当前发送队列里积压的帧数。加锁取值, 故不是 noexcept。
  std::size_t pending_frames() const;

private:
  /// 夹取非法配置 (send_rate_hz <= 0 等), 构造时用
  static SerialConfig sanitize(SerialConfig config);

  /// @brief 尝试打开串口**一次**。成功 → device_open/connected 置位并返回 true;
  ///        失败 → 返回 false, **不重试、不睡眠**。
  ///        断线重连由 send_loop / receive_loop 在检测到 I/O 失败后按
  ///        reconnect_interval_ms 自行负责。
  bool open_device();
  void close_device();

  /// 重连: 关闭旧 fd, 睡眠 reconnect_interval_ms, 再试一次。成功返回 true
  bool reconnect();

  void send_loop();
  void receive_loop();

  const SerialConfig config_;

  // ── 通道 ──
  std::atomic<int> fd_{-1};
  std::mutex fd_mutex_;              ///< 串行化 open/close 的换手

  // ── 线程 ──
  /// 串行化 start()/stop()。没有它, 两个线程同时 stop() 会对同一个 std::thread
  /// 并发 join() —— 那是 UB, 不是"其中一个白跑一次"。
  std::mutex lifecycle_mutex_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<bool> ever_started_{false};
  std::thread send_thread_;
  std::thread receive_thread_;

  // ── 发送队列 (显式锁, 不追求无锁 —— 发送线程 50Hz, 竞争极低) ──
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<std::vector<std::uint8_t>> queue_;
  static constexpr std::size_t kMaxQueuedFrames = 2;   ///< 任务书 §5.3 明确给的 2

  // ── 回调 (加锁保护: 允许 start() 之后再装配) ──
  mutable std::mutex callback_mutex_;
  HeartbeatProvider heartbeat_provider_;
  FrameCallback frame_callback_;

  // ── 接收 ──
  ProtocolDecoder decoder_;

  /// 链路世代号: 每次**成功重连** +1。
  ///
  /// ⚠ 发送线程 (reconnect 的执行者) 只 bump 这个原子量, **绝不直接碰 decoder_** ——
  ///   decoder_ 的唯一所有者是接收线程 (见 receive_loop 里同步统计的注释),
  ///   跨线程调它的 reset() 是数据竞争, 不是"提前清一下"。
  ///   接收线程发现世代号变了, 自己去清解码器的链路状态。
  std::atomic<std::uint64_t> link_generation_{0};

  // ── 统计 (由各自线程写, 读时加锁取快照) ──
  mutable std::mutex stats_mutex_;
  SerialStats stats_;
  std::uint64_t crc_errors_since_last_good_{0};   ///< 好帧到达即清零
  std::chrono::steady_clock::time_point start_time_{};

  std::atomic<bool> crc_alarm_{false};
};

}  // namespace communication
}  // namespace br_perception
