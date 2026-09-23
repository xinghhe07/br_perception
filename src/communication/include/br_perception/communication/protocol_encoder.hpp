#pragma once

/// ═══════════════════════════════════════════════════════════════════════════
/// protocol_encoder.hpp — 感知帧 → 紧凑二进制协议 (任务书 §5.1)
///
/// 依赖: rclcpp 消息类型 (br_perception/msg/PerceptionFrame) —— **不含任何节点/订阅**。
///       它不是 rclcpp::Node, 是"给一个消息、还一串字节"的纯函数式工具, 故可被
///       gtest 直接单测 (构造一个 PerceptionFrame 即可, 不需要 spin)。
///
/// 帧格式 (任务书 §5.1, 字节序一律**小端**):
///   0-1    帧头 0xAA 0x55
///   2      帧序号 (0-255 循环)
///   3      数据包类型 (见 PacketType)
///   4-5    数据长度 uint16 (不含帧头/长度/CRC 自身)
///   6..N   Payload
///   N+1..N+2 CRC16 (小端)
///
/// Payload 分组 (0x01 感知帧; 逐字段的权威表见任务书 §5.1):
///   头部信息 16B  timestamp_ms / self_pose(x,y,z_cm, yaw×10) / 比赛态 4 个 uint8
///   穆斯蒂卡  8B  status / sanctuary / odometry_reliable / (x,y_cm, z_cm)
///   柱子      2B  两根柱的 occupied 标志
///   建筑位  1+4N  计数 + 每项 4B (**按源消息字段名**: spot_id / state / top_color / ownership)
///   障碍物 1+18M  计数 + 每项 18B, M ≤ max_obstacles 超出截断
///           ⚠ 其中 is_ally_tr / obs_dwell_ms / suspected_pushing / obs_held / threat_level
///             五个字段 Obstacle.msg 没有, 暂写 0 —— 它们是融合层产物, 见 .cpp 注释
///
/// ⚠ **协议尚未冻结** —— 目前没有下位机团队可确认, 故 CRC16 做成**完全参数化**
///   (多项式/初值/反射/异或出口全可配), 默认取 CRC-16/CCITT-FALSE。
///   冻结后只需改 ProtocolConfig, 不必动本文件或 .cpp:
///     · 改用 CRC-16/MODBUS → {0x8005, 0xFFFF, reflect_in=true, reflect_out=true, xor_out=0}
///     · 改用 CRC-16/XMODEM → {0x1021, 0x0000, false, false, 0}
///
/// ⚠ **与容器化设计 §4 硬性规则的关系**: 该规则要求"算法类不碰 ROS"。本模块**刻意
///   不适用**该规则 —— 它的输入契约**就是** ROS 消息本身 (任务书 §5.1 白纸黑字),
///   再造一个镜像 struct 会多出一份会漂移的重复定义。它仍然满足规则的实质要求:
///   没有 Node、没有订阅发布、构造不接 rclcpp::Node*、行为完全由入参决定。
///
/// 线程安全: **可并发调用**。唯一的可变状态是帧序号, 用原子自增, 溢出自然回绕。
///           (并发调用时帧序号的**分配**是原子的, 但两帧的**发送顺序**由调用方保证。)
/// ═══════════════════════════════════════════════════════════════════════════

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "br_perception/msg/perception_frame.hpp"

namespace br_perception {
namespace communication {

/// 帧头两字节
constexpr std::uint8_t kFrameHeader0 = 0xAA;
constexpr std::uint8_t kFrameHeader1 = 0x55;

/// 帧头 + 序号 + 类型 + 长度 = 6 字节
constexpr std::size_t kFrameOverheadBytes = 6;
/// CRC16 占 2 字节
constexpr std::size_t kCrcBytes = 2;

/// 数据包类型 (任务书 §5.1)
enum class PacketType : std::uint8_t {
  PERCEPTION = 0x01,   ///< 感知帧
  HEARTBEAT  = 0x02,   ///< 心跳 (空 payload)
  /// @brief 紧急停止 / 失效上报。
  ///
  /// ⚠️ 2026-09-22: 原为 `ERROR = 0x03`，**一直是"未实现"**，而全仓库零引用
  ///    （只在本枚举里出现过），故直接收窄语义并实现 —— 不是新增一个包。
  ///
  /// ⚠️ 它解决的**具体问题**（务必别退化）：只发 ROS 话题的紧急停止，
  ///    **STM32 什么也收不到** —— `comm_node` 只订阅 `/perception/output` 一个话题。
  ///    更要命的是"感知算不出坐标"与"串口线断了"在主控侧**完全不可区分**，
  ///    而这两件事的应对**正相反**（一个该等感知，一个该急停失能）。
  ///    有了本包，主控能分清：
  ///      · 收到 `0x03 estop=1` → **链路活着，是感知侧要求停车**
  ///      · 什么都没收到         → **链路死了**
  ESTOP      = 0x03,
  DEBUG      = 0x7F,   ///< 调试字符串 (未实现)
};

/// @brief `0x03` ESTOP 包的 payload 取值
///
/// ⚠️ payload 刻意只有 **5 字节**（`uint32 timestamp_ms` + `uint8 estop`）：
///    每多一个字节就多一条与下位机的契约，而我们**没有**下位机可确认。
///    ⚠️ 特别是**故意不带"原因码"** —— 区分"算不出坐标"与"传感器全失"
///       靠的是"收到 0x03"对比"什么都没收到"（见 `PacketType::ESTOP` 的注释），
///       已经够用。细分原因留在 ROS 侧的 `/system/degradation_state` 给人看。
///       将来主控若确实需要原因码，那是**加字段 + 定编码**，要与下位机一起定。
enum EstopValue : std::uint8_t {
  kEstopClear  = 0,   ///< 解除：可以恢复行驶
  kEstopActive = 1,   ///< **要求停车**
};

/// @brief 协议参数。**协议冻结后, 改动只会发生在这里。**
///
/// ⚠ 改 CRC 参数后**必须**用标准验证向量验一遍。四条业界标准 check 值, 输入均为
///   ASCII 字符串 `"123456789"` (9 字节):
///     CCITT-FALSE {0x1021, 0xFFFF, false, false, 0x0000} → 0x29B1  ← 当前默认
///     XMODEM      {0x1021, 0x0000, false, false, 0x0000} → 0x31C3
///     MODBUS      {0x8005, 0xFFFF, true,  true,  0x0000} → 0x4B37
///     KERMIT      {0x1021, 0x0000, true,  true,  0x0000} → 0x2189
///   后两条走反射分支 —— **只测前两条会漏掉整条反射路径** (曾经真的漏过)。
/// @brief `0x01` 感知帧里**状态标志字节**的位定义
///
/// ⚠️ 2026-09-22: 这个字节**原先叫 `odometry_reliable`，是个 0/1 的 uint8**
///    —— 7 位白扔。现在当标志位用，**长度一个字节都没变** ⇒ 帧大小、CRC 覆盖、
///    所有"固定段 16 字节"的算式**全都不用改**。
///
/// ⚠️⚠️ **这是对下位机的破坏性改动**：原来读 bit0 当"里程计可靠"的实现仍然正确
///    （bit0 语义没动），但"整字节 0/1"的假设不再成立 —— 收到 0x05 时按 0/1 解读
///    会判成"真"，实际 pose_valid=1 而 odometry_reliable=0。
///    **必须与下位机确认后启用**（并入 `config/comm_params.yaml` 头部那份未冻结清单）。
///
/// 为什么需要这几个位（各自的来龙去脉见 `doc/模块进度.md`）：
///   · `kFlagPoseValid` —— 让"**位置不可用，但状态照发**"成为**合法状态**。
///     没有它，感知算不出 world 坐标时只能**整帧不发** ⇒ 主控连比赛计时都断了。
///   · `kFlagSanctuaryValid` / `kFlagHeldCountValid` —— 见下方 `0` 值的说明。
enum StatusFlag : std::uint8_t {
  kFlagOdometryReliable = 1u << 0,   ///< 里程计可靠（**原 `odometry_reliable` 的 0/1**）
  /// 位置类字段（self_pose_*、建筑位/障碍物坐标、mustika_pose）**有效**
  kFlagPoseValid        = 1u << 1,
  /// `sanctuary_mandate_fulfilled` 字段有效。**0 = 该字段无意义，主控必须忽略它**
  kFlagSanctuaryValid   = 1u << 2,
  /// `held_object_count` 字段有效。**0 = 该字段无意义，主控必须忽略它**
  kFlagHeldCountValid   = 1u << 3,
  // bit4..bit7 **保留**，发送端必须置 0；接收端必须忽略。
};

struct ProtocolConfig {
  // ── CRC16 (⚠ 待与下位机确认; 默认 CRC-16/CCITT-FALSE) ──
  std::uint16_t crc16_polynomial{0x1021};
  std::uint16_t crc16_initial{0xFFFF};
  std::uint16_t crc16_xor_out{0x0000};
  bool crc16_reflect_in{false};
  bool crc16_reflect_out{false};

  // ── 单帧容量上限 (任务书 §5.1 明确给出的两个 16/10) ──
  /// 建筑位最多 16 个, 超出**截断**(不丢整帧)
  std::uint8_t max_building_spots{16};
  /// 障碍物最多 10 个, 超出**截断**
  std::uint8_t max_obstacles{10};

  // ── payload 尺寸构成 (改 encode() 的字段就必须同步改这里) ──
  /// 固定段: 头部 16 + 穆斯蒂卡 8 + 柱子 2
  static constexpr std::size_t kFixedPayloadBytes = 16 + 8 + 2;
  static constexpr std::size_t kBytesPerSpot = 4;       ///< spot_id/state/top_color/ownership
  static constexpr std::size_t kBytesPerObstacle = 18;  ///< 见 .cpp 的逐字段写入

  /// `0x03` ESTOP 包的 payload 长度: `uint32 timestamp_ms` + `uint8 estop`
  /// ⚠️ 与 `0x01` 无关（不是 kFixedPayloadBytes 的一部分），单独给常量
  static constexpr std::size_t kEstopPayloadBytes = 4 + 1;

  /// payload 上界 [字节] —— **由本 config 的实际上限算出**。
  /// ⚠ 原先写成常量 `16 + 8 + 2 + (1+4*16) + (1+18*10)`，那两个 16/10 是**独立硬编码**:
  ///   有人把 max_building_spots 改成 24 时它不会跟着变 —— 缓冲区预留算小, 且不报错。
  static constexpr std::size_t max_payload_bytes(const ProtocolConfig& c) noexcept
  {
    return kFixedPayloadBytes
         + (1 + kBytesPerSpot * static_cast<std::size_t>(c.max_building_spots))
         + (1 + kBytesPerObstacle * static_cast<std::size_t>(c.max_obstacles));
  }
};

/// @brief 编码结果。
///
/// 截断情况**不靠日志**(本模块不碰 ROS, 没法打日志), 而是回给调用方:
/// `spots_written < frame.building_spots.size()` 即发生了截断, 由薄壳决定怎么告警。
struct EncodeResult {
  std::vector<std::uint8_t> bytes;      ///< 完整帧 (帧头 → CRC), 可直接送串口
  std::uint8_t spots_written{0};        ///< 实际写入的建筑位数
  std::uint8_t obstacles_written{0};    ///< 实际写入的障碍物数
};

/// @brief CRC16 查表计算器 —— 表在**构造时**按 ProtocolConfig 生成一次 (256 项)。
///
/// 做成类而不是自由函数, 两个理由:
///   1. 表只建一次。自由函数每次调用要么重建 256 项表, 要么退回逐位计算 (8 次/字节)
///   2. protocol_decoder 要持有一个**同参数**的实例 —— 收发两端必须逐位一致
/// ⚠ 构造之后再改 config 会导致表与参数不一致 (静默算错)。要换参数就重建对象。
class Crc16
{
public:
  explicit Crc16(const ProtocolConfig& config = ProtocolConfig());

  /// @param data 待校验数据; len == 0 时返回初始值经反射/异或出口处理后的结果
  std::uint16_t compute(const std::uint8_t* data, std::size_t len) const;

private:
  ProtocolConfig config_;
  std::array<std::uint16_t, 256> table_{};
};

/// @brief 感知帧编码器 (任务书 §5.1)
class ProtocolEncoder
{
public:
  explicit ProtocolEncoder(const ProtocolConfig& config = ProtocolConfig());

  /// @brief 把一帧感知结果编码为完整字节流 (帧头 → payload → CRC16)。
  ///
  /// 可并发调用。每次调用消耗一个帧序号 (0..255 循环)。
  /// ⚠ **帧序号在所有 PacketType 之间共享** —— 心跳也消耗序号。
  ///   本条待下位机确认; 若约定改成"每种类型独立编号", 需按 type 各维护一个计数器。
  ///
  /// ⚠️ **状态标志字节**（见 `StatusFlag`）：本函数把
  ///    `frame.odometry_reliable` / `frame.pose_valid` /
  ///    `frame.sanctuary_valid` / `frame.held_count_valid` 打包成一个字节。
  ///    ⚠️ **valid 位为 0 时对应字段照样写进字节流**（位置写 0 等），
  ///       主控**必须按 valid 位忽略它们** —— 见 `StatusFlag` 上方的破坏性改动说明。
  ///
  /// @return 字节流 + 实际写入的建筑位/障碍物数量 (用于判断是否截断)
  EncodeResult encode(const br_perception::msg::PerceptionFrame& frame);

  /// @brief 心跳帧 (类型 0x02, 空 payload)。同样消耗一个帧序号。
  std::vector<std::uint8_t> encode_heartbeat();

  /// @brief 紧急停止帧 (类型 0x03, payload = timestamp_ms + estop)。
  ///        同样消耗一个帧序号。
  ///
  /// @param active `true` = 要求主控停车；`false` = **解除**（能恢复行驶了）
  /// @param stamp  **由调用方给**的时刻 —— 编码器**不持有时钟**，也不该自己取：
  ///               本节点可能跑在 `use_sim_time` 下，自己取墙上时间会让
  ///               紧急帧与感知帧的时间戳**不同源**，现场核对时对不上。
  ///               统一由调用方（`comm_node`）决定用哪个时钟。
  ///
  /// ⚠️ **两个方向都要发** —— 只发"停车"不发"解除"，主控就永远停在那个状态。
  ///    （`comm_node` 那边是按**状态变化**触发的，不是周期性发。）
  ///
  /// ⚠️ 本包**不做截断/限频**：它是安全信号，晚了就等于没发。
  ///    调用方不要在它前面排队 —— 见 `comm_node` 里"紧急帧**绕过节流队列**"的处理。
  std::vector<std::uint8_t> encode_estop(bool active,
                                         const builtin_interfaces::msg::Time& stamp);

  /// @brief 调试帧 (类型 0x7F, payload = ASCII 原文), 供台架联调。
  ///
  /// ⚠ 下位机侧对 0x7F 的处理**没有规格** —— 比赛时不要开 (comm_node 的 debug_mode
  ///   默认就是 false)。这里仍然放在编码器里, 而不是让调用方自己拼帧:
  ///   帧格式只在一个地方实现, 多一处就多一处会漂移的副本。
  ///
  /// 超过 payload 上界时**截断** —— 半截日志的价值远低于"帧结构必须完整"。
  std::vector<std::uint8_t> encode_debug_text(const std::string& text);

  /// 下一个将被使用的帧序号 (诊断用)。
  /// ⚠ 多线程下这是**近似值** —— 两次 encode() 之间读到的值可能已被别的线程推进。
  std::uint8_t current_seq() const noexcept { return seq_.load(); }

  /// 当前协议参数
  const ProtocolConfig& config() const noexcept { return config_; }

private:
  /// 取下一个帧序号。原子自增; uint8 的**无符号**回绕 (255 → 0) 是定义良好的行为。
  ///
  /// ⚠ TODO(protocol-freeze): 序号空间是否跨 PacketType 共享, 待下位机确认。
  ///   当前实现为**共享** (心跳/调试帧也消耗同一个计数器)。若确认改为"每种类型
  ///   独立编号", 需按 type 各维护一个计数器, 并同步改 test_protocol_encoder.cpp
  ///   的 SequenceTest.心跳与感知帧共享序号空间。
  ///   查全部未冻结项: grep -rn "TODO(protocol-freeze)" src/
  std::uint8_t next_seq() noexcept { return seq_.fetch_add(1); }

  ProtocolConfig config_;
  Crc16 crc_;
  std::atomic<std::uint8_t> seq_{0};
};

}  // namespace communication
}  // namespace br_perception
