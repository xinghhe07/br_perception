#include "br_perception/communication/protocol_encoder.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace br_perception {
namespace communication {

namespace {

constexpr double kPi = 3.14159265358979323846;

// ═══════════════════════════════════════════════════════════════════════════
// 字节序列化 (一律小端)
// ═══════════════════════════════════════════════════════════════════════════

void put_u8(std::vector<std::uint8_t>& out, std::uint8_t v)
{
  out.push_back(v);
}

void put_i8(std::vector<std::uint8_t>& out, std::int8_t v)
{
  put_u8(out, static_cast<std::uint8_t>(v));
}

void put_u16(std::vector<std::uint8_t>& out, std::uint16_t v)
{
  out.push_back(static_cast<std::uint8_t>(v & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFFu));
}

void put_i16(std::vector<std::uint8_t>& out, std::int16_t v)
{
  put_u16(out, static_cast<std::uint16_t>(v));
}

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t v)
{
  out.push_back(static_cast<std::uint8_t>(v & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xFFu));
  out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xFFu));
}

// ═══════════════════════════════════════════════════════════════════════════
// 数值转换: 四舍五入 → 夹到目标类型范围
//
// ⚠ **必须夹**, 不是可选的保险:
//   - 有符号溢出是 **UB** (不是"回绕"), 编译器可以据此做任意优化
//   - 无符号窄化会静默截断, 产生一个"看起来正常"的荒谬值
//   非有限值 (NaN/Inf) 一律落 0 —— 脏数据不该变成随机数发出去。
// ═══════════════════════════════════════════════════════════════════════════

std::int16_t to_i16(double v)
{
  if (!std::isfinite(v)) {
    return 0;
  }
  const double r = std::round(v);
  if (r <= -32768.0) {
    return -32768;
  }
  if (r >= 32767.0) {
    return 32767;
  }
  return static_cast<std::int16_t>(r);
}

std::int8_t to_i8(double v)
{
  if (!std::isfinite(v)) {
    return 0;
  }
  const double r = std::round(v);
  if (r <= -128.0) {
    return -128;
  }
  if (r >= 127.0) {
    return 127;
  }
  return static_cast<std::int8_t>(r);
}

std::uint8_t to_u8(double v)
{
  if (!std::isfinite(v) || v <= 0.0) {
    return 0;
  }
  const double r = std::round(v);
  if (r >= 255.0) {
    return 255;
  }
  return static_cast<std::uint8_t>(r);
}

/// 米 → 厘米
std::int16_t m_to_cm_i16(double m) { return to_i16(m * 100.0); }
std::int8_t m_to_cm_i8(double m) { return to_i8(m * 100.0); }
std::uint8_t m_to_cm_u8(double m) { return to_u8(m * 100.0); }

/// m/s → cm/s。
/// ⚠ int8 量程只有 ±1.27 m/s —— 赛场够用, 但将来要描述快速目标时这个字段类型得改。
std::int8_t mps_to_cmps_i8(double mps) { return to_i8(mps * 100.0); }

/// 弧度 → 度×10 (协议规定 0.1° 精度)
double rad_to_deg10(double rad) { return rad * (180.0 / kPi) * 10.0; }

/// ROS 时间戳 → 毫秒 (uint32, 约 49 天回绕)
std::uint32_t stamp_to_ms(const builtin_interfaces::msg::Time& stamp)
{
  const std::int64_t ms = static_cast<std::int64_t>(stamp.sec) * 1000
                        + static_cast<std::int64_t>(stamp.nanosec / 1000000u);
  if (ms < 0) {
    // 负时间戳是上游 bug。夹到 0, 而不是取低 32 位 —— 后者会静默变成
    // ~0xFFFFFFFF, 即"遥不可及的未来", 主控拿到只会更懵。
    return 0;
  }
  return static_cast<std::uint32_t>(ms);   // 取低 32 位, 约 49 天回绕
}

// ═══════════════════════════════════════════════════════════════════════════
// CRC16 位反射
// ═══════════════════════════════════════════════════════════════════════════

std::uint8_t reflect8(std::uint8_t v)
{
  v = static_cast<std::uint8_t>(((v & 0xF0u) >> 4) | ((v & 0x0Fu) << 4));
  v = static_cast<std::uint8_t>(((v & 0xCCu) >> 2) | ((v & 0x33u) << 2));
  v = static_cast<std::uint8_t>(((v & 0xAAu) >> 1) | ((v & 0x55u) << 1));
  return v;
}

std::uint16_t reflect16(std::uint16_t v)
{
  std::uint16_t r = 0;
  for (int i = 0; i < 16; ++i) {
    r = static_cast<std::uint16_t>((r << 1) | ((v >> i) & 1u));
  }
  return r;
}

// ═══════════════════════════════════════════════════════════════════════════
// 封帧: 帧头 + 序号 + 类型 + 长度 + payload + CRC16
// ═══════════════════════════════════════════════════════════════════════════

/// 长度字段是 uint16, 而两个容量上限都是 uint8 → 理论最坏 payload 也只有约 5.6KB,
/// 结构性安全。这一行把结论钉在**编译期**: 将来谁改 config 改爆了, 编译就失败。
static_assert(ProtocolConfig::kFixedPayloadBytes
                  + (1 + ProtocolConfig::kBytesPerSpot * 255)
                  + (1 + ProtocolConfig::kBytesPerObstacle * 255)
              <= 0xFFFFu,
              "payload 上界已超 uint16 长度字段, 必须把长度字段升级为 uint32");

/// ⚠ CRC **覆盖范围** = 从帧头第 1 字节到 payload 末尾 (即除 CRC 自身以外的整帧)。
///   这也是一条需要与下位机确认的约定 —— 另一种常见做法是只校验 payload。
std::vector<std::uint8_t> wrap_frame(PacketType type,
                                     const std::vector<std::uint8_t>& payload,
                                     std::uint8_t seq,
                                     const Crc16& crc_calc)
{
  // 运行时兜底。长度窄化是**静默错值** (下位机按错长度切帧), 宁可返回空帧让调用方
  // 立刻发现。按上面的 static_assert, 这条实际不可达。
  if (payload.size() > 0xFFFFu) {
    return {};
  }

  std::vector<std::uint8_t> out;
  out.reserve(kFrameOverheadBytes + payload.size() + kCrcBytes);

  put_u8(out, kFrameHeader0);
  put_u8(out, kFrameHeader1);
  put_u8(out, seq);
  put_u8(out, static_cast<std::uint8_t>(type));
  put_u16(out, static_cast<std::uint16_t>(payload.size()));
  out.insert(out.end(), payload.begin(), payload.end());

  put_u16(out, crc_calc.compute(out.data(), out.size()));
  return out;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Crc16
// ═══════════════════════════════════════════════════════════════════════════

Crc16::Crc16(const ProtocolConfig& config)
  : config_(config)
{
  // table[b] = 把字节 b 单独过一个 8 位移位循环 (MSB-first, 用**原始**多项式)。
  //
  // ⚠ 这里**不能**反射 byte。输入反射必须施加在 compute() 里进入索引的那个**数据字节**上。
  //   把反射挪到建表阶段会让 reflect_in 分支**整条算错** (MODBUS 得 2BDF 而非 4B37),
  //   而非反射分支照样正确 —— 单测只测 CCITT-FALSE/XMODEM 根本发现不了。
  //   标准验证向量见 ProtocolConfig 的注释。
  for (std::uint32_t i = 0; i < 256; ++i) {
    std::uint16_t crc = static_cast<std::uint16_t>(static_cast<std::uint16_t>(i) << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u)
                ? static_cast<std::uint16_t>((crc << 1) ^ config_.crc16_polynomial)
                : static_cast<std::uint16_t>(crc << 1);
    }
    table_[i] = crc;
  }
}

std::uint16_t Crc16::compute(const std::uint8_t* data, std::size_t len) const
{
  std::uint16_t crc = config_.crc16_initial;
  for (std::size_t i = 0; i < len; ++i) {
    // 输入反射作用在**数据字节**上 (而不是表里) —— 这样同一张表对两种模式通用
    std::uint8_t byte = data[i];
    if (config_.crc16_reflect_in) {
      byte = reflect8(byte);
    }
    const std::uint8_t index =
        static_cast<std::uint8_t>((crc >> 8) ^ static_cast<std::uint16_t>(byte));
    crc = static_cast<std::uint16_t>((crc << 8) ^ table_[index]);
  }
  if (config_.crc16_reflect_out) {
    crc = reflect16(crc);
  }
  return static_cast<std::uint16_t>(crc ^ config_.crc16_xor_out);
}

// ═══════════════════════════════════════════════════════════════════════════
// ProtocolEncoder
// ═══════════════════════════════════════════════════════════════════════════

ProtocolEncoder::ProtocolEncoder(const ProtocolConfig& config)
  : config_(config)
  , crc_(config)
{
}

/// @brief 把 `PerceptionFrame` 的四个有效位/状态位打包成 `0x01` 帧的状态标志字节
///
/// ⚠️ 位定义**只有一个事实源**：`protocol_encoder.hpp` 的 `StatusFlag`。
///    这里只做打包，不重新定义任何编号。
/// ⚠️ `bit4..bit7` 保留位**必须置 0**（接收端要忽略它们；置了 1 的将来可能被赋含义）。
std::uint8_t build_status_flags(const br_perception::msg::PerceptionFrame& frame)
{
  std::uint8_t f = 0;
  if (frame.odometry_reliable) f |= kFlagOdometryReliable;
  if (frame.pose_valid)        f |= kFlagPoseValid;
  if (frame.sanctuary_valid)   f |= kFlagSanctuaryValid;
  if (frame.held_count_valid)  f |= kFlagHeldCountValid;
  return f;   // 保留位已保证为 0（上面只置了 4 个已定义的位）
}

EncodeResult ProtocolEncoder::encode(const br_perception::msg::PerceptionFrame& frame)
{
  std::vector<std::uint8_t> payload;
  payload.reserve(ProtocolConfig::max_payload_bytes(config_));

  // ── 状态标志字节要用的两位有效位（2026-09-22 新增）──
  const bool pose_ok = frame.pose_valid;
  const bool held_ok = frame.held_count_valid;

  // ⚠️⚠️ **`pose_valid == false` 时，所有位置字段一律写 0。**
  //
  //    这不是"省字节"，而是**保证过期/无效的位置一个都漏不出去**：
  //    写 0 + valid=0，主控按 valid 位忽略；而"主控忘了看 valid 位"时，
  //    它拿到的也是 0 而不是一个**看起来合理**的旧位置。
  //
  //    ⚠️ 这条不变式刻意放在**编码器**里而不是调用方：放调用方，
  //       迟早会漏掉某一个字段（位置字段散在头部/穆斯蒂卡/建筑位/障碍物四处）。
  auto pose_i16 = [pose_ok](double meters) -> std::int16_t {
    return pose_ok ? m_to_cm_i16(meters) : 0;
  };
  auto pose_i8 = [pose_ok](double meters) -> std::int8_t {
    return pose_ok ? m_to_cm_i8(meters) : 0;
  };
  auto pose_u8 = [pose_ok](double meters) -> std::uint8_t {
    return pose_ok ? m_to_cm_u8(meters) : 0;
  };

  // ── 头部信息 (16B) ──
  put_u32(payload, stamp_to_ms(frame.header.stamp));
  put_i16(payload, pose_i16(frame.ego_pose.x));
  put_i16(payload, pose_i16(frame.ego_pose.y));
  // ⚠ self_pose_z_cm 暂时恒为 0: PerceptionFrame.ego_pose 是 Pose2D, 没有 z。
  //    协议要求它 (机器人在 L1=0.6 / L2=0.9 的哪一层)。
  //    TODO: 把 ego_pose 换成 geometry_msgs/PoseStamped 后接上 .pose.position.z
  put_i16(payload, 0);
  put_i16(payload, to_i16(rad_to_deg10(frame.ego_pose.theta)));
  put_u8(payload, frame.match_remaining_sec);
  put_u8(payload, held_ok ? frame.held_object_count : 0u);
  // ⚠️ 持握数无效时"是否超限"同样无意义（它是那个数的函数）
  put_u8(payload, held_ok ? frame.holding_over_limit : 0u);
  put_u8(payload, frame.match_end_release_warning);

  // ── 穆斯蒂卡状态 (8B) ──
  put_u8(payload, frame.mustika_state);
  // ⚠️ 无效时写 0；主控**必须**按标志字节里的 valid 位忽略它
  put_u8(payload, frame.sanctuary_valid ? frame.sanctuary_mandate_fulfilled : 0u);
  // ⚠️ 状态标志字节 —— 2026-09-22 起**替代原先那个 0/1 的 `odometry_reliable`**。
  //    ⚠️ 长度没变（还是一个字节），故帧大小与 CRC 覆盖都不受影响。
  //    ⚠️ 这是**对下位机的破坏性改动**，未确认前不要在实机上启用 —— 见 `StatusFlag`。
  put_u8(payload, build_status_flags(frame));
  put_i16(payload, pose_i16(frame.mustika_pose.pose.position.x));
  put_i16(payload, pose_i16(frame.mustika_pose.pose.position.y));
  put_i8(payload, pose_i8(frame.mustika_pose.pose.position.z));

  // ── 柱子状态 (2B) ──
  put_u8(payload, frame.mustika_pillar.has_object_on_top ? 1u : 0u);
  put_u8(payload, frame.core_pillar.has_object_on_top ? 1u : 0u);

  // ── 建筑位状态 (1 + 4N, N ≤ max_building_spots) ──
  const std::size_t spot_total = frame.building_spots.size();
  const std::size_t spot_cap = config_.max_building_spots;
  const std::size_t spot_n = spot_total < spot_cap ? spot_total : spot_cap;

  put_u8(payload, static_cast<std::uint8_t>(spot_n));
  for (std::size_t i = 0; i < spot_n; ++i) {
    const auto& spot = frame.building_spots[i];
    put_u8(payload, to_u8(static_cast<double>(spot.spot_id)));
    put_u8(payload, spot.state);
    put_u8(payload, spot.top_color);
    put_u8(payload, spot.ownership);
  }

  // ── 附近障碍物 (1 + 18M, M ≤ max_obstacles) ──
  const std::size_t obs_total = frame.obstacles.size();
  const std::size_t obs_cap = config_.max_obstacles;
  const std::size_t obs_n = obs_total < obs_cap ? obs_total : obs_cap;

  put_u8(payload, static_cast<std::uint8_t>(obs_n));
  for (std::size_t i = 0; i < obs_n; ++i) {
    const auto& obs = frame.obstacles[i];
    put_u8(payload, to_u8(static_cast<double>(obs.id)));
    // ⚠️ 位置/尺寸都走 `pose_*` —— `pose_valid == false` 时自动写 0（见上方说明）
    put_i16(payload, pose_i16(obs.position.x));
    put_i16(payload, pose_i16(obs.position.y));
    put_i8(payload, pose_i8(obs.position.z));
    // bbox_size 语义是 (长/宽/高) = (x/y/z), 协议用 (width/depth/height) —— 按 x→宽, y→深 对应
    put_u8(payload, pose_u8(obs.bbox_size.x));
    put_u8(payload, pose_u8(obs.bbox_size.y));
    put_u8(payload, pose_u8(obs.bbox_size.z));
    put_u8(payload, obs.type);

    // ⚠ 以下 5 个字段协议要求、但 Obstacle.msg 目前**没有**, 暂时恒 0。
    //    它们是**融合层**的产物: 停留时长/威胁等级/己方 TR 识别都要跨帧跟踪才拿得到,
    //    不属于雷达的单帧检测输出。补齐方式见 doc/模块进度.md 第六节。
    put_u8(payload, 0);       // is_ally_tr
    put_u16(payload, 0);      // obs_dwell_ms
    put_u8(payload, 0);       // suspected_pushing
    put_u8(payload, 0);       // obs_held

    put_i8(payload, mps_to_cmps_i8(obs.velocity.x));   // obs_vx_cm_s
    put_i8(payload, mps_to_cmps_i8(obs.velocity.y));   // obs_vy_cm_s
    put_u8(payload, 0);       // threat_level
  }

  EncodeResult result;
  result.spots_written = static_cast<std::uint8_t>(spot_n);
  result.obstacles_written = static_cast<std::uint8_t>(obs_n);
  result.bytes = wrap_frame(PacketType::PERCEPTION, payload, next_seq(), crc_);
  return result;
}

std::vector<std::uint8_t> ProtocolEncoder::encode_heartbeat()
{
  return wrap_frame(PacketType::HEARTBEAT, {}, next_seq(), crc_);
}

std::vector<std::uint8_t> ProtocolEncoder::encode_estop(
    bool active, const builtin_interfaces::msg::Time& stamp)
{
  std::vector<std::uint8_t> payload;
  payload.reserve(ProtocolConfig::kEstopPayloadBytes);

  // ⚠️ 字段顺序与 `0x01` 保持一致：**时间戳在前**。下位机解析两类包时
  //    前 4 字节的读法相同，少一处容易写错的地方。
  put_u32(payload, stamp_to_ms(stamp));
  put_u8(payload, active ? static_cast<std::uint8_t>(kEstopActive)
                         : static_cast<std::uint8_t>(kEstopClear));

  // ⚠️ 不截断、不排队 —— 调用方必须让它**立刻上线**（安全信号，晚了等于没发）
  return wrap_frame(PacketType::ESTOP, payload, next_seq(), crc_);
}

std::vector<std::uint8_t> ProtocolEncoder::encode_debug_text(const std::string& text)
{
  std::vector<std::uint8_t> payload(text.begin(), text.end());
  const std::size_t cap = ProtocolConfig::max_payload_bytes(config_);
  if (payload.size() > cap) {
    payload.resize(cap);      // 截断而不是丢弃: 帧结构必须完整, 日志内容可以让步
  }
  return wrap_frame(PacketType::DEBUG, payload, next_seq(), crc_);
}

}  // namespace communication
}  // namespace br_perception
