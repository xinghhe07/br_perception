/// ═══════════════════════════════════════════════════════════════════════════
/// test_protocol_encoder.cpp — 协议编码器单元测试 (任务书 §5.1 / §11.1)
///
/// ⚠ 本测试**不起 ROS 节点** —— 编码器不是 Node, 构造一个 PerceptionFrame
///   直接喂进去即可 (不需要 spin / 不需要话题)。
///
/// ══ CRC16 标准验证向量 (活文档) ══
/// 输入恒为 ASCII 字符串 `"123456789"` (9 字节), 四条业界通用 check 值:
///
///     CCITT-FALSE  {0x1021, 0xFFFF, false, false, 0x0000} → 0x29B1   ← 当前默认
///     XMODEM       {0x1021, 0x0000, false, false, 0x0000} → 0x31C3
///     MODBUS       {0x8005, 0xFFFF, true,  true,  0x0000} → 0x4B37
///     KERMIT       {0x1021, 0x0000, true,  true,  0x0000} → 0x2189
///
/// ⚠ **后两条必须测**。曾经真的漏过: 建表阶段错误地施加了输入反射, 结果
///   非反射的两条全对、反射的两条全错 (MODBUS 得 2BDF) —— 只测默认变体发现不了。
/// ═══════════════════════════════════════════════════════════════════════════

#include "br_perception/communication/protocol_encoder.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace br_perception {
namespace communication {
namespace {

constexpr double kPi = 3.14159265358979323846;

/// 空输入测试用 (避免给 compute 传 nullptr)
const std::uint8_t kDummy = 0;

// ═══════════════════════════════════════════════════════════════════════════
// Golden 帧 —— **与实现无关的外部验证**
//
// 下面这串 hex 由独立的**位逐位** Python 实现算出 (不查表, 与被测的 Crc16 无共享代码)。
// 生成脚本, 照抄即可复现 (Python 3, 无需第三方库):
//
//   import struct
//   def ccitt_false(data):
//       crc = 0xFFFF
//       for b in data:
//           crc ^= b << 8
//           for _ in range(8):
//               crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
//       return crc
//   payload = bytes.fromhex('39300000' '9600' '1FFF' '0000' '8403' '64010000'
//                           '01000F' '2602CF03' '36' '0100' '0000')
//   body = bytes([0xAA,0x55,0x00,0x01]) + struct.pack('<H', len(payload)) + payload
//   print((body + struct.pack('<H', ccitt_false(body))).hex().upper())
//
// ⚠ 改了 payload 布局**必须重跑这个脚本**更新下面的常量。**不许直接把实现的输出贴进来** ——
//   那样等于把外部验证退化成循环验证: 实现自己证明自己, 错了也照过。
//
// ⚠ 2026-09-22: payload 里 `01 00 0F` 的**第三个字节**原先是 `odometry_reliable`(0/1),
//   现在改叫**状态标志字节**, 四位置满 = 0x0F（见 protocol_encoder.hpp 的 `StatusFlag`）。
//   ⚠ **长度没变**（还是一个字节）—— 上面 `struct.pack('<H', len(payload))` 仍得 28。
// ═══════════════════════════════════════════════════════════════════════════
constexpr char kGoldenFrameHex[] =
    "AA5500011C003930000096001FFF000084036401000001000F2602CF033601000000A1A0";

/// 字节流 → 大写十六进制 (走 golden 比对用)
std::string to_hex(const std::vector<std::uint8_t>& bytes)
{
  static const char* kDigits = "0123456789ABCDEF";
  std::string out;
  out.reserve(bytes.size() * 2);
  for (const std::uint8_t b : bytes) {
    out.push_back(kDigits[b >> 4]);
    out.push_back(kDigits[b & 0x0F]);
  }
  return out;
}

/// 计算字符串的 CRC (测试辅助)
std::uint16_t crc_of(const ProtocolConfig& config, const std::string& data)
{
  const Crc16 calc(config);
  return calc.compute(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
}

// ── 四个变体的 config (字段全显式写出, 免得读的人去翻默认值) ──

ProtocolConfig cfg_ccitt_false()
{
  ProtocolConfig c;
  c.crc16_polynomial = 0x1021;
  c.crc16_initial = 0xFFFF;
  c.crc16_reflect_in = false;
  c.crc16_reflect_out = false;
  c.crc16_xor_out = 0x0000;
  return c;
}

ProtocolConfig cfg_xmodem()
{
  ProtocolConfig c;
  c.crc16_polynomial = 0x1021;
  c.crc16_initial = 0x0000;
  c.crc16_reflect_in = false;
  c.crc16_reflect_out = false;
  c.crc16_xor_out = 0x0000;
  return c;
}

ProtocolConfig cfg_modbus()
{
  ProtocolConfig c;
  c.crc16_polynomial = 0x8005;
  c.crc16_initial = 0xFFFF;
  c.crc16_reflect_in = true;
  c.crc16_reflect_out = true;
  c.crc16_xor_out = 0x0000;
  return c;
}

ProtocolConfig cfg_kermit()
{
  ProtocolConfig c;
  c.crc16_polynomial = 0x1021;
  c.crc16_initial = 0x0000;
  c.crc16_reflect_in = true;
  c.crc16_reflect_out = true;
  c.crc16_xor_out = 0x0000;
  return c;
}

// ── 构造测试帧 ──

br_perception::msg::PerceptionFrame make_frame()
{
  br_perception::msg::PerceptionFrame f;
  f.header.frame_id = "world";
  f.header.stamp.sec = 12;
  f.header.stamp.nanosec = 345000000;   // +345ms → 12345ms

  f.ego_pose.x = 1.5;                   // → 150 cm
  f.ego_pose.y = -2.25;                 // → -225 cm
  f.ego_pose.theta = kPi / 2.0;         // → 900 (90.0° ×10)

  f.match_remaining_sec = 100;
  f.held_object_count = 1;
  f.holding_over_limit = 0;
  f.match_end_release_warning = 0;

  f.mustika_state = 1;                  // 柱上
  f.sanctuary_mandate_fulfilled = 0;
  f.odometry_reliable = 1;
  // ⚠️ 2026-09-22 新增的三个**有效位**。编码器在它们为 false 时会把对应字段**清零**
  //    （不变式，防"过期位置漏给主控"），所以这个"字段齐全"的测试帧必须把它们置 1
  //    —— 否则下面 golden 帧里的坐标会全变 0。
  //    "无效时清零"这条单独测：见 `FrameLayoutTest.有效位为假时位置一律写零`。
  f.pose_valid = true;
  f.sanctuary_valid = true;
  f.held_count_valid = true;
  f.mustika_pose.pose.position.x = 5.5;    // 550
  f.mustika_pose.pose.position.y = 9.75;   // 975
  f.mustika_pose.pose.position.z = 0.54;   // 54

  f.mustika_pillar.has_object_on_top = true;
  f.core_pillar.has_object_on_top = false;
  return f;
}

br_perception::msg::PerceptionFrame make_frame_with_spots(std::size_t n)
{
  auto f = make_frame();
  for (std::size_t i = 0; i < n; ++i) {
    br_perception::msg::BuildingSpot s;
    s.spot_id = static_cast<std::uint32_t>(i);
    s.state = 3;       // FULL_TOWER
    s.top_color = 1;   // RED
    s.ownership = 1;   // OWN
    f.building_spots.push_back(s);
  }
  return f;
}

br_perception::msg::PerceptionFrame make_frame_with_obstacles(std::size_t n)
{
  auto f = make_frame();
  for (std::size_t i = 0; i < n; ++i) {
    br_perception::msg::Obstacle o;
    o.id = static_cast<std::uint32_t>(i);
    o.position.x = 1.0;
    o.position.y = 2.0;
    o.position.z = 0.0;
    o.bbox_size.x = 0.35;
    o.bbox_size.y = 0.35;
    o.bbox_size.z = 0.35;
    o.type = 2;        // DROPPED_BLOCK
    f.obstacles.push_back(o);
  }
  return f;
}

/// 取帧尾的 CRC16 (小端)
std::uint16_t trailing_crc(const std::vector<std::uint8_t>& bytes)
{
  const std::size_t n = bytes.size();
  return static_cast<std::uint16_t>(
      bytes[n - 2] | (static_cast<std::uint16_t>(bytes[n - 1]) << 8));
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// CRC16
// ═══════════════════════════════════════════════════════════════════════════

TEST(Crc16Test, 四条标准验证向量)
{
  const std::string check_input = "123456789";
  EXPECT_EQ(0x29B1, crc_of(cfg_ccitt_false(), check_input));
  EXPECT_EQ(0x31C3, crc_of(cfg_xmodem(), check_input));
  EXPECT_EQ(0x4B37, crc_of(cfg_modbus(), check_input));       // ← 走反射分支
  EXPECT_EQ(0x2189, crc_of(cfg_kermit(), check_input));       // ← 走反射分支
}

TEST(Crc16Test, 空输入四条变体)
{
  // 无数据 → 只剩初值经反射/异或出口处理后的结果。
  // 四条一起测, 顺带覆盖 **reflect_out 在零长度输入上的行为**。
  EXPECT_EQ(0xFFFF, Crc16(cfg_ccitt_false()).compute(&kDummy, 0));
  EXPECT_EQ(0x0000, Crc16(cfg_xmodem()).compute(&kDummy, 0));
  EXPECT_EQ(0xFFFF, Crc16(cfg_modbus()).compute(&kDummy, 0));   // reflect16(0xFFFF) 仍是全 1
  EXPECT_EQ(0x0000, Crc16(cfg_kermit()).compute(&kDummy, 0));   // reflect16(0) = 0
}

TEST(Crc16Test, 默认配置就是CCITTFALSE)
{
  // ProtocolConfig 的默认值若被改动, 这条会失败并提醒同步上面的向量表
  const std::string check_input = "123456789";
  EXPECT_EQ(0x29B1, crc_of(ProtocolConfig{}, check_input));
}

// ═══════════════════════════════════════════════════════════════════════════
// 封帧布局
// ═══════════════════════════════════════════════════════════════════════════

TEST(FrameLayoutTest, 逐字节校验最小帧)
{
  ProtocolEncoder enc;                       // 默认 config
  const EncodeResult r = enc.encode(make_frame());

  EXPECT_EQ(0xAA, r.bytes[0]);
  EXPECT_EQ(0x55, r.bytes[1]);
  EXPECT_EQ(0x00, r.bytes[2]);               // 首帧序号
  EXPECT_EQ(0x01, r.bytes[3]);               // PacketType::PERCEPTION
  EXPECT_EQ(0x1C, r.bytes[4]);               // 长度 28, 小端低字节
  EXPECT_EQ(0x00, r.bytes[5]);

  // payload —— 期望值由独立的 Python 实现算出, 不是手算的
  const std::vector<std::uint8_t> expected = {
      0x39, 0x30, 0x00, 0x00,   // timestamp_ms = 12345
      0x96, 0x00,               // self_pose_x_cm = 150
      0x1F, 0xFF,               // self_pose_y_cm = -225
      0x00, 0x00,               // self_pose_z_cm = 0  (⚠ ego_pose 无 z, 见 .cpp TODO)
      0x84, 0x03,               // self_pose_yaw = 900  (π/2 rad → 90.0° ×10)
      0x64,                     // match_remaining_sec = 100
      0x01,                     // held_object_count = 1
      0x00,                     // holding_over_limit
      0x00,                     // match_end_release_warning
      0x01,                     // mustika_state = 1 (柱上)
      0x00,                     // sanctuary_mandate_fulfilled
      0x0F,                     // 状态标志字节 = 四位全置(见 StatusFlag); 原为 odometry_reliable=0x01
      0x26, 0x02,               // mustika_x_cm = 550
      0xCF, 0x03,               // mustika_y_cm = 975
      0x36,                     // mustika_z_cm = 54
      0x01, 0x00,               // 穆斯蒂卡柱有物 / 核心柱无物
      0x00,                     // 建筑位计数 = 0
      0x00,                     // 障碍物计数 = 0
  };
  ASSERT_EQ(expected.size() + 6u + kCrcBytes, r.bytes.size())
      << "帧总长 = 6 (帧头+序号+类型+长度) + payload + 2 (CRC)";
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(expected[i], r.bytes[6 + i]) << "payload[" << i << "]";
  }
}

TEST(FrameLayoutTest, 与独立Python实现逐字节一致)
{
  ProtocolEncoder enc;
  const EncodeResult r = enc.encode(make_frame());

  // 一条断言同时钉死: payload 布局 + 各字段字节序 + 长度字段大小端 +
  // **CRC 的覆盖范围与取值** (golden 里的 CRC 是脚本那份位逐位实现在整帧上算的;
  //  覆盖范围若不对, 这里必挂 —— 这正是上一条测试做不到的)
  EXPECT_EQ(std::string(kGoldenFrameHex), to_hex(r.bytes));
}

// ⚠️ 本条测的是**不变式**（2026-09-22 新增）：`pose_valid == false` 时，
//    编码器必须把**所有**位置字段写成 0。
//    这不是"省字节"，而是保证**过期/无效的位置一个都漏不出去** ——
//    写 0 + valid=0：主控万一忘了看 valid 位，拿到的也是 0，
//    而不是一个**看起来合理**的旧位置。
//    不变式刻意放在**编码器**里：位置字段散在头部 / 穆斯蒂卡 / 建筑位 / 障碍物四处，
//    放调用方迟早会漏掉某一个。
TEST(FrameLayoutTest, 有效位为假时位置一律写零)
{
  // payload 各字段的字节偏移（与 .cpp 的写入顺序一一对应，改布局必须同步改这里）
  constexpr std::size_t kOffSelfX   = 4;    // int16 self_pose_x_cm
  constexpr std::size_t kOffSelfY   = 6;    // int16 self_pose_y_cm
  constexpr std::size_t kOffMatch   = 12;   // uint8 match_remaining_sec
  constexpr std::size_t kOffHeld    = 13;   // uint8 held_object_count
  constexpr std::size_t kOffMustika = 16;   // uint8 mustika_state
  constexpr std::size_t kOffFlags   = 18;   // uint8 状态标志字节
  constexpr std::size_t kOffMustX   = 19;   // int16 mustika_x_cm
  constexpr std::size_t kOffMustY   = 21;   // int16 mustika_y_cm
  constexpr std::size_t kOffMustZ   = 23;   // int8  mustika_z_cm
  constexpr std::size_t kHdr        = 6;    // 帧头+序号+类型+长度

  ProtocolEncoder enc;
  auto f = make_frame();               // 字段齐全（三个 valid 位都是 1）
  f.pose_valid = false;                // ← 只关这一个

  const EncodeResult r = enc.encode(f);
  ASSERT_GE(r.bytes.size(), kHdr + kOffFlags + 1);

  // ① 标志位本身：只剩 ODOM_RELIABLE|SANCTUARY_VALID|HELD_COUNT_VALID = 0x0D
  EXPECT_EQ(0x0D, r.bytes[kHdr + kOffFlags])
      << "只该掉 POSE_VALID 那一位(0x0F -> 0x0D)";

  // ② **位置全变 0**
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffSelfX]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffSelfX + 1]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffSelfY]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffSelfY + 1]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffMustX]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffMustX + 1]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffMustY]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffMustY + 1]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffMustZ]);

  // ③ ⚠️ **非位置字段不许被连带清掉** —— 这一半才是这条测试的价值所在：
  //    只断言"位置变 0"的话，一个"把整个 payload 清零"的错误实现照样通过。
  EXPECT_EQ(0x64, r.bytes[kHdr + kOffMatch])   << "比赛计时与该有效位无关";
  EXPECT_EQ(0x01, r.bytes[kHdr + kOffHeld])    << "held_count_valid 仍为真";
  EXPECT_EQ(0x01, r.bytes[kHdr + kOffMustika]) << "mustika_state 是 0/1/2..6 的状态码, 不是位置";
}

// ⚠️ 另一半：`sanctuary_valid` / `held_count_valid` 为假时，对应字段写 0。
//    理由见 PerceptionFrame.msg —— 这两个字段**没有未知档**，
//    填 0 就是替主控断言"使命未达成"/"一块都没拿"，所以必须配 valid 位一起用。
TEST(FrameLayoutTest, 有效位为假时对应字段写零)
{
  constexpr std::size_t kOffHeld    = 13;
  constexpr std::size_t kOffOver    = 14;   // holding_over_limit
  constexpr std::size_t kOffSanct   = 17;   // sanctuary_mandate_fulfilled
  constexpr std::size_t kOffFlags   = 18;
  constexpr std::size_t kHdr        = 6;

  ProtocolEncoder enc;
  auto f = make_frame();
  f.held_object_count = 2;             // 故意给个非零值，才能看出"被清掉了"
  f.holding_over_limit = 1;
  f.sanctuary_mandate_fulfilled = 1;
  f.held_count_valid = false;
  f.sanctuary_valid  = false;

  const EncodeResult r = enc.encode(f);
  ASSERT_GE(r.bytes.size(), kHdr + kOffFlags + 1);

  // 只剩 ODOM_RELIABLE|POSE_VALID = 0x03
  EXPECT_EQ(0x03, r.bytes[kHdr + kOffFlags]);
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffHeld])  << "无效 → 写 0，而不是把 2 发出去";
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffOver])  << "持握数无效时'是否超限'同样无意义";
  EXPECT_EQ(0x00, r.bytes[kHdr + kOffSanct]) << "无效 → 写 0，而不是断言'使命已达成'";
}

TEST(FrameLayoutTest, CRC覆盖整帧而非仅payload)
{
  ProtocolEncoder enc;
  const EncodeResult r = enc.encode(make_frame());

  const std::size_t n = r.bytes.size();
  ASSERT_GT(n, kCrcBytes);

  // ⚠ 两个期望值都是**独立 Python 实现**算的 (脚本见上面 golden 那段), 不是拿 Crc16
  //   重算一遍再比对 —— 那是循环验证: 只能证明"覆盖范围和测试里写的一致",
  //   证明不了"和协议约定一致"。错得自洽的实现照样通过。
  // ⚠️ 2026-09-23 首次编译+跑测: 这两个常量**漏跟 2026-09-22 的状态标志字节改动**
  //    (0x9955 / 0xFAD7 恰好是状态字节仍为 `0x01` 那一版 payload 的整帧 CRC ——
  //    用上面那段独立 Python 实现把第 25 字节改回 0x01 即可复现)。
  //    golden hex 当时更新了, 这里没有, 于是"实现对了、测试红了"。
  //    下面两个值是用同一段独立 Python 实现在**当前** golden 帧上重算的, 不是抄实现输出。
  const Crc16 calc(enc.config());
  EXPECT_EQ(0xA0A1, calc.compute(r.bytes.data(), n - kCrcBytes));      // 覆盖到 payload 末尾
  EXPECT_EQ(0x89A0, calc.compute(r.bytes.data(), n - kCrcBytes - 1));  // 少覆盖一个字节

  EXPECT_EQ(0xA0A1, trailing_crc(r.bytes));   // 帧尾两字节确实就是整帧 CRC
}

TEST(FrameLayoutTest, 心跳帧)
{
  ProtocolEncoder enc;
  const std::vector<std::uint8_t> bytes = enc.encode_heartbeat();

  ASSERT_EQ(8u, bytes.size());               // 6 + 0 + 2
  EXPECT_EQ(0xAA, bytes[0]);
  EXPECT_EQ(0x55, bytes[1]);
  EXPECT_EQ(0x00, bytes[2]);                 // 首帧序号
  EXPECT_EQ(0x02, bytes[3]);                 // PacketType::HEARTBEAT
  EXPECT_EQ(0x00, bytes[4]);                 // 长度 0
  EXPECT_EQ(0x00, bytes[5]);
}

// ⚠️ 0x03 = 紧急停止。它存在的意义是让主控能**区分**：
//      · 收到 0x03 estop=1 → 链路活着，是感知侧要求停车
//      · 什么都没收到      → 链路死了
//    这两件事的应对**正相反**（一个该等感知，一个该急停失能），
//    而在此之前它们在主控侧完全不可区分。
//
//    本条只做**结构**断言（类型/长度/payload 逐字节）：CRC 机制已由上面
//    golden 那两条覆盖，这里再算一遍 CRC 属于重复，且容易退化成循环验证。
TEST(FrameLayoutTest, ESTOP帧结构)
{
  constexpr std::uint32_t kTsMs = 12345;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(kTsMs / 1000u);
  stamp.nanosec = (kTsMs % 1000u) * 1000000u;

  ProtocolEncoder enc;

  // ── 触发 ──
  const std::vector<std::uint8_t> on = enc.encode_estop(true, stamp);
  ASSERT_EQ(6u + ProtocolConfig::kEstopPayloadBytes + 2u, on.size());
  EXPECT_EQ(0xAA, on[0]);
  EXPECT_EQ(0x55, on[1]);
  EXPECT_EQ(0x00, on[2]);                    // 首帧序号
  EXPECT_EQ(0x03, on[3]);                    // PacketType::ESTOP
  EXPECT_EQ(0x05, on[4]);                    // 长度 5 (uint32 + uint8)
  EXPECT_EQ(0x00, on[5]);
  EXPECT_EQ(0x39, on[6]);                     // timestamp_ms = 12345 小端
  EXPECT_EQ(0x30, on[7]);
  EXPECT_EQ(0x00, on[8]);
  EXPECT_EQ(0x00, on[9]);
  EXPECT_EQ(static_cast<std::uint8_t>(kEstopActive), on[10]);

  // ── ⚠️ 解除**也要发** —— 只发"停车"不发"解除"，主控会永远停在那个状态 ──
  const std::vector<std::uint8_t> off = enc.encode_estop(false, stamp);
  ASSERT_EQ(on.size(), off.size());
  EXPECT_EQ(0x03, off[3]);
  EXPECT_EQ(static_cast<std::uint8_t>(kEstopClear), off[10]);
  EXPECT_EQ(0x01, off[2]) << "第二帧序号应为 1 —— 紧急帧同样消耗序号(与其他包共享)";
}

TEST(FrameLayoutTest, 长度字段等于payload实际字节数)
{
  ProtocolEncoder enc;
  const EncodeResult r = enc.encode(make_frame_with_spots(3));
  const std::uint16_t len = static_cast<std::uint16_t>(r.bytes[4] | (r.bytes[5] << 8));

  EXPECT_EQ(r.bytes.size() - 6 - 2, len);
  EXPECT_EQ(16 + 8 + 2 + (1 + 4 * 3) + 1, len);   // 固定段 + 3 个建筑位 + 空障碍物
}

// ═══════════════════════════════════════════════════════════════════════════
// 截断 (超限不丢整帧)
// ═══════════════════════════════════════════════════════════════════════════

TEST(TruncationTest, 建筑位超限截断到上限)
{
  ProtocolEncoder enc;                       // max_building_spots = 16
  const EncodeResult r = enc.encode(make_frame_with_spots(20));

  EXPECT_EQ(16u, r.spots_written);           // 20 → 16
  EXPECT_EQ(0u, r.obstacles_written);
  EXPECT_EQ(16 + 8 + 2 + (1 + 4 * 16) + 1, r.bytes.size() - 6 - 2);
  EXPECT_EQ(16, r.bytes[6 + 26]);            // 建筑位计数 (payload 偏移 26)
}

TEST(TruncationTest, 障碍物超限截断到上限)
{
  ProtocolEncoder enc;                       // max_obstacles = 10
  const EncodeResult r = enc.encode(make_frame_with_obstacles(15));

  EXPECT_EQ(10u, r.obstacles_written);       // 15 → 10
  EXPECT_EQ(0u, r.spots_written);
  EXPECT_EQ(16 + 8 + 2 + 1 + (1 + 18 * 10), r.bytes.size() - 6 - 2);
  EXPECT_EQ(10, r.bytes[6 + 27]);            // 障碍物计数 (payload 偏移 27)
}

TEST(TruncationTest, 未超限时写入数量等于输入数量)
{
  ProtocolEncoder enc;
  const EncodeResult r = enc.encode(make_frame_with_spots(5));
  EXPECT_EQ(5u, r.spots_written);
}

// ═══════════════════════════════════════════════════════════════════════════
// 数值边界
//
// ⚠ 正负溢出两条必须都测: 有符号溢出是 **UB**, 忘记夹取时 Debug 可能"过"、
//   Release 下挂 —— 是最难查的一类。
// ═══════════════════════════════════════════════════════════════════════════

TEST(NumericBoundaryTest, 正溢出夹到INT16_MAX)
{
  ProtocolEncoder enc;
  br_perception::msg::PerceptionFrame f = make_frame();
  f.ego_pose.x = 1e10;                       // 折合 1e12 cm
  const EncodeResult r = enc.encode(f);

  EXPECT_EQ(0xFF, r.bytes[6 + 4]);           // 32767 = 0x7FFF, 小端
  EXPECT_EQ(0x7F, r.bytes[6 + 5]);
}

TEST(NumericBoundaryTest, 负溢出夹到INT16_MIN)
{
  ProtocolEncoder enc;
  br_perception::msg::PerceptionFrame f = make_frame();
  f.ego_pose.x = -1e10;
  const EncodeResult r = enc.encode(f);

  EXPECT_EQ(0x00, r.bytes[6 + 4]);           // -32768 = 0x8000, 小端
  EXPECT_EQ(0x80, r.bytes[6 + 5]);
}

TEST(NumericBoundaryTest, NaN与Inf落零且输出可重复)
{
  ProtocolEncoder enc;
  br_perception::msg::PerceptionFrame f = make_frame();
  f.ego_pose.x = std::nan("");
  f.ego_pose.y = std::numeric_limits<double>::infinity();

  const EncodeResult r1 = enc.encode(f);
  const EncodeResult r2 = enc.encode(f);     // 同一输入再编一次

  ASSERT_EQ(r1.bytes.size(), r2.bytes.size());
  for (std::size_t i = 6; i + kCrcBytes < r1.bytes.size(); ++i) {
    EXPECT_EQ(r1.bytes[i], r2.bytes[i]) << "payload[" << (i - 6) << "] 不稳定";
  }
  EXPECT_EQ(0x00, r1.bytes[6 + 4]);          // NaN  → 0
  EXPECT_EQ(0x00, r1.bytes[6 + 5]);
  EXPECT_EQ(0x00, r1.bytes[6 + 6]);          // +Inf → 0
  EXPECT_EQ(0x00, r1.bytes[6 + 7]);
}

TEST(NumericBoundaryTest, 非有限速度落零)
{
  ProtocolEncoder enc;
  br_perception::msg::PerceptionFrame f = make_frame_with_obstacles(1);
  f.obstacles[0].velocity.x = std::nan("");
  f.obstacles[0].velocity.y = std::numeric_limits<double>::infinity();
  const EncodeResult r = enc.encode(f);

  const std::size_t obs0 = 6 + 28;           // 障碍物段起点 (0 个建筑位时)
  EXPECT_EQ(0x00, r.bytes[obs0 + 15]);       // obs_vx_cm_s
  EXPECT_EQ(0x00, r.bytes[obs0 + 16]);       // obs_vy_cm_s
}

TEST(NumericBoundaryTest, 负尺寸落零)
{
  // to_u8 对负数返回 0 (而不是 UB 或回绕)。融合层若因坐标变换写错给出负 bbox_size,
  // 编码器必须是"静默发 0", 不是崩溃或发出荒谬值。
  ProtocolEncoder enc;
  br_perception::msg::PerceptionFrame f = make_frame_with_obstacles(1);
  f.obstacles[0].bbox_size.x = -0.35;
  const EncodeResult r = enc.encode(f);

  const std::size_t o = 6 + 28;
  EXPECT_EQ(0x00, r.bytes[o + 6]);           // obs_width_cm ← bbox_size.x
}

// ═══════════════════════════════════════════════════════════════════════════
// 帧序号
// ═══════════════════════════════════════════════════════════════════════════

TEST(SequenceTest, 连续三百次跨越256回绕)
{
  ProtocolEncoder enc;
  const auto& f = make_frame();

  for (int i = 0; i < 256; ++i) {
    const EncodeResult r = enc.encode(f);
    ASSERT_EQ(static_cast<std::uint8_t>(i), r.bytes[2]) << "第 " << i << " 帧";
  }
  EXPECT_EQ(std::uint8_t{0}, enc.current_seq());     // 256 次后回到 0

  for (int i = 0; i < 44; ++i) {
    const EncodeResult r = enc.encode(f);
    ASSERT_EQ(static_cast<std::uint8_t>(i), r.bytes[2]) << "回绕后第 " << i << " 帧";
  }
  EXPECT_EQ(std::uint8_t{44}, enc.current_seq());
}

TEST(SequenceTest, 心跳与感知帧共享序号空间)
{
  // ⚠ 本测试锁定的是**当前的协议假设** (见 protocol_encoder.hpp:
  //   "seq 空间在所有 PacketType 间共享, 心跳也消耗序号")。
  //   下位机若确认改成"每种类型独立编号", 本测试与实现都要同步改 ——
  //   别看到这三条挂了就以为是自己改坏了。
  ProtocolEncoder enc;

  const auto hb0 = enc.encode_heartbeat();
  ASSERT_GE(hb0.size(), 3u);
  EXPECT_EQ(0, hb0[2]);

  EXPECT_EQ(1, enc.encode(make_frame()).bytes[2]);

  const auto hb1 = enc.encode_heartbeat();
  ASSERT_GE(hb1.size(), 3u);
  EXPECT_EQ(2, hb1[2]);
}

// ═══════════════════════════════════════════════════════════════════════════
// 未冻结字段: 活的待办标记
//
// Obstacle.msg 缺 5 个协议要求的字段, 编码时暂写 0。融合层把它们补上之后,
// 下面这条会**失败** —— 那正是提醒"该把真实值接上了"。
// ═══════════════════════════════════════════════════════════════════════════

// ⚠ **本组测试失败时** = Obstacle.msg 补齐了字段。请同步三处:
//     (a) 本测试的期望值   (b) protocol_encoder.cpp 的编码逻辑   (c) 协议表 (任务书 §5.1)
TEST(UnfrozenFieldsTest, 五个缺失字段目前恒为零)
{
  ProtocolEncoder enc;
  br_perception::msg::PerceptionFrame f = make_frame_with_obstacles(1);
  f.obstacles[0].velocity.x = 0.05;          // +5 cm/s
  f.obstacles[0].velocity.y = -0.10;         // -10 cm/s
  f.obstacles[0].id = 7;

  const EncodeResult r = enc.encode(f);

  // 障碍物段起点 (payload 偏移 28), 每项 18 字节, 字段顺序见 .cpp
  const std::size_t o = 6 + 28;
  EXPECT_EQ(7, r.bytes[o + 0]);              // obs_id
  EXPECT_EQ(0x00, r.bytes[o + 10]);          // is_ally_tr         ← 缺失
  EXPECT_EQ(0x00, r.bytes[o + 11]);          // obs_dwell_ms 低    ← 缺失
  EXPECT_EQ(0x00, r.bytes[o + 12]);          // obs_dwell_ms 高    ← 缺失
  EXPECT_EQ(0x00, r.bytes[o + 13]);          // suspected_pushing  ← 缺失
  EXPECT_EQ(0x00, r.bytes[o + 14]);          // obs_held           ← 缺失
  EXPECT_EQ(5, r.bytes[o + 15]);             // obs_vx_cm_s = +5   (可填, 证明偏移没算错)
  EXPECT_EQ(0xF6, r.bytes[o + 16]);          // obs_vy_cm_s = -10
  EXPECT_EQ(0x00, r.bytes[o + 17]);          // threat_level       ← 缺失
}

}  // namespace communication
}  // namespace br_perception
