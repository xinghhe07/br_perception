/// ═══════════════════════════════════════════════════════════════════════════════
/// test_health_monitor.cpp — 模块 26 热管理与健康判定：**纯函数**单元测试
///
/// ⚠️ 本文件只测**判定逻辑**。节点外壳（订诊断 / 读 `hwmon` / 发话题）不在覆盖范围。
///
/// ── 为什么热管理状态机值得单测 ──
///   规格 §6.2 的场景是：**无风扇迷你主机** + 长时间高负载 ⇒ 核心到 90°C 降频
///   ⇒ **推理延迟 30→50ms 超预算**（那直接变成分数）。
///   而它的两个保护机制**都容易被写掉**：
///     · **滞回死区**（85/75 之间那 10 度）—— 没有它，温度在阈值附近抖动
///       ⇒ 降级/恢复**反复切换** ⇒ 相机管线一会儿开一会儿关，**比一直降级更糟**；
///     · **30 秒持续** —— 没有它，刚降到 74 就恢复 ⇒ 负载一上来又冲回 90，同样在抖。
///   ⇒ 单测里最贵的是那两条"**不该动作**"的用例。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=ThermalTest.*`）。
/// ⚠️ 全部用例确定性：手造状态、无时间、无硬件。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/core/health_monitor.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

namespace br_perception {
namespace core {

using HM = HealthMonitor;
using St = HealthMonitor::ThermalState;
using Lv = HealthMonitor::Level;

namespace {
constexpr St kNormal = HM::ThermalState::kNormal;
constexpr St kThrottled = HM::ThermalState::kThrottled;
}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// ★ 热管理状态机
// ═══════════════════════════════════════════════════════════════════════════

TEST(ThermalTest, 超上限立即降级)
{
  HM::ThermalParams p;                    // 85 / 75 / 30s
  const auto v = HM::step_thermal(kNormal, 86.0, 0.0, 0.1, p);
  EXPECT_EQ(v.state, kThrottled);
  EXPECT_TRUE(v.changed) << "状态切换了就该发一次告警";
}

TEST(ThermalTest, 边界恰好等于上限不降级)
{
  // ⚠️ 严格 `>`：恰好 85.0 **不**降级。改成 `>=` 会让"刚好到 85 的常态"也降级。
  HM::ThermalParams p;
  const auto v = HM::step_thermal(kNormal, 85.0, 0.0, 0.1, p);
  EXPECT_EQ(v.state, kNormal) << "恰好等于阈值不该降级（判据是严格 >）";
  EXPECT_FALSE(v.changed);
}

TEST(ThermalTest, 低温不足三十秒不恢复)
{
  HM::ThermalParams p;
  // 已经低了 29.9 秒，这一步 +0.1 ⇒ 30.0 ⇒ **该恢复**
  const auto almost = HM::step_thermal(kThrottled, 70.0, 29.7, 0.1, p);
  EXPECT_EQ(almost.state, kThrottled) << "29.8s 还不够";
  EXPECT_FALSE(almost.changed);

  const auto now = HM::step_thermal(kThrottled, 70.0, 29.9, 0.1, p);
  EXPECT_EQ(now.state, kNormal) << "累计到 30.0s 该恢复（非严格 >=）";
  EXPECT_TRUE(now.changed);
}

TEST(ThermalTest, 恢复后连续计时清零)
{
  HM::ThermalParams p;
  const auto v = HM::step_thermal(kThrottled, 70.0, 30.0, 0.1, p);
  EXPECT_EQ(v.state, kNormal);
  EXPECT_DOUBLE_EQ(v.hold_sec, 0.0) << "恢复后要从零重新计";
}

// ── ★★ 滞回（本文件最贵的两条）──

TEST(ThermalTest, 滞回区内保持降级不恢复)
{
  // ⚠️⚠️ **这条钉的是滞回死区的全部意义**：
  //    温度在 75~85 之间（比如 80），**不该恢复** —— 哪怕已经在那里待了很久。
  //    没有这条判据的话，降温到 84 就恢复 ⇒ 负载一上来又冲回 90 ⇒ **来回抖**。
  HM::ThermalParams p;
  auto v = HM::step_thermal(kThrottled, 80.0, 999.0, 0.1, p);
  EXPECT_EQ(v.state, kThrottled) << "在死区里不该恢复";
  EXPECT_TRUE(v.in_hysteresis);
  EXPECT_FALSE(v.changed);
  EXPECT_DOUBLE_EQ(v.hold_sec, 0.0) << "死区要打断'连续低温'的计时";
}

TEST(ThermalTest, 滞回区内也不降级)
{
  // 反向：正常态在死区里（比如 80）**不该降级**
  HM::ThermalParams p;
  const auto v = HM::step_thermal(kNormal, 80.0, 0.0, 0.1, p);
  EXPECT_EQ(v.state, kNormal);
  EXPECT_TRUE(v.in_hysteresis);
}

TEST(ThermalTest, 死区打断连续低温计时)
{
  // ⚠️ "**持续**低于 75" 的字面意思就是不能中断：
  //    掉到 74 待 29 秒 → 弹回 76 一下 → 再到 74，那**不是**连续 30 秒。
  //    ⚠️ 实现成"暂停"而不是"清零"的话，这个场景会在几秒后就误恢复。
  HM::ThermalParams p;
  auto v = HM::step_thermal(kThrottled, 74.0, 29.0, 1.0, p);   // 连续 30s ⇒ 恢复
  EXPECT_EQ(v.state, kNormal);

  // 重新降级，再来一遍但中途弹回死区
  v = HM::step_thermal(kNormal, 90.0, 0.0, 0.1, p);
  ASSERT_EQ(v.state, kThrottled);
  v = HM::step_thermal(kThrottled, 74.0, 0.0, 29.0, p);
  ASSERT_EQ(v.state, kThrottled) << "29s 还不够";
  ASSERT_DOUBLE_EQ(v.hold_sec, 29.0);
  v = HM::step_thermal(kThrottled, 76.0, 29.0, 1.0, p);        // 弹回死区
  EXPECT_DOUBLE_EQ(v.hold_sec, 0.0) << "死区必须**清零**而不是暂停";
  v = HM::step_thermal(kThrottled, 74.0, 0.0, 29.0, p);        // 再来 29s
  EXPECT_EQ(v.state, kThrottled) << "累计已被打断 ⇒ 29s 还不够";
}

TEST(ThermalTest, 边界恰好等于恢复温度不算低温)
{
  // ⚠️ 严格 `<`：恰好 75.0 **不算**低温 ⇒ 落在"死区"那一支
  HM::ThermalParams p;
  const auto v = HM::step_thermal(kThrottled, 75.0, 100.0, 0.1, p);
  EXPECT_EQ(v.state, kThrottled);
  EXPECT_TRUE(v.in_hysteresis);
  EXPECT_DOUBLE_EQ(v.hold_sec, 0.0);
}

TEST(ThermalTest, 高温把连续低温计时清零)
{
  // 连续低温计到 20s，然后冲上 90 ⇒ 计时必须清零
  HM::ThermalParams p;
  const auto v = HM::step_thermal(kNormal, 90.0, 20.0, 0.1, p);
  EXPECT_EQ(v.state, kThrottled);
  EXPECT_DOUBLE_EQ(v.hold_sec, 0.0) << "高温要打断'连续低温'";
}

TEST(ThermalTest, 非正dt不推进计时)
{
  HM::ThermalParams p;
  EXPECT_DOUBLE_EQ(HM::step_thermal(kThrottled, 70.0, 5.0, 0.0, p).hold_sec, 5.0);
  EXPECT_DOUBLE_EQ(HM::step_thermal(kThrottled, 70.0, 5.0, -1.0, p).hold_sec, 5.0);
}

TEST(ThermalTest, 参数合理性检查抓得出写反的滞回)
{
  // ⚠️ 静默失效形态：`recover_c >= warn_c` ⇒ 没有死区 ⇒ 状态反复翻转，
  //    而**日志里只会看到一串切换**，看不出是配置问题。
  HM::ThermalParams ok;
  EXPECT_TRUE(HM::thermal_params_sane(ok));

  HM::ThermalParams bad;
  bad.recover_c = 85.0;      // 与 warn 相等 ⇒ 死区为 0
  EXPECT_FALSE(HM::thermal_params_sane(bad)) << "死区为 0 就没有滞回";

  HM::ThermalParams worse;
  worse.recover_c = 90.0;    // 彻底写反
  EXPECT_FALSE(HM::thermal_params_sane(worse));
}

TEST(ThermalTest, 一次降温过程不来回抖)
{
  // 端到端一点的行为验证：模拟一段温度曲线（冲高 → 回落 → 在死区里抖），
  // **状态切换次数必须是 1**（只降级一次，不该在死区里反复切）。
  HM::ThermalParams p;
  auto st = kNormal;
  double hold = 0.0;
  int switches = 0;
  const double curve[] = {60, 70, 80, 88, 90, 86, 82, 79, 81, 78, 76, 77, 75, 74, 76, 73};
  for (double t : curve) {
    const auto v = HM::step_thermal(st, t, hold, 1.0, p);
    if (v.changed) ++switches;
    st = v.state; hold = v.hold_sec;
  }
  EXPECT_EQ(switches, 1) << "这段曲线只该降级一次 —— 多出来就是在抖";
  EXPECT_EQ(st, kThrottled) << "最后温度还在死区/高温 ⇒ 应保持降级";
}

// ═══════════════════════════════════════════════════════════════════════════
// 告警级别聚合
// ═══════════════════════════════════════════════════════════════════════════

TEST(LevelTest, 取更严重的那个)
{
  EXPECT_EQ(HM::worst(Lv::kOk, Lv::kError), Lv::kError);
  EXPECT_EQ(HM::worst(Lv::kFatal, Lv::kWarning), Lv::kFatal);
  EXPECT_EQ(HM::worst(Lv::kOk, Lv::kOk), Lv::kOk);
}

TEST(LevelTest, 一条ERROR不会被后面的OK盖掉)
{
  // ⚠️ 取"最新"或"平均"的症状：**故障刚发生就被标记成正常**。
  const auto v = HM::aggregate({Lv::kOk, Lv::kError, Lv::kOk, Lv::kOk});
  EXPECT_EQ(v, Lv::kError) << "整机级别是各模块里最坏的那个";
}

TEST(LevelTest, 空列表是OK)
{
  EXPECT_EQ(HM::aggregate({}), Lv::kOk);
}

TEST(LevelTest, 名字)
{
  EXPECT_STREQ(HM::level_name(Lv::kOk), "OK");
  EXPECT_STREQ(HM::level_name(Lv::kWarning), "WARNING");
  EXPECT_STREQ(HM::level_name(Lv::kError), "ERROR");
  EXPECT_STREQ(HM::level_name(Lv::kFatal), "FATAL");
  EXPECT_NE(HM::level_name(static_cast<Lv>(99)), nullptr);   // ⚠️ 不能是空指针
}

// ═══════════════════════════════════════════════════════════════════════════
// 单项指标
// ═══════════════════════════════════════════════════════════════════════════

TEST(MetricTest, CPU使用率)
{
  HM::MetricParams p;                  // 0.80
  EXPECT_EQ(HM::judge_cpu_usage(0.5, p), Lv::kOk);
  EXPECT_EQ(HM::judge_cpu_usage(0.80, p), Lv::kOk) << "恰好 80% 不告警（严格 >）";
  EXPECT_EQ(HM::judge_cpu_usage(0.81, p), Lv::kWarning);
}

TEST(MetricTest, 延迟)
{
  HM::MetricParams p;                  // 70ms
  EXPECT_EQ(HM::judge_latency(56.0, p), Lv::kOk) << "56ms 是 pipeline 自己的 warn 线，不是健康线";
  EXPECT_EQ(HM::judge_latency(70.0, p), Lv::kOk);
  EXPECT_EQ(HM::judge_latency(70.1, p), Lv::kWarning);
}

TEST(MetricTest, 相机帧率低是ERROR不是WARNING)
{
  // ⚠️ 相机饿死会让整条视觉链（含那 250 分的穆斯蒂卡判定）**没有输入** ——
  //    那与"慢了"不是一个量级。
  HM::MetricParams p;                  // 5fps
  EXPECT_EQ(HM::judge_camera_fps(30.0, p), Lv::kOk);
  EXPECT_EQ(HM::judge_camera_fps(5.0, p), Lv::kOk) << "恰好等于下限不报错（严格 <）";
  EXPECT_EQ(HM::judge_camera_fps(4.9, p), Lv::kError);
  EXPECT_EQ(HM::judge_camera_fps(0.0, p), Lv::kError);
}

TEST(MetricTest, CRC错误率)
{
  HM::MetricParams p;                  // 1%
  EXPECT_EQ(HM::judge_crc_error_rate(0.0, p), Lv::kOk);
  EXPECT_EQ(HM::judge_crc_error_rate(0.01, p), Lv::kOk);
  EXPECT_EQ(HM::judge_crc_error_rate(0.02, p), Lv::kError);
}

// ═══════════════════════════════════════════════════════════════════════════
// ★★ CPU 温度读取（`acpitz` 那个坑）
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// 造一个假的 `hwmon` 目录树（测试用真文件系统，不是 mock —— 那段代码就是读文件的）
struct FakeHwmon
{
  std::string root;
  explicit FakeHwmon(const std::string& tag)
  {
    root = "/tmp/br_test_hwmon_" + tag + "_" + std::to_string(::getpid());
    std::system(("rm -rf " + root).c_str());
    std::system(("mkdir -p " + root).c_str());
  }
  ~FakeHwmon() { std::system(("rm -rf " + root).c_str()); }
  FakeHwmon(const FakeHwmon&) = delete;
  FakeHwmon& operator=(const FakeHwmon&) = delete;

  /// 造一路传感器：`hwmonN/{name, temp1_input, temp1_label}`
  void add(const std::string& idx, const std::string& name,
           long milli_c, const std::string& label = "") const
  {
    const std::string d = root + "/" + idx;
    std::system(("mkdir -p " + d).c_str());
    std::ofstream(d + "/name") << name << "\n";
    std::ofstream(d + "/temp1_input") << milli_c << "\n";
    if (!label.empty()) std::ofstream(d + "/temp1_label") << label << "\n";
  }
};

}  // namespace

TEST(ThermalReadTest, 按名字找而不是拿第一个)
{
  // ⚠️⚠️ **本文件第二贵的一条**（与滞回并列）：本机 2026-09-25 实测
  //    `acpitz` 读出来是 **20.0°C**（未接的 ACPI 占位值），而 `k10temp` 才是真 CPU 温度。
  //    天真实现"遍历 hwmon 拿第一个温度"会**永远读到 20°C** ⇒ **热保护永不触发**，
  //    而症状是"什么都没发生" —— 到 90°C 降频了也没人管，**查都没法查**。
  FakeHwmon f("byname");
  f.add("hwmon0", "acpitz", 20000);        // 那个假的 20°C
  f.add("hwmon1", "k10temp", 45700, "Tctl");

  const auto r = HM::read_cpu_temp(f.root, "k10temp");
  ASSERT_TRUE(r.found) << "必须按名字找到 k10temp";
  EXPECT_DOUBLE_EQ(r.temp_c, 45.7) << "拿到的是 45.7 而不是 acpitz 的 20.0";
  EXPECT_EQ(r.sensor, "k10temp");
  EXPECT_EQ(r.label, "Tctl");
}

TEST(ThermalReadTest, 毫摄氏度要换算)
{
  // ⚠️ hwmon 的单位是**毫摄氏度** ⇒ 漏了 /1000 会得到 "45700 °C"，
  //    而那个值会让热保护**立刻触发**（或恒不触发，取决于阈值方向）。
  FakeHwmon f("milli");
  f.add("hwmon0", "k10temp", 45700, "Tctl");
  const auto r = HM::read_cpu_temp(f.root, "k10temp");
  ASSERT_TRUE(r.found);
  EXPECT_DOUBLE_EQ(r.temp_c, 45.7);
}

TEST(ThermalReadTest, 找不到那个传感器时如实报false)
{
  // ⚠️ **不许返回 0 假装成功**（§零「准」： "没量到"与"量到是 0 度"是两件事）
  FakeHwmon f("missing");
  f.add("hwmon0", "acpitz", 20000);        // 只有那个假的
  const auto r = HM::read_cpu_temp(f.root, "k10temp");
  EXPECT_FALSE(r.found) << "没有 k10temp 就该报 found=false";
  // ⚠️ 而且判据必须是 WARNING（"不知道"），不是 OK（"没事"）
  EXPECT_EQ(HM::judge_thermal_reading(r), Lv::kWarning)
      << "读不到温度报 OK ⇒ 诊断里一片绿而热保护永不触发";
}

TEST(ThermalReadTest, hwmon根目录不存在也不崩)
{
  const auto r = HM::read_cpu_temp("/tmp/绝对不存在的目录_br_test", "k10temp");
  EXPECT_FALSE(r.found);
  EXPECT_EQ(HM::judge_thermal_reading(r), Lv::kWarning);
}

TEST(ThermalReadTest, 优先取Tctl)
{
  // ⚠️ `k10temp` 通常同时给 `Tctl`(控制用) 与 `Tdie`(裸片)，规格 §6.2 点名 **Tctl/Tdie**。
  //    没有优先级的话会取到 `readdir` 先返回的那个 —— 而**顺序不保证稳定**。
  FakeHwmon f("tctl");
  const std::string d = f.root + "/hwmon0";
  std::system(("mkdir -p " + d).c_str());
  std::ofstream(d + "/name") << "k10temp\n";
  std::ofstream(d + "/temp1_input") << 41000 << "\n";   // 先造的：假设是 Tdie
  std::ofstream(d + "/temp1_label") << "Tdie\n";
  std::ofstream(d + "/temp2_input") << 45700 << "\n";
  std::ofstream(d + "/temp2_label") << "Tctl\n";

  const auto r = HM::read_cpu_temp(f.root, "k10temp");
  ASSERT_TRUE(r.found);
  EXPECT_EQ(r.label, "Tctl") << "该优先 Tctl，而不是 readdir 先返回的那个";
  EXPECT_DOUBLE_EQ(r.temp_c, 45.7);
}

TEST(ThermalReadTest, 读不到输入文件时如实报false)
{
  // 只有 name、没有 temp*_input ⇒ found=false（不是拿 0 顶上）
  FakeHwmon f("noinput");
  const std::string d = f.root + "/hwmon0";
  std::system(("mkdir -p " + d).c_str());
  std::ofstream(d + "/name") << "k10temp\n";
  const auto r = HM::read_cpu_temp(f.root, "k10temp");
  EXPECT_FALSE(r.found);
}

TEST(ThermalReadTest, 本机真hwmon上按名字读得到)
{
  // ⚠️ **活体验证**（不是假目录）：本机有 `k10temp`（AMD Zen 的封装温度）。
  //    ⚠️ 别的机器可能没有 ⇒ 找不到时**只警告不失败**（这不是本包的部署前提）。
  const auto r = HM::read_cpu_temp();     // 默认 /sys/class/hwmon + k10temp
  if (!r.found) {
    GTEST_SKIP() << "本机没有 k10temp（非 AMD / 无该传感器）—— 跳过，不算失败";
  }
  EXPECT_EQ(r.sensor, "k10temp");
  EXPECT_GT(r.temp_c, 0.0);
  EXPECT_LT(r.temp_c, 120.0) << "超过 120°C 说明单位或传感器选错了";
}

}  // namespace core
}  // namespace br_perception
