/// ═══════════════════════════════════════════════════════════════════════════════
/// test_fusion_decisions.cpp — 模块 21 的**判定规则**：纯函数单元测试
///
/// ⚠️ 本文件不覆盖节点（订阅/发布/调 18/19 由 `fusion_node` 做，那是另一刀）。
///
/// ── 为什么规则值得单测（它们都是"几行 if"）──
///   因为**规则错了不报错，只表现为"分没了"**：
///     · `sanctuary_mandate_fulfilled` 判错 ⇒ 要么白丢 **250 分**，
///       要么让主控去做规则上不允许的动作（TR 取五色石的前置，规则 4.5.1）；
///     · `holding_over_limit` 判错 ⇒ 误报违规或漏报；
///     · `out_of_bounds` 判错 ⇒ 把一台**贴着围栏跑**的 BR 永久移除（规则 6.4 不可逆）。
///   ⇒ 最贵的几条是**边界方向**与**多条件与关系**的用例。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=SanctuaryTest.*`）。
/// ⚠️ 全部用例确定性：手造 POD、无时间、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/fusion/fusion_decisions.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace br_perception {
namespace fusion {

using FD = FusionDecisions;
using Spot = FusionDecisions::SpotState;
using Held = FusionDecisions::HeldObject;

namespace {

Spot spot(std::uint32_t id, bool complete, bool shared)
{
  Spot s;
  s.id = id;
  s.complete = complete;
  s.in_shared_zone = shared;
  return s;
}

/// 按 `field_geometry.yaml` 的真实布局造 10 个位：
/// L1 六个（其中 id 4/5 是共享区位），L2 四个（**全在 L2 共享区内**）
std::vector<Spot> field_of(bool complete_ids_1_to_6, bool complete_l2)
{
  std::vector<Spot> v;
  for (std::uint32_t i = 0; i < 10; ++i) {
    const bool is_l1 = i < 6;
    const bool shared = (i == 4 || i == 5) || (!is_l1);   // L1 的 4/5 + 全部 L2
    const bool complete = is_l1 ? complete_ids_1_to_6 : complete_l2;
    v.push_back(spot(i, complete, shared));
  }
  return v;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// ★ 圣所使命 —— **值 250 分**（规则 4.5.1：TR 取五色石的前置）
// ═══════════════════════════════════════════════════════════════════════════

TEST(SanctuaryTest, 两座完整塔其中一座在共享区即达成)
{
  FD::SanctuaryParams p;
  std::vector<Spot> spots{
      spot(0, true, false),    // L1 非共享位
      spot(4, true, true),     // L1 共享位 (5.5, 2.8)
      spot(1, false, false),
  };
  EXPECT_TRUE(FD::sanctuary_mandate_fulfilled(spots, p));
}

TEST(SanctuaryTest, 两座塔都在非共享位时未达成)
{
  // ⚠️⚠️ **最容易错的一档**：塔数够了（2 座）就以为成了。
  //    规则要求"**其中 ≥1 座位于共享区**"—— 这两座都在非共享位 ⇒ **未达成**。
  FD::SanctuaryParams p;
  std::vector<Spot> spots{
      spot(0, true, false),    // L1 非共享
      spot(1, true, false),    // L1 非共享
      spot(4, false, true),    // 共享位上是个半成品 —— 不算
  };
  EXPECT_FALSE(FD::sanctuary_mandate_fulfilled(spots, p))
      << "塔数够但没一座在共享区 ⇒ 不是圣所使命达成";
}

TEST(SanctuaryTest, 共享位上只有半成品不算数)
{
  // ⚠️ 判据是「**完整塔**且**在共享区**」两个条件同时成立，
  //    不是"全场塔数≥2 && 全场有共享区塔"（那样会把这两条解耦）。
  FD::SanctuaryParams p;
  std::vector<Spot> spots{
      spot(0, true, false),
      spot(1, true, false),
      spot(4, false, true),    // 共享位但**没建完**
  };
  EXPECT_FALSE(FD::sanctuary_mandate_fulfilled(spots, p));
}

TEST(SanctuaryTest, 只有一座完整塔时未达成)
{
  // ≥2 是硬条件
  FD::SanctuaryParams p;
  std::vector<Spot> spots{spot(4, true, true)};
  EXPECT_FALSE(FD::sanctuary_mandate_fulfilled(spots, p));
}

TEST(SanctuaryTest, 三座塔两座在共享区)
{
  FD::SanctuaryParams p;
  std::vector<Spot> spots{
      spot(0, true, false),
      spot(4, true, true),
      spot(5, true, true),
  };
  EXPECT_TRUE(FD::sanctuary_mandate_fulfilled(spots, p));
}

TEST(SanctuaryTest, 空场未达成)
{
  FD::SanctuaryParams p;
  EXPECT_FALSE(FD::sanctuary_mandate_fulfilled({}, p));
}

TEST(SanctuaryTest, 按真实场地布局全建满即达成)
{
  // L1 六座 + L2 四座全建满 ⇒ 10 座完整塔，其中 6 座在共享区
  FD::SanctuaryParams p;
  const auto spots = field_of(/*complete_ids_1_to_6=*/true, /*complete_l2=*/true);
  ASSERT_EQ(spots.size(), 10u);
  EXPECT_TRUE(FD::sanctuary_mandate_fulfilled(spots, p));
}

TEST(SanctuaryTest, 按真实布局只有L2两个位建满也达成)
{
  // L2 四位**全在共享区** ⇒ 只要 2 座 L2 塔就同时满足两条
  FD::SanctuaryParams p;
  std::vector<Spot> spots{
      spot(6, true, true),   // L2 (4.25,4.25)
      spot(7, true, true),   // L2 (6.75,4.25)
  };
  EXPECT_TRUE(FD::sanctuary_mandate_fulfilled(spots, p));
}

TEST(SanctuaryTest, 阈值可配)
{
  FD::SanctuaryParams p;
  p.min_complete_towers = 1;
  p.min_in_shared_zone = 1;
  EXPECT_TRUE(FD::sanctuary_mandate_fulfilled({spot(4, true, true)}, p));
}

// ── ★ 「没量成」的位（`state == 4 UNKNOWN`）怎么算 ──
//    ⚠️ 这是**Gate 着 250 分**的那条判定里最容易被随手写错的一档。

TEST(SanctuaryTest, 全部量成时结论可信)
{
  FD::SanctuaryParams p;
  std::vector<Spot> spots{spot(0, true, false), spot(4, true, true)};
  const auto v = FD::judge_sanctuary(spots, p);
  EXPECT_TRUE(v.fulfilled);
  EXPECT_TRUE(v.valid) << "没有 unknown 位 ⇒ 结论可信";
  EXPECT_EQ(v.unknown_count, 0);
  EXPECT_EQ(v.complete_total, 2);
  EXPECT_EQ(v.complete_in_shared, 1);
}

TEST(SanctuaryTest, 有没量成的位时结论不可信)
{
  // ⚠️⚠️ **这条是核心**：哪怕量成的那些位已经够判"达成"了，
  //    只要**还有一位没量成**，这个结论就**不是完整数据的结果** ⇒ `valid = false`。
  //    ⇒ 下游（`PerceptionFrame.sanctuary_valid`）据此决定要不要把它当事实发给主控。
  FD::SanctuaryParams p;
  std::vector<Spot> spots{
      spot(0, true, false),
      spot(4, true, true),     // 量成了，够判达成
      spot(7, false, true),    // ⚠️ 这一位没量成
  };
  spots[2].known = false;
  const auto v = FD::judge_sanctuary(spots, p);
  EXPECT_TRUE(v.fulfilled) << "按量成的位算是达成的";
  EXPECT_FALSE(v.valid) << "但有位没量成 ⇒ 结论不可信，必须如实标出来";
  EXPECT_EQ(v.unknown_count, 1);
}

TEST(SanctuaryTest, 没量成的位不算完整塔)
{
  // ⚠️ **不能把 unknown 当 complete** —— 那会**凭空造出一座塔**，
  //    让主控去做规则不允许的动作（比如以为前置已达成，去取那 250 分的球）。
  FD::SanctuaryParams p;
  std::vector<Spot> spots{spot(0, true, false), spot(4, true, true)};
  spots[1].known = false;      // 共享位那位没量成
  spots[1].complete = true;    // 就算上游"填了 complete"，也不该算数
  const auto v = FD::judge_sanctuary(spots, p);
  EXPECT_EQ(v.complete_total, 1) << "没量成的位不该被算成完整塔";
  EXPECT_EQ(v.complete_in_shared, 0);
  EXPECT_FALSE(v.fulfilled) << "共享位没量成 ⇒ 那 1 座不该算数";
  EXPECT_FALSE(v.valid);
}

TEST(SanctuaryTest, 没量成的位也不算空)
{
  // ⚠️ 另一半：也不能因此判"未达成"就当它不存在 —— 它照样让结论不可信。
  //    这与 `BuildingSpot.msg` 的"4 与 0 必须分开"是同一条纪律。
  FD::SanctuaryParams p;
  std::vector<Spot> spots{spot(0, true, false), spot(4, true, true)};
  spots.push_back(spot(9, false, false));
  spots[2].known = false;      // 一个无关的位没量成
  const auto v = FD::judge_sanctuary(spots, p);
  EXPECT_TRUE(v.fulfilled);
  EXPECT_FALSE(v.valid) << "只要有一位没量成，就如实报不可信（哪怕它不影响结论）";
  EXPECT_EQ(v.unknown_count, 1);
}

TEST(SanctuaryTest, 便利包装丢掉了可信度信息)
{
  // ⚠️ `sanctuary_mandate_fulfilled` 只是 `judge_sanctuary(...).fulfilled` 的包装 ——
  //    它在 `valid == false` 时**照样可能返回 true**。
  //    ⇒ 生产代码该用 `judge_sanctuary`，别用这个（头文件里写了）。
  FD::SanctuaryParams p;
  std::vector<Spot> spots{spot(0, true, false), spot(4, true, true), spot(9, false, false)};
  spots[2].known = false;
  EXPECT_TRUE(FD::sanctuary_mandate_fulfilled(spots, p));
  EXPECT_FALSE(FD::judge_sanctuary(spots, p).valid)
      << "同一个输入：bool 说达成，但结论不可信 —— 这就是为什么别只用 bool";
}

// ═══════════════════════════════════════════════════════════════════════════
// 持握超限（规则 4.4.2：BR 限持 2 块）
// ═══════════════════════════════════════════════════════════════════════════

TEST(HoldingTest, 三块在半径内算超限)
{
  FD::HoldingParams p;             // 半径 0.6, 上限 2
  std::vector<Held> objs{
      {0.1, 0.0, true}, {0.2, 0.0, true}, {0.3, 0.0, true},
  };
  EXPECT_TRUE(FD::holding_over_limit(0.0, 0.0, objs, p));
}

TEST(HoldingTest, 恰好两块不算超限)
{
  // ⚠️ **严格大于**：规则原文是"限持 2 块" ⇒ 2 块合法
  FD::HoldingParams p;
  std::vector<Held> objs{{0.1, 0.0, true}, {0.2, 0.0, true}};
  EXPECT_FALSE(FD::holding_over_limit(0.0, 0.0, objs, p))
      << "恰好等于上限不该判超限（改写成 >= 会在这里红）";
}

TEST(HoldingTest, 未持握的不计入)
{
  // 半径内有 5 个方块，但只有 2 个是 is_held ⇒ 不超限
  FD::HoldingParams p;
  std::vector<Held> objs{
      {0.1, 0.0, true}, {0.2, 0.0, true},
      {0.3, 0.0, false}, {0.4, 0.0, false}, {0.5, 0.0, false},
  };
  EXPECT_FALSE(FD::holding_over_limit(0.0, 0.0, objs, p));
}

TEST(HoldingTest, 半径外的不计入)
{
  FD::HoldingParams p;             // 半径 0.6
  std::vector<Held> objs{
      {0.1, 0.0, true}, {0.2, 0.0, true},
      {1.0, 0.0, true},              // 太远，不是这台 BR 拿的
  };
  EXPECT_FALSE(FD::holding_over_limit(0.0, 0.0, objs, p));
}

TEST(HoldingTest, 边界恰好等于半径算在范围内)
{
  FD::HoldingParams p;
  p.hold_check_radius_m = 0.6;
  std::vector<Held> objs{
      {0.6, 0.0, true}, {0.6, 0.0, true}, {0.6, 0.0, true},   // 三个都恰好压线
  };
  EXPECT_EQ(FD::held_within(0.0, 0.0, objs, p.hold_check_radius_m), 3)
      << "边界非严格（<=）";
}

TEST(HoldingTest, 持握计数)
{
  FD::HoldingParams p;
  std::vector<Held> objs{{0.1, 0.0, true}, {0.2, 0.0, false}, {0.3, 0.0, true}};
  EXPECT_EQ(FD::held_within(0.0, 0.0, objs, p.hold_check_radius_m), 2);
  EXPECT_EQ(FD::held_within(5.0, 5.0, objs, p.hold_check_radius_m), 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 终场告警（规则 4.2.2）
// ═══════════════════════════════════════════════════════════════════════════

TEST(MatchEndTest, 剩五秒且有持握时告警)
{
  FD::MatchEndParams p;            // 180s / 窗口 5s
  EXPECT_TRUE(FD::match_end_release_warning(175.0, 1, p));   // 剩 5.0
  EXPECT_TRUE(FD::match_end_release_warning(178.0, 2, p));   // 剩 2.0
}

TEST(MatchEndTest, 还早时不告警)
{
  FD::MatchEndParams p;
  EXPECT_FALSE(FD::match_end_release_warning(170.0, 3, p));  // 剩 10
}

TEST(MatchEndTest, 没有持握目标时不告警)
{
  // ⚠️ **这条是防噪音**：没拿东西就没什么可松手的，发了只会让主控在最忙时多处理一条
  FD::MatchEndParams p;
  EXPECT_FALSE(FD::match_end_release_warning(179.0, 0, p));
  EXPECT_FALSE(FD::match_end_release_warning(200.0, 0, p));
}

TEST(MatchEndTest, 超时后仍有持握则持续告警)
{
  FD::MatchEndParams p;
  EXPECT_TRUE(FD::match_end_release_warning(181.0, 1, p));
}

TEST(MatchEndTest, 剩余时间可以报负不钳零)
{
  // ⚠️ 与仓库对"时间是负的"的一贯口径一致：负值本身就是要暴露的信号
  FD::MatchEndParams p;
  EXPECT_DOUBLE_EQ(FD::remaining_sec(175.0, p), 5.0);
  EXPECT_DOUBLE_EQ(FD::remaining_sec(185.0, p), -5.0);
  EXPECT_LT(FD::remaining_sec(185.0, p), 0.0);
}

TEST(MatchEndTest, 边界剩余恰好等于窗口时开始告警)
{
  // ⚠️ 方向是"宁早不晚" —— 松手是个动作，晚一秒就来不及
  FD::MatchEndParams p;
  p.match_duration_sec = 180.0;
  p.release_window_sec = 5.0;
  EXPECT_TRUE(FD::match_end_release_warning(175.0, 1, p))
      << "恰好剩 5.0s 就该开始告警";
}

// ═══════════════════════════════════════════════════════════════════════════
// 出界（规则 6.4：永久移除，**不可逆**）
// ═══════════════════════════════════════════════════════════════════════════

TEST(BoundsTest, 场地内不算出界)
{
  FD::BoundsParams p;              // 11m 场地, 宽容 0.1
  EXPECT_FALSE(FD::out_of_bounds(5.0, 5.0, p));
  EXPECT_FALSE(FD::out_of_bounds(0.0, 0.0, p));
  EXPECT_FALSE(FD::out_of_bounds(11.0, 11.0, p));
}

TEST(BoundsTest, 压线在宽容带内不算出界)
{
  // ⚠️⚠️ **这条是防"永久移除"误伤**：一台**贴着围栏跑**的 BR
  //    会因标定/噪声偶尔报出 -0.05 这种值。没有宽容带 ⇒ 每帧都判出界
  //    ⇒ 3 秒后**永久移除**，而它其实一直在界内。
  FD::BoundsParams p;              // 宽容 0.10
  EXPECT_FALSE(FD::out_of_bounds(-0.05, 5.0, p)) << "轻微压线不该判出界";
  EXPECT_FALSE(FD::out_of_bounds(11.05, 5.0, p));
  EXPECT_FALSE(FD::out_of_bounds(5.0, -0.10, p)) << "恰好压在宽容带边界上算出界内";
}

TEST(BoundsTest, 明显出界才算)
{
  FD::BoundsParams p;
  EXPECT_TRUE(FD::out_of_bounds(-0.5, 5.0, p));
  EXPECT_TRUE(FD::out_of_bounds(5.0, 12.0, p));
}

TEST(BoundsTest, 回界内立刻清零不是衰减)
{
  // ⚠️ 判据是"**连续**出界 N 秒"。做成衰减的话，
  //    "反复进出边界"的目标会把累计值攒到阈值 ⇒ 被误移除。
  FD::BoundsParams p;
  double t = 0.0;
  t = FD::step_out_of_bounds(t, false, 2.0);   // 出界 2 秒
  EXPECT_DOUBLE_EQ(t, 2.0);
  t = FD::step_out_of_bounds(t, true, 0.1);    // 回界内一下
  EXPECT_DOUBLE_EQ(t, 0.0) << "必须清零，不能衰减";
  t = FD::step_out_of_bounds(t, false, 1.0);   // 再出界 1 秒 —— 只剩 1 秒，不该移除
  EXPECT_DOUBLE_EQ(t, 1.0);
  EXPECT_FALSE(FD::should_forget(t, p));
}

TEST(BoundsTest, 连续出界达阈值才移除)
{
  FD::BoundsParams p;              // 3.0s
  double t = 0.0;
  for (int i = 0; i < 29; ++i) t = FD::step_out_of_bounds(t, false, 0.1);  // 2.9s
  EXPECT_FALSE(FD::should_forget(t, p));
  t = FD::step_out_of_bounds(t, false, 0.1);   // 3.0s
  EXPECT_TRUE(FD::should_forget(t, p)) << "达到阈值就该移除（非严格 >=）";
}

TEST(BoundsTest, 非正dt不推进计时)
{
  FD::BoundsParams p;
  EXPECT_DOUBLE_EQ(FD::step_out_of_bounds(1.5, false, 0.0), 1.5);
  EXPECT_DOUBLE_EQ(FD::step_out_of_bounds(1.5, false, -1.0), 1.5);
}

// ═══════════════════════════════════════════════════════════════════════════
// ⑤ 共享区停留（规则 6.6 —— ⚠️ **启发式，不是硬性时限**）
// ═══════════════════════════════════════════════════════════════════════════

namespace {
FD::Zone zone(double x0, double x1, double y0, double y1)
{
  FD::Zone z;
  z.x_min = x0; z.x_max = x1; z.y_min = y0; z.y_max = y1;
  return z;
}
}  // namespace

TEST(DwellTest, 区内区外判定)
{
  const auto z = zone(4.0, 7.0, 4.0, 7.0);   // L2 共享区（field_geometry 的真值）
  EXPECT_TRUE(FD::in_zone(5.5, 5.5, z));
  EXPECT_TRUE(FD::in_zone(4.0, 4.0, z)) << "边界非严格，贴着边缘算在内";
  EXPECT_TRUE(FD::in_zone(7.0, 7.0, z));
  EXPECT_FALSE(FD::in_zone(3.9, 5.5, z));
  EXPECT_FALSE(FD::in_zone(5.5, 7.1, z));
}

TEST(DwellTest, 连续停留才累加_离开就清零)
{
  // ⚠️ **与出界同款**：判据是"**连续**停留"。衰减的话，
  //    "反复进出共享区"的目标会把累计值攒到阈值 —— 那不是"停留"。
  FD::DwellParams p;
  double t = 0.0;
  for (int i = 0; i < 30; ++i) t = FD::step_dwell(t, true, 0.1);   // 连续 3 秒
  EXPECT_NEAR(t, 3.0, 1e-9);
  t = FD::step_dwell(t, false, 0.1);                               // 出去一下
  EXPECT_DOUBLE_EQ(t, 0.0) << "离开必须清零";
}

TEST(DwellTest, 非正dt不推进)
{
  EXPECT_DOUBLE_EQ(FD::step_dwell(2.0, true, 0.0), 2.0);
  EXPECT_DOUBLE_EQ(FD::step_dwell(2.0, true, -1.0), 2.0);
}

TEST(DwellTest, 超阈值给的是提示不是判罚)
{
  // ⚠️⚠️ **这条钉的是语义，不是数值**：规格 §4.5 的 v1.6 修订明写
  //    「dwell 计时器保留，注释改为"**启发式，非规则 6.6 硬性规定**"」，
  //    而规则 6.6 **没有 5 秒硬性时限、由裁判判定**。
  //    ⇒ 函数名是 `dwell_over_hint`（**hint**），下游不许拿它当判罚依据。
  FD::DwellParams p;                 // 默认 5.0s
  EXPECT_FALSE(FD::dwell_over_hint(4.9, p));
  EXPECT_TRUE(FD::dwell_over_hint(5.0, p)) << "非严格（>=）";
  EXPECT_TRUE(FD::dwell_over_hint(9.0, p));
}

}  // namespace fusion
}  // namespace br_perception
