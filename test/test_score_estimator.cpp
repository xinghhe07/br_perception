/// ═══════════════════════════════════════════════════════════════════════════════
/// test_score_estimator.cpp — 模块 29 实时得分估计：**纯规则**单元测试
///
/// ⚠️ 规格 §4.7 原文：「**仅作场下/赛中策略参考，不上报裁判**」⇒ 它算错**不影响比赛进程**。
///    但那也是它值得写对的原因：**拿错的分数去决定"还追不追分"是自欺**。
///
/// ── 最值钱的几条 ──
///   · **`state == 4 (UNKNOWN)` 一分都不该计入** —— 合 `BuildingSpot.msg` 的
///     「`4` 与 `0` 必须分开」：当"空"会**低估自己的分**、当"建成"会**高估**，两件都是自欺；
///   · **穆斯蒂卡 250 分需圣所使命达成** —— 只判"球到位"是**最贵的错**：
///     那 250 分拿不到，而策略层会以为拿到了；
///   · **L1/L2 分值差一倍** —— 拿 `state` 当唯一输入的实现会把 L2 的塔算成 L1 的分。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=ScoreTest.*`）。
/// ⚠️ 全部用例确定性：手造 POD、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/fusion/score_estimator.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace br_perception {
namespace fusion {

using SE = ScoreEstimator;
using Spot = ScoreEstimator::SpotInput;

namespace {

constexpr std::uint8_t kEmpty = 0, kOne = 1, kTwo = 2, kFull = 3, kUnknown = 4;
constexpr std::uint8_t kNone = 0, kRed = 1, kBlue = 2, kColorUnknown = 3;

Spot spot(std::uint32_t id, std::uint8_t state, std::uint8_t color,
          bool l2 = false, float conf = 1.0f)
{
  Spot s;
  s.id = id; s.state = state; s.top_color = color; s.is_l2 = l2; s.confidence = conf;
  return s;
}

/// 默认参数下一分不给的调用（各用例只关心自己要验的那几项）
SE::Verdict run(const std::vector<Spot>& spots, bool sanctuary = true,
                bool mustika = false, int rt = 0, int bt = 0)
{
  SE::Params p;
  return SE::score(spots, sanctuary, mustika, rt, bt, p);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 计分表
// ═══════════════════════════════════════════════════════════════════════════

TEST(ScoreTest, L1塔底中顶是10_20_40)
{
  SE::Params p;
  EXPECT_EQ(SE::spot_points(kEmpty, false, p), 0);
  EXPECT_EQ(SE::spot_points(kOne,   false, p), 10);
  EXPECT_EQ(SE::spot_points(kTwo,   false, p), 20);
  EXPECT_EQ(SE::spot_points(kFull,  false, p), 40);
}

TEST(ScoreTest, L2塔是20_40_80)
{
  // ⚠️ 拿 `state` 当唯一输入的实现会把 L2 的塔按 L1 算 ⇒ **少算一半的分**
  //    ⇒ 策略层以为落后，实际上领先。这条钉的就是"平台层必须进判定"。
  SE::Params p;
  EXPECT_EQ(SE::spot_points(kOne,  true, p), 20);
  EXPECT_EQ(SE::spot_points(kTwo,  true, p), 40);
  EXPECT_EQ(SE::spot_points(kFull, true, p), 80);
}

TEST(ScoreTest, 完整塔按颜色分别计入)
{
  const auto v = run({spot(0, kFull, kRed), spot(1, kFull, kBlue)});
  EXPECT_EQ(v.red_score, 40);
  EXPECT_EQ(v.blue_score, 40);
  EXPECT_EQ(v.red_towers, 1);
  EXPECT_EQ(v.blue_towers, 1);
}

TEST(ScoreTest, L2的塔按双倍计入总分)
{
  const auto v = run({spot(0, kFull, kRed, /*l2=*/true)});
  EXPECT_EQ(v.red_score, 80) << "L2 完整塔值 80 不是 40";
  EXPECT_EQ(v.red_towers, 1);
}

TEST(ScoreTest, 空位是确定性的零分不标不确定)
{
  // ⚠️ "量到了、这里是空的"是**确定信息** ⇒ 不该让总分变"不确定"
  const auto v = run({spot(0, kEmpty, kNone)});
  EXPECT_EQ(v.red_score, 0);
  EXPECT_FALSE(v.uncertain);
  EXPECT_EQ(v.uncounted_spots, 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// ★ 「没量成」与「归属没判出来」
// ═══════════════════════════════════════════════════════════════════════════

TEST(ScoreTest, 没量成的位一分不计且如实报出来)
{
  // ⚠️⚠️ **本文件最贵的一条**（合 `BuildingSpot.msg` 的「`4` 与 `0` 必须分开」）：
  //    当"空"  ⇒ **低估自己的分**（白白去追分）；
  //    当"建成" ⇒ **高估**（不去追）。
  //    两件都是自欺 ⇒ 不算，并让调用方知道有位没量成。
  const auto v = run({spot(0, kUnknown, kRed), spot(1, kFull, kRed)});
  EXPECT_EQ(v.red_score, 40) << "只有那 1 座完整塔该计分";
  EXPECT_EQ(v.red_towers, 1);
  EXPECT_EQ(v.unknown_spots, 1) << "没量成的位要单独计数";
  EXPECT_EQ(v.uncounted_spots, 1);
  EXPECT_TRUE(v.uncertain) << "有位没量成 ⇒ 总分不可全信";
}

TEST(ScoreTest, 有塔但颜色判不出来也不计分)
{
  // ⚠️ 这不是"没量成"（塔是量到了的），是"**归属没判出来**" ⇒ 单独归到 uncounted。
  //    把"未知颜色"当成"红"会**凭空给一方加分** ⇒ 策略判反。
  const auto v = run({spot(0, kFull, kColorUnknown), spot(1, kFull, kRed)});
  EXPECT_EQ(v.red_score, 40);
  EXPECT_EQ(v.red_towers, 1) << "颜色未知那位的塔不该算进任何一方";
  EXPECT_EQ(v.unknown_spots, 0) << "它不是'没量成'——塔量到了";
  EXPECT_EQ(v.uncounted_spots, 1) << "它是'归属判不出来'";
  EXPECT_TRUE(v.uncertain);
}

TEST(ScoreTest, 颜色为空也不计分)
{
  const auto v = run({spot(0, kFull, kNone)});
  EXPECT_EQ(v.red_score, 0);
  EXPECT_EQ(v.blue_score, 0);
  EXPECT_EQ(v.uncounted_spots, 1);
}

TEST(ScoreTest, 颜色有效性判据)
{
  EXPECT_TRUE(SE::color_is_valid(kRed));
  EXPECT_TRUE(SE::color_is_valid(kBlue));
  EXPECT_FALSE(SE::color_is_valid(kNone)) << "0 是'无'，不是一种归属";
  EXPECT_FALSE(SE::color_is_valid(kColorUnknown));
}

TEST(ScoreTest, 没量成的state不越界索引)
{
  // ⚠️⚠️ `spot_points` 用 `state` 当数组下标，而下标只有 0~3。
  //    拿着 `4` 去索引就是**越界读**（UB）—— 直接调一次把它钉住。
  SE::Params p;
  EXPECT_EQ(SE::spot_points(kUnknown, false, p), 0);
  EXPECT_EQ(SE::spot_points(kUnknown, true, p), 0);
  EXPECT_EQ(SE::spot_points(255, false, p), 0) << "任何越界 state 都不该读数组";
}

TEST(ScoreTest, 低置信度的项让总分标不确定)
{
  // 规格："spot confidence 逐项传导，< 0.5 标记不确定"
  SE::Params p;
  const auto ok = run({spot(0, kFull, kRed, false, 0.9f)});
  EXPECT_FALSE(ok.uncertain);
  const auto low = run({spot(0, kFull, kRed, false, 0.4f)});
  EXPECT_TRUE(low.uncertain) << "低于阈值 ⇒ 总分不可全信";
  EXPECT_EQ(low.red_score, 40) << "但分照算（标不确定，不是不算）";
}

// ═══════════════════════════════════════════════════════════════════════════
// ★★ 穆斯蒂卡 250 分：**需圣所使命达成**（规则 4.5.1）
// ═══════════════════════════════════════════════════════════════════════════

TEST(ScoreTest, 穆斯蒂卡需圣所使命达成才计)
{
  // ⚠️⚠️ **最贵的错**：只判"球到位"就计那 250 分 ⇒
  //    实际拿不到，而**策略层会以为拿到了**（然后不去做前置）。
  const auto not_yet = run({}, /*sanctuary=*/false, /*mustika=*/true);
  EXPECT_FALSE(not_yet.mustika_scored) << "球到位但前置没达成 ⇒ 不计";

  const auto ok = run({}, /*sanctuary=*/true, /*mustika=*/true);
  EXPECT_TRUE(ok.mustika_scored);
}

TEST(ScoreTest, 球没到位时不计就算前置达成)
{
  const auto v = run({}, /*sanctuary=*/true, /*mustika=*/false);
  EXPECT_FALSE(v.mustika_scored);
}

TEST(ScoreTest, 穆斯蒂卡不猜归属)
{
  // ⚠️ 球算谁的要靠"我方/对方"，而那是**本模块不猜**的东西（类注释的 ⚠️⚠️）。
  //    ⇒ 它只标"这 250 分该计了"，**不计入任何一方**。
  const auto v = run({}, true, true);
  EXPECT_TRUE(v.mustika_scored);
  EXPECT_EQ(v.red_score, 0) << "不该凭空算到红队头上";
  EXPECT_EQ(v.blue_score, 0) << "也不该算到蓝队头上";
}

// ═══════════════════════════════════════════════════════════════════════════
// 转运
// ═══════════════════════════════════════════════════════════════════════════

TEST(ScoreTest, 转运每块五分按颜色计)
{
  const auto v = run({}, true, false, /*rt=*/3, /*bt=*/1);
  EXPECT_EQ(v.red_score, 15);
  EXPECT_EQ(v.blue_score, 5);
  EXPECT_EQ(v.red_transferred, 3);
  EXPECT_EQ(v.blue_transferred, 1);
}

TEST(ScoreTest, 转运与建筑位分叠加)
{
  const auto v = run({spot(0, kFull, kRed, true)}, true, false, 2, 0);
  EXPECT_EQ(v.red_score, 80 + 10) << "L2 完整塔 80 + 转运 2 块 × 5";
}

// ═══════════════════════════════════════════════════════════════════════════
// 综合
// ═══════════════════════════════════════════════════════════════════════════

TEST(ScoreTest, 真实布局一整套)
{
  // L1 两座红塔(40×2) + L2 一座蓝塔(80) + 转运红 2 块(10) + 一位没量成
  const auto v = run({
      spot(0, kFull, kRed, false),
      spot(4, kFull, kRed, false),
      spot(6, kFull, kBlue, true),
      spot(7, kUnknown, kNone, true),
  }, /*sanctuary=*/true, /*mustika=*/true, /*rt=*/2, /*bt=*/0);

  EXPECT_EQ(v.red_score, 80 + 10);
  EXPECT_EQ(v.blue_score, 80);
  EXPECT_EQ(v.red_towers, 2);
  EXPECT_EQ(v.blue_towers, 1);
  EXPECT_EQ(v.unknown_spots, 1);
  EXPECT_TRUE(v.uncertain);
  EXPECT_TRUE(v.mustika_scored);
}

TEST(ScoreTest, 空场全零)
{
  const auto v = run({}, false, false);
  EXPECT_EQ(v.red_score, 0);
  EXPECT_EQ(v.blue_score, 0);
  EXPECT_EQ(v.unknown_spots, 0);
  EXPECT_FALSE(v.uncertain);
  EXPECT_FALSE(v.mustika_scored);
}

}  // namespace fusion
}  // namespace br_perception
