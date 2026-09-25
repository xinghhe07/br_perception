#include "br_perception/fusion/score_estimator.hpp"

namespace br_perception {
namespace fusion {

namespace {

/// `BuildingSpot.state` / `top_color` 的取值（**与 msg 逐字对应**）
constexpr std::uint8_t kStateEmpty     = 0;
constexpr std::uint8_t kStateFullTower = 3;
constexpr std::uint8_t kStateUnknown   = 4;

constexpr std::uint8_t kColorNone    = 0;
constexpr std::uint8_t kColorRed     = 1;
constexpr std::uint8_t kColorBlue    = 2;
constexpr std::uint8_t kColorUnknown = 3;

}  // namespace

bool ScoreEstimator::color_is_valid(std::uint8_t top_color)
{
  // ⚠️ **`0 = 无` 与 `3 = 未知` 都**不是**有效归属** —— 它们不能算到任何一方头上。
  //    把"未知"当成"我方"会**高估自己**（然后策略上误判"不用追了"）。
  return top_color == kColorRed || top_color == kColorBlue;
}

int ScoreEstimator::spot_points(std::uint8_t state, bool is_l2, const Params& params)
{
  // ⚠️⚠️ **`state == 4 (UNKNOWN)` 不查表，直接 0**。
  //    下标只有 0~3；若拿着 4 去索引数组就是**越界读**（UB）。
  //    而"没量成"本来就**不该计分**（见 `score()` 的 ⚠️⚠️）⇒ 返回 0 是安全且正确的。
  if (state > kStateFullTower) return 0;
  return is_l2 ? params.l2_points[state] : params.l1_points[state];
}

ScoreEstimator::Verdict ScoreEstimator::score(
    const std::vector<SpotInput>& spots,
    bool sanctuary_ok,
    bool mustika_taken,
    int red_transfer,
    int blue_transfer,
    const Params& params)
{
  Verdict v;

  for (const auto& s : spots) {
    // ── ⚠️⚠️ "没量成"的位：一分不计，但**如实报出来** ──
    //    合 `BuildingSpot.msg` 的「`4` 与 `0` 必须分开」：
    //    把"没量成"当"空" ⇒ **低估自己的分**（然后白白去追分）；
    //    当"建成"       ⇒ **高估**（然后不去追）。
    //    两件都是自欺 ⇒ 不算，并让调用方知道有位没量成。
    if (s.state == kStateUnknown) {
      ++v.unknown_spots;
      ++v.uncounted_spots;
      v.uncertain = true;
      continue;
    }
    if (s.state == kStateEmpty) continue;      // **量到了、这里是空的** ⇒ 0 分（确定）

    // ── 归属：**按颜色**，不猜"我方"（见类注释的 ⚠️⚠️）──
    if (!color_is_valid(s.top_color)) {
      // ⚠️ 有塔但**不知道算谁的**（颜色没判出来）⇒ 也一分不计。
      //    这不是"没量成"（塔是量到了的），是"归属没判出来" ⇒ 单独计 uncounted。
      ++v.uncounted_spots;
      v.uncertain = true;
      continue;
    }

    const int pts = spot_points(s.state, s.is_l2, params);
    if (s.top_color == kColorRed) {
      v.red_score += pts;
      if (s.state == kStateFullTower) ++v.red_towers;
    } else {   // kColorBlue
      v.blue_score += pts;
      if (s.state == kStateFullTower) ++v.blue_towers;
    }

    // ⚠️ **置信度逐项传导**：任何一项低于阈值 ⇒ 总分标"不确定"（规格原话）。
    if (s.confidence < params.uncertain_confidence) v.uncertain = true;
  }

  // ── 转运：每块 5 分（**永久归队** ⇒ 只加不减）──
  if (red_transfer > 0) {
    v.red_score += red_transfer * params.transfer_per_block;
    v.red_transferred = red_transfer;
  }
  if (blue_transfer > 0) {
    v.blue_score += blue_transfer * params.transfer_per_block;
    v.blue_transferred = blue_transfer;
  }

  // ── 穆斯蒂卡 250 分：**需圣所使命达成**（规则 4.5.1）──
  //    ⚠️ 两个条件**同时**成立才计：球到位 **且** 前置达成。
  //       只判球到位是**最贵的错**：那 250 分拿不到，而策略层会以为拿到了。
  if (mustika_taken && sanctuary_ok) {
    // ⚠️ 球算谁的要靠"我方/对方" —— 而那正是本模块**不猜**的东西（见类注释）。
    //    ⇒ 这里**只标记"这 250 分该计了"**，不计入任何一方；
    //      计入哪一方由上层按队伍配置决定。
    v.mustika_scored = true;
  }

  return v;
}

}  // namespace fusion
}  // namespace br_perception
