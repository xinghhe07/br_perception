#pragma once

#include <cstdint>
#include <vector>

namespace br_perception {
namespace fusion {

/**
 * @brief 实时得分估计（模块 29，规则 8.1/8.2/8.5）
 *
 * ⚠️ 规格 §4.7 原文：「**仅作场下/赛中策略参考，不上报裁判**」
 *    ⇒ 它算错**不影响比赛进程**，只影响我们自己的策略判断。
 *    （但那也是它值得写对的原因：**拿错的分数去决定"还追不追分"是自欺**。）
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ⚠️⚠️ **一处与规格的有意偏差：按【颜色】算分，不按"我方/对方"**
 *
 * 规格 §4.7 写「归属语义：地球方块归属 = **我方**放置；天空方块归属 = 当前朝上色」。
 * 但**仓库里根本没有"我们是哪一队"这个配置** —— 这一点模块 27 的 `ownership`
 * 字段已经踩过（台账 §二 模块 27 行原文）：
 * > `ownership`（我方/对方）**没做** —— 仓库里没有"我们是哪一队"的配置，
 * > **靠猜会把归属判反**，那是决策层该给输入的事
 *
 * ⇒ 本模块**按颜色分开算**（红队分 / 蓝队分），**不由它去猜谁是"我方"**。
 *    ⚠️ 谁把颜色映射到队伍，是**上层的事**（决策层 / launch 参数）——
 *    在这里猜一次，就会和别处的猜测打架。
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * 计分表（规格 §4.7 原文）
 * ```
 *   L1 塔  底/中/顶 = 10 / 20 / 40 分
 *   L2 塔  底/中/顶 = 20 / 40 / 80 分      ← 同一塔在 L2 值双倍
 *   转运   每块 5 分（永久归队）
 *   穆斯蒂卡 250 分（**需圣所使命达成**）
 * ```
 * 「底/中/顶」对应 `BuildingSpot.state` 的 `1 / 2 / 3`
 * （`ONE_EARTH` / `TWO_EARTH` / `FULL_TOWER`）。
 */
class ScoreEstimator
{
public:
  // ══════════════════════════════════════════════════════════════════════
  // 输入
  // ══════════════════════════════════════════════════════════════════════

  /// 一个建筑位的计分输入
  struct SpotInput
  {
    std::uint32_t id{0};
    /// `BuildingSpot.state`：0=空 1=1个地球 2=2个地球 3=完整塔 4=未知
    std::uint8_t state{0};
    /// `BuildingSpot.top_color`：0=无 1=红 2=蓝 3=未知
    /// ⚠️ 它决定**这个位算到谁头上**（天空方块会被对手翻 ⇒ 归属会变）
    std::uint8_t top_color{0};
    /// 该位是不是 **L2**（平台高 0.9）⇒ 分值翻倍。
    /// ⚠️ 由调用方按 `field_geometry.yaml` 的 `building_spot_z` 填（0.6=L1, 0.9=L2）
    bool is_l2{false};
    float confidence{1.0f};
  };

  struct Params
  {
    /// ⚠️ 下标 = `state`（0~3）；**`state == 4 (UNKNOWN) 不查表**，见 `score()` 的 ⚠️⚠️
    int l1_points[4] = {0, 10, 20, 40};   ///< L1: 空 / 1个地球 / 2个地球 / 完整塔
    int l2_points[4] = {0, 20, 40, 80};   ///< L2: 同形状，分值翻倍
    int transfer_per_block{5};            ///< 转运到位的方块，每块
    int mustika_points{250};              ///< 穆斯蒂卡（需圣所使命达成）
    /// ⚠️ **低于它就标"不确定"**（规格："spot confidence 逐项传导，< 0.5 标记不确定"）
    float uncertain_confidence{0.5f};
  };

  // ══════════════════════════════════════════════════════════════════════
  // 输出
  // ══════════════════════════════════════════════════════════════════════

  struct Verdict
  {
    int red_score{0};
    int blue_score{0};
    /// 完整塔数（按颜色分）
    int red_towers{0};
    int blue_towers{0};
    /// 转运到位计入的方块数（按颜色分）
    int red_transferred{0};
    int blue_transferred{0};
    /// 穆斯蒂卡那 250 分**算上了没有**（取决于圣所使命是否达成）
    bool mustika_scored{false};
    /// ⚠️ **有低置信度或"没量成"的项** ⇒ 总分**不可全信**。
    ///    规格："spot confidence 逐项传导，< 0.5 标记不确定"。
    bool uncertain{false};
    /// `state == UNKNOWN` 的位数（⚠️ 与 `uncertain` 分开：这是"没量成"，那是"量了但没把握"）
    int unknown_spots{0};
    /// 既没量成、颜色也未知的位 —— 它们**一分都没被计入**
    int uncounted_spots{0};
  };

  // ══════════════════════════════════════════════════════════════════════
  // 计分
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 统计得分
  /// @param spots           建筑位
  /// @param sanctuary_ok    **圣所使命是否达成**（由 21 的 `judge_sanctuary` 给）
  /// @param mustika_taken   穆斯蒂卡球是否已被取走/到位
  /// @param red_transfer    红队已转运到位的方块数（⚠️ 来自转运区到达事件，不是本模块算的）
  /// @param blue_transfer   蓝队
  ///
  /// ⚠️⚠️ **`state == 4 (UNKNOWN)` 的位一分都不计入**，且会把 `uncounted_spots` 加一 ——
  ///    合 `BuildingSpot.msg` 的「`4` 与 `0` 必须分开」：把"没量成"当"空"会**低估自己的分**，
  ///    当"建成"会**高估**。两件都是自欺，所以**不算，并如实报出来**。
  [[nodiscard]] static Verdict score(
      const std::vector<SpotInput>& spots,
      bool sanctuary_ok,
      bool mustika_taken,
      int red_transfer,
      int blue_transfer,
      const Params& params);

  /// @brief 单个建筑位值多少分（`state` 与平台层决定）
  /// ⚠️ `state == 4 (UNKNOWN)` ⇒ 返回 **0**，但这**不是**"它是空的"——
  ///    调用方要自己看 `state` 决定要不要标"不确定"。见 `score()` 的 ⚠️⚠️。
  [[nodiscard]] static int spot_points(std::uint8_t state, bool is_l2,
                                       const Params& params);

  /// @brief 颜色 → 是不是"有效归属"（0=无 与 3=未知 都不算有效）
  [[nodiscard]] static bool color_is_valid(std::uint8_t top_color);
};

}  // namespace fusion
}  // namespace br_perception
