#pragma once

#include <cstdint>
#include <vector>

namespace br_perception {
namespace fusion {

/**
 * @brief 融合层的**判定规则**（模块 21 `fusion_node` 的纯逻辑部分）
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ⚠️ 本文件**只有规则，没有节点** —— 订阅/发布/调 18/19 由 `fusion_node` 做。
 *    理由与 18/19 同款（见 `kalman_tracker.hpp`）：**规则能用闭式解手算，节点不能**。
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ⚠️⚠️ **本文件里最值钱的是 `sanctuary_mandate_fulfilled` —— 它值 250 分**
 *
 * 规则 4.5.1：**TR 取五色石的前置是「圣所使命达成」**。
 * 而"柱上有没有球"（那 250 分）的判定又依赖它 ⇒ 这个 bool 判错，
 * **要么白丢 250 分，要么让主控去做一个规则上不允许的动作**。
 *
 * 判据（任务书 §4.5 v1.6 修正后）：
 * > 场上已有 **≥2 座完整塔**，且其中 **≥1 座位于共享区**
 *
 * ⚠️ **v1.6 修正「地面共享区无建筑位」** —— 所以参与判定的共享区建筑位只有：
 *    · **L1 共享区**：2 个位 `(5.5, 2.8)` / `(5.5, 8.2)`，平台高 0.6
 *    · **L2**：4 个位 `(4.25/6.75, 4.25/6.75)`，平台高 0.9 —— **4 个全在 L2 共享区内**（x,y∈[4,7]）
 *    ⇒ 共 **6 个**共享区建筑位（场地真值见 `config/field_geometry.yaml` 第 11 节）。
 *    ⚠️ **"在哪个区"由调用方填 `in_shared_zone`** —— 本文件**不读 yaml**（保持纯函数可测）。
 *       21 接线时按上表把 10 个位映射进来。
 */
class FusionDecisions
{
public:
  // ══════════════════════════════════════════════════════════════════════
  // ① 圣所使命（**值 250 分**）
  // ══════════════════════════════════════════════════════════════════════

  /// 一个建筑位的**判定输入**（只取判定用得着的字段）
  struct SpotState
  {
    std::uint32_t id{0};
    /// 该位上是否是**完整塔**（雷达给的塔高判出来的，本文件不重复判）
    bool complete{false};
    /// 该位是否**共享区建筑位**（见类注释的 6 个位；由 21 按 `field_geometry.yaml` 填）
    bool in_shared_zone{false};
    /// ⚠️⚠️ **该位这一帧"量成了"吗** —— `BuildingSpot.state != 4 (UNKNOWN)`。
    ///
    /// 为什么它必须进来（`BuildingSpot.msg` 的 ⚠️⚠️ 原文）：
    /// > `0` = 量到了、这里是空的；`4` = **没量成** ⇒ **两件事的动作完全不同**。
    ///
    /// 对一个**Gate 着 250 分**的判定来说，"没量成"的位有三种处理方式，**只有一种是诚实的**：
    ///   · 当成"不是完整塔" ⇒ 会**凭空否掉**一座可能存在的塔 ⇒ 白丢 250 分；
    ///   · 当成"是完整塔"   ⇒ 会**凭空造出**一座塔 ⇒ 让主控去做规则不允许的动作；
    ///   · **不算数，但把结论标成"不可信"** ✅ —— 这就是 `known` 的用途。
    bool known{true};
  };

  struct SanctuaryParams
  {
    /// 至少几座完整塔（规则 4.5.1：≥2）
    int min_complete_towers{2};
    /// 其中至少几座在共享区（规则 4.5.1：≥1）
    int min_in_shared_zone{1};
  };

  /// 圣所判定的**完整结论**（不只是那个 bool）
  struct SanctuaryVerdict
  {
    /// 是否达成（**只按量成了的位算**，见 `valid`）
    bool fulfilled{false};
    /// ⚠️ **这个结论可不可信** —— 只要有一个位 `known == false`，就是 `false`。
    ///
    /// ⇒ 下游（`perception_pipeline` → `PerceptionFrame.sanctuary_valid`）据此决定
    ///   **要不要把这个 bool 当成事实发给主控**。
    ///   ⚠️ 合 §零「准」轴：**"未知"要有哨兵**，不许把一个基于不完整数据的结论
    ///   当成完整数据的结果送出去。
    bool valid{false};
    int complete_total{0};      ///< 量成了、且是完整塔的位数
    int complete_in_shared{0};  ///< 其中在共享区的
    int unknown_count{0};       ///< 没量成的位数（> 0 ⇒ `valid == false`）
  };

  /// @brief 圣所判定的**完整结论**（推荐用这个，它带"可不可信"）
  ///
  /// ⚠️ **两个条件是与关系，且都要满足**：
  ///    ① 完整塔**总数** ≥ `min_complete_towers`
  ///    ② 其中在**共享区**的 ≥ `min_in_shared_zone`
  ///    ⇒ 2 座塔全在非共享位 = **未达成**（这是最容易错的一档：塔数够了就以为成了）。
  ///
  /// ⚠️ **只统计 `known == true` 的位**；`known == false` 的位**不算完整塔**（保守），
  ///    但会把 `valid` 置 false —— 见 `SpotState::known` 的 ⚠️⚠️。
  ///
  /// ⚠️ 本函数**不去重、不查重 id** —— 调用方保证 `spots` 是**每位一条**。
  ///    若同一个位被填两遍，会被算成两座塔。
  [[nodiscard]] static SanctuaryVerdict judge_sanctuary(
      const std::vector<SpotState>& spots, const SanctuaryParams& params);

  /// @brief 便利包装：只看那个 bool（**丢掉可信度信息**）
  /// ⚠️ **能用 `judge_sanctuary` 就别用这个** —— `valid == false` 时这个 bool
  ///    是个**基于不完整数据的结论**，直接发给主控就违反了 §零「准」轴。
  [[nodiscard]] static bool sanctuary_mandate_fulfilled(
      const std::vector<SpotState>& spots, const SanctuaryParams& params);

  // ══════════════════════════════════════════════════════════════════════
  // ② 「v1.6」持握超限（规则 4.4.2：BR 限持 2 块）
  // ══════════════════════════════════════════════════════════════════════

  struct HeldObject
  {
    double x{0.0}, y{0.0};
    bool is_held{false};
  };

  struct HoldingParams
  {
    /// 判定半径（m）：BR 位置这个半径内的**持握**方块才算它拿的
    double hold_check_radius_m{0.6};
    /// 持有上限（规则 4.4.2：2）
    int holding_limit_count{2};
  };

  /// @brief 某台 BR 是否**持握超限**（> `holding_limit_count`）
  /// @param br_x/br_y  这台 BR 的位置
  /// @param objects    场上所有**方块**（含未持握的；未持握的自然不计）
  ///
  /// ⚠️ 判据是 **严格大于** 上限：恰好 2 块**不算**超限（规则原文是"限持 2 块"）。
  /// ⚠️ 只用**水平距离**（`xy`）—— 持握的方块在夹爪里、可能悬空，
  ///    z 与 BR 中心差得远，带上 z 会把"拿着"判成"没拿"。
  [[nodiscard]] static bool holding_over_limit(double br_x, double br_y,
                                               const std::vector<HeldObject>& objects,
                                               const HoldingParams& params);

  /// @brief 该半径内**持握**方块的计数（诊断用，也是上面那条的原料）
  [[nodiscard]] static int held_within(double br_x, double br_y,
                                       const std::vector<HeldObject>& objects,
                                       double radius_m);

  // ══════════════════════════════════════════════════════════════════════
  // ③ 「v1.6」终场告警（规则 4.2.2）
  // ══════════════════════════════════════════════════════════════════════

  struct MatchEndParams
  {
    /// 比赛时长（规则 4.2.2 固定 180s）
    double match_duration_sec{180.0};
    /// 剩余多少秒内、且有持握目标 ⇒ 开始告警
    double release_window_sec{5.0};
  };

  /// @brief 是否该发「终场松手」告警
  /// @param elapsed_sec 本节点**自计时**的比赛已进行时长（⚠️ 见下）
  /// @param held_count  当前 `is_held` 的目标数
  ///
  /// ⚠️ **`elapsed_sec` 是感知层自计时**（规则 4.2.2 固定时长；心跳对时偏差 <1s）。
  ///    它**不是**个精密的量 —— 但对"提前 5 秒提醒松手"这个用途够用。
  /// ⚠️ 判据：剩余 `<= release_window_sec` **且** `held_count > 0`。
  ///    没有持握目标时**不发** —— 发了是噪音，会让主控在最忙的时候多处理一条。
  [[nodiscard]] static bool match_end_release_warning(double elapsed_sec,
                                                      int held_count,
                                                      const MatchEndParams& params);

  /// @brief 剩余时间（秒）。⚠️ **可以小于 0**（超时了就如实报负），不钳到 0 ——
  ///        与仓库对"时间是负的"的一贯口径一致（见 `SlotStatus.msg`）。
  [[nodiscard]] static double remaining_sec(double elapsed_sec,
                                            const MatchEndParams& params);

  // ══════════════════════════════════════════════════════════════════════
  // ④ 「v1.6」出界移除（规则 6.4：永久移除）
  // ══════════════════════════════════════════════════════════════════════

  struct BoundsParams
  {
    /// 场地边长（m）。场地是正方形 `[0, size]²`（见 `field_geometry.yaml` 的 `field.size`）
    double field_size{11.0};
    /// 出界**连续**多少秒 ⇒ 判定出界移除
    double out_of_bounds_forget_sec{3.0};
    /// 宽容带（m）：稍微压线不算出界（标定/噪声都有误差）
    double margin_m{0.10};
  };

  /// @brief 该点是否**出界**
  /// ⚠️ 带 `margin_m` 宽容带：`[-margin, size+margin]` **之内**算出界内。
  ///    没有宽容带的话，一台贴着围栏跑的 BR 会**每一帧都判出界**。
  [[nodiscard]] static bool out_of_bounds(double x, double y, const BoundsParams& params);

  /// @brief 出界计时推进：本帧在界内 ⇒ 清零；在界外 ⇒ 累加 `dt`
  /// @param cur_sec    当前累计
  /// @param inside     本帧是否在界内
  /// @return 推进后的累计秒数
  [[nodiscard]] static double step_out_of_bounds(double cur_sec, bool inside, double dt);

  /// @brief 累计是否已达"该移除"
  [[nodiscard]] static bool should_forget(double out_sec, const BoundsParams& params);

  // ══════════════════════════════════════════════════════════════════════
  // ⑤ 「v1.2」共享区停留统计（规格 §4.5 / 规则 6.6）
  // ══════════════════════════════════════════════════════════════════════

  /// 一个**轴对齐**的共享区（`field_geometry.yaml` 的 `*_x_range` / `*_y_range` 就是这个形状）
  struct Zone
  {
    double x_min{0.0}, x_max{0.0};
    double y_min{0.0}, y_max{0.0};
  };

  struct DwellParams
  {
    /// ⚠️⚠️ **这是「启发式提示」，不是规则硬性时限**。
    ///
    /// 规格 §4.5 的 v1.6 修订原文：
    /// > 「v1.6」dwell 计时器保留，注释改为"**启发式，非规则 6.6 硬性规定**"
    /// 而规则 6.6 **没有 5 秒的硬性时限，由裁判判定**。
    /// ⇒ **本模块只输出"停留了很久"这件事，不输出"违规"**。
    ///    ⚠️ 下游**不许**拿它当判罚依据 —— 那会让主控去做规则上不允许的动作。
    double dwell_limit_sec{5.0};
  };

  /// @brief 该点是否在共享区内（**只看水平 xy**）
  /// ⚠️ 边界**非严格**（贴着 `x_range` 的边缘算在内）—— 与场地划分的语义一致。
  [[nodiscard]] static bool in_zone(double x, double y, const Zone& zone);

  /// @brief 停留计时推进：在区内 ⇒ 累加 `dt`；离开 ⇒ **清零**
  ///
  /// ⚠️ 与 `step_out_of_bounds` 同款语义：**离开就清零**（不是衰减）。
  ///    判据是"**连续**停留"—— 做成衰减会让"反复进出共享区"的目标把累计值攒到阈值。
  [[nodiscard]] static double step_dwell(double cur_sec, bool inside, double dt);

  /// @brief 是否已超过启发式阈值
  /// ⚠️ **非严格**（>=）。⚠️ 见 `DwellParams::dwell_limit_sec` 的 ⚠️⚠️：
  ///    这个值**不是规则依据**，只用来提示。
  [[nodiscard]] static bool dwell_over_hint(double dwell_sec, const DwellParams& params);
};

}  // namespace fusion
}  // namespace br_perception
