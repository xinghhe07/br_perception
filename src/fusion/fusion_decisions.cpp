#include "br_perception/fusion/fusion_decisions.hpp"

#include <cmath>

namespace br_perception {
namespace fusion {

// ═══════════════════════════════════════════════════════════════════════════
// ① 圣所使命（**值 250 分**）
// ═══════════════════════════════════════════════════════════════════════════
FusionDecisions::SanctuaryVerdict FusionDecisions::judge_sanctuary(
    const std::vector<SpotState>& spots, const SanctuaryParams& params)
{
  SanctuaryVerdict v;

  for (const auto& s : spots) {
    // ⚠️⚠️ **"没量成"的位单独计数，不参与完整塔的统计。**
    //    把它当"不是完整塔"会凭空否掉一座可能存在的塔（白丢 250 分）；
    //    把它当"是完整塔"会凭空造一座（让主控做规则不允许的动作）。
    //    ⇒ 不算数，但把 `valid` 置 false —— 见 `SpotState::known` 的 ⚠️⚠️。
    if (!s.known) {
      ++v.unknown_count;
      continue;
    }
    if (!s.complete) continue;
    ++v.complete_total;
    // ⚠️ **两个条件是"与"，而且"在共享区"那条是**在完整塔里**再筛**（不是全场筛）——
    //    写成"全场有共享区塔 && 全场塔数≥2"会把两条条件解耦，
    //    于是"2 座塔都在非共享位 + 共享位上有个半成品"会被误判成达成。
    if (s.in_shared_zone) ++v.complete_in_shared;
  }

  v.fulfilled = v.complete_total >= params.min_complete_towers &&
                v.complete_in_shared >= params.min_in_shared_zone;
  // ⚠️ 只要有一位没量成，这个结论就**不是完整数据的结果** ⇒ 标记为不可信。
  v.valid = (v.unknown_count == 0);
  return v;
}

bool FusionDecisions::sanctuary_mandate_fulfilled(
    const std::vector<SpotState>& spots, const SanctuaryParams& params)
{
  return judge_sanctuary(spots, params).fulfilled;
}

// ═══════════════════════════════════════════════════════════════════════════
// ② 持握超限
// ═══════════════════════════════════════════════════════════════════════════
int FusionDecisions::held_within(double br_x, double br_y,
                                 const std::vector<HeldObject>& objects,
                                 double radius_m)
{
  const double r2 = radius_m * radius_m;
  int n = 0;
  for (const auto& o : objects) {
    if (!o.is_held) continue;                 // ⚠️ 只数**持握**的
    // ⚠️ **只用水平距离**（见头文件）：持握的方块在夹爪里、可能悬空，
    //    z 与 BR 中心差得远，带上 z 会把"拿着"判成"没拿"。
    const double dx = o.x - br_x;
    const double dy = o.y - br_y;
    // ⚠️ 边界**非严格**（<=）：恰好压在半径上算"在范围内"。
    if (dx * dx + dy * dy <= r2) ++n;
  }
  return n;
}

bool FusionDecisions::holding_over_limit(double br_x, double br_y,
                                         const std::vector<HeldObject>& objects,
                                         const HoldingParams& params)
{
  // ⚠️ **严格大于**：恰好 `holding_limit_count` 块**不算**超限
  //    （规则 4.4.2 原文是"限持 2 块" ⇒ 2 块合法）。
  return held_within(br_x, br_y, objects, params.hold_check_radius_m) >
         params.holding_limit_count;
}

// ═══════════════════════════════════════════════════════════════════════════
// ③ 终场告警
// ═══════════════════════════════════════════════════════════════════════════
double FusionDecisions::remaining_sec(double elapsed_sec, const MatchEndParams& params)
{
  // ⚠️ **不钳到 0**：超时了就如实报负 —— 与仓库对"时间是负的"的一贯口径一致
  //    （见 `SlotStatus.msg`：负值本身就是"时钟跳变/异常"要暴露的信号）。
  return params.match_duration_sec - elapsed_sec;
}

bool FusionDecisions::match_end_release_warning(double elapsed_sec,
                                                int held_count,
                                                const MatchEndParams& params)
{
  // ⚠️ **没有持握目标时一律不发** —— 发了是噪音，会让主控在最忙的时候多处理一条。
  if (held_count <= 0) return false;
  // ⚠️ 边界**非严格**：剩余正好等于窗口（如正好 5.0s）就**开始告警**。
  //    这条边界的方向是"宁早不晚" —— 松手是个动作，晚一秒就来不及。
  return remaining_sec(elapsed_sec, params) <= params.release_window_sec;
}

// ═══════════════════════════════════════════════════════════════════════════
// ④ 出界
// ═══════════════════════════════════════════════════════════════════════════
bool FusionDecisions::out_of_bounds(double x, double y, const BoundsParams& params)
{
  // ⚠️ **带 `margin_m` 宽容带**：`[-margin, size+margin]` **之内**算出界内。
  //    没有宽容带的话，一台**贴着围栏跑**的 BR 会每帧都判出界
  //    —— 而它其实一直在界内，只是标定误差让它偶尔压线。
  const double lo = -params.margin_m;
  const double hi = params.field_size + params.margin_m;
  // ⚠️ 边界**非严格**：恰好压在宽容带边界上算**界内**（同样宁松不严 ——
  //    误判出界的代价是"永久移除"，那是不可逆的）。
  return !(x >= lo && x <= hi && y >= lo && y <= hi);
}

double FusionDecisions::step_out_of_bounds(double cur_sec, bool inside, double dt)
{
  // ⚠️ **在界内必须清零**（不是"慢慢减"）—— 判据是"**连续**出界 out_of_bounds_forget_sec"，
  //    回界内一下就该重新计时。做成衰减会让"反复进出边界"的目标累积到移除。
  if (inside) return 0.0;
  // ⚠️ `dt <= 0` 不动（防调用方传零/负把它算回去）
  return (dt > 0.0) ? (cur_sec + dt) : cur_sec;
}

bool FusionDecisions::should_forget(double out_sec, const BoundsParams& params)
{
  // ⚠️ **非严格**（>=）：达到阈值就移除。这条是"到点执行"，不是"多等一帧更保险"——
  //    规则 6.4 说的是"永久移除"，多等的那几帧会让主控白等。
  return out_sec >= params.out_of_bounds_forget_sec;
}

// ═══════════════════════════════════════════════════════════════════════════
// ⑤ 共享区停留
// ═══════════════════════════════════════════════════════════════════════════
bool FusionDecisions::in_zone(double x, double y, const Zone& zone)
{
  // ⚠️ **只看水平 xy** —— 共享区的 z 就是平台高（0.6 / 0.9），
  //    而目标的 z 受遮挡与俯仰误差影响远大于 xy。带上 z 会让"明明站在共享区里"的目标漏判。
  // ⚠️ 边界**非严格**：贴着 `x_range` 边缘算在内。
  return x >= zone.x_min && x <= zone.x_max &&
         y >= zone.y_min && y <= zone.y_max;
}

double FusionDecisions::step_dwell(double cur_sec, bool inside, double dt)
{
  // ⚠️ **离开就清零**（不是衰减）—— 判据是"**连续**停留"。
  //    与 `step_out_of_bounds` 同款理由：衰减会让"反复进出"的目标攒到阈值。
  if (!inside) return 0.0;
  return (dt > 0.0) ? (cur_sec + dt) : cur_sec;
}

bool FusionDecisions::dwell_over_hint(double dwell_sec, const DwellParams& params)
{
  // ⚠️⚠️ **这个 bool 只表示"停留很久了"，不表示"违规"** ——
  //    规则 6.6 没有硬性时限（v1.6 修订明写"启发式"）。
  //    ⇒ 下游**不许**拿它当判罚依据。
  return dwell_sec >= params.dwell_limit_sec;
}

}  // namespace fusion
}  // namespace br_perception
