# FusionState.msg — 融合层（模块 21 `fusion_node`）的输出
#
# ⚠️ 2026-09-25 新建。**它不是 `/perception/output`** ——
#    用户 2026-09-25 拍板「**`perception_pipeline` 保有上行权**」：
#    `/perception/output` 是 `comm_node` 唯一的串口上行入口，两个模块都发同一个话题
#    = 下游收到谁看运气。⇒ 21 只发**融合结果**，由 pipeline 汇总进 `PerceptionFrame`。
#
# ⚠️⚠️ **本消息的字段是照着 `PerceptionFrame` 那几个"融合结论"字段 1:1 摆的**
#    （`sanctuary_mandate_fulfilled` / `held_object_count` / `holding_over_limit` /
#     `match_remaining_sec` / `match_end_release_warning` + 两个 `*_valid`）。
#    ⇒ 目的是让 pipeline 那边**只做一次搬运、不做任何再推导**。
#    ⚠️ 若让 pipeline 自己再算一遍，两处就会**各自漂移**，而症状是"主控看到的值
#       和感知层以为的值不一样" —— 那种错没人查得出来。

std_msgs/Header header

# ── 圣所使命（**Gate 着 250 分**，规则 4.5.1）──
uint8 sanctuary_mandate_fulfilled
# ⚠️ **这个结论可不可信**：有任何建筑位 `state == UNKNOWN(4)` 就是 0。
#    ⇒ 下游据此决定要不要把它当事实发给主控（合 §零「准」轴：未知要有哨兵）。
bool sanctuary_valid

# ── 持握（规则 4.4.2：BR 限持 2 块）──
uint8 held_object_count
# ⚠️ `held_object_count` 可不可信（雷达/视觉对"持握"的判定依赖外参与遮挡）
bool held_count_valid
# BR 位置 hold_check_radius 内持握方块数 > holding_limit_count ⇒ 1（规则 4.4.2）
uint8 holding_over_limit

# ── 终场（规则 4.2.2）──
# ⚠️⚠️ **是 `int16` 不是 `uint8`** —— 这不是随手选的：
#    `PerceptionFrame.match_remaining_sec` 是 **`uint8`**，而它**表达不了"超时"**。
#    感知层自计时过了 180s 之后，剩余时间是**负数**（规则 4.2.2 是固定时长），
#    而 `uint8` 只能 0~255 ⇒ 超时会被写成 0，**丢掉"已经超时了"这个信号**。
#    ⇒ 本消息用 `int16`（-32768~32767）先把真值留住；
#      **那一处 `uint8` 是个已知的契约缺口，已记入台账，需与主控对齐。**
int16 match_remaining_sec
# 剩余 ≤ match_end_release_window_sec 且存在持握目标 ⇒ 1（**每帧重发**，直到终场）
uint8 match_end_release_warning

# ── 追踪目标（模块 18 的输出）──
Track[] tracks

# ── 诊断 ──
# 本帧关联上了几条观测 / 新建了几个候选 / 清了几个 —— 供 `health_monitor` 汇总
uint16 associated_count
uint16 new_candidate_count
uint16 track_total
float32 fusion_ms
