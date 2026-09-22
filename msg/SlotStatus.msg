# SlotStatus.msg — 相机层聚合帧里**一个槽**的状态（CameraFrame.slot_status[] 的元素）
#
# 为什么要单独一条消息、而不是在 CameraFrame 里放两个平行数组：
#   平行数组（stale_sec[] + detector_status[]）靠"同下标、同长度"这种**隐式约定**
#   维系，加一维（比如再多一路相机）就会漂移，而且编译器不会提醒。
#   结构化之后由**类型**保证：一个 SlotStatus 就是一个槽的全部状态，拆不开、错不了位。

# ── 这个槽是谁 ──
#   kind 与 msg/CameraFrame.msg 的数组一一对应：
#     0 = mustika  → CameraFrame.mustika[]
#     1 = pillar   → CameraFrame.pillars[]
#     2 = spot     → CameraFrame.building_spots[]
#     3 = depth    → CameraFrame.depth
uint8 K_MUSTIKA      = 0
uint8 K_PILLAR       = 1
uint8 K_SPOT         = 2
uint8 K_DEPTH        = 3
uint8 kind

# 相机 ID: 0 = 宽角 (wide) / 1 = 长焦 (tele)
#   ⚠️ 对 kind=K_DEPTH 恒为 0（深度只有一路）
uint8 camera_id

# ── 这个槽现在是什么状态（**机器可读**，下游按这个分支，不要按 stale_sec 猜）──
#   0 = NEVER  从未收到过任何消息
#       ⚠️ 与 STALE 是**两回事**：NEVER 多半是话题名对不上 / 发布端没起 /
#          这路相机压根没装；STALE 是"跑过，但慢了"。现场排查方向完全不同。
#   1 = FRESH  收到过，且 stale_sec 在容差内（= "当前"）
#   2 = STALE  收到过，但已经比容差更旧（该路慢了或停了）
#
#   ⚠️ FRESH 的判据是 `stale_sec > fresh_tolerance_sec`（**参数**，默认 0.5s），
#      **不是** `stale_sec > 0`：曝光时刻永远早于组帧时刻，`stale_sec` 恒为正，
#      拿 0 当界会让这一档**永远到不了**。容差取值见
#      camera_perception_node.hpp 的 `fresh_tolerance_sec_`。
uint8 S_NEVER = 0
uint8 S_FRESH = 1
uint8 S_STALE = 2
uint8 state

# ── 年龄 ──
#   stale_sec = **组帧时刻 − 该槽最新结果的 header.stamp**（秒）
#
#   ⚠️⚠️ 基准是**组帧时刻**，不是"最新那个槽的时间戳"—— 这是个改过一次的地方：
#      若拿各槽里最新的那个 stamp 当基准，所有检测器**同时停发**时，那个"最新槽"
#      也跟着停住不前，于是它的 stale_sec 恒为 0、state 恒为 FRESH，
#      帧 header.stamp 也一起冻住 —— **恰恰把"某路检测器死了"这个要暴露的
#      失效模式掩盖掉**，而那正是本字段存在的唯一理由。
#
#   · state == S_NEVER → 本字段**无意义**（填 0），看 state
#   · 负值 → 该槽的时间戳**在未来**（时钟跳变 / 驱动异常）。
#            **本消息如实报负值，不钳到 0** —— "时间是负的"本身就是要暴露的异常，
#            钳掉就等于把它藏了。
float32 stale_sec

# ── 人读备注（日志/诊断用；**不要**拿它做机器分支，机器分支看 state）──
string note
