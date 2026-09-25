# Track.msg — 融合层的**追踪目标**（模块 18 `kalman_tracker` / 19 `target_associator` 的公共契约）
#
# ⚠️ 2026-09-25 新建。为什么先建它：模块 18/19/21 的规格都写着"输入/输出**追踪列表**"，
#    而仓库里**没有任何消息能表达它** —— 这和 `fusion_node` 那条"前置：消息接口必须先落"
#    是同一类问题（见台账 §五）。**消息接口不先落，后面三个模块都写不了。**
#
# 与 `Obstacle.msg` 的关系（**别混**）：
#   · `Obstacle` = **一次观测**（单帧、无历史），雷达聚类层出的
#   · `Track`    = **一个被追踪的目标**（跨帧、有速度、有状态机），融合层维护的
#   ⇒ 19 干的事就是"把 Obstacle（观测）关联到 Track（目标）"。

std_msgs/Header header

# ── 身份 ──
uint32 id

# ── 状态机（模块 18 维护）──
#   CANDIDATE: 才观测到，**不输出给下游**（等确认，防单帧噪声）
#   CONFIRMED: 连续 confirm_frame_count 帧命中 ⇒ 确认，输出
#   LOST:      连续 delete_frame_count 帧没命中 ⇒ 待删（还在预测）
uint8 S_CANDIDATE = 0
uint8 S_CONFIRMED = 1
uint8 S_LOST      = 2
uint8 state

# ── 位姿（卡尔曼滤波后的**估计值**，不是原始观测）──
geometry_msgs/Point    position     # 世界系（⚠️ 见下 frame_id 口径）
geometry_msgs/Vector3  velocity     # m/s
geometry_msgs/Vector3  bbox_size    # m

# ── 语义 ──
#   ⚠️ `type` 与 `source` 的枚举**沿用 `Obstacle.msg`**（同一个东西别定义两遍）：
#      type:   0=UNKNOWN 1=EARTH_CUBE 2=SKY_CUBE 3=ENEMY_ROBOT 4=MUSTIKA
#      source: 见 Obstacle.msg（雷达/相机/融合）
uint8   type
uint8   source
float32 confidence                 # [0,1]

# ── 关联统计（19 写、18 用）──
#   ⚠️ 这两个是**状态机判据**的原料：`hit_count` 到了 confirm_frame_count 才升 CONFIRMED，
#      `miss_count` 到了 delete_frame_count 才删。**别拿别的量去凑**。
uint32 hit_count
uint32 miss_count
# 最后一次**被观测到**的时刻（秒）。⚠️ 与 `header.stamp` 不同义：
#   header.stamp = 本消息的发出时刻；本字段 = 目标最后一次真实被看见的时刻。
#   下游判"这个 track 是不是还在"要看它（遮挡/持握时两者会拉开）。
float64 last_seen_sec

# ── 「v1.2」推移轨迹：窗口内 z 基本恒定 + 水平匀速滑动（无抬起-放下）⇒ 疑似推移而非搬运
#    供主控参考规则 3.5.3 / 8.6.4
bool suspected_pushing

# ── 「v1.3」持握：高置信度视觉但无雷达点位匹配（夹爪粘连/悬空）⇒ 单目 PnP 粗定位建独立 track
#    ⚠️ 这类 track **不该**与雷达目标强行关联（会造成错误融合）
bool is_held

# ── 「v1.2」己方 TR 候选：位置在己方转运区附近且运动学吻合
#    ⚠️ 置位后**必须排除出"敌方机器人"分类**（规则 11.10 禁无线通信，但 BR 可"看到"TR）
bool is_ally_tr

# ── 来源明细（诊断用：这个 track 最近一次是被谁更新的）──
#   ⚠️ 用位掩码而不是 bool，因为"雷达+相机都命中"是三态里的第三态
#      （规格 §4.3 的三种匹配场景），两个 bool 要四个组合、其中一个非法。
uint8 SRC_NONE   = 0
uint8 SRC_LIDAR  = 1
uint8 SRC_CAMERA = 2
uint8 last_update_src
