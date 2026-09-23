# 感知帧 —— 融合层最终输出, 经通信层编码后发给 STM32 主控 (设计文档 §5.2)
# 字段顺序按协议 §5.1 的分组排列, 便于与 protocol_encoder 逐段对照。

# frame_id = "world"。原先用裸 builtin_interfaces/Time, 丢了坐标系 —— 下游无从判断
std_msgs/Header header

# 自身位姿 (world 系, 2D)。
# 刻意保持 Pose2D: 机器人在哪一层由任务状态单独给出, 位姿本身不需要 z。
geometry_msgs/Pose2D ego_pose

# 里程计可靠性 (§9.1 v1.5): Point-LIO 退化 / IMU 振动 / 超时 → false。
# 主控收到 false 应把所有 world 坐标整体降权 (× confidence_decay, 默认 0.5)
bool odometry_reliable

# ── 比赛状态 (协议 §5.1 v1.6 组) ──
# 剩余比赛秒数 0~180 (感知自计时)
uint8 match_remaining_sec
# 被持握方块总数
# ⚠️ **看 `held_count_valid`** —— 感知层无从观测夹爪，填不了时该位为 0，
#    此时本字段是**占位 0 而不是"一块都没拿"**。
uint8 held_object_count
# 0/1 持握超限 (规则 4.4.2: BR 限持 2 块)
uint8 holding_over_limit
# 0/1 终场释放告警 (规则 8.6.1: 蜂鸣时仍手持 = 该物体 0 分)
uint8 match_end_release_warning

# ── 穆斯蒂卡 ──
# 0=未知 1=柱上 2=TR转运中 3=BR持有 4=已供奉 5=掉落 6=重试归还中
# 6 为 v1.6 新增: 感知置 6 → 主控开始重试流程; 看到穆斯蒂卡回到柱上置回 1,
# 该变化即主控的"归还完成"信号 (规则 5.1.3/5.3.2)
uint8 mustika_state
# 0/1 圣所使命达成 (规则 4.5.1: ≥2 完整塔且 ≥1 在共享区) —— TR 取穆斯蒂卡的前置。
# 达成后**永久有效**, 塔之后被对手破坏也不复位 (规则 3.6.2)
# ⚠️ **看 `sanctuary_valid`**。本字段只有 0/1、**没有未知档** —— 感知层填不了，
#    而"填 0"是**具体断言"使命未达成"**（→ TR 永远不会去取穆斯蒂卡）；
#    "填 1"更糟（TR 会违规去取）。故 valid 位为 0 时主控**必须忽略本字段**。
uint8 sanctuary_mandate_fulfilled
# 穆斯蒂卡位置。必须是 PoseStamped 而非 Pose2D: 靠 z 区分"在柱上"还是"被拿起"
geometry_msgs/PoseStamped mustika_pose

# ── 柱子 / 建筑位 / 障碍物 ──
PillarStatus mustika_pillar
PillarStatus core_pillar
BuildingSpot[] building_spots
Obstacle[] obstacles

# ══════════════════════════════════════════════════════════════════════════════
# ── 字段有效性 (2026-09-22 新增) ──
#
# 为什么需要它们：**空字段与"真的没有"在下游看起来完全一样，分不出来**
# （同一句话已写在 CameraFrame.msg 里）。本消息里有三个字段**没有天然的未知档**，
# 一旦填了个 0 就是**替下游断言一个具体事实**：
#
#   · `mustika_pose` 零值在协议里是一个**合法的世界坐标 (0,0,0) = 场地西南角**
#   · `sanctuary_mandate_fulfilled` 只有 0/1，填 0 = "使命未达成"
#     → **TR 永远不去取穆斯蒂卡，白丢 250 分**
#   · `held_object_count` 恒 0 = "一块都没拿"，而夹爪状态**只有主控知道**
#
# 所以：**能填的填，填不了的把对应的 valid 位置 0，下游必须忽略那个字段。**
# ⚠️ 这几个位经 `protocol_encoder` 打包进 `0x01` 帧的**状态标志字节**
#    （见 protocol_encoder.hpp 的 `StatusFlag`），**要跟下位机确认后才能启用**。
# ══════════════════════════════════════════════════════════════════════════════

# 位置类字段是否有效（ego_pose / mustika_pose / 建筑位与障碍物坐标）。
#   false → **位置字段全部无意义，下游必须忽略**，只能看"链路活没活"与状态字段。
#   ⚠️ 这正是"**位置不可用，但状态照发**"所依赖的那一位：有了它，
#      感知算不出 world 坐标时不必整帧静默（那会让主控连比赛计时都断）。
bool pose_valid

# sanctuary_mandate_fulfilled 是否有效。
#   ⚠️ 感知层**填不了**这个字段（要判"两座全塔且至少一座在公共区"，那是融合层的活），
#      在当前架构下它应由主控自己维护 → 本节点发 false。
bool sanctuary_valid

# held_object_count / holding_over_limit 是否有效。
#   ⚠️ 夹爪上有几块**只有主控知道**，感知链路无从观测 → 本节点发 false。
bool held_count_valid

# ── 诊断 ──
diagnostic_msgs/DiagnosticStatus[] diagnostics

# 帧序号 (递增)。原名 frame_id 与 Header.frame_id 语义撞名 (一个是序号一个是坐标系), 故改名
uint32 seq
