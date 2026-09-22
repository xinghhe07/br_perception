# CameraFrame.msg — 相机层**聚合帧**（camera_perception_node 的输出，任务书 §15.1 模块 10）
#
# 为什么需要它：
#   三个传统 CV 检测器（mustika / pillar / building_spot）本来各自独立发一条消息，
#   每路相机还各发一条 —— 一共 6 条话题。融合层要的是"**某一时刻整车看到的东西**"，
#   自己去对齐 6 条异步话题既啰嗦又容易写错（谁按谁对齐？谁等谁？）。
#   本消息把"同一时刻的那一帧"打成一个包，时间对齐在**一处**完成。
#
# ⚠️ 本消息**只有图像空间**的内容，没有任何 world 坐标 —— 转世界坐标是融合层
#    （有 TF、有外参、有里程计）的事。相机层不碰外参，也不碰里程计。
#   故**不能**用 PerceptionFrame 代替本消息：那个要求 ego_pose / mustika_state /
#    计分字段，相机层一个都填不了，硬填一半是空的 —— 而"空字段"与"真的没有"
#    在下游看起来完全一样，分不出来。

# frame_id 在**本层刻意留空**：
#   本帧装的是两台不同相机各自的 2D 结果，**没有**一个共同的坐标系可言。
#   填 "camera_frame" 之类的名字是虚构一个不存在的坐标系 —— 而它**没有对应的 TF**，
#   下游若照着它去 TF 变换/反投影，必然失败或算错，且错得没有提示。
#   （每路相机自己的坐标系在各自消息所在的 TF 树里，那是融合层的事。）
std_msgs/Header header

# ── 本帧的"时刻基准" ──
#   header.stamp = **各槽里最新的那个曝光时刻**，即"这批检测结果里最新的一条"。
#   ⚠️ 它不是组帧时刻 —— 组帧时刻见 frame_age_sec。
#
#   ⚠️⚠️ header.stamp 会**冻住**：所有检测器同时停发时，最新时刻也跟着停住不前。
#      所以**单看它判断不了"数据是不是还新"** —— 必须配 frame_age_sec 一起看。
#      （当初把 stale_sec 的基准错设成这个 stamp，就是这个坑的另一面。）
#
# 本帧基准相对**组帧时刻**已经过去了多久（秒）= 组帧时刻 − header.stamp。
#
#   · ≈ 0        → 有某路检测器正在持续产出，帧是"活"的
#   · 越来越大    → **所有检测器都停了**（或都慢了）。这是最要紧的一条读法：
#                  只要这个值在稳定增长，无论 header.stamp 看着多正常，数据都是旧的。
#   · 无任何数据时 → 组帧节点填 0 并把 header.stamp 设为组帧时刻
float32 frame_age_sec

# ── 检测结果 ──
#   三个数组**各自定长 2**，下标 = **camera_id**（0 = 宽角 wide / 1 = 长焦 tele）。
#   ⚠️ 是三个独立数组、各有各的 0..1 空间 —— **不是一个统一的槽空间**。
#      读 mustika[1] 是"长焦看到的穆斯蒂卡"；它和 pillars[] 没有任何下标关系。
#
#   未收到数据的槽 = 默认构造的消息（detected=false）。
#   ⚠️ 想知道"某槽到底有没有数据"，**看 slot_status**，不要看数组长度、
#      也不要用 detected==false 推断 —— "没数据"和"看了但没有"是两件事，
#      detected 字段表达不了后者之外的东西。
CameraMustika[2]     mustika
CameraPillar[2]      pillars
CameraBuildingSpot[2] building_spots

# ── 深度辅助 (D435i) ──
#   ⚠️ 深度节点 99% 时间在 IDLE 模式（没收到目标 ROI）。**别读 depth.target_depth_m
#      之前忘了查模式** —— 为此下面单独提炼了 depth_measuring，让下游一眼能判。
CameraDepth depth

# 深度是否处于**测量模式**（= depth.mode == CameraDepth.MODE_MEASURE）。
#   false → depth 里的 target_depth_* / target_u,v / confidence **全部无意义**
#           （那时它只是一条低频健康检查），只有 valid / valid_ratio 可用。
#   ⚠️ 之所以提炼成独立字段：聚合层的职责就是"让下游一眼看清"，
#      而不是让每个下游都记得去查 depth.mode 这个内嵌枚举。
bool depth_measuring

# ── 逐槽状态 ──
#   下标 = **kind × 2 + camera_id**（见 SlotStatus.msg），共 8 个：
#     [0] mustika_wide   [1] mustika_tele
#     [2] pillar_wide    [3] pillar_tele
#     [4] spot_wide      [5] spot_tele
#     [6] depth          [7] （保留：深度只有一路，此槽恒为 NEVER）
#   ⚠️ 定长 8 = 4 种 kind × 2 路相机，与上面三个数组**各自**的 0..1 区分开：
#      这里是"种类×相机"的扁平空间，上面是"每类各自的相机"空间。
SlotStatus[8] slot_status

# ── 本帧统计 ──
#   组帧本身的耗时 [ms]（**不含**检测耗时，也**不含发布本身**）
#   ⚠️ 本节点不等待任何检测器 —— 它慢只有一种可能：消息体变大了。
float32 assembly_ms
