# 建筑位状态消息
# 描述单个建筑位的位置、当前状态、塔顶颜色和归属信息

# 建筑位唯一标识符
uint32 spot_id

# 建筑位中心位置。
# ⚠️ 坐标系**由发布者的 header.frame_id 决定，不是固定的 world**（同 Obstacle.msg）：
#      · 雷达层发的是**输入点云的系**；融合层才发 world
# ⚠️ 雷达层知道每个建筑位的**先验位置**（场地地图里有），所以这个坐标即使
#    点云没扫到也可能被填上 —— 判断"到底看没看见"要看 `confidence` 与
#    发布端诊断里的 `point_count`，不要只看坐标非零。
geometry_msgs/Point position

# 建筑位当前状态
# 0 = 空 (EMPTY)        — 建筑位无任何地球/方块
# 1 = 1个地球 (ONE_EARTH)  — 建筑位上有 1 个地球
# 2 = 2个地球 (TWO_EARTH)  — 建筑位上有 2 个地球
# 3 = 完整塔 (FULL_TOWER)  — 建筑位上已有完整塔
uint8 state

# 塔顶颜色（仅在 state != EMPTY 时有意义）
# 0 = 无 (NONE)
# 1 = 红色 (RED)
# 2 = 蓝色 (BLUE)
# 3 = 未知 (UNKNOWN)
uint8 top_color

# 建筑位/塔归属
# 0 = 无 (NONE)   — 中立或空
# 1 = 我方 (OWN)
# 2 = 对方 (OPPONENT)
uint8 ownership

# 检测来源
# 0 = 雷达 (LIDAR)
# 1 = 相机 (CAMERA)
# 2 = 融合 (FUSION)
uint8 source

# 检测置信度 [0.0, 1.0]
float32 confidence
