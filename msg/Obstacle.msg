# 障碍物消息
# 描述单个障碍物的位置、尺寸、运动状态和分类信息

# 障碍物唯一标识符
uint32 id

# 障碍物中心位置。
# ⚠️ 坐标系**由发布者的 header.frame_id 决定，不是固定的 world**：
#      · 雷达层 (`lidar_perception_node`) 发的是**输入点云的系**（它不做坐标变换）
#      · 融合层 (`fusion_node`) 才是发 world 的那一个
#    下游**必须读 header.frame_id**，不能假定是 world。
geometry_msgs/Point position

# 包围盒尺寸（长/宽/高）[m]
geometry_msgs/Vector3 bbox_size

# 速度向量 [m/s]
geometry_msgs/Vector3 velocity

# 障碍物类型标签
# 0 = 未知 (UNKNOWN)
# 1 = 敌方机器人 (ENEMY_ROBOT)
# 2 = 掉落方块 (DROPPED_BLOCK)
# 3 = 其他 (OTHER)
uint8 type

# 检测置信度 [0.0, 1.0]
float32 confidence

# 检测来源
# 0 = 雷达 (LIDAR)
# 1 = 相机 (CAMERA)
# 2 = 融合 (FUSION)
uint8 source
