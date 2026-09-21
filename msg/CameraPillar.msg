# CameraPillar.msg — 相机检测到的柱子（图像空间结果）
#
# 穆斯蒂卡柱：深绿色 PVC 管，高 500mm，直径 270mm
# 核心支柱：棕色，高 800mm，直径 270mm

# 相机 ID: 0 = 宽角 (wide, JR H200S 宽) / 1 = 长焦 (tele, JR H200S 长焦)
# ⚠️ 2026-09-20 新增 header —— 任务书 §四「时间戳约定」要求:
#   所有检测结果的时间戳必须是**曝光时刻**(或曝光中点), **不是**"处理完发出来的时刻"。
#   相机通路延迟 ≈43ms, 取发出时刻会多算位移, 污染 kalman_tracker 的外推。
#   检测器必须**原样继承**输入的 PreprocessedImage.header.stamp, 禁止用 now()。
std_msgs/Header header

uint8 camera_id

# 是否检测到柱子
bool detected

# 柱子类型: 0 = 穆斯蒂卡柱 (MUSTIKA_PILLAR), 1 = 核心支柱 (CORE_PILLAR), 2 = 未知 (UNKNOWN)
uint8 pillar_type

# 包围盒中心像素坐标
float32 u
float32 v

# 包围盒宽高 (pixels)
float32 width
float32 height

# 检测置信度 [0.0, 1.0]
float32 confidence

# 柱顶是否有物体 (对雷达融合有帮助)
bool has_top_object
