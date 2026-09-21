# CameraMustika.msg — 相机检测到的穆斯蒂卡（v1.1 排球: 黄底+蓝纹+白字, Ø≈210）图像空间结果
# ⚠ 2026-09-20 更正: 原写"金色球体"系中文 v1.0 过时译本; v1.1 一手原文为 Mikasa V300W 排球
#
# 与 MustikaStatus.msg（世界坐标系，融合级）不同，
# 本消息承载纯图像空间的检测原始结果，供调试、验证和融合模块使用。

# 相机 ID: 0 = 宽角 (wide, JR H200S 宽) / 1 = 长焦 (tele, JR H200S 长焦)
# ⚠️ 2026-09-20 新增 header —— 任务书 §四「时间戳约定」要求:
#   所有检测结果的时间戳必须是**曝光时刻**(或曝光中点), **不是**"处理完发出来的时刻"。
#   相机通路延迟 ≈43ms, 取发出时刻会多算位移, 污染 kalman_tracker 的外推。
#   检测器必须**原样继承**输入的 PreprocessedImage.header.stamp, 禁止用 now()。
std_msgs/Header header

uint8 camera_id

# 球心在图像中的像素坐标
float32 u
float32 v

# 球的像素半径
float32 radius

# 检测置信度 [0.0, 1.0]
float32 confidence

# 穆斯蒂卡是否在视野中
bool detected
