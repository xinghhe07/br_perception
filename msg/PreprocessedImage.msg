# 预处理后的图像 —— image_preprocessor → 各视觉检测器
#
# 为什么不用裸 sensor_msgs/Image:
#   预处理管线做了 **letterbox**（等比缩放 + 居中补 114 灰边，见任务书 §3.1），
#   检测出的 bbox 坐标是「letterbox 坐标系」的，**不是原图坐标**。
#   要反算回原图（2D→3D 投影、PnP 都需要原图坐标下的 K），必须知道当时的变换参数。
#   而那几个参数**无法从图像本身推出** —— 所以随图一起发。
#
# ⚠️ 检测器**必须**用它反算，否则所有框坐标整体偏移，
#   而且偏得很隐蔽：近处看不出来、远处错得离谱。
#   基础设施见 src/camera/include/br_perception/camera/image_preprocessor.hpp
#   的 LetterboxTransform / compute_letterbox()。

std_msgs/Header header

# letterbox 后的图像（尺寸 = model_input_w × model_input_h）
sensor_msgs/Image image

# ── letterbox 变换（检测框反算用）──
#   反算公式：
#     x_orig = (x_letterbox − pad_x) / scale
#     y_orig = (y_letterbox − pad_y) / scale
float32 scale

# 填充偏移（像素）：pad_x = 左侧填充宽，pad_y = 上侧填充高
int32 pad_x
int32 pad_y

# letterbox **之前**的尺寸（= ROI 裁剪之后的尺寸，不是相机原始分辨率）
int32 orig_w
int32 orig_h
