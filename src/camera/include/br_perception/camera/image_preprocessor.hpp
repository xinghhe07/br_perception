#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// image_preprocessor.hpp — ROBOCON 2027 双相机图像预处理节点
///
/// 依赖: OpenCV (core + imgproc) + cv_bridge
/// 职责:
///   1. 订阅 /camera_wide/image_raw 和 /camera_tele/image_raw
///   2. ROI 裁剪 (过滤无关注视区域)
///   3. CLAHE 自适应直方图均衡化 (LAB 色彩空间 L 通道)
///   4. 曝光补偿 (基于直方图拉伸的软件补偿)
///   5. Resize 到模型输入尺寸 (640×384)
///   6. BGR → RGB 转换
///   7. 可选归一化 (mean/std, 适配 ONNX 推理)
///   8. 发布预处理图像 + 诊断信息
///
/// 输出话题:
///   /perception/camera_wide/preprocessed   (sensor_msgs::Image)
///   /perception/camera_tele/preprocessed    (sensor_msgs::Image)
///   /perception/camera/diagnostics          (diagnostic_msgs::DiagnosticArray)
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

// 预处理图 + letterbox 变换 (检测器反算检测框用) —— 2026-09-20 新增
#include "br_perception/msg/preprocessed_image.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>
#include <mutex>

namespace br_perception {
namespace camera {

// ═════════════════════════════════════════════════════════════════════════════
// letterbox 变换 —— **预处理与下游检测器共用同一份计算**
//
// ⚠️ 2026-09-20 新增。起因: 原实现在 resize_to_model 里做**硬拉伸**
//    (cv::resize 直接压到目标尺寸), 相机 1280×720 (16:9) → 640×384 (5:3) 会
//    **纵向拉伸 6.7%**, 精确打在下游的几何判据上 (直角评分/边长一致性/
//    长宽比/圆形度)。而 Ultralytics 训练时用的是 **letterbox**(等比+补边) ——
//    训练/推理分布不一致。
//
//    改成 letterbox 后, **检测框的坐标是"缩放+填充后"的, 必须反算回原图**,
//    否则 2D→3D 投影全偏。反算口径必须与这里完全一致, 故集中在本结构里。
// ═════════════════════════════════════════════════════════════════════════════

/// @brief letterbox 的填充灰度值
/// **必须与训练时 Ultralytics 用的一致 (114 灰)**, 不是 0 黑。
constexpr int kLetterboxPadValue = 114;

/// @brief letterbox 几何变换参数
struct LetterboxTransform {
  double scale  = 1.0;   ///< 等比缩放系数 (原图 → 内容区)
  int    pad_x  = 0;     ///< 左侧填充宽度  [px]
  int    pad_y  = 0;     ///< 上侧填充高度  [px]
  int    out_w  = 0;     ///< letterbox 输出宽
  int    out_h  = 0;     ///< letterbox 输出高
  int    orig_w = 0;     ///< letterbox 之前的宽
  int    orig_h = 0;     ///< letterbox 之前的高

  /// 内容区尺寸 (缩放后、未填充)
  int content_w() const { return static_cast<int>(orig_w * scale + 0.5); }
  int content_h() const { return static_cast<int>(orig_h * scale + 0.5); }

  /// ── 检测框反算 (下游必须用这两个) ──
  /// letterbox 坐标 → 原图坐标
  double to_orig_x(double x) const { return (x - pad_x) / scale; }
  double to_orig_y(double y) const { return (y - pad_y) / scale; }
  /// 原图坐标 → letterbox 坐标
  double to_box_x(double x) const { return x * scale + pad_x; }
  double to_box_y(double y) const { return y * scale + pad_y; }
};

/// @brief 反算**位置类**量（中心点 / 角点 / 像素坐标）
/// 减填充偏移 → 除 scale。两个都要。
inline void letterbox_to_orig_xy(float scale, std::int32_t pad_x, std::int32_t pad_y,
                                 double& x, double& y)
{
  const double s = (scale > 1e-6f) ? static_cast<double>(scale) : 1.0;  // 防 0
  x = (x - static_cast<double>(pad_x)) / s;
  y = (y - static_cast<double>(pad_y)) / s;
}

/// @brief 反算**尺度类**量（半径 / 宽 / 高）
/// ⚠️ **只除 scale，不减填充偏移** —— 减了会算出负数或错尺寸。
///    这是整套反算里最易错的一处：把 width/height 当位置一起减偏移，
///    在小目标上会直接出负宽度。
inline void letterbox_to_orig_size(float scale, double& w, double& h)
{
  const double s = (scale > 1e-6f) ? static_cast<double>(scale) : 1.0;
  w = w / s;
  h = h / s;
}

/// @brief 圆形目标的便捷反算（中心 + 半径）：= xy 反算 + size 反算
/// @param u,v,r  in: letterbox 坐标; out: 原图坐标与半径
inline void letterbox_to_orig(float scale, std::int32_t pad_x, std::int32_t pad_y,
                              double& u, double& v, double& r)
{
  const double s = (scale > 1e-6f) ? static_cast<double>(scale) : 1.0;
  letterbox_to_orig_xy(scale, pad_x, pad_y, u, v);
  r = r / s;                                  // 半径按尺度量换, 不减偏移
}

/// @brief 计算 letterbox 变换 (等比缩放 + 居中填充)
/// @param orig_w/orig_h    letterbox 之前的尺寸 (ROI 裁剪**之后**的尺寸!)
/// @param target_w/target_h 模型输入尺寸
inline LetterboxTransform compute_letterbox(int orig_w, int orig_h,
                                            int target_w, int target_h)
{
  LetterboxTransform tf;
  tf.orig_w = orig_w;
  tf.orig_h = orig_h;
  tf.out_w  = target_w;
  tf.out_h  = target_h;
  if (orig_w <= 0 || orig_h <= 0 || target_w <= 0 || target_h <= 0) {
    return tf;  // 退化: 保持单位变换
  }
  // 等比缩放到"能完整放进目标框"
  tf.scale = std::min(static_cast<double>(target_w) / orig_w,
                      static_cast<double>(target_h) / orig_h);
  // 居中填充
  tf.pad_x = (target_w - static_cast<int>(orig_w * tf.scale + 0.5)) / 2;
  tf.pad_y = (target_h - static_cast<int>(orig_h * tf.scale + 0.5)) / 2;
  return tf;
}

/**
 * @brief 双相机图像预处理组件
 *
 * 管线顺序 (每路独立并行):
 *   原始 BGR 图像 → ROI 裁剪 → CLAHE 增强 → [曝光补偿] → **letterbox** → BGR→RGB → [归一化] → 发布
 *
 * ⚠️ 归一化的正确配置: `enable_normalize: true` + mean=[0,0,0] + std=[1,1,1]
 *    → (pixel/255 - 0)/1 = **0–1 float32**, 正是 Ultralytics ONNX 要的输入。
 *    用 ImageNet 的 mean/std 是错的(训练时 Ultralytics 只除 255)。
 *
 * 前后相机独立处理, 各自拥有独立的参数和回调,
 * 共享同一个节点和诊断发布器。
 */
class ImagePreprocessor : public rclcpp::Node
{
public:
  explicit ImagePreprocessor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ── ROS 回调 ──
  void wide_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void tele_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter>& params);

  // ═════════════════════════════════════════════════════════════════════════════
  // 通用预处理管线 (前后相机共用)
  // ═════════════════════════════════════════════════════════════════════════════

  /// @brief 完整预处理管线
  /// @param bgr      输入 BGR 图像 (CV_8UC3)
  /// @param roi      ROI 裁剪区域 [x, y, w, h], 空或全零=不裁剪
  /// @param cla      CLAHE 参数 {enabled, clip_limit, tile_size}
  /// @param model    {target_w, target_h}
  /// @param norm     归一化参数 {enabled, mean[3], std[3]}
  /// @param out_enc  输出编码 (由管线判定并回填): "rgb8" 或 "32FC3"
  /// @return 预处理后的图像, 空 Mat 表示失败
  cv::Mat preprocess(const cv::Mat& bgr,
                     const std::vector<int64_t>& roi,
                     bool clahe_enabled, double clahe_clip, int clahe_tile,
                     int target_w, int target_h,
                     bool normalize_enabled,
                     const std::vector<double>& norm_mean,
                     const std::vector<double>& norm_std,
                     std::string& out_encoding,
                     LetterboxTransform* tf_out = nullptr);

  // ── 子步骤 ──

  /// @brief ROI 裁剪
  static cv::Mat crop_roi(const cv::Mat& image, const std::vector<int64_t>& roi);

  /// @brief CLAHE 增强 (LAB 色彩空间 L 通道, 避免颜色失真)
  static cv::Mat apply_clahe(const cv::Mat& bgr, double clip_limit, int tile_size);

  /// @brief 曝光补偿 — V 通道直方图拉伸 (clip 两端 0.5%)
  /// ⚠️ 默认**关闭** (见 exposure_compensation_enabled_): Ultralytics 训练时不做这一步,
  ///    推理侧开着会造成训练/推理分布错配。要用就必须训练数据也过同一套预处理。
  static cv::Mat compensate_exposure(const cv::Mat& bgr);

  /// @brief **letterbox** 到模型输入尺寸 (等比缩放 + 居中补 kLetterboxPadValue 灰边)
  /// 替代原先的"硬拉伸" —— 后者会让 16:9 的相机画面在 5:3 的输入上纵向拉伸 6.7%。
  /// @param tf_out 若不为空, 回填本次的变换参数, **交给下游反算检测框坐标**
  static cv::Mat letterbox_to_model(const cv::Mat& image, int target_w, int target_h,
                                    LetterboxTransform* tf_out = nullptr);

  /// @brief BGR → RGB
  static cv::Mat convert_bgr_to_rgb(const cv::Mat& bgr);

  /// @brief 归一化: (pixel/255.0 - mean) / std → float32
  static cv::Mat apply_normalization(const cv::Mat& rgb,
                                     const std::vector<double>& mean,
                                     const std::vector<double>& stddev);

  /// @brief 单路相机通用的回调处理逻辑
  /// ⚠️ 2026-09-20: 发布类型从 sensor_msgs::Image 改为 **PreprocessedImage** ——
  ///    后者把 letterbox 变换一起带走, 检测器才能把 bbox 反算回原图坐标。
  void process_and_publish(
      const sensor_msgs::msg::Image::SharedPtr msg,
      const std::string& cam_name,
      bool clahe_enabled, double clahe_clip, int clahe_tile,
      const std::vector<int64_t>& roi,
      const rclcpp::Publisher<br_perception::msg::PreprocessedImage>::SharedPtr& pub);

  // ── 诊断 ──
  void publish_diagnostics(const std::string& cam_name,
                           double preprocess_ms, size_t frame_seq,
                           int orig_w, int orig_h,
                           int out_w, int out_h,
                           const std::string& out_encoding);

  // ── 订阅 ──
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_wide_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_tele_;

  // ── 发布 ──
  // ⚠️ 类型是 PreprocessedImage (图 + letterbox 变换), 不是裸 Image —— 见 msg 文件头
  rclcpp::Publisher<br_perception::msg::PreprocessedImage>::SharedPtr pub_wide_;
  rclcpp::Publisher<br_perception::msg::PreprocessedImage>::SharedPtr pub_tele_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  // ── 参数回调句柄 ──
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // ═════════════════════════════════════════════════════════════════════════════
  // 全局模型参数
  // ═════════════════════════════════════════════════════════════════════════════
  int    model_input_w_{640};
  int    model_input_h_{384};
  // ⚠️⚠️ **必须保持 false**（2026-09-21 VM 实测踩过，与 config 的值必须一致）
  //
  //   2026-09-20 曾把它改成 true，想给 ONNX 喂 0–1 float32。后果是**整条相机链路断掉**：
  //     · 本节点输出 32FC3 (float32)
  //     · 订阅这条话题的 3 个传统 CV 检测器做 `toCvShare(msg->image, msg, BGR8)`
  //       —— float32 转不了 → **抛异常**，而且不报错日志，表现是"一帧都收不到"
  //
  //   归一化是**模型专属**的事，不属于共享预处理。将来 YOLO 检测器自己在推理前
  //   `convertTo(CV_32F, 1.0/255.0)` 即可 —— 但它不能改变这条共享话题的编码。
  //
  //   ⚠️ 这与 `config/camera_params.yaml` 的 `enable_normalize` **必须一致**：
  //      两处不一致时，传不传 --params-file 会得到不同行为（本仓库踩过这类坑）。
  bool   enable_normalize_{false};
  std::vector<double> norm_mean_{0.0, 0.0, 0.0};
  std::vector<double> norm_std_{1.0, 1.0, 1.0};
  // 曝光补偿开关 (V 通道直方图拉伸). 默认 false: 与 Ultralytics 训练一致。
  bool   exposure_compensation_enabled_{false};
  // 最近一次的 letterbox 变换 (每路各存一份, 供诊断/反查)
  LetterboxTransform wide_letterbox_;
  LetterboxTransform tele_letterbox_;

  // ═════════════════════════════════════════════════════════════════════════════
  // 前向相机参数
  // ═════════════════════════════════════════════════════════════════════════════
  bool   wide_clahe_enabled_{true};
  double wide_clahe_clip_{2.0};
  int    wide_clahe_tile_{8};
  /// ⚠️ 默认值必须与 `camera_params.yaml` 的 `wide_roi` / 相机实际输出一致 ——
  ///    它是**参数缺失时的兜底**，而 ROI 是内参的一部分（裁切平移主点）。
  std::vector<int64_t> wide_roi_{0, 0, 1280, 720};
  /// ⚠️⚠️ **死状态**（2026-09-26 核实）：声明 + 读参 + clamp 之后再无消费者，
  ///    本文件与 .cpp 全无引用。出界兜底靠 `sky_block_detector.cpp` 的
  ///    `roi & cv::Rect(0,0,cols,rows)` 求交，不靠它。**别在这上面加逻辑** ——
  ///    真要恢复做尺寸校验，先看清它现在**记的是错的**（两路尺寸已不同：
  ///    wide 1280×720 / tele 1624×976，而这里只有一对单值）。
  int    wide_orig_w_{1280};
  int    wide_orig_h_{720};

  // ═════════════════════════════════════════════════════════════════════════════
  // 后向相机参数
  // ═════════════════════════════════════════════════════════════════════════════
  bool   tele_clahe_enabled_{true};
  double tele_clahe_clip_{2.0};
  int    tele_clahe_tile_{8};
  /// ⚠️ 默认值必须与 `camera_params.yaml` 的 `tele_roi` / 相机实际输出一致。
  ///    2026-09-26: 长焦改成**相机内裁**到 1624×976（见 `camera_hardware.yaml`
  ///    的 `camera_tele.roi`）⇒ 相机输出即该尺寸，这里写全图（命中 `crop_roi`
  ///    的全图短路、不再裁）。**改尺寸必须两边一起改**。
  std::vector<int64_t> tele_roi_{0, 0, 1624, 976};
  /// ⚠️⚠️ **死状态** —— 同上面的 `wide_orig_*`，见那段注释
  int    tele_orig_w_{1280};
  int    tele_orig_h_{720};

  // ── 帧计数器 (诊断用) ──
  uint64_t wide_frame_seq_{0};
  uint64_t tele_frame_seq_{0};

  // ── 互斥锁 (前/后相机回调可并发) ──
  std::mutex diag_mutex_;
};

}  // namespace camera
}  // namespace br_perception
