#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// color_utils.hpp — ROBOCON 2027 传统 CV 颜色工具模块
///
/// 依赖: OpenCV (core + imgproc)
/// 职责:
///   1. RGB ↔ HSV 高效转换
///   2. 从 color_thresholds.yaml 加载 HSV 颜色阈值
///   3. 判断像素是否属于指定颜色范围（带容差）
///   4. 统计区域内指定颜色的像素占比
///   5. 颜色标签 → RGB/BGR 颜色映射（用于可视化）
///   6. 光照归一化辅助函数（基于白平衡参考）
/// ═══════════════════════════════════════════════════════════════════════════════

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace br_perception {
namespace vision {

// ═══════════════════════════════════════════════════════════════════════════════
// 数据结构
// ═══════════════════════════════════════════════════════════════════════════════

/// @brief 单组 HSV 阈值范围
///
/// OpenCV HSV 约定:
///   H ∈ [0, 180)   (实际 H*2 = 360°)
///   S ∈ [0, 255]
///   V ∈ [0, 255]
struct HsvRange
{
  cv::Scalar lower;   // (H_min, S_min, V_min)
  cv::Scalar upper;   // (H_max, S_max, V_max)

  HsvRange() = default;
  HsvRange(const cv::Scalar& lo, const cv::Scalar& hi)
    : lower(lo), upper(hi) {}
};

/// @brief 一个颜色标签的完整阈值配置
///
/// 每个颜色可配置多组阈值 (如红色跨 0°/180° 边界需两段)，
/// 检测时取并集: mask = group0 | group1 | group2 | ...
struct ColorThreshold
{
  std::string label;
  std::vector<HsvRange> groups;
};

// ═══════════════════════════════════════════════════════════════════════════════
// 1. RGB → HSV 高效转换
// ═══════════════════════════════════════════════════════════════════════════════

/// @brief RGB (0-255) → HSV (OpenCV 范围)
///
/// 直接基于整数运算，避免浮点除法在 CPU 上的延迟。
/// 返回值: (H, S, V) 均 uint8_t
///   H ∈ [0, 180)  理由: (360°/2) 适配 OpenCV 8U 存储
///   S ∈ [0, 255]
///   V ∈ [0, 255]
///
/// 参考: https://en.wikipedia.org/wiki/HSL_and_HSV
inline cv::Vec3b rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b) noexcept
{
  // ── 快速路径: 灰度像素 ──
  if (r == g && g == b) {
    return cv::Vec3b(0, 0, r);   // H=0, S=0, V=r
  }

  uint8_t max_val = r;
  uint8_t min_val = r;
  if (g > max_val) max_val = g;
  if (b > max_val) max_val = b;
  if (g < min_val) min_val = g;
  if (b < min_val) min_val = b;

  const uint8_t delta = max_val - min_val;

  // V
  const uint8_t v = max_val;

  // S — 使用 255*delta/max 的整数近似
  //   255 * delta / max  → 避免浮点
  //   使用 16-bit 中间值防溢出 (255*255 = 65025 < 65535)
  const uint8_t s = (max_val > 0)
    ? static_cast<uint8_t>((static_cast<uint16_t>(255) * delta) / max_val)
    : 0;

  // H — 0~360°, 再 ÷2 映射到 OpenCV [0, 180)
  uint8_t h = 0;

  if (delta > 0) {
    int16_t hue = 0;  // 0~360 范围

    if (max_val == r) {
      hue = static_cast<int16_t>(60) * (static_cast<int16_t>(g) - static_cast<int16_t>(b)) / delta;
    } else if (max_val == g) {
      hue = static_cast<int16_t>(60) * (static_cast<int16_t>(b) - static_cast<int16_t>(r)) / delta + 120;
    } else {  // max_val == b
      hue = static_cast<int16_t>(60) * (static_cast<int16_t>(r) - static_cast<int16_t>(g)) / delta + 240;
    }

    // 规范化到 [0, 360)
    if (hue < 0)   hue += 360;
    if (hue >= 360) hue -= 360;

    // OpenCV: H/2 ∈ [0, 180)
    h = static_cast<uint8_t>(hue / 2);
  }

  return cv::Vec3b(h, s, v);
}

/// @brief RGB pixel → HSV (重载，直接传 Vec3b)
inline cv::Vec3b rgb_to_hsv(const cv::Vec3b& rgb) noexcept
{
  return rgb_to_hsv(rgb[2], rgb[1], rgb[0]);
  // NOTE: cv::Vec3b 默认 BGR 顺序 → 参数传 r=g[2], g=g[1], b=g[0]
}

/// @brief BGR pixel → HSV (与 OpenCV 存储顺序一致)
inline cv::Vec3b bgr_to_hsv(const cv::Vec3b& bgr) noexcept
{
  return rgb_to_hsv(bgr[2], bgr[1], bgr[0]);
}

/// @brief 批量 BGR → HSV (对整个 Mat 逐像素转换，适合小 ROI)
/// @param bgr  Input BGR 3-channel image (CV_8UC3)
/// @param hsv  Output HSV 3-channel image
/// @note 大图请直接用 cv::cvtColor 并在 GPU/NEON 路径上受益；
///       本函数仅用于小 ROI 或需逐像素精确控制的场景
inline void bgr_to_hsv_roi(cv::InputArray bgr, cv::OutputArray hsv)
{
  cv::Mat src = bgr.getMat();
  CV_Assert(src.type() == CV_8UC3);

  hsv.create(src.size(), CV_8UC3);
  cv::Mat dst = hsv.getMat();

  const int rows = src.rows;
  const int cols = src.cols;

  // 并行化 (OpenCV parallel_for_)
  cv::parallel_for_(cv::Range(0, rows), [&](const cv::Range& range) {
    for (int y = range.start; y < range.end; ++y) {
      const uint8_t* src_row = src.ptr<uint8_t>(y);
      uint8_t* dst_row = dst.ptr<uint8_t>(y);
      for (int x = 0; x < cols; ++x) {
        const int idx = x * 3;
        const auto& hsv_px = bgr_to_hsv(
            cv::Vec3b(src_row[idx], src_row[idx + 1], src_row[idx + 2]));
        dst_row[idx]     = hsv_px[0];
        dst_row[idx + 1] = hsv_px[1];
        dst_row[idx + 2] = hsv_px[2];
      }
    }
  });
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2. YAML 阈值加载
// ═══════════════════════════════════════════════════════════════════════════════

/// @brief 从 color_thresholds.yaml 加载所有颜色阈值
///
/// 使用 OpenCV FileStorage 解析 YAML，免外部 yaml-cpp 依赖。
/// 输入 YAML 格式示例:
///   red_team:
///     groups:
///       - { lower: [0, 100, 50], upper: [8, 255, 255] }
///       - { lower: [170, 100, 50], upper: [180, 255, 255] }
///
/// @param yaml_path  color_thresholds.yaml 的完整路径
/// @return  color_label → ColorThreshold 映射表
/// @throws cv::Exception 若文件不存在或格式错误
std::unordered_map<std::string, ColorThreshold> load_color_thresholds(
    const std::string& yaml_path);

// ═══════════════════════════════════════════════════════════════════════════════
// 3. 像素颜色判定
// ═══════════════════════════════════════════════════════════════════════════════

/// @brief 判断一个 HSV 像素是否落在指定颜色范围内
///
/// 对 ColorThreshold 中所有 groups 取并集，
/// 任一 group 匹配即返回 true。
/// 支持每通道独立容差 (tolerance 向外扩展阈值)。
///
/// @param hsv         像素 HSV 值 (H:0-180, S:0-255, V:0-255)
/// @param threshold   颜色阈值集合 (可含多组)
/// @param h_tolerance H 通道容差 (向外扩展 ±h_tol)
/// @param s_tolerance S 通道容差
/// @param v_tolerance V 通道容差
/// @return true 像素属于该颜色
bool pixel_in_color_range(
    const cv::Vec3b& hsv,
    const ColorThreshold& threshold,
    int h_tolerance = 0,
    int s_tolerance = 0,
    int v_tolerance = 0);

/// @brief 判断一个 HSV 像素是否落在单组 HSV range 内 (带容差)
inline bool pixel_in_hsv_range(
    const cv::Vec3b& hsv,
    const HsvRange& range,
    int h_tol = 0, int s_tol = 0, int v_tol = 0) noexcept
{
  const int h = hsv[0];
  const int s = hsv[1];
  const int v = hsv[2];

  const int h_lo = range.lower[0] - h_tol;
  const int h_hi = range.upper[0] + h_tol;
  const int s_lo = range.lower[1] - s_tol;
  const int s_hi = range.upper[1] + s_tol;
  const int v_lo = range.lower[2] - v_tol;
  const int v_hi = range.upper[2] + v_tol;

  // H 通道: 处理跨 0/180 边界情况
  bool h_ok = false;
  if (h_lo <= h_hi) {
    // 正常区间 (如 [100, 130])
    h_ok = (h >= h_lo && h <= h_hi);
  } else {
    // 跨边界区间 (如 [170, 8] → 170~180 ∪ 0~8)
    h_ok = (h >= h_lo || h <= h_hi);
  }

  return h_ok &&
         (s >= s_lo && s <= s_hi) &&
         (v >= v_lo && v <= v_hi);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. 区域颜色像素占比统计
// ═══════════════════════════════════════════════════════════════════════════════

/// @brief 统计图像 ROI 区域内指定颜色的像素占比
///
/// @param hsv_image   HSV 图像 (CV_8UC3, H:0-180 S:0-255 V:0-255)
/// @param roi         感兴趣区域
/// @param threshold   颜色阈值
/// @param h_tolerance S/V 通道容差
/// @param s_tolerance
/// @param v_tolerance
/// @return 匹配像素占比 [0.0, 1.0]
double color_pixel_ratio(
    const cv::Mat& hsv_image,
    const cv::Rect& roi,
    const ColorThreshold& threshold,
    int h_tolerance = 0,
    int s_tolerance = 0,
    int v_tolerance = 0);

/// @brief 统计 mask 区域内指定颜色的像素占比
///
/// 只统计 mask != 0 的像素。
///
/// @param hsv_image   HSV 图像
/// @param mask        二值 mask (CV_8UC1, 非零=有效区域)
/// @param threshold   颜色阈值
/// @param h_tolerance
/// @param s_tolerance
/// @param v_tolerance
/// @return 匹配像素占比 [0.0, 1.0]；若 mask 为空则返回 0
double color_pixel_ratio(
    const cv::Mat& hsv_image,
    const cv::Mat& mask,
    const ColorThreshold& threshold,
    int h_tolerance = 0,
    int s_tolerance = 0,
    int v_tolerance = 0);

/// @brief 批量统计：一次扫描计算所有颜色标签的占比
///
/// 对 ROI 内的每个像素，遍历所有颜色阈值，累计匹配数。
/// 比多次调用 color_pixel_ratio() 更高效。
///
/// @param hsv_image  HSV 图像
/// @param roi        感兴趣区域 (空 Rect = 全图)
/// @param thresholds 所有需要统计的颜色阈值
/// @return color_label → ratio [0.0, 1.0]
std::unordered_map<std::string, double> color_pixel_ratios_batch(
    const cv::Mat& hsv_image,
    const cv::Rect& roi,
    const std::unordered_map<std::string, ColorThreshold>& thresholds);

// ═══════════════════════════════════════════════════════════════════════════════
// 5. 颜色标签 → RGB/BGR 颜色映射 (可视化)
// ═══════════════════════════════════════════════════════════════════════════════

/// @brief 获取颜色标签对应的 BGR 显示色
///
/// 用途: 在输出图像上绘制检测结果时，用约定颜色绘制对应目标的边界框/轮廓。
///
/// @param label  颜色标签名 (如 "red_team", "blue_team", "yellow_ball")
/// @return BGR Scalar  (默认: 白色)
cv::Scalar label_to_bgr(const std::string& label);

/// @brief 获取颜色标签对应的 RGB 显示色
inline cv::Scalar label_to_rgb(const std::string& label)
{
  // BGR → RGB: 交换 [0] 和 [2]
  cv::Scalar bgr = label_to_bgr(label);
  return cv::Scalar(bgr[2], bgr[1], bgr[0], bgr[3]);
}

/// @brief 获取所有已知颜色标签 → BGR 映射表
///
/// 可遍历此表来绘制图例:
///   for (const auto& [label, bgr] : get_color_map()) { ... }
const std::unordered_map<std::string, cv::Scalar>& get_color_map();

// ═══════════════════════════════════════════════════════════════════════════════
// 6. 光照归一化辅助函数
// ═══════════════════════════════════════════════════════════════════════════════

/// @brief 基于"灰世界假设"的白平衡归一化
///
/// 算法:
///   1. 计算参考区域 (ref_rect) 或全图的各通道均值
///   2. 计算各通道增益: gain[c] = gray_target / mean[c]
///   3. 所有像素乘以增益 (clip to 0~255)
///
/// 默认 gray_target 取三通道均值的平均值 (保持整体亮度不变)。
///
/// 优点: 计算量 O(N)，适合实时管线 (每帧 < 1ms @1080p)。
/// 局限: 假设场景平均反射率为中性灰; 大色块场景可能偏色。
///       可配合 ref_rect 参数 → 使用白平衡参考卡区域。
///
/// @param src      输入 BGR 图像 (CV_8UC3)
/// @param dst      输出 BGR 图像
/// @param ref_rect 参考白/灰基准区域 (默认全图)
void gray_world_normalize(
    cv::InputArray src,
    cv::OutputArray dst,
    cv::Rect ref_rect = cv::Rect());

/// @brief 基于参考白的白平衡归一化
///
/// 假设 ref_rect 区域为纯白 (如白板、白线)，
/// 直接以该区域的 RGB 均值作为白点，拉伸各通道。
///
/// @param src      输入 BGR 图像
/// @param dst      输出 BGR 图像
/// @param ref_rect 白色参考区域 (必须有效)
void white_patch_normalize(
    cv::InputArray src,
    cv::OutputArray dst,
    const cv::Rect& ref_rect);

/// @brief 简单的亮度直方图拉伸 (对比度归一化)
///
/// 对 V 通道做 min-max 拉伸到 [0, 255]，
/// 同时可选 clip 两端各 clip_percent% 的像素以减少噪声影响。
///
/// @param src           输入 BGR 图像
/// @param dst           输出 BGR 图像
/// @param clip_percent  裁剪两端像素百分比 (0.0 ~ 5.0)
void histogram_normalize(
    cv::InputArray src,
    cv::OutputArray dst,
    double clip_percent = 0.5);

}  // namespace vision
}  // namespace br_perception
