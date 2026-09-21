/// ═══════════════════════════════════════════════════════════════════════════════
/// color_utils.cpp — ROBOCON 2027 颜色工具模块实现
///
/// 传统 CV HSV 阈值方法，用于:
///   - 建筑位颜色判定 (green_building_spot)
///   - 穆斯蒂卡识别 (v1.1 排球: yellow_ball / blue_mark / white_mark)
///   - 队伍颜色识别 (red_team / blue_team)
///   - 围栏/边界检测 (brown_fence)
///   - 核心支柱识别 (brown_core_pillar / dark_green_pillar)
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/vision/color_utils.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace br_perception {
namespace vision {

// ═══════════════════════════════════════════════════════════════════════════════
// 内部常量
// ═══════════════════════════════════════════════════════════════════════════════

namespace {

/// 颜色标签 → BGR 显示色映射表 (ROBOCON 2027 比赛场地约定)
///
/// 每个颜色的显示色选取 "足够区分" 的高饱和色，
/// 且与场地实际颜色有明显视觉差异，便于叠加后观察。
///
/// 注意: 存储为 BGR (OpenCV 默认)，直接用于 cv::rectangle/cv::drawContours 等
const std::unordered_map<std::string, cv::Scalar> kColorMapBGR = {
  // 比赛元素
  {"red_team",            cv::Scalar(  34,  34, 255)},  // BGR: 红 → (0,0,255) 显示大红
  {"blue_team",           cv::Scalar( 255,   0,  50)},  // BGR: 蓝 → (255,0,0) 显示蓝
  // ✅ 2026-09-20: 穆斯蒂卡按 v1.1 改正为**排球**(黄底+蓝纹+白字)。
  //    golden_mustika 保留仅为向后兼容; 新代码用 yellow_ball / blue_mark / white_mark。
  {"yellow_ball",         cv::Scalar(  28, 184, 255)},  // BGR: 排球黄底 (Pantone 1235C)
  {"blue_mark",           cv::Scalar( 126,  37,   3)},  // BGR: 排球蓝纹 (Pantone 3591C)
  {"white_mark",          cv::Scalar( 245, 245, 245)},  // BGR: 排球白字
  {"golden_mustika",      cv::Scalar(  32, 165, 218)},  // BGR: 金 → 已废弃, 仅兼容
  {"dark_green_pillar",   cv::Scalar(  50, 150,  40)},  // BGR: 深绿
  {"brown_core_pillar",   cv::Scalar(   0,  62, 100)},  // BGR: 棕
  {"green_building_spot", cv::Scalar(  50, 200,  40)},  // BGR: 绿 (比深绿亮一些)
  {"brown_fence",         cv::Scalar(   0,  90, 150)},  // BGR: 浅棕
  {"light_colors",        cv::Scalar( 240, 240, 240)},  // BGR: 浅灰白

  // ── 穆斯蒂卡状态细分 (用于调试可视化) ──
  {"mustika_gold",        cv::Scalar(  32, 215, 255)},  // BGR: 金
  {"mustika_silver",      cv::Scalar( 220, 220, 220)},  // BGR: 银灰
  {"mustika_empty",       cv::Scalar( 128, 128, 128)},  // BGR: 灰

  // ── 支柱状态 ──
  {"pillar_brown",        cv::Scalar(   0,  62, 100)},  // BGR: 棕
  {"pillar_green",        cv::Scalar(  50, 180,  40)},  // BGR: 绿

  // ── 通用 / 调试 ──
  {"unknown",             cv::Scalar(  64,  64,  64)},  // BGR: 暗灰
  {"default",             cv::Scalar( 255, 255, 255)},  // BGR: 白
  {"highlight",           cv::Scalar(   0, 255, 255)},  // BGR: 黄 (高亮框)
  {"rejected",            cv::Scalar(   0,   0, 255)},  // BGR: 红 (拒绝/告警)
  {"confirmed",           cv::Scalar(   0, 255,   0)},  // BGR: 绿 (确认)
};

/// YAML 中 threshold 组上限
constexpr int kMaxGroupsPerColor = 8;

}  // anonymous namespace

// ═══════════════════════════════════════════════════════════════════════════════
// 2. YAML 阈值加载
// ═══════════════════════════════════════════════════════════════════════════════

std::unordered_map<std::string, ColorThreshold> load_color_thresholds(
    const std::string& yaml_path)
{
  // ── 读文件 + 补 YAML 指令头 ──
  // ⚠️⚠️ 为什么必须补这一行 (2026-09-21 VM 实测发现):
  //   OpenCV 的 FileStorage **靠开头几个字节猜格式** (`%YAML` / `<?xml` / `{`),
  //   **不看文件后缀**。而本文件开头是 `# ═══` 注释框 → 它在 sniff 阶段就
  //   报 `Input file is invalid` 放弃 (persistence.cpp:682),
  //   **根本轮不到语法解析**。
  //   后果（踩过一次, 很贵）: 三个检测器全部静默退回硬编码阈值 ——
  //   hsv_tuner.py 存盘白存, 现场对着实物怎么调都不生效, 只 WARN 一句就过去。
  //   ⚠️ 加了表头之后, 本文件的 flow 风格 `{ lower: [...], upper: [...] }`
  //      是**能被正常解析**的 (2026-09-21 实测), 所以这里不需要改 yaml 写法。
  std::ifstream ifs(yaml_path);
  if (!ifs) {
    throw std::runtime_error("color_utils: cannot open color threshold file: " + yaml_path);
  }
  std::stringstream buf;
  buf << ifs.rdbuf();
  std::string text = buf.str();
  if (text.rfind("%YAML", 0) != 0) {
    text = "%YAML:1.0\n---\n" + text;
  }

  // ── 从内存解析（不走文件路径 —— 上面已经自己读进来了）──
  cv::FileStorage fs(text, cv::FileStorage::READ |
                           cv::FileStorage::MEMORY |
                           cv::FileStorage::FORMAT_YAML);
  if (!fs.isOpened()) {
    throw std::runtime_error("color_utils: cannot parse color threshold file: " + yaml_path);
  }

  std::unordered_map<std::string, ColorThreshold> result;

  // ── 遍历顶层节点 ──
  // YAML 顶层包含颜色标签 (如 red_team, blue_team, ...)
  // 以及非颜色条目 (如 morphology_kernel_size, min_color_pixel_ratio)
  cv::FileNode root = fs.root();

  for (auto it = root.begin(); it != root.end(); ++it)
  {
    const std::string node_name = (*it).name();

    // 跳过非颜色配置项 (标量值而非结构体)
    if (!(*it).isMap()) {
      continue;  // 如 morphology_kernel_size: 3 这种标量
    }

    // 读取 groups 数组
    cv::FileNode groups_node = (*it)["groups"];
    if (groups_node.empty()) {
      continue;
    }

    ColorThreshold ct;
    ct.label = node_name;

    for (auto g_it = groups_node.begin();
         g_it != groups_node.end() && ct.groups.size() < kMaxGroupsPerColor;
         ++g_it)
    {
      HsvRange range;
      (*g_it)["lower"] >> range.lower;
      (*g_it)["upper"] >> range.upper;
      ct.groups.push_back(range);
    }

    if (!ct.groups.empty()) {
      result[node_name] = ct;
    }
  }

  fs.release();
  return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. 像素颜色判定
// ═══════════════════════════════════════════════════════════════════════════════

bool pixel_in_color_range(
    const cv::Vec3b& hsv,
    const ColorThreshold& threshold,
    int h_tolerance,
    int s_tolerance,
    int v_tolerance)
{
  for (const auto& range : threshold.groups) {
    if (pixel_in_hsv_range(hsv, range, h_tolerance, s_tolerance, v_tolerance)) {
      return true;  // 短路: 任一组匹配
    }
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. 区域颜色像素占比统计
// ═══════════════════════════════════════════════════════════════════════════════

double color_pixel_ratio(
    const cv::Mat& hsv_image,
    const cv::Rect& roi,
    const ColorThreshold& threshold,
    int h_tolerance,
    int s_tolerance,
    int v_tolerance)
{
  CV_Assert(hsv_image.type() == CV_8UC3);

  // ── 修正 ROI 边界 ──
  cv::Rect valid_roi = roi & cv::Rect(0, 0, hsv_image.cols, hsv_image.rows);
  if (valid_roi.width <= 0 || valid_roi.height <= 0) {
    return 0.0;
  }

  const int total_pixels = valid_roi.area();
  int match_count = 0;

  // ── 遍历 ROI ──
  for (int y = valid_roi.y; y < valid_roi.y + valid_roi.height; ++y) {
    const uint8_t* row = hsv_image.ptr<uint8_t>(y);
    for (int x = valid_roi.x; x < valid_roi.x + valid_roi.width; ++x) {
      const int idx = x * 3;
      const cv::Vec3b hsv(row[idx], row[idx + 1], row[idx + 2]);

      if (pixel_in_color_range(hsv, threshold,
                               h_tolerance, s_tolerance, v_tolerance)) {
        ++match_count;
      }
    }
  }

  return static_cast<double>(match_count) / static_cast<double>(total_pixels);
}

double color_pixel_ratio(
    const cv::Mat& hsv_image,
    const cv::Mat& mask,
    const ColorThreshold& threshold,
    int h_tolerance,
    int s_tolerance,
    int v_tolerance)
{
  CV_Assert(hsv_image.type() == CV_8UC3);
  CV_Assert(mask.type() == CV_8UC1);
  CV_Assert(hsv_image.size() == mask.size());

  int total_masked = 0;
  int match_count = 0;

  for (int y = 0; y < hsv_image.rows; ++y) {
    const uint8_t* hsv_row = hsv_image.ptr<uint8_t>(y);
    const uint8_t* mask_row = mask.ptr<uint8_t>(y);

    for (int x = 0; x < hsv_image.cols; ++x) {
      if (mask_row[x] == 0) continue;
      ++total_masked;

      const int idx = x * 3;
      const cv::Vec3b hsv(hsv_row[idx], hsv_row[idx + 1], hsv_row[idx + 2]);

      if (pixel_in_color_range(hsv, threshold,
                               h_tolerance, s_tolerance, v_tolerance)) {
        ++match_count;
      }
    }
  }

  if (total_masked == 0) return 0.0;
  return static_cast<double>(match_count) / static_cast<double>(total_masked);
}

std::unordered_map<std::string, double> color_pixel_ratios_batch(
    const cv::Mat& hsv_image,
    const cv::Rect& roi,
    const std::unordered_map<std::string, ColorThreshold>& thresholds)
{
  CV_Assert(hsv_image.type() == CV_8UC3);

  // ── 准备计数器 ──
  std::unordered_map<std::string, int> match_counts;
  for (const auto& [label, _] : thresholds) {
    match_counts[label] = 0;
  }

  // ── 修正 ROI ──
  cv::Rect valid_roi = roi;
  if (valid_roi.width <= 0 || valid_roi.height <= 0) {
    valid_roi = cv::Rect(0, 0, hsv_image.cols, hsv_image.rows);
  }
  valid_roi &= cv::Rect(0, 0, hsv_image.cols, hsv_image.rows);
  if (valid_roi.width <= 0 || valid_roi.height <= 0) {
    return {};
  }

  const int total_pixels = valid_roi.area();

  // ── 遍历 ROI: 每像素匹配所有颜色 ──
  for (int y = valid_roi.y; y < valid_roi.y + valid_roi.height; ++y) {
    const uint8_t* row = hsv_image.ptr<uint8_t>(y);
    for (int x = valid_roi.x; x < valid_roi.x + valid_roi.width; ++x) {
      const int idx = x * 3;
      const cv::Vec3b hsv(row[idx], row[idx + 1], row[idx + 2]);

      // 对每个颜色阈值检查
      for (const auto& [label, threshold] : thresholds) {
        if (pixel_in_color_range(hsv, threshold)) {
          ++match_counts[label];
        }
      }
    }
  }

  // ── 转为占比 ──
  std::unordered_map<std::string, double> result;
  if (total_pixels > 0) {
    for (const auto& [label, count] : match_counts) {
      result[label] = static_cast<double>(count) / static_cast<double>(total_pixels);
    }
  }

  return result;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. 颜色标签 → BGR 显示色映射
// ═══════════════════════════════════════════════════════════════════════════════

cv::Scalar label_to_bgr(const std::string& label)
{
  auto it = kColorMapBGR.find(label);
  if (it != kColorMapBGR.end()) {
    return it->second;
  }
  return kColorMapBGR.at("default");  // 白色兜底
}

const std::unordered_map<std::string, cv::Scalar>& get_color_map()
{
  return kColorMapBGR;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6. 光照归一化
// ═══════════════════════════════════════════════════════════════════════════════

void gray_world_normalize(
    cv::InputArray src,
    cv::OutputArray dst,
    cv::Rect ref_rect)
{
  cv::Mat input = src.getMat();
  CV_Assert(input.type() == CV_8UC3);

  // ── 计算参考区域均值 ──
  cv::Rect valid_rect = ref_rect;
  if (valid_rect.width <= 0 || valid_rect.height <= 0) {
    valid_rect = cv::Rect(0, 0, input.cols, input.rows);
  }
  valid_rect &= cv::Rect(0, 0, input.cols, input.rows);

  cv::Mat ref_roi = input(valid_rect);
  cv::Scalar mean_bgr = cv::mean(ref_roi);

  // 灰度目标: 三通道均值的平均 (保持整体亮度)
  const double gray_target = (mean_bgr[0] + mean_bgr[1] + mean_bgr[2]) / 3.0;

  // ── 计算增益 ──
  double gain_b = gray_target / std::max(mean_bgr[0], 1.0);
  double gain_g = gray_target / std::max(mean_bgr[1], 1.0);
  double gain_r = gray_target / std::max(mean_bgr[2], 1.0);

  // ── 应用增益 ──
  dst.create(input.size(), CV_8UC3);
  cv::Mat output = dst.getMat();

  // 使用查找表 (LUT) 加速
  uint8_t lut_b[256], lut_g[256], lut_r[256];
  for (int i = 0; i < 256; ++i) {
    lut_b[i] = cv::saturate_cast<uint8_t>(i * gain_b);
    lut_g[i] = cv::saturate_cast<uint8_t>(i * gain_g);
    lut_r[i] = cv::saturate_cast<uint8_t>(i * gain_r);
  }

  const int total_pixels = input.rows * input.cols;
  uint8_t* out_data = output.ptr<uint8_t>(0);
  const uint8_t* in_data = input.ptr<uint8_t>(0);

  for (int i = 0; i < total_pixels * 3; i += 3) {
    out_data[i]     = lut_b[in_data[i]];       // B
    out_data[i + 1] = lut_g[in_data[i + 1]];   // G
    out_data[i + 2] = lut_r[in_data[i + 2]];   // R
  }
}

void white_patch_normalize(
    cv::InputArray src,
    cv::OutputArray dst,
    const cv::Rect& ref_rect)
{
  cv::Mat input = src.getMat();
  CV_Assert(input.type() == CV_8UC3);
  CV_Assert(ref_rect.width > 0 && ref_rect.height > 0);
  CV_Assert((ref_rect & cv::Rect(0, 0, input.cols, input.rows)) == ref_rect);

  // ── 参考区域均值作为白点 ──
  cv::Mat ref_roi = input(ref_rect);
  cv::Scalar white_bgr = cv::mean(ref_roi);

  // 白点应接近 (255, 255, 255)，计算增益
  double gain_b = 255.0 / std::max(white_bgr[0], 1.0);
  double gain_g = 255.0 / std::max(white_bgr[1], 1.0);
  double gain_r = 255.0 / std::max(white_bgr[2], 1.0);

  // 限制最大增益，避免噪声放大
  gain_b = std::min(gain_b, 3.0);
  gain_g = std::min(gain_g, 3.0);
  gain_r = std::min(gain_r, 3.0);

  // ── LUT 应用 ──
  dst.create(input.size(), CV_8UC3);
  cv::Mat output = dst.getMat();

  uint8_t lut_b[256], lut_g[256], lut_r[256];
  for (int i = 0; i < 256; ++i) {
    lut_b[i] = cv::saturate_cast<uint8_t>(i * gain_b);
    lut_g[i] = cv::saturate_cast<uint8_t>(i * gain_g);
    lut_r[i] = cv::saturate_cast<uint8_t>(i * gain_r);
  }

  const int total_pixels = input.rows * input.cols;
  uint8_t* out_data = output.ptr<uint8_t>(0);
  const uint8_t* in_data = input.ptr<uint8_t>(0);

  for (int i = 0; i < total_pixels * 3; i += 3) {
    out_data[i]     = lut_b[in_data[i]];
    out_data[i + 1] = lut_g[in_data[i + 1]];
    out_data[i + 2] = lut_r[in_data[i + 2]];
  }
}

void histogram_normalize(
    cv::InputArray src,
    cv::OutputArray dst,
    double clip_percent)
{
  cv::Mat input = src.getMat();
  CV_Assert(input.type() == CV_8UC3);

  // ── 转为 HSV，在 V 通道上做拉伸 ──
  cv::Mat hsv;
  cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

  // 分离通道
  std::vector<cv::Mat> channels(3);
  cv::split(hsv, channels);
  cv::Mat& v_channel = channels[2];  // V = value/brightness

  // ── 计算 V 通道直方图 ──
  int hist_size = 256;
  float range[] = {0, 256};
  const float* hist_range = {range};
  cv::Mat hist;
  cv::calcHist(&v_channel, 1, nullptr, cv::Mat(), hist, 1, &hist_size, &hist_range);

  // ── clip 两端 ──
  const double total = v_channel.total();
  const double clip_count = total * (clip_percent / 100.0);

  // 从低端累加
  int v_min = 0;
  {
    double accum = 0;
    for (int i = 0; i < 256; ++i) {
      accum += hist.at<float>(i);
      if (accum > clip_count) {
        v_min = i;
        break;
      }
    }
  }

  // 从高端累加
  int v_max = 255;
  {
    double accum = 0;
    for (int i = 255; i >= 0; --i) {
      accum += hist.at<float>(i);
      if (accum > clip_count) {
        v_max = i;
        break;
      }
    }
  }

  // ── 拉伸 V 通道 ──
  if (v_max > v_min) {
    const double scale = 255.0 / static_cast<double>(v_max - v_min);
    for (int y = 0; y < v_channel.rows; ++y) {
      uint8_t* row = v_channel.ptr<uint8_t>(y);
      for (int x = 0; x < v_channel.cols; ++x) {
        int val = row[x];
        val = cv::saturate_cast<uint8_t>((val - v_min) * scale);
        row[x] = static_cast<uint8_t>(val);
      }
    }
  }

  // ── 合并回 BGR ──
  cv::merge(channels, hsv);
  cv::cvtColor(hsv, dst, cv::COLOR_HSV2BGR);
}

}  // namespace vision
}  // namespace br_perception
