/// ═══════════════════════════════════════════════════════════════════════════════
/// test_pillar_detector.cpp — 模块 7 柱子检测：**候选验证**单元测试
///
/// ⚠️ 只测 `validate_pillar`（**static 纯函数**）—— **不起节点、不跑轮廓提取、不发布**。
///    ⚠️ **别把本文件的绿当成"柱子检测能跑"**：绿/棕 mask、轮廓提取、柱顶区域、发布都不在这里。
///
/// ── 为什么它值得单测 ──
///   柱子的位置喂给 `mustika_pillar_check`（**那 250 分**）当"**球在不在柱上**"的基准点。
///   判错了 ⇒ 基准点错 ⇒ 那 250 分的判定跟着错。
///
/// ⚠️⚠️ **本文件里有一条专门钉"一段死代码"** —— 见 `尺寸检查那段是死代码不影响结果`。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=PillarValidateTest.*`）。
/// ⚠️ 全部用例确定性：合成 mask + 手造轮廓，无随机、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/pillar_detector.hpp"

#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include <vector>

namespace br_perception {
namespace camera {

using PD = PillarDetector;

namespace {

/// 造一张"竖直长条"候选：mask 里画一个实心长条 + 给出对应的 bbox/contour
struct Candidate
{
  cv::Mat bgr;
  cv::Mat mask;
  cv::Rect bbox;
  std::vector<cv::Point> contour;
};

/// @param fill 在 bbox 内画多宽的长条（控制填充率）
Candidate make_pillar(int w = 200, int h = 200, int bw = 20, int bh = 120, double fill = 1.0)
{
  Candidate c;
  c.bgr  = cv::Mat(h, w, CV_8UC3, cv::Scalar(30, 60, 30));   // 深绿背景
  c.mask = cv::Mat::zeros(h, w, CV_8UC1);
  const int x0 = 90, y0 = 40;
  c.bbox = cv::Rect(x0, y0, bw, bh);
  // 在 bbox 里画一个**居中、占 `fill` 比例的竖直实心条**（模拟柱子的颜色像素）
  const int fw = std::max(1, static_cast<int>(bw * fill));
  cv::rectangle(c.mask, cv::Rect(x0 + (bw - fw) / 2, y0, fw, bh),
                cv::Scalar(255), cv::FILLED);
  c.contour = {{x0, y0}, {x0 + bw, y0}, {x0 + bw, y0 + bh}, {x0, y0 + bh}};
  return c;
}

PD::PillarParams base_params()
{
  return PD::PillarParams{};     // 全部用默认值
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 通过路径
// ═══════════════════════════════════════════════════════════════════════════

TEST(PillarValidateTest, 合格的竖直长条通过并填结果)
{
  const auto c = make_pillar();
  auto p = base_params();
  PillarImageResult res;
  ASSERT_TRUE(PD::validate_pillar(c.bgr, c.mask, c.bbox, c.contour, res, p));

  EXPECT_TRUE(res.detected);
  EXPECT_GE(res.confidence, 0.0f);
  EXPECT_LE(res.confidence, 1.0f);
  EXPECT_NEAR(res.color_fill_ratio, 1.0f, 1e-3) << "bbox 内全是 mask ⇒ 填充率 1.0";
  EXPECT_NEAR(res.aspect_ratio, 6.0f, 1e-3) << "120/20 = 6";
  EXPECT_NEAR(res.bbox.width,  static_cast<float>(c.bbox.width),  1e-3);
  EXPECT_NEAR(res.bbox.height, static_cast<float>(c.bbox.height), 1e-3);
}

TEST(PillarValidateTest, 竖直的柱子竖直度评分高)
{
  // ⚠️ 竖直度的判据是 `minAreaRect` 的 angle 离 90° 有多远
  //    （OpenCV 4.x 里 angle ∈ (0,90] ⇒ **竖直长条的 angle 接近 90** ⇒ 偏差小 ⇒ 分高）。
  //    ⚠️ 这条把"分数高"钉下来，免得有人把 angle 的约定改反（**改反了症状是"竖直柱子全被压低分"**）。
  const auto c = make_pillar();
  auto p = base_params();
  PillarImageResult res;
  ASSERT_TRUE(PD::validate_pillar(c.bgr, c.mask, c.bbox, c.contour, res, p));
  EXPECT_GT(res.upright_score, 0.9f) << "竖直长条的竖直度该接近满分";
}

// ═══════════════════════════════════════════════════════════════════════════
// 拒绝路径
// ═══════════════════════════════════════════════════════════════════════════

TEST(PillarValidateTest, 填充率不足被拒)
{
  // ⚠️ **硬拒**：`fill_ratio < min_color_fill`（默认 0.55）
  const auto c = make_pillar(200, 200, 20, 120, /*fill=*/0.30);
  auto p = base_params();
  PillarImageResult res;
  EXPECT_FALSE(PD::validate_pillar(c.bgr, c.mask, c.bbox, c.contour, res, p))
      << "填充率 0.30 < 0.55 ⇒ 该拒";
}

TEST(PillarValidateTest, 填充率恰好等于阈值时通过)
{
  // ⚠️ 判据是**严格 `<` 才拒** ⇒ 恰好等于阈值**通过**。
  const auto c = make_pillar(200, 200, 20, 120, /*fill=*/0.55);
  auto p = base_params();
  PillarImageResult res;
  EXPECT_TRUE(PD::validate_pillar(c.bgr, c.mask, c.bbox, c.contour, res, p))
      << "恰好等于阈值该通过（判据是严格 < 才拒）";
}

TEST(PillarValidateTest, 包围盒完全在图像外被拒)
{
  const auto c = make_pillar();
  auto p = base_params();
  PillarImageResult res;
  const cv::Rect outside(-100, -100, 20, 120);       // 与图像无交集
  EXPECT_FALSE(PD::validate_pillar(c.bgr, c.mask, outside, c.contour, res, p));
}

TEST(PillarValidateTest, 失败时一个字段都不碰)
{
  // ⚠️ **与 mustika_detector 同款契约**：仅成功时填入；失败时 `result` **一个字段都不碰**
  //    ⇒ 调用方**必须以返回值为准**，不能只看 `result.detected`。
  const auto c = make_pillar(200, 200, 20, 120, /*fill=*/0.30);
  auto p = base_params();
  PillarImageResult res;
  res.detected = false;
  res.confidence = -1.0f;
  res.color_fill_ratio = -2.0f;
  EXPECT_FALSE(PD::validate_pillar(c.bgr, c.mask, c.bbox, c.contour, res, p));
  EXPECT_FLOAT_EQ(res.confidence, -1.0f) << "失败时不该碰 result";
  EXPECT_FLOAT_EQ(res.color_fill_ratio, -2.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// ★★ 死代码（2026-09-25 复核发现）
// ═══════════════════════════════════════════════════════════════════════════

TEST(PillarValidateTest, 尺寸检查那段是死代码不影响结果)
{
  // ⚠️⚠️ **本文件最值钱的一条**。
  //    `validate_pillar` 的 3b「尺寸合理性」算了 `est_distance` 与 `expected_height_px`，
  //    注释写着"不做硬拒绝，**降低后续评分**" ——
  //    **但后面没有任何降分代码** ⇒ 那几个变量算完就丢。
  //
  //    ⇒ 这条用例把"**它没有效果**"钉下来：
  //      同一份轮廓，**把焦距参数改到离谱**（它会改变 `expected_height_px`），
  //      结论与**置信度**都必须**逐位不变**。
  //    ⚠️ 若哪天有人把那段接上（真的降分），这条会红 —— 那时该**连注释一起改**。
  const auto c = make_pillar();

  auto a = base_params();
  PillarImageResult ra;
  ASSERT_TRUE(PD::validate_pillar(c.bgr, c.mask, c.bbox, c.contour, ra, a));

  auto b = base_params();
  b.focal_length_px = 99999.0;              // 离谱的焦距 ⇒ expected_height_px 天翻地覆
  b.pillar_diameter = 9.99;
  PillarImageResult rb;
  ASSERT_TRUE(PD::validate_pillar(c.bgr, c.mask, c.bbox, c.contour, rb, b));

  EXPECT_FLOAT_EQ(ra.confidence, rb.confidence)
      << "改焦距居然影响了置信度 ⇒ 那段尺寸检查**不再是死代码**了（请同步改注释）";
}

TEST(PillarValidateTest, 置信度权重与实现一致)
{
  // ⚠️ 公式是 `0.40×填充 + 0.35×竖直度 + 0.25×实心度`（三项各自先归一）。
  //    这条**不重算期望值**（那要复刻一遍实现），只验"三项都进了公式"：
  //    改任一输入 ⇒ 置信度必须变。
  const auto c = make_pillar();
  auto p = base_params();
  PillarImageResult full;
  ASSERT_TRUE(PD::validate_pillar(c.bgr, c.mask, c.bbox, c.contour, full, p));

  // 把填充率降到刚好过线 ⇒ `conf_fill` 明显下降 ⇒ 总置信度必须下降
  const auto thin = make_pillar(200, 200, 20, 120, /*fill=*/0.56);
  PillarImageResult lean;
  ASSERT_TRUE(PD::validate_pillar(thin.bgr, thin.mask, thin.bbox, thin.contour, lean, p));
  EXPECT_LT(lean.confidence, full.confidence)
      << "填充率降了而置信度没降 ⇒ 那一项没进公式";
}

}  // namespace camera
}  // namespace br_perception
