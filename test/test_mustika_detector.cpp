/// ═══════════════════════════════════════════════════════════════════════════════
/// test_mustika_detector.cpp — 模块 5 穆斯蒂卡球检测：**纯函数**单元测试
///
/// ⚠️ 只测 `extract_golden_mask`（金色 mask）与 `validate_circle`（候选圆验证）——
///    **不起节点、不跑 HoughCircles、不订阅**。
///    ⚠️ **别把本文件的绿当成"检测器能跑"**：Hough 参数、回调、反算、发布都不在这里。
///
/// ── 为什么这个模块值得单测（它出的是**那 250 分**那条链的输入）──
///   `mustika_pillar_check` 拿它检出的球去判"在柱上吗"，而那 250 分是规则 4.5.1 的得分动作。
///   而它的三个阈值 **2026-09-20 改过值**（半径 0.065→0.105、圆形度 0.85→0.70、
///   黄底占比 0.60→0.50）—— 台账 P1-9 原话是「**需重编重测**」。**本文件就是"重测"。**
///
/// ⚠️⚠️ **本文件里最贵的一条是 `半径判据是两套来源取交集`** —— 见那条用例的说明。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=GoldenMaskTest.*`）。
/// ⚠️ 全部用例确定性：合成图（HSV 造色 → 转 BGR），无随机、无外部文件、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/mustika_detector.hpp"

#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include <vector>

namespace br_perception {
namespace camera {

using MD = MustikaDetector;

namespace {

/// ⚠️ **与 `config/color_thresholds.yaml` 的 `yellow_ball` 逐字一致**（2026-09-25 抄的）。
///    硬编码是有意的：单测不该依赖 yaml 文件在不在；
///    ⚠️ 但**改 yaml 就要改这里** —— 那正是"重编重测"的意思。
void add_real_golden(MD::MustikaParams& p)
{
  p.golden_groups.push_back({cv::Scalar(20, 60, 100), cv::Scalar(50, 255, 255)});
  p.golden_groups.push_back({cv::Scalar(18, 50,  80), cv::Scalar(55, 255, 230)});  // 暗光补偿
}

/// 造一张纯色 BGR（**在 HSV 空间造再转** —— 保证转回来一定落在预期区间里）
cv::Mat solid(int h, int s, int v, int w = 200, int hh = 200)
{
  cv::Mat hsv(hh, w, CV_8UC3, cv::Scalar(h, s, v));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}

/// 非金背景上画一个金色圆盘（**先造一个金色圆、周围是蓝的**，再整体转 BGR）
cv::Mat golden_disc(int w, int h, int cx, int cy, int r)
{
  cv::Mat hsv(h, w, CV_8UC3, cv::Scalar(110, 200, 220));           // 蓝底（非金）
  cv::circle(hsv, cv::Point(cx, cy), r, cv::Scalar(35, 200, 220), cv::FILLED);  // 金圆(H=35)
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr;
}

MD::MustikaParams base_params()
{
  MD::MustikaParams p;
  add_real_golden(p);
  return p;
}

bool mask_has_golden(const cv::Mat& m) { return cv::countNonZero(m) > 0; }

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// ★ 金色 mask（**实配阈值的"重测"**）
// ═══════════════════════════════════════════════════════════════════════════

TEST(GoldenMaskTest, 金色区域被检出来)
{
  auto p = base_params();
  cv::Mat mask;
  MD::extract_golden_mask(solid(35, 200, 220), mask, p);   // H=35 落在 [20,50] 里
  EXPECT_TRUE(mask_has_golden(mask)) << "**实配阈值检不出金色 ⇒ 检测器永远找不到球**";
  EXPECT_EQ(mask.type(), CV_8UC1);
  EXPECT_EQ(mask.size(), cv::Size(200, 200));
}

TEST(GoldenMaskTest, 暗光补偿那组也生效)
{
  // 第 2 组阈值覆盖 H 18~55 / S 50~255 / V 80~230 ⇒ H=19, V=100 只被第 2 组覆盖
  auto p = base_params();
  cv::Mat mask;
  MD::extract_golden_mask(solid(19, 100, 100), mask, p);
  EXPECT_TRUE(mask_has_golden(mask)) << "暗光补偿那组没生效 ⇒ 暗处找不到球";
}

TEST(GoldenMaskTest, 蓝色不误判)
{
  // ⚠️ 实物是"黄底 + **蓝纹** + 白字" ⇒ 蓝纹**不能**被算成金色，
  //    否则球面上的蓝纹会把 mask 打碎（而后面还要靠 mask 算圆形度）
  auto p = base_params();
  cv::Mat mask;
  MD::extract_golden_mask(solid(110, 200, 220), mask, p);
  EXPECT_FALSE(mask_has_golden(mask)) << "蓝色被当成金色 ⇒ 蓝纹会污染 mask";
}

TEST(GoldenMaskTest, 白色和灰色不误判)
{
  auto p = base_params();
  for (int v : {255, 128}) {
    cv::Mat mask;
    MD::extract_golden_mask(solid(0, 0, v), mask, p);      // S=0 ⇒ 白/灰
    EXPECT_FALSE(mask_has_golden(mask)) << "S=0/V=" << v << " 不该是金色";
  }
}

TEST(GoldenMaskTest, 空图或非三通道出全零mask且不抛)
{
  // ⚠️ 上游给空图/灰度图时**抛异常会把整条相机链拖垮**，
  //    而"这一帧没有金色"本身是完全正常的结果。
  MD::MustikaParams p;                     // 空阈值组也行
  cv::Mat mask;
  cv::Mat empty;
  EXPECT_NO_THROW(MD::extract_golden_mask(empty, mask, p));
  EXPECT_EQ(cv::countNonZero(mask), 0);

  cv::Mat gray(10, 10, CV_8UC1, cv::Scalar(128));
  EXPECT_NO_THROW(MD::extract_golden_mask(gray, mask, p));
  EXPECT_EQ(cv::countNonZero(mask), 0);
}

TEST(GoldenMaskTest, 阈值组为空时出全零mask)
{
  MD::MustikaParams p;                     // golden_groups 空
  cv::Mat mask;
  MD::extract_golden_mask(solid(35, 200, 220), mask, p);
  EXPECT_EQ(cv::countNonZero(mask), 0) << "没有阈值组 ⇒ 什么都不该被选中";
}

TEST(GoldenMaskTest, 闭运算能填补内部小洞)
{
  // ⚠️ 闭运算的作用就是"填补球体表面暗斑"。造一个**中心有暗点**的金球：
  //    不跑闭运算时中心是洞，跑了之后被填上。
  auto p = base_params();
  cv::Mat hsv(100, 100, CV_8UC3, cv::Scalar(35, 200, 220));
  cv::circle(hsv, cv::Point(50, 50), 3, cv::Scalar(35, 200, 100), cv::FILLED);  // 暗斑
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

  cv::Mat with_close, without_close;
  MD::extract_golden_mask(bgr, with_close, p);
  auto p2 = p; p2.morph_close_iters = 0;
  MD::extract_golden_mask(bgr, without_close, p2);
  EXPECT_GE(cv::countNonZero(with_close), cv::countNonZero(without_close))
      << "闭运算不该让金色变少（它只填洞）";
}

// ═══════════════════════════════════════════════════════════════════════════
// ★★ validate_circle
// ═══════════════════════════════════════════════════════════════════════════

TEST(ValidateCircleTest, 合格的金球通过并填结果)
{
  auto p = base_params();
  const int cx = 150, cy = 150, r = 40;
  cv::Mat bgr = golden_disc(300, 300, cx, cy, r);
  cv::Mat mask;
  MD::extract_golden_mask(bgr, mask, p);

  MustikaImageResult res;
  ASSERT_TRUE(MD::validate_circle(bgr, mask, cv::Vec3f(cx, cy, r), res, p));
  EXPECT_TRUE(res.detected);
  EXPECT_FLOAT_EQ(res.u, static_cast<float>(cx));
  EXPECT_FLOAT_EQ(res.v, static_cast<float>(cy));
  EXPECT_FLOAT_EQ(res.radius, static_cast<float>(r));
  EXPECT_GT(res.confidence, 0.0f);
  EXPECT_LE(res.confidence, 1.0f);
  EXPECT_GT(res.golden_ratio, p.min_golden_ratio);
}

TEST(ValidateCircleTest, 失败时一个字段都不碰)
{
  // ⚠️ **头文件写死的契约**："仅成功时填入。任何一条判据不过就 return false 早退，
  //    result **一个字段都不碰**（保持调用前的值）⇒ 调用方**必须以返回值为准**"。
  //    ⚠️ 若失败时也写字段，调用方看 `result.detected` 就会**读到一个半成品**。
  auto p = base_params();
  cv::Mat bgr = solid(110, 200, 220, 300, 300);      // 全蓝 ⇒ 没有金色
  cv::Mat mask;
  MD::extract_golden_mask(bgr, mask, p);

  MustikaImageResult res;
  res.detected = false;
  res.u = -1.0f; res.v = -2.0f; res.radius = -3.0f; res.confidence = -4.0f;
  EXPECT_FALSE(MD::validate_circle(bgr, mask, cv::Vec3f(150, 150, 40), res, p));
  EXPECT_FALSE(res.detected);
  EXPECT_FLOAT_EQ(res.u, -1.0f) << "失败时不该碰 result";
  EXPECT_FLOAT_EQ(res.v, -2.0f);
  EXPECT_FLOAT_EQ(res.radius, -3.0f);
  EXPECT_FLOAT_EQ(res.confidence, -4.0f);
}

TEST(ValidateCircleTest, 半径判据是两套来源取交集)
{
  // ⚠️⚠️ **本文件最贵的一条**（头文件早就标注了这个陷阱，但没有测试钉住它）：
  // ```
  //   eff_min = max(hough_min_radius, r_min_by_dist / 2)
  //   eff_max = min(hough_max_radius, r_max_by_dist * 2)
  // ```
  //    默认参数下：`focal=600`、`real_r=0.105`、`min_dist=0.2`、`max_dist=3.0`
  //      ⇒ r_min_by_dist = 600×0.105/3.0 = **21**
  //      ⇒ r_max_by_dist = 600×0.105/0.2 = **315**
  //      ⇒ eff_max = min(**150**, 315×2=630) = **150**  ← **`hough_max_radius` 才是那个约束！**
  //    ⇒ `hough_max_radius` 配小，会**把按距离算对的上界夹回去**，
  //      而症状是"**明明在可见距离内的球检不出来**" —— **不报错**。
  //    ⇒ 本用例把这条"交集"关系**量出来**：同一个 r=100 的球，
  //      在 `hough_max_radius=150` 下通过、在 `=50` 下被拒 —— 尽管它离图像边界很远、
  //      金色占比也够（即**它本来该被判"能看见"**）。
  const int cx = 200, cy = 200, r = 100;
  cv::Mat bgr = golden_disc(400, 400, cx, cy, r);

  auto wide = base_params();                 // hough_max_radius = 150
  wide.hough_max_radius = 150;
  cv::Mat mask_w;
  MD::extract_golden_mask(bgr, mask_w, wide);
  MustikaImageResult r1;
  EXPECT_TRUE(MD::validate_circle(bgr, mask_w, cv::Vec3f(cx, cy, r), r1, wide))
      << "r=100 该在 [10,150] 里";

  auto narrow = wide;
  narrow.hough_max_radius = 50;              // ⚠️ 把上界夹到 50
  MustikaImageResult r2;
  EXPECT_FALSE(MD::validate_circle(bgr, mask_w, cv::Vec3f(cx, cy, r), r2, narrow))
      << "hough_max_radius 夹小了 ⇒ 这个球被拒 —— 而它本来在可见距离内（症状：漏检且不报错）";
}

TEST(ValidateCircleTest, 超出图像边界被拒)
{
  auto p = base_params();
  cv::Mat bgr = golden_disc(300, 300, 150, 150, 40);
  cv::Mat mask;
  MD::extract_golden_mask(bgr, mask, p);
  MustikaImageResult res;
  // 圆心贴着左上角 ⇒ 圆盘出界
  EXPECT_FALSE(MD::validate_circle(bgr, mask, cv::Vec3f(5, 5, 40), res, p));
}

TEST(ValidateCircleTest, 金色占比不足被拒)
{
  // ⚠️ 这条钉的是**根因判据**：`min_golden_ratio`（2026-09-20 由 0.60 改到 **0.50**）
  auto p = base_params();
  cv::Mat bgr = solid(110, 200, 220, 300, 300);        // 全蓝 ⇒ 占比 0
  cv::Mat mask;
  MD::extract_golden_mask(bgr, mask, p);
  MustikaImageResult res;
  EXPECT_FALSE(MD::validate_circle(bgr, mask, cv::Vec3f(150, 150, 40), res, p));

  // 把阈值放到 0 ⇒ 占比 0 也能过（证明上面那条拒的是**占比**，不是别的）
  auto loose = p;
  loose.min_golden_ratio = 0.0;
  loose.min_circularity = 0.0;                        // 圆形度也放开，只留半径判据
  MustikaImageResult res2;
  EXPECT_TRUE(MD::validate_circle(bgr, mask, cv::Vec3f(150, 150, 40), res2, loose))
      << "放开占比与圆形度后就该过 —— 说明前面拒的确实是占比";
}

TEST(ValidateCircleTest, 占比阈值真的在起作用)
{
  // ⚠️⚠️ **构造要点**：必须让"候选圆内的金色占比"**明显小于 1**，
  //    否则抬阈值也拒不掉它 —— 我第一版就是拿**实心金盘**去测，
  //    它的占比 ≈ 1.0，把阈值抬到 0.99 照样通过（**用例红了而原因是构造错**）。
  //    ⇒ 这里让**金盘 (r=40) 落在候选圆 (r=60) 里面** ⇒ 占比 ≈ (40/60)² ≈ 0.44。
  const int cx = 150, cy = 150, disc_r = 40, cand_r = 60;
  cv::Mat bgr = golden_disc(300, 300, cx, cy, disc_r);
  auto p = base_params();
  cv::Mat mask;
  MD::extract_golden_mask(bgr, mask, p);

  // 现值 0.50 > 0.44 ⇒ **本来就该拒**
  MustikaImageResult res;
  EXPECT_FALSE(MD::validate_circle(bgr, mask, cv::Vec3f(cx, cy, cand_r), res, p))
      << "占比 0.44 < 阈值 0.50，该拒";

  // 把阈值放到 0.40（低于 0.44）⇒ 同一个候选**该过**
  auto loose = p;
  loose.min_golden_ratio = 0.40;
  MustikaImageResult ok;
  EXPECT_TRUE(MD::validate_circle(bgr, mask, cv::Vec3f(cx, cy, cand_r), ok, loose))
      << "阈值放到 0.40 后就该过 —— 说明 `min_golden_ratio` 真的在起作用"
         "（若哪天它被硬编码，这条会红）";
  EXPECT_NEAR(ok.golden_ratio, 0.44f, 0.1f) << "占比该在 (40/60)² 附近";
}

}  // namespace camera
}  // namespace br_perception
