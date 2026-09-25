/// ═══════════════════════════════════════════════════════════════════════════════
/// test_building_spot_detector.cpp — 模块 6 建筑位检测：**四边形几何验证**单测
///
/// ⚠️ 只测 `validate_angles` 与 `validate_opposite_sides`（都是 **static 纯函数**）——
///    **不起节点、不喂图、不跑 findContours**。
///    ⚠️ **别把本文件的绿当成"建筑位检测能跑"**：绿 mask、轮廓、凸度、填充率、发布都不在这里。
///
/// ── 为什么这两条"几行几何"值得单测 ──
///   它们判的是"检出的四边形**是不是一个矩形**"，即"这团绿色**是不是一个建筑位**"。
///   而**建筑位是那 40/80 分判色的投影锚点**（位坐标 → 图像 ROI）⇒ 判错了：
///     · 太松 ⇒ 把绿色杂物当建筑位（**多报位**）
///     · 太紧 ⇒ 真建筑位在斜视角下被拒（**漏报位**）
///   ⇒ 本文件里有一条专门**量出这套判据有多松**（见 `对边判据偏松`）。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=QuadAnglesTest.*`）。
/// ⚠️ 全部用例确定性：手造四边形（整数坐标）、无图、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/building_spot_detector.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace br_perception {
namespace camera {

using BS = BuildingSpotDetector;
using Pts = std::vector<cv::Point>;

namespace {

/// 一个正方形（**整数坐标 ⇒ 角度与长度全是精确值**）
Pts square(int side = 10, int x0 = 0, int y0 = 0)
{
  return {{x0, y0}, {x0 + side, y0}, {x0 + side, y0 + side}, {x0, y0 + side}};
}

/// 一个"每角都偏 `dev` 度"的菱形（把正方形的顶点拉到偏离 90°）
/// ⚠️ 用整数近似：顶点取 (0,0),(s,0),(s+d,d),(d,d) 这种平行四边形
Pts parallelogram(int sx, int sy, double shear)
{
  const int dx = static_cast<int>(sy * shear);
  return {{0, 0}, {sx, 0}, {sx + dx, sy}, {dx, sy}};
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// validate_angles
// ═══════════════════════════════════════════════════════════════════════════

TEST(QuadAnglesTest, 正方形的内角评分是满分)
{
  float score = 0.0f;
  EXPECT_TRUE(BS::validate_angles(square(10), score, 0.80));
  EXPECT_NEAR(score, 1.0f, 1e-4) << "四角都是精确 90° ⇒ 评分 1.0";
}

TEST(QuadAnglesTest, 一个角偏离过大直接拒不看评分)
{
  // ⚠️ **两层判据**：任一角偏离 90° 超过 **30°** ⇒ **硬拒**（直接 return false），
  //    哪怕另外三个角都是完美的直角（均值可能还挺高）。
  //    取 (0,0),(10,0),(10,10),(1,3)：最后一个点被拉进来 ⇒ 某个角明显不是 90°。
  const Pts quad{{0, 0}, {10, 0}, {10, 10}, {1, 3}};
  float score = 0.0f;
  EXPECT_FALSE(BS::validate_angles(quad, score, 0.0))
      << "阈值给 0 也该拒 —— 因为硬判据（>30°）在评分之前";
}

TEST(QuadAnglesTest, 评分阈值是非严格的大于等于)
{
  // ⚠️ `return (angle_score >= min_angle_score)` ⇒ 恰好等于**通过**。
  float score = 0.0f;
  ASSERT_TRUE(BS::validate_angles(square(10), score, 0.0));
  const float exact = score;                       // 正方形的精确评分
  EXPECT_TRUE(BS::validate_angles(square(10), score, exact))
      << "恰好等于阈值该通过（判据是非严格 >=）";
  EXPECT_FALSE(BS::validate_angles(square(10), score, exact + 1e-3))
      << "略高于评分就该拒";
}

TEST(QuadAnglesTest, 顶点重合的退化四边形被拒)
{
  // ⚠️ 某条边长为 0 ⇒ 实现里 `mag < 1e-6` 直接 return false。
  //    不防的话 `dot/(mag1*mag2)` 就是**除零** ⇒ NaN ⇒ 比较全为 false（**侥幸一样，但 NaN 会流出去**）。
  const Pts degenerate{{0, 0}, {0, 0}, {10, 10}, {0, 10}};
  float score = 0.0f;
  EXPECT_FALSE(BS::validate_angles(degenerate, score, 0.0));
}

TEST(QuadAnglesTest, 平行四边形角度也接近直角)
{
  // ⚠️ 切变小的平行四边形，四个角仍然接近 90° ⇒ **角度判据拦不住它**
  //    （拦边长要靠 `validate_opposite_sides`）。这条把"两条判据分工"钉下来。
  const Pts shear = parallelogram(100, 100, 0.05);   // 切变 5%
  float score = 0.0f;
  EXPECT_TRUE(BS::validate_angles(shear, score, 0.80))
      << "小切变的平行四边形，角度判据拦不住 —— 它靠的是对边/凸度那几条";
}

// ═══════════════════════════════════════════════════════════════════════════
// validate_opposite_sides
// ═══════════════════════════════════════════════════════════════════════════

TEST(QuadSidesTest, 正方形的对边评分是满分)
{
  float score = 0.0f;
  EXPECT_TRUE(BS::validate_opposite_sides(square(10), score, 0.70));
  // 两组对边都完全平行（|dot| = 1）、长度全等（ratio = 1）
  // ⇒ 0.6×1 + 0.4×1 = 1.0
  EXPECT_NEAR(score, 1.0f, 1e-4);
}

TEST(QuadSidesTest, 长条矩形照样满分_本函数不查长宽比)
{
  // ⚠️⚠️ **这条是记录真实行为，不是断言它错** —— 我第一版想当然写成"长宽比 10:1 该被拒"，
  //    **用例红了而原因是我的假设错**：这里的"长度比"比的是**对边**（0↔1、2↔3），
  //    **不是相邻边** ⇒ 一个 100×10 的长条，两组对边各自等长 ⇒ `ratio = 1.0`
  //    ⇒ `side_score = 0.6×1.0 + 0.4×1.0 = 1.0` **满分通过**。
  //    ⇒ **本函数完全不查长宽比**。一条细长的绿色条带**靠它拦不住** ——
  //      真正拦它的是 `min_convexity_` 与绿色填充率那几条（以及轮廓本身的形状）。
  const Pts strip{{0, 0}, {100, 0}, {100, 10}, {0, 10}};
  float score = 0.0f;
  EXPECT_TRUE(BS::validate_opposite_sides(strip, score, 0.70))
      << "对边等长就等于满分 —— 本函数不查长宽比";
  EXPECT_NEAR(score, 1.0f, 1e-4);
}

TEST(QuadSidesTest, 顶点重合被拒)
{
  const Pts degenerate{{0, 0}, {0, 0}, {10, 10}, {0, 10}};
  float score = 0.0f;
  EXPECT_FALSE(BS::validate_opposite_sides(degenerate, score, 0.0))
      << "有零长边 ⇒ 直接拒（那条边的单位方向是 (0,0)）";
}

TEST(QuadSidesTest, 评分阈值是非严格的大于等于)
{
  float score = 0.0f;
  ASSERT_TRUE(BS::validate_opposite_sides(square(10), score, 0.0));
  const float exact = score;
  EXPECT_TRUE(BS::validate_opposite_sides(square(10), score, exact));
  EXPECT_FALSE(BS::validate_opposite_sides(square(10), score, exact + 1e-3));
}

TEST(QuadSidesTest, 对边判据偏松_明显不规则的四边形仍通过)
{
  // ⚠️⚠️ **这条是"量出这套判据有多松"**，不是断言它错 ——
  //    用一个**明显不规则**的四边形（顶点 (0,0),(10,0),(10,10),(5,8)），
  //    手算：平行度 ≈ (0.928 + 0.848)/2 = 0.888、长度比 ≈ (0.539 + 0.943)/2 = 0.741
  //    ⇒ side_score ≈ 0.6×0.888 + 0.4×0.741 = **0.829 > 0.70** ⇒ **通过**。
  //    ⚠️ 而且"平行度"用的是 `|dot|` ⇒ 对**等腰梯形**这类"两腰镜像"的形状
  //       会给出接近 1 的值（**它测的是"两腰都够陡"，不是"两腰平行"**）。
  //    ⇒ **别指望这一条单独拦住不规则的形状** —— 真正拦住它们的是
  //      `min_convexity_`(0.88) 与绿色填充率那几条。这条用例把这个分工写下来。
  const Pts irregular{{0, 0}, {10, 0}, {10, 10}, {5, 8}};
  float score = 0.0f;
  EXPECT_TRUE(BS::validate_opposite_sides(irregular, score, 0.70))
      << "这条判据偏松，别指望它单独拦住不规则形状";
  EXPECT_NEAR(score, 0.83f, 0.05f) << "手算 ≈ 0.829";
}

}  // namespace camera
}  // namespace br_perception
