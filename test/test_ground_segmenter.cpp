/// ═══════════════════════════════════════════════════════════════════════════════
/// test_ground_segmenter.cpp — 模块 12 地面分割：**纯函数**单元测试
///
/// ⚠️ 只测 **static 纯函数**（`validate_plane_normal`）—— 不起节点、不喂点云。
///    ⚠️ **别把本文件的绿当成"地面分割能跑"**：RANSAC 拟合、自适应精筛、发布都不在这里。
///
/// ── 为什么这个函数值得单测（它只有 5 行）──
///   它是**地面分割的第一道闸**：法向量不像地面就整帧不认（地面点全空）。
///   而它有三处**都会静默出错**的地方：
///     · **不取 `|c|`** ⇒ RANSAC 系数朝向随机 ⇒ **一半的地面被否掉**，
///       且只在"这次 RANSAC 恰好朝下"时复现 —— 看起来像随机故障；
///     · **零法向量不早退** ⇒ `acos(0/0)` = NaN ⇒ 侥幸结果一样，
///       但 NaN 一旦流出去会**污染上游的一切比较**；
///     · **`values` 长度不足** ⇒ 裸 `vector` 的越界读。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=PlaneNormalTest.*`）。
/// ⚠️ 全部用例确定性：手造系数、无点云、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/lidar/ground_segmenter.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace br_perception {
namespace lidar {

using GS = GroundSegmenter;

namespace {

constexpr double kPi = 3.14159265358979323846;

pcl::ModelCoefficients coeff(double a, double b, double c, double d = 0.0)
{
  pcl::ModelCoefficients m;
  m.values = {static_cast<float>(a), static_cast<float>(b),
              static_cast<float>(c), static_cast<float>(d)};
  return m;
}

}  // namespace

TEST(PlaneNormalTest, 水平面是地面)
{
  // 法向量 (0,0,1) ⇒ 与 Z 轴夹角 0 ⇒ 是地面
  EXPECT_TRUE(GS::validate_plane_normal(coeff(0, 0, 1), 0.1));
}

TEST(PlaneNormalTest, 法向量朝下也算地面)
{
  // ⚠️⚠️ **这条钉的是"取 |c|"**：RANSAC 给的系数**朝向是任意的**，
  //    `(0,0,-1)` 与 `(0,0,1)` 描述的是**同一个平面**。
  //    ⚠️ 不取绝对值 ⇒ **一半的地面会被否掉**，而且只在"这次朝下"时复现
  //    ⇒ 现场看起来像**随机故障**，最难查。
  EXPECT_TRUE(GS::validate_plane_normal(coeff(0, 0, -1), 0.1))
      << "法向量朝向不该影响结论 —— 不取 |c| 会让一半的地面消失";
}

TEST(PlaneNormalTest, 垂直面不是地面)
{
  // 法向量 (1,0,0) ⇒ 与 Z 轴夹角 90° ⇒ 墙，不是地面
  EXPECT_FALSE(GS::validate_plane_normal(coeff(1, 0, 0), 0.1));
  EXPECT_FALSE(GS::validate_plane_normal(coeff(0, 1, 0), 0.1));
}

TEST(PlaneNormalTest, 倾斜超过阈值就不是)
{
  // 与 Z 轴夹角 30° = 0.5236 rad；阈值 0.1 rad ⇒ 否
  const double a = 30.0 * kPi / 180.0;
  EXPECT_FALSE(GS::validate_plane_normal(coeff(std::sin(a), 0, std::cos(a)), 0.1));
  // 同一平面，阈值放宽到 0.6 rad ⇒ 通过
  EXPECT_TRUE(GS::validate_plane_normal(coeff(std::sin(a), 0, std::cos(a)), 0.6));
}

TEST(PlaneNormalTest, 边界恰好等于阈值不算地面)
{
  // ⚠️ 严格 `<`。用**可精确构造**的角度：法向量 (0,1,1)/√2 与 Z 轴夹角正好 45° = π/4
  const double angle = kPi / 4.0;
  // 阈值恰好等于 π/4 ⇒ **不通过**（严格 <）
  EXPECT_FALSE(GS::validate_plane_normal(coeff(0, 1, 1), angle));
  // 略大一点 ⇒ 通过
  EXPECT_TRUE(GS::validate_plane_normal(coeff(0, 1, 1), angle + 1e-9));
}

TEST(PlaneNormalTest, 未归一化的法向量也算得对)
{
  // ⚠️ 判据里有 `norm` 归一化 ⇒ `(0,0,5)` 与 `(0,0,1)` 同判。
  //    漏了归一化的话，阈值语义会**随法向量长度漂**。
  EXPECT_TRUE(GS::validate_plane_normal(coeff(0, 0, 5), 0.1));
  EXPECT_TRUE(GS::validate_plane_normal(coeff(0, 0, 0.001), 0.1));
  const double a = 30.0 * kPi / 180.0;
  EXPECT_FALSE(GS::validate_plane_normal(coeff(100 * std::sin(a), 0, 100 * std::cos(a)), 0.1));
}

TEST(PlaneNormalTest, 零法向量返回false而不是NaN)
{
  // ⚠️ `acos(0/0)` = NaN，而 `NaN < threshold` **恒为 false** ⇒ 侥幸结果一样。
  //    但**一旦 NaN 从这里流出去就会污染上游的一切比较** —— 必须在这里挡掉。
  EXPECT_FALSE(GS::validate_plane_normal(coeff(0, 0, 0), 0.1));
  EXPECT_FALSE(GS::validate_plane_normal(coeff(0, 0, 0), 10.0))
      << "阈值再大也不该让零法向量通过";
}

TEST(PlaneNormalTest, 系数个数不足时不越界读)
{
  // ⚠️ `ModelCoefficients::values` 是个**裸 vector<float>**，长度不保证是 4。
  //    不检查就索引 = **越界读（UB）** —— 而这种 UB 常常"看起来正常"，
  //    直到某天内存布局变了才炸。
  pcl::ModelCoefficients too_short;
  too_short.values = {0.0f, 0.0f};              // 只有 2 个
  EXPECT_FALSE(GS::validate_plane_normal(too_short, 0.1));

  pcl::ModelCoefficients empty;
  EXPECT_FALSE(GS::validate_plane_normal(empty, 0.1));
}

TEST(PlaneNormalTest, 阈值必须为正才可能通过)
{
  // 夹角恒 >= 0 ⇒ 阈值 <= 0 时**没有任何平面能通过**（严格 <）。
  // ⚠️ 这不是 bug，是语义；钉住它免得有人把"阈值 0"当成"不设限"。
  EXPECT_FALSE(GS::validate_plane_normal(coeff(0, 0, 1), 0.0));
  EXPECT_FALSE(GS::validate_plane_normal(coeff(0, 0, 1), -0.1));
  EXPECT_TRUE(GS::validate_plane_normal(coeff(0, 0, 1), 1e-6));
}

}  // namespace lidar
}  // namespace br_perception
