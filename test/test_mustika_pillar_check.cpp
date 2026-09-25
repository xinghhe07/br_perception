/// ═══════════════════════════════════════════════════════════════════════════════
/// test_mustika_pillar_check.cpp — 模块 20「穆斯蒂卡在柱上吗」单元测试
///
/// ⚠️ **它值 250 分**（规则 4.5.1：TR 取五色石的前置是圣所使命达成，而"柱上有没有球"
///    是 §3.0.1 点名的得分动作）—— 而它**此前一个单测都没有**（台账里那句
///    "没有单测 ⇒ 到不了 🟢；跑起来算得对不对目前只有手工观察"）。
///
/// ⚠️ 本文件**只测两个 static 纯函数**（不起 ROS 节点、不用 TF）：
///    ① `range_from_pixel_radius` —— 已知球径 + 像素半径 ⇒ 反推距离
///    ② `back_project`           —— 像素 + 深度 ⇒ **光心系**三维点
///    这两个正好是两类**最贵的错**的所在：
///      · ① 错了 ⇒ 距离按比例错 ⇒ 与雷达柱顶的残差全部对不上 ⇒ **恒判"未知"**
///        （而它会让人反复去标外参 —— 上一轮"长焦焦距两处差 3 倍"就是这个症状）；
///      · ② 错了 ⇒ **整体绕 X 轴差 90°、且不报错**（2026-09-23 复核抓到过一次，
///        原实现把光心系的数直接填进机身系的 `PointStamped`）。
///        ⇒ 所以下面**专门有条用例钉"光心系"的符号约定**（右/下/前）。
///
/// ⚠️ 全部用例**确定性**，期望值**全部手算**（不是"跑出来是什么就写什么"）——
///    手算过的期望值才是判据；照抄输出的期望值只能防回归，防不了"一开始就错"。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=BallRangeTest.*`）。
///
/// ⚠️ 没测的部分（**如实写在这里，别让人以为测全了**）：`judge_one` 里
///    "光心系 → 机身系"的那三行内联转换（`x_body = opt.z` / `y_body = −opt.x` /
///    `z_body = −opt.y`）、TF 查询、`decide` 的三档状态机 —— 它们要么吃成员状态、
///    要么要真实 ROS 图，属集成测试。
///    ⚠️ 那条内联转换的**正确性靠跨模块一致性看着**：`sky_block_detector` 的
///       `body2opt = (−b.y, −b.z, +b.x)` 与这里的逆向**互逆**（手算复合回原值）——
///       两个模块用的是同一套约定。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/fusion/mustika_pillar_check.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>

namespace br_perception {
namespace fusion {
namespace {

// 真实感的内参（两路）—— 与仓库里的占位/仿真值同源，便于和现场数字对上：
//   · 宽角 fx = 521.9  （仿真 640×384、hfov=1.1 推出来的，见 bringup_debug 的 SIM_INTRINSICS）
//   · 长焦 fx = 1578.6 （仿真 hfov=0.40）
//   · 长焦 fx = 2380    （`config/camera_hardware.yaml` 的**实机占位值**）
constexpr double kFxWide      = 521.9;
constexpr double kFxTeleSim   = 1578.6;
constexpr double kFxTeleReal  = 2380.0;
constexpr double kBallDiaM    = 0.21;      // Mikasa V300W Ø≈210mm

std::optional<double> range(double r_px, double f_px, double D = kBallDiaM)
{
  return MustikaPillarCheck::range_from_pixel_radius(r_px, f_px, D);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// ① 像素半径 → 距离
// ═══════════════════════════════════════════════════════════════════════════════
class BallRangeTest : public ::testing::Test {};

TEST_F(BallRangeTest, 手算三个真实量级)
{
  // r ≈ f·(D/2)/d  ⇒  d = f·D/(2r)
  //   长焦实机 f=2380、r=100px：2380×0.21/(2×100) = 2.499 m
  ASSERT_TRUE(range(100.0, kFxTeleReal).has_value());
  EXPECT_NEAR(*range(100.0, kFxTeleReal), 2.499, 1e-3);

  //   长焦仿真 f=1578.6、r=100px：1578.6×0.21/200 = 1.6575 m
  EXPECT_NEAR(*range(100.0, kFxTeleSim), 1.65753, 1e-4);

  //   宽角 f=521.9、r=25px：521.9×0.21/50 = 2.192 m
  EXPECT_NEAR(*range(25.0, kFxWide), 2.19198, 1e-4);
}

TEST_F(BallRangeTest, 像素半径翻倍则距离减半)
{
  // ⚠️ 性质用例（不是某个具体的数）：它抓的是"公式被写成平方/线性之外的东西"。
  //    真值：r ∝ 1/d ⇒ r×2 ⇒ d÷2。
  const double d1 = *range(40.0, kFxTeleSim);
  const double d2 = *range(80.0, kFxTeleSim);
  EXPECT_NEAR(d1, d2 * 2.0, 1e-9);
}

TEST_F(BallRangeTest, 球径变化按比例影响距离)
{
  // 球径 210mm → 420mm（换了个更大的球）在同像素半径下距离**加倍**
  EXPECT_NEAR(*range(50.0, kFxTeleSim, 0.42), *range(50.0, kFxTeleSim, 0.21) * 2.0, 1e-9);
}

TEST_F(BallRangeTest, 非法输入一律返回空而不是猜一个数)
{
  // ⚠️ 三条都是"算不出来"而不是"算出 0/负/NaN" —— 后者会一路传下去变成假结论。
  EXPECT_FALSE(range(0.0,  kFxTeleSim).has_value());          // 半径为 0
  EXPECT_FALSE(range(-1.0, kFxTeleSim).has_value());          // 负半径
  EXPECT_FALSE(range(100.0, 0.0).has_value());                // 焦距为 0（内参没标定）
  EXPECT_FALSE(range(100.0, kFxTeleSim, 0.0).has_value());    // 球径为 0（配置缺失）
  EXPECT_FALSE(range(100.0, -kFxTeleSim, kBallDiaM).has_value());
}

TEST_F(BallRangeTest, 极小像素半径给出很大但有限的距离)
{
  // 远处一个小球：半径 1px、f=2380 ⇒ d = 2380×0.21/2 = 249.9 m
  //   ⚠️ 这里**有意不设上限**：函数只管换算，"太远以致不可信"是调用方
  //      （`min_radius_px_ = 8`）该拦的事 —— 职责分开，各自有用例。
  const auto d = range(1.0, kFxTeleReal);
  ASSERT_TRUE(d.has_value());
  EXPECT_NEAR(*d, 249.9, 1e-2);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ② 像素 + 深度 → **光心系**三维点
// ═══════════════════════════════════════════════════════════════════════════════
class BackProjectTest : public ::testing::Test
{
protected:
  // 内参（拿仿真的宽角）
  const double fx = kFxWide, fy = kFxWide, cx = 320.0, cy = 192.0;

  void run(double u, double v, double depth, double* xyz)
  {
    MustikaPillarCheck::back_project(u, v, depth, fx, fy, cx, cy, xyz);
  }
};

TEST_F(BackProjectTest, 主点落在光轴上)
{
  double xyz[3] = {9.0, 9.0, 9.0};
  run(cx, cy, 3.0, xyz);
  EXPECT_NEAR(xyz[0], 0.0, 1e-12);
  EXPECT_NEAR(xyz[1], 0.0, 1e-12);
  EXPECT_NEAR(xyz[2], 3.0, 1e-12);      // ⚠️ 光心系 +Z 就是**深度**（朝前）
}

TEST_F(BackProjectTest, 符号约定_右是加X_下是加Y_前是加Z光心系)
{
  // ⚠️⚠️ **这条用例是给"静默错 90°"立的碑**（2026-09-23 复核抓到过一次）：
  //    本函数返回的是**光心系**（REP-103 的光学约定：+X 右 / +Y 下 / +Z 前），
  //    **不是**本节点用来查 TF 的机身系（+X 前 / +Y 左 / +Z 上）。
  //    ⇒ 谁把这里"顺手改成机身系"（或反过来把结果当机身系用），这两条断言会红。
  double xyz[3];
  run(cx + fx, cy, 2.0, xyz);            // 主点右侧一个焦距
  EXPECT_NEAR(xyz[0], 2.0, 1e-12);       // +X = 右
  EXPECT_NEAR(xyz[1], 0.0, 1e-12);
  EXPECT_NEAR(xyz[2], 2.0, 1e-12);

  run(cx, cy + fy, 2.0, xyz);            // 主点下方一个焦距
  EXPECT_NEAR(xyz[0], 0.0, 1e-12);
  EXPECT_NEAR(xyz[1], 2.0, 1e-12);       // +Y = 下
  EXPECT_NEAR(xyz[2], 2.0, 1e-12);
}

TEST_F(BackProjectTest, 与球的距离公式自洽)
{
  // ⚠️ **两个纯函数之间的交叉验证**：把一个直径 0.21m 的球放在正前方 d 处，
  //    它在图像里的像素半径应是 r = f·D/(2d)；把 (cx+r, cy, d) 反投影回去，
  //    得到的点与光轴的距离必须正好是**球半径 0.105m**。
  //    ⇒ 这条把 ① 和 ② 绑在一起：任何一个改了公式而另一个没跟上，它会红。
  for (double d : {0.5, 1.0, 2.5, 5.0}) {
    const double r_px = kFxWide * kBallDiaM / (2.0 * d);   // 正向：球在这个距离下的像素半径
    double xyz[3];
    run(cx + r_px, cy, d, xyz);
    const double radial = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1]);
    EXPECT_NEAR(radial, kBallDiaM / 2.0, 1e-9) << "d=" << d;
  }
}

TEST_F(BackProjectTest, 深度是0或负数时给出的是数学结果而非崩溃)
{
  // ⚠️ 本函数**不做参数校验**（约定：调用方保证 depth > 0 —— `judge_one` 只在
  //    反推成功后才调它）。这条用例**只钉"不崩、不 UB"**，不假装它会报错。
  double xyz[3];
  run(cx + 10.0, cy, 0.0, xyz);
  EXPECT_DOUBLE_EQ(xyz[2], 0.0);
  run(cx + 10.0, cy, -1.0, xyz);
  EXPECT_DOUBLE_EQ(xyz[2], -1.0);
}

}  // namespace fusion
}  // namespace br_perception
