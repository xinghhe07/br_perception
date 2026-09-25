/// ═══════════════════════════════════════════════════════════════════════════════
/// test_image_preprocessor.cpp — 模块 4 图像预处理：**letterbox 反算**单元测试
///
/// ⚠️ 只测 **`LetterboxTransform` 与两个反算自由函数**（都是 header 里的 inline 纯函数）
///    —— **不起节点、不喂图、不跑 OpenCV**。
///    ⚠️ **别把本文件的绿当成"预处理器能跑"**：CLAHE/normalize/ROI/发布都不在这里。
///
/// ── 为什么这套反算值得单测（P0-3 就是为它立的项）──
///   训练用 **letterbox**（等比 + 补边），所以**检测框坐标是"缩放并补边后"的**，
///   必须反算回原图 —— 而**下游每一个检测器都要用它**（球心、半径、方块框、柱子）。
///   ⇒ 算错了，**每一个检测框的坐标都是错的**，且在"原图与输入同尺寸"时**完全看不出来**。
///
/// ⚠️⚠️ **本文件最贵的一条是 `尺度类反算不能减填充偏移`** —— 见那条用例的说明。
///    头文件原话：「这是整套反算里**最易错的一处**：把 width/height 当位置一起减偏移，
///    在小目标上会**直接出负宽度**。」
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=LetterboxTest.*`）。
/// ⚠️ 全部用例确定性：纯算术，无图、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/image_preprocessor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace br_perception {
namespace camera {

using LB = LetterboxTransform;

namespace {

/// 造一个变换（**照真实的 letterbox 算法**：等比缩放 + 居中补边）
LB make(double orig_w, double orig_h, double out_w, double out_h)
{
  LB t;
  t.orig_w = static_cast<int>(orig_w);
  t.orig_h = static_cast<int>(orig_h);
  t.out_w  = static_cast<int>(out_w);
  t.out_h  = static_cast<int>(out_h);
  t.scale  = std::min(out_w / orig_w, out_h / orig_h);
  const int cw = static_cast<int>(orig_w * t.scale + 0.5);
  const int ch = static_cast<int>(orig_h * t.scale + 0.5);
  t.pad_x = static_cast<int>((out_w - cw) / 2.0);
  t.pad_y = static_cast<int>((out_h - ch) / 2.0);
  return t;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 真实分辨率（**本仓库的相机就是这个组合**）
// ═══════════════════════════════════════════════════════════════════════════

TEST(LetterboxTest, 1280x720到640x384的几何)
{
  // ⚠️ 这是**相机层实际在用的组合**（`camera_hardware.yaml` 1280×720，
  //    而 `camera_params.yaml` 的 model_input 是 640×384）。
  //    ⇒ 用真数字手算一遍，而不是造个虚构的尺寸。
  const auto t = make(1280, 720, 640, 384);
  EXPECT_DOUBLE_EQ(t.scale, 0.5);          // min(640/1280=0.5, 384/720≈0.5333) = 0.5
  EXPECT_EQ(t.content_w(), 640);           // 1280×0.5
  EXPECT_EQ(t.content_h(), 360);           // 720×0.5
  EXPECT_EQ(t.pad_x, 0);                   // (640-640)/2
  EXPECT_EQ(t.pad_y, 12);                  // (384-360)/2  ← **上下各补 12px 的灰边**
}

TEST(LetterboxTest, 补边灰值必须与训练一致)
{
  // ⚠️ Ultralytics 训练时用 **114 灰**补边，不是 0 黑。
  //    换成 0 会让推理看到的图与训练分布不一致 —— **不报错，只是精度掉**。
  EXPECT_EQ(kLetterboxPadValue, 114);
}

// ═══════════════════════════════════════════════════════════════════════════
// 往返自洽
// ═══════════════════════════════════════════════════════════════════════════

TEST(LetterboxTest, 位置往返自洽)
{
  const auto t = make(1280, 720, 640, 384);
  for (double x : {0.0, 100.0, 640.0, 1279.0}) {
    for (double y : {0.0, 200.0, 360.0, 719.0}) {
      const double bx = t.to_box_x(x), by = t.to_box_y(y);
      EXPECT_NEAR(t.to_orig_x(bx), x, 1e-9);
      EXPECT_NEAR(t.to_orig_y(by), y, 1e-9);
    }
  }
}

TEST(LetterboxTest, 内容区四角映射正确)
{
  const auto t = make(1280, 720, 640, 384);
  // 原图左上角 ⇒ letterbox 的 (pad_x, pad_y)
  EXPECT_DOUBLE_EQ(t.to_box_x(0.0), static_cast<double>(t.pad_x));
  EXPECT_DOUBLE_EQ(t.to_box_y(0.0), static_cast<double>(t.pad_y));
  // 原图右下角 ⇒ letterbox 的内容区右下角
  EXPECT_NEAR(t.to_box_x(1280.0), t.content_w() + t.pad_x, 1e-9);
  EXPECT_NEAR(t.to_box_y(720.0),  t.content_h() + t.pad_y, 1e-9);
}

// ═══════════════════════════════════════════════════════════════════════════
// ★★ 位置 vs 尺度（**头文件点名的「最易错的一处」**）
// ═══════════════════════════════════════════════════════════════════════════

TEST(LetterboxTest, 位置类反算要减偏移再除scale)
{
  double x = 100.0f, y = 212.0f;          // letterbox 坐标
  letterbox_to_orig_xy(0.5f, 0, 12, x, y);
  EXPECT_NEAR(x, 200.0, 1e-6);           // (100-0)/0.5
  EXPECT_NEAR(y, 400.0, 1e-6);           // (212-12)/0.5  ← **减掉上边那 12px**
}

TEST(LetterboxTest, 尺度类反算只除scale不减偏移)
{
  // ⚠️ 这是为**半径/宽/高**准备的：它们**没有"位置"**，减偏移就是错的。
  double w = 40.0, h = 40.0;             // letterbox 里的直径
  letterbox_to_orig_size(0.5, w, h);
  EXPECT_NEAR(w, 80.0, 1e-9);            // 只 /0.5
  EXPECT_NEAR(h, 80.0, 1e-9);
}

TEST(LetterboxTest, 把尺度当位置反算会出负尺寸)
{
  // ⚠️⚠️ **本文件最贵的一条**（头文件原话："这是整套反算里最易错的一处"）。
  //    若把 width/height 拿去走 `letterbox_to_orig_xy`（**减了 pad**），
  //    在**小目标**上会直接出**负宽度** —— 而症状是：
  //      · 圆形度算不出来（半径负）
  //      · 面积过滤把真目标当噪点滤掉
  //    **不报错**，只是"某些小目标检不出来"。
  //    本例把那个负值**量出来**，证明"两条路不能混用"。
  const std::int32_t pad_y = 12;
  // ⚠️ 把"高度"塞进 **y 槽**（签名是 `(scale, pad_x, pad_y, x, y)`）——
  //    第一版我塞进了 x 槽而 pad_x 给的 0，于是算出 +16 而不是负数（**用例红了而原因是传参错**）。
  double dummy = 100.0;
  double h_wrong = 8.0;                   // 小目标：letterbox 里只有 8px 高
  letterbox_to_orig_xy(0.5f, 0, pad_y, dummy, h_wrong);   // ⚠️ 错误用法（把尺寸当位置）
  EXPECT_LT(h_wrong, 0.0) << "这就是那条陷阱的具体形状：小目标的尺寸反算成负数";

  // 正确用法
  double w2 = 8.0, h2 = 8.0;
  letterbox_to_orig_size(0.5, w2, h2);
  EXPECT_GT(w2, 0.0) << "正确用法永远是正的（尺寸 = 长度，不为负）";
  EXPECT_NEAR(w2, 16.0, 1e-9);
}

// ═══════════════════════════════════════════════════════════════════════════
// 边界：scale 为 0 / 负
// ═══════════════════════════════════════════════════════════════════════════

TEST(LetterboxTest, scale为零时当1处理不出NaN或inf)
{
  // ⚠️ 实现里有 `(scale > 1e-6f) ? scale : 1.0` 的防护 —— 除零会出 inf/NaN，
  //    而 NaN 一旦进了检测框坐标就会**污染整条下游**（且比较全为 false）。
  double x = 100.0f, y = 100.0f;
  letterbox_to_orig_xy(0.0f, 0, 0, x, y);
  EXPECT_TRUE(std::isfinite(x));
  EXPECT_TRUE(std::isfinite(y));
  EXPECT_DOUBLE_EQ(x, 100.0) << "scale 当 1 ⇒ 坐标不变";

  double w = 8.0, h = 8.0;
  letterbox_to_orig_size(0.0, w, h);
  EXPECT_TRUE(std::isfinite(w));
  EXPECT_DOUBLE_EQ(w, 8.0);
}

TEST(LetterboxTest, scale为负也当1处理)
{
  double x = 50.0f, y = 50.0f;
  letterbox_to_orig_xy(-1.0f, 0, 0, x, y);
  EXPECT_TRUE(std::isfinite(x));
  EXPECT_DOUBLE_EQ(x, 50.0) << "负 scale 会让结果翻符号 —— 也必须兜住";
}

TEST(LetterboxTest, 补边为负时如实计算不钳零)
{
  // ⚠️ `pad` 为负在正常 letterbox 里不会出现，但**别偷偷钳到 0** ——
  //    钳了会把"上游算错了"这件事**掩盖掉**（而那个错会一路穿到坐标里）。
  double x = 100.0f, y = 100.0f;
  letterbox_to_orig_xy(0.5f, -20, -20, x, y);
  EXPECT_NEAR(x, 240.0, 1e-6) << "(100-(-20))/0.5 = 240 —— 如实算，不钳";
}

}  // namespace camera
}  // namespace br_perception
