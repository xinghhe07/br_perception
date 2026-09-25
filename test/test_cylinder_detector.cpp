/// ═══════════════════════════════════════════════════════════════════════════════
/// test_cylinder_detector.cpp — 模块 13 圆柱检测：**纯函数**单元测试
///
/// ⚠️ 只测 `classify_by_height`（按柱高贴语义标签）—— 不起节点、不喂点云。
///    ⚠️ **别把本文件的绿当成"圆柱检测能跑"**：切片 / 合并 / 法向量 / 顶部检测都不在这里。
///
/// ── 为什么这个"几行 if"值得单测 ──
///   它给的是**语义标签**（穆斯蒂卡柱 / 核心支柱），而那两个标签下游直接喂
///   **那 250 分**的链（`mustika_pillar_check` 要拿穆斯蒂卡柱的位置去比对）。
///   而它有一处**刻意留的空档**最容易被人"顺手填上"：
/// ```
///   穆斯蒂卡柱  0.40 ~ 0.60
///   ─── 空档 ───               ← 填上它 = 给每个噪声团贴标签
///   核心支柱    0.70 ~ 0.90
/// ```
///   场上两种柱本来就分得开（500mm vs 800mm 级）⇒ 空档是留给**测量误差与台阶边缘**的。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=ClassifyByHeightTest.*`）。
/// ⚠️ 全部用例确定性：手造结构、无点云、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/lidar/cylinder_detector.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace br_perception {
namespace lidar {

using CD = CylinderDetector;
using Cyl = CylinderDetection;
using HP = CylinderDetector::HeightClassParams;

namespace {

Cyl cyl(float height, float conf = 0.0f, Cyl::Type t = Cyl::UNKNOWN)
{
  Cyl c;
  c.height = height;
  c.confidence = conf;
  c.type = t;
  return c;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 两段区间
// ═══════════════════════════════════════════════════════════════════════════

TEST(ClassifyByHeightTest, 低柱是穆斯蒂卡柱)
{
  HP p;                                  // 0.40~0.60 / 0.70~0.90
  std::vector<Cyl> v{cyl(0.50f)};
  CD::classify_by_height(v, p);
  EXPECT_EQ(v[0].type, Cyl::MUSTIKA_PILLAR);
  EXPECT_FLOAT_EQ(v[0].confidence, p.classified_confidence);
}

TEST(ClassifyByHeightTest, 高柱是核心支柱)
{
  HP p;
  std::vector<Cyl> v{cyl(0.80f)};
  CD::classify_by_height(v, p);
  EXPECT_EQ(v[0].type, Cyl::CORE_PILLAR);
  EXPECT_FLOAT_EQ(v[0].confidence, p.classified_confidence);
}

// ═══════════════════════════════════════════════════════════════════════════
// ★ 中间那条空档（最容易被"顺手填上"的地方）
// ═══════════════════════════════════════════════════════════════════════════

TEST(ClassifyByHeightTest, 空档里的柱不分类)
{
  // ⚠️⚠️ **本文件最贵的一条**。0.60~0.70 那段**刻意留空** ——
  //    它是留给**测量误差与台阶边缘**的：一个"半截柱子"落在里面，**报未知比报错好**。
  //    ⚠️ 把 `else if` 改成 `else`（或把区间连起来）⇒ **每个噪声团都会被贴上标签**，
  //    而症状是"检测出一些奇怪的柱子"—— 不报错，只是下游拿到错的语义。
  HP p;
  std::vector<Cyl> v{cyl(0.65f)};
  CD::classify_by_height(v, p);
  EXPECT_EQ(v[0].type, Cyl::UNKNOWN) << "空档里必须留 UNKNOWN —— 填上它 = 给噪声贴标签";
  EXPECT_FLOAT_EQ(v[0].confidence, 0.0f) << "不分类就不该抬置信度";
}

TEST(ClassifyByHeightTest, 空档两端之外也不分类)
{
  HP p;
  std::vector<Cyl> v{cyl(0.20f), cyl(1.50f), cyl(0.0f)};
  CD::classify_by_height(v, p);
  for (const auto& c : v) {
    EXPECT_EQ(c.type, Cyl::UNKNOWN) << "h=" << c.height << " 不该被分类";
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 边界（闭区间）
// ═══════════════════════════════════════════════════════════════════════════

TEST(ClassifyByHeightTest, 穆斯蒂卡区间端点是闭的)
{
  // ⚠️⚠️ **必须用二进制精确表示的值测端点**：
  //    `CylinderDetection::height` 是 **float**，而 `HeightClassParams` 是 **double**，
  //    且 `0.60` 在二进制里**不精确** —— `float(0.60)` = 0.6000000238 **大于** `double(0.60)`。
  //    ⇒ 拿 `0.60f` 测"恰好等于上界"**永远够不着**（差 2.4e-8 m，物理上无意义，
  //      但它确实让"闭区间"这条语义在**默认参数上演示不出来**）。
  //    与 `cluster_extractor` 的 `floating_z_min` 那次是**同一个坑**（见 §八 9.12）。
  //    ⇒ 换成 2 的幂：0.5 / 0.75，float/double 都精确。
  HP p;
  p.mustika_h_min = 0.5;    // 2⁻¹
  p.mustika_h_max = 0.75;   // 3·2⁻²
  std::vector<Cyl> v{cyl(0.5f), cyl(0.75f)};
  CD::classify_by_height(v, p);
  EXPECT_EQ(v[0].type, Cyl::MUSTIKA_PILLAR) << "恰好等于下界算在内（闭区间）";
  EXPECT_EQ(v[1].type, Cyl::MUSTIKA_PILLAR) << "恰好等于上界算在内（闭区间）";
}

TEST(ClassifyByHeightTest, 核心支柱区间端点是闭的)
{
  HP p;
  // ⚠️ **必须先把穆斯蒂卡那段挪开** —— 它是 `if` 的第一支，
  //    测试高度若落在 [mustika_min, mustika_max] 里会**先被穆斯蒂卡认走**。
  //    （我第一版就忘了这点，用例红了而原因是测试写错。）
  p.mustika_h_min = 10.0;
  p.mustika_h_max = 20.0;
  p.core_h_min = 0.5;       // 2⁻¹，可精确表示
  p.core_h_max = 1.0;
  std::vector<Cyl> v{cyl(0.5f), cyl(1.0f)};
  CD::classify_by_height(v, p);
  EXPECT_EQ(v[0].type, Cyl::CORE_PILLAR) << "恰好等于下界算在内";
  EXPECT_EQ(v[1].type, Cyl::CORE_PILLAR) << "恰好等于上界算在内";
}

TEST(ClassifyByHeightTest, 默认参数下的端点差异是float精度不是逻辑错)
{
  // ⚠️ **把上面那个 float/double 现象本身钉下来**，免得下一个人把它当成"闭区间写错了"。
  //    默认上界 0.60 是 double；高度 0.60f 是 float ⇒ 差 2.4e-8 ⇒ **落在区间之外**。
  //    ⚠️ 修法**不是**把 `<=` 改成容忍误差 —— 那会把"略超一点"也吃掉。
  HP p;   // 0.40 ~ 0.60
  std::vector<Cyl> v{cyl(0.60f)};
  CD::classify_by_height(v, p);
  EXPECT_EQ(v[0].type, Cyl::UNKNOWN)
      << "这是 float(0.60) > double(0.60) 的必然结果，不是闭区间写错了";
  // 而 0.59 明确在内
  std::vector<Cyl> v2{cyl(0.59f)};
  CD::classify_by_height(v2, p);
  EXPECT_EQ(v2[0].type, Cyl::MUSTIKA_PILLAR);
}

TEST(ClassifyByHeightTest, 端点外一点点就落空档或之外)
{
  HP p;
  std::vector<Cyl> v{cyl(0.399f), cyl(0.601f), cyl(0.699f), cyl(0.901f)};
  CD::classify_by_height(v, p);
  for (const auto& c : v) {
    EXPECT_EQ(c.type, Cyl::UNKNOWN) << "h=" << c.height << " 在区间外不该被分类";
  }
}

TEST(ClassifyByHeightTest, 区间重叠时穆斯蒂卡优先)
{
  // ⚠️ 判定顺序是 `if (穆斯蒂卡) … else if (核心)` ⇒ **重叠时穆斯蒂卡优先**。
  //    当前默认不重叠，但参数可改 —— 顺序要钉住，免得有人调参后语义悄悄变了。
  HP p;
  p.core_h_min = 0.50; p.core_h_max = 0.90;   // 故意与穆斯蒂卡区间重叠
  std::vector<Cyl> v{cyl(0.55f)};             // 两段都命中
  CD::classify_by_height(v, p);
  EXPECT_EQ(v[0].type, Cyl::MUSTIKA_PILLAR)
      << "重叠时穆斯蒂卡优先 —— 改顺序会让语义悄悄变";
}

// ═══════════════════════════════════════════════════════════════════════════
// 置信度
// ═══════════════════════════════════════════════════════════════════════════

TEST(ClassifyByHeightTest, 置信度只抬不降)
{
  // ⚠️ 写成直接赋值会把上游**更可靠**的置信度压下来
  HP p;   // classified_confidence = 0.75
  std::vector<Cyl> v{cyl(0.50f, /*conf=*/0.95f)};
  CD::classify_by_height(v, p);
  EXPECT_FLOAT_EQ(v[0].confidence, 0.95f) << "已有更高置信度时不该被压低";

  std::vector<Cyl> low{cyl(0.50f, /*conf=*/0.10f)};
  CD::classify_by_height(low, p);
  EXPECT_FLOAT_EQ(low[0].confidence, 0.75f) << "低置信度该被抬到分类下界";
}

TEST(ClassifyByHeightTest, 空列表不崩)
{
  HP p;
  std::vector<Cyl> v;
  CD::classify_by_height(v, p);
  EXPECT_TRUE(v.empty());
}

TEST(ClassifyByHeightTest, 已有标签且落在空档时保持原标签)
{
  // ⚠️ "不分类"= **不动**，不是"清成 UNKNOWN" ——
  //    若调用方之前已经贴过标签，这里不该把它擦掉。
  HP p;
  std::vector<Cyl> v{cyl(0.65f, 0.5f, Cyl::CORE_PILLAR)};
  CD::classify_by_height(v, p);
  EXPECT_EQ(v[0].type, Cyl::CORE_PILLAR) << "空档里应保持调用前的值，不是擦成 UNKNOWN";
  EXPECT_FLOAT_EQ(v[0].confidence, 0.5f);
}

}  // namespace lidar
}  // namespace br_perception
