/// ═══════════════════════════════════════════════════════════════════════════════
/// test_cluster_extractor.cpp — 模块 14 障碍物聚类与分类：**纯函数**单元测试
///
/// ⚠️ 本文件只测 `ClusterExtractor` 的 **static 纯函数**（不起 ROS 节点、不建图、不用 TF）：
///    欧式聚类 / 包围盒·质心·PCA / 规则过滤 / 尺寸分类 / 圆柱掩膜。
///    ⚠️ 没测的部分（回调组帧 / MarkerArray 发布 / 诊断上报 / 动态参数回设）需要真实
///    ROS 图，留给集成测试 —— 它们**一个都没被覆盖**，别把本文件的绿当成"节点能跑"。
///
/// ── 为什么值得单测（任务清单 `doc/容器化重构_任务清单_20260914.md` 任务 5）──
///   > 单测：不同距离下的聚类精度、尺寸过滤正确性、**分类标签正确性**
///   > 验收：重点看 `/perception/lidar/obstacles`，**分类标签必须 100% 一致**
///
///   而"分类标签"是这套代码里**最容易被静默改坏**的东西：
///   `classify_obstacle` 是"从大到小、第一个 `score>0.5` 的分支胜出"——**改一行顺序**
///   就会把敌方机器人判成天空方块，且**不报任何错**。过滤器的四条判据同理。
///
/// ⚠️⚠️ **覆盖面必须说准**：本文件保护的是**组件版** `src/lidar/cluster_extractor.cpp`。
///   实际在跑的是单体 `src/lidar/lidar_perception_node.cpp`，它把同一套逻辑**另写了一份**
///   （§六 的重复实现台账，等容器化重构合并）。≈ 本文件的绿**不等于**在跑的那份是绿的。
///   两份当前语义一致（2026-09-25 逐行核过），但**没有任何东西强制它们保持一致**。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=FilterTest.*`）。
/// ⚠️ 全部用例确定性：输入是手造格点点云，无随机、无时间、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/lidar/cluster_extractor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace br_perception {
namespace lidar {

using CE = ClusterExtractor;
using P  = ClusterExtractor::Params;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// 某轴第 `i` 个格点的归一化位置 ∈ [0, 1]（`m == 1` 时退化成一个点）
double axis_f(int i, int m)
{
  return (m > 1) ? static_cast<double>(i) / (m - 1) : 0.0;
}

/// 规则格点长方体：包围盒**精确**等于 (sx, sy, sz)，最小角在 (ox, oy, oz)。
/// 每轴 `m` 个格点 ⇒ 点数 = mx·my·mz，该轴间距 = s / (m − 1)。
///
/// ⚠️ 用**格点**而不是随机点：包围盒 / 质心 / 点数 / 特征值全是**可以手算的精确值**，
///    期望值不必靠"跑一遍看看是多少"来定（那种随实现漂的用例没有约束力）。
CE::PointCloudPtr grid_box(double sx, double sy, double sz,
                           int mx, int my, int mz,
                           double ox = 0.0, double oy = 0.0, double oz = 0.0)
{
  auto c = std::make_shared<CE::PointCloud>();
  for (int i = 0; i < mx; ++i) {
    for (int j = 0; j < my; ++j) {
      for (int k = 0; k < mz; ++k) {
        c->push_back(pcl::PointXYZ(
            static_cast<float>(ox + sx * axis_f(i, mx)),
            static_cast<float>(oy + sy * axis_f(j, my)),
            static_cast<float>(oz + sz * axis_f(k, mz))));
      }
    }
  }
  c->width  = c->size();
  c->height = 1;
  c->is_dense = true;
  return c;
}

/// 某轴间距 ≤ `step` 所需的最少格点数
int grid_n(double s, double step)
{
  return std::max(2, static_cast<int>(std::ceil(s / step)) + 1);
}

/// 致密长方体：保证**同一坨内部**任意相邻格点间距 ≤ `step`
/// ⇒ 只要 `step < cluster_tolerance`，这一坨必然聚成**一簇**（而不是散成几十簇）。
CE::PointCloudPtr dense_box(double sx, double sy, double sz,
                            double ox, double oy, double oz, double step)
{
  return grid_box(sx, sy, sz,
                  grid_n(sx, step), grid_n(sy, step), grid_n(sz, step),
                  ox, oy, oz);
}

/// 一簇的点索引 = 整份点云的 0..n−1（"点云就是这一簇"时用）
pcl::PointIndices all_of(const CE::PointCloudPtr& c)
{
  pcl::PointIndices idx;
  idx.indices.resize(c->size());
  for (size_t i = 0; i < c->size(); ++i) {
    idx.indices[i] = static_cast<int>(i);
  }
  return idx;
}

/// 只填判定用得到的字段的 Obstacle（`min_z` 默认 0 = 落在地上）
Obstacle obs_of(float l, float w, float h, int pts, float min_z = 0.0f)
{
  Obstacle o;
  o.length      = l;
  o.width       = w;
  o.height      = h;
  o.point_count = pts;
  o.min_z       = min_z;
  o.max_z       = min_z + h;
  o.centroid_z  = min_z + h * 0.5f;
  return o;
}

/// 主轴方向按**模 π** 比较。
/// ⚠️ PCA 的主轴有 **± 二义性**（`v1` 与 `−v1` 都是合法特征向量，Eigen 取哪个取决于
///    内部符号约定）⇒ `yaw` 只在模 π 意义下有意义。直接断言 `yaw == 30°` 会写出一个
///    **随 Eigen/LAPACK 版本飘**的用例。
void expect_yaw_mod_pi(double got, double want)
{
  double d = std::fmod(std::abs(got - want), kPi);
  if (d > kPi / 2.0) d = kPi - d;
  EXPECT_NEAR(d, 0.0, 1e-4) << "got=" << got << " want=" << want;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 规则过滤 —— 4 条判据 + 4 条**边界**
// ═══════════════════════════════════════════════════════════════════════════

TEST(FilterTest, 正常物体通过)
{
  P p;
  EXPECT_FALSE(CE::filter_obstacle(obs_of(0.35f, 0.35f, 0.35f, 100), p));
}

TEST(FilterTest, 过长被滤掉)
{
  P p;
  EXPECT_TRUE(CE::filter_obstacle(obs_of(2.5f, 0.30f, 0.30f, 100), p));
}

TEST(FilterTest, 过宽被滤掉)
{
  // 长和宽**分别**判 —— 只判长会漏掉"横向的墙"
  P p;
  EXPECT_TRUE(CE::filter_obstacle(obs_of(0.30f, 2.5f, 0.30f, 100), p));
}

TEST(FilterTest, 过高被滤掉)
{
  P p;
  EXPECT_TRUE(CE::filter_obstacle(obs_of(0.50f, 0.50f, 2.50f, 100), p));
}

TEST(FilterTest, 点数过少被滤掉)
{
  P p;
  EXPECT_TRUE(CE::filter_obstacle(obs_of(0.35f, 0.35f, 0.35f, 4), p));
}

TEST(FilterTest, 悬空被滤掉)
{
  P p;  // floating_z_min 默认 0.8
  EXPECT_TRUE(CE::filter_obstacle(obs_of(0.35f, 0.35f, 0.35f, 100, 1.20f), p));
}

TEST(FilterTest, 边界_恰好等于最大尺寸不滤)
{
  // ⚠️ 四条判据全是**严格** > / < ⇒ 恰好等于阈值**不**剔除。
  //    写成 `>=` 会让"刚好 2.0m 的板"从"保留"变成"剔除"，且没有任何报错。
  P p;
  EXPECT_FALSE(CE::filter_obstacle(obs_of(2.0f, 2.0f, 2.0f, 100), p));
}

TEST(FilterTest, 边界_恰好等于最大高度不滤)
{
  P p;
  EXPECT_FALSE(CE::filter_obstacle(obs_of(0.30f, 0.30f, 2.0f, 100), p));
}

TEST(FilterTest, 边界_恰好等于最小点数不滤)
{
  P p;  // min_object_points 默认 5
  EXPECT_FALSE(CE::filter_obstacle(obs_of(0.35f, 0.35f, 0.35f, 5), p));
}

TEST(FilterTest, 边界_恰好等于悬空阈值不滤)
{
  // ⚠️ **为什么不用默认的 0.8 测这条边界**：`floating_z_min` 是 **double**，
  //    而 `Obstacle::min_z` 是 **float**，且 0.8 在二进制里**不可精确表示**——
  //    `float(0.8)` = 0.800000011920929 **大于** `double(0.8)` = 0.8。
  //    ⇒ 拿 0.8f 喂进去**必然**被判成"高于阈值"，**永远演示不到"恰好相等"**。
  //    （只差 1.2e-8 m，物理上毫无意义；但它确实让"严格大于"这条语义在默认值上不可测。）
  //    这里换一个 float / double 都能**精确表示**的值来测那条语义本身。
  P p;
  p.floating_z_min = 0.5;  // 0.5 = 2⁻¹，两种类型都精确
  EXPECT_FALSE(CE::filter_obstacle(obs_of(0.35f, 0.35f, 0.35f, 100, 0.5f), p));
  // 再加一丝丝就必须被滤 —— 证明上一条不是"恒返回 false"
  EXPECT_TRUE(CE::filter_obstacle(obs_of(0.35f, 0.35f, 0.35f, 100, 0.5f + 1e-3f), p));
}

TEST(FilterTest, 判据取自参数而不是硬编码)
{
  // 改参数必须真的改变结论（否则又是"配了不生效"那一类）
  P p;
  const Obstacle o = obs_of(0.35f, 0.35f, 0.35f, 100);
  ASSERT_FALSE(CE::filter_obstacle(o, p));
  p.max_object_size = 0.20;
  EXPECT_TRUE(CE::filter_obstacle(o, p));
}

// ═══════════════════════════════════════════════════════════════════════════
// 尺寸分类 —— 验收要求"分类标签必须 100% 一致"
// ═══════════════════════════════════════════════════════════════════════════

TEST(ClassifyTest, 350mm立方体是地球方块)
{
  P p;
  Obstacle o = obs_of(0.35f, 0.35f, 0.35f, 100);
  CE::classify_obstacle(o, p);
  EXPECT_EQ(o.type, ObstacleType::EARTH_CUBE);
  EXPECT_NEAR(o.confidence, 1.0f, 1e-6);  // 三轴都精确命中
}

TEST(ClassifyTest, 700mm立方体是敌方机器人)
{
  P p;
  Obstacle o = obs_of(0.70f, 0.70f, 0.70f, 100);
  CE::classify_obstacle(o, p);
  EXPECT_EQ(o.type, ObstacleType::ENEMY_ROBOT);
  EXPECT_NEAR(o.confidence, 1.0f, 1e-6);
}

TEST(ClassifyTest, 200mm各向异性是天空方块)
{
  P p;
  Obstacle o = obs_of(0.20f, 0.20f, 0.20f, 100);
  o.eigen_ratio_1 = 1.00f;  // λ1 ≈ λ2
  o.eigen_ratio_2 = 5.00f;  // 但 λ2 ≫ λ3 ⇒ 明显不是球
  CE::classify_obstacle(o, p);
  EXPECT_EQ(o.type, ObstacleType::SKY_CUBE);
  EXPECT_NEAR(o.confidence, 1.0f, 1e-6);
}

TEST(ClassifyTest, 200mm各向同性是穆斯蒂卡)
{
  P p;
  Obstacle o = obs_of(0.20f, 0.20f, 0.20f, 100);
  o.eigen_ratio_1 = 1.05f;
  o.eigen_ratio_2 = 1.05f;
  CE::classify_obstacle(o, p);
  EXPECT_EQ(o.type, ObstacleType::MUSTIKA);
  EXPECT_NEAR(o.confidence, 0.85f, 1e-6);  // size_score(1.0) × 0.85
}

TEST(ClassifyTest, 球的置信度低于同尺寸立方体)
{
  // "球体判断更不确定"这条**必须**落在置信度上，否则下游无法在两者间取舍
  P p;
  Obstacle cube = obs_of(0.20f, 0.20f, 0.20f, 100);
  cube.eigen_ratio_1 = 1.00f;
  cube.eigen_ratio_2 = 5.00f;

  Obstacle ball = obs_of(0.20f, 0.20f, 0.20f, 100);
  ball.eigen_ratio_1 = 1.05f;
  ball.eigen_ratio_2 = 1.05f;

  CE::classify_obstacle(cube, p);
  CE::classify_obstacle(ball, p);
  ASSERT_EQ(cube.type, ObstacleType::SKY_CUBE);
  ASSERT_EQ(ball.type, ObstacleType::MUSTIKA);
  EXPECT_LT(ball.confidence, cube.confidence);
}

TEST(ClassifyTest, 尺寸对不上是未知)
{
  P p;
  Obstacle o = obs_of(0.50f, 0.50f, 0.50f, 100);  // 550mm 落在 350 与 700 之间的空档
  CE::classify_obstacle(o, p);
  EXPECT_EQ(o.type, ObstacleType::UNKNOWN);
  EXPECT_FLOAT_EQ(o.confidence, 0.0f);
}

TEST(ClassifyTest, 分支顺序_700mm且各向同性仍判敌方机器人)
{
  // ⚠️ 顺序敏感：敌方分支**先于**球体判据 ⇒ 一个"像球的 700mm 团块"必须是敌方机器人。
  //    场上不存在 700mm 的球，若让 MUSTIKA 抢走就是**把敌方机器人报成 250 分的球**。
  P p;
  Obstacle o = obs_of(0.70f, 0.70f, 0.70f, 100);
  o.eigen_ratio_1 = 1.00f;
  o.eigen_ratio_2 = 1.00f;  // 各向同性 —— 球体判据会成立
  CE::classify_obstacle(o, p);
  EXPECT_EQ(o.type, ObstacleType::ENEMY_ROBOT);
}

TEST(ClassifyTest, 特征比值未算出时不判球)
{
  // `compute_pca_orientation` 在 n<3 时填的是 1.0（各向同性），而 `ratio > 0` 这两条
  // 挡的是真正的 0 —— 但反过来说：**比值恰好为 0（真的没填）时不许判球**。
  P p;
  Obstacle o = obs_of(0.20f, 0.20f, 0.20f, 100);
  o.eigen_ratio_1 = 0.0f;
  o.eigen_ratio_2 = 0.0f;
  CE::classify_obstacle(o, p);
  EXPECT_EQ(o.type, ObstacleType::SKY_CUBE);
}

TEST(ClassifyTest, mustika_size是活的_改它会改变结论)
{
  // ⚠️ 2026-09-25 之前 `mustika_size` 是**死参数**：声明了、读了、日志里打了、还能回设，
  //    但**任何计算都没用它** ⇒ 改 yaml 里这个键**静默不生效**（§零「稳」轴点名那一类）。
  //    本用例钉住"它现在真的参与判定"：故意把球尺寸配错，球就该判不出来。
  P p;
  p.mustika_size = 0.30;  // 故意配错
  Obstacle o = obs_of(0.20f, 0.20f, 0.20f, 100);
  o.eigen_ratio_1 = 1.05f;
  o.eigen_ratio_2 = 1.05f;
  CE::classify_obstacle(o, p);
  EXPECT_EQ(o.type, ObstacleType::SKY_CUBE);  // 退到立方体分支
}

TEST(ClassifyTest, 两尺寸键相等时结论与改前一致)
{
  // 兜底：本轮把"只算 sky_cube_size"改成"两个键各算各的分"，
  // 在**今天 yaml 的取值**（两者都是 0.20）下必须**逐位不变**。
  P p;
  ASSERT_DOUBLE_EQ(p.mustika_size, p.sky_cube_size);

  Obstacle ball = obs_of(0.20f, 0.20f, 0.20f, 100);
  ball.eigen_ratio_1 = 1.05f;
  ball.eigen_ratio_2 = 1.05f;
  CE::classify_obstacle(ball, p);
  EXPECT_EQ(ball.type, ObstacleType::MUSTIKA);
  EXPECT_NEAR(ball.confidence, 0.85f, 1e-6);

  Obstacle cube = obs_of(0.20f, 0.20f, 0.20f, 100);
  cube.eigen_ratio_1 = 1.00f;
  cube.eigen_ratio_2 = 5.00f;
  CE::classify_obstacle(cube, p);
  EXPECT_EQ(cube.type, ObstacleType::SKY_CUBE);
  EXPECT_NEAR(cube.confidence, 1.0f, 1e-6);
}

// ═══════════════════════════════════════════════════════════════════════════
// PCA 主轴方向
// ═══════════════════════════════════════════════════════════════════════════

TEST(PcaTest, 沿x轴的细长点云_主轴在x向且判为elongated)
{
  // ⚠️ 三个轴的展布**必须两两分得开**（0.80 / 0.20 / 0.02 ⇒ 方差比 16 : 1 : 0.01）：
  //    若让 y 与 z 展布相同（如 0.02/0.02），λ2 与 λ3 就**简并**，主轴方向数值上变得
  //    敏感，`yaw` 会飘出 1e-2 量级 —— 那是"题目本身病态"，不是代码错。
  P p;
  auto c = grid_box(0.80, 0.20, 0.02, 3, 3, 3);
  Obstacle o;
  CE::compute_pca_orientation(c, all_of(c), o, p);
  expect_yaw_mod_pi(o.yaw, 0.0);
  EXPECT_TRUE(o.is_elongated);
  EXPECT_GT(o.eigen_ratio_1, p.elongation_ratio);
}

TEST(PcaTest, 30度斜线的主轴偏航角)
{
  P p;
  auto c = std::make_shared<CE::PointCloud>();
  const double a  = 30.0 * kPi / 180.0;
  const double ca = std::cos(a);
  const double sa = std::sin(a);
  for (int i = 0; i <= 20; ++i) {
    const double t = 0.40 * i / 20.0;      // 沿主轴走 0.40m
    for (int j = 0; j < 3; ++j) {
      const double off = 0.001 * j;        // 横向给一点点厚度，别让 λ2 恰好为 0
      c->push_back(pcl::PointXYZ(
          static_cast<float>(t * ca - off * sa),
          static_cast<float>(t * sa + off * ca),
          static_cast<float>(0.001 * j)));
    }
  }
  Obstacle o;
  CE::compute_pca_orientation(c, all_of(c), o, p);
  expect_yaw_mod_pi(o.yaw, a);
}

TEST(PcaTest, 点太少时回落成各向同性而不是零)
{
  // ⚠️ n<3 时填的是 **1.0（各向同性）**，不是 0。填 0 会让 `classify_obstacle` 的
  //    球体判据 `ratio > 0` 直接判假 ⇒ 把"没算出来"当成"不是球"。
  P p;
  auto c = grid_box(0.20, 0.20, 0.20, 2, 2, 2);
  pcl::PointIndices idx;
  idx.indices = {0, 1};  // 只喂 2 个点
  Obstacle o;
  CE::compute_pca_orientation(c, idx, o, p);
  EXPECT_FLOAT_EQ(o.eigen_ratio_1, 1.0f);
  EXPECT_FLOAT_EQ(o.eigen_ratio_2, 1.0f);
  EXPECT_FALSE(o.is_elongated);
  EXPECT_FLOAT_EQ(o.yaw, 0.0f);
}

TEST(PcaTest, 对称点云各向同性_不是elongated)
{
  P p;
  auto c = std::make_shared<CE::PointCloud>();
  const float r = 0.10f;
  for (int ax = 0; ax < 3; ++ax) {
    for (int s = -1; s <= 1; s += 2) {
      float v[3] = {0.0f, 0.0f, 0.0f};
      v[ax] = r * static_cast<float>(s);
      c->push_back(pcl::PointXYZ(v[0], v[1], v[2]));
    }
  }
  Obstacle o;
  CE::compute_pca_orientation(c, all_of(c), o, p);
  EXPECT_NEAR(o.eigen_ratio_1, 1.0f, 1e-5);
  EXPECT_NEAR(o.eigen_ratio_2, 1.0f, 1e-5);
  EXPECT_FALSE(o.is_elongated);
}

TEST(PcaTest, 完整立方体的特征值也是各向同性的)
{
  // ⚠️⚠️ **这根钉子很重要**：一个**采样完整**的立方体，其协方差矩阵在三个轴上
  //      完全对称 ⇒ λ1≈λ2≈λ3 ⇒ 球体判据**成立**。
  //      也就是说 `classify_obstacle` 的"球 vs 立方"分叉**不是**靠形状本身，
  //      而是靠**雷达的遮挡**（只看得到两三个面 ⇒ 采样偏斜 ⇒ 特征值才分开）。
  //      本用例把这个前提**写死**：谁要是以为"立方体天然各向异性"，会在这里撞到。
  P p;
  auto c = grid_box(0.20, 0.20, 0.20, 4, 4, 4);  // 完整立方体
  Obstacle o;
  CE::compute_pca_orientation(c, all_of(c), o, p);
  EXPECT_NEAR(o.eigen_ratio_1, 1.0f, 1e-4);
  EXPECT_NEAR(o.eigen_ratio_2, 1.0f, 1e-4);
}

TEST(PcaTest, 直接调用不必先填质心)
{
  // ⚠️ **这条钉的是 2026-09-25 抓到的一个静默陷阱**：
  //    `compute_pca_orientation` 原先直接读 `obs.centroid_x/y/z` ⇒ 传一个**默认构造**
  //    的 `Obstacle` 进来，它会绕**原点**求协方差，给出一组"看着合理"的错特征值与错主轴，
  //    **不报错、不抛异常**。`compute_obstacle` 恰好先填了质心，所以节点路径一直是对的
  //    —— 也正因如此，这个坑**读代码看不出来**，只有"单独调一次并对比"才露头。
  //    改成自算质心后，两条路必须给同一个答案；谁把它改回读 `obs.centroid_*`，这里立刻红。
  P p;
  // 点云**故意离原点很远** (1, 2, 0.5)：绕原点求协方差与绕质心求，结果天差地别
  auto c = grid_box(0.80, 0.20, 0.02, 3, 3, 3, 1.0, 2.0, 0.5);

  Obstacle direct;                            // 质心字段全 0（没填）
  ASSERT_FLOAT_EQ(direct.centroid_x, 0.0f);
  CE::compute_pca_orientation(c, all_of(c), direct, p);

  Obstacle via = CE::compute_obstacle(c, all_of(c), 0, p);  // 正常路径（先填质心）

  EXPECT_NEAR(direct.eigen_ratio_1, via.eigen_ratio_1, 1e-4);
  EXPECT_NEAR(direct.eigen_ratio_2, via.eigen_ratio_2, 1e-4);
  EXPECT_EQ(direct.is_elongated, via.is_elongated);
  expect_yaw_mod_pi(direct.yaw, via.yaw);
}

// ═══════════════════════════════════════════════════════════════════════════
// 单聚类 → Obstacle（包围盒 / 质心 / 点数）
// ═══════════════════════════════════════════════════════════════════════════

TEST(ComputeObstacleTest, 包围盒质心点数都是精确值)
{
  P p;
  // 0.35 立方体，最小角在 (1, 2, 0.5)，每轴 3 个格点 ⇒ 27 点
  auto c = grid_box(0.35, 0.35, 0.35, 3, 3, 3, 1.0, 2.0, 0.5);
  Obstacle o = CE::compute_obstacle(c, all_of(c), 7, p);

  EXPECT_EQ(o.id, 7);
  EXPECT_EQ(o.point_count, 27);
  EXPECT_NEAR(o.min_x, 1.00f, 1e-5);
  EXPECT_NEAR(o.max_x, 1.35f, 1e-5);
  EXPECT_NEAR(o.min_y, 2.00f, 1e-5);
  EXPECT_NEAR(o.max_y, 2.35f, 1e-5);
  EXPECT_NEAR(o.min_z, 0.50f, 1e-5);
  EXPECT_NEAR(o.max_z, 0.85f, 1e-5);
  EXPECT_NEAR(o.length, 0.35f, 1e-5);
  EXPECT_NEAR(o.width,  0.35f, 1e-5);
  EXPECT_NEAR(o.height, 0.35f, 1e-5);
  // 格点对称 ⇒ 质心 = 几何中心
  EXPECT_NEAR(o.centroid_x, 1.175f, 1e-5);
  EXPECT_NEAR(o.centroid_y, 2.175f, 1e-5);
  EXPECT_NEAR(o.centroid_z, 0.675f, 1e-5);
}

TEST(ComputeObstacleTest, 只看簇内的点不看整份点云)
{
  P p;
  auto c = grid_box(0.35, 0.35, 0.35, 3, 3, 3);  // 27 点
  pcl::PointIndices idx;
  idx.indices = {0, 1, 2};  // 只取 i=0,j=0 那一列（x, y 全相同，z 走满）
  Obstacle o = CE::compute_obstacle(c, idx, 0, p);

  EXPECT_EQ(o.point_count, 3);
  EXPECT_NEAR(o.min_x, 0.0f, 1e-5);
  EXPECT_NEAR(o.max_x, 0.0f, 1e-5);
  EXPECT_NEAR(o.min_z, 0.0f, 1e-5);
  EXPECT_NEAR(o.max_z, 0.35f, 1e-5);
}

TEST(ComputeObstacleTest, 空簇返回全零且不崩)
{
  P p;
  auto c = grid_box(0.35, 0.35, 0.35, 3, 3, 3);
  pcl::PointIndices empty;
  Obstacle o = CE::compute_obstacle(c, empty, 3, p);
  EXPECT_EQ(o.id, 3);
  EXPECT_EQ(o.point_count, 0);
  EXPECT_FLOAT_EQ(o.length, 0.0f);
  EXPECT_FLOAT_EQ(o.height, 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// 圆柱掩膜
// ═══════════════════════════════════════════════════════════════════════════

namespace {

/// 直观点云：直接给 (x, y, z)
CE::PointCloudPtr pts(std::initializer_list<pcl::PointXYZ> in)
{
  auto c = std::make_shared<CE::PointCloud>();
  for (const auto& p : in) c->push_back(p);
  c->width = c->size();
  c->height = 1;
  c->is_dense = true;
  return c;
}

}  // namespace

TEST(MaskCylindersTest, 圆内删掉圆外保留)
{
  P p;
  p.cylinder_mask_radius = 0.25;
  p.cylinder_centers = {{0.0, 0.0}};
  auto c = pts({{0.00f, 0.0f, 0.0f},    // 命中（圆心）
                {0.10f, 0.0f, 0.0f},    // 命中
                {0.24f, 0.0f, 0.0f},    // 命中
                {0.26f, 0.0f, 0.0f},    // 保留
                {1.00f, 1.0f, 0.0f}});  // 保留
  auto out = CE::mask_cylinders(c, p);
  ASSERT_EQ(out->size(), 2u);
  EXPECT_FLOAT_EQ(out->points[0].x, 0.26f);
  EXPECT_FLOAT_EQ(out->points[1].x, 1.00f);
}

TEST(MaskCylindersTest, 边界_恰好等于半径的点保留)
{
  // ⚠️ 边界是**严格小于** ⇒ 恰好落在半径上的点保留
  P p;
  p.cylinder_mask_radius = 0.25;
  p.cylinder_centers = {{0.0, 0.0}};
  auto c = pts({{0.25f, 0.0f, 0.0f}});
  auto out = CE::mask_cylinders(c, p);
  EXPECT_EQ(out->size(), 1u);
}

TEST(MaskCylindersTest, 不判z_圆柱正上方也被删)
{
  // ⚠️ 只判**水平**距离：z 完全不参与 ⇒ 圆柱正上方 5m 处的点也会被删。
  //    这是**有意**的（圆柱全长都可能是残点），但要知道后果。
  P p;
  p.cylinder_mask_radius = 0.25;
  p.cylinder_centers = {{0.0, 0.0}};
  auto c = pts({{0.0f, 0.0f, 5.0f}});
  auto out = CE::mask_cylinders(c, p);
  EXPECT_EQ(out->size(), 0u);
}

TEST(MaskCylindersTest, 没配圆柱时原样返回同一个指针)
{
  // 不复制 —— 调用方拿到的是同一块内存（下游只读，安全）
  P p;  // cylinder_centers 默认空
  auto c = pts({{1.0f, 2.0f, 3.0f}});
  auto out = CE::mask_cylinders(c, p);
  EXPECT_EQ(out.get(), c.get());
}

TEST(MaskCylindersTest, 多个圆柱中心任一命中即删)
{
  P p;
  p.cylinder_mask_radius = 0.25;
  p.cylinder_centers = {{0.0, 0.0}, {1.0, 0.0}};
  auto c = pts({{1.10f, 0.0f, 0.0f}});  // 命中第二个中心
  auto out = CE::mask_cylinders(c, p);
  EXPECT_EQ(out->size(), 0u);
}

// ═══════════════════════════════════════════════════════════════════════════
// 欧式聚类 —— 对应验收里的"不同距离下的聚类精度"
// ═══════════════════════════════════════════════════════════════════════════

TEST(ExtractClustersTest, 相距远的两坨分成两簇)
{
  P p;  // cluster_tolerance 默认 0.05
  auto c = std::make_shared<CE::PointCloud>();
  *c += *dense_box(0.04, 0.04, 0.04, 0.0, 0.0, 0.0, p.cluster_tolerance * 0.8);
  *c += *dense_box(0.04, 0.04, 0.04, 0.5, 0.0, 0.0, p.cluster_tolerance * 0.8);
  auto cl = CE::extract_clusters(c, p);
  EXPECT_EQ(cl.size(), 2u);
}

TEST(ExtractClustersTest, 间距小于容差的两坨粘连成一簇)
{
  // ⚠️ 这是**固定容差**的固有行为，不是 bug：两物体最近点间距 0.03m < 0.05m ⇒ 合成一簇。
  //    场上靠"物体实际间距 ≫ 0.05m"回避，而不是靠算法 —— 把这个前提钉下来。
  P p;
  auto c = std::make_shared<CE::PointCloud>();
  *c += *dense_box(0.04, 0.04, 0.04, 0.00, 0.0, 0.0, p.cluster_tolerance * 0.8);
  *c += *dense_box(0.04, 0.04, 0.04, 0.07, 0.0, 0.0, p.cluster_tolerance * 0.8);
  auto cl = CE::extract_clusters(c, p);
  EXPECT_EQ(cl.size(), 1u);
}

TEST(ExtractClustersTest, 同样间距挪到远处结论不变)
{
  // ⚠️ 规格写的是"不同距离下的聚类精度"，但本算法用的是**固定欧式容差** ——
  //    判据不是"距离"本身，而是**物体之间的间距**。同样间距挪到 10m 外，结论必须一样。
  P p;
  auto c = std::make_shared<CE::PointCloud>();
  *c += *dense_box(0.04, 0.04, 0.04, 10.0, 0.0, 0.0, p.cluster_tolerance * 0.8);
  *c += *dense_box(0.04, 0.04, 0.04, 10.5, 0.0, 0.0, p.cluster_tolerance * 0.8);
  auto cl = CE::extract_clusters(c, p);
  EXPECT_EQ(cl.size(), 2u);
}

TEST(ExtractClustersTest, 小于最小簇点数的坨被丢掉)
{
  P p;
  p.min_cluster_size = 50;
  auto c = dense_box(0.04, 0.04, 0.04, 0.0, 0.0, 0.0, p.cluster_tolerance * 0.8);
  ASSERT_LT(static_cast<int>(c->size()), p.min_cluster_size);
  auto cl = CE::extract_clusters(c, p);
  EXPECT_EQ(cl.size(), 0u);
}

TEST(ExtractClustersTest, 超过最大簇点数的坨被丢掉)
{
  P p;
  p.max_cluster_size = 20;
  // ⚠️ 这坨必须**又大又密**（> 20 点且内部间距 < 容差）：用 0.04 的小盒只会得到
  //    8 个格点，连阈值都够不着，测的就成了"点数不足"那条路。
  auto c = dense_box(0.20, 0.20, 0.20, 0.0, 0.0, 0.0, p.cluster_tolerance * 0.8);
  ASSERT_GT(static_cast<int>(c->size()), p.max_cluster_size);
  auto cl = CE::extract_clusters(c, p);
  EXPECT_EQ(cl.size(), 0u);
}

TEST(ExtractClustersTest, 点云总点数不足时直接早退)
{
  // ⚠️ 这是**点数不足**的早退（连 KdTree 都不建），不是"聚类结果为空" ——
  //    下游看起来一模一样，但耗时不同。
  P p;
  p.min_cluster_size = 100;
  auto c = dense_box(0.04, 0.04, 0.04, 0.0, 0.0, 0.0, p.cluster_tolerance * 0.8);
  ASSERT_LT(static_cast<int>(c->size()), p.min_cluster_size);
  auto cl = CE::extract_clusters(c, p);
  EXPECT_EQ(cl.size(), 0u);
}

TEST(ExtractClustersTest, 两簇点数之和等于参与聚类的点数)
{
  P p;
  auto c = std::make_shared<CE::PointCloud>();
  *c += *dense_box(0.04, 0.04, 0.04, 0.0, 0.0, 0.0, p.cluster_tolerance * 0.8);
  *c += *dense_box(0.04, 0.04, 0.04, 0.5, 0.0, 0.0, p.cluster_tolerance * 0.8);
  auto cl = CE::extract_clusters(c, p);
  ASSERT_EQ(cl.size(), 2u);
  size_t total = 0;
  for (const auto& x : cl) total += x.indices.size();
  EXPECT_EQ(total, c->size());
}

// ═══════════════════════════════════════════════════════════════════════════
// 纯函数串起来跑一遍 —— 直接对应验收那句"分类标签必须 100% 一致"
// ═══════════════════════════════════════════════════════════════════════════

TEST(PipelineTest, 三物体场景的分类标签逐条核对)
{
  P p;
  p.cluster_tolerance = 0.10;  // 放宽一点 ⇒ 簇内点数少、跑得快（不改判定语义）

  const double step = p.cluster_tolerance * 0.8;
  auto c = std::make_shared<CE::PointCloud>();
  *c += *dense_box(0.35, 0.35, 0.35, 0.0, 0.0, 0.0, step);  // 地球方块
  *c += *dense_box(0.70, 0.70, 0.70, 2.0, 0.0, 0.0, step);  // 敌方机器人
  *c += *dense_box(0.20, 0.20, 0.06, 4.0, 0.0, 0.0, step);  // 天空方块（扁 ⇒ 各向异性）

  auto clusters = CE::extract_clusters(c, p);
  ASSERT_EQ(clusters.size(), 3u);

  // ⚠️ 不依赖 PCL 返回簇的顺序（那是实现细节）⇒ 按质心 x 排序后再核对
  std::vector<std::pair<float, ObstacleType>> got;
  for (size_t i = 0; i < clusters.size(); ++i) {
    Obstacle o = CE::compute_obstacle(c, clusters[i], static_cast<int>(i), p);
    ASSERT_FALSE(CE::filter_obstacle(o, p))
        << "第 " << i << " 簇被过滤掉了，场景不该有物体被滤";
    CE::classify_obstacle(o, p);
    got.emplace_back(o.centroid_x, o.type);
  }
  ASSERT_EQ(got.size(), 3u);
  std::sort(got.begin(), got.end(),
            [](const std::pair<float, ObstacleType>& a,
               const std::pair<float, ObstacleType>& b) { return a.first < b.first; });

  EXPECT_EQ(got[0].second, ObstacleType::EARTH_CUBE);
  EXPECT_EQ(got[1].second, ObstacleType::ENEMY_ROBOT);
  EXPECT_EQ(got[2].second, ObstacleType::SKY_CUBE);
}

TEST(PipelineTest, 圆柱掩膜在聚类之前生效)
{
  // 掩膜删掉圆柱附近的点 ⇒ 那坨不再成簇。串起来验证调用顺序的**效果**。
  P p;
  p.cluster_tolerance = 0.10;
  const double step = p.cluster_tolerance * 0.8;

  auto c = std::make_shared<CE::PointCloud>();
  *c += *dense_box(0.10, 0.10, 0.10, 0.0, 0.0, 0.0, step);  // 会被掩膜吃掉
  *c += *dense_box(0.10, 0.10, 0.10, 2.0, 0.0, 0.0, step);  // 离得远，保留

  p.cylinder_mask_radius = 0.30;
  p.cylinder_centers = {{0.05, 0.05}};   // 罩住第一坨

  auto masked = CE::mask_cylinders(c, p);
  ASSERT_LT(masked->size(), c->size());

  auto cl = CE::extract_clusters(masked, p);
  ASSERT_EQ(cl.size(), 1u);
  Obstacle o = CE::compute_obstacle(masked, cl[0], 0, p);
  EXPECT_GT(o.centroid_x, 1.0f);  // 活下来的是远处那一坨
}

}  // namespace lidar
}  // namespace br_perception
