/// ═══════════════════════════════════════════════════════════════════════════════
/// test_target_associator.cpp — 模块 19 观测↔追踪关联：**纯函数**单元测试
///
/// ⚠️ `TargetAssociator` 全是 static 纯函数（不起节点、不碰 ROS 消息）⇒ 直接喂。
///    话题订阅/发布归模块 21，不在本文件覆盖范围。
///
/// ── 为什么这个模块值得单测 ──
///   它的**核心语义是"落区优先于就近"**（见头文件的 ⚠️⚠️），而这条语义
///   **改错了不会报错**：
///     · 改成"纯就近" ⇒ 一个观测离 track A 更近、但明显落在已知位置 B 的区域内时，
///       会被错配给 A ⇒ **建筑位归属偶尔串位**，只在特定位姿下偶发；
///     · 而它下游连着 kalman_tracker 的 id、再下游连着 40/80 分的 `top_color` 归属。
///   ⇒ 本文件最贵的一条是 `落区优先于就近`。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=AssociateTest.*`）。
/// ⚠️ 全部用例确定性：手造 POD、无时间、无随机。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/fusion/target_associator.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace br_perception {
namespace fusion {

using TA = TargetAssociator;
using Obs = TargetAssociator::Observation;
using Trk = TargetAssociator::Track;
using Reg = TargetAssociator::KnownRegion;

namespace {

constexpr std::uint8_t kLidar  = 1;
constexpr std::uint8_t kCamera = 2;

Obs obs(double x, double y, double z = 0.0,
        std::uint8_t src = kLidar, bool held = false)
{
  Obs o;
  o.x = x; o.y = y; o.z = z;
  o.source = src;
  o.is_held = held;
  o.confidence = 1.0f;
  return o;
}

Trk trk(std::uint32_t id, double x, double y, double z = 0.0, bool held = false)
{
  Trk t;
  t.id = id; t.x = x; t.y = y; t.z = z;
  t.is_held = held;
  return t;
}

Reg reg(std::uint32_t id, double x, double y, double r, double z = 0.0)
{
  Reg g;
  g.track_id = id; g.x = x; g.y = y; g.z = z; g.radius_m = r;
  return g;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// ★ 核心语义：落区优先于就近
// ═══════════════════════════════════════════════════════════════════════════

TEST(AssociateTest, 落区优先于就近)
{
  // 布局：track 1 在 (0,0)（**滤波估计**出来的位置，就在观测旁边）；
  //       track 2 在 (5,5)，它有一个**已知区域** 半径 1.0。
  // 观测落在 (5.2, 5.0)：离 track 1 有 7.2m（远超门限），落在 track 2 的区域内。
  // ⇒ 必须认 **track 2**（区域是场地事实）。
  // ⚠️ 这条本身不稀奇；稀奇的是下一条（观测离 A 更近、却仍要认 B）。
  TA::Params p;
  std::vector<Obs> o{obs(5.2, 5.0)};
  std::vector<Trk> t{trk(1, 0.0, 0.0), trk(2, 5.0, 5.0)};
  std::vector<Reg> g{reg(2, 5.0, 5.0, 1.0)};

  auto m = TA::associate(o, t, g, p);
  ASSERT_EQ(m.size(), 1u);
  EXPECT_EQ(m[0].track_index, 1) << "应认领已知区域指向的 track 2（下标 1）";
  EXPECT_TRUE(m[0].by_region) << "应标成落区认领（①级）";
}

TEST(AssociateTest, 观测离A更近但落在B的已知区域内时必须认B)
{
  // ⚠️⚠️ **这条是全文件最贵的** —— 它正是"查表"与"纯就近"的分野。
  //    track 10 在 (5.0, 5.0)，观测在 (5.6, 5.0)：距离 0.6m；
  //    track 20 在 (5.6, 5.0) **恰好就是观测位置**，且带已知区域半径 0.3。
  //    两个 track 都在门限内、track 20 距离为 0 ⇒ 纯就近必配 20（碰巧也对）。
  //    ⇒ 换个布局：让 track 10 **比** 20 更近，但观测落在 20 的区域内。
  //      track 10 在 (5.9, 5.0)（距观测 0.3m）；track 20 在 (5.0, 5.0)，
  //      区域半径 1.2 ⇒ 观测 (5.6,5.0) 距 20 有 0.6m，**但落在它的区域内**。
  //      纯就近会配给 10（0.3 < 0.6），正确实现必须配给 **20**。
  TA::Params p;
  std::vector<Obs> o{obs(5.6, 5.0)};
  std::vector<Trk> t{trk(10, 5.9, 5.0), trk(20, 5.0, 5.0)};
  std::vector<Reg> g{reg(20, 5.0, 5.0, 1.2)};

  auto m = TA::associate(o, t, g, p);
  ASSERT_EQ(m.size(), 1u);
  EXPECT_EQ(m[0].track_index, 1)
      << "落区(20, 下标1) 必须赢过更近的 10(下标0) —— 否则建筑位归属会串位";
  EXPECT_TRUE(m[0].by_region);
}

TEST(AssociateTest, 敌方机器人不被建筑位认领)
{
  // ⚠️⚠️ **2026-09-25 实测抓出来的**：
  //    一个 ENEMY_ROBOT(3) 位于 (3.0, 3.0)，建筑位在 (2.8, 2.8)
  //    ⇒ 距离 0.283m < 0.35m ⇒ **原先**被那个**静态建筑位**认领走了
  //    ⇒ 它不再是移动目标 ⇒ **BR_RESET 也清不掉它**。
  //    ⇒ 落区认领必须**按类型门控**：静态位不认领会动的目标。
  TA::Params p;
  Obs enemy = obs(3.0, 3.0);
  enemy.type = 3;                     // ENEMY_ROBOT
  std::vector<Obs> o{enemy};
  std::vector<Trk> t{trk(0, 2.8, 2.8, 0.6)};
  std::vector<Reg> g{reg(0, 2.8, 2.8, 0.35, 0.6)};

  auto m = TA::associate(o, t, g, p);
  ASSERT_EQ(m.size(), 1u);
  EXPECT_FALSE(m[0].by_region) << "会动的目标不该被静态已知位置认领";
}

TEST(AssociateTest, 方块仍可被建筑位认领)
{
  // 反向：**静态**类型（地球/天空方块、穆斯蒂卡、未知）照样走落区认领
  TA::Params p;
  for (std::uint8_t ty : {0, 1, 2, 4}) {
    Obs blk = obs(2.9, 2.9);
    blk.type = ty;
    std::vector<Obs> o{blk};
    std::vector<Trk> t{trk(0, 2.8, 2.8, 0.6)};
    std::vector<Reg> g{reg(0, 2.8, 2.8, 0.35, 0.6)};
    auto m = TA::associate(o, t, g, p);
    EXPECT_TRUE(m[0].by_region) << "type=" << static_cast<int>(ty) << " 该被认领";
  }
}


TEST(AssociateTest, 已知位置的目标不参与就近指派)
{
  // ⚠️⚠️ **2026-09-25 端到端实测抓出的第二半**：
  //    一个敌方机器人位于 (7.0, 7.0)，L2 建筑位在 (6.75, 6.75)
  //    ⇒ 距离 0.354m —— **没进 0.35m 的认领区，却进了 0.6m 的就近门限**
  //    ⇒ 被就近指派给了那个静态建筑位 ⇒ 它不再是移动目标 ⇒ BR_RESET 清不掉它。
  //    ⇒ 已知位置的目标**只接受落区认领**。
  TA::Params p;
  Obs enemy = obs(7.0, 7.0);
  enemy.type = 3;
  std::vector<Obs> o{enemy};
  std::vector<Trk> t{trk(6, 6.75, 6.75, 0.9)};
  t[0].is_known_position = true;
  std::vector<Reg> g{reg(6, 6.75, 6.75, 0.35, 0.9)};   // 观测不在区域内

  auto m = TA::associate(o, t, g, p);
  ASSERT_EQ(m.size(), 1u);
  EXPECT_EQ(m[0].track_index, -1)
      << "已知位置的目标不该因为离得近就抢走一个会动的目标";
}

TEST(AssociateTest, 普通目标仍参与就近指派)
{
  // 反向：没有 is_known_position 的照旧走就近。
  // ⚠️ **z 要给对**：`distance3d` 是**三维**的 —— 地面上和 L2 平台上的是**两个不同物体**。
  //    （这条与「落区只看 xy」不矛盾：那里判的是"在不在那个位上"，观测的 z 噪声大；
  //      这里判的是"是不是同一个物体"，z 是区分平台层的关键。）
  TA::Params p;
  std::vector<Obs> o{obs(7.0, 7.0, 0.9)};
  std::vector<Trk> t{trk(6, 6.75, 6.75, 0.9)};
  auto m = TA::associate(o, t, {}, p);
  EXPECT_EQ(m[0].track_index, 0);

  // z 差一层 ⇒ 不该配上（哪怕 xy 只差 0.35m）
  std::vector<Obs> o2{obs(7.0, 7.0, 0.0)};   // 地面上的
  auto m2 = TA::associate(o2, t, {}, p);
  EXPECT_EQ(m2[0].track_index, -1) << "跨平台层不该被当成同一个物体";
}

TEST(AssociateTest, 没有已知区域时退化为就近)
{
  TA::Params p;
  std::vector<Obs> o{obs(3.1, 0.0)};
  std::vector<Trk> t{trk(1, 0.0, 0.0), trk(2, 3.0, 0.0)};
  auto m = TA::associate(o, t, {}, p);      // 空 known
  ASSERT_EQ(m.size(), 1u);
  EXPECT_EQ(m[0].track_index, 1);
  EXPECT_FALSE(m[0].by_region);
}

// ═══════════════════════════════════════════════════════════════════════════
// 门限
// ═══════════════════════════════════════════════════════════════════════════

TEST(AssociateTest, 超出门限不关联)
{
  // ⚠️ 超出门限**不硬塞**给最近的 track —— 该观测要去建新候选。
  //    硬塞的后果是"噪声污染已有目标的位置"，而那看不出来。
  TA::Params p;             // gate_m 默认 0.60
  std::vector<Obs> o{obs(10.0, 0.0)};
  std::vector<Trk> t{trk(1, 0.0, 0.0)};
  auto m = TA::associate(o, t, {}, p);
  ASSERT_EQ(m.size(), 1u);
  EXPECT_EQ(m[0].track_index, -1) << "超门限必须不匹配";
}

TEST(AssociateTest, 边界恰好在门限上算匹配)
{
  // ⚠️ 边界是**非严格**（<=）。这里同时钉住"恰好等于"这个语义，
  //    免得将来有人改成 < 而没人发现。
  TA::Params p;
  p.gate_m = 0.60;
  std::vector<Obs> o{obs(0.60, 0.0)};
  std::vector<Trk> t{trk(1, 0.0, 0.0)};
  auto m = TA::associate(o, t, {}, p);
  EXPECT_EQ(m[0].track_index, 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 结构性质
// ═══════════════════════════════════════════════════════════════════════════

TEST(AssociateTest, 返回与观测等长同序)
{
  // ⚠️ 调用方按 obs[i] 直接取 result[i] —— 若实现改成"只返回匹配上的"，
  //    调用方就得自己判断哪条没配上，那是**漏建候选 track** 的温床。
  TA::Params p;
  std::vector<Obs> o{obs(0.0, 0.0), obs(999.0, 0.0), obs(0.1, 0.0)};
  std::vector<Trk> t{trk(1, 0.0, 0.0)};
  auto m = TA::associate(o, t, {}, p);
  ASSERT_EQ(m.size(), o.size());
  EXPECT_EQ(m[0].obs_index, 0);
  EXPECT_EQ(m[1].obs_index, 1);
  EXPECT_EQ(m[2].obs_index, 2);
  EXPECT_EQ(m[1].track_index, -1) << "中间那条超门限";
}

TEST(AssociateTest, 一个追踪最多被认领一次)
{
  // 两条观测都在同一个 track 的门限内 ⇒ 只能配上一个，另一个必须 -1（去建新候选）
  TA::Params p;
  std::vector<Obs> o{obs(0.0, 0.0), obs(0.1, 0.0)};
  std::vector<Trk> t{trk(1, 0.0, 0.0)};
  auto m = TA::associate(o, t, {}, p);
  int hits = 0;
  for (const auto& x : m) if (x.track_index == 0) ++hits;
  EXPECT_EQ(hits, 1) << "一对一：同一个 track 不能被两条观测同时认领";
}

TEST(AssociateTest, 最近的先配)
{
  // 两个 track、两条观测：必须配成 (近↔近)(远↔远)，不是 (0↔0)(1↔1) 的盲配
  TA::Params p;
  p.gate_m = 5.0;
  std::vector<Obs> o{obs(0.1, 0.0), obs(2.0, 0.0)};
  std::vector<Trk> t{trk(1, 2.1, 0.0), trk(2, 0.0, 0.0)};
  auto m = TA::associate(o, t, {}, p);
  EXPECT_EQ(m[0].track_index, 1) << "观测 0 应配 track 2（下标 1，距离 0.1）";
  EXPECT_EQ(m[1].track_index, 0) << "观测 1 应配 track 1（下标 0，距离 0.1）";
}

TEST(AssociateTest, 空输入不崩且返回等长空表)
{
  TA::Params p;
  auto a = TA::associate({}, {}, {}, p);
  EXPECT_TRUE(a.empty());
  auto b = TA::associate({obs(0, 0)}, {}, {}, p);      // 有观测无追踪
  ASSERT_EQ(b.size(), 1u);
  EXPECT_EQ(b[0].track_index, -1);
}

TEST(AssociateTest, 同一份输入永远同一份输出)
{
  // ⚠️ 确定性：代价相等时必须有稳定的 tie-break，否则"偶发串位"没法复现。
  //    这条用同一份输入跑两次比结果 —— 保证贪心排序里没有依赖未定义顺序的地方。
  TA::Params p;
  p.gate_m = 10.0;
  std::vector<Obs> o{obs(1.0, 0.0), obs(1.0, 0.0)};    // 两条**完全相同**的观测
  std::vector<Trk> t{trk(1, 1.0, 0.0), trk(2, 1.0, 0.0)};
  auto a = TA::associate(o, t, {}, p);
  auto b = TA::associate(o, t, {}, p);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].track_index, b[i].track_index);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 落区的细节
// ═══════════════════════════════════════════════════════════════════════════

TEST(AssociateTest, 落区只看水平xy不看z)
{
  // ⚠️ 头文件写明"刻意不判 z"：雷达在斜视角下**对 z 的估计最差**，
  //    带上 z 会让"明明就在那个位上"的观测频繁认领失败。
  TA::Params p;
  std::vector<Obs> o{obs(5.0, 5.0, 99.0)};     // z 离谱
  std::vector<Trk> t{trk(1, 5.0, 5.0, 0.0)};
  std::vector<Reg> g{reg(1, 5.0, 5.0, 0.5, 0.0)};
  auto m = TA::associate(o, t, g, p);
  EXPECT_EQ(m[0].track_index, 0) << "z 差得离谱仍应认领（只判 xy）";
  EXPECT_TRUE(m[0].by_region);
}

TEST(AssociateTest, 落区边界恰好等于半径算在内)
{
  TA::Params p;
  std::vector<Obs> o{obs(5.5, 5.0)};           // 距中心正好 0.5
  std::vector<Trk> t{trk(1, 5.0, 5.0)};
  std::vector<Reg> g{reg(1, 5.0, 5.0, 0.5)};
  auto m = TA::associate(o, t, g, p);
  EXPECT_TRUE(m[0].by_region) << "边界是非严格(<=)";
}

TEST(AssociateTest, 区域半径为零时用兜底半径)
{
  TA::Params p;
  p.default_region_m = 0.35;
  std::vector<Obs> o{obs(5.2, 5.0)};           // 距中心 0.2 < 0.35
  std::vector<Trk> t{trk(1, 5.0, 5.0)};
  std::vector<Reg> g{reg(1, 5.0, 5.0, 0.0)};   // radius_m = 0 ⇒ 用兜底
  auto m = TA::associate(o, t, g, p);
  EXPECT_TRUE(m[0].by_region);
}

TEST(AssociateTest, 区域指向的目标不在追踪列表里时跳过)
{
  // ⚠️ 不凭空造 track —— 建新候选是调用方的事。
  //    若这里硬造一个 index，调用方会拿它去索引 tracks[] ⇒ 越界。
  TA::Params p;
  std::vector<Obs> o{obs(5.0, 5.0)};
  std::vector<Trk> t{trk(1, 0.0, 0.0)};        // 没有 id=99 的
  std::vector<Reg> g{reg(99, 5.0, 5.0, 1.0)};
  auto m = TA::associate(o, t, g, p);
  ASSERT_EQ(m.size(), 1u);
  EXPECT_EQ(m[0].track_index, -1);
  EXPECT_FALSE(m[0].by_region);
}

// ═══════════════════════════════════════════════════════════════════════════
// 「v1.3」持握目标
// ═══════════════════════════════════════════════════════════════════════════

TEST(AssociateTest, 持握目标拒绝雷达观测)
{
  // ⚠️ 理由（头文件）：雷达点云与夹爪粘连、或方块不在建筑位上方被
  //    building_spot_analyzer 忽略 ⇒ 拿雷达去"纠正"它只会把它拽错。
  TA::Params p;
  std::vector<Obs> o{obs(0.1, 0.0, 0.0, kLidar)};
  std::vector<Trk> t{trk(1, 0.0, 0.0, 0.0, /*held=*/true)};
  auto m = TA::associate(o, t, {}, p);
  EXPECT_EQ(m[0].track_index, -1) << "持握目标不该被雷达关联上";
}

TEST(AssociateTest, 持握目标仍接受相机观测)
{
  // 规格：「高置信度视觉检测但无雷达点位匹配」是**持握的判据本身** ⇒ 相机必须能配
  TA::Params p;
  std::vector<Obs> o{obs(0.1, 0.0, 0.0, kCamera)};
  std::vector<Trk> t{trk(1, 0.0, 0.0, 0.0, /*held=*/true)};
  auto m = TA::associate(o, t, {}, p);
  EXPECT_EQ(m[0].track_index, 0);
}

TEST(AssociateTest, 关掉持握门控后雷达也能配)
{
  TA::Params p;
  p.hold_rejects_lidar = false;
  std::vector<Obs> o{obs(0.1, 0.0, 0.0, kLidar)};
  std::vector<Trk> t{trk(1, 0.0, 0.0, 0.0, true)};
  auto m = TA::associate(o, t, {}, p);
  EXPECT_EQ(m[0].track_index, 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 小工具
// ═══════════════════════════════════════════════════════════════════════════

TEST(AssociateTest, 三维距离)
{
  EXPECT_DOUBLE_EQ(TA::distance3d(0, 0, 0, 3, 4, 0), 5.0);
  EXPECT_DOUBLE_EQ(TA::distance3d(1, 1, 1, 1, 1, 1), 0.0);
  EXPECT_DOUBLE_EQ(TA::distance3d(0, 0, 0, 0, 0, 2.5), 2.5);
}

TEST(AssociateTest, 来源名字)
{
  EXPECT_STREQ(TA::source_name(kLidar), "lidar");
  EXPECT_STREQ(TA::source_name(kCamera), "camera");
  EXPECT_STREQ(TA::source_name(0), "none");
  EXPECT_STREQ(TA::source_name(77), "unknown");
  EXPECT_NE(TA::source_name(77), nullptr);   // ⚠️ 调用方拿去 %s，不能是 nullptr
}

}  // namespace fusion
}  // namespace br_perception
