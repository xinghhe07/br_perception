/// ═══════════════════════════════════════════════════════════════════════════════
/// test_kalman_tracker.cpp — 模块 18 多目标追踪：**纯函数**单元测试
///
/// ⚠️ `KalmanTracker` 不碰 ROS（无节点、无话题）⇒ 直接喂。**发布归模块 21**。
///
/// ── 为什么这个模块必须有单测（**它错得最看不出来**）──
///   卡尔曼滤波错了**不会报错**：增益算错、噪声矩阵量纲错、dt 用错，
///   症状全是"**看起来在跟，但跟得不对**"—— 残差偏大、滞后、或不收敛。
///   而这些在 RViz 里**都是一条平滑的轨迹**，肉眼分不出对错。
///   ⇒ 本文件的价值在于**用闭式解手算的那几条**：
///     · 恒速运动 ⇒ 新息必须趋近 0（模型对了就没有残差）
///     · 静止目标 ⇒ 速度估计必须收敛到 0（不会凭空长出速度）
///     · 噪声矩阵 ⇒ 量纲（dt⁴/dt³/dt²）与对称性
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=KfMathTest.*`）。
/// ⚠️ 全部用例确定性：手造状态、无时间、无随机 —— 卡尔曼的收敛性用**固定次数的迭代**验证，
///    不靠"跑一会儿看看"。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/fusion/kalman_tracker.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace br_perception {
namespace fusion {

using KT = KalmanTracker;
using V6 = KalmanTracker::Vec6;
using M6 = KalmanTracker::Mat6;

namespace {

constexpr std::uint8_t kLidar = 1;

/// 造一个"位置已知、速度未知"的初值（与 `add_track` 同款）
void init_state(V6& x, M6& P, double px, double py, double pz, const KT::Params& p)
{
  x = V6::Zero();
  x(0) = px; x(1) = py; x(2) = pz;
  P = M6::Zero();
  P(0,0) = P(1,1) = P(2,2) = p.sigma_z * p.sigma_z;
  P(3,3) = P(4,4) = P(5,5) = 100.0;
}

/// 协方差是否对称且对角线非负（数值健康度）
bool cov_healthy(const M6& P)
{
  if ((P - P.transpose()).cwiseAbs().maxCoeff() > 1e-9) return false;
  for (int i = 0; i < 6; ++i) {
    if (!(P(i, i) >= 0.0)) return false;
    if (std::isnan(P(i, i))) return false;
  }
  return true;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 矩阵：量纲与结构
// ═══════════════════════════════════════════════════════════════════════════

TEST(KfMathTest, 状态转移矩阵把dt放在位置对速度块上)
{
  // 恒速模型：位置按 dt 外推、速度不变
  const M6 F = KT::transition(0.5);
  EXPECT_DOUBLE_EQ(F(0, 3), 0.5);
  EXPECT_DOUBLE_EQ(F(1, 4), 0.5);
  EXPECT_DOUBLE_EQ(F(2, 5), 0.5);
  // ⚠️ **反向**（速度受位置影响）在物理上就是错的，但矩阵长得差不多、不会报错
  EXPECT_DOUBLE_EQ(F(3, 0), 0.0) << "速度不该被位置影响 —— 写反了不会报错，只会让它发散";
  EXPECT_DOUBLE_EQ(F(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(F(3, 3), 1.0);
}

TEST(KfMathTest, 过程噪声的量纲是dt4_dt3_dt2)
{
  // ⚠️⚠️ **这条钉的是量纲** —— 写成"看着差不多"的 dt²/dt/dt 不会报错，
  //    只会让滤波器悄悄失谐（过度平滑或过度信任观测）。
  const double dt = 2.0, sa = 3.0;
  const M6 Q = KT::process_noise(dt, sa);
  const double q = sa * sa;
  EXPECT_DOUBLE_EQ(Q(0, 0),     q * std::pow(dt, 4) / 4.0);   // 位置块 dt⁴
  EXPECT_DOUBLE_EQ(Q(3, 3),     q * std::pow(dt, 2));         // 速度块 dt²
  EXPECT_DOUBLE_EQ(Q(0, 3),     q * std::pow(dt, 3) / 2.0);   // 交叉项 dt³
}

TEST(KfMathTest, 过程噪声必须对称)
{
  // ⚠️ 漏了交叉项的镜像 ⇒ Q 不对称 ⇒ S 求逆出 NaN，
  //    而症状只是"某个 track 突然跳到 NaN 位置"，很难反查到矩阵上。
  const M6 Q = KT::process_noise(0.033, 2.0);
  EXPECT_NEAR((Q - Q.transpose()).cwiseAbs().maxCoeff(), 0.0, 1e-12);
}

TEST(KfMathTest, 观测矩阵只看位置不看速度)
{
  const auto H = KT::observation();
  EXPECT_DOUBLE_EQ(H(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(H(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(H(2, 2), 1.0);
  EXPECT_DOUBLE_EQ(H(0, 3), 0.0) << "速度不该被直接观测 —— 它只能从位置序列里估";
  EXPECT_DOUBLE_EQ(H(0, 4), 0.0);
  // ⚠️ 先取出来再断言 —— `H.block<3, 3>(...)` 里的逗号**不在括号内**，
  //    直接写进 `EXPECT_*` 会被预处理器当成参数分隔符（编不过，报 "passed 3 arguments"）。
  const double hI_sum = H.block<3, 3>(0, 0).cwiseAbs().sum();
  EXPECT_DOUBLE_EQ(hI_sum, 3.0);
}

TEST(KfMathTest, 观测噪声是各向同性的)
{
  const auto R = KT::measurement_noise(0.05);
  EXPECT_DOUBLE_EQ(R(0, 0), 0.0025);
  EXPECT_DOUBLE_EQ(R(0, 1), 0.0);
  EXPECT_DOUBLE_EQ(R(2, 2), 0.0025);
}

// ═══════════════════════════════════════════════════════════════════════════
// ★ 滤波器行为：用闭式解手算的那几条
// ═══════════════════════════════════════════════════════════════════════════

TEST(KfMathTest, 预测按速度外推且速度不变)
{
  KT::Params p;
  V6 x; M6 P;
  init_state(x, P, 1.0, 2.0, 3.0, p);
  x(3) = 1.0; x(4) = -2.0; x(5) = 0.0;      // 已知速度

  KT::predict(x, P, 0.5, p);
  EXPECT_NEAR(x(0), 1.0 + 1.0 * 0.5, 1e-12);   // 1 + vx·dt
  EXPECT_NEAR(x(1), 2.0 + (-2.0) * 0.5, 1e-12);
  EXPECT_NEAR(x(2), 3.0, 1e-12);
  EXPECT_DOUBLE_EQ(x(3), 1.0) << "恒速模型下速度不因预测而变";
  EXPECT_DOUBLE_EQ(x(4), -2.0);
}

TEST(KfMathTest, 恒速目标的新息收敛到零)
{
  // ⚠️⚠️ **最贵的一条**。
  //    物理依据：如果目标**真的**做恒速运动，而模型也是恒速的 ⇒ **模型对了就没有残差**。
  //    喂 N 帧真值后，预测位置必须落在真值上（新息 ≈ 0）。
  //    这条同时验证了 F/Q/H/R 四个矩阵的量纲和用法是**自洽的** ——
  //    任何一个量纲错，新息都不会趋近 0（要么滞后、要么过冲）。
  KT::Params p;
  const double dt = 0.1, vx = 1.5, vy = -0.5;
  V6 x; M6 P;
  init_state(x, P, 0.0, 0.0, 0.0, p);

  double t = 0.0, tx = 0.0, ty = 0.0;
  for (int i = 0; i < 60; ++i) {
    t += dt; tx += vx * dt; ty += vy * dt;
    KT::predict(x, P, dt, p);
    KT::update(x, P, tx, ty, 0.0, p);
    ASSERT_TRUE(cov_healthy(P)) << "第 " << i << " 步协方差失稳";
  }

  // 再预测一步，看它落在哪
  KT::predict(x, P, dt, p);
  const double true_x = tx + vx * dt, true_y = ty + vy * dt;
  EXPECT_NEAR(x(0), true_x, 0.01) << "恒速运动下预测位置应落在真值上";
  EXPECT_NEAR(x(1), true_y, 0.01);
  // 速度估计也应收敛到真值
  EXPECT_NEAR(x(3), vx, 0.05) << "速度估计应收敛到真速度";
  EXPECT_NEAR(x(4), vy, 0.05);
}

TEST(KfMathTest, 静止目标的速度估计收敛到零)
{
  // ⚠️ 反向验证：喂**同一位置** N 帧，速度估计必须趋近 0。
  //    若噪声矩阵量纲写错（比如速度块给了 dt⁴），滤波器会凭空长出速度，
  //    而症状是"静止的塔在抖"—— RViz 里看不出来是 bug。
  KT::Params p;
  V6 x; M6 P;
  init_state(x, P, 5.0, 5.0, 0.0, p);

  for (int i = 0; i < 60; ++i) {
    KT::predict(x, P, 0.1, p);
    KT::update(x, P, 5.0, 5.0, 0.0, p);
  }
  EXPECT_NEAR(x(0), 5.0, 0.01);
  EXPECT_NEAR(x(1), 5.0, 0.01);
  EXPECT_NEAR(std::abs(x(3)), 0.0, p.still_speed_eps) << "静止目标不该被估出速度";
  EXPECT_NEAR(std::abs(x(4)), 0.0, p.still_speed_eps);
  EXPECT_NEAR(std::abs(x(5)), 0.0, p.still_speed_eps);
}

TEST(KfMathTest, 更新后协方差应当变小)
{
  // 信息只会增加：观测之后位置的不确定度必须下降（这是卡尔曼的基本性质）
  KT::Params p;
  V6 x; M6 P;
  init_state(x, P, 0.0, 0.0, 0.0, p);
  const double before = P(0, 0);
  KT::update(x, P, 0.01, 0.0, 0.0, p);
  EXPECT_LT(P(0, 0), before);
}

TEST(KfMathTest, 协方差全程保持对称且非负)
{
  KT::Params p;
  V6 x; M6 P;
  init_state(x, P, 0.0, 0.0, 0.0, p);
  for (int i = 0; i < 200; ++i) {
    KT::predict(x, P, 0.05, p);
    KT::update(x, P, 0.1 * i, 0.0, 0.0, p);
    ASSERT_TRUE(cov_healthy(P)) << "第 " << i << " 步失稳";
  }
}

TEST(KfMathTest, dt为零时预测不动)
{
  // ⚠️ dt<=0 会让 Q 退化成零、F 变单位阵 —— 直接不动作，别往下算
  KT::Params p;
  V6 x; M6 P;
  init_state(x, P, 1.0, 1.0, 1.0, p);
  const V6 x0 = x;
  const M6 P0 = P;
  KT::predict(x, P, 0.0, p);
  EXPECT_EQ((x - x0).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_EQ((P - P0).cwiseAbs().maxCoeff(), 0.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 状态机
// ═══════════════════════════════════════════════════════════════════════════

TEST(StateMachineTest, 攒够命中才升确认)
{
  KT::Params p;
  p.confirm_frame_count = 3;
  // 第 1、2 帧还在 CANDIDATE（**不输出给下游**），第 3 帧才 CONFIRMED
  EXPECT_EQ(KT::step_state(KT::kCandidate, true, 1, 0, 0.0, p), KT::kCandidate);
  EXPECT_EQ(KT::step_state(KT::kCandidate, true, 2, 0, 0.0, p), KT::kCandidate);
  EXPECT_EQ(KT::step_state(KT::kCandidate, true, 3, 0, 0.0, p), KT::kConfirmed);
  EXPECT_EQ(KT::step_state(KT::kCandidate, true, 9, 0, 0.0, p), KT::kConfirmed);
}

TEST(StateMachineTest, 连续丢失达阈值判死)
{
  KT::Params p;
  p.delete_frame_count = 5;
  EXPECT_EQ(KT::step_state(KT::kConfirmed, false, 3, 4, 0.01, p), KT::kConfirmed)
      << "还没到 5 帧，应保持（仍在预测）";
  EXPECT_EQ(KT::step_state(KT::kConfirmed, false, 3, 5, 0.01, p), KT::kLost);
}

TEST(StateMachineTest, 时间闸优先级高于帧闸)
{
  // ⚠️ 帧率掉到 1fps 时，帧闸（丢 5 帧）要 5 秒才触发 —— 那时它还在算五分钟前的预测。
  //    时间闸必须独立生效：**只丢 1 帧**但已经过了 2 秒 ⇒ 直接判死。
  KT::Params p;             // max_prediction_age_sec 默认 0.5
  EXPECT_EQ(KT::step_state(KT::kConfirmed, false, 3, 1, 2.0, p), KT::kLost)
      << "时间闸没生效 ⇒ 低帧率下会输出基于陈旧预测的位置";
}

TEST(StateMachineTest, 命中会重置丢失计数由调用方保证)
{
  // ⚠️ 本函数**不看** miss_count 当命中时 —— 调用方必须在命中时把 miss_count 清零。
  //    这条把契约钉下来（若哪天想改成"函数内自己清"，得同时改这里）。
  KT::Params p;
  EXPECT_EQ(KT::step_state(KT::kConfirmed, true, 10, 0, 0.0, p), KT::kConfirmed);
}

TEST(StateMachineTest, 建筑位目标不立即删而是多留)
{
  KT::Params p;
  p.keep_on_spot_frames = 30;
  // 非建筑位：LOST 即可删
  EXPECT_TRUE(KT::is_dead(KT::kLost, false, 0, p));
  // 建筑位：LOST 后还要再留 keep_on_spot_frames 帧
  EXPECT_FALSE(KT::is_dead(KT::kLost, true, 0, p));
  EXPECT_FALSE(KT::is_dead(KT::kLost, true, 29, p));
  EXPECT_TRUE(KT::is_dead(KT::kLost, true, 30, p));
}

TEST(StateMachineTest, 未到LOST的都不算死)
{
  KT::Params p;
  EXPECT_FALSE(KT::is_dead(KT::kCandidate, false, 0, p));
  EXPECT_FALSE(KT::is_dead(KT::kConfirmed, false, 0, p));
}

// ═══════════════════════════════════════════════════════════════════════════
// 「v1.2」推移检测
// ═══════════════════════════════════════════════════════════════════════════

namespace {

std::vector<KT::Track::Sample> make_history(
    std::initializer_list<std::array<double, 4>> pts)
{
  std::vector<KT::Track::Sample> h;
  for (const auto& a : pts) h.push_back({a[0], a[1], a[2], a[3]});
  return h;
}

/// 生成 n 个点：水平匀速、z 恒定
std::vector<KT::Track::Sample> pushing_track(int n, double z = 0.0)
{
  std::vector<KT::Track::Sample> h;
  for (int i = 0; i < n; ++i) {
    h.push_back({0.1 * i, 0.2 * i, 0.0, z});
  }
  return h;
}

}  // namespace

TEST(PushingTest, z恒定且水平在动判为推移)
{
  KT::Params p;
  EXPECT_TRUE(KT::is_pushing(pushing_track(20), p));
}

TEST(PushingTest, z有跳变说明抬起过不是推移)
{
  // 抬起-放下：z 在中间跳起来 ⇒ 是搬运不是推移（规则 3.5.3 / 8.6.4 的分界）
  KT::Params p;
  auto h = pushing_track(20, 0.0);
  h[10].z = 0.30;                 // 抬起来 30cm
  EXPECT_FALSE(KT::is_pushing(h, p)) << "z 跳变过就不是推移";
}

TEST(PushingTest, 静止目标不是推移)
{
  // ⚠️⚠️ **最糟的误判**：静止目标 z 恒定、水平也不动。
  //    若只判"z 恒定"就下结论，每个静止的塔都会被标成"被推"。
  KT::Params p;
  std::vector<KT::Track::Sample> h;
  for (int i = 0; i < 20; ++i) h.push_back({0.1 * i, 5.0, 5.0, 0.0});  // 完全不动
  EXPECT_FALSE(KT::is_pushing(h, p)) << "静止 ≠ 被推";
}

TEST(PushingTest, 样本不足不下结论)
{
  // ⚠️ 样本太少 ⇒ 返回 false（**不当成推移**）。宁可漏报也不能误判搬运。
  KT::Params p;                    // pushing_min_samples 默认 8
  EXPECT_FALSE(KT::is_pushing(pushing_track(5), p));
  EXPECT_FALSE(KT::is_pushing({}, p));
}

TEST(PushingTest, 水平位移不够也不算)
{
  KT::Params p;
  std::vector<KT::Track::Sample> h;
  for (int i = 0; i < 20; ++i) h.push_back({0.1 * i, 5.0 + 0.001 * (i % 2), 5.0, 0.0});
  EXPECT_FALSE(KT::is_pushing(h, p)) << "抖动不算水平移动";
}

// ═══════════════════════════════════════════════════════════════════════════
// 管理
// ═══════════════════════════════════════════════════════════════════════════

TEST(ManageTest, 新建追踪的初值是位置已知速度未知)
{
  KT::Params p;
  KT t;
  const auto id = t.add_track(1.0, 2.0, 3.0, 0.5, p);
  ASSERT_NE(t.find(id), nullptr);
  const auto& tr = *t.find(id);
  EXPECT_EQ(tr.id, id);
  EXPECT_EQ(tr.state, static_cast<std::uint8_t>(KT::kCandidate));
  EXPECT_EQ(tr.hit_count, 1u);
  EXPECT_EQ(tr.miss_count, 0u);
  EXPECT_DOUBLE_EQ(tr.last_seen_sec, 0.5);
  EXPECT_DOUBLE_EQ(tr.kf_state(0), 1.0);
  EXPECT_DOUBLE_EQ(tr.kf_state(3), 0.0) << "速度初值给 0";
  // ⚠️ 但**协方差必须给大** —— 我们并不知道它速度多少。
  //    给 0 且协方差也小的话，滤波器会"坚信它不动"，前几帧严重滞后。
  EXPECT_GT(tr.kf_cov(3, 3), 10.0) << "速度初值协方差太小 ⇒ 滤波器会坚信它不动";
  EXPECT_EQ(tr.history.size(), 1u);
}

TEST(ManageTest, id单调递增且不复用)
{
  // ⚠️ 复用 id 会让下游把新旧目标混为一谈（追踪会"跳"到新目标上）
  KT::Params p;
  KT t;
  EXPECT_EQ(t.add_track(0, 0, 0, 0.0, p), 1u);
  EXPECT_EQ(t.add_track(0, 0, 0, 0.0, p), 2u);
  EXPECT_EQ(t.add_track(0, 0, 0, 0.0, p), 3u);
  EXPECT_EQ(t.next_id(), 4u);
}

TEST(ManageTest, 清移动目标时保留建筑位与指定类别)
{
  // 规则 5.3.2：已放置物体保持原位 ⇒ `on_known_spot` 的不许清
  KT::Params p;
  KT t;
  // ⚠️ `add_track` 返回 **id**（不再返回引用 —— vector 重分配会让引用悬空）
  const auto ia = t.add_track(0, 0, 0, 0.0, p); t.find(ia)->type = 1;
  const auto ib = t.add_track(1, 0, 0, 0.0, p); t.find(ib)->type = 4;
  const auto ic = t.add_track(2, 0, 0, 0.0, p); t.find(ic)->type = 2; t.find(ic)->on_known_spot = true;
  ASSERT_EQ(t.tracks().size(), 3u);

  const auto removed = t.clear_mobile_tracks({4});
  EXPECT_EQ(removed, 1u) << "只该清掉那个移动目标";
  ASSERT_EQ(t.tracks().size(), 2u);
  bool has_mustika = false, has_spot = false;
  for (const auto& x : t.tracks()) {
    if (x.type == 4) has_mustika = true;
    if (x.on_known_spot) has_spot = true;
  }
  EXPECT_TRUE(has_mustika) << "点名保留的类别不该被清";
  EXPECT_TRUE(has_spot) << "建筑位顶部目标不该被清（规则 5.3.2）";
}

TEST(ManageTest, 持握计数)
{
  KT::Params p;
  KT t;
  EXPECT_EQ(t.held_object_count(), 0u);
  t.add_track(0, 0, 0, 0.0, p);
  t.add_track(1, 0, 0, 0.0, p);
  t.find(1)->is_held = true;
  EXPECT_EQ(t.held_object_count(), 1u);
  t.find(2)->is_held = true;
  EXPECT_EQ(t.held_object_count(), 2u);
}

// ═══════════════════════════════════════════════════════════════════════════
// ★ 2026-09-25 代码复核后补的用例（每一条对应 review 里的一个真问题）
// ═══════════════════════════════════════════════════════════════════════════

TEST(ManageTest, add_track返回id后连加两个第一个仍可用)
{
  // ⚠️⚠️ **这条钉的是原实现的一个真 UB**：
  //    原 `add_track` 返回 `tracks_.back()` 的**引用**，而底层是 `std::vector` ——
  //    下一次 `push_back` 可能**重新分配** ⇒ 上一次的引用**悬空**。
  //    而"连加两个再回头用第一个"是**最自然的写法**。
  //    ⇒ 改成返回 id 之后，这个用例必须成立；哪天有人改回返回引用，这里会红。
  KT::Params p;
  KT t;
  const auto a = t.add_track(0.0, 0.0, 0.0, 0.0, p);
  const auto b = t.add_track(1.0, 1.0, 1.0, 0.1, p);      // ← 这行可能触发 realloc
  ASSERT_NE(a, b);
  ASSERT_NE(t.find(a), nullptr) << "加完第二个之后，第一个仍必须 find 得到";
  ASSERT_NE(t.find(b), nullptr);
  // ⚠️ 批量加，逼出重新分配（初始 reserve 是 64）
  std::vector<std::uint32_t> ids;
  for (int i = 0; i < 200; ++i) ids.push_back(t.add_track(0, 0, 0, 0.0, p));
  for (std::size_t i = 0; i < ids.size(); ++i) {
    ASSERT_NE(t.find(ids[i]), nullptr) << "第 " << i << " 个在批量插入后丢了";
  }
}

TEST(ManageTest, find找不到时返回空指针)
{
  KT::Params p;
  KT t;
  t.add_track(0, 0, 0, 0.0, p);
  EXPECT_EQ(t.find(999), nullptr);
  const KT& ct = t;
  EXPECT_EQ(ct.find(999), nullptr);
  EXPECT_NE(t.find(1), nullptr);
}

// ── ★ 已知位置常驻 track（2026-09-25 接线时发现的缺口）──
//    19 的 `KnownRegion::track_id` 要求指向**已存在的 track**，
//    而普通 `add_track` 的 id 来自跑着的计数器 ⇒ 对不上 ⇒ 落区认领一个也认不到。

TEST(ManageTest, 常驻track用指定id且状态直接确认)
{
  KT::Params p;
  KT t;
  // 建筑位编号 4（L1 共享位）
  const auto& tr = t.add_known_track(4, 5.5, 2.8, 0.6, 0.0, p);
  EXPECT_EQ(tr.id, 4u) << "id 必须是调用方指定的（= 建筑位编号）";
  EXPECT_EQ(tr.state, static_cast<std::uint8_t>(KT::kConfirmed))
      << "场地常量不需要靠观测确认 —— 走 CANDIDATE 会让它在确认窗口里不被输出";
  EXPECT_TRUE(tr.on_known_spot) << "它是常驻的，丢了也不该按 LOST 删";
  EXPECT_DOUBLE_EQ(tr.kf_state(0), 5.5);
  EXPECT_DOUBLE_EQ(tr.kf_state(1), 2.8);
  EXPECT_DOUBLE_EQ(tr.kf_state(2), 0.6);
}

TEST(ManageTest, 常驻track的速度协方差给极小)
{
  // ⚠️⚠️ **与普通 track 相反**：普通 track 是"位置已知、速度未知"（协方差给 100），
  //    而建筑位是**场地常量、压根不会动** —— 给它大协方差就等于允许滤波器把它拽走，
  //    那正是"塔归属串位"的另一种成因。
  KT::Params p;
  KT t;
  const auto& known = t.add_known_track(0, 1.0, 1.0, 0.6, 0.0, p);
  EXPECT_LT(known.kf_cov(3, 3), 1e-3) << "常驻 track 的速度不确定度必须极小";

  const auto id = t.add_track(2.0, 2.0, 0.0, 0.0, p);
  EXPECT_GT(t.find(id)->kf_cov(3, 3), 10.0) << "普通 track 相反：速度未知 ⇒ 协方差给大";
}

TEST(ManageTest, 常驻track把next_id抬到它之上)
{
  // ⚠️ 不抬的话，之后的普通 track 会撞号 —— 撞号 = 下游把两个目标混为一谈
  KT::Params p;
  KT t;
  t.add_known_track(9, 0, 0, 0, 0.0, p);          // 建 10 个位里的最后一个
  const auto id = t.add_track(1.0, 1.0, 0.0, 0.0, p);
  EXPECT_EQ(id, 10u) << "普通 track 的 id 必须跳过已用的 0~9";
  EXPECT_NE(t.find(9), nullptr);
  EXPECT_NE(t.find(10), nullptr);
}

TEST(ManageTest, 常驻track不会被清移动目标清掉)
{
  // 规则 5.3.2：已放置物体保持原位 ⇒ 建筑位上的常驻 track 不该被 clear 掉
  KT::Params p;
  KT t;
  t.add_known_track(0, 5.5, 2.8, 0.6, 0.0, p);
  const auto mob = t.add_track(1.0, 1.0, 0.0, 0.0, p);
  t.find(mob)->type = 1;
  ASSERT_EQ(t.tracks().size(), 2u);
  t.clear_mobile_tracks({});                      // 什么都不点名保留
  ASSERT_EQ(t.tracks().size(), 1u);
  EXPECT_NE(t.find(0), nullptr) << "常驻 track 不该被清";
  EXPECT_EQ(t.find(mob), nullptr);
}

TEST(ManageTest, 历史长度被真正截断)
{
  // ⚠️⚠️ **这条钉的是原实现的"注释在撒谎"**：`history_max` 写在类注释里说"有上限，
  //    不会无限长"，而**代码里没有任何一处执行它** ⇒ 长时间比赛 + 高帧率会让
  //    `history` **单调涨**（内存泄漏式的退化）。
  //    ⇒ 现在 `push_history` 执行它，这条用例保证将来不会被改回去。
  KT::Params p;
  KT t;
  const auto id = t.add_track(0, 0, 0, 0.0, p);
  auto* tr = t.find(id);
  ASSERT_NE(tr, nullptr);
  ASSERT_EQ(tr->history.size(), 1u);

  // 塞 3 倍上限
  const std::size_t n = KT::history_max * 3;
  for (std::size_t i = 0; i < n; ++i) {
    t.push_history(*tr, {0.01 * static_cast<double>(i), 0.0, 0.0, 0.0});
  }
  EXPECT_EQ(tr->history.size(), KT::history_max)
      << "history 没被截断 ⇒ 长时间跑会单调涨";
  // ⚠️ 而且要保留**最新的**那批（丢掉的是旧的）—— 判据是最后一条的时间戳
  EXPECT_DOUBLE_EQ(tr->history.back().t, 0.01 * static_cast<double>(n - 1));
}

TEST(PushingTest, 边界_z峰峰值恰好等于容差算通过)
{
  // ⚠️ 边界是**非严格**（`>` 才否决）⇒ 恰好等于容差**不否决**。
  //    原注释写了这条语义但**没有用例钉它** —— 补上。
  KT::Params p;                     // pushing_z_tol_m 默认 0.05
  auto h = pushing_track(20, 0.0);
  h[10].z = p.pushing_z_tol_m;      // 峰峰值正好 = 容差
  EXPECT_TRUE(KT::is_pushing(h, p)) << "<= 容差应通过；改成 < 会在这里红";

  h[10].z = p.pushing_z_tol_m * 1.0001;
  EXPECT_FALSE(KT::is_pushing(h, p)) << "超过容差一点就该否决";
}

TEST(PushingTest, 边界_水平位移恰好等于阈值不算通过)
{
  // ⚠️ 与上一条相反：水平位移是**严格大于**才通过。
  //    两条边界方向不同，**必须各自钉住** —— 否则"统一改成 <="这种改动不会被发现。
  KT::Params p;                     // pushing_min_horiz_m 默认 0.05
  std::vector<KT::Track::Sample> h;
  const int n = 20;
  const double step = p.pushing_min_horiz_m / (n - 1);   // 首尾位移正好 = 阈值
  for (int i = 0; i < n; ++i) h.push_back({0.1 * i, step * i, 5.0, 0.0});
  EXPECT_FALSE(KT::is_pushing(h, p)) << "恰好等于阈值不算通过（要求严格大于）";

  // 再加一点就通过
  h.back().y += p.pushing_min_horiz_m * 0.01;
  EXPECT_TRUE(KT::is_pushing(h, p));
}

TEST(KfMathTest, 量纲比值抓得出量纲写错)
{
  // ⚠️ 换个**独立**的写法钉量纲：位置块 / 速度块 = dt²/4。
  //    上面逐项钉的是绝对值，这条钉的是**比值** —— 两条互相独立，
  //    任一条单独都能抓出"dt⁴ 写成 dt²"这类错。
  for (double dt : {0.05, 1.0, 3.0}) {
    const M6 Q = KT::process_noise(dt, 2.0);
    EXPECT_NEAR(Q(0, 0) / Q(3, 3), dt * dt / 4.0, 1e-12)
        << "dt=" << dt << " 位置块/速度块的比值不对 ⇒ 量纲写错了";
    EXPECT_NEAR(Q(0, 3) / Q(3, 3), dt / 2.0, 1e-12) << "交叉项/速度块应是 dt/2";
  }
}

TEST(KfMathTest, 观测极准时协方差显著收缩)
{
  // 「喂同一位置 100 帧，P 显著收缩」—— 信息只会增加
  KT::Params p;
  V6 x; M6 P;
  init_state(x, P, 3.0, 3.0, 0.0, p);
  const double before = P(0, 0);
  for (int i = 0; i < 100; ++i) {
    KT::predict(x, P, 0.05, p);
    KT::update(x, P, 3.0, 3.0, 0.0, p);
  }
  EXPECT_LT(P(0, 0), before * 0.5) << "100 帧观测后 P 应显著收缩";
  EXPECT_GT(P(0, 0), 0.0);
  // ⚠️ 而且位置估计必须收敛到观测值
  EXPECT_NEAR(x(0), 3.0, 1e-3);
  EXPECT_NEAR(x(1), 3.0, 1e-3);
}

TEST(KfMathTest, 参数取零不产生NaN)
{
  // ⚠️⚠️ **这条钉的是"钳位"**（原实现没有）：
  //    · sigma_z = 0 ⇒ R 零矩阵 ⇒ S 可能奇异 ⇒ `S.solve()` 出 NaN，
  //      而 NaN 进了 x/P 就**再也回不来**；
  //    · sigma_a = 0 ⇒ Q 全零 ⇒ 速度不再被更新，退化成纯位置滤波。
  //    两者现在都在使用点被 `kSigmaFloor` 兜住。
  KT::Params p;
  p.sigma_z = 0.0;
  p.sigma_a = 0.0;
  V6 x; M6 P;
  init_state(x, P, 0.0, 0.0, 0.0, p);
  for (int i = 0; i < 50; ++i) {
    KT::predict(x, P, 0.1, p);
    KT::update(x, P, 0.1 * i, 0.0, 0.0, p);
    ASSERT_FALSE(std::isnan(x(0))) << "第 " << i << " 步出 NaN ⇒ 钳位没生效";
    ASSERT_FALSE(std::isnan(P(0, 0))) << "第 " << i << " 步 P 出 NaN";
  }
  EXPECT_TRUE(cov_healthy(P));
}

}  // namespace fusion
}  // namespace br_perception
