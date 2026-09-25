#include "br_perception/fusion/kalman_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace br_perception {
namespace fusion {

// ═══════════════════════════════════════════════════════════════════════════
// 滤波器矩阵
// ═══════════════════════════════════════════════════════════════════════════
KalmanTracker::Mat6 KalmanTracker::transition(double dt)
{
  // F = [[I, dt·I],
  //      [0,   I ]]
  // ⚠️ 恒速模型：位置按 dt 外推、**速度保持不变**。这正是"被追踪的目标不做机动"这条假设
  //    —— 它错了不会报错，只会表现为"转弯时跟丢/滞后"，而那要靠 `sigma_a` 调大去容忍。
  Mat6 F = Mat6::Identity();
  F(0, 3) = F(1, 4) = F(2, 5) = dt;
  return F;
}

KalmanTracker::Mat6 KalmanTracker::process_noise(double dt, double sigma_a)
{
  // Q = σa² · [[dt⁴/4·I, dt³/2·I],
  //            [dt³/2·I, dt²·I  ]]
  // ⚠️⚠️ **量纲必须对**：位置块 dt⁴、交叉项 dt³、速度块 dt²。
  //    写成"看着差不多"的 dt²/dt/dt **不会报错**，只会让滤波器**悄悄失谐**：
  //    要么过度平滑（跟不上机动）、要么过度相信观测（抖）。
  const double d2 = dt * dt, d3 = d2 * dt, d4 = d3 * dt;
  const double q = sigma_a * sigma_a;
  Mat6 Q = Mat6::Zero();
  for (int i = 0; i < 3; ++i) {
    Q(i, i)         = q * d4 / 4.0;
    Q(i + 3, i + 3) = q * d2;
    Q(i, i + 3)     = q * d3 / 2.0;
    Q(i + 3, i)     = q * d3 / 2.0;   // ⚠️ Q 必须**对称**，漏了这条 S 求逆会出 NaN
  }
  return Q;
}

Eigen::Matrix<double, 3, 6> KalmanTracker::observation()
{
  // H = [I₃ | 0₃] —— **只观测位置，不观测速度**（雷达/相机给的都是位置）
  // ⚠️ **缓存**：它**不依赖任何 track 状态**，每帧每 track 重算纯属浪费。
  //    函数内 static 的初始化在 C++11 起是**线程安全**的。
  //    ⚠️ 签名保持 `Matrix<3,6>`（按值返回）不变 —— 单测依赖它，改成 const& 是接口变更。
  static const Eigen::Matrix<double, 3, 6> kH = [] {
    Eigen::Matrix<double, 3, 6> h = Eigen::Matrix<double, 3, 6>::Zero();
    h.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    return h;
  }();
  return kH;
}

Eigen::Matrix3d KalmanTracker::measurement_noise(double sigma_z)
{
  return Eigen::Matrix3d::Identity() * (sigma_z * sigma_z);
}

namespace {

/// ⚠️ **参数下限钳位**（在使用点，不改默认值 —— 改默认值会误导读者）。
///
/// `sigma_z = 0` ⇒ `R` 是零矩阵 ⇒ `S` 可能奇异 ⇒ **`S.solve()` 出 NaN**，
/// 而 NaN 一旦进了 `x`/`P` 就**再也回不来**（后面全是 NaN）。
/// `sigma_a = 0` ⇒ `Q` 全零 ⇒ 滤波器认为模型绝对可信，**速度不再被更新**，退化成纯位置滤波。
/// ⇒ 两者都用这个下界兜住。⚠️ 用 `1e-6` 而不是 `0`：既防奇异，又不至于显著改变行为。
constexpr double kSigmaFloor = 1e-6;

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 预测 / 更新
// ═══════════════════════════════════════════════════════════════════════════
void KalmanTracker::predict(Vec6& x, Mat6& P, double dt, const Params& params)
{
  if (dt <= 0.0) return;    // ⚠️ dt<=0 会让 Q 退化、F 变单位阵 —— 直接不动，别往下算
  const Mat6 F = transition(dt);
  x = F * x;
  // ⚠️ 钳位（见 kSigmaFloor 的说明）：sigma_a=0 ⇒ Q 全零 ⇒ 速度不再被更新
  const double sa = std::max(params.sigma_a, kSigmaFloor);
  P = F * P * F.transpose() + process_noise(dt, sa);
  // ⚠️ **对称化**：上面两步的浮点误差会让 P 慢慢失去对称性，
  //    久了 S 求逆会出 NaN，而症状只是"某个 track 突然跳到 NaN 位置"。
  P = (P + P.transpose()) * 0.5;
}

void KalmanTracker::update(Vec6& x, Mat6& P,
                           double mx, double my, double mz,
                           const Params& params)
{
  const auto H  = observation();
  // ⚠️ 钳位（见 kSigmaFloor）：sigma_z=0 ⇒ R 零矩阵 ⇒ S 奇异 ⇒ 出 NaN，
  //    而 NaN 进了 x/P 就再也回不来。
  const double sz = std::max(params.sigma_z, kSigmaFloor);
  const auto R  = measurement_noise(sz);
  Eigen::Matrix<double, 3, 1> z(mx, my, mz);

  const Eigen::Matrix<double, 3, 1> y = z - H * x;          // 新息
  const Eigen::Matrix3d S = H * P * H.transpose() + R;      // 新息协方差

  // ⚠️ **用 LDLT 分解回代，不要 `S.inverse()`**（2026-09-25 代码复核意见）：
  //    P 刚初始化、或 dt 很大时 S 可能接近奇异，`inverse()` 会**放大误差甚至把 NaN
  //    送进状态**；分解回代最差只给最小二乘解，不会传播 NaN。
  // ⚠️ 维度推导：K = P·Hᵀ·S⁻¹ ⇒ Kᵀ = S⁻ᵀ·(P·Hᵀ)ᵀ = **S⁻¹·H·P**（S 对称）
  //    ⇒ K = (S⁻¹·H·P)ᵀ —— 这就是下面这一行，比先求逆再左乘少两次矩阵乘。
  const Eigen::Matrix<double, 6, 3> K = S.ldlt().solve(H * P).transpose();
  x = x + K * y;
  P = (Mat6::Identity() - K * H) * P;
  P = (P + P.transpose()) * 0.5;                            // 同上，对称化
}

// ═══════════════════════════════════════════════════════════════════════════
// 状态机
// ═══════════════════════════════════════════════════════════════════════════
std::uint8_t KalmanTracker::step_state(std::uint8_t cur, bool hit,
                                       std::uint32_t hit_count,
                                       std::uint32_t miss_count,
                                       double age_sec,
                                       const Params& params)
{
  // ⚠️ 刻意**不接收 `on_known_spot`**（原签名里有，2026-09-25 按代码复核意见删掉）：
  //    留一个用不到的参数会让调用方误以为它影响状态机 —— 那是"注释在撒谎"的另一种形式。
  //    "丢了没丢"是事实，"要不要因此删掉它"是策略（在 `is_dead()` 里）。
  if (hit) {
    // 攒够连续命中才确认。⚠️ 判据是 `hit_count`（**帧数**），不是"距上次多久"（时间）——
    //    后者会让语义随帧率变。
    return (hit_count >= static_cast<std::uint32_t>(params.confirm_frame_count))
               ? kConfirmed : kCandidate;
  }

  // ── 丢失 ──
  // ⚠️ **时间闸**：超过 `max_prediction_age_sec` 直接判死，**不管丢了几帧**。
  //    它防的是"帧率掉到 1fps 时，它还在算五分钟前的预测"—— 帧闸挡不住这种情况。
  if (age_sec > params.max_prediction_age_sec) return kLost;

  // ⚠️ **帧闸**：连续丢失达到阈值判死。对 CANDIDATE 与 CONFIRMED **一视同仁** ——
  //    规格只给了这一条丢失规则，而且 CANDIDATE 本来就不输出，多留几帧无害。
  if (miss_count >= static_cast<std::uint32_t>(params.delete_frame_count)) return kLost;

  return cur;   // 保持（CONFIRMED 还在预测，位置靠 `predict` 外推）
}

bool KalmanTracker::is_dead(std::uint8_t state, bool on_known_spot,
                            int kept_on_spot_frames, const Params& params)
{
  if (state != kLost) return false;
  // ⚠️⚠️ **`on_known_spot` 只在这里起作用，不在 `step_state` 里** —— 这是有意的：
  //    状态机该判"丢了"就判"丢了"（那是**事实**），
  //    "**要不要因此删掉它**"是另一件事（那是**策略**）。两件事混在一起写，
  //    将来想改保留策略就得动状态机，容易把事实也改错。
  if (!on_known_spot) return true;
  return kept_on_spot_frames >= params.keep_on_spot_frames;
}

// ═══════════════════════════════════════════════════════════════════════════
// 「v1.2」推移检测
// ═══════════════════════════════════════════════════════════════════════════
bool KalmanTracker::is_pushing(const std::vector<Track::Sample>& history,
                               const Params& params)
{
  // ⚠️ **样本不足 ⇒ 不下结论（返回 false）**，不是"当成推移"。
  //    宁可漏报也不能把搬运误判成推移 —— 那是规则 3.5.3 / 8.6.4 的分界。
  if (history.size() < static_cast<std::size_t>(std::max(2, params.pushing_min_samples))) {
    return false;
  }

  // ① 窗口内 **z 的峰峰值** 必须小于容差（= 没被抬起放下）
  double z_min = history.front().z, z_max = history.front().z;
  for (const auto& s : history) {
    z_min = std::min(z_min, s.z);
    z_max = std::max(z_max, s.z);
  }
  if ((z_max - z_min) > params.pushing_z_tol_m) return false;

  // ② 还要**确实在水平动**：首尾水平位移超过阈值。
  //    ⚠️ 缺了这条，一个**静止**目标（z 恒定、水平也不动）会被判成推移 —— 那是最糟的误判。
  const auto& a = history.front();
  const auto& b = history.back();
  const double dx = b.x - a.x, dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy) > params.pushing_min_horiz_m;
}

// ═══════════════════════════════════════════════════════════════════════════
// 管理
// ═══════════════════════════════════════════════════════════════════════════
std::uint32_t KalmanTracker::add_track(double mx, double my, double mz,
                                       double t_sec, const Params& params)
{
  // ⚠️ 顺手的 reserve：场上目标数是个位数级，省掉前几次 push_back 的 realloc。
  //    用 capacity 判而不是 empty 判 —— 清空过之后不必再 reserve。
  if (tracks_.capacity() < 64) tracks_.reserve(64);

  Track t;
  t.id = next_id_++;          // ⚠️ 单调递增、**不复用** —— 复用会让下游把新旧目标混为一谈
  t.state = static_cast<std::uint8_t>(kCandidate);   // ⚠️ 显式映射到 `Track.msg` 的 S_CANDIDATE，别写裸 0
  t.hit_count = 1;
  t.miss_count = 0;
  t.last_seen_sec = t_sec;

  t.kf_state = Vec6::Zero();
  t.kf_state(0) = mx; t.kf_state(1) = my; t.kf_state(2) = mz;
  // ⚠️ **速度先给 0，但初始协方差要给大** —— 我们并不知道它速度是多少。
  //    给 0 且协方差也给小的话，滤波器会"坚信它不动"，前几帧严重滞后。
  t.kf_cov = Mat6::Zero();
  const double sz_c = std::max(params.sigma_z, kSigmaFloor);   // 同 update 的钳位理由
  const double p_pos = sz_c * sz_c;
  const double p_vel = 100.0;                 // (m/s)² —— 大到"随便它动"
  t.kf_cov(0,0) = t.kf_cov(1,1) = t.kf_cov(2,2) = p_pos;
  t.kf_cov(3,3) = t.kf_cov(4,4) = t.kf_cov(5,5) = p_vel;

  // ⚠️ 走 `push_history`（本地的 t，move 之前）—— 让 `history_max` **真的被执行**，
  //    而不是只活在注释里。第一条当然不会超上限，但统一走同一条路，杜绝"某处漏了"。
  push_history(t, {t_sec, mx, my, mz});

  tracks_.push_back(std::move(t));   // ⚠️ move：`Track` 里有 `vector<Sample>`，省一次深拷贝
  // ⚠️⚠️ **返回 id，不返回 `tracks_.back()` 的引用**（2026-09-25 代码复核抓出）：
  //    底层是 vector，下一次 `push_back` 可能重新分配 ⇒ 上次返回的引用**悬空**。
  //    而"连加两个再回头用第一个"是最自然的写法 ⇒ 从接口上杜绝。
  return tracks_.back().id;
}

KalmanTracker::Track& KalmanTracker::add_known_track(std::uint32_t id,
                                                     double x, double y, double z,
                                                     double t_sec, const Params& params)
{
  if (tracks_.capacity() < 64) tracks_.reserve(64);

  Track t;
  t.id = id;
  // ⚠️ 状态**直接 CONFIRMED**：它是**场地常量**，不需要靠观测去确认。
  //    走 CANDIDATE 的话，前 confirm_frame_count 帧它**不会被输出**，
  //    而那段窗口里落区认领照样会把它认领上 —— 语义上说不通。
  t.state = static_cast<std::uint8_t>(kConfirmed);
  // ⚠️ **它是常驻的**：位置来自配置，丢了也不该按 LOST 删（「v1.2」规则）。
  t.on_known_spot = true;
  t.hit_count = 1;
  t.miss_count = 0;
  t.last_seen_sec = t_sec;

  t.kf_state = Vec6::Zero();
  t.kf_state(0) = x; t.kf_state(1) = y; t.kf_state(2) = z;

  // ⚠️⚠️ **与普通 track 相反**：位置与速度都给**极小**协方差。
  //    普通 track 是"位置已知、速度未知"（速度协方差给 100）；
  //    而这个是**场地常量、压根不会动** —— 给它大协方差就等于允许滤波器把它拽走，
  //    而那正是"塔归属串位"的另一种成因。
  t.kf_cov = Mat6::Zero();
  const double sz_c = std::max(params.sigma_z, kSigmaFloor);
  const double p_pos = sz_c * sz_c;
  const double p_vel = 1e-6;               // (m/s)² —— 它不动
  t.kf_cov(0,0) = t.kf_cov(1,1) = t.kf_cov(2,2) = p_pos;
  t.kf_cov(3,3) = t.kf_cov(4,4) = t.kf_cov(5,5) = p_vel;

  push_history(t, {t_sec, x, y, z});
  tracks_.push_back(std::move(t));

  // ⚠️ 把 `next_id_` 抬到它之上，免得之后的普通 track 撞号
  //    （撞号 = 下游把两个目标混为一谈、追踪"跳"过去）
  if (next_id_ <= id) next_id_ = id + 1;
  return tracks_.back();
}

KalmanTracker::Track* KalmanTracker::find(std::uint32_t id)
{
  for (auto& t : tracks_) {
    if (t.id == id) return &t;
  }
  return nullptr;   // ⚠️ 找不到返回 nullptr（不抛、不造）
}

const KalmanTracker::Track* KalmanTracker::find(std::uint32_t id) const
{
  for (const auto& t : tracks_) {
    if (t.id == id) return &t;
  }
  return nullptr;
}

void KalmanTracker::push_history(Track& track, const Track::Sample& sample)
{
  track.history.push_back(sample);
  // ⚠️⚠️ **这一句就是 `history_max` 的全部实现**（2026-09-25 代码复核抓出：
  //    原版把上限只写在注释里、代码里**没有任何一处执行它** ⇒ 长时间比赛 + 高帧率
  //    会让 `history` **单调涨**，是内存泄漏式的退化）。
  //    ⚠️ 调用方**必须走本函数**往 history 里加 —— 直接 `push_back` 会绕过上限。
  if (track.history.size() > history_max) {
    const auto drop = track.history.size() - history_max;
    track.history.erase(track.history.begin(),
                        track.history.begin() + static_cast<std::ptrdiff_t>(drop));
  }
}

std::size_t KalmanTracker::clear_mobile_tracks(const std::vector<std::uint8_t>& keep_types)
{
  const auto before = tracks_.size();
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [&keep_types](const Track& t) {
                       // ⚠️ 规则 5.3.2：**已放置物体保持原位** ⇒ 建筑位顶部目标不能清。
                       if (t.on_known_spot) return false;
                       // 调用方点名要保留的类别（穆斯蒂卡等）也不能清。
                       return std::find(keep_types.begin(), keep_types.end(), t.type)
                              == keep_types.end();
                     }),
      tracks_.end());
  return before - tracks_.size();
}

std::size_t KalmanTracker::held_object_count() const
{
  return static_cast<std::size_t>(
      std::count_if(tracks_.begin(), tracks_.end(),
                    [](const Track& t) { return t.is_held; }));
}

}  // namespace fusion
}  // namespace br_perception
