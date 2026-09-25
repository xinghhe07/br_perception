#pragma once

#include <Eigen/Dense>

#include <array>
#include <cstdint>
#include <vector>

namespace br_perception {
namespace fusion {

/**
 * @brief 多目标追踪：6 态恒速卡尔曼 + CANDIDATE/CONFIRMED/LOST 状态机（模块 18）
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ⚠️ **本模块是纯 C++ 库，不是节点**（与 19 `target_associator` 同款理由）
 *
 * 规格 §4.2 末行写着「发布 `/perception/tracks` 和 `/perception/tracks/markers`」——
 * **这两个发布归模块 21 `fusion_node`**。理由：21 是融合层唯一的节点
 * （你 2026-09-25 拍板「fusion_node 只发融合结果，pipeline 保有上行权」），
 * 由它统一发布，避免再多一个节点发同一批数据。**这是一处有意的偏差，记在 §八。**
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ⚠️⚠️ **本模块的"正确"极难从输出看出来** —— 这是它必须有单测的原因
 *
 * 卡尔曼滤波错了**不会报错**：增益算错、噪声矩阵量纲错、dt 用错，
 * 症状全是"**看起来在跟，但跟得不对**"——残差偏大、滞后、或者干脆不收敛。
 * 而这些在 RViz 里**都是一条平滑的轨迹**，肉眼分不出对错。
 * ⇒ 单测里最值钱的是那几条**能用闭式解手算**的：恒速运动必须零残差、静止目标速度必须收敛到 0。
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * 状态机（§4.2 原文）
 * ```
 *   CANDIDATE: 连续观测到 1 帧       → **不输出给下游**（等确认，防单帧噪声）
 *   CONFIRMED: 连续命中 confirm_frame_count 帧 → 确认，输出
 *   LOST:      连续丢失 delete_frame_count 帧  → 删除
 * ```
 * ⚠️ `hit_count` / `miss_count` 是**状态机的唯一判据原料** ——
 *    别拿"距上次观测多久"之类去凑（那会把帧数判据变成时间判据，
 *    而帧率一变语义就变了）。
 *
 * ⚠️ **「v1.2」建筑位顶部目标（天空方块）不受 LOST 删除约束**：
 *    它们的位置是**场地常量**，且"被翻走"是高频事件（规则 3.5.6-3.5.8）——
 *    按 LOST 删掉再重建，会让**归属在重建窗口里丢失**，而那正是 40/80 分的判据。
 *    ⇒ 标了 `on_known_spot` 的 track，在 LOST 后仍保留 `keep_on_spot_frames` 帧。
 */
class KalmanTracker
{
public:
  // ══════════════════════════════════════════════════════════════════════
  // 状态标记（与 `msg/Track.msg` 的 S_* 逐字对应）
  // ══════════════════════════════════════════════════════════════════════
  enum State : std::uint8_t {
    kCandidate = 0,
    kConfirmed = 1,
    kLost      = 2,
  };

  using Vec6 = Eigen::Matrix<double, 6, 1>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;

  struct Params
  {
    /// 预测步的 dt 上限（s）。⚠️ 超过它 ⇒ 该 track 的预测不可信 ⇒ 删除。
    /// ⚠️ 这是**时间**判据，与 `delete_frame_count`（**帧数**判据）是**两把不同的闸**：
    ///    帧闸防"连续丢帧"，时间闸防"帧率掉到 1fps 时它还在算五分钟前的预测"。
    double max_prediction_age_sec{0.5};
    /// 连续命中多少帧升 CONFIRMED（§4.2 的 `confirm_frame_count`）
    int confirm_frame_count{3};
    /// 连续丢失多少帧删（§4.2 的 `delete_frame_count`）
    int delete_frame_count{5};
    /// 「v1.2」`on_known_spot` 的 track 在 LOST 后**再多留**多少帧
    int keep_on_spot_frames{30};

    // ── 滤波器参数 ──
    /// 加速度过程噪声标准差 σa (m/s²)：**不是**"实际加速度"，是"模型允许它多不听话"。
    /// ⚠️ 调大的效果是"更信观测、跟得快但抖"；调小则"更信模型、平滑但滞后"。
    double sigma_a{2.0};
    /// 观测噪声标准差 σz (m)：雷达聚类的位置精度量级。
    double sigma_z{0.05};

    // ── 「v1.2」推移检测 ──
    /// 判定"z 基本恒定"的容差 (m)：整个窗口内 z 的**峰峰值**小于它才算"没被抬起"
    double pushing_z_tol_m{0.05};
    /// 至少要有这么多采样才敢判定（样本太少不下结论）
    int pushing_min_samples{8};
    /// 推移判定还要"确实在水平动"：窗口首尾的**水平位移**超过它 (m) 才算
    /// ⚠️ 没有它的话，一个**静止**目标（z 恒定、水平也不动）会被判成推移 ——
    ///    那是最糟的误判（静止 ≠ 被推）。
    double pushing_min_horiz_m{0.05};

    /// 速度小于它 ⇒ 认为静止（m/s）。用于"静止目标速度必须收敛到 0"那条判据。
    double still_speed_eps{0.05};
  };

  /// 一个候选/已确认目标
  struct Track
  {
    std::uint32_t id{0};
    std::uint8_t  state{kCandidate};
    std::uint8_t  type{0};            ///< 沿用 `Obstacle.msg` 的 ObstacleType
    std::uint8_t  last_update_src{0}; ///< 见 `Track.msg` 的 SRC_*
    float         confidence{0.0f};

    /// 「v1.2」位置在**已知建筑位顶部** ⇒ 不按 LOST 删（见类注释）
    bool on_known_spot{false};
    /// 「v1.3」持握：高置信度视觉但无雷达点位匹配 ⇒ 不丢弃、不强融合
    bool is_held{false};
    /// 「v1.2」己方 TR 候选（由 19 置位）
    bool is_ally_tr{false};
    /// 「v1.2」推移轨迹（见 `is_pushing`）
    bool suspected_pushing{false};

    std::uint32_t hit_count{0};       ///< 累计命中（**状态机判据原料**）
    std::uint32_t miss_count{0};      ///< **连续**丢失（同上；命中时清 0）
    double        last_seen_sec{0.0}; ///< 最后一次**被观测到**的时刻

    Vec6 kf_state{Vec6::Zero()};
    Mat6 kf_cov{Mat6::Identity()};

    /// 历史轨迹（诊断与推移检测用）。⚠️ 有上限（见 `history_max`），**不会无限长**。
    struct Sample { double t, x, y, z; };
    std::vector<Sample> history;
  };

  /// 历史轨迹保留上限。⚠️ 无限增长 = 长时间比赛下的**内存泄漏式**退化。
  static constexpr std::size_t history_max = 200;

  // ══════════════════════════════════════════════════════════════════════
  // 滤波器（**static 纯函数** —— 单测直接喂）
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 恒速模型的状态转移矩阵 `F = [[I, dt·I], [0, I]]`
  [[nodiscard]] static Mat6 transition(double dt);

  /// @brief 过程噪声 `Q`（恒速模型的**连续白噪声加速度**离散化）
  /// ```
  ///   Q = σa² · [[dt⁴/4·I, dt³/2·I],
  ///              [dt³/2·I, dt²·I  ]]
  /// ```
  /// ⚠️ **量纲必须对**：位置块是 dt⁴、速度块是 dt²、交叉项是 dt³。
  ///    写成 dt²/dt/dt 那种"看着差不多"的版本**不会报错**，只会让滤波器**悄悄失谐**。
  [[nodiscard]] static Mat6 process_noise(double dt, double sigma_a);

  /// @brief 观测矩阵 `H = [I₃ | 0₃]` —— **只观测位置，不观测速度**
  [[nodiscard]] static Eigen::Matrix<double, 3, 6> observation();

  /// @brief 观测噪声 `R = σz²·I₃`
  [[nodiscard]] static Eigen::Matrix3d measurement_noise(double sigma_z);

  /// @brief 预测步：`x ← Fx`、`P ← FPFᵀ + Q`
  static void predict(Vec6& x, Mat6& P, double dt, const Params& params);

  /// @brief 更新步：标准卡尔曼增益更新（只测位置）
  /// ⚠️ 用 **Joseph 形式**不必要（6 维、数值条件好），但**别省掉对称化** ——
  ///    浮点误差会让 P 慢慢失去对称性，久了 S 求逆出 NaN。
  static void update(Vec6& x, Mat6& P,
                     double mx, double my, double mz,
                     const Params& params);

  // ══════════════════════════════════════════════════════════════════════
  // 状态机（**static 纯函数**）
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 一步状态机推进
  /// @param cur         当前状态
  /// @param hit         本帧是否被观测到
  /// @param hit_count   推进后的累计命中数（调用方先自增）
  /// @param miss_count  推进后的**连续**丢失数（本帧命中时应为 0）
  /// @param age_sec     距上次被观测到的时长
  /// @param params      参数
  /// @return 推进后的状态；调用方据 `is_dead()` 决定是否删除
  ///
  /// ⚠️ **`age_sec` 的单位是「秒」，且必须是同一单调时钟的差值**（`now − last_seen_sec`）。
  ///    **函数内防不住单位错** —— 把毫秒当秒传进来会**静默**让时间闸早触发 1000 倍，
  ///    症状是"目标一丢失就被删"。调用方钉死这条约定。
  ///
  /// ⚠️ 本函数**不看** `on_known_spot`（原签名里有这个参数，2026-09-25 按代码复核意见删掉了）——
  ///    "丢了没丢"是**事实**，"要不要因此删掉它"是**策略**（在 `is_dead()` 里）。
  ///    ⚠️ 留一个**用不到的参数**会让调用方误以为它影响状态机 —— 那是"注释在撒谎"的另一种形式。
  [[nodiscard]] static std::uint8_t step_state(std::uint8_t cur, bool hit,
                                               std::uint32_t hit_count,
                                               std::uint32_t miss_count,
                                               double age_sec,
                                               const Params& params);

  /// @brief 该状态是否已到"该删除"
  /// @param kept_on_spot_frames 已经在 LOST 上多留了多少帧（见 `keep_on_spot_frames`）
  [[nodiscard]] static bool is_dead(std::uint8_t state, bool on_known_spot,
                                    int kept_on_spot_frames,
                                    const Params& params);

  // ══════════════════════════════════════════════════════════════════════
  // 「v1.2」推移轨迹检测
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 是否"疑似推移"：窗口内 **z 峰峰值 < 容差**（没被抬起放下）
  ///        且 **水平位移足够大**（确实在动）。
  ///
  /// ⚠️ 为什么要这个判据：规则 3.5.3 / 8.6.4 里"**搬运**"与"**推移**"的待遇不同，
  ///    而雷达只看得到轨迹。**抬起-放下**的轨迹在 z 上一定有跳变 ⇒ 这是雷达唯一能用的分界。
  /// ⚠️ **样本不足时返回 false**（不下结论），不是"当成推移" ——
  ///    宁可漏报也不能把搬运误判成推移。
  ///
  /// ⚠️⚠️ **判定窗口 = 传入的整个 `history`，本函数不做任何窗口截取。**
  ///    "窗口内 z 恒定"里的**窗口由调用方决定**：传 `track.history`（= 最近 `history_max` 条）
  ///    就是"最近 200 帧"，只传尾部若干条就是更短的窗。
  ///    ⇒ **判定精度直接取决于这个选择**，所以它是**调用方的契约**，不是本函数的内部细节。
  ///    （原注释只写"窗口内"，没说清是整个 history 还是最近 N 帧 —— 已按代码复核意见写明。）
  /// ⚠️ 边界是**非严格**：`z 峰峰值 == pushing_z_tol_m` **算通过**（不超）、
  ///    水平位移 `== pushing_min_horiz_m` **不算通过**（要求严格大于）。两条都有用例钉住。
  [[nodiscard]] static bool is_pushing(const std::vector<Track::Sample>& history,
                                       const Params& params);

  // ══════════════════════════════════════════════════════════════════════
  // 管理（**非 static，但不碰 ROS** —— 纯数据管理，仍可单测）
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 新建一个候选 track 并用观测初始化（速度先给 0，由后续帧估出来）
  /// @return **新 track 的 id**（**不是引用**）
  ///
  /// ⚠️⚠️ **为什么返回 id 而不是 `Track&`**（2026-09-25 代码复核抓出）：
  ///    底层是 `std::vector`，`push_back` 可能重新分配 ⇒ **上一次返回的引用会悬空**。
  ///    而"连加两个 track 再回头用第一个"是**最自然的写法**：
  ///    ```cpp
  ///    auto& a = tr.add_track(...);   // ⚠️ 危险写法
  ///    tr.add_track(...);             //    这行之后 a 已失效
  ///    a.hit_count++;                 //    UB —— 而且多半不崩，只是数据悄悄错
  ///    ```
  ///    ⇒ 返回 id + `find(id)` 取用，**从接口上杜绝**这个坑。
  ///    ⚠️ 当初的 `std::move(t)` 保留（`Track` 里有 `vector<Sample>`，move 省一次深拷贝）。
  ///
  /// ⚠️ 顺手**执行 `history_max`**（原实现只把上限写在注释里、代码没做 ⇒ 那是"注释在撒谎"）。
  std::uint32_t add_track(double mx, double my, double mz, double t_sec,
                          const Params& params);

  /// @brief 建一个**已知位置的常驻 track**（建筑位 / 柱子 / 穆斯蒂卡柱）
  ///
  /// ⚠️⚠️ **为什么需要它**（2026-09-25 接线时发现的设计缺口）：
  ///   模块 19 的 `KnownRegion::track_id` 要求指向一个**已存在的 track**，
  ///   而普通 `add_track` 的 id 来自 `next_id_++`（跑着的计数器）
  ///   ⇒ **建筑位编号与 track id 对不上** ⇒ 落区认领**一个也认不到**
  ///     ⇒ 19 的"落区优先于就近"这条核心语义**整条失效**（塔归属会串位）。
  ///
  /// ⇒ 本函数让调用方**指定 id**（就用建筑位编号），并：
  ///   · `on_known_spot = true`（⇒「v1.2」LOST 后仍保留，见 `keep_on_spot_frames`）
  ///   · 状态直接置 **CONFIRMED**（它是**场地常量**，不需要靠观测去确认）
  ///   · 速度给 0 且**速度协方差给小**（它不会动！与普通 track 的"给大"相反）
  ///
  /// ⚠️ 会把 `next_id_` 抬到 `id + 1` 之上，**避免之后的普通 track 撞号**。
  /// ⚠️ 位置是**场地事实** ⇒ 调用方要传**配置里的常量坐标**，不要传滤波估计值。
  Track& add_known_track(std::uint32_t id, double x, double y, double z,
                         double t_sec, const Params& params);

  /// @brief 按 id 找 track。**找不到返回 `nullptr`**（不抛、不造）。
  /// ⚠️ 返回的指针同样**在下一次 `add_track` 之后可能失效** —— 别跨调用持有。
  [[nodiscard]] Track* find(std::uint32_t id);
  [[nodiscard]] const Track* find(std::uint32_t id) const;

  /// @brief 追加一条历史采样，**并执行 `history_max` 上限**。
  ///
  /// ⚠️ 调用方**必须走这个函数**往 `Track::history` 里加 —— 直接 `push_back` 会绕过上限，
  ///    而"上限只活在注释里"正是 2026-09-25 复核抓出的问题（长时间比赛 + 高帧率 ⇒ 内存单调涨）。
  void push_history(Track& track, const Track::Sample& sample);

  /// 🆕「v1.6」清空**移动目标** track，保留建筑位顶部目标与穆斯蒂卡（规则 5.3.2：已放置物体保持原位）
  /// @param keep_types  要保留的 `type` 列表（穆斯蒂卡等）
  /// @return 被清掉的条数
  std::size_t clear_mobile_tracks(const std::vector<std::uint8_t>& keep_types);

  /// 🆕「v1.6」当前 `is_held == true` 的 track 数（终场告警 / 持握超限判定用）
  [[nodiscard]] std::size_t held_object_count() const;

  [[nodiscard]] const std::vector<Track>& tracks() const { return tracks_; }
  [[nodiscard]] std::vector<Track>& tracks() { return tracks_; }

  /// 下一个可用的 id（单调递增，**不复用** —— 复用会让下游把新旧目标混为一谈）
  [[nodiscard]] std::uint32_t next_id() const { return next_id_; }

private:
  std::vector<Track> tracks_;
  std::uint32_t next_id_{1};   ///< ⚠️ 从 1 起：0 留给"无效 id"
};

}  // namespace fusion
}  // namespace br_perception
