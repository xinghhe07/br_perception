#pragma once

/// ═══════════════════════════════════════════════════════════════════════════
/// kalman_tracker.hpp — 多目标卡尔曼追踪 (任务书 §4.2, 模块 18)
///
/// 依赖: **Eigen3** (仅头文件库) —— 无 ROS / PCL / OpenCV。
///
/// 职责 (§4.2 任务清单, **本文件冻结的是纯算法核心**):
///   1. 6 状态卡尔曼滤波器 [x, y, z, vx, vy, vz] (恒速模型)  → KalmanFilter6
///   2. 每个追踪目标维护独立滤波器                            → Track::filter
///   3. 预测步骤 (每帧执行) + 超时删除                        → predictAll()
///   4. 更新步骤 (有测量值时执行; 未命中的建新目标)            → applyDetections()
///   5. 追踪状态机 CANDIDATE / CONFIRMED / LOST               → Track::state
///   6. v1.2 建筑位顶部目标不受 LOST 删除约束                 → keep_on_spot_frames
///   7. v1.2 推移轨迹检测 → suspected_pushing                 → detectPushing()
///   8. v1.3 持握目标 (高置信视觉 + 无雷达匹配) → is_held     → Detection::is_held
///   9. v1.6 clear_mobile_tracks(keep_categories)             → clearMobileTracks()
///  10. v1.6 held_object_count                                → heldObjectCount()
///  11. 每个追踪目标存 id / 历史轨迹 / 速度 / 类别 / 置信度    → Track
///
/// ⚠ 刻意不做: 不发布 ROS topic (归 fusion_node); 不做单目 PnP (归
///   coordinate_transformer); 不做观测↔追踪关联 (归 target_associator / 模块 19)。
///
/// ═══ 相对 v1 的优化 (本轮修订) ═══
///   1. confirmedTrackRefs(): **零拷贝**访问 CONFIRMED 轨迹 (const Track* 视图),
///      替代每帧深拷贝整个 Track (含 6×6 协方差 + 历史 deque)。confirmedTracks()
///      保留为兼容包装。
///   2. MatchIndex + applyDetections(): 对 Match 建一次索引 (detection→track 命中表 +
///      track→detection)。**命中更新与建新目标合并成单次调用**, 好让"索引只建一次"
///      在结构上成立 —— 拆成两个方法就只能各建一次, 优化目标会落空。
///      匹配查找从 O(N·M) 降到 O(N+M)。
///   3. predict(): 对角过程噪声改为 P.diagonal() += noise, 省掉 Q 矩阵构造与
///      整矩阵加; dt ≤ 0 在最底层防御 (该类独立单测, 不能只靠调用方过滤)。
///   4. clearMobileTracks 改 const 引用传参; 补 noexcept / [[nodiscard]]。
///
/// ⚠ 与 target_associator (模块 19) 的分工: 关联不在本模块。调用方先跑模块 19
///   拿到 Match 列表, 再把 detections + matches 一起喂给 applyDetections()。
///
/// ═══ 任务书未定义、本文件自行判定的 5 处 (实现前请确认) ═══
///   A. 删除判据 max_prediction_age 与 delete_frame_count **任一满足即删**。
///   B. keep_on_spot_frames 从**最后一次被观测**起算 (丢失计时), 非创建起算。
///   C. Q = diag(process_noise) 逐步累加不乘 dt; R = 标量 × I₃ (各向同性)。
///   D. dt ≤ 0 → 跳过预测但不清状态, 计入 diagnostics (KalmanFilter6 层亦防御)。
///   E. 非有限值 Detection 直接丢弃并计数, 不进滤波器; 全程不抛异常。
///
/// 线程安全: **非线程安全**, 单线程持有 (fusion_node 内部调用)。
/// 单位约定: 位置 m, 速度 m/s, 时间 s; 方差参数按**方差**解释, 不是标准差。
/// ═══════════════════════════════════════════════════════════════════════════

#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>   // std::pair (MatchIndex::by_track_id_)
#include <vector>

#include <Eigen/Core>

namespace br_perception {
namespace fusion {

// ═══════════════════════════════════════════════════════════════════════════
// 基础类型
// ═══════════════════════════════════════════════════════════════════════════

using Point3 = Eigen::Vector3d;

/// @brief 追踪目标类别。前四档逐字对应 msg/Obstacle.msg 的 type 字段 (0-3)。
enum class TrackCategory : std::uint8_t {
  UNKNOWN = 0,
  ENEMY_ROBOT = 1,
  DROPPED_BLOCK = 2,
  OTHER = 3,
  BUILDING_SPOT_TOP = 4,   ///< 建筑位顶部目标 —— 走 keep_on_spot_frames 例外
  MUSTIKA = 5,             ///< 穆斯蒂卡 —— clear_mobile_tracks 时保留
};

enum class DetectionSource : std::uint8_t {
  LIDAR = 0,
  CAMERA = 1,
  FUSION = 2,
};

enum class TrackState : std::uint8_t {
  CANDIDATE = 0,   ///< 观测 1 帧, 不输出, 等待确认
  CONFIRMED = 1,   ///< 连续观测 ≥ confirm_frame_count, **输出给下游**
  LOST = 2,        ///< 丢失中; 达 delete_frame_count 帧后删除
};

/// @brief 单个观测 (一帧里的一条 3D 检测结果)。由调用方构造。
struct Detection {
  Point3 position = Point3::Zero();          ///< world 系 3D 位置 (m)
  TrackCategory category = TrackCategory::UNKNOWN;
  DetectionSource source = DetectionSource::LIDAR;
  float confidence = 0.0F;                   ///< [0, 1]
  double stamp_sec = 0.0;                    ///< 采集时刻 (s)
  bool is_held = false;                      ///< v1.3: 被 BR 持握
};

/// @brief 一条关联结果 (模块 19 的输出)。
struct Match {
  std::size_t detection_index = 0;   ///< 指向传入 applyDetections() 的 detections 下标
  std::uint32_t track_id = 0;
};

// ═══════════════════════════════════════════════════════════════════════════
// 6 状态卡尔曼滤波器 [x, y, z, vx, vy, vz] (恒速模型)
// ═══════════════════════════════════════════════════════════════════════════

class KalmanFilter6 {
public:
  static constexpr int kStateDim = 6;
  static constexpr int kMeasDim = 3;

  /// @brief 用首个观测初始化: 位置 = 观测, 速度 = 0, 协方差 = 给定初值
  [[nodiscard]] static KalmanFilter6 initialize(const Point3& position,
                                                double position_var,
                                                double velocity_var) noexcept;

  /// @brief 预测: x ← F x, P ← F P Fᵀ + Q。
  ///
  /// 实现优化: Q 为对角阵, 不求整矩阵 —— 直接 `P.diagonal() += process_noise`。
  /// **dt ≤ 0 时直接返回 (防御性, 调用方仍应按判定 D 计数 diagnostics)。**
  void predict(double dt_sec,
               const Eigen::Matrix<double, 6, 1>& process_noise) noexcept;

  /// @brief 标准卡尔曼更新 (3 维位置观测, 各向同性量测方差)。
  /// @param measurement_var 量测**方差** (m²)。必须 > 0 —— 传 ≤ 0 视为脏数据直接忽略。
  void update(const Point3& measured_position, double measurement_var);

  [[nodiscard]] Point3 position() const noexcept { return x_.head<3>(); }
  [[nodiscard]] Point3 velocity() const noexcept { return x_.tail<3>(); }

  /// @brief 位置分量方差 (协方差对角前 3 项, 用于置信度换算)
  [[nodiscard]] Point3 positionVariance() const noexcept {
    return p_.diagonal().head<3>();
  }

  [[nodiscard]] const Eigen::Matrix<double, 6, 1>& state() const noexcept { return x_; }
  [[nodiscard]] const Eigen::Matrix<double, 6, 6>& covariance() const noexcept { return p_; }

private:
  Eigen::Matrix<double, 6, 1> x_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> p_ = Eigen::Matrix<double, 6, 6>::Identity();
};

// ═══════════════════════════════════════════════════════════════════════════
// 追踪目标
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 一个追踪目标。⚠ 体积较大 (6×6 协方差 + 历史 deque),
///        **避免按值拷贝**, 优先用 confirmedTrackRefs() 的指针视图访问。
struct Track {
  std::uint32_t id = 0;
  TrackState state = TrackState::CANDIDATE;
  TrackCategory category = TrackCategory::UNKNOWN;
  float confidence = 0.0F;

  KalmanFilter6 filter;
  std::deque<Point3> history;          ///< 历史轨迹 (最新在尾), 上限 kMaxHistory

  std::uint32_t hit_streak = 0;
  std::uint32_t miss_streak = 0;
  double last_seen_sec = 0.0;          ///< 最后一次被观测时刻 (s) —— 判定 A / B 都用它

  bool is_held = false;                ///< v1.3
  bool suspected_pushing = false;      ///< v1.2

  static constexpr std::size_t kMaxHistory = 60;
};

// ═══════════════════════════════════════════════════════════════════════════
// 参数 (⚠ 字段名与 config/fusion_params.yaml 的键名**逐字一致**)
// ═══════════════════════════════════════════════════════════════════════════
//
/// yaml 键名对应 (改一处必须改另一处):
///   kalman.process_noise / kalman.measurement_noise_lidar /
///   kalman.measurement_noise_camera / kalman.max_prediction_age /
///   confirm_frame_count / delete_frame_count (这两个是 ros__parameters 直接子键) /
///   tracking.keep_on_spot_frames / tracking.push_detect_window_ms
struct TrackerParams {
  Eigen::Matrix<double, 6, 1> process_noise =
      (Eigen::Matrix<double, 6, 1>() << 0.05, 0.05, 0.05, 0.10, 0.10, 0.10).finished();

  double measurement_noise_lidar = 0.01;    ///< 雷达量测方差 (m²)
  double measurement_noise_camera = 0.05;   ///< 相机量测方差 (m²)
  double max_prediction_age = 1.0;          ///< 最大预测时长 (s), 超出删除

  std::uint32_t confirm_frame_count = 3;
  std::uint32_t delete_frame_count = 2;

  std::uint32_t keep_on_spot_frames = 300;  ///< 建筑位顶部目标丢失后强制保持帧数
  double push_detect_window_ms = 1000.0;    ///< 推移检测滑动窗口 (ms)

  // 下列三个是**本文件新增的常量**, 任务书与 yaml 都没有 (判定值, 可调)
  double initial_position_var = 0.25;   ///< 新建 track 位置初方差 (m², σ=0.5m)
  double initial_velocity_var = 4.0;    ///< 新建 track 速度初方差 (m²/s², σ=2m/s)
  double push_max_height_var = 0.04;    ///< 推移判定: 窗口内 z 方差上限 (m², σ=0.2m)
};

// ═══════════════════════════════════════════════════════════════════════════
// 诊断 (§7 风格: 计数器, 不抛异常)
// ═══════════════════════════════════════════════════════════════════════════

struct TrackerDiagnostics {
  std::uint64_t frames = 0;
  std::uint64_t created = 0;
  std::uint64_t confirmed = 0;           ///< CANDIDATE → CONFIRMED 累计次数
  std::uint64_t deleted_age = 0;
  std::uint64_t deleted_miss = 0;
  std::uint64_t kept_on_spot = 0;
  std::uint64_t dropped_non_finite = 0;  ///< 判定 E
  std::uint64_t skipped_dt = 0;          ///< 判定 D
  std::uint64_t pushing_flagged = 0;
};

// ═══════════════════════════════════════════════════════════════════════════
// 匹配索引 (本轮新增): 把 vector<Match> 转成 O(1) 查找结构。**由
// applyDetections() 在单帧内构造一次**, 命中更新与建新目标共用同一份,
// 摊掉平方级扫描。
// ═══════════════════════════════════════════════════════════════════════════

/// @brief Match 的查找索引。一次构建, 多次查询。
///
///   detectionMatched(i)  → detection i 是否已被某 track 命中 (建新目标用)
///   matchForTrack(id)    → track id 对应 Match 的下标 (命中更新用)
///
/// 公开而非藏在 .cpp 里, 是为了**能脱离 Tracker 单独单测** (只有查找表语义, 无状态机)。
///
/// ⚠ 本结构不拥有数据, 生命周期内必须保证 detections / matches 不被修改。
class MatchIndex {
public:
  /// @param num_detections 本帧 detection 总数
  /// @param matches        模块 19 输出的关联结果 (需先按 track_id 去重校验,
  ///                       同一 track 出现多条时保留第一条)
  MatchIndex(std::size_t num_detections, const std::vector<Match>& matches);

  [[nodiscard]] bool detectionMatched(std::size_t detection_index) const noexcept;
  /// @return 命中的 Match 下标; 未命中返回 npos
  [[nodiscard]] std::size_t matchForTrack(std::uint32_t track_id) const noexcept;

  static constexpr std::size_t npos = static_cast<std::size_t>(-1);

private:
  std::vector<std::uint8_t> det_matched_;      ///< 每 detection 是否被命中
  std::vector<std::pair<std::uint32_t, std::size_t>> by_track_id_;  ///< 已按 id 排序
};

// ═══════════════════════════════════════════════════════════════════════════
// 追踪器
// ═══════════════════════════════════════════════════════════════════════════

/// @brief 多目标追踪器。调用顺序 (每帧):
///
///     tracker.predictAll(stamp_sec);                                    // 1
///     auto matches = associator.associate(dets, tracker.allTracks());   // 2 模块 19
///     tracker.applyDetections(dets, matches);                           // 3 更新命中 + 建新目标
///     for (const Track* t : tracker.confirmedTrackRefs()) { ... }       // 4 零拷贝输出
class KalmanTracker {
public:
  explicit KalmanTracker(const TrackerParams& params = TrackerParams());

  // ── 每帧调用 ──

  /// @brief 步骤 1: 全部 track 预测到 stamp_sec, 按判定 A 剪枝, 刷新推移标记。
  void predictAll(double stamp_sec);

  /// @brief 步骤 3: **更新命中的 + 把未命中的建成新目标**, 一次调用做完。
  ///
  /// 两个动作合并成一个入口, 是为了让 MatchIndex **只建一次**在结构上成立
  /// (拆成两个方法就只能各建一次索引, 优化目标落空), 不靠调用方自觉。
  ///
  /// - 命中的 Detection → 更新对应 track (状态机迁移 + 滤波器更新 + 历史)
  /// - 未命中的 Detection → 建成新 track (CANDIDATE)
  /// - 非有限值 Detection → 丢弃并计数 (判定 E); 其命中的 track 按**未命中**处理
  ///   (miss_streak +1, 状态机照常向 LOST 迁移)
  void applyDetections(const std::vector<Detection>& detections,
                       const std::vector<Match>& matches);

  // ── v1.6 ──

  /// @brief 重试时清空移动目标 track, 保留 keep_categories 里的。
  void clearMobileTracks(const std::vector<TrackCategory>& keep_categories);

  /// @brief v1.6: 当前 is_held == true 的 track 数
  [[nodiscard]] std::size_t heldObjectCount() const noexcept;

  // ── 查询 ──

  /// @brief **推荐**: CONFIRMED 轨迹的零拷贝视图 (const Track*, 指向内部存储)。
  ///        生命周期内不得调用 predictAll/applyDetections/reset/clearMobileTracks。
  [[nodiscard]] std::vector<const Track*> confirmedTrackRefs() const;

  /// @brief CONFIRMED 轨迹副本 —— 兼容旧调用方。**每帧深拷贝, 优先用
  ///        confirmedTrackRefs()**。仅当需要跨帧持有数据时使用本接口。
  [[nodiscard]] std::vector<Track> confirmedTracks() const;

  /// @brief 全部 track (含 CANDIDATE / LOST), 供模块 19 关联与可视化
  [[nodiscard]] const std::vector<Track>& allTracks() const noexcept { return tracks_; }

  [[nodiscard]] const TrackerDiagnostics& diagnostics() const noexcept { return diag_; }

  /// @brief 清空全部 track 与诊断 (BR_RESET)。参数保留。
  void reset() noexcept;

private:
  bool exemptFromDeletion(const Track& t, double now_sec) const noexcept;
  void detectPushing(Track& t) const;
  void applyMeasurement(Track& t, const Detection& det);

  TrackerParams params_;
  std::vector<Track> tracks_;
  TrackerDiagnostics diag_;
  std::uint32_t next_id_ = 1;   ///< 从 1 开始; 0 保留为"无效 id"
};

}  // namespace fusion
}  // namespace br_perception
