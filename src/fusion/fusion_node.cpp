#include "br_perception/fusion/fusion_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace br_perception {
namespace fusion {

namespace {

/// `BuildingSpot.state` 的取值（**与 `msg/BuildingSpot.msg` 逐字对应**）
constexpr std::uint8_t kSpotEmpty      = 0;
constexpr std::uint8_t kSpotOneEarth   = 1;
constexpr std::uint8_t kSpotTwoEarth   = 2;
constexpr std::uint8_t kSpotFullTower  = 3;
/// ⚠️⚠️ **`4` 是"没量成"，不是"空"** —— 与 `0` 必须分开。
///    合并的后果：塔明明建着，主控以为没建（或反之）—— 两件事的动作完全不同。
constexpr std::uint8_t kSpotUnknown    = 4;

/// `Track.msg` 的 SRC_*
constexpr std::uint8_t kSrcLidar = 1;

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════════════
FusionNode::FusionNode(const rclcpp::NodeOptions& options)
  : Node("fusion_node", options),
    camera_buf_(0.5)   // ⚠️ 真实窗口在下面按参数重设（成员初始化时参数还没读到）
{
  auto safe_param = [this](const char* name, auto& out) {
    using T = std::decay_t<decltype(out)>;
    try {
      this->declare_parameter<T>(name, out);
      this->get_parameter(name, out);
    } catch (const std::exception& e) {
      // ⚠️ 参数类型不匹配时 rclcpp **抛异常**（不是回落默认值），而且是在
      //    **`declare_parameter`** 就抛（覆盖在"声明"那刻应用）⇒ 只兜 get 没用。
      //    这是在构造函数里 ⇒ 未捕获 = **整个节点 terminate**。见 §八 9.18 ③。
      RCLCPP_ERROR(this->get_logger(),
        "参数 '%s' 类型不对 ⇒ **沿用默认值**。%s", name, e.what());
    }
  };

  safe_param("lidar_topic", lidar_topic_);
  safe_param("camera_topic", camera_topic_);
  safe_param("from_master_topic", from_master_topic_);
  safe_param("reset_keep_types", reset_keep_types_);
  safe_param("state_topic", state_topic_);
  safe_param("diag_topic", diag_topic_);
  safe_param("sync_window_sec", sync_window_sec_);
  safe_param("shared_spot_ids", shared_spot_ids_);
  // ⚠️ 这三个键就在 `config/field_geometry.yaml` 第 11 节里（`/**` 通配 ⇒ 能匹配上）
  safe_param("building_spot_x", building_spot_x_);
  safe_param("building_spot_y", building_spot_y_);
  safe_param("building_spot_z", building_spot_z_);
  safe_param("region_radius_m", region_radius_m_);
  safe_param("enable_tracking", enable_tracking_);
  safe_param("camera_timeout_sec", camera_timeout_sec_);

  safe_param("sanctuary_min_complete_towers", sanctuary_params_.min_complete_towers);
  safe_param("sanctuary_min_in_shared_zone", sanctuary_params_.min_in_shared_zone);
  safe_param("hold_check_radius_m", holding_params_.hold_check_radius_m);
  safe_param("holding_limit_count", holding_params_.holding_limit_count);
  safe_param("match_duration_sec", match_end_params_.match_duration_sec);
  safe_param("match_end_release_window_sec", match_end_params_.release_window_sec);
  safe_param("out_of_bounds_forget_sec", bounds_params_.out_of_bounds_forget_sec);
  safe_param("field_size", bounds_params_.field_size);
  safe_param("assoc_gate_m", assoc_params_.gate_m);
  safe_param("assoc_default_region_m", assoc_params_.default_region_m);
  safe_param("confirm_frame_count", tracker_params_.confirm_frame_count);
  safe_param("delete_frame_count", tracker_params_.delete_frame_count);
  safe_param("max_prediction_age_sec", tracker_params_.max_prediction_age_sec);

  // ⚠️ 时间同步窗口按参数重设（构造 `camera_buf_` 时参数还没读到 ⇒ 这里是唯一正确位置）
  camera_buf_ = TimeSyncBuffer<br_perception::msg::CameraFrame>(sync_window_sec_);

  sub_lidar_ = this->create_subscription<br_perception::msg::LidarResult>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FusionNode::on_lidar, this, std::placeholders::_1));
  sub_camera_ = this->create_subscription<br_perception::msg::CameraFrame>(
      camera_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FusionNode::on_camera, this, std::placeholders::_1));
  // ⚠️ 默认 QoS（RELIABLE）—— 与 `comm_node` 的发布端一致。
  //    ⚠️ **这条订阅曾经不存在** ⇒ `/communication/from_master` 是个**零订阅者的孤儿话题**
  //    ⇒ 主控发的 `BR_RESET` **到了 ROS 就没人接**，规则 5.3.2 那条联动**从没生效过**。
  sub_from_master_ = this->create_subscription<br_perception::msg::MasterCommand>(
      from_master_topic_, rclcpp::QoS(10),
      std::bind(&FusionNode::on_from_master, this, std::placeholders::_1));

  // ⚠️ 输出用默认 QoS（RELIABLE）—— 它是**给 pipeline 消费的结果**，不是高频原始流；
  //    丢了会让主控看到旧的圣所结论，那个代价比"迟到"高。
  pub_state_ = this->create_publisher<br_perception::msg::FusionState>(state_topic_, 10);
  pub_diag_  = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diag_topic_, 10);

  RCLCPP_INFO(this->get_logger(),
    "FusionNode 起 | lidar=%s camera=%s → state=%s | sync=%.2fs 共享位=%zu 个 | tracking=%s",
    lidar_topic_.c_str(), camera_topic_.c_str(), state_topic_.c_str(),
    sync_window_sec_, shared_spot_ids_.size(), enable_tracking_ ? "ON" : "OFF");
  // ⚠️ **建筑位常驻 track 在启动时建一次** —— 它们是场地常量，不该每帧重建。
  build_known_regions();

  RCLCPP_INFO(this->get_logger(),
    "⚠️ 本节点**不发 /perception/output**（pipeline 保有上行权，用户 2026-09-25 拍板）");
}

// ═══════════════════════════════════════════════════════════════════════════
// 已知位置（建筑位）—— **让 19 的"落区认领"那一级真的生效**
// ═══════════════════════════════════════════════════════════════════════════
void FusionNode::build_known_regions()
{
  const std::size_t n = std::min({building_spot_x_.size(),
                                  building_spot_y_.size(),
                                  building_spot_z_.size()});
  if (n == 0) {
    // ⚠️ **不静默**：没有已知位置 ⇒ 19 只剩"就近指派"那一级 ⇒ 塔归属会串位。
    //    这是"看着在跑、语义没生效"的典型，必须喊出来。
    RCLCPP_WARN(this->get_logger(),
      "**没有建筑位坐标**（building_spot_x/y/z 为空或长度不齐）⇒ 19 的【落区认领】不生效，"
      "只剩就近指派 ⇒ **塔归属可能串位**。"
      "⚠️ 检查 launch 有没有把 config/field_geometry.yaml 传进来。");
    return;
  }
  if (building_spot_x_.size() != building_spot_y_.size() ||
      building_spot_x_.size() != building_spot_z_.size()) {
    RCLCPP_WARN(this->get_logger(),
      "building_spot_x/y/z 长度不齐（%zu/%zu/%zu）⇒ 只取前 %zu 个",
      building_spot_x_.size(), building_spot_y_.size(), building_spot_z_.size(), n);
  }

  const double t0 = this->now().seconds();
  known_regions_.clear();
  known_regions_.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto id = static_cast<std::uint32_t>(i);
    // ⚠️ **建常驻 track，id 就用建筑位编号** —— 19 的 `KnownRegion::track_id`
    //    要求指向一个**已存在的 track**，而普通 `add_track` 的 id 来自跑着的计数器
    //    ⇒ 对不上。见 `KalmanTracker::add_known_track` 的 ⚠️⚠️。
    tracker_.add_known_track(id, building_spot_x_[i], building_spot_y_[i],
                             building_spot_z_[i], t0, tracker_params_);

    TargetAssociator::KnownRegion r;
    r.track_id = id;
    r.x = building_spot_x_[i];
    r.y = building_spot_y_[i];
    r.z = building_spot_z_[i];
    r.radius_m = region_radius_m_;
    known_regions_.push_back(r);
  }

  RCLCPP_INFO(this->get_logger(),
    "已知位置 %zu 个（建筑位）+ 常驻 track ⇒ **落区认领生效**；半径 %.2fm，共享位 %zu 个",
    n, region_radius_m_, shared_spot_ids_.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// 建筑位 → 圣所判定的输入
// ═══════════════════════════════════════════════════════════════════════════
bool FusionNode::is_shared_spot(std::uint32_t id) const
{
  return std::find(shared_spot_ids_.begin(), shared_spot_ids_.end(),
                   static_cast<int64_t>(id)) != shared_spot_ids_.end();
}

std::vector<FusionDecisions::SpotState> FusionNode::build_spots(
    const br_perception::msg::LidarResult& msg) const
{
  std::vector<FusionDecisions::SpotState> out;
  out.reserve(msg.building_spots.size());
  for (const auto& bs : msg.building_spots) {
    FusionDecisions::SpotState s;
    s.id = bs.spot_id;
    // ⚠️⚠️ **`4 (UNKNOWN)` 映射成 `known = false`，绝不能映射成 `complete = false`。**
    //    合 `BuildingSpot.msg` 的原文："`0` = 量到了、这里是空的；`4` = 没量成
    //    ⇒ **两件事的动作完全不同**"。
    //    对一个 Gate 着 250 分的判定来说，把"没量成"当成"不是塔"
    //    会**凭空否掉**一座可能存在的塔。
    s.known = (bs.state != kSpotUnknown);
    s.complete = (bs.state == kSpotFullTower);
    s.in_shared_zone = is_shared_spot(bs.spot_id);
    out.push_back(s);
  }
  return out;
}

// ═══════════════════════════════════════════════════════════════════════════
// 相机帧：只进时间同步缓冲
// ═══════════════════════════════════════════════════════════════════════════
void FusionNode::on_camera(const br_perception::msg::CameraFrame::SharedPtr msg)
{
  // ⚠️ **本节点目前只用它做时间对齐，不参与判定** —— 逐目标融合（相机给颜色/语义）
  //    里穆斯蒂卡那条已在模块 20 落、其余归 `perception_pipeline`（它已在做
  //    "用相机确认色覆盖雷达 top_color"）。**本节点再做一遍就是第二处真值来源。**
  //    见头文件的三处已知边界之 ①。
  const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  camera_buf_.push(t, *msg);
}

// ═══════════════════════════════════════════════════════════════════════════
// 「v1.6」BR_RESET 联动（规则 5.3.2：**已放置物体保持原位**）
// ═══════════════════════════════════════════════════════════════════════════
void FusionNode::on_from_master(const br_perception::msg::MasterCommand::SharedPtr msg)
{
  // ⚠️ **只认 `BR_RESET`，其余原样不管** —— `comm_node.cpp:240` 写着
  //    「一律原样发布: 不解析、不丢弃。主控侧**除 0x10 外都没有字节级定义**」
  //    ⇒ 别的类型**没有可依据的语义**，猜着处理比不处理更危险。
  if (msg->type != br_perception::msg::MasterCommand::BR_RESET) return;

  ++reset_count_;

  // ── 规则 5.3.2：已放置物体保持原位 ──
  //    ⇒ 清**移动**目标（敌方机器人 / 障碍 / 掉落方块），
  //      保留**建筑位顶部目标**（`on_known_spot` 在 `clear_mobile_tracks` 里硬编码保留）
  //      与**点名类别**（默认穆斯蒂卡 —— 它在柱上，不会动）。
  std::vector<std::uint8_t> keep;
  keep.reserve(reset_keep_types_.size());
  for (const auto t : reset_keep_types_) {
    keep.push_back(static_cast<std::uint8_t>(t));
  }
  const auto before = tracker_.tracks().size();
  const auto removed = tracker_.clear_mobile_tracks(keep);

  // ⚠️ **WARN 级**：这是一次"主控要求复位"的事件，日志里要看得见 ——
  //    静默清掉一堆 track 会让现场误以为"追踪自己丢了"。
  RCLCPP_WARN(this->get_logger(),
    "收到 BR_RESET(0x10) zone=%u（第 %u 次）⇒ 按规则 5.3.2 清移动目标："
    "track %zu → %zu（清掉 %zu，保留建筑位与 %zu 类点名类别）",
    static_cast<unsigned>(msg->seq), reset_count_, before, tracker_.tracks().size(),
    removed, keep.size());

  // ⚠️ **位姿先验注入 / 触发重定位不在这里** —— 它们要 `coordinate_transformer`
  //    的那套位姿注入接口（§4.1 的 `pose_prior_window_sec`）。本节点只做
  //    "**把事件变成 track 侧的联动**"，位姿那条留给接线时定。
  //    ⚠️ 没做的部分**如实记在台账**，不假装它做了。
}

// ═══════════════════════════════════════════════════════════════════════════
// 主回调：每收到一帧雷达结果跑一次融合
// ═══════════════════════════════════════════════════════════════════════════
void FusionNode::on_lidar(const br_perception::msg::LidarResult::SharedPtr msg)
{
  const auto t0 = std::chrono::steady_clock::now();
  const double now_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  // ⚠️ **比赛自计时从"首次收到雷达结果"起** —— 拿节点启动时刻当起点的话，
  //    "起得早"会变成"感知以为比赛开始了"，而终场告警是按这个算的。
  if (!match_started_) {
    match_started_ = true;
    match_start_sec_ = now_sec;
    RCLCPP_INFO(this->get_logger(), "比赛自计时开始（首帧雷达结果）");
  }
  const double elapsed = now_sec - match_start_sec_;

  auto out = br_perception::msg::FusionState();
  out.header = msg->header;

  // ── ① 圣所使命（**Gate 着 250 分**）──────────────────────────────────
  const auto verdict = FusionDecisions::judge_sanctuary(build_spots(*msg), sanctuary_params_);
  out.sanctuary_mandate_fulfilled = verdict.fulfilled ? 1 : 0;
  // ⚠️ **把"可不可信"一起发出去** —— 只要有一位 state==UNKNOWN，这个 bool
  //    就不是完整数据的结果。下游据此决定要不要把它当事实送给主控
  //    （合 §零「准」轴：未知要有哨兵）。
  out.sanctuary_valid = verdict.valid;

  // ── ② 关联 + 追踪 ───────────────────────────────────────────────────
  std::vector<TargetAssociator::Observation> obs;
  obs.reserve(msg->obstacles.size());
  for (const auto& o : msg->obstacles) {
    TargetAssociator::Observation x;
    x.id = o.id;
    x.x = o.position.x; x.y = o.position.y; x.z = o.position.z;
    x.confidence = o.confidence;
    x.type = o.type;
    x.source = kSrcLidar;
    // ⚠️ 持握标记这一帧还没有（它来自视觉），先 false —— 见头文件的边界 ①。
    x.is_held = false;
    obs.push_back(x);
  }

  std::vector<TargetAssociator::Track> tracks_snapshot;
  tracks_snapshot.reserve(tracker_.tracks().size());
  for (const auto& tr : tracker_.tracks()) {
    TargetAssociator::Track x;
    x.id = tr.id;
    x.x = tr.kf_state(0); x.y = tr.kf_state(1); x.z = tr.kf_state(2);
    x.confidence = tr.confidence;
    x.type = tr.type;
    x.is_held = tr.is_held;
    // ⚠️ **必须传**：否则已知位置的目标会参与就近指派，
    //    把一个离得近的移动目标抢走（实测 0.354m < 0.6m 门限）
    x.is_known_position = tr.on_known_spot;
    tracks_snapshot.push_back(x);
  }

  // ⚠️ **落区认领那一级现在真的生效了** —— `known_regions_` 由 `build_known_regions()`
  //    在启动时按 `building_spot_x/y/z`（场地真值）铺好，并配了同 id 的常驻 track。
  //    见 `KalmanTracker::add_known_track` 的 ⚠️⚠️（为什么必须"常驻 + 指定 id"）。
  const auto matches = TargetAssociator::associate(obs, tracks_snapshot,
                                                   known_regions_, assoc_params_);

  std::size_t associated = 0, new_cand = 0;
  if (enable_tracking_) {
    // ① 已有 track：预测 + （若被关联上）更新
    for (std::size_t j = 0; j < tracker_.tracks().size(); ++j) {
      auto& tr = tracker_.tracks()[j];
      const double dt = now_sec - tr.last_seen_sec;
      if (dt > 0.0) {
        KalmanTracker::predict(tr.kf_state, tr.kf_cov, dt, tracker_params_);
      }
      // 找有没有观测关联到它
      const TargetAssociator::Observation* hit = nullptr;
      for (const auto& m : matches) {
        if (m.track_index == static_cast<int>(j) && m.obs_index >= 0) {
          hit = &obs[static_cast<std::size_t>(m.obs_index)];
          break;
        }
      }
      const bool got = (hit != nullptr);
      if (got) {
        KalmanTracker::update(tr.kf_state, tr.kf_cov, hit->x, hit->y, hit->z, tracker_params_);
        tr.hit_count++;
        tr.miss_count = 0;                    // ⚠️ 命中必须清零（状态机的契约）
        tr.last_seen_sec = now_sec;
        tr.confidence = hit->confidence;
        tr.type = hit->type;
        tr.last_update_src = kSrcLidar;
        ++associated;
      } else {
        tr.miss_count++;                      // ⚠️ 只有丢失才累加
      }
      // 轨迹（诊断与推移检测用）。⚠️ 走 push_history ⇒ 执行 history_max 上限。
      tracker_.push_history(tr, {now_sec, tr.kf_state(0), tr.kf_state(1), tr.kf_state(2)});
      tr.suspected_pushing = KalmanTracker::is_pushing(tr.history, tracker_params_);

      tr.state = KalmanTracker::step_state(tr.state, got, tr.hit_count, tr.miss_count,
                                           now_sec - tr.last_seen_sec, tracker_params_);
    }

    // ② 未匹配的观测：建新候选
    for (const auto& m : matches) {
      if (m.track_index >= 0) continue;
      const auto& o = obs[static_cast<std::size_t>(m.obs_index)];
      const auto id = tracker_.add_track(o.x, o.y, o.z, now_sec, tracker_params_);
      if (auto* p = tracker_.find(id)) {
        p->type = o.type;
        p->confidence = o.confidence;
        p->last_update_src = kSrcLidar;
      }
      ++new_cand;
    }

    // ③ 删掉该死掉的（⚠️ 走 is_dead 才能让 `on_known_spot` 的保留策略生效）
    auto& ts = tracker_.tracks();
    ts.erase(std::remove_if(ts.begin(), ts.end(),
                            [this](const KalmanTracker::Track& t) {
                              return KalmanTracker::is_dead(t.state, t.on_known_spot,
                                                            /*kept_on_spot_frames=*/0,
                                                            tracker_params_);
                            }),
             ts.end());
  }

  // ── ③ 持握 / 终场 ────────────────────────────────────────────────────
  // ⚠️ 持握判定需要**方块**（不是 track）+ **BR 位置**。前者来自雷达的 obstacles
  //    （带 is_held 的更上游），后者来自 track。当前两者都还没接全
  //    （见头文件边界 ①）⇒ 这里如实报 **0 / 不可信**，而不是编一个数出来。
  //    ⚠️ 合 §零「准」轴：**不许用 0 冒充"没量到"** —— 所以 `held_count_valid = false`。
  const int held_count = 0;
  out.held_object_count = static_cast<std::uint8_t>(held_count);
  out.held_count_valid = false;   // ⚠️ 还没有真值来源 ⇒ 明确说不可信，不装作有
  out.holding_over_limit = 0;

  out.match_remaining_sec = static_cast<std::int16_t>(
      std::lround(FusionDecisions::remaining_sec(elapsed, match_end_params_)));
  out.match_end_release_warning =
      FusionDecisions::match_end_release_warning(elapsed, held_count, match_end_params_) ? 1 : 0;

  // ── ④ 追踪列表 + 诊断 ────────────────────────────────────────────────
  for (const auto& tr : tracker_.tracks()) {
    br_perception::msg::Track t;
    t.header = msg->header;
    t.id = tr.id;
    t.state = tr.state;
    t.position.x = tr.kf_state(0); t.position.y = tr.kf_state(1); t.position.z = tr.kf_state(2);
    t.velocity.x = tr.kf_state(3); t.velocity.y = tr.kf_state(4); t.velocity.z = tr.kf_state(5);
    t.type = tr.type;
    t.source = tr.last_update_src;
    t.confidence = tr.confidence;
    t.hit_count = tr.hit_count;
    t.miss_count = tr.miss_count;
    t.last_seen_sec = tr.last_seen_sec;
    t.suspected_pushing = tr.suspected_pushing;
    t.is_held = tr.is_held;
    t.is_ally_tr = tr.is_ally_tr;
    t.last_update_src = tr.last_update_src;
    out.tracks.push_back(t);
  }
  out.associated_count = static_cast<std::uint16_t>(associated);
  out.new_candidate_count = static_cast<std::uint16_t>(new_cand);
  out.track_total = static_cast<std::uint16_t>(tracker_.tracks().size());

  const auto t1 = std::chrono::steady_clock::now();
  out.fusion_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

  pub_state_->publish(out);
  publish_diagnostics(out.fusion_ms, tracker_.tracks().size(), obs.size(), new_cand);
}

// ═══════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════
void FusionNode::publish_diagnostics(double ms, std::size_t tracks, std::size_t obs,
                                     std::size_t new_cand)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "fusion_node";
  st.hardware_id = "fusion";

  if (ms < 10.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "OK";
  } else if (ms < 25.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "SLOW";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "VERY SLOW";
  }

  auto add = [&](const std::string& k, double v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k; kv.value = std::to_string(v);
    st.values.push_back(kv);
  };
  add("fusion_ms", ms);
  add("track_total", static_cast<double>(tracks));
  add("obs_count", static_cast<double>(obs));
  add("new_candidates", static_cast<double>(new_cand));
  add("camera_buf_size", static_cast<double>(camera_buf_.size()));
  add("reset_count", static_cast<double>(reset_count_));   // ⚠️ 收到过几次 BR_RESET

  arr.status.push_back(st);
  pub_diag_->publish(arr);
}

}  // namespace fusion
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::fusion::FusionNode)
