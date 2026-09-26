/// ═══════════════════════════════════════════════════════════════════════════════
/// camera_perception_node.cpp — 相机层聚合"薄壳"实现（任务书 §15.1 模块 10，P0-2）
///
/// 把三个检测器 × 两路相机 + 深度，聚合成"某一时刻的那一帧"。
/// **只做时间对齐 + 打包** —— 不检测、不变换坐标、不碰外参。
/// 四条组帧口径见头文件。
///
/// ⚠️ 本文件**一行都没编译过**（本机无 C++ 编译器、无 ROS），
///    按 doc/待编译验证清单_20260920.md 的做法做 grep 自检。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/camera_perception_node.hpp"

#include <algorithm>
#include <functional>
#include <type_traits>   // std::is_same_v —— store_slot 里按消息类型挡掉 camera_id 检查
#include <utility>       // std::move

namespace br_perception {
namespace camera {

namespace {
/// @brief 消息**没填** header.stamp 的判据（ROS 时间 0 点 = 未初始化）
inline bool stamp_is_unset(const rclcpp::Time& t)
{
  return t.nanoseconds() == 0;
}

/// @brief 节流用：距上次告警是否已过 `ws`（**单调时钟** —— 不用 ROS 时间，
///        仿真里它会暂停/跳变，拿它算间隔会得到负数或天文数字）
inline bool throttled(std::chrono::steady_clock::time_point& last, double ws)
{
  const auto now_tp = std::chrono::steady_clock::now();
  if (last.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now_tp - last).count() < ws) {
    return true;
  }
  last = now_tp;
  return false;
}
}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// 槽位名
// ═══════════════════════════════════════════════════════════════════════════════
const char* CameraPerceptionNode::slot_name(size_t slot)
{
  switch (slot) {
    case 0: return "mustika_wide";
    case 1: return "mustika_tele";
    case 2: return "pillar_wide";
    case 3: return "pillar_tele";
    case 4: return "spot_wide";
    case 5: return "spot_tele";
    case 6: return "depth";
    case 7: return "depth_tele(保留)";
    default: return "?";
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════════════════
CameraPerceptionNode::SlotVerdict CameraPerceptionNode::judge_slot(
    bool has_data, double age_sec, double fresh_tolerance)
{
  SlotVerdict v;
  if (!has_data) {
    // ⚠️ NEVER 与 STALE 是**两回事**（见 msg/SlotStatus.msg）：
    //    NEVER 多半是"话题名对不上 / 发布端没起 / 这路相机压根没装"，
    //    STALE 是"跑过，但慢了"。现场排查方向完全不同。
    v.state = br_perception::msg::SlotStatus::S_NEVER;
    v.stale_sec = 0.0f;
    v.note = "never";
    return v;
  }

  // ⚠️ **如实报负值，不钳到 0** —— "时间是负的"本身就是要暴露的异常
  v.stale_sec = static_cast<float>(age_sec);

  // ⚠️⚠️ **判据是 `age > tolerance`，不是 `age > 0`**：
  //    曝光时刻**永远早于**组帧时刻 ⇒ `age` 恒为正 ⇒ 拿 `age > 0` 当判据会让
  //    `state` **永远到不了 FRESH**，那一档等于不存在。
  // ⚠️ **负 age（时间戳在未来）算 FRESH**：那是"**钟不对**"，不是"数据旧"，
  //    该异常由 `stale_sec` 的负值本身暴露。
  // ⚠️ 边界**非严格**：恰好等于容差算 FRESH。
  const bool stale = age_sec > fresh_tolerance;
  v.state = stale ? br_perception::msg::SlotStatus::S_STALE
                  : br_perception::msg::SlotStatus::S_FRESH;
  v.note = stale ? "stale" : "ok";
  return v;
}

CameraPerceptionNode::CameraPerceptionNode(const rclcpp::NodeOptions& options)
  : Node("camera_perception", options)
{
  // ── 参数 ──
  // ⚠️ 话题参数是**前缀**，不是全名（见头文件）。名字里带 prefix 是为了让
  //    "填了带后缀的全名 → 得到 ..._wide_wide → 静默无数据"这个坑不可能踩。
  this->declare_parameter<std::string>("mustika_topic_prefix",
                                       "/perception/camera/mustika");
  this->declare_parameter<std::string>("pillar_topic_prefix",
                                       "/perception/camera/pillars");
  this->declare_parameter<std::string>("spot_topic_prefix",
                                       "/perception/camera/building_spots");
  this->declare_parameter<std::string>("depth_topic",   "/perception/camera/depth");
  this->declare_parameter<std::string>("frame_topic",   "/perception/camera/frame");
  this->declare_parameter<std::string>("diag_topic",    "/perception/camera/frame_diagnostics");
  this->declare_parameter<double>("frame_rate_hz",   frame_rate_hz_);
  this->declare_parameter<double>("diag_rate_hz",    diag_rate_hz_);
  this->declare_parameter<double>("assembly_warn_ms", assembly_warn_ms_);
  this->declare_parameter<double>("future_tolerance_sec", future_tolerance_sec_);
  this->declare_parameter<double>("fresh_tolerance_sec", fresh_tolerance_sec_);

  this->get_parameter("mustika_topic_prefix", mustika_topic_prefix_);
  this->get_parameter("pillar_topic_prefix",  pillar_topic_prefix_);
  this->get_parameter("spot_topic_prefix",    spot_topic_prefix_);
  this->get_parameter("depth_topic",   depth_topic_);
  this->get_parameter("frame_topic",   frame_topic_);
  this->get_parameter("diag_topic",    diag_topic_);
  this->get_parameter("frame_rate_hz",   frame_rate_hz_);
  this->get_parameter("diag_rate_hz",    diag_rate_hz_);
  this->get_parameter("assembly_warn_ms", assembly_warn_ms_);
  this->get_parameter("future_tolerance_sec", future_tolerance_sec_);
  this->get_parameter("fresh_tolerance_sec", fresh_tolerance_sec_);

  // ── 规范化 ──
  // ⚠️ 组帧频率上限 200Hz: 再高就只是在重复打包同一批没变的数据, 纯烧 CPU
  //    (目标机 6 核全分完了, 详见 doc/模块进度.md 的 CPU 预算说明)
  frame_rate_hz_ = std::max(1.0, std::min(frame_rate_hz_, 200.0));
  diag_rate_hz_  = std::max(0.1, std::min(diag_rate_hz_, 10.0));
  assembly_warn_ms_ = std::max(0.0, assembly_warn_ms_);
  future_tolerance_sec_ = std::max(0.0, future_tolerance_sec_);
  // ⚠️ 下限 0.01s：填 0 会让每个槽都恒报 STALE（见上面 FRESH/STALE 那段注释）
  fresh_tolerance_sec_  = std::max(0.01, fresh_tolerance_sec_);

  // ── 订阅：6 个检测结果 ──
  // ⚠️ QoS 的**兼容方向是单向的**，别记反：
  //    · RELIABLE 订阅 ← BEST_EFFORT 发布  = **不兼容**（静默收不到，不报错）
  //    · BEST_EFFORT 订阅 ← RELIABLE 发布  = 兼容（能收到）
  //    所以订阅端取 BEST_EFFORT 是**安全侧**；⚠️ 反过来改 RELIABLE 才会踩雷。
  //
  // ⚠️⚠️ **队列深度 = 1**（任务书 §3.0 硬约束第 2 条，2026-09-22 落地）：
  //    原先这里是 `SensorDataQoS()`，depth 是 **5** —— 回调慢下来时攒 5 帧、
  //    之后永远落后，且**卡顿过去也不恢复**。完整理由见
  //    `image_preprocessor.cpp` 的 `stream_qos` 那段。
  //    ⚠️ 六个检测器**发布端也已同时改成 depth=1**，两侧一起改才生效。
  const auto stream_qos = rclcpp::QoS(1).best_effort();

  sub_mustika_wide_ = this->create_subscription<br_perception::msg::CameraMustika>(
    mustika_topic_prefix_ + "_wide", stream_qos,
    std::bind(&CameraPerceptionNode::mustika_wide_callback, this, std::placeholders::_1));
  sub_mustika_tele_ = this->create_subscription<br_perception::msg::CameraMustika>(
    mustika_topic_prefix_ + "_tele", stream_qos,
    std::bind(&CameraPerceptionNode::mustika_tele_callback, this, std::placeholders::_1));

  sub_pillar_wide_ = this->create_subscription<br_perception::msg::CameraPillar>(
    pillar_topic_prefix_ + "_wide", stream_qos,
    std::bind(&CameraPerceptionNode::pillar_wide_callback, this, std::placeholders::_1));
  sub_pillar_tele_ = this->create_subscription<br_perception::msg::CameraPillar>(
    pillar_topic_prefix_ + "_tele", stream_qos,
    std::bind(&CameraPerceptionNode::pillar_tele_callback, this, std::placeholders::_1));

  sub_spot_wide_ = this->create_subscription<br_perception::msg::CameraBuildingSpot>(
    spot_topic_prefix_ + "_wide", stream_qos,
    std::bind(&CameraPerceptionNode::spot_wide_callback, this, std::placeholders::_1));
  sub_spot_tele_ = this->create_subscription<br_perception::msg::CameraBuildingSpot>(
    spot_topic_prefix_ + "_tele", stream_qos,
    std::bind(&CameraPerceptionNode::spot_tele_callback, this, std::placeholders::_1));

  // ── 订阅：深度 ──
  // ⚠️ 深度用 depth=1（与 camera_depth_node 的发布端一致）——
  //    晚到的深度帧没有价值, 堆积只会让下游拿到更旧的数据
  sub_depth_ = this->create_subscription<br_perception::msg::CameraDepth>(
    depth_topic_, rclcpp::QoS(1).best_effort(),
    std::bind(&CameraPerceptionNode::depth_callback, this, std::placeholders::_1));

  // ── 发布 ──
  pub_frame_ = this->create_publisher<br_perception::msg::CameraFrame>(
    frame_topic_, stream_qos);
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    diag_topic_, 10);

  // ── 组帧心跳 ──
  // ⚠️ 定时器驱动（见头文件口径①）: 帧率与任何一个检测器的耗时**无关**。
  //    慢的那路只会让 stale_sec 变大, 不会拖慢整帧。
  frame_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / frame_rate_hz_),
    std::bind(&CameraPerceptionNode::frame_tick, this));

  last_diag_time_        = std::chrono::steady_clock::now();
  last_future_warn_time_ = std::chrono::steady_clock::time_point{};

  RCLCPP_INFO(this->get_logger(),
    "CameraPerceptionNode ready | in=%s_{wide,tele} %s_{wide,tele} %s_{wide,tele} %s | "
    "out=%s | frame=%.1fHz diag=%.1fHz | slots=%zu",
    mustika_topic_prefix_.c_str(), pillar_topic_prefix_.c_str(),
    spot_topic_prefix_.c_str(), depth_topic_.c_str(), frame_topic_.c_str(),
    frame_rate_hz_, diag_rate_hz_, kSlotCount);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 槽写入（模板 —— 6 个检测回调 + 深度回调共用）
// ═══════════════════════════════════════════════════════════════════════════════
template <typename MsgT>
void CameraPerceptionNode::store_slot(size_t slot,
                                      std::shared_ptr<MsgT> SlotData::*member,
                                      const typename MsgT::SharedPtr& msg,
                                      uint8_t expected_cam)
{
  if (!msg || slot >= kSlotCount) return;

  std::lock_guard<std::mutex> lock(slot_mutex_);
  SlotData& s = slot_data_[slot];
  s.*member = msg;

  // ── camera_id 一致性检查 ──
  // 槽由**话题后缀**定（_wide/_tele），消息里又自带 camera_id —— 重映射接错线
  // 或发布端写错时两者会打架。不查就是无声的错误数据；查了至少有一条日志。
  // ⚠️ 深度消息没有 camera_id 字段，这个检查对它是空操作（用 if constexpr 挡掉）。
  if constexpr (std::is_same_v<MsgT, br_perception::msg::CameraMustika> ||
                std::is_same_v<MsgT, br_perception::msg::CameraPillar> ||
                std::is_same_v<MsgT, br_perception::msg::CameraBuildingSpot>) {
    if (msg->camera_id != expected_cam) {
      if (!s.warned_cam_mismatch) {
        s.warned_cam_mismatch = true;
        RCLCPP_ERROR(this->get_logger(),
          "槽 %s 收到 camera_id=%u 的消息（该槽由话题后缀隐含为 %u）—— "
          "话题重映射接错线，或发布端写错了 camera_id。下游按槽取用会张冠李戴",
          slot_name(slot), static_cast<unsigned>(msg->camera_id),
          static_cast<unsigned>(expected_cam));
      }
    }
  }

  // ── 时间戳 ──
  // ⚠️ 取 `msg->header.stamp`（曝光时刻），**不取到达时刻** —— 任务书 §四
  //    要求所有检测结果的时间戳是曝光时刻；到达时刻含处理与传输抖动，
  //    正是下游要判断的东西，混进来会掩盖真实时序问题。
  // ⚠️ 用 get_clock()->now() 而不是 this->now()：前者不取节点的回调锁，
  //    可以安全地在持 slot_mutex_ 时调用。
  const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
  if (stamp_is_unset(stamp)) {
    // 发布方没填时间戳。退回到达时刻 —— 不理想，但比让下游按"1970 年"算年龄强。
    // ⚠️ 按槽告警（不能用 RCLCPP_WARN_ONCE：wide/tele 共用同一个模板实例，
    //    mustika 的 wide 报过之后 tele 就静默了）。
    if (!s.warned_zero_stamp) {
      s.warned_zero_stamp = true;
      RCLCPP_WARN(this->get_logger(),
        "槽 %s 的消息 header.stamp 为 0（发布方未填时间戳）—— 本节点退回用到达时刻。"
        "请去对应检测器补齐 header.stamp = 曝光时刻",
        slot_name(slot));
    }
    s.stamp = this->get_clock()->now();
  } else {
    s.stamp = stamp;
  }
  s.has_data = true;
}

// ── 6 个检测回调（模板实例化 + 定槽）──
void CameraPerceptionNode::mustika_wide_callback(
    const br_perception::msg::CameraMustika::SharedPtr msg)
{
  store_slot(slot_of(kKindMustika, kCamWide), &SlotData::mustika, msg, kCamWide);
}

void CameraPerceptionNode::mustika_tele_callback(
    const br_perception::msg::CameraMustika::SharedPtr msg)
{
  store_slot(slot_of(kKindMustika, kCamTele), &SlotData::mustika, msg, kCamTele);
}

void CameraPerceptionNode::pillar_wide_callback(
    const br_perception::msg::CameraPillar::SharedPtr msg)
{
  store_slot(slot_of(kKindPillar, kCamWide), &SlotData::pillar, msg, kCamWide);
}

void CameraPerceptionNode::pillar_tele_callback(
    const br_perception::msg::CameraPillar::SharedPtr msg)
{
  store_slot(slot_of(kKindPillar, kCamTele), &SlotData::pillar, msg, kCamTele);
}

void CameraPerceptionNode::spot_wide_callback(
    const br_perception::msg::CameraBuildingSpot::SharedPtr msg)
{
  store_slot(slot_of(kKindSpot, kCamWide), &SlotData::spot, msg, kCamWide);
}

void CameraPerceptionNode::spot_tele_callback(
    const br_perception::msg::CameraBuildingSpot::SharedPtr msg)
{
  store_slot(slot_of(kKindSpot, kCamTele), &SlotData::spot, msg, kCamTele);
}

void CameraPerceptionNode::depth_callback(
    const br_perception::msg::CameraDepth::SharedPtr msg)
{
  // 深度只有一路 —— camera_id 恒为 wide（`CameraDepth` 也没有该字段）
  store_slot(kSlotDeepIdx, &SlotData::depth, msg, kCamWide);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 组帧心跳
// ═══════════════════════════════════════════════════════════════════════════════
void CameraPerceptionNode::frame_tick()
{
  // ⚠️ 耗时用**单调时钟**（steady_clock），不用 ROS 时间 ——
  //    仿真里 ROS 时间会暂停/跳变，拿它量时长会量出负数或天文数字，
  //    然后误报"组帧超阈值"。
  const auto t0 = std::chrono::steady_clock::now();

  auto frame = std::make_unique<br_perception::msg::CameraFrame>();

  // ── 取快照 ──
  // ⚠️ 只在这一小段持 slot_mutex_，**发布在锁外** —— 发布要走 DDS，可能阻塞，
  //    拿着锁发布会把 7 个回调一起卡住。
  std::array<SlotData, kSlotCount> snap;
  {
    std::lock_guard<std::mutex> lock(slot_mutex_);
    snap = slot_data_;
  }

  // ── 基准时刻 ──
  // `newest_stamp` 只是**帧的时间戳**（"这批结果里最新那条的曝光时刻"），
  // 它**不是** stale_sec 的基准 —— 见下方。
  const rclcpp::Time now_ros = this->get_clock()->now();

  rclcpp::Time newest_stamp = now_ros;
  bool any = false;
  for (size_t i = 0; i < kSlotCount; ++i) {
    if (!snap[i].has_data) continue;
    // ⚠️ `!any` 是必须的 —— 不能用 `newest_stamp` 的初值（now_ros）去比较：
    //    真实曝光时刻**总是早于**组帧时刻，若拿 now_ros 当比较基准，**一个槽都填不进去**，
    //    于是所有槽都判成"没有数据"(NEVER)。这里第一个有数据的槽无条件入位。
    if (!any || snap[i].stamp > newest_stamp) {
      newest_stamp = snap[i].stamp;
      any = true;
    }
  }

  // 有数据 → 帧时间戳用最新曝光时刻；一条都没有 → 用组帧时刻（否则会是 ROS 纪元 0）
  frame->header.stamp = any ? newest_stamp : now_ros;
  // ⚠️ frame_id **刻意留空** —— 本帧装的是两台不同相机各自的 2D 结果，
  //    没有共同的坐标系可言；填个名字等于虚构一个**没有 TF** 的坐标系，
  //    下游照它做变换必然失败且无提示。理由见 msg/CameraFrame.msg。
  frame->header.frame_id = "";

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️⚠️ stale_sec 的基准 = **组帧时刻（now_ros）**，不是 newest_stamp。
  //
  //    这是个改过一次的地方，原实现拿 newest_stamp 当基准，后果是：
  //    所有检测器**同时停发**时，newest_stamp 也跟着停住不前，那个"最新槽"的
  //    stale_sec 就恒为 0、恒报 FRESH —— **恰恰把"某路检测器死了"这个要暴露的
  //    失效模式掩盖掉**，而那正是本字段存在的唯一理由。
  //    现在：只要检测器停发，所有槽的 stale_sec 就一起往上走，一眼可见。
  // ═══════════════════════════════════════════════════════════════════════════
  frame->frame_age_sec = any
    ? static_cast<float>((now_ros - newest_stamp).seconds())
    : 0.0f;
  // 留一份给诊断（同源同值）—— 见头文件里这个成员的注释
  last_frame_age_sec_ = static_cast<double>(frame->frame_age_sec);

  // ── 填充三个检测数组（下标 = camera_id，各定长 2）──
  // ⚠️ 消息里声明的是 `CameraMustika[2]`（**定长**），生成的 C++ 类型是
  //    `std::array<...>`，不是 vector —— 所以**没有 resize**，也不用手工清零：
  //    消息默认构造时已把整个数组值初始化了。
  //    未收到数据的槽就保持默认构造的 `detected=false`，其"有没有数据"
  //    由 slot_status[].state 表达（见口径④）。
  for (uint8_t cam = 0; cam < kCamCount; ++cam) {
    const auto& m = snap[slot_of(kKindMustika, cam)].mustika;
    if (m) frame->mustika[cam] = *m;
    const auto& p = snap[slot_of(kKindPillar, cam)].pillar;
    if (p) frame->pillars[cam] = *p;
    const auto& b = snap[slot_of(kKindSpot, cam)].spot;
    if (b) frame->building_spots[cam] = *b;
  }

  // ── 逐槽状态 ──
  // ⚠️ 没有 resize —— `SlotStatus[8]` 生成 `std::array<SlotStatus, 8>`（定长数组
  //    在 rosidl_generator_cpp 里映射成 std::array，不是 vector），默认构造即零初始化。
  //    长度由上面的 static_assert 在编译期与 kSlotCount 对齐。
  size_t populated = 0;
  size_t stale_cnt = 0;

  for (size_t i = 0; i < kSlotCount; ++i) {
    auto& st = frame->slot_status[i];
    // ⚠️ 深度的 tele 槽恒无数据（深度只有一路），如实标 NEVER 即可
    st.kind      = static_cast<uint8_t>(i / kCamCount);
    st.camera_id = static_cast<uint8_t>(i % kCamCount);

    // ⚠️ 判定逻辑在 `judge_slot()`（static 纯函数，可被单测直接喂）——
    //    这里只做搬运，**不再内联一份判据**（判据只实现一次，那是防漂移的唯一办法）。
    if (!snap[i].has_data) {
      const auto v = judge_slot(false, 0.0, fresh_tolerance_sec_);
      st.state = v.state;
      st.stale_sec = v.stale_sec;   // ⚠️ NEVER 时本字段无意义（见 msg/SlotStatus.msg）
      st.note = v.note;
      continue;
    }

    // ⚠️ 负值 = 该槽时间戳在**未来**（时钟跳变/驱动异常）。
    //    **如实报负值，不钳到 0** —— "时间是负的"本身就是要暴露的异常。
    const double age = (now_ros - snap[i].stamp).seconds();
    st.stale_sec = static_cast<float>(age);

    // ── FRESH / STALE 的分界 ⚠️⚠️ 这里改过一次 ──
    //   判据是 `age > fresh_tolerance_sec_`，**不是 `age > 0`**。
    //   曝光时刻**永远早于**组帧时刻 ⇒ age 恒为正数 ⇒ 拿 `age > 0` 当判据会让
    //   `state` **永远到不了 FRESH**，与 msg/SlotStatus.msg 的字段定义
    //   （FRESH = "当前" / STALE = "比本帧基准旧"）和
    //   doc/待编译验证清单_20260920.md §I 的验收判据直接打架 ——
    //   那样的槽永远报 STALE，FRESH 这一档等于不存在。
    //   ⚠️ 负值（时间戳在未来）算 FRESH 不算是旧：它是"钟不对"，不是"数据旧"，
    //      该异常由下面的全局告警 + stale_sec 的负值本身暴露。
    const auto v = judge_slot(true, age, fresh_tolerance_sec_);
    st.state = v.state;
    st.stale_sec = v.stale_sec;   // ⚠️ 如实报（可能是负的，不钳 0）
    st.note = v.note;
    ++populated;
    if (v.state == br_perception::msg::SlotStatus::S_STALE) ++stale_cnt;
  }

  // ── 深度 ──
  if (const auto& d = snap[kSlotDeepIdx].depth) {
    frame->depth = *d;
    // ⚠️ 提炼出来，免得每个下游都得记得去查 depth.mode 这个内嵌枚举
    frame->depth_measuring =
      (d->mode == br_perception::msg::CameraDepth::MODE_MEASURE);
  } else {
    frame->depth_measuring = false;
  }

  // ── 时间戳在未来的告警（节流，全局一条；槽级异常已在 slot_status 里如实报出）──
  if (any && frame->frame_age_sec < -future_tolerance_sec_) {
    if (!throttled(last_future_warn_time_, 5.0)) {
      RCLCPP_WARN(this->get_logger(),
        "检测结果的时间戳比当前 ROS 时间还未来 %.3fs —— 时钟跳变或驱动异常？"
        "该异常已如实写进 slot_status[].stale_sec（负值）",
        -frame->frame_age_sec);
    }
  }

  // ── 统计 ──
  frame->assembly_ms = static_cast<float>(
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count());

  const float assembly_ms = frame->assembly_ms;

  pub_frame_->publish(std::move(frame));   // ⚠️ move 之后 frame 不可再用

  // ⚠️ 用 assembly_ms 这个局部量 —— frame 已在上面 move 掉，读它必崩。
  if (assembly_ms > assembly_warn_ms_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "组帧耗时 %.2fms 超过阈值 %.2fms（组帧本身不等待任何检测器 —— "
      "检查消息体是否变大）", assembly_ms, assembly_warn_ms_);
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️ 下面这些统计量**只在定时器线程里读写**（frame_tick 是唯一调用点，
  //    publish_diagnostics 是唯一读点，且由本函数同步调用）——
  //    **没有跨线程竞争，故不加锁**。
  //    原实现用了 diag_mutex_，换来的只是个"调用方必须持锁、被调方不许加锁"
  //    的非递归锁陷阱（而且真的踩过一次自死锁）。去掉锁，陷阱随之消失。
  // ═══════════════════════════════════════════════════════════════════════════
  ++frame_seq_;
  last_assembly_ms_ = static_cast<double>(assembly_ms);
  last_populated_   = populated;
  last_stale_count_ = stale_cnt;

  // ── 诊断（低频，节流用单调时钟）──
  const auto now_tp = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now_tp - last_diag_time_).count() >=
      1.0 / diag_rate_hz_) {
    publish_diagnostics();
    last_diag_time_ = now_tp;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════════
void CameraPerceptionNode::publish_diagnostics()
{
  // ⚠️ 本函数**不加任何锁**，理由见 frame_tick() 末尾那段 —— 所有被读的量都
  //    只由定时器线程写，且本函数只在定时器线程里被调用。
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "camera_perception";
  st.hardware_id = "camera_layer";

  // ⚠️ 分级只看"**是不是彻底收不到东西**"这一件事 —— 与口径③一致：
  //    本节点不做"够不够新"的判断（那是下游的政策）。
  //    ⚠️ 现场"某路相机本来就没装"是常态，那种情况**不该刷 WARN** ——
  //       所以 0 < populated < kSlotCount 只报 OK，把计数放进 values 里让人看。
  if (last_populated_ == 0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "一个槽都没有数据";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "OK";
  }

  auto kv = [](const std::string& k, const std::string& v) {
    diagnostic_msgs::msg::KeyValue p;
    p.key = k;
    p.value = v;
    return p;
  };

  st.values.push_back(kv("frame_seq", std::to_string(frame_seq_)));
  st.values.push_back(kv("populated_slots",
                         std::to_string(last_populated_) + "/" + std::to_string(kSlotCount)));
  st.values.push_back(kv("stale_slots", std::to_string(last_stale_count_)));
  st.values.push_back(kv("assembly_ms", std::to_string(last_assembly_ms_)));
  st.values.push_back(kv("frame_rate_hz", std::to_string(frame_rate_hz_)));

  // ⚠️ 这里再取一次槽快照 —— 与组帧那次**不是同一份**，无需严格一致：
  //    诊断是 1Hz 的人读信息，不是控制输入。
  std::array<SlotData, kSlotCount> snap;
  {
    std::lock_guard<std::mutex> lock(slot_mutex_);
    snap = slot_data_;
  }
  // 依次列出每个槽有没有来过数据, 便于现场一眼看出是哪路没起
  for (size_t i = 0; i < kSlotCount; ++i) {
    st.values.push_back(kv(
      std::string("slot_") + std::to_string(i) + "_" + slot_name(i),
      snap[i].has_data ? "has_data" : "none"));
  }

  arr.status.push_back(st);

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️ 端到端**年龄**（2026-09-26 加）—— **与雷达那路同口径、同键名**。
  //
  //   为什么要单开一条、而不是让人去 `CameraFrame.frame_age_sec` 里看：
  //   对"三路总延迟 < 50ms"对账时，雷达在**诊断**里、相机在**数据消息**里 ——
  //   要开两种工具才能并排看。这里补一份同名的，`ros2 topic echo` 一次就能对。
  //
  //   ⚠️⚠️ **键名必须与 `lidar_perception_node` 的 `end_to_end_latency`
  //      逐字一致**（`age_newest_ms` / `period_ms`）—— 名字不一致的"对齐"
  //      本身就在制造歧义。
  //
  //   ⚠️ **相机没有"最旧/最新"之分**：整帧在同一瞬间曝光，所以只报一个 age。
  //      雷达那边一帧横跨 100ms，必须报两个 —— **这个差别是本质的，别去"统一"它**。
  //
  //   ⚠️ 阈值 56/80 与 `config/perception_params.yaml` 的
  //      `latency_warn_ms` / `latency_error_ms` 同源（那正是 §15.3 给视觉的 46+10/…）。
  // ═══════════════════════════════════════════════════════════════════════════
  diagnostic_msgs::msg::DiagnosticStatus lx;
  lx.name = "end_to_end_latency";
  lx.hardware_id = "camera_layer";

  const double age_ms = last_frame_age_sec_ * 1000.0;
  if (age_ms < 56.0) {
    lx.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    lx.message = "Latency OK";
  } else if (age_ms < 80.0) {
    lx.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    lx.message = "Latency over warn (56ms)";
  } else {
    lx.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    lx.message = "Latency over error (80ms)";
  }
  lx.values.push_back(kv("age_newest_ms", std::to_string(age_ms)));
  // ⚠️ `last_populated_ == 0` 时 `frame_age_sec` 是 0（没有数据 ⇒ 无从谈年龄），
  //    会被读成"延迟极好" —— **恰好把"一路都没数据"这个要暴露的情况盖住**。
  //    所以照 `camera_perception` 那条的做法，把槽计数一并放进 values；
  //    真正判"有没有数据"看上面那条 `camera_perception` 的 `populated_slots`。
  lx.values.push_back(kv("populated_slots",
                         std::to_string(last_populated_) + "/" + std::to_string(kSlotCount)));

  arr.status.push_back(lx);

  pub_diagnostics_->publish(arr);
}

}  // namespace camera
}  // namespace br_perception

// ⚠️ `RCLCPP_COMPONENTS_REGISTER_NODE` 由这个头文件定义 —— 漏了它不是"未定义标识符"
//    那么直白，而是 "expected constructor, destructor, or type conversion at end of input"
//    （2026-09-23 首次编译实测，浪费一轮）。lidar_perception_node.cpp 也是这个写法。
#include <rclcpp_components/register_node_macro.hpp>

// ═══════════════════════════════════════════════════════════════════════════════
// 组件注册（容器化时被容器加载; 也生成同名独立可执行）
// ═══════════════════════════════════════════════════════════════════════════════
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::CameraPerceptionNode)
