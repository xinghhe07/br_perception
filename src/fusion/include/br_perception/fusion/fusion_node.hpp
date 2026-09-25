#pragma once

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/header.hpp>

#include <br_perception/msg/camera_frame.hpp>
#include <br_perception/msg/fusion_state.hpp>
#include <br_perception/msg/lidar_result.hpp>
#include <br_perception/msg/master_command.hpp>

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "br_perception/fusion/fusion_decisions.hpp"
#include "br_perception/fusion/kalman_tracker.hpp"
#include "br_perception/fusion/target_associator.hpp"

namespace br_perception {
namespace fusion {

/**
 * @brief **时间同步环形缓冲**（规格 §4.5：「保留最近 500ms 的数据」）
 *
 * ⚠️ 纯数据类，**不碰 ROS** ⇒ 可被 `test/` 直接喂。
 *
 * ⚠️ 语义（**这三条都是容易含糊的地方，写死**）：
 *   · 窗口是**按时间戳**算的，不是"存 N 条"。帧率一变，N 条覆盖的时长就变了
 *     ⇒ 用 N 条当窗口的实现在掉帧时会静默退化。
 *   · `closest(t)` 在窗口内找**时间戳最接近**的一条；**并列时取更早的**
 *     （两条一样近，宁可用旧的那条也不要用未来的）。
 *   · **超窗的在 `push` 时就清掉**，不等到查询 —— 免得缓冲无界增长。
 */
template <typename T>
class TimeSyncBuffer
{
public:
  explicit TimeSyncBuffer(double window_sec = 0.5) : window_sec_(window_sec) {}

  /// 追加一条，并顺手清掉超出窗口的
  void push(double t_sec, const T& value)
  {
    // ⚠️ **乱序到达也要能处理**：后到的帧未必时间戳更大（多话题的到达顺序不保证）。
    //    策略是"按时间戳插到该在的位置"，而不是"无脑 push_back" ——
    //    后者会让乱序帧永远排错位置，而 `closest` 只看首尾就会漏掉它。
    auto it = buf_.begin();
    while (it != buf_.end() && it->t <= t_sec) ++it;
    buf_.insert(it, Entry{t_sec, value});

    // ⚠️⚠️ **窗口锚点取"缓冲里最新的那条"，不是"刚压入的这条"**（2026-09-25 实测修正）。
    //    用刚压入的做锚点，一条**迟到的旧帧**会把锚点一起拉旧 ⇒
    //    `prune` 的反向清理就会把**更新的合法帧**当成"未来帧"清掉
    //    ⇒ **迟到一帧就把正当数据清了**。这条是被单测抓出来的（见台账）。
    //    缓冲按时间戳有序 ⇒ `back()` 就是最新那条。
    if (!buf_.empty()) prune(buf_.back().t);
  }

  /// 清掉比 `now_sec` 早超过一个窗口的
  void prune(double now_sec)
  {
    while (!buf_.empty() && (now_sec - buf_.front().t) > window_sec_) {
      buf_.pop_front();
    }
    // ⚠️ 再兜一道：**比 `now_sec` 晚超过一个窗口的**也要清。
    //    正常不会出现（时间戳来自同一时钟），但**时钟跳变**时会出现 ——
    //    不清的话它会永远留在缓冲里、永远被 `closest` 选中。
    while (!buf_.empty() && (buf_.back().t - now_sec) > window_sec_) {
      buf_.pop_back();
    }
  }

  /// 窗口内**时间戳最接近** `t_sec` 的那条；并列取**更早**的；没有返回 `nullptr`
  [[nodiscard]] const T* closest(double t_sec) const
  {
    if (buf_.empty()) return nullptr;
    const Entry* best = nullptr;
    double best_d = 0.0;
    for (const auto& e : buf_) {
      const double d = (e.t > t_sec) ? (e.t - t_sec) : (t_sec - e.t);
      // ⚠️ **严格小于**才换 ⇒ 并列时保留**先遇到的**（= 更早的那条）
      if (best == nullptr || d < best_d) {
        best = &e;
        best_d = d;
      }
    }
    return best ? &best->v : nullptr;
  }

  [[nodiscard]] std::size_t size() const { return buf_.size(); }
  [[nodiscard]] bool empty() const { return buf_.empty(); }
  [[nodiscard]] double window_sec() const { return window_sec_; }

private:
  struct Entry { double t; T v; };
  std::deque<Entry> buf_;
  double window_sec_;
};

/**
 * @brief 融合层主节点（模块 21 第二刀）
 *
 * ── 它做什么 ──────────────────────────────────────────────────────────────
 * ```
 *   /perception/lidar/result  ─┐
 *                              ├─→ [时间同步缓冲] → 19 关联 → 18 追踪 → 规则判定
 *   /perception/camera/frame  ─┘                                        │
 *                                    ┌──────────────────────────────────┘
 *                                    ↓
 *                    /perception/fusion/state  +  /perception/fusion/diagnostics
 * ```
 *
 * ⚠️ **它不发 `/perception/output`** —— 见 `msg/FusionState.msg` 顶部的说明
 *    （用户 2026-09-25 拍板：`perception_pipeline` 保有上行权）。
 *
 * ── 三处**已知的边界**（都写在代码里，不是遗漏）────────────────────────────
 * 1. ⚠️ **相机那一路目前只进缓冲、不参与判定**。规格 §4.4 的"逐目标融合"
 *    （雷达给位置、相机给颜色/语义）里，**穆斯蒂卡那一条已在模块 20 落了**
 *    （`mustika_pillar_check`），其余（天空方块 top_color 等）**归 `perception_pipeline`**——
 *    它已经在做"用相机确认色覆盖雷达 top_color"那件事了（见模块 27 行）。
 *    ⇒ 本节点再做一遍就是**第二处真值来源**，那正是要避免的。
 * 2. ⚠️ **`in_shared_zone` 的映射来自参数**（`shared_spot_ids`），不是从 yaml 现读 ——
 *    默认值 6 个位按 `config/field_geometry.yaml` 第 11 节的顺序给，**换场地要改**。
 * 3. ⚠️ **BRESET 联动 / dwell 统计没做**（规格 §4.5 的两条 v1.2/v1.6 增量）——
 *    它们要 `protocol_decoder` 的事件输入，见台账。
 */
class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void on_lidar(const br_perception::msg::LidarResult::SharedPtr msg);
  void on_camera(const br_perception::msg::CameraFrame::SharedPtr msg);
  /// 「v1.6」收到主控的 `BR_RESET(0x10)` ⇒ 清移动目标 track（规则 5.3.2）
  void on_from_master(const br_perception::msg::MasterCommand::SharedPtr msg);

  /// 从雷达的建筑位数组造圣所判定的输入（⚠️ `state==4` 映射成 `known=false`）
  [[nodiscard]] std::vector<FusionDecisions::SpotState> build_spots(
      const br_perception::msg::LidarResult& msg) const;

  /// 该 id 是不是共享区建筑位（来自参数 `shared_spot_ids`）
  [[nodiscard]] bool is_shared_spot(std::uint32_t id) const;

  /// 按 `building_spot_x/y/z` 建**常驻 track** 并铺出 `known_regions_`
  /// ⚠️ 启动时调一次 —— 这些是**场地常量**，不该每帧重建。
  void build_known_regions();

  void publish_diagnostics(double ms, std::size_t tracks, std::size_t obs,
                           std::size_t new_cand);

  // ── 参数 ──
  std::string lidar_topic_{"/perception/lidar/result"};
  std::string camera_topic_{"/perception/camera/frame"};
  /// ⚠️ `comm_node` 把主控来的帧**原样转发**到这个话题（`comm_node.cpp:240`
  ///    「不解析、不丢弃」）⇒ `BR_RESET(0x10)` 也在这条路上。
  ///    ⚠️ 它**必须**是本节点的输入 —— 否则那条链是断的（见 `on_from_master`）。
  std::string from_master_topic_{"/communication/from_master"};
  /// BR_RESET 时**要保留**的 track 类型（穆斯蒂卡等）。
  /// ⚠️ 默认只留穆斯蒂卡（`type == 4`）—— 建筑位那批靠 `on_known_spot` 保，
  ///    不靠这个列表（那条是规则 5.3.2 的硬要求，不该由参数决定）。
  std::vector<int64_t> reset_keep_types_{4};
  std::string state_topic_{"/perception/fusion/state"};
  std::string diag_topic_{"/perception/fusion/diagnostics"};

  /// 时间同步窗口（规格 §4.5：保留最近 500ms）
  double sync_window_sec_{0.5};
  /// 共享区建筑位 id（⚠️ 默认按 `field_geometry.yaml` 第 11 节的顺序；换场地要改）
  std::vector<int64_t> shared_spot_ids_{4, 5, 6, 7, 8, 9};

  /// ⚠️ **建筑位坐标 = 场地事实**，来自 `config/field_geometry.yaml` 第 11 节
  ///    （`field_geometry.yaml` 的参数块是 `/**` 通配 ⇒ launch 传它就能匹配上，
  ///     与 `lidar_perception_node` / `sky_block_detector` 读的是**同一组键**）。
  ///    ⚠️ **不是占位值** —— 这三个数组是官方场地真值（`building_spot_*`）。
  std::vector<double> building_spot_x_;
  std::vector<double> building_spot_y_;
  std::vector<double> building_spot_z_;
  /// 落区认领的半径（m）。⚠️ 与建筑位的 `spot_half_size`（0.25）同量级；
  ///    取太大会让相邻位互相抢（L2 四位间距 2.5m，取 0.35 安全）。
  double region_radius_m_{0.35};

  FusionDecisions::SanctuaryParams sanctuary_params_;
  FusionDecisions::HoldingParams   holding_params_;
  FusionDecisions::MatchEndParams  match_end_params_;
  FusionDecisions::BoundsParams    bounds_params_;
  KalmanTracker::Params            tracker_params_;
  TargetAssociator::Params         assoc_params_;

  bool   enable_tracking_{true};
  double camera_timeout_sec_{0.5};

  // ── 状态 ──
  rclcpp::Subscription<br_perception::msg::LidarResult>::SharedPtr sub_lidar_;
  rclcpp::Subscription<br_perception::msg::CameraFrame>::SharedPtr sub_camera_;
  rclcpp::Subscription<br_perception::msg::MasterCommand>::SharedPtr sub_from_master_;
  /// 收到过几次 BR_RESET（诊断）
  std::uint32_t reset_count_{0};
  rclcpp::Publisher<br_perception::msg::FusionState>::SharedPtr pub_state_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  TimeSyncBuffer<br_perception::msg::CameraFrame> camera_buf_;
  KalmanTracker tracker_;
  /// 已知位置（建筑位）—— **启动时建一次**，喂给 19 的落区认领
  std::vector<TargetAssociator::KnownRegion> known_regions_;
  /// ⚠️ 比赛自计时（规则 4.2.2）：**首次收到雷达结果**时才起表 ——
  ///    拿节点启动时刻当起点会让"起得早"变成"感知以为比赛开始了"。
  bool   match_started_{false};
  double match_start_sec_{0.0};
};

}  // namespace fusion
}  // namespace br_perception
