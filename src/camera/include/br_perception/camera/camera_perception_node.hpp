#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// camera_perception_node.hpp — 相机层聚合"薄壳"（任务书 §15.1 模块 10，P0-2）
///
/// **单一职责：把三个检测器 × 两路相机 + 深度，聚合成"某一时刻的那一帧"。**
///   不做检测（检测器们已经做完了）、不做坐标变换（那是融合层的事）、
///   不碰外参、不碰里程计。**本节点只做时间对齐 + 打包。**
///
/// ── 为什么是"薄壳聚合"而不是"单体编排"（2026-09-21 定，用户选 A）──
///   `LidarPerceptionNode` 是单体：各阶段是私有方法，一个节点串起整条雷达流水线。
///   相机层**刻意不照抄那个形状**，两条理由：
///     ① `camera_depth_node` 有一条不能违背的约束 —— "**独立回调, 不阻塞主感知链**"。
///        并成单体节点、单线程执行器跑，深度处理就会阻塞三个检测器，直接违反约束。
///        相机层拆多节点是 **§3.0 三相机并行的硬性要求**，不是历史遗留。
///     ② 职责上也确实更细：雷达是"一个传感器一条流水线"，相机是"三个传感器、
///        四个正交检测器"，本来就该是可独立起停的多个节点。
///
/// ── 输入话题（7 路，全部 SensorDataQoS）──
///   ⚠️ 下面这些是**前缀**（`*_topic_prefix` 参数），节点会自己拼 `_wide` / `_tele`：
///       {前缀}_wide 与 {前缀}_tele    →  6 路
///     写成带后缀的全名会得到 `..._wide_wide`，**静默收不到任何数据**（不报错）。
///   前缀默认值：
///     mustika_topic_prefix  = /perception/camera/mustika
///     pillar_topic_prefix   = /perception/camera/pillars
///     spot_topic_prefix     = /perception/camera/building_spots
///   不带后缀的那一路：
///     depth_topic           = /perception/camera/depth      (QoS depth=1)
///
/// ── 输出 ──
///   /perception/camera/frame                CameraFrame
///   /perception/camera/frame_diagnostics    diagnostic_msgs/DiagnosticArray
///
/// ── 组帧口径（⚠️ 四条，别改错了）──
///   ① **定时器驱动，不是"等齐了才发"**。帧率由 `frame_rate_hz` 定，与任何一个
///      检测器的耗时**无关** —— 慢的那路只会让它的 `stale_sec` 变大，不会拖慢整帧。
///   ② **`stale_sec` 的基准是"组帧时刻"，不是"最新那个槽的时间戳"**。
///      这是个改过一次的地方：若拿各槽里最新的 stamp 当基准，所有检测器同时停发时
///      那个"最新槽"也跟着停住，于是它恒为 0、恒报 FRESH —— **恰恰把"某路检测器
///      死了"这个要暴露的失效模式掩盖掉**，而那正是该字段存在的唯一理由。
///   ③ **年龄如实报，但"新旧"这一档要有个界**。协议上 `state` 只有三档
///      （NEVER / FRESH / STALE），所以本节点**必须**给出一个 FRESH↔STALE 的分界
///      —— 那是 `fresh_tolerance_sec`（默认 0.5s），可调。**它不是"替下游做决定"**：
///      下游该按 `stale_sec` 的**实际数值**去判断"这个目标多久没更新还能用"，
///      `state` 只用来一眼看出"哪路死了"。两者都发，各司其职。
///   ④ **`detected=false` 的结果照常入帧**，不下沉、不丢弃 —— 它是有效信息
///      （"这路相机看了，没有"）。"没数据"是另一回事，由 `slot_status[].state`
///      表达，**不要**用 `detected` 或数组长度去推断。
///
/// ── 时钟（⚠️ 仿真下会咬人）──
///   两种时间**刻意分开用**：
///     · `header.stamp` / `stale_sec` → **ROS 时间**（必须与消息时间戳同源；
///       `use_sim_time` 下 `rclcpp::Time` 相减要求 clock_type 一致，否则抛异常）
///     · 各种**耗时/节流**（组帧耗时、诊断间隔）→ `std::chrono::steady_clock`
///       因为 ROS 时间在仿真里会**暂停/跳变**，拿它量时长会量出负数或天文数字
///
/// ── 线程安全 ──
///   7 个回调都只写自己那一格 `slot_data_[]`，定时器只读（持 `slot_mutex_` 取快照）。
///   用单线程执行器（默认）时本就无并发；加锁是为了**不依赖执行器配置** ——
///   将来换多线程执行器不会变成数据竞争。
///   ⚠️ 诊断统计量（`last_*`、`frame_seq_`）**只在定时器线程里读写**，无需加锁 ——
///      见 `publish_diagnostics()` 的注释。
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>

#include "br_perception/msg/camera_frame.hpp"
#include "br_perception/msg/camera_mustika.hpp"
#include "br_perception/msg/camera_pillar.hpp"
#include "br_perception/msg/camera_building_spot.hpp"
#include "br_perception/msg/camera_depth.hpp"
#include "br_perception/msg/slot_status.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <tuple>   // std::tuple_size —— 上面 kSlotStatusLen 用

namespace br_perception {
namespace camera {

/**
 * @brief 相机层聚合节点
 */
class CameraPerceptionNode : public rclcpp::Node
{
public:
  explicit CameraPerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief 槽位种类（与 msg/SlotStatus.msg 的 `K_*` 常量**逐值一致**）
  enum Kind : uint8_t {
    kKindMustika = 0,   ///< → CameraFrame.mustika[]
    kKindPillar  = 1,   ///< → CameraFrame.pillars[]
    kKindSpot    = 2,   ///< → CameraFrame.building_spots[]
    kKindDepth   = 3,   ///< → CameraFrame.depth
    kKindCount   = 4
  };

  /// @brief 相机 ID（与 msg/CameraMustika.msg 的 camera_id 逐值一致，**不可重排**）
  static constexpr uint8_t kCamWide = 0;
  static constexpr uint8_t kCamTele = 1;

  /// @brief 相机路数 —— 每个检测结果数组定长 2，**下标 = camera_id**
  static constexpr uint8_t kCamCount = 2;

  /// @brief 内部扁平槽数 = 4 种 kind × 2 路相机 = 8
  ///        ⚠️ 与 `CameraFrame.slot_status` 的长度一致（含恒为 NEVER 的深度 tele 槽）
  static constexpr size_t kSlotCount = kKindCount * kCamCount;

  /// @brief `slot_status` 数组长度 —— **从消息定义反向推导**，不在此处重写一遍。
  ///        这样"kSlotCount 与 msg 里的 [8] 必须相等"就不再是个隐式约定：
  ///        msg 改了长度而这里没跟上 → **编译期**就报（static_assert）。
  /// 用的是 rosidl_generator_cpp 为每个字段生成的公开别名 `_<field>_type`
  ///   （即 `CameraFrame::_slot_status_type`），它对定长数组是
  ///   `std::array<SlotStatus, 8>`、对变长数组是 `std::vector<...>` ——
  ///   两种形态都支持 `std::tuple_size`。
  static constexpr size_t kSlotStatusLen =
      std::tuple_size<br_perception::msg::CameraFrame::_slot_status_type>::value;

  static_assert(kSlotStatusLen == kSlotCount,
                "CameraFrame.slot_status 的长度与 kSlotCount 不一致 —— "
                "改 msg/CameraFrame.msg 的 [8] 时忘了同步这里");

  /// @brief 深度槽的 kind 值（深度只有一路，camera_id 恒为 kCamWide）
  static constexpr size_t kSlotDeepIdx = static_cast<size_t>(kKindDepth) * kCamCount + kCamWide;

  /// @brief 扁平槽下标 → kind 项（`CameraFrame` 的四个成员之一）
  static constexpr size_t slot_of(Kind kind, uint8_t cam)
  {
    return static_cast<size_t>(kind) * kCamCount + cam;
  }

  /// @brief 槽位名（日志与 `SlotStatus.note` 用）
  static const char* slot_name(size_t slot);

private:
  // ⚠️ 前向声明 —— 下面两个模板的签名里要用到，而它的定义在本类靠后的位置
  struct SlotData;

  // ── ROS 回调：6 个检测结果 + 1 个深度 ──
  void mustika_wide_callback(const br_perception::msg::CameraMustika::SharedPtr msg);
  void mustika_tele_callback(const br_perception::msg::CameraMustika::SharedPtr msg);
  void pillar_wide_callback(const br_perception::msg::CameraPillar::SharedPtr msg);
  void pillar_tele_callback(const br_perception::msg::CameraPillar::SharedPtr msg);
  void spot_wide_callback(const br_perception::msg::CameraBuildingSpot::SharedPtr msg);
  void spot_tele_callback(const br_perception::msg::CameraBuildingSpot::SharedPtr msg);
  void depth_callback(const br_perception::msg::CameraDepth::SharedPtr msg);

  // ── 组帧 ──
  /// @brief 定时器心跳：取快照 → 组帧 → 发布
  void frame_tick();

  /// @brief 把新到的消息存进它的槽（⚠️ 只碰 `slot_data_`，不发布）
  ///
  /// 6 个检测回调 + 1 个深度回调走同一段逻辑 —— 差别只有"存哪个成员、哪个下标"，
  /// 所以做成模板按成员指针分派，而不是把同一段写七遍（写七遍就会漏改一处）。
  ///
  /// @param slot       扁平槽下标
  /// @param member     `SlotData` 里对应的成员指针（因消息类型不同而不同）
  /// @param msg        新到的消息
  /// @param expected_cam 该槽**话题后缀**隐含的 camera_id；与消息里自带的
  ///                     `camera_id` 比对，不一致就报（重映射接错线时两者会打架）
  ///
  /// ⚠️ **时间戳取 `msg->header.stamp`，不取 `this->now()`** ——
  ///    任务书 §四「时间戳约定」要求所有检测结果的时间戳是**曝光时刻**。
  ///    到达时刻含处理与传输抖动，正是下游要判断的东西，混进来会掩盖真实时序问题。
  template <typename MsgT>
  void store_slot(size_t slot,
                  MsgT SlotData::*member,
                  const typename MsgT::SharedPtr& msg,
                  uint8_t expected_cam);

  // ── 诊断 ──
  /// @brief 发布诊断数组。⚠️ **只在定时器线程里被调用**，不持任何锁（见实现注释）。
  void publish_diagnostics();

  // ═════════════════════════════════════════════════════════════════════════════
  // 参数
  // ═════════════════════════════════════════════════════════════════════════════
  /// @brief 组帧频率 [Hz] —— **与检测器耗时无关**（见类头 ①）
  double frame_rate_hz_{30.0};
  /// @brief 诊断发布频率 [Hz]（**不是每帧**）
  double diag_rate_hz_{1.0};
  /// @brief 组帧耗时超此值就在诊断里告警 [ms]
  ///        ⚠️ 这只是**组帧本身**的开销（拷 6~7 条小消息），不含检测耗时。
  double assembly_warn_ms_{5.0};

  /// @brief 时间戳"未来多少秒以内"算正常（超过就报，多半是时钟跳变/驱动异常）
  ///        给一点余量：不同主机的时钟不可能严丝合缝
  double future_tolerance_sec_{0.05};

  /// @brief **FRESH / STALE 的分界** [s]：`stale_sec` 超过它就算 STALE
  ///
  /// ⚠️ 这个参数是**必需**的，不是可选的精细化 —— 曝光时刻永远早于组帧时刻，
  ///    `stale_sec` 恒为正数，所以"`age > 0` 就算旧"会让 `state` **永远到不了
  ///    FRESH**（与 `msg/SlotStatus.msg` 的定义直接打架）。
  ///
  /// 容差同时要满足两件相反的事，调之前先想清楚：
  ///   · 比"检测器**正常**周期"大 —— 处理耗时 20~40ms、偶有抖动的健康路不该闪 STALE
  ///   · 比"真停了"短 —— 停掉的检测器要在**人能反应过来的时间内**变成 STALE
  /// 默认 0.5s 是这两条的折中：约 15 帧（30Hz）没更新才算旧。
  double fresh_tolerance_sec_{0.5};

  /// @brief 输入话题**前缀**（⚠️ 不是全名 —— 节点自己加 `_wide` / `_tele` 后缀）
  std::string mustika_topic_prefix_;
  std::string pillar_topic_prefix_;
  std::string spot_topic_prefix_;
  /// @brief 深度话题（这一路**不带**后缀 —— 深度只有一路）
  std::string depth_topic_;
  std::string frame_topic_;
  std::string diag_topic_;

  // ═════════════════════════════════════════════════════════════════════════════
  // 订阅 / 发布
  // ═════════════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<br_perception::msg::CameraMustika>::SharedPtr      sub_mustika_wide_;
  rclcpp::Subscription<br_perception::msg::CameraMustika>::SharedPtr      sub_mustika_tele_;
  rclcpp::Subscription<br_perception::msg::CameraPillar>::SharedPtr       sub_pillar_wide_;
  rclcpp::Subscription<br_perception::msg::CameraPillar>::SharedPtr       sub_pillar_tele_;
  rclcpp::Subscription<br_perception::msg::CameraBuildingSpot>::SharedPtr sub_spot_wide_;
  rclcpp::Subscription<br_perception::msg::CameraBuildingSpot>::SharedPtr sub_spot_tele_;
  rclcpp::Subscription<br_perception::msg::CameraDepth>::SharedPtr        sub_depth_;

  rclcpp::Publisher<br_perception::msg::CameraFrame>::SharedPtr           pub_frame_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr     pub_diagnostics_;

  rclcpp::TimerBase::SharedPtr frame_timer_;

  // ═════════════════════════════════════════════════════════════════════════════
  // 运行时状态
  // ═════════════════════════════════════════════════════════════════════════════
  /// @brief 一个槽的最新一份结果
  ///
  /// ⚠️ 存 SharedPtr **不是**为了省拷贝那么简单 —— 消息里的数组字段本来就是内部
  ///    shared_ptr，但构造/析构仍要走一遍字段树。存指针则组帧时按值拷那一条即可，
  ///    且天然表达"这个槽还是空的"（nullptr）。
  struct SlotData {
    br_perception::msg::CameraMustika::SharedPtr      mustika;   ///< kKindMustika 用
    br_perception::msg::CameraPillar::SharedPtr       pillar;    ///< kKindPillar  用
    br_perception::msg::CameraBuildingSpot::SharedPtr spot;      ///< kKindSpot    用
    br_perception::msg::CameraDepth::SharedPtr        depth;     ///< kKindDepth   用

    /// @brief 该消息 header.stamp（**曝光时刻**，不是到达时刻）
    ///        ⚠️ 必须是 ROS 时间 —— 与组帧时刻相减时 clock_type 要一致，
    ///           否则 rclcpp::Time 相减会抛。
    rclcpp::Time stamp;
    bool         has_data{false};

    /// @brief 该槽是否已经就"时间戳为 0"告警过
    ///        ⚠️ 不能用 `RCLCPP_WARN_ONCE` —— 那个宏按**调用点**只报一次，
    ///           而 wide/tele 共用同一个模板实例，于是 mustika 的 wide 报过之后，
    ///           tele 的同类问题就**静默**了。按槽记才是对的。
    bool         warned_zero_stamp{false};
    /// @brief 该槽是否已经就"camera_id 与槽不符"告警过（同上，按槽记）
    bool         warned_cam_mismatch{false};
  };

  std::mutex slot_mutex_;
  std::array<SlotData, kSlotCount> slot_data_;

  /// @brief 已发出的帧数（诊断用；**只在定时器线程读写**）
  uint64_t frame_seq_{0};

  // ── 最近一次组帧的统计（**只在定时器线程读写**，无需加锁）──
  double last_assembly_ms_{0.0};
  size_t last_populated_{0};
  size_t last_stale_count_{0};

  /// @brief 上次发诊断的**单调时钟**时刻 —— ⚠️ 不用 ROS 时间：仿真里会暂停/跳变
  std::chrono::steady_clock::time_point last_diag_time_;

  /// @brief 上次发"某槽时间戳在未来"告警的时刻（按单调时钟节流）
  std::chrono::steady_clock::time_point last_future_warn_time_;
};

}  // namespace camera
}  // namespace br_perception
