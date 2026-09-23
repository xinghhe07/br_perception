#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// perception_pipeline.hpp — 感知编排与降级薄壳（任务书 §6.1 模块 27）
///
/// **单一职责：把各层的结果组装成"要发给主控的那一帧"，并在某层死掉时如实降级。**
///
/// ── 这个节点存在的**唯一理由**（2026-09-22 核实）──
///   `/perception/output` 是通信层 `comm_node` 唯一的上行入口
///   （`comm_node.cpp:125` 订阅它 → 编码 → 串口），而**全仓库没有任何发布者**。
///   也就是说：雷达链、相机链各自跑到自己的输出就断了，**下位机一帧都收不到**。
///   本节点补上这一环。
///
/// ── 与任务书 §6.1 的**有意偏差**（评审请重点看这一节）──
///   任务书把模块 27 描述成"整个感知系统的大脑"，列了一长串职责。逐条交代去向：
///
///   | 任务书要求 | 去向 | 理由 |
///   |---|---|---|
///   | 管理子节点生命周期 | **launch** | `bringup_full.launch.py` 已在起各节点；ROS 2 另有 lifecycle node。两处都管会互相打架 |
///   | 线程池（2-3 核心线程） | **rclcpp executor** | 容器已用 `component_container_mt`（多线程执行器）。自建线程池会与执行器抢线程并绕过 ROS 回调调度 |
///   | 共享内存 / 零拷贝 | **intra-process comms** | 容器内 `use_intra_process_comms` 已在做。手写会把零拷贝从 ROS 手里抢走，还要自己管生命周期 |
///   | 监听 rqt_reconfigure | **ROS 2 参数回调** | ROS 2 里**没有 rqt_reconfigure**（那是 ROS 1 的）。见下方 `on_set_parameters` |
///   | 通信断开 → 缓存补发 | **comm_node** | 补发要**持有串口链路**才做得到，链路在 comm_node 手里（已有重连+队列）。感知侧再缓存一份是重复，且两份队列的丢弃策略必然不一致 |
///   | 帧率 / 端到端延迟统计 | **`timer_utils`** | §7.3 已实现 `FpsCounter` / `LatencyStats` / `classify_latency`。**本节点只读取并发布，不重复实现** |
///
///   **留下的才是真缺口**：降级决策、紧急停止、最终帧组装、具名诊断。
///
/// ── ⚠️⚠️ 本节点**填不满** `PerceptionFrame`，而且**不许假装填满** ──
///
///   **原则（2026-09-22 用户拍板）：「未知」必须由协议/消息**显式定义**，
///   不能用 `0` 冒充。** 任何"没来源就填 0"都是在**替主控断言一个具体事实**：
///   零值 `PoseStamped` 不是"没填"，是"西南角那个点"；`sanctuary=0` 不是"不知道"，
///   是"使命未达成"→ **TR 永远不去取穆斯蒂卡，白丢 250 分**。这两条都是会直接
///   改变比赛行为的假信息，比不发更糟。
///
///   逐字段交代（**看这张表就知道哪个值能信、哪个还得改协议**）：
///
///   | 字段 | 本节点 | 状态 |
///   |---|---|---|
///   | `header` | 时间戳=最新输入的时刻；`frame_id` 见"坐标系"一节 | ✅ 可信 |
///   | `ego_pose` | `CoordinateTransformer::robot_pose_world()`（模块 17） | ✅ 有里程计时可信 |
///   | `odometry_reliable` | `CoordinateTransformer::odometry_reliable()` | ✅ 可信 |
///   | `mustika_pillar` / `core_pillar` | 透传 `LidarResult` | ✅ 可信 |
///   | `building_spots` / `obstacles` | 透传 `LidarResult`，**TF 到 world** | ✅ 可信（位置不可得时不发，见下） |
///   | `diagnostics` | 本节点 + 各路降级状态 | ✅ 可信 |
///   | `seq` | 自增 | ✅ 可信 |
///   | `mustika_state` | **来自融合层**（订阅 `/perception/fusion/mustika_state`）；没收到/超时 → `0 = 未知` | ✅ 见下方说明 |
///   | `mustika_pose` | 零值 **+ `mustika_state = 0`** | ⚠️ **需协议兜底**：见下 |
///   | `sanctuary_mandate_fulfilled` | **不发**（见下） | ❌ **需协议改** |
///   | `held_object_count` | **不发**（见下） | ❌ **需协议改** |
///   | `holding_over_limit` | `0` | ⚠️ 可接受：风险不对称，恒 0 是**安全默认**（不会误报超限） |
///   | `match_remaining_sec` | 自计时（近似） | ⚠️ **需契约**：以主控计时为准 |
///   | `match_end_release_warning` | 自计时到点且有输入时置 1 | ⚠️ 保守 |
///
///   **`mustika_state` 的来路（2026-09-23 变更）**：
///     原先本节点**写死 `0`**，理由是"归融合层" —— 但那条路**从来没接上过**：
///     `mustika_pillar_check`（模块 20）会发 `/perception/fusion/mustika_state`，
///     而本节点没有订阅它 ⇒ **那个节点的结论再对也到不了主控**。
///     现在订阅了，取值规则三档回退（见实现）：
///       · 收到过、没过 `fusion_timeout_sec_` → 用它的 **0 / 1 / 5**
///       · 没收到过 / 已超时 → **`0 = 未知`**
///     ⚠️ 超时**退回未知，而不是"保持上一次的结论"**：球可能早被拿走了，
///        而主控会照"在柱上"去派 TR。
///     ⚠️ 诊断里会写明这个 `0` 是**"融合层给的真结论"**还是**"回退的未知"**——
///        两者在数值上都是 0，但现场要做的事完全不同。
///
///   **三项必须落到协议文档里（只写在节点注释里没用 —— 编码器不读注释）**：
///
///   ① **`mustika_pose`** —— 零值会被编码器照发成 `int16 mustika_x_cm=0, y=0, z=0`，
///      主控会当成场地西南角。二选一，**必须与主控团队选定一个**：
///        (a) 协议加 `mustika_pose_valid` 位（推荐 —— 见下方 `pose_valid` 那条，
///            两者是同一个机制）；或
///        (b) 写死契约 **"`mustika_state == 0` 时主控必须忽略 `mustika_pose`"**，
///            且编码器在该状态下**跳过这三个字段**的发送。
///
///   ② **`sanctuary_mandate_fulfilled`** —— 该字段**没有未知档**（只有 0/1）。
///      本节点填 0 就是断言"使命未达成"；填 1 更糟（TR 会违规去取）。
///      **正确做法是把这个字段移出上行帧**：圣所使命是"塔建成什么样"的函数，
///      归**融合层**判定；在融合层落地前，**由主控自己维护**。
///
///   ③ **`held_object_count`** —— "0 块"是具体断言，而**夹爪上有几块只有主控知道**，
///      感知链路根本无从观测。它当初被放进上行帧，是因为旧设计假定"感知是大脑"；
///      决策上主机之后这个前提已经不成立（见 `doc/模块进度.md` §四）。
///      **应移出上行帧**，由主控用自己的夹爪状态填。
///
///   ⚠️ 在上面三条落地前，本节点**照常发帧**（否则 `/perception/output` 又是空的），
///      但把这三项**按上述约定填**并且在 `diagnostics` 里**逐条点名**
///      （"sanctuary_mandate_fulfilled: NOT PROVIDED BY PERCEPTION"），
///      让主控侧一旦读错能立刻查到原因，而不是去怀疑感知算法。
///
/// ── ⚠️⚠️ 坐标系：**算不出 world 就绝不发坐标** —— 但绝不等于"静默"（2026-09-22 用户复核后改）──
///
///   `PerceptionFrame.header.frame_id` 的约定是 `"world"`，协议编码器把位置当
///   **世界坐标**写成 int16 cm 发给主控。而雷达层给的是**传感器系**坐标
///   （`LidarResult.header.frame_id`，见该 msg 的注释）。
///   把传感器系坐标当 world 发出去，主控会**按错误位置去走** —— 这是会撞车的错误。
///   **这一条不动：算不出 world，就绝不发那个坐标。**
///
///   ⚠️ 但**原来的"TF 不可用 → 不发帧"是错的**，用户复核指出：那样只是**静默**，不是停车。
///   核实过的事实：`comm_node.cpp:125` **只订阅 `PerceptionFrame` 一个话题**，
///   全仓库对 `/perception/emergency_stop` **零订阅者**；`protocol_encoder.hpp:66` 的
///   `ERROR = 0x03` 至今是"未实现"。也就是说 —— **只发一个 ROS 话题的紧急停止，
///   STM32 什么也收不到**，而且"感知算不出坐标"与"串口线断了"在它眼里**完全不可区分**，
///   可这两件事的应对正相反（一个该等感知，一个该急停失能）。
///
///   所以降级要**落到线上**，共四层，从必须到加分（`[ ]` = 尚未实现，是**待办**）：
///
///   - [ ] **① `comm_node` 订阅 `/perception/emergency_stop` 并落到线上** ——
///         编码成 `0x03`（把那个未实现的包实现掉），或塞进 `0x02` 心跳的一个
///         `degraded/estop` 位。**不做这步，`emergency_stop` 只是给 RViz 和日志看的。**
///   - [ ] **② 协议在 `0x01` 帧里加 `pose_valid` 位** —— 让"**位置不可用，但状态照发**"
///         成为**合法状态**。这样最坏情况主控仍知道：链路活、感知活、只是没坐标 →
///         它能**安全停车**，而不是继续执行上一条命令。
///   - [ ] **③ 从"全丢"降级为"部分丢"** —— 现在是"一个点 TF 失败就整帧不发"
///         （`transform_lidar_to_world` 要求全成功）。有了 `pose_valid` 之后改成
///         "位置列表清空 + `pose_valid=0`，`match_remaining_sec` / 柱状态 / 诊断**照发**"。
///         整帧静默会让主控**连比赛计时都断了**。
///   - [ ] **④ 与 STM32 团队拍死一条契约**：**心跳继续发的前提下，
///         "连续 N ms 没有 0x01 感知帧 → 主控停车"**。
///         若主控没有这个语义，① 就必须靠 ②③ 兜底 —— **这条不拍死，整套降级不成立。**
///
///   **本节点在当前（①②③未落地）时的行为**，如实写死：
///     · TF 可用 → 逐点 `transform_to_world()`，`frame_id = "world"`，正常发
///     · TF 不可用 → **不发 `PerceptionFrame`**（不发错坐标），改为
///       同时发 `emergency_stop` **和**降级诊断，并在日志里明确区分
///       "算不出坐标"与"链路断了"——**这两件事别让现场靠猜**。
///
///   ⚠️ 这条与"通信断开→缓存补发"是两类问题，别混：那是链路断了，这是**算不出正确坐标**。
///
/// ── 降级状态机（任务书 §6.1 那五条）──
///   逐条对着任务书：
///     · 雷达断连   → `kVisionOnly`。⚠️ **但那个模式当前【一帧都不发】** ——
///                    相机是 2D、要融合层才有 3D，而融合层没写。
///                    **它是诊断标签，不是可用模式**，详见 `Mode::kVisionOnly` 的注释
///     · 相机断连   → `kLidarOnly`（天空方块顶面颜色本来就判不了，标 UNKNOWN —— 见
///                    `BuildingSpot.top_color`，雷达层**永远**发 UNKNOWN，不因降级而改变）
///     · 融合异常   → `fusion_bypass = true` → **透传雷达结果**。
///                    ⚠️ 融合层（模块 21）**尚未实现**，所以**现在恒为 true** —— 这是如实反映
///     · 通信断开   → 由 comm_node 自理（见上表）；本节点只**观测**并在诊断里点名
///     · 全部断连   → `kEmergency` → 发 `/perception/emergency_stop`
///
///   ⚠️ **"从未见过"不算故障**：节点刚起来、话题还没来时，各路都是 NEVER。
///      若把它当故障，本节点**一启动就会发紧急停止**。所以紧急停止的条件是
///      "**曾经能用、现在两个环境传感器都没了**"（见实现里的 `ever_ok`）。
///
///   ⚠️ **同理，诊断里 NEVER 也不许报 WARN/ERROR**（2026-09-22 用户要求）：
///      融合层（模块 21）**今天就是不存在的**，`kSrcFusion` 会长期是 NEVER。
///      若把它当故障报黄灯，从今天到模块 21 落地这段时间诊断面板**一直挂黄**，
///      大家很快学会无视它 —— **真出事时就看不见了**。
///      所以 NEVER 只作为**信息**列出（"未接入"），`level` 保持 OK。
///
/// ── 发布契约（★ 数字背后是职责边界，不是"够用"）──
///
///   **① 感知帧不参与底层控制闭环。**
///      这是**契约**，写死在这里，将来有人嫌 10Hz 慢要往上调时先看这条：
///      感知帧是**场景状态**（塔在哪、障碍在哪、比赛还剩多久），不是控制反馈；
///      BR 的底层闭环在**主控自己的编码器/IMU**里。
///      ⚠️ 若真拿它做避障，按 BR 最高 **0.5 m/s**、允许 **5cm** 位置过时反推：
///         预算 = 5cm ÷ 0.5 m/s = **100ms** → 10Hz 刚好卡在线上，
///         再加相机通路 ~43ms + 处理 + 串口，**立刻就超**。
///         所以"嫌慢"时正确的做法是**别用它做闭环**，而不是提高帧率。
///
///   **② 事件补帧**（性价比最高的一条）：场景显著变化时**立即补发一帧**，
///      不等下一个 tick。触发条件见 `event_trigger_enabled_` 的实现。
///      反应延迟从 ≤100ms 掉到接近 0，而**平均带宽不变**。
///
///   **③ 与通信层的发送节奏 —— ✅ 已核实无需去重（2026-09-22，附更正）**
///
///      ⚠️ 本节原先写的是"`send_rate_hz: 50` + 本节点 10Hz ⇒ 同一帧会被连发约 5 次，
///         需要与 comm 约定去重"。**那条断言是错的** —— 当时只看了 `send_rate_hz`
///         这个数就下结论，**没读发送循环**。核实 `serial_manager.cpp:326-440`：
///           · 队列空时发送线程 `wait_for` 超时返回，**什么都不发，不重发**；
///           · `post_frame` 会 `notify_one` **立即唤醒**（`:308` 原注释：
///             "投递到发出的延迟因此不受 send_rate_hz 轮询周期约束"）。
///         ⇒ `send_rate_hz` 只是"多久醒一次看有没有新帧"的**兜底周期**，
///           **有效发送率 = 本节点发布率**。
///
///      ⚠️ 真正要盯的是**事件补帧把线上帧率抬高**：最坏
///         `1 / event_publish_min_interval_sec`（默认 20Hz）。按满载荷 280 B/帧
///         = 5.6 kB/s，而 460800 bps ≈ 46 kB/s，**余量足够**；
///         **改小那个间隔前先重算这条账**。
///
///      ~~下面这两条二选一的约定已作废，留档说明当时的判断错在哪：~~
///        (a) comm 侧改成"**有新帧才发** + 心跳保活"（推荐：省带宽且语义清楚）；或
///        (b) 主控按 `seq` **去重**。
///      ⚠️ 不约定就等着出问题：下游看到重复动作时会**先怀疑到感知头上**，
///         而根因在两边频率没对齐。
///
///   **④ `match_remaining_sec` 以主控计时为准**（契约）：感知自计时是**近似**
///      （裁判蜂鸣器感知无从得知），本节点只用它做 `match_end_release_warning`
///      的**预警**，**不作**主控的计时依据。
///
/// ── 时钟（★ 本节点的两套时钟，别用混）──
///   · **降级判定 / 节流 / 耗时** → `std::chrono::steady_clock`（**单调**）。
///     ROS 时间在仿真里会暂停/跳变，拿它算"多久没来"会得到负数或天文数字，
///     然后**误判整条链路都断了** —— 而仿真恰是最常跑的场景。
///   · **消息时间戳**（`header.stamp`） → ROS 时间，且**继承输入的曝光时刻**，
///     不用 `now()`（任务书 §四 时间戳约定）。
///
/// 线程安全: 回调与定时器**同属一个 mutually-exclusive group**（默认），故状态无需加锁
///           —— 与 `lidar_perception_node` 同口径。⚠️ 将来若改用多线程回调组，
///           必须给共享状态补锁（`camera_perception_node` 的 `slot_mutex_` 是范例）。
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>

#include "br_perception/msg/lidar_result.hpp"
#include "br_perception/msg/camera_frame.hpp"
#include "br_perception/msg/perception_frame.hpp"

#include "br_perception/fusion/coordinate_transformer.hpp"
#include "br_perception/utils/timer_utils.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>   // std::pair —— PublishSignature::spot_states 用
#include <vector>    // std::vector —— 同上

namespace br_perception {
namespace core {

/**
 * @brief 感知编排与降级节点（任务书 §6.1 模块 27）
 *
 * 详见本文件头的设计说明 —— **特别是"填不满 PerceptionFrame"与"坐标系"两节**，
 * 那两条决定了本节点为什么这么写。
 */
class PerceptionPipeline : public rclcpp::Node
{
public:
  explicit PerceptionPipeline(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief 被监视的来源（下标进 `SourceHealth[]`，顺序**不可重排**）
  enum Source : size_t {
    kSrcLidar  = 0,   ///< 雷达流水线（`LidarResult`）
    kSrcCamera = 1,   ///< 相机流水线（`CameraFrame`）
    kSrcFusion = 2,   ///< 融合层 —— ⚠️ **尚未实现**，恒 NEVER（见头文件"降级状态机"）
    kSrcOdom   = 3,   ///< 里程计（`CoordinateTransformer::odometry_state()`）
    kSrcComm   = 4,   ///< 通信链路（`/communication/diagnostics` 的 `connected` 键）
    kSrcCount  = 5
  };

  /// @brief 一路来源的在线状态
  enum SourceState : uint8_t {
    kNever  = 0,   ///< **从没见过** → 配置问题（还没起 / 话题名错），**不是故障**
    kOk     = 1,   ///< 在容差内
    kStale  = 2,   ///< 见过，但超时没更新
    kFailed = 3    ///< 有数据但自报不健康（如 `LidarResult.watchdog_ok == false`）
  };

  /// @brief 降级模式
  ///
  /// ⚠️ 前五个由**环境传感器**的可用性决定；`kNoPose` 是**另一维**（能不能出 world 坐标），
  ///    混进来是因为**对主控而言它同样致命** —— 见下方单列的模式。
  enum Mode : uint8_t {
    kStarting  = 0,   ///< 还没收到**任何**环境传感器数据（启动初期，或话题全配错）
    kNormal    = 1,   ///< 雷达 + 相机都在线
    kLidarOnly = 2,   ///< 相机不可用 → 纯雷达
    /// @brief 雷达不可用 —— ⚠️⚠️ **但本节点当前在这个模式下【一帧都不发】**
    ///
    /// **这不是一个"能跑的模式"，它现在只是个诊断标签。**（2026-09-23 复核指出）
    ///
    /// 事实：雷达不 OK 时 `assemble_frame()` **直接返回空** ⇒ 零帧输出。
    /// 理由是头文件"坐标系"那节的硬约束：相机给的是 **2D**，要经融合层投影成 3D
    /// 才有用，而融合层（模块 21）还没实现 ⇒ **雷达一停，本节点就是没有 3D 内容**，
    /// 如实不发比发一份旧的好。
    /// ⇒ **下游从这个模式得不到任何东西**；主控侧的观感是"完全收不到"。
    ///
    /// ⚠️⚠️ 所以**别读成"降级后仍可用"**。它要变成**真模式**，前提是协议先能表达
    ///     "**位置不可用，但状态照发**"（即 `0x01` 帧加 `pose_valid` 位）。
    ///     在那之前，主控**无法区分**"雷达停了"与"整条链路断了"——
    ///     而这两件事的应对**正相反**（一个该等感知，一个该急停失能）。
    ///     本节点能做的只有：发 `emergency_stop` + 在诊断里**点名**。
    ///
    /// ⇒ 这一条已从"协议待办"升级为**本模式的成立条件**，
    ///    见 `config/comm_params.yaml` 未冻结清单里的 `0x01` 状态位那条。
    kVisionOnly= 3,
    kEmergency = 4,   ///< **曾经能用、现在两个环境传感器都没了**
    /// @brief ⚠️ **曾经能出 world 坐标、现在出不来了** → **零帧输出**
    ///
    /// 为什么必须单列（2026-09-22 用户复核 P0-3）：TF 一断，`assemble_frame()` 恒返回
    /// 空 → **主控一帧都收不到**，而原来的 `decide_mode()` 只看雷达/相机，
    /// 于是诊断里照样报 `NORMAL` / `level=OK` ——
    /// **最严重的失效模式在诊断里完全看不见**，与头文件反复强调的
    /// "不许让诊断撒谎"直接冲突。
    ///
    /// ⚠️ 与 `kStarting` 的区别是**同一套 NEVER 逻辑**：从没出过坐标 = 启动初期
    ///    （Point-LIO 还没起来是常态），**不是故障**，不许报警；
    ///    **出过、又没了**才是 `kNoPose`。
    kNoPose    = 5
  };

  // ── 供诊断 / 日志使用的名字表 ──
  static const char* source_name(size_t src);
  static const char* state_name(uint8_t st);
  static const char* mode_name(uint8_t mode);

private:
  // ═══════════════════════════════════════════════════════════════════════════
  // 回调
  // ═══════════════════════════════════════════════════════════════════════════
  void on_lidar_result(const br_perception::msg::LidarResult::SharedPtr msg);
  void on_camera_frame(const br_perception::msg::CameraFrame::SharedPtr msg);
  void on_comm_diag(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  /// @brief 融合层的穆斯蒂卡状态 → 转进 `PerceptionFrame.mustika_state`
  ///
  /// ⚠️ **原样转发，不做任何加工**：`mustika_pillar_check` 已经把
  ///    "判不了说未知、判得了才说在柱上/掉落"的规矩守住了（见它头文件的三条边界）。
  ///    本节点在这里再判一次，只会多出一处会漂移的判据。
  /// ⚠️ **只认 0/1/5 三档**，其余值（2=TR转运中 … 6=重试归还中）说明上游超出了
  ///    它自己的定义域 → **丢弃并降级为 0**，别把不认识的值透传给主控。
  void on_fusion_state(const std_msgs::msg::UInt8::SharedPtr msg);

  /// @brief 定时器心跳：推进超时判定 → 组帧 → 发布
  void tick();

  /// @brief 组装并按需发布一帧（定时器与输入回调**共用这一段**）
  /// @param is_event true = 由**输入回调**触发的事件补帧
  ///                 （`scene_changed()` 通过且过了防抖闸才真发）；
  ///                 false = 定时器基线帧（到点就发）
  ///
  /// ⚠️ 事件补帧**必须从输入回调发起**，不能在定时器里"提前"发 ——
  ///    变化是在回调那一刻被观测到的，等下一个 tick 就又是 ≤100ms 的延迟，
  ///    等于没做（见头文件发布契约 ②）。
  /// ⚠️ 两条路径**共用本函数**：发布逻辑只写一遍，避免"事件帧和基线帧
  ///    字段不一致"这类只在某个路径上出现的 bug。
  void try_publish(bool is_event);

  // ═══════════════════════════════════════════════════════════════════════════
  // 内部步骤
  // ═══════════════════════════════════════════════════════════════════════════

  /// @brief 记一路"刚收到了数据"
  /// @param src     哪一路
  /// @param healthy 该路自报的健康位（没有这个概念时传 true，只按超时判）
  void mark_seen(size_t src, bool healthy);

  /// @brief 推进所有路的超时判定（`tick()` 开头调）
  void refresh_states();

  /// @brief 由各路状态推导当前模式（**纯函数**，便于单独推理/测）
  uint8_t decide_mode() const;

  /// @brief 组帧失败的原因 —— **必须分开**（2026-09-22 用户复核 P0-1）
  ///
  /// ⚠️ 原先三种原因都算 `skipped_no_tf_` 并共用同一条 WARN（"排查方向：里程计/Point-LIO
  ///    —— 不要去看串口"）。而**现场最常见的恰恰是"相机起了、雷达没起"**，
  ///    那条文案会把排查方向**指错**。而头文件又把 `skipped_no_tf_` 称作
  ///    "现场排查的头号指标" —— 一个指错方向的头号指标比没有还糟。
  enum FrameFailReason : uint8_t {
    kFailNone         = 0,   ///< 成功
    kFailNoLidar      = 1,   ///< **还没有过**任何雷达结果（还没起 / 话题名错）
    kFailLidarNotOk   = 2,   ///< 有雷达结果，但**该路现在不 OK**（STALE / FAILED）
    kFailEmptyFrameId = 3,   ///< 雷达结果的 `frame_id` 为空 → 无从 TF
    kFailTfFailed     = 4    ///< TF 查询失败（里程计/Point-LIO 没起、TF 树不通）
  };

  /// @brief 从当前状态快照组一帧 `PerceptionFrame`
  /// @param[out] reason 失败原因（成功时置 `kFailNone`）——
  ///                    **调用方必须按它分开计数与出文案**，不要合并成一种
  /// @return 组好的帧；无法诚实组帧时返回 `std::nullopt`（**不发帧**，见头文件"坐标系"）
  std::optional<br_perception::msg::PerceptionFrame> assemble_frame(uint8_t& reason);

  /// @brief 发布降级状态（低频；`DiagnosticArray`）
  void publish_degradation_state();

  /// @brief 发布紧急停止（**只在状态变化时发**，避免每秒刷屏；见实现注释）
  void publish_emergency_stop(bool stop);

  /// @brief 把 `LidarResult` 里的目标点逐个 TF 到 world
  ///
  /// @return **全部转成功**才返回 true。
  ///
  /// ⚠️ 这是**当前（协议还没 `pose_valid` 位时）的权宜做法**，不是终态：
  ///    一半世界系一半传感器系的帧比不发还糟，所以宁可整帧不发。
  ///    **协议加上 `pose_valid` 位之后必须改成"部分丢"**（头文件降级那节 ③）：
  ///    位置列表清空 + `pose_valid=0`，而 `match_remaining_sec` / 柱状态 /
  ///    诊断**照发** —— 否则整帧静默会让主控连比赛计时都断了。
  bool transform_lidar_to_world(const br_perception::msg::LidarResult& in,
                                br_perception::msg::PerceptionFrame& out);

  /// @brief 一帧的"显著特征"：只回答"与上次相比变没变"
  ///
  /// ⚠️ 只存**判据用得到的那几项**，不存整帧 —— 存整帧会让这个结构随消息一起
  ///    长大，还要深拷贝。
  /// ⚠️ 定义**必须在这里**（而不是挪到 `last_sig_` 旁边）：它出现在下面
  ///    `make_signature` / `scene_changed` 的签名里，C++ 要求嵌套类型先声明后使用。
  ///    2026-09-23 首次编译实测：定义放后段会报 "PublishSignature does not name a type"。
  struct PublishSignature {
    bool        valid{false};          ///< 还没有过上一帧时为 false
    size_t      obstacle_count{0};     ///< 障碍物个数变了就算变
    double      nearest_obstacle_m{-1.0};  ///< 最近障碍物距离 [m]；<0 = 没有障碍物
    uint8_t     mode{0};               ///< 降级模式翻转（含 kNormal ↔ kLidarOnly）
    /// 建筑位状态：`(spot_id, state)` 的排序列表 —— 塔状态翻转（EMPTY↔TOWER）
    /// 是**比赛里最要紧的事件之一**，必须能触发补帧
    std::vector<std::pair<uint32_t, uint8_t>> spot_states;
    /// 柱顶有无物体：穆斯蒂卡"被取走/被放回"的信号
    bool        mustika_pillar_occupied{false};
    bool        core_pillar_occupied{false};
  };

  /// @brief 由**原始雷达数据**算"场景显著特征" —— **刻意不做 TF、不组帧**
  ///
  /// ⚠️ 为什么用原始数据而不是组好的帧（2026-09-22 用户复核 P1-6）：
  ///    输入回调是 10Hz(雷达) + 30Hz(相机)，若每次都先完整组帧再判"要不要发"，
  ///    等于**每秒白付约 40 次逐点 TF**。而判据用得到的量
  ///    （障碍数 / 最近距离 / 塔状态 / 柱顶有无物体）**在传感器系里就能算**，
  ///    一个 TF 都不用 —— 机器人就在传感器原点附近，**距离在哪个系里算都一样**。
  PublishSignature make_signature(const br_perception::msg::LidarResult& lidar) const;

  /// @brief 与上次发布的特征相比，是否有"值得立即补发"的变化
  ///
  /// ⚠️ 这只是**必要条件**，不是充分条件：调用方**还必须**过
  ///    `event_publish_min_interval_sec_` 那道防抖闸，否则障碍物列表每帧的
  ///    小幅变化会让它退化成"每个输入回调都发"，**正好绕过 10Hz 那个职责边界**。
  bool scene_changed(const PublishSignature& s) const;

  /// @brief 组帧失败时按**原因**出告警（每条原因独立文案 + 独立节流）
  ///
  /// ⚠️ 合并成一条的代价（2026-09-22 用户复核 P0-1）：现场最常见的是
  ///    "相机起了、雷达没起"，而 TF 失败的文案会把排查方向指向里程计 ——
  ///    **指错方向的头号指标比没有还糟**。
  void warn_frame_failure(uint8_t reason);

  // ═══════════════════════════════════════════════════════════════════════════
  // 参数（⚠️ 名字与 `config/perception_params.yaml` 的键**逐字一致**）
  // ═══════════════════════════════════════════════════════════════════════════
  std::string lidar_topic_{"/perception/lidar/result"};
  std::string camera_topic_{"/perception/camera/frame"};
  std::string comm_diag_topic_{"/communication/diagnostics"};
  /// @brief 融合层穆斯蒂卡状态话题（发布端 = `mustika_pillar_check`，模块 20）
  ///        ⚠️ 它的新鲜度判据**复用 `fusion_timeout_sec_`**（下面「各路超时」那组里，
  ///           不另立一个 —— 两个数指同一件事时迟早漂开）。
  ///        ⚠️ 超时后退回 `0 = 未知`（**不是**"保持上一次的结论"）：旧结论比没结论危险
  std::string fusion_state_topic_{"/perception/fusion/mustika_state"};
  std::string output_topic_{"/perception/output"};
  std::string emergency_stop_topic_{"/perception/emergency_stop"};
  std::string degradation_topic_{"/system/degradation_state"};

  /// @brief 组帧发布频率 [Hz]
  ///        ⚠️ 与传感器频率**无关**：慢了的那路只会让它自己的状态变 STALE，不拖慢发布。
  ///           默认 10Hz —— **不是 30**：这是发给下位机的协议帧，串口侧
  ///           `comm_params.yaml` 的 `send_rate_hz` 默认 50，但那是 comm_node 的上限；
  ///           感知帧本身不需要跟相机同频，10Hz 已远超建塔任务的动作带宽。
  double publish_rate_hz_{10.0};

  /// @brief 降级状态发布频率 [Hz]（**不是每帧**）
  double degradation_rate_hz_{1.0};

  /// @brief 各路超时 [s] —— ⚠️ **必须按该路自己的周期给**，不能一律同一个值：
  ///        雷达 10Hz 与通信心跳 1Hz 用同一个超时会**必然误报**。
  double lidar_timeout_sec_{0.5};
  double camera_timeout_sec_{0.5};
  double fusion_timeout_sec_{2.0};
  double odom_timeout_sec_{0.5};
  double comm_timeout_sec_{3.0};

  /// @brief 比赛时长 [s]（用于 `match_remaining_sec` 自计时）
  double match_duration_sec_{180.0};

  /// @brief 组帧耗时超此值告警 [ms]
  double assembly_warn_ms_{5.0};

  /// @brief `emergency_stop` 只在**状态变化**时发布；这是"仍处于紧急状态"时的
  ///        重复发布间隔 [s]（**不是**每帧发）—— 下游可能中途才连上，
  ///        完全不重发会让它永远不知道。0 = 只在变化时发一次。
  double emergency_repeat_sec_{2.0};

  /// @brief 开关**事件补帧**（见头文件发布契约 ②）。默认开。
  ///        ⚠️ 关掉它不会省多少带宽，但会让"障碍物刚进 2m"这类事件
  ///           最多晚 100ms（一个 tick）才被主控看到。
  bool event_trigger_enabled_{true};

  /// @brief 事件补帧的**最小间隔** [s] —— 防抖动的硬闸
  ///        ⚠️ 这个值决定"最坏情况下帧率被悄悄提到多少"：0.05 ⇒ 上限 20Hz。
  ///           调小之前先回头看头文件"感知帧不参与底层控制闭环"那条契约。
  double event_publish_min_interval_sec_{0.05};

  /// @brief 事件补帧判据：障碍物进入这个距离内算"显著变化" [m]
  double event_obstacle_near_m_{2.0};

  // ═══════════════════════════════════════════════════════════════════════════
  // 订阅 / 发布
  // ═══════════════════════════════════════════════════════════════════════════
  rclcpp::Subscription<br_perception::msg::LidarResult>::SharedPtr      sub_lidar_;
  rclcpp::Subscription<br_perception::msg::CameraFrame>::SharedPtr      sub_camera_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_comm_diag_;

  /// @brief 融合层的穆斯蒂卡状态（`mustika_pillar_check` 的输出，模块 20）
  ///
  /// ⚠️⚠️ **这条订阅的意义是"把链接上"**（2026-09-23 补）：
  ///    `mustika_pillar_check` 发 `/perception/fusion/mustika_state`，
  ///    而在补这条之前**本节点根本没有订阅者** —— 也就是说那个节点就算写完，
  ///    它的结论也**到不了 `PerceptionFrame`**，与当初 `/perception/output`
  ///    **零发布者**是同一个形态（方向相反）。
  ///    这正是 §零 第一宗旨"稳"那条要防的：**看着配好了，实际没接上**。
  /// ⚠️ 现在还没有发布者 → 这一路恒为 NEVER → `mustika_state` 仍是 `0 = 未知`，
  ///    **与今天的行为完全一致**（没引入任何回归）。
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_fusion_state_;

  rclcpp::Publisher<br_perception::msg::PerceptionFrame>::SharedPtr     pub_frame_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                     pub_emergency_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr   pub_degradation_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ═══════════════════════════════════════════════════════════════════════════
  // 运行时状态（**只在单线程回调组内读写**，无需加锁 —— 见头文件"线程安全"）
  // ═══════════════════════════════════════════════════════════════════════════
  /// @brief 一路来源的健康记录
  struct SourceHealth {
    /// 最后一次见到的时间（**单调时钟**；`time_point{}` = 从没见过）
    std::chrono::steady_clock::time_point last_seen{};
    bool  healthy{true};      ///< 最近一次自报的健康位
    bool  ever_ok{false};     ///< **曾经**在线过 —— 区分"从没起过"与"后来死了"
    uint8_t state{kNever};    ///< 由 `refresh_states()` 刷新
  };
  std::array<SourceHealth, kSrcCount> health_;

  /// @brief 最近一次 `LidarResult`（透传用；`nullptr` = 还没有过）
  br_perception::msg::LidarResult::SharedPtr last_lidar_;
  /// @brief 最近一次 `CameraFrame` 的**时间戳**（本节点只关心它活没活）
  std::optional<rclcpp::Time> last_camera_stamp_;

  /// @brief 融合层最近给的穆斯蒂卡状态（`0/1/5`；其余值不收，见 `on_fusion_state`）
  uint8_t last_fusion_state_{0};
  /// @brief 上面那个值是什么时候到的（**单调时钟**）—— 用来判"还新不新"
  std::chrono::steady_clock::time_point fusion_state_at_{};

  bool emergency_active_{false};   ///< 当前是否处于紧急状态
  std::chrono::steady_clock::time_point last_emergency_pub_{};

  /// @brief 比赛自计时起点（单调时钟；**首次收到任意环境传感器数据**时启动）
  ///        ⚠️ 这是**近似**：真正的比赛开始由裁判蜂鸣器决定，感知无从得知。
  ///           见 `match_duration_sec_` 的说明。
  std::chrono::steady_clock::time_point match_start_{};
  bool match_started_{false};

  /// @brief 上一次发降级诊断的时刻（单调时钟）
  std::chrono::steady_clock::time_point last_degradation_pub_{};

  /// @brief "组帧耗时超标"这条告警的节流时间戳（单调时钟）
  ///
  /// ⚠️ **必须与 `last_degradation_pub_` 分开** —— 两个不同的告警共用一个节流量
  ///    会互相重置对方的窗口（诊断刚发过，这条就被吞掉，反之亦然），
  ///    现场看到的是"有时报有时不报"，**这种间歇性最难查**。
  ///    ⚠️ 组帧**失败**那几条告警的节流在 `fail_warn_time_`（按原因各一个）。
  std::chrono::steady_clock::time_point last_assembly_warn_{};

  /// @brief "端到端延迟超标"告警的节流时间戳（单调时钟）—— 同上，必须独立
  std::chrono::steady_clock::time_point last_latency_warn_{};

  /// @brief 本节点**能否产出 world 坐标**（P0-3 的独立状态，不再藏在模式里）
  bool pose_available_{false};
  /// @brief **曾经**能产出过 world 坐标 —— 区分"启动初期"与"后来断了"（同 NEVER 的逻辑）
  bool ever_had_pose_{false};

  /// @brief 相机时间戳告警标志（**成员而不是函数级 static**）
  ///        ⚠️ 函数级 `static` 在多线程执行器下是竞态，且同进程多实例会互相影响 ——
  ///           而本文件自己的注释还强调过"要按来源告警"，用 static 恰好违背它。
  bool warned_camera_stamp_{false};

  uint32_t frame_seq_{0};
  uint64_t assembled_frames_{0};   ///< 成功发布的帧数
  uint64_t event_frames_{0};       ///< 其中有多少是**事件补发**的（用来盯带宽有没有失控）
  /// @brief 组帧失败计数，**下标 = `FrameFailReason`** —— 分开计数是 P0-1 的修复核心：
  ///        "还没雷达"（去查话题）与"TF 失败"（去查里程计）**现场要做的事完全不同**，
  ///        合成一个数就只能靠猜。
  std::array<uint64_t, 5> fail_counts_{};
  /// @brief 每条失败原因的**独立节流时间戳**（下标同上）——
  ///        共用一个会让不同原因的告警互相吞掉（见 `last_assembly_warn_` 那段）
  std::array<std::chrono::steady_clock::time_point, 5> fail_warn_time_{};

  /// @brief 上一次**实际发出去**的帧的"显著特征"（事件补帧的比对基准）
  ///        ⚠️ 结构体本身定义在类体前段（`transform_lidar_to_world` 之后）。
  PublishSignature last_sig_;

  /// @brief 上次**事件补帧**的时刻（单调时钟）—— 防抖动闸，见
  ///        `event_publish_min_interval_sec_`
  std::chrono::steady_clock::time_point last_event_pub_{};

  /// @brief 发布帧率 —— **复用 `timer_utils::FpsCounter`**（§7.3 已实现，不重复造）
  br_perception::utils::FpsCounter fps_;

  /// @brief 端到端延迟统计 —— **复用 `timer_utils::StageProfiler`**
  ///
  /// ⚠️ 用 `StageProfiler` 而**不是** `LatencyStats`：后者是个**纯数据 struct**
  ///    （`count`/`last_ms`/`mean_ms`/`min_ms`/`max_ms` 五个字段，**没有任何记录方法**），
  ///    它是 `StageProfiler::stats(idx)` 的**返回值类型**。直接拿它当成员就得自己
  ///    再写一遍累加/最小/最大 —— 那正是"不重复造"要避免的。
  ///    用法：初始化期 `declare("end_to_end")` 拿下标，之后每帧 `record()`。
  br_perception::utils::StageProfiler profiler_;
  /// @brief `profiler_` 里"端到端延迟"那一栏的下标（构造期 `declare` 得到）
  std::size_t idx_end_to_end_{0};
  /// @brief 延迟告警阈值（默认取 `timer_utils` 的 §15.3 / §7.3 值：56ms / 80ms）
  br_perception::utils::LatencyThresholds latency_thresh_;

  /// @brief 里程计 / TF —— 模块 17。本节点**只借它的坐标变换与可靠性判定**，
  ///        不做融合（那是模块 20/21 的活）
  std::unique_ptr<br_perception::fusion::CoordinateTransformer> tf_;
};

}  // namespace core
}  // namespace br_perception
