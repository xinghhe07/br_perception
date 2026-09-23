#pragma once

/// ═══════════════════════════════════════════════════════════════════════════════
/// mustika_pillar_check.hpp — **最小融合**：相机与雷达交叉确认「穆斯蒂卡在柱上」
/// （任务书 §15.5「融合最早可用节点」，模块 20 `lidar_camera_fusion` 的第一刀）
///
/// ── 为什么先做这个（§15.5 原话）──
///   「完成模块 5 (mustika_detector) + 模块 17 (coordinate_transformer) 后，
///     即可做最小融合验证：**相机检测到排球 → TF 投影到雷达坐标系 →
///     和雷达圆柱检测的空间位置比对 → 确认 mustika_on_pillar 状态**。」
///   两个前置**都已完成**，所以这是**最早能做、且唯一不依赖真实标定就能先把管线跑通**的融合。
///
/// ── 它值多少分（这才是它排在前面的理由）──
///   规则 4.5.1：**TR 取五色石的前置是圣所使命达成**；而"柱上有没有球"是
///   §3.0.1 表里点名的一项得分动作（**250 分**）。雷达能测到柱顶**有东西**，
///   但**分不清是球还是随便一块方块** —— 差的就是视觉这一路。
///   ⇒ 这个节点要回答的，正是"**雷达看到的那个凸起，是不是球**"。
///
/// ── 两边各自说什么 ──
///   · **雷达**（`LidarResult.mustika_pillar`）：柱子的 3D 位置、高度、直径，
///     以及 `has_object_on_top`（柱顶**有没有东西**）。
///     ⚠️ 更正一句**容易想当然的**话：雷达**并非**"完全说不了是什么" ——
///     `cylinder_detector.cpp:593` 其实算了 `top_object_radius`（顶部簇半径），
///     Ø210 的球（r≈0.105）与 Ø350 的方块（r≈0.175+）**在半径上分得开**。
///     **但代码里没有做这个判别** —— 而 `:657-669` 的 RViz 可视化**直接把它画成球**，
///     等于**把"柱上是个球"当成已知事实画出来**。所以"是不是球"这件事目前
///     **没有人验过**：雷达测了半径但没判，可视化替它假设成了球。
///     ⇒ 本节点的价值正在这里：**用一个独立传感器（相机）去证实那个假设**。
///   · **相机**（`CameraMustika`）：球心的**像素坐标 (u,v)** 与**像素半径 r**。
///     单目没有深度，但球**直径已知（Ø210mm）** ⇒ 可由像素半径反推距离：
///         `d ≈ f · D / (2r)`   （f = 焦距[px]，D = 球直径[m]）
///     这是**已知尺寸单目测距**的标准做法，精度随 r 变小而变差（误差 ∝ 1/r²），
///     所以它只够做**交叉确认**，不够做抓取引导 —— 那要 D435i 的深度。
///
/// ── 判据（两个方向都对上才算）──
///   ① **位置**：相机估出的球心（TF 到 world 后）与雷达给的柱顶位置距离 < `pos_tolerance_m`
///   ② **高度**：球心高度落在「柱顶 + `center_above_pillar_top`」附近
///      （球是**坐在** Ø180 凹槽口上，不是掉进去 —— 球 Ø210 比槽 Ø180 还大）
///
///      ⚠️ **这个常数从 `config/field_geometry.yaml` 的
///      `objects.mustika.seated_on_receptacle.center_above_pillar_top` 读，不要在本文件里重算。**
///      （它现在 = **0.054** m。原文本写的是 "√(R²−r²) ≈ 54mm" ——
///        球径一改那个式子就跟着变，而**球径本身还没拍板**
///        （排球 Ø210 vs 规则 V1 的 4 号足球 Ø200），两处必然对不上。
///        引用场地文件的那个键，改一处就够了。）
///   两个都过 → `在柱上`；否则 → `未知`（**不是"不在柱上"**，见下）。
///
/// ── ⚠️⚠️ 三条必须守住的边界 ──
///
///   **① 判不了就说"未知"，不说"不在"；但"真掉在地上"是另一回事**
///
///   本节点输出三档：`1 = 在柱上` / `5 = 掉落` / `0 = 未知`。
///   **永远不输出"不在柱上"** —— 因为"没看到"的原因太多了（球被遮挡 / 相机没对准 /
///   阈值没调好 / 那一路根本没起），把它们一律判成"不在"是**把不确定性当成结论**。
///   下游拿 `0` 应理解为"**这条信息不可用**"，而不是"球不在"。
///   ⚠️ `PerceptionFrame.mustika_state` 的 0 本来就定义为"未知" ✓ 语义对得上。
///
///   ⚠️ **但 `5 = 掉落` 与 `0 = 未知` 必须分开**（2026-09-23 复核建议，采纳）：
///   有一种情况**不是不确定，而是正观测** —— 相机**明确检出**了球，
///   经 TF 后位置**远离柱子且贴近地面**（条件见 `fallen_dist_m_` / `fallen_height_max_m_`）。
///   这时"球掉在地上"是**看到了才说的**，与"没看到所以不知道"是两回事。
///   ⇒ 决策层据此能区分：
///     · `0` → 信息不可用，**别据此决策**（也别据此认为球没了）
///     · `5` → 球真掉了 → 走捡回 / 请裁判归还流程（规则 6.5）
///   两者**不矛盾**：把"基于没看到的否定"禁掉，不等于连"基于看到的肯定"也禁掉。
///
///   **② ⚠️⚠️ 外参未标定时，结果必须标成不可信**
///   **内参**（`focal_px` / `cx_px` / `cy_px`，事实源在 `config/camera_hardware.yaml`）
///   与**外参**（`bringup_full` 从 `config/fusion_params.yaml` 的 `static_tf:` 块
///   读出来广播的那组静态 TF）**目前都是占位值**。
///   ⚠️ 2026-09-23 更正：原写"`bringup_full` 里**写死的**静态 TF" —— 同一轮已把它
///      改成**读 `static_tf:` 块**（那块现在才是唯一事实源）。照旧文字去改 launch
///      会改错文件。用占位外参算出来的"位置一致"是**巧合**，不一致才是常态 ——
///   而这个节点最危险的地方就在这儿：**它可能碰巧对，然后被当成标定过了**。
///   ⇒ 设参数 `extrinsics_calibrated`（**默认 false**）。为 false 时：
///     · 照样计算、照样发诊断（**诊断里给出残差，这正是标定要盯的数**）
///     · 但 `mustika_state` **恒发 0**，绝不发 1
///     · 诊断 level = WARN，message 写"外参未标定，结果不可信"
///   ⚠️ 标定完成后**必须显式把它改成 true** —— 这是个人工闸，故意的：
///      让"开始信任这个节点"是一个**明确动作**，而不是某天悄悄变成真的。
///
///   **③ 外参取自 TF 树，不读 `camera_hardware.yaml`**
///   理由：`transform_to_world()` 走 TF，而 TF 树里 `base_link ← camera_wide_frame`
///   由 `bringup_full` 的 `static_transform_publisher` 提供 —— **那才是运行期的事实源**。
///   再去读 `camera_hardware.yaml` 就有了两处外参，正是本仓库反复吃亏的形态
///   （见 §零 第一宗旨的"稳"）。
///   ⚠️ **代价要说清**：外参标定完之后，**要改的是
///      `config/fusion_params.yaml` 的 `static_tf:` 块**（2026-09-23 起它是运行期
///      唯一事实源 —— launch 直接读它广播）。**不是 launch 文件里的数值**（那里已经没有数值了）。
///      ⚠️ 而 `camera_hardware.yaml` 的 `extrinsics` 是给**标定脚本**读写的记录，
///      **没有任何 C++ 读它**（核实：只有 `scripts/calibrate_camera.py` /
///      `calibrate_lidar_camera.py` 两个 Python 读）⇒ 标定脚本写完**不会自动生效**，
///      要人工把值搬进 `static_tf:` 块。**这条不一致已知、待一并解决**（正解是让标定
///      脚本多写一份运行期 params），别在本节点里再造一个第三处。
///
/// ── 输入 / 输出 ──
///   订：`/perception/camera/mustika_wide`、`/perception/camera/mustika_tele`（CameraMustika）
///       `/perception/lidar/result`（LidarResult）
///   发：`/perception/fusion/mustika_state`（`std_msgs/UInt8`，取值同 PerceptionFrame.mustika_state）
///       `/perception/fusion/mustika_check`（DiagnosticArray，**给人看**：残差、两边各说了什么）
///
///   ⚠️ **为什么状态走一条独立的 `UInt8`**：它的唯一消费者是 `perception_pipeline`
///      （往 `PerceptionFrame.mustika_state` 里填）。用一条窄接口，比让本节点
///      直接去组装 `PerceptionFrame`（那要凑齐 15 个字段，一半它填不了）干净得多。
///
/// 线程安全：单线程执行器（默认），三个订阅与定时器同属一个 mutually-exclusive group，
///           故状态无需加锁 —— 与 `camera_perception_node` / `perception_pipeline` 同口径。
/// ═══════════════════════════════════════════════════════════════════════════════

#include <rclcpp/rclcpp.hpp>

#include "br_perception/msg/camera_mustika.hpp"
#include "br_perception/msg/lidar_result.hpp"

// ⚠️ **不 include coordinate_transformer.hpp** —— 本节点只用只读 TF 查询，
//    不用那个类（它会连带起一个 TF 广播器，见成员 `tf_buffer_` 的注释）。
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>

// ⚠️⚠️ **这个头不是可选的**（2026-09-23 复核抓出）：
//    `tf2_ros::Buffer::transform<PointStamped>()` 内部调 `tf2::doTransform`，
//    而 `geometry_msgs::msg::PointStamped` 的那个重载**声明在 tf2_geometry_msgs 里**。
//    CMake 的 `ament_target_dependencies` 只给编译/链接路径，**不会**把重载带进来。
//    ⚠️ 本仓库此前**只有 `coordinate_transformer.cpp:4` 一处 include 过它**，
//       而那个类的调用方（如 perception_pipeline）走的是它的成员函数、不直接调
//       `Buffer::transform` ⇒ "照抄依赖列表"**不足以**保证本文件能编过。
//    ⚠️ 与 hpp 里"漏宏头会报得像语法错"那条不同：这条报的是
//       `no matching function for call to 'doTransform'` —— 是**响的**。
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>   // std::optional —— range_from_pixel_radius / pillar_top_ 用
#include <string>

namespace br_perception {
namespace fusion {

/**
 * @brief 「穆斯蒂卡在柱上吗」的相机-雷达交叉确认（任务书 §15.5）
 *
 * 详见本文件头 —— **特别是"三条必须守住的边界"那节**：
 * 判不了说未知 / 未标定不许发可信结论 / 外参只认 TF。
 */
class MustikaPillarCheck : public rclcpp::Node
{
public:
  explicit MustikaPillarCheck(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /// @brief 输出状态（与 `msg/PerceptionFrame.msg` 的 `mustika_state` **逐值一致**）
  ///        ⚠️ 本节点**只会发这三档**，其余档位（2=TR转运中 / 3=BR持有 /
  ///           4=已供奉 / 6=重试归还中）它**无从判断** —— 那些要看到 TR/BR 的动作。
  enum State : uint8_t {
    kUnknown = 0,    ///< 未知 —— **"这条信息不可用"，不是"球不在"**（见头文件边界①）
    kOnPillar = 1,   ///< 在柱上（位置与高度两个方向都验过）
    kFallen = 5,     ///< **球掉在地上** —— 正观测，不是不确定（条件见两个 fallen_* 参数）
  };

  /// @brief 相机路数（下标 = `CameraMustika.camera_id`，**不可重排**）
  static constexpr size_t kCamWide = 0;
  static constexpr size_t kCamTele = 1;
  static constexpr size_t kCamCount = 2;

private:
  // ═══════════════════════════════════════════════════════════════════════════
  // 回调
  // ═══════════════════════════════════════════════════════════════════════════
  void on_camera(uint8_t cam, const br_perception::msg::CameraMustika::SharedPtr msg);
  void on_lidar(const br_perception::msg::LidarResult::SharedPtr msg);

  /// @brief 主心跳：判定 → 发布状态与诊断
  void tick();

  // ═══════════════════════════════════════════════════════════════════════════
  // 判定
  // ═══════════════════════════════════════════════════════════════════════════

  /// @brief 用**已知球径**从像素半径反推距离：`d = f · D / (2r)`
  /// @return 距离 [m]；`r <= 0` 或焦距非法 → `std::nullopt`
  static std::optional<double> range_from_pixel_radius(double radius_px, double focal_px,
                                                       double ball_diameter_m);

  /// @brief 像素坐标 + 距离 → **相机坐标系**下的 3D 点（针孔反投影）
  ///
  /// ⚠️ 得到的是**光心坐标系**下的点（+Z 朝前、+X 右、+Y 下）—— 与该相机在 TF 树里的
  ///    frame（如 `camera_wide_frame`）**方向约定必须一致**。若将来发现 TF 里那个 frame
  ///    是光学系而这里按机身系算，结果会整体错 90° 且**不报错** —— 标定时盯残差。
  static void back_project(double u, double v, double depth, double fx, double fy,
                           double cx, double cy, double* xyz);

  /// @brief 一路相机的判定结果（诊断要用，所以把中间量都留着）
  struct CameraVerdict {
    bool     used{false};          ///< 这一路的数据够不够判（false = 没数据/没检出/算不出来）
    double   range_m{0.0};         ///< 反推的距离 [m]
    /// **与雷达柱位的【水平】残差** [m] —— 标定时盯这个数。
    /// ⚠️ 刻意只算水平距离：位置与高度是**两条互相独立**的判据，
    ///    把高度混进来的话，"球在柱正上方但高了半米"会同时让两条都不通过，
    ///    诊断里就看不出**到底错在哪一条**了。
    double   residual_m{-1.0};
    /// 球心在 world 系里的高度 [m] —— ⚠️ **判"掉落"要用它**：
    /// "水平距离远"只说明球不在柱那儿（可能是被人拿走了），
    /// **加上"且贴近地面"才是"掉在地上"这个正观测**（见头文件边界①）。
    double   ball_z_world{0.0};
    bool     position_ok{false};
    bool     height_ok{false};
    std::string note;              ///< 为什么没用 / 差在哪（人读）
  };

  /// @brief 对**一路**相机做位置与高度两个方向的比对
  CameraVerdict judge_one(size_t cam,
                          const std::optional<geometry_msgs::msg::Point>& pillar_top);

  /// @brief 汇总所有路的判定 → 最终状态 + 诊断文案
  ///
  /// ── ⚠️ **两路相机的合并规则（原先没写，2026-09-23 复核指出后补上）** ──
  ///   原则：**两路是"互相印证"，不是"任一路通过就算"**（宽角与长焦看的是同一个球，
  ///   它们**独立**给出距离估计 —— 这是零成本的交叉验证，白扔可惜）。
  ///
  ///   ① **任一路明确反对 → 不能判 `kOnPillar`**。反对 = 该路**算得出残差且超限**
  ///      （注意与"没数据/没算出"区分：那不算反对，只算缺席）。
  ///   ② **判 `kOnPillar` 要求**：至少一路通过（位置+高度都过，且残差在限内），
  ///      **且没有任何一路反对**。
  ///   ③ **两路都算出了距离时，加一条一致性检查**：两者距离差 / 较小者 > `cross_check_rel_`
  ///      ⇒ 降级为 `kUnknown`（相机之间都打架，说明标定或检测有问题，**别硬判**）。
  ///   ④ **长焦是距离主判**：f 大 ⇒ 同样的球在像素上更大 ⇒ 由 `d = f·D/(2r)` 反推的
  ///      相对误差更小。宽角主要提供**存在性/冗余**。两路都可用时，
  ///      **诊断里两个数都打**，但一致性检查按上面第③条走。
  ///   ⑤ `kFallen` 同样要求**没有一路反对**（一路说"在柱上"、另一路说"在地上"时
  ///      必须降级为 `kUnknown` —— 那是矛盾，不是结论）。
  /// @param vs 两路的判定结果 —— **由调用方算一次传进来**。
  ///           ⚠️ 原先 `decide()` 内部自己调 `judge_one`，而 `tick()` 为了出诊断
  ///              又调了一遍 ⇒ **同一次心跳里 `judge_one` 跑两遍**：既浪费，
  ///              两次结果还可能不一致（中间若来了新消息），诊断说的和判定用的就对不上了。
  uint8_t decide(const std::array<CameraVerdict, kCamCount>& vs, std::string& why) const;

  /// @brief 两路相机距离估计的**相对一致性**门槛（见 `decide` 第③条）
  ///        ⚠️ 0.3 = 两路差超过 30% 就不判 —— 标定后按实测收紧
  double cross_check_rel_{0.3};

  // ═══════════════════════════════════════════════════════════════════════════
  // 参数
  //
  // ⚠️ 名字与 `config/fusion_params.yaml` 的 **`mustika_pillar_check:`** 块逐字一致。
  //    ⚠️ 是**按节点名作用域**（不是 `mustika_check:` 这种自造前缀）——
  //       仓库体例同 `comm_node:` / `perception_pipeline:` / `camera_perception:`。
  //    ⚠️ 本行**原先声称"与 mustika_check: 块逐字一致"，而那个块当时根本不存在**
  //       （2026-09-23 复核抓出的"注释里的事实没核实"）。块已在 `.cpp` 落地时补上。
  // ⚠️ 名字以本类成员为准（改 yaml 时照抄代码，别反过来让代码迁就 yaml）。
  // ⚠️ 这些参数**全都得靠 yaml 显式赋予**才生效 —— 漏传 `fusion_params.yaml`
  //    会静默吃代码默认值，而其中 `focal_px_` / `cx_px_` / `cy_px_` 是**占位内参**。
  //    自动核对：`python3 scripts/check_param_consumers.py` 的 D 段。
  // ═══════════════════════════════════════════════════════════════════════════
  std::string wide_topic_{"/perception/camera/mustika_wide"};
  std::string tele_topic_{"/perception/camera/mustika_tele"};
  std::string lidar_topic_{"/perception/lidar/result"};
  std::string state_topic_{"/perception/fusion/mustika_state"};
  std::string diag_topic_{"/perception/fusion/mustika_check"};

  /// @brief ⚠️⚠️ **人工闸：外参是否已标定**（默认 `false`）
  ///
  /// 为 `false` 时本节点**只发 `kUnknown`**，绝不发 `kOnPillar` ——
  /// 理由见头文件边界②：用占位外参算出的"一致"是巧合，而这个节点最危险的地方
  /// 就是**它可能碰巧对，然后被当成标定过了**。
  /// 标定完成后**显式改成 `true`** —— 让"开始信任它"是一个明确动作。
  bool extrinsics_calibrated_{false};

  /// @brief 球直径 [m] —— 穆斯蒂卡 = **Mikasa V300W 排球 Ø≈210**
  ///        ⚠️ 规则 V1（国内赛中文版）说五色石是 **4 号室内五人制足球**，
  ///           色号 218-165-32；而 ABU v1.1 原文是排球。**两个口径未拍板** ——
  ///           改这个值之前先看 `doc/模块进度.md` 与记忆里的"五色石"那条。
  ///           直径差 10mm 会让测距差约 5%，对"是不是在柱上"这个判断不致命。
  double ball_diameter_m_{0.21};

  /// @brief 焦距 [px]（宽 / 长焦，下标 = camera_id）
  ///        ⚠️ 目前是**占位值**，与 `config/camera_hardware.yaml` 的 `camera_matrix` 对应。
  ///           标定后要**两处一起改** —— 这份是运行期用的，那份是人看的记录。
  ///           （台账已记"camera_hardware.yaml 没有任何 C++ 读它"这笔债。）
  std::array<double, kCamCount> focal_px_{{800.0, 2380.0}};
  /// @brief 主点 [px]（宽 / 长焦）
  std::array<double, kCamCount> cx_px_{{640.0, 640.0}};
  std::array<double, kCamCount> cy_px_{{360.0, 360.0}};

  /// @brief 两路相机在 **TF 树里**的 frame 名（下标 = camera_id）
  ///        ⚠️ 必须与 `bringup_full.launch.py` 的 `static_transform_publisher`
  ///           （其值又来自 `fusion_params.yaml` 的 `static_tf:` 块）**逐字一致**。
  ///           对不上 → TF 查不到 → 本节点恒判"未知"（**不报错**，只是永远没结论）。
  ///        ⚠️ 与 `fusion_params.yaml` 的 `camera_wide_frame` / `camera_tele_frame` 同源。
  std::array<std::string, kCamCount> camera_frames_{{"camera_wide_frame",
                                                    "camera_tele_frame"}};

  /// @brief TF 里的世界坐标系名
  std::string world_frame_{"world"};

  /// @brief 球**坐在**柱顶凹槽口时，球心比柱顶高多少 [m]
  ///
  /// ⚠️ **这个值的事实源在 `config/field_geometry.yaml`**：
  ///    `objects.mustika.seated_on_receptacle.center_above_pillar_top`（现 = **0.054**）。
  ///    这里做成参数是为了**别在代码里重算** √(R²−r²) —— 球径一改
  ///    （排球 Ø210 vs 规则 V1 的 4 号足球，**尚未拍板**）那个式子就跟着变，
  ///    两处必然对不上。**改场地那个键时，把这个默认值一起改。**
  double ball_center_above_pillar_top_m_{0.054};

  /// @brief 位置比对容差 [m]：相机估的球心与雷达柱顶的距离超过它就不算"同一处"
  ///        ⚠️ 给得比单目测距误差大 —— 单目靠像素半径反推距离，远处误差可达几十厘米。
  ///           这个值决定"会不会误判成在柱上"，标定时按实测残差收
  double pos_tolerance_m_{0.35};

  /// @brief 高度比对容差 [m]：球心相对预期的偏差容差
  double height_tolerance_m_{0.15};

  /// @brief 检出置信度下限：低于它就不采信这一路的检测结果
  ///        （`CameraMustika.confidence`；`detected == false` 另有一道，见实现）
  double min_confidence_{0.5};

  /// @brief 判"掉落"的两条（**正观测**，见头文件边界①）：
  ///        · 球心离柱的水平距离 > `fallen_dist_m_`
  ///        · **且**球心高度 < `fallen_height_max_m_`（贴近地面 —— 半米以下基本就是掉了）
  /// ⚠️ 两条**同时**满足才判掉落。只满足"远离"可能是**球在别人手里被搬走**
  ///    （那是 2=TR转运中 / 3=BR持有，本节点看不到，所以只能退回 `0 = 未知`）。
  ///    加高度这条就是为了把"在地上"从"不在柱上"里筛出来。
  double fallen_dist_m_{0.8};
  double fallen_height_max_m_{0.4};

  /// @brief 位置残差的**上限**：超过它就认为"这一路与雷达明显不一致"，不采信
  ///
  /// ⚠️ 它同时也是 `extrinsics_calibrated_` 那道人工闸的**数字依据**
  ///    （2026-09-23 复核建议，采纳）：诊断里给出「当前残差 / 本阈值」，
  ///    翻闸时才有个客观的数可看，而不是凭感觉翻。
  ///    ⚠️ 现在给 0.5 是**标定前的宽松值** —— 标定后应按实测残差的 3σ 收紧。
  double residual_max_m_{0.5};

  /// @brief 单目距离的**可信下限** —— 像素半径小于它时误差太大，直接弃用该路
  ///        （误差 ∝ 1/r²：r 减半，距离误差翻四倍）
  ///
  /// ⚠️⚠️ **它与 `pos_tolerance_m_` 是同一件事的两头，不应各自拍**（2026-09-23 复核指出）：
  ///    由 `d = f·D/(2r)` 对 r 求导：`δd = f·D/(2r²) · δr`
  ///    令 `δd ≤ tol` 反解出**该路能用的最小像素半径**：
  ///
  ///        r_min = √( f · D · δr / (2 · tol) )
  ///
  ///    （f=焦距[px]，D=球径[m]，δr=像素半径的测量噪声，经验取 **0.5~1 px**）
  ///
  ///    ⇒ `min_radius_px_` 应当**由容差与噪声推出来**，而不是独立取值；
  ///      改 `pos_tolerance_m_` 时**必须回头重算这个下限**，否则会出现
  ///      "容差收紧了、却仍在采用 r=8px 的粗估"这种自相矛盾的组合。
  ///   ⚠️ 现在是手填值 —— 实现里请**用上面的式子算出实际下限并写进诊断**，
  ///      让人一眼看出"这一路是因为半径太小被弃用的"。
  double min_radius_px_{8.0};
  /// @brief 像素半径的测量噪声估计 [px]（上面那个式子的 δr）
  ///        0.5 = 亚像素级分割；1.0 = 轮廓有毛刺。标定时按实测残差反标定
  double radius_noise_px_{1.0};

  /// @brief 数据保鲜窗口 [s]：相机/雷达的消息超过它就当"没有"，不参与判定
  double stale_sec_{1.0};

  /// @brief TF 查询超时 [s] —— ⚠️⚠️ **不要用 tf2 的默认值 0**
  ///
  /// 理由（2026-09-23 复核指出）：TF 由 `coordinate_transformer` **按里程计时间戳**广播，
  /// 而本节点查询用的是**消息自带的时间戳**，两边**跨进程**。超时给 0 时，只要那一刻
  /// 缓冲区里还没有包住该时刻的样本（起始几帧 / Point-LIO 抖动 / 发布排队），
  /// 就直接抛 `ExtrapolationException` → 判未知 —— 而症状是"雷达柱位不可得"，
  /// 会把现场引去查雷达/标定，**查错方向**。
  /// ⚠️ 同仓库 `coordinate_transformer` 对**同一件事**用的是
  ///    `config/fusion_params.yaml` 的 `tf_timeout_sec: 0.05` —— 这里取同一个值。
  ///    **改那边时把这里一起改**（两个数指同一件事时迟早漂开）。
  double tf_timeout_sec_{0.05};

  /// @brief 诊断与状态发布频率 [Hz]（不是每帧）
  double publish_rate_hz_{2.0};

  // ═══════════════════════════════════════════════════════════════════════════
  // 订阅 / 发布
  // ═══════════════════════════════════════════════════════════════════════════
  std::array<rclcpp::Subscription<br_perception::msg::CameraMustika>::SharedPtr, kCamCount>
      camera_subs_;
  rclcpp::Subscription<br_perception::msg::LidarResult>::SharedPtr lidar_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ═══════════════════════════════════════════════════════════════════════════
  // 运行时状态（单线程回调组内读写，无需加锁）
  // ═══════════════════════════════════════════════════════════════════════════
  std::array<br_perception::msg::CameraMustika::SharedPtr, kCamCount> last_ball_;
  /// @brief 球消息的到达时刻（**单调时钟** —— ROS 时间在仿真里会暂停/跳变）
  std::array<std::chrono::steady_clock::time_point, kCamCount> ball_at_{};

  /// @brief 雷达柱顶位置，**必须已经 TF 到 world 系** —— `nullopt` = 还没收到/
  ///        该柱子未检出（全零哨兵）/ **TF 失败**
  ///
  /// ⚠️⚠️ **构造 = `transform_to_world(mustika_pillar.position 抬到柱顶)`，
  ///    不是直接拿 `mustika_pillar.position`。**（2026-09-23 复核抓到的真 bug：
  ///    本字段的注释原先写"world 系，= mustika_pillar.position 抬到顶部" —— 那是错的。）
  ///
  /// **为什么错得危险**：`/perception/lidar/result` 里的坐标在**传感器系**里 ——
  /// 这是本仓库自己定的（`LidarResult.header.frame_id` **继承输入点云**，
  /// 转 world 是 `perception_pipeline` 的活，见该文件与 `LidarResult.msg` 的注释）。
  /// 照原注释实现，就是**拿传感器系的值当 world 去和相机的 world 球心比距离**。
  /// 而两边若**错着同一个外参**，残差可能**碰巧落进 `pos_tolerance_m_` 的容差** ——
  /// 那正好踩中本文件边界②要防的事：**"碰巧对"比"明显不对"危险得多**。
  ///
  /// ⇒ 两条硬要求（实现里必须都做到）：
  ///   ① 本字段**只在 TF 成功**时才被赋值；TF 失败 → `nullopt` → 判定为"未知"
  ///   ② **两个方向的位置必须同一坐标系** —— 相机的球心也 TF 到 world 之后再比，
  ///      **绝不允许一边 world 一边传感器系**
  ///   ⚠️ 诊断里要**打印两边用到的 frame_id 与 TF 成败** —— 否则这类错配在数字上
  ///      看不出来，只有把"用了哪个系"写出来才查得动。
  std::optional<geometry_msgs::msg::Point> pillar_top_;
  /// @brief 雷达说"柱顶有东西"（`mustika_pillar.has_object_on_top`）
  bool lidar_says_object_{false};
  std::chrono::steady_clock::time_point lidar_at_{};

  /// @brief TF 查询 —— ⚠️ **只读，不广播**（2026-09-23 定）
  ///
  /// ⚠️⚠️ 为什么**不用 `CoordinateTransformer`**：
  ///    那个类（模块 17）在构造时**无条件建一个 `TransformBroadcaster`**
  ///    （`coordinate_transformer.cpp:86`）并广播 `odom→base_link` / `world→odom`。
  ///    而 `perception_pipeline` 已经起了一个 —— 本节点若再起，
  ///    就是**第三个广播同一条变换**，tf2 会刷 `TF_REPEATED_DATA`。
  ///    **本节点是纯消费者，没有任何理由广播。**
  ///    ⚠️ 这笔账要记：`CoordinateTransformer` 现在是"变换 + 广播"捆在一起的，
  ///       用它的地方稍多就会撞车（`fusion_node` 落地时是第四个）。
  ///       它应当拆成"**广播器**（唯一一份）"与"**查询器**（可以有多个）"——
  ///       留给容器化重构/模块 21 一起做，**别在这里第三次复制那个广播器**。
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace fusion
}  // namespace br_perception
