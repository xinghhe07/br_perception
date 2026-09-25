#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace br_perception {
namespace core {

/**
 * @brief 系统健康与**热管理**（模块 26，规格 §6.2）
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ⚠️ 本文件落的是**判定逻辑**（纯函数），节点外壳（订诊断、读 hwmon、发话题）在别处。
 *    理由同 18/19/21：**逻辑能用闭式解手算，节点不能**。
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * ⚠️⚠️ **本模块里最容易写错、代价也最大的是那个温度状态机**
 *
 * 规格 §6.2 的「v1.3+v1.5」原文：
 * > 国内室内场地 + **无风扇迷你主机**，长时间持续高负载下 Zen 3+ 核心温度仍易达 90°C 降频，
 * > **推理延迟 30→50ms 超预算** ⇒ 超过 `cpu_temp_warn_threshold`(85°C) 立即触发降级；
 * > 「v1.3」热恢复：温度回落至 `cpu_temp_recover_threshold`(75°C, **滞回**)并**持续 30s** → 自动恢复
 *
 * ⚠️ **滞回不是可选项**：85 与 75 之间那 10 度是刻意留的**死区**。
 *    没有它，温度在 85 附近抖动 ⇒ **降级/恢复反复切换** ⇒ 相机管线一会儿开一会儿关，
 *    而每一次切换都要重配分辨率与订阅 —— 那比"一直降级"更糟。
 *    ⇒ 单测专门有一条钉它。
 *
 * ⚠️ **30 秒持续**也不是可选项：刚降到 74 度就恢复 ⇒ 负载一上来又冲回 90
 *    ⇒ 同样是在抖。⇒ 低于恢复温度必须**连续**保持。
 *    ⚠️ 而"回到 75~85 的死区"时，那个连续计时**要清零**（不是暂停）——
 *    "持续低于 75" 的字面意思就是不能中断。
 */
class HealthMonitor
{
public:
  // ══════════════════════════════════════════════════════════════════════
  // 告警级别（规格 §6.2 原文的四档）
  // ══════════════════════════════════════════════════════════════════════
  enum class Level : std::uint8_t {
    kOk      = 0,   ///< 绿色：一切正常
    kWarning = 1,   ///< 黄色：指标接近阈值（如 CPU > 80%、延迟 > 70ms）
    kError   = 2,   ///< 红色：模块异常但系统仍在运行（如相机断连、CRC 错误增多）
    kFatal   = 3,   ///< 黑色：系统崩溃，需要紧急停止
  };

  // ══════════════════════════════════════════════════════════════════════
  // ① 热管理状态机
  // ══════════════════════════════════════════════════════════════════════

  enum class ThermalState : std::uint8_t {
    kNormal    = 0,   ///< 全速：双相机 + 全分辨率
    kThrottled = 1,   ///< 降级：关后相机 ONNX + 前相机降分辨率 + 暂停调试点云
  };

  struct ThermalParams
  {
    /// 超过它 ⇒ **立即**降级（规格默认 85°C）
    double warn_c{85.0};
    /// 回落到它以下**并持续** `recover_hold_sec` ⇒ 恢复（规格默认 75°C）
    /// ⚠️ **必须小于 `warn_c`** —— 这是滞回死区；相等或更大就没有滞回，
    ///    温度抖动时状态会反复翻转。
    double recover_c{75.0};
    /// 恢复需要的**连续**低温时长（规格默认 30s）
    double recover_hold_sec{30.0};
  };

  struct ThermalVerdict
  {
    ThermalState state{ThermalState::kNormal};
    /// 本步发生了状态切换（⇒ 该发一次 `/perception/alerts`）
    bool changed{false};
    /// 已经连续低于 `recover_c` 多久（s）
    double hold_sec{0.0};
    /// 本步是否处在**滞回死区**（`recover_c` ~ `warn_c` 之间，不动作）
    bool in_hysteresis{false};
  };

  /// @brief 推进热管理状态机一步
  /// @param cur       当前状态
  /// @param temp_c    当前 CPU 封装温度（°C）
  /// @param hold_sec  上一步的连续低温累计（调用方保存，本函数返回新值）
  /// @param dt        距上一步的时长（s）
  /// @param params    参数
  ///
  /// ⚠️ 判据用**严格**不等号：`temp > warn_c` 才降级（恰好 85.0 **不**降级）、
  ///    `temp < recover_c` 才算低温（恰好 75.0 **不算**）。两条边界都有用例钉住。
  [[nodiscard]] static ThermalVerdict step_thermal(ThermalState cur, double temp_c,
                                                   double hold_sec, double dt,
                                                   const ThermalParams& params);

  /// @brief 参数本身合不合理（**滞回死区必须为正**）
  /// ⚠️ 一个"配置写反了"的静默失效形态：`recover_c >= warn_c` ⇒ 没有死区 ⇒
  ///    温度抖动时状态反复翻转，而**日志里只会看到一串切换**，看不出是配置问题。
  [[nodiscard]] static bool thermal_params_sane(const ThermalParams& params);

  // ══════════════════════════════════════════════════════════════════════
  // ② 告警级别聚合
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 取更严重的那个（规格：整机级别 = 各模块级别里**最坏**的）
  /// ⚠️ 不能取"最新"或"平均" —— 一条 ERROR 被后面的 OK 盖掉，
  ///    症状是**故障刚发生就被标记成正常**。
  [[nodiscard]] static Level worst(Level a, Level b);

  /// @brief 一组级别的聚合
  [[nodiscard]] static Level aggregate(const std::vector<Level>& levels);

  /// @brief 级别 → 可读名（`RCLCPP_*` 的日志与 `/perception/health` 都用它）
  [[nodiscard]] static const char* level_name(Level level);

  // ══════════════════════════════════════════════════════════════════════
  // ③ 各项指标的阈值判定（规格 §6.2 的表）
  // ══════════════════════════════════════════════════════════════════════

  struct MetricParams
  {
    /// CPU 使用率上限（规格："CPU > 80%" ⇒ WARNING）
    double cpu_usage_warn{0.80};
    /// 端到端延迟上限（规格："延迟 > 70ms" ⇒ WARNING）
    /// ⚠️ 与 `perception_params.yaml` 的 `latency_warn_ms`(56) 是**两回事**：
    ///    那个是 pipeline 自己判"这一帧慢了"，这个是**整机健康**的视角。
    double latency_warn_ms{70.0};
    /// 相机帧率下限（低于它 ⇒ ERROR，相机断连/饿死）
    double camera_fps_min{5.0};
    /// 通信 CRC 错误率上限
    double crc_error_rate_warn{0.01};
  };

  static Level judge_cpu_usage(double usage, const MetricParams& p);
  static Level judge_latency(double ms, const MetricParams& p);
  static Level judge_camera_fps(double fps, const MetricParams& p);
  static Level judge_crc_error_rate(double rate, const MetricParams& p);

  // ══════════════════════════════════════════════════════════════════════
  // ④ CPU 温度读取（`/sys/class/hwmon`）
  // ══════════════════════════════════════════════════════════════════════

  /// 一次温度读取的结果
  struct ThermalReading
  {
    /// ⚠️ **读到了没有**。⚠️ `found == false` 时 `temp_c` **无意义** ——
    ///    调用方**不许**把它当 0°C 用（那会让热保护永远不触发，且"看着正常"）。
    bool found{false};
    double temp_c{0.0};
    std::string sensor;   ///< 用的是哪个 hwmon（如 `k10temp`）
    std::string label;    ///< 传感器内部的标签（如 `Tctl`）
  };

  /// @brief 从 `hwmon` 里按**名字**找一个传感器并读它的温度
  /// @param hwmon_root  `hwmon` 根目录（默认 `/sys/class/hwmon`；**测试传临时目录**）
  /// @param sensor_name 要**按名字匹配**的传感器（默认 `k10temp`）
  ///
  /// ⚠️⚠️ **必须按名字找，不能"读第一个"** —— 本机 2026-09-25 实测：
  /// ```
  ///   acpitz   读出来 20.0 °C     ← 未接的 ACPI 占位值
  ///   k10temp  读出来 45.7 °C     ← 真正的 CPU 封装温度（Zen 3+ 的 Tctl）
  /// ```
  ///    天真实现"遍历 hwmon 拿第一个温度"会**永远读到 20°C**
  ///    ⇒ **热保护永不触发**，而症状是"**什么都没发生**" —— 到 90°C 降频了也没人管，**查都没法查**。
  ///
  /// ⚠️ **读不到就如实报 `found = false`**，**不许返回 0**（合 §零「准」轴：
  ///    "没量到"与"量到是 0 度"是两件事）。
  [[nodiscard]] static ThermalReading read_cpu_temp(
      const std::string& hwmon_root = "/sys/class/hwmon",
      const std::string& sensor_name = "k10temp");

  /// @brief 判据：这次读数能不能拿去做热管理
  /// ⚠️ `found == false` ⇒ **不能触发降级，也不能当成"温度正常"** ——
  ///    正确动作是**报一条 WARNING 说"温度读不到"**（那是"不知道"，不是"没事"）。
  [[nodiscard]] static Level judge_thermal_reading(const ThermalReading& r);
};

}  // namespace core
}  // namespace br_perception
