#include "br_perception/core/health_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <dirent.h>
#include <fstream>
#include <string>
#include <sys/stat.h>

namespace br_perception {
namespace core {

namespace {

/// 读整个文件并去掉尾部空白（`hwmon` 的值都是 `12345\n` 这种）
bool read_trimmed(const std::string& path, std::string& out)
{
  std::ifstream f(path);
  if (!f.good()) return false;
  std::getline(f, out);
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r' || out.back() == ' ')) {
    out.pop_back();
  }
  return true;
}

bool is_dir(const std::string& path)
{
  struct stat st{};
  return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

}  // namespace


// ═══════════════════════════════════════════════════════════════════════════
// ① 热管理状态机
// ═══════════════════════════════════════════════════════════════════════════
bool HealthMonitor::thermal_params_sane(const ThermalParams& params)
{
  // ⚠️ **滞回死区必须为正**。写反了（`recover_c >= warn_c`）的症状是
  //    "温度在阈值附近时状态反复翻转"，而**日志里只会看到一串切换** ——
  //    看不出是配置问题。⇒ 专门有个函数让调用方能在启动时喊一声。
  return params.recover_c < params.warn_c;
}

HealthMonitor::ThermalVerdict HealthMonitor::step_thermal(ThermalState cur, double temp_c,
                                                          double hold_sec, double dt,
                                                          const ThermalParams& params)
{
  ThermalVerdict v;
  v.state = cur;
  v.hold_sec = hold_sec;

  // ── 超上限 ⇒ **立即**降级（不等、不累计）──
  // ⚠️ 严格 `>`：恰好 85.0 **不**降级。这条边界要写死并测。
  if (temp_c > params.warn_c) {
    v.hold_sec = 0.0;                       // ⚠️ 高温把连续低温计时**清零**（不是暂停）
    if (cur != ThermalState::kThrottled) {
      v.state = ThermalState::kThrottled;
      v.changed = true;
    }
    return v;
  }

  // ── 低于恢复温度 ⇒ 累计"连续低温"──
  // ⚠️ 严格 `<`：恰好 75.0 **不算**低温。
  if (temp_c < params.recover_c) {
    v.hold_sec = (dt > 0.0) ? (hold_sec + dt) : hold_sec;   // ⚠️ 非正 dt 不推进
    if (cur == ThermalState::kThrottled && v.hold_sec >= params.recover_hold_sec) {
      v.state = ThermalState::kNormal;
      v.changed = true;
      v.hold_sec = 0.0;                     // 恢复后清零，下一轮重新计
    }
    return v;
  }

  // ── 滞回死区（`recover_c` ~ `warn_c`）⇒ **保持当前状态，不动作**──
  // ⚠️⚠️ **这一支就是滞回的全部意义**：没有它，温度在这 10 度里抖动
  //    ⇒ 降级/恢复反复切换 ⇒ 相机管线一会儿开一会儿关，比"一直降级"更糟。
  // ⚠️ 而且要把连续低温计时**清零** —— "持续低于 75" 的字面意思就是**不能中断**：
  //    掉到 74 待了 29 秒、弹回 76 一下、再到 74，那**不是**连续 30 秒。
  v.in_hysteresis = true;
  v.hold_sec = 0.0;
  return v;
}

// ═══════════════════════════════════════════════════════════════════════════
// ② 告警级别
// ═══════════════════════════════════════════════════════════════════════════
HealthMonitor::Level HealthMonitor::worst(Level a, Level b)
{
  // ⚠️ **取更严重的那个**（枚举值越大越严重 ⇒ 取 max 即可，但这个函数的存在
  //    是为了把"为什么是 max 而不是最新/平均"这件事写在代码里）。
  //    ⚠️ 取"最新"的症状：一条 ERROR 被紧随其后的 OK 盖掉
  //    ⇒ **故障刚发生就被标记成正常**。
  return (static_cast<std::uint8_t>(a) >= static_cast<std::uint8_t>(b)) ? a : b;
}

HealthMonitor::Level HealthMonitor::aggregate(const std::vector<Level>& levels)
{
  Level acc = Level::kOk;
  for (const auto l : levels) acc = worst(acc, l);
  return acc;
}

const char* HealthMonitor::level_name(Level level)
{
  switch (level) {
    case Level::kOk:      return "OK";
    case Level::kWarning: return "WARNING";
    case Level::kError:   return "ERROR";
    case Level::kFatal:   return "FATAL";
    default:              return "UNKNOWN";
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// ③ 单项指标
// ═══════════════════════════════════════════════════════════════════════════
HealthMonitor::Level HealthMonitor::judge_cpu_usage(double usage, const MetricParams& p)
{
  // ⚠️ 严格 `>`：恰好 80% **不**告警（规格原文是 "CPU > 80%"）。
  if (usage > p.cpu_usage_warn) return Level::kWarning;
  return Level::kOk;
}

HealthMonitor::Level HealthMonitor::judge_latency(double ms, const MetricParams& p)
{
  // ⚠️ 同样严格 `>`（规格："延迟 > 70ms"）。
  // ⚠️ 负延迟（时间戳在未来）**不**当告警 —— 那是时钟问题，
  //    由 `SlotStatus.msg` 那条口径去暴露，不归这里。
  if (ms > p.latency_warn_ms) return Level::kWarning;
  return Level::kOk;
}

HealthMonitor::Level HealthMonitor::judge_camera_fps(double fps, const MetricParams& p)
{
  // ⚠️ **相机帧率低是 ERROR 不是 WARNING** —— 相机断连/饿死会让整条视觉链
  //    （含那 250 分的穆斯蒂卡判定）**没有输入**，那与"慢了"不是一个量级。
  // ⚠️ 严格 `<`：恰好等于下限**不**报错。
  if (fps < p.camera_fps_min) return Level::kError;
  return Level::kOk;
}

HealthMonitor::Level HealthMonitor::judge_crc_error_rate(double rate, const MetricParams& p)
{
  // ⚠️ CRC 错率升高说明**串口链路在劣化** —— 按规格是 ERROR（"模块异常但系统仍在运行"）。
  //    它不是 FATAL：帧坏了就丢，不会让系统崩。但**必须让人看见**，
  //    因为它往往是"线松了/干扰大"的前兆。
  if (rate > p.crc_error_rate_warn) return Level::kError;
  return Level::kOk;
}

// ═══════════════════════════════════════════════════════════════════════════
// ④ CPU 温度读取
// ═══════════════════════════════════════════════════════════════════════════
HealthMonitor::ThermalReading HealthMonitor::read_cpu_temp(const std::string& hwmon_root,
                                                           const std::string& sensor_name)
{
  ThermalReading out;

  DIR* dir = ::opendir(hwmon_root.c_str());
  if (dir == nullptr) return out;                 // ⚠️ 如实报 found=false，不造数据

  std::string chosen;                             // 命中的那个 hwmonN 目录
  std::string chosen_name;
  while (struct dirent* e = ::readdir(dir)) {
    const std::string name = e->d_name;
    if (name.size() < 6 || name.compare(0, 5, "hwmon") != 0) continue;
    const std::string full = hwmon_root + "/" + name;
    if (!is_dir(full)) continue;
    std::string dev;
    if (!read_trimmed(full + "/name", dev)) continue;
    // ⚠️⚠️ **按名字精确匹配** —— 这就是 `acpitz` 那个坑的修法。
    //    遍历顺序由 `readdir` 决定（**不保证稳定**）⇒ "拿第一个" 是错的。
    if (dev == sensor_name) {
      chosen = full;
      chosen_name = dev;
      break;
    }
  }
  ::closedir(dir);
  if (chosen.empty()) return out;                 // 没这个传感器 ⇒ found=false

  // ── 在这个传感器里挑一路温度 ──
  // ⚠️ 优先 `Tctl`：`k10temp` 通常同时给 `Tctl`(控制用) 与 `Tdie`(实际裸片温度)，
  //    而**规格 §6.2 点名的是 Tctl/Tdie**。找不到 Tctl 就退到第一个 `temp*_input`。
  std::string best_path, best_label;
  DIR* d2 = ::opendir(chosen.c_str());
  if (d2 == nullptr) return out;
  while (struct dirent* e = ::readdir(d2)) {
    const std::string name = e->d_name;
    // ⚠️ `temp1_input` **正好 11 字符** ⇒ 下界必须是 `< 11` 而不是 `<= 11`
    //    （第一版写成 `<= 11` 把最短的合法名字**全排除了**，被单测当场抓到）。
    const auto pos = name.find("temp");
    if (pos != 0 || name.size() < 11 ||
        name.compare(name.size() - 6, 6, "_input") != 0) {
      continue;
    }
    const std::string stem = chosen + "/" + name.substr(0, name.size() - 6);
    std::string label;
    read_trimmed(stem + "_label", label);
    const std::string path = stem + "_input";
    if (label == "Tctl") { best_path = path; best_label = label; break; }
    if (best_path.empty()) { best_path = path; best_label = label; }
  }
  ::closedir(d2);
  if (best_path.empty()) return out;

  std::string raw;
  if (!read_trimmed(best_path, raw)) return out;
  try {
    // ⚠️ `hwmon` 的温度单位是**毫摄氏度**（整数）⇒ 除以 1000。
    //    漏了这一步会得到 "45700 °C"，而那个值会让**热保护立刻触发**（或恒不触发，取决于阈值方向）。
    out.temp_c = std::stod(raw) / 1000.0;
  } catch (...) {
    return out;                                   // 解析失败 ⇒ 如实报"没读到"
  }
  out.found = true;
  out.sensor = chosen_name;
  out.label = best_label;
  return out;
}

HealthMonitor::Level HealthMonitor::judge_thermal_reading(const ThermalReading& r)
{
  // ⚠️ **读不到温度是"不知道"，不是"没事"** —— 报 WARNING 让人看见。
  //    ⚠️ 但如果这里返回 OK（或让 temp_c=0 参与判定），症状就是
  //    "热保护永不触发、而诊断里一片绿" —— 那比报错还糟。
  if (!r.found) return Level::kWarning;
  return Level::kOk;
}

}  // namespace core
}  // namespace br_perception
