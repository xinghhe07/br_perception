#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace br_perception {
namespace camera {

/**
 * @brief **海康（Hikrobot）GigE 相机驱动** —— 走 MVS SDK，与 `v4l2_camera` 并列
 *
 * ── 为什么不能用 `v4l2_camera` ─────────────────────────────────────────────
 * `MV-CS020-10GC` 是 **GigE** 相机，`v4l2-ctl --list-devices` 里**根本没有它**
 * （本机实测只有 icSpring 一行）⇒ 它不产生 `/dev/video*`，V4L2 那条路走不通。
 * 只能走海康 MVS SDK（本机 `/opt/MVS` 已完整安装）。
 *
 * ── 台位分工（2026-09-25 定）──────────────────────────────────────────────
 *   · **wide** = JR 720p（USB2，UVC）→ 近：穆斯蒂卡球 250 分 / 建筑位 / 柱子
 *   · **tele** = 海康 MV-CS020-10GC（GigE）→ 远：天空方块塔顶
 *   ⚠️ 两者**驱动不同但契约相同**：同样发 `best_effort` + `depth 1`、
 *      同样的 `timestamp_mode` / `frame_id` / 诊断口径 ⇒ 下游六路检测器**一行不用改**。
 *
 * ── 时间戳（**实测定的，不是猜的**）──────────────────────────────────────
 * `MV_FRAME_OUT_INFO_EX::nHostTimeStamp` 官方文档只写"主机生成的时间戳"，
 * **没写单位**。本机实测（2026-09-25）：
 * ```
 *   nHostTimeStamp   = 1790316463495          ← 13 位
 *   host_realtime_ns = 1790316463512594070    ← 19 位
 *   ⇒ nHostTimeStamp 是 **Unix 纪元毫秒**，且只滞后真机时钟约 **17.6 ms**
 *   ⇒ 相邻帧间隔 17~18 ms ⇒ 与实测 57.6 fps 吻合
 * ```
 * **17.6ms 这个数是这条 GigE 链路的真实底噪** —— 比 USB 那条干净得多（没有 USB 缓冲）。
 *
 * ⚠️⚠️ **单位必须写死成"毫秒"**：猜错单位 = 重犯 `usb_cam` 那个
 * "时间戳恒定偏几百 ms 且不报错"的错（详见 §八 9.15）。
 * 若将来换 MVS 大版本，**先重跑上面那三行对照**再改这里。
 *
 * ── 与 `v4l2_camera` 的三处**有意的不对称** ────────────────────────────────
 * 1. **Bayer**：本机这台 `MV-CS020-10GC` **不支持机内转 BGR8/RGB8**
 *    （实测 `MV_CC_SetEnumValueByString("PixelFormat","RGB8_Packed")` 返回 `0x80000004`），
 *    默认输出 `BayerRG8` ⇒ 主机必须 debayer（`cv::COLOR_BayerRG2BGR`）。
 *    ⚠️ **相位已实测验证过**（CFA 平面法，见 §八 9.14 ①）：R>G>B 与暖光吻合，别改。
 * 2. **曝光单位是 µs**（MVS 的 `ExposureTime`），**不是 V4L2 那种 100µs**。
 *    ⚠️ 这两个驱动单位不同，配置里别混。
 * 3. **GigE 要协商包长**（`GevSCPSPacketSize`），不做吞吐上不去。
 */
class MvsCameraNode : public rclcpp::Node
{
public:
  explicit MvsCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~MvsCameraNode() override;

  MvsCameraNode(const MvsCameraNode&) = delete;
  MvsCameraNode& operator=(const MvsCameraNode&) = delete;

  // ══════════════════════════════════════════════════════════════════════
  // 纯函数 —— **static + public**，可被 `test/test_camera_drivers.cpp` 直接调用
  // ══════════════════════════════════════════════════════════════════════

  /// @brief 固定长字节数组 → string（**截到第一个 NUL**；没有 NUL 就取满 n 个）
  /// ⚠️ MVS 的 `chModelName` / `chSerialNumber` 都是定长 `char[]`，**不保证带 NUL**
  ///    ⇒ 直接 `std::string(buf)` 会读越界（或把后面的垃圾带进序列号）。
  [[nodiscard]] static std::string mvs_str(const char* buf, int n);

  /// @brief `nHostTimeStamp` → ROS 时间。
  ///
  /// ⚠️⚠️ **单位是"Unix 纪元毫秒"，这是本机 2026-09-25 实测定的**（官方文档只写
  ///    "主机生成的时间戳"，**没写单位**）：
  /// ```
  ///   nHostTimeStamp   = 1790316463495            ← 13 位
  ///   host_realtime_ns = 1790316463512594070      ← 19 位
  ///   ⇒ 毫秒；且只滞后真机时钟 17.6 ms
  /// ```
  /// **猜错单位 = 重犯 `usb_cam` 那个"时间戳恒定偏几百 ms 且不报错"的错。**
  /// ⇒ 单测里同时钉两件事：**精确值**（13 位进 → 19 位出）与**量级合理性**
  ///   （改成年/微秒会让结果落到荒谬的纪元，必须被抓住）。
  [[nodiscard]] static rclcpp::Time stamp_from_host_ms(int64_t host_ms);

  /// @brief MVS 错误码 → 可读名（认不出就返回兜底串）
  /// ⚠️ **必须按 `unsigned int` 比较**：MVS 错误码在 `0x80000000` 这一档，
  ///    拿 `int` 做 `switch` 会触发 `-Wnarrowing`，而本仓库告警即错误 ⇒ 编不过。
  [[nodiscard]] static const char* err_name(int ret);

private:
  void capture_loop();
  bool open_device();
  void close_device();
  void publish_diagnostics(double capture_to_publish_ms,
                           double capture_vs_publish_ms,
                           uint64_t frames,
                           uint64_t drops);

  // ── 参数（名字尽量与 v4l2_camera / camera_hardware.yaml 对齐）──
  /// 按序列号选相机（多台时用）。**空 = 用枚举到的第一台**
  std::string serial_number_{""};
  std::string frame_id_{"camera_tele_frame"};
  std::string camera_name_{"camera_tele"};
  /// ⚠️ 单位 **µs**（与 v4l2_camera 的 100µs 不同！）
  double exposure_us_{5000.0};
  double gain_db_{0.0};
  /// "Off" / "Once" / "Continuous"
  std::string auto_exposure_{"Off"};
  /// "Off" / "Once" / "Continuous"（⚠️ 关掉 —— 自动白平衡会动判色阈值）
  std::string auto_white_balance_{"Off"};
  /// 相机输出像素格式（默认 Bayer 原样，主机 debayer）
  std::string pixel_format_{"BayerRG8"};
  /// debayer 相位："RG" / "BG"。⚠️ 错了会**红蓝互换且不报错**
  std::string debayer_{"RG"};
  /// "capture"（用 nHostTimeStamp）| "publish"（发布时刻，仅排查用）
  std::string timestamp_mode_{"capture"};
  /// GigE 包长，0 = 自动协商
  int packet_size_{0};
  /// 采集超时（ms）
  int grab_timeout_ms_{1000};
  /// 帧率上限。**0 = 不限制（自由跑）**。
  ///
  /// ⚠️ 2026-09-25 加：本机海康**自由跑 44.8 fps**，而比赛只要 30 ⇒ 多出来的
  ///    那一半是**白烧的电**，直接变成热量（用户反馈「海康太烫」）。
  ///    限到 30fps **少约 1/3 的读出功耗**。
  /// ⚠️ **代价**：帧周期从 22ms 变 33ms ⇒ 相机源年龄 +11ms（31→42ms）。
  ///    总延迟仍在 warn 56ms 以内，**换发热是划算的**。
  /// ⚠️ 这台相机**没有暴露温度特性**（MVS 里搜不到 `DeviceTemperature`）
  ///    ⇒ 软件里读不到温度，只能从源头减发热。
  double frame_rate_{0.0};
  /// **输出 ROI**（0 = 不裁，用全分辨率）。
  ///
  /// ⚠️ 2026-09-25 加，理由是**每帧的搬运量**：本机这台全幅是 **1624×1240×3 = 6.04 MB/帧**
  ///    （JR 那 720p 路只有 2.76 MB）。实测本节点 `capture_to_publish_ms ≈ 29.7 ms`
  ///    ⇒ **上限约 33 fps，正好卡在相机 30fps 的边界上，一点开销就掉帧**。
  ///
  /// ⚠️ **本机这台不支持 Binning / Decimation**（实测 `0x80000109` 不支持）
  ///    ⇒ 减负**只能用 Width/Height**（可写，32~1624 / 32~1240）。
  ///
  /// ⚠️⚠️ **改它会改内参**：这是**相机内裁剪**，`cx/cy` 跟着偏移
  ///    ⇒ **标定必须用同一个 ROI**，否则判色链的投影 ROI 系统性偏（40/80 分）。
  ///    **换尺寸 = 重标定**，别当纯性能开关。
  int output_width_{0};
  int output_height_{0};
  /// 输出 ROI 的**左上角偏移**（`OffsetX` / `OffsetY`）。0 = 不偏。
  ///
  /// ⚠️⚠️ 2026-09-26 加 —— **不加这两个, `output_width/height` 就是"左上角裁"**：
  ///    GenICam 的 `OffsetX/OffsetY` 默认 **0**，于是 `Width=1280, Height=720`
  ///    对一台 1624×1240 的相机裁出来的是 **左上角**那块，不是中间那块。
  ///    后果不是"小一点"，是**光轴跑到画幅的右下**：FOV 偏轴、主点不在图心，
  ///    而它**不报任何错** —— `config/camera_params.yaml` 的 `tele_roi` 当时
  ///    就踩了同一颗雷（配置写着像"全图"，实际在裁左上角）。
  ///
  /// ⚠️ **居中裁的算式**（本机 1624×1240 → 1280×720）：
  ///      OffsetX = (1624 − 1280) / 2 = **172**
  ///      OffsetY = (1240 −  720) / 2 = **260**
  ///    ⚠️ 这两个数**不是随手凑的**：它们就是「原生主点 → 裁后主点」的平移量。
  ///       居中裁的好处正是 **`cx/cy` 平移回图心**，于是「一份标定结果」在
  ///       「相机输出」与「检测器看到的图」两个参照系里**是同一个值** ——
  ///       内参的三处落点不用再各存一份不同的。
  ///
  /// ⚠️ 设的**顺序**：Offset 在 Width/Height **之前**（GenICam 要求先定左上角、
  ///    再定宽高，反了可能被相机判成越界而回退）。两者都在**设像素格式之前**。
  int output_offset_x_{0};
  int output_offset_y_{0};

  // ── 话题 ──
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  /// MVS 的设备句柄。⚠️ 头文件里**不出现 MVS 类型**（保持接口干净）
  void* handle_{nullptr};
  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace camera
}  // namespace br_perception
