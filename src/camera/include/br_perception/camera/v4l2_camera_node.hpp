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
#include <vector>

namespace br_perception {
namespace camera {

/**
 * @brief **本仓库自己的相机驱动** —— V4L2 直采，替换 `usb_cam`
 *
 * ── 为什么要自己写（2026-09-25 实测，不是推测）────────────────────────────
 * `usb_cam` 写进 `header.stamp` 的时间戳比"发布时刻"**老了约 600~800 ms**：
 *
 * | 采集器 | 订阅端看到的年龄（同一台相机、同一个探针） |
 * |---|---|
 * | `usb_cam`（C++，时间戳取自驱动缓冲区） | **810 ms** |
 * | 自建发布器（时间戳 = 发布时刻） | **170 ms**（其中大头是 Python 端 2.76MB 消息的开销） |
 *
 * 而且已经排除的：**不是**订阅端排队（`camera_info` 只有几十字节，年龄与图像**一模一样**）、
 * **不是**时钟漂移（连采 42 s 不漂）、**不是**按数据量的队列（640×480 反而比 720p 更差）。
 *
 * ── 三个设计决定 ───────────────────────────────────────────────────────
 *
 * **① QoS 必须是 `best_effort` + `depth 1`** —— 这是任务书 §3.0「三相机并行硬约束」
 *    第 2 条，`image_preprocessor.cpp:111-117` 已记着：**"晚了就丢，不堆积 ——
 *    否则一次卡顿会让延迟永久滚大"**。`SensorDataQoS()` 的 depth 是 5，**不合规**。
 *    ⚠️ 顺带修掉 `usb_cam` 发 **RELIABLE** 而仓库六路检测器收 best-effort 的**不匹配**。
 *
 * **② 时间戳：`timestamp_mode` 是**显式参数**，默认 `"capture"`（诚实）**
 *    · `"capture"` = 用驱动给的缓冲区时间戳（= 曝光后那一帧被驱动收下的时刻），
 *      **每帧重算一次单调钟→ROS 钟的偏移**。⚠️ 这正是不信 `usb_cam` 的地方：
 *      如果它在初始化时**只算一次偏移**，之后系统时钟被 NTP 调整过，那个偏移就**永久错了**
 *      —— 症状恰好是"恒定几百 ms、跨次运行不同、单次运行内不漂"。
 *    · `"publish"` = 发布时刻。**只用于对照排查**，因为它会把真实的采集延迟**藏起来**。
 *    ⚠️ 两种模式的差值会**每帧上报**到 `~/capture_diagnostics`（见下）——
 *      **这个差值是判定"画面到底旧不旧"的唯一判据**，别把它关掉。
 *
 * **③ 专门的采集线程，不用 ROS 定时器** —— 定时器 + 单线程执行器会让
 *    "解码 + 2.76MB 序列化" 与 "等下一帧" 互相排队。独立线程里
 *    **采集/解码在锁外做，只有发布在锁内**，发布时刻的时间戳才可信。
 *
 * ── 与仓库其余部分的契约 ────────────────────────────────────────────────
 * · 发布话题是**相对名** `image_raw` / `camera_info` ⇒ 由 launch 用
 *   `namespace=` 或 `remap` 摆到 `/camera_wide/image_raw` 上。
 *   ⚠️ `image_preprocessor.cpp:126,130` 是**硬编码字面量**，所以名字必须落准。
 * · `frame_id` 必须逐字等于 `config/camera_params.yaml:207` 的
 *   `camera_frames: ["camera_wide_frame","camera_tele_frame"]` —— 否则
 *   `sky_block_detector` 的 TF lookup 失败 ⇒ 判色（40/80 分）整条不工作。
 * · 参数名与 `config/camera_hardware.yaml` 对齐；⚠️ **曝光单位是 100µs**
 *   （V4L2 `V4L2_CID_EXPOSURE_ABSOLUTE` 的定义），所以 `exposure_time: 5000` µs
 *   ⇒ 这里填 **50**。填 5000 = 500 ms 曝光 = 一片白。
 */
class V4l2CameraNode : public rclcpp::Node
{
public:
  explicit V4l2CameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~V4l2CameraNode() override;

  V4l2CameraNode(const V4l2CameraNode&) = delete;
  V4l2CameraNode& operator=(const V4l2CameraNode&) = delete;

  // ══════════════════════════════════════════════════════════════════════
  // 纯函数 —— **static + public**，可被 `test/test_camera_drivers.cpp` 直接调用
  // （仓库既有模式：`ClusterExtractor::filter_obstacle`、`SkyBlockDetector::judge_top_color`）
  // ══════════════════════════════════════════════════════════════════════

  /// @brief V4L2 四字符码：`'MJPG'` → cpu 序整数
  /// ⚠️ **字节序写反了不会报错，只会"格式设不上"** 而驱动回退到别的格式 ⇒ 出花屏。
  ///    单测拿它和内核自己的 `V4L2_PIX_FMT_MJPEG` 对比，**这是唯一能钉死字节序的判据**。
  [[nodiscard]] static std::uint32_t fourcc(const char* s);

  /// @brief **单调钟时刻 → ROS 时间**（本驱动存在的主要理由就在这个函数里）
  ///
  /// 公式：`ros_time_at_buf = ros_now − (mono_now − buf_ns)`
  ///
  /// ⚠️⚠️ **调用方必须每帧重新取 `mono_now_ns` 与 `ros_now`** —— 这正是 `usb_cam`
  ///    （推测中）那个坑的反面：它只在初始化时算一次偏移，之后系统时钟被 NTP
  ///    步进/驯服过，偏移就**永久错**了；症状恰好是"**恒定几百 ms、跨次运行不同、
  ///    单次运行内不漂**"，与本仓库 2026-09-25 的实测完全吻合（§八 9.15）。
  ///    ⇒ 单测里专门有一条钉"**传不同的 mono_now 必须得到不同的结果**"。
  [[nodiscard]] static rclcpp::Time stamp_from_monotonic(int64_t buf_ns,
                                                         int64_t mono_now_ns,
                                                         const rclcpp::Time& ros_now);

private:
  // ── 采集线程主体：开设备 → 流式采集 → 解 → 发 ──
  void capture_loop();

  /// 打开设备并配好格式/控制项。@return 成功
  bool open_device();
  /// 申请 mmap 缓冲并开始流式采集。@return 成功
  bool start_streaming();
  /// 停流 + 释放 mmap + 关设备（可重入，析构与错误路径都调它）
  void close_device();

  /// 把驱动给的缓冲区时间戳换算成 ROS 时间。
  /// ⚠️ **每帧都重算偏移** —— 初始化时算一次就是 `usb_cam` 那个坑。
  rclcpp::Time stamp_from_buffer(uint64_t sec, uint64_t usec, bool monotonic) const;

  /// 每帧上报：采集→发布的耗时、两种时间戳模式的差值（**判定"画面旧不旧"的判据**）
  void publish_diagnostics(double capture_to_publish_ms,
                           double capture_vs_publish_ms,
                           uint64_t frames,
                           uint64_t drops);

  // ── 参数（**名字 = yaml 键名**，三方逐字一致）──
  std::string video_device_{"/dev/video0"};
  std::string frame_id_{"camera_wide_frame"};
  std::string camera_name_{"camera_wide"};
  int    image_width_{1280};
  int    image_height_{720};
  double framerate_{30.0};
  int    buffer_count_{4};
  std::string pixel_format_{"MJPG"};
  std::string timestamp_mode_{"capture"};   ///< "capture" | "publish"
  /// ⚠️ 单位 **100µs**：50 = 5ms。与 `camera_hardware.yaml` 的 `exposure_time:5000`(µs) 等价
  int    exposure_absolute_{50};
  bool   auto_exposure_{false};
  int    gain_{0};
  /// 2×2 CFA 相位。⚠️ **错了会红蓝互换且不报错** —— 海康那台是 `BayerRG8`，
  /// 需 debayer；本机 JR 出的是解好的 MJPG，所以默认 `""`（不解 Bayer）
  std::string debayer_{""};

  // ── 话题 ──
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  // ── 设备句柄（只在采集线程里碰）──
  int fd_{-1};
  struct MappedBuffer { void* start{nullptr}; size_t length{0}; };
  std::vector<MappedBuffer> buffers_;
  std::uint32_t pixfmt_{0};

  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace camera
}  // namespace br_perception
