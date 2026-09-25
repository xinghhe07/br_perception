#include "br_perception/camera/v4l2_camera_node.hpp"

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace br_perception {
namespace camera {

namespace {

/// ioctl 的正确写法：V4L2 的 ioctl 会改结构体，必须传**可写**指针
template <typename T>
int xioctl(int fd, unsigned long req, T* arg)
{
  int r;
  do { r = ::ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}

int64_t ns_from_timespec(const struct timespec& t)
{
  return static_cast<int64_t>(t.tv_sec) * 1000000000LL + static_cast<int64_t>(t.tv_nsec);
}

}  // namespace

// ── 纯函数（static public，见头文件；单测直接调）────────────────────────────
std::uint32_t V4l2CameraNode::fourcc(const char* s)
{
  return static_cast<std::uint32_t>(s[0]) |
         (static_cast<std::uint32_t>(s[1]) << 8) |
         (static_cast<std::uint32_t>(s[2]) << 16) |
         (static_cast<std::uint32_t>(s[3]) << 24);
}

rclcpp::Time V4l2CameraNode::stamp_from_monotonic(int64_t buf_ns,
                                                  int64_t mono_now_ns,
                                                  const rclcpp::Time& ros_now)
{
  // ros_time_at_buf = ros_now − (mono_now − buf_ns)
  // ⚠️ 两个 now 都由**调用方**传进来（不在这里取），这样它才是纯函数、可单测；
  //    而"每帧重取"这件事由调用方保证 —— 单测有一条专门钉它。
  return ros_now - rclcpp::Duration::from_nanoseconds(mono_now_ns - buf_ns);
}

// ═══════════════════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════════════════
V4l2CameraNode::V4l2CameraNode(const rclcpp::NodeOptions& options)
  : Node("v4l2_camera", options)
{
  // ── 参数：名字与 config/camera_hardware.yaml / camera_bringup.launch.py 对齐 ──
  // ⚠️⚠️ **声明与读取必须一起兜异常**。
  //    参数类型不匹配时 rclcpp **抛异常**（不是回落默认值），而且**在
  //    `declare_parameter` 就抛** —— 命令行/launch 的覆盖是在"声明"那一刻应用的。
  //    ⇒ 只兜 `get_parameter` **没用**。这又是在构造函数里 ⇒ 未捕获 = **整个节点 terminate**。
  //    实测：`-p framerate:=30`（整数打到 double 参数）直接崩掉。
  //    ⇒ 兜住 + **点名**，节点活着、错误看得见（合 §零「稳」轴：不许静默失效）。
  auto safe_param = [this](const char* name, auto& out) {
    using T = std::decay_t<decltype(out)>;
    try {
      this->declare_parameter<T>(name, out);
      this->get_parameter(name, out);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
        "参数 '%s' 类型不对 ⇒ **沿用默认值**。%s "
        "（小数参数要带小数点：写 30.0 而不是 30）", name, e.what());
    }
  };
  safe_param("video_device", video_device_);   safe_param("frame_id", frame_id_);
  safe_param("camera_name", camera_name_);     safe_param("image_width", image_width_);
  safe_param("image_height", image_height_);   safe_param("framerate", framerate_);
  safe_param("buffer_count", buffer_count_);   safe_param("pixel_format", pixel_format_);
  safe_param("timestamp_mode", timestamp_mode_); safe_param("exposure_absolute", exposure_absolute_);
  safe_param("auto_exposure", auto_exposure_); safe_param("gain", gain_);
  safe_param("debayer", debayer_);

  if (timestamp_mode_ != "capture" && timestamp_mode_ != "publish") {
    RCLCPP_WARN(this->get_logger(),
      "timestamp_mode='%s' 不认识（只认 capture/publish）⇒ 回落到 'capture'",
      timestamp_mode_.c_str());
    timestamp_mode_ = "capture";
  }

  // ── QoS：**best_effort + depth 1**（任务书 §3.0 硬约束第 2 条）──
  // ⚠️ 别改成 SensorDataQoS() —— 它的 depth 是 5，"晚了就丢不堆积"就不成立了。
  //    ⚠️ 也别改成 RELIABLE —— 仓库六路检测器收的是 best-effort，发 RELIABLE 会反压。
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.best_effort().durability_volatile();

  // ⚠️ 话题用**相对名**，由 launch 的 namespace/remap 摆到 /camera_wide/image_raw。
  pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", image_qos);
  pub_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", image_qos);
  pub_diag_  = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/capture_diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "V4l2CameraNode 起 | device=%s %dx%d @%.0ffps fmt=%s frame_id=%s ts=%s "
    "exposure=%d(×100µs) auto_exp=%d gain=%d buf=%d",
    video_device_.c_str(), image_width_, image_height_, framerate_,
    pixel_format_.c_str(), frame_id_.c_str(), timestamp_mode_.c_str(),
    exposure_absolute_, static_cast<int>(auto_exposure_), gain_, buffer_count_);

  running_ = true;
  thread_ = std::thread(&V4l2CameraNode::capture_loop, this);
}

V4l2CameraNode::~V4l2CameraNode()
{
  running_ = false;
  if (thread_.joinable()) thread_.join();
  // ⚠️ 设备句柄只在采集线程里碰 ⇒ 必须**等线程退出后**再关，否则关的和用的打架
  close_device();
}

// ═══════════════════════════════════════════════════════════════════════════
// 设备：打开 + 配格式 + 配控制项
// ═══════════════════════════════════════════════════════════════════════════
bool V4l2CameraNode::open_device()
{
  fd_ = ::open(video_device_.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "打不开 %s：%s（`v4l2-ctl --list-devices` 看设备号）",
                 video_device_.c_str(), std::strerror(errno));
    return false;
  }

  // ── 格式 ──
  struct v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width       = static_cast<uint32_t>(image_width_);
  fmt.fmt.pix.height      = static_cast<uint32_t>(image_height_);
  fmt.fmt.pix.pixelformat = fourcc(pixel_format_.c_str());
  fmt.fmt.pix.field       = V4L2_FIELD_NONE;
  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_S_FMT(%s) 失败：%s",
                 pixel_format_.c_str(), std::strerror(errno));
    return false;
  }
  pixfmt_ = fmt.fmt.pix.pixelformat;
  // ⚠️ 驱动**可能改掉**你要的参数（分辨率/格式都可能被裁）⇒ 回读并以回读值为准，
  //    不然后面按错的分辨率 reshape 会算出花屏（而且不报错）。
  if (fmt.fmt.pix.width != static_cast<uint32_t>(image_width_) ||
      fmt.fmt.pix.height != static_cast<uint32_t>(image_height_)) {
    RCLCPP_WARN(this->get_logger(),
      "驱动把分辨率改成 %ux%u（要的是 %dx%d）⇒ 以驱动为准",
      fmt.fmt.pix.width, fmt.fmt.pix.height, image_width_, image_height_);
    image_width_  = static_cast<int>(fmt.fmt.pix.width);
    image_height_ = static_cast<int>(fmt.fmt.pix.height);
  }

  // ── 帧率 ──
  struct v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator   = 1;
  parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(framerate_ + 0.5);
  if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
    RCLCPP_WARN(this->get_logger(), "VIDIOC_S_PARM 失败（非致命）：%s", std::strerror(errno));
  }

  // ── 控制项：曝光 / 增益 ──
  auto set_ctl = [&](uint32_t id, int value, const char* what) {
    struct v4l2_control c{};
    c.id = id; c.value = value;
    if (xioctl(fd_, VIDIOC_S_CTRL, &c) < 0) {
      // ⚠️ 不致命：不同驱动的控制项名字不同（本机 video2 就没有 exposure_time_absolute 的别名）。
      //    但要**喊出来** —— 静默失败 = "配了不生效"，那是本仓库点名的坑。
      RCLCPP_WARN(this->get_logger(),
        "控制项 %s=%d 设置失败：%s（该驱动可能不支持，曝光将不受控）",
        what, value, std::strerror(errno));
    }
  };
  set_ctl(V4L2_CID_EXPOSURE_AUTO,
          auto_exposure_ ? V4L2_EXPOSURE_APERTURE_PRIORITY : V4L2_EXPOSURE_MANUAL,
          "auto_exposure");
  if (!auto_exposure_) {
    // ⚠️ V4L2 的曝光单位是 **100µs**：50 = 5ms。
    set_ctl(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute_, "exposure_absolute");
    // ⚠️ **必须关掉「自动降帧」**（V4L2_CID_EXPOSURE_AUTO_PRIORITY）。
    //    它默认常是**开**的（本机 /dev/video2 实测 value=1），语义是
    //    "暗光下允许拉长曝光、代价是掉帧"。比赛要的是**稳定帧率**
    //    （§零：延迟就是分数）⇒ 显式置 0。
    //    ⚠️ 原来这一步是 launch 里用 `ExecuteProcess` 调 `v4l2-ctl` 干的 ——
    //       那是权宜之计（依赖外部工具、结果不可复现）；现在在节点内做掉。
    set_ctl(V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0, "exposure_auto_priority");
  }
  set_ctl(V4L2_CID_GAIN, gain_, "gain");

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// 流式采集：申请 mmap 缓冲
// ═══════════════════════════════════════════════════════════════════════════
bool V4l2CameraNode::start_streaming()
{
  struct v4l2_requestbuffers req{};
  req.count  = static_cast<uint32_t>(buffer_count_);
  req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_REQBUFS 失败：%s", std::strerror(errno));
    return false;
  }
  if (req.count < 2) {
    RCLCPP_ERROR(this->get_logger(), "驱动只给了 %u 个缓冲（至少要 2）", req.count);
    return false;
  }

  buffers_.resize(req.count);
  for (uint32_t i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      RCLCPP_ERROR(this->get_logger(), "VIDIOC_QUERYBUF[%u] 失败：%s", i, std::strerror(errno));
      return false;
    }
    buffers_[i].length = buf.length;
    buffers_[i].start  = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd_, buf.m.offset);
    if (buffers_[i].start == MAP_FAILED) {
      buffers_[i].start = nullptr;
      RCLCPP_ERROR(this->get_logger(), "mmap[%u] 失败：%s", i, std::strerror(errno));
      return false;
    }
    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      RCLCPP_ERROR(this->get_logger(), "首次 VIDIOC_QBUF[%u] 失败：%s", i, std::strerror(errno));
      return false;
    }
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    RCLCPP_ERROR(this->get_logger(), "VIDIOC_STREAMON 失败：%s", std::strerror(errno));
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "流式采集已启动：%zu 个缓冲", buffers_.size());
  return true;
}

void V4l2CameraNode::close_device()
{
  if (fd_ >= 0) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);   // 没开流时失败也无所谓
  }
  for (auto& b : buffers_) {
    if (b.start && b.start != MAP_FAILED) ::munmap(b.start, b.length);
    b.start = nullptr;
  }
  buffers_.clear();
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// ═══════════════════════════════════════════════════════════════════════════
// 时间戳换算 —— ⚠️ **每帧重算**，这是与 usb_cam 的关键差别
// ═══════════════════════════════════════════════════════════════════════════
rclcpp::Time V4l2CameraNode::stamp_from_buffer(uint64_t sec, uint64_t usec, bool monotonic) const
{
  const int64_t buf_ns = static_cast<int64_t>(sec) * 1000000000LL +
                         static_cast<int64_t>(usec) * 1000LL;
  if (!monotonic) {
    // 驱动用的是 REALTIME（epoch）⇒ 直接就是 ROS 时间
    return rclcpp::Time(buf_ns, RCL_ROS_TIME);
  }
  // ⚠️⚠️ **每帧读一次单调钟，与当前 ROS 时间配对求偏移。**
  //    若像（推测中的）usb_cam 那样**只在初始化时算一次**，之后系统时钟被 NTP
  //    步进/驯服过，那个偏移就**永久错**了 —— 症状恰好是
  //    "恒定几百 ms、跨次运行不同、单次运行内不漂"，与实测完全吻合。
  struct timespec mono{};
  clock_gettime(CLOCK_MONOTONIC, &mono);
  return stamp_from_monotonic(buf_ns, ns_from_timespec(mono), this->now());
}

// ═══════════════════════════════════════════════════════════════════════════
// 采集线程
// ═══════════════════════════════════════════════════════════════════════════
void V4l2CameraNode::capture_loop()
{
  if (!open_device() || !start_streaming()) {
    RCLCPP_ERROR(this->get_logger(),
      "采集起不来 —— 节点**继续存活**（不拖垮容器里的其它相机），但本路不会有图。"
      "判据是 `ros2 topic hz <namespace>/image_raw`，不是「进程还在」。");
    close_device();
    return;
  }

  uint64_t frames = 0, drops = 0;
  const bool want_mjpeg = (pixfmt_ == fourcc("MJPG"));

  while (running_) {
    struct v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
      if (errno == EAGAIN) {                 // 还没帧，正常
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "VIDIOC_DQBUF 失败：%s", std::strerror(errno));
      ++drops;
      continue;
    }

    // ── 时间戳：**先把驱动的采集时刻记下来**（缓冲还没还回去）──
    const uint64_t b_sec  = static_cast<uint64_t>(buf.timestamp.tv_sec);
    const uint64_t b_usec = static_cast<uint64_t>(buf.timestamp.tv_usec);
    const bool monotonic  = (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
                            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    const rclcpp::Time capture_stamp = stamp_from_buffer(b_sec, b_usec, monotonic);

    // ── 解码（在**还缓冲之前**做完，保证按帧序）──
    cv::Mat bgr;
    {
      const auto* p = static_cast<const std::uint8_t*>(buffers_[buf.index].start);
      if (want_mjpeg) {
        cv::Mat jpg(1, static_cast<int>(buf.bytesused), CV_8UC1, const_cast<std::uint8_t*>(p));
        bgr = cv::imdecode(jpg, cv::IMREAD_COLOR);     // → BGR8
      } else if (pixfmt_ == fourcc("YUYV")) {
        cv::Mat yuyv(image_height_, image_width_, CV_8UC2, const_cast<std::uint8_t*>(p));
        cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUYV);
      } else if (pixfmt_ == fourcc("RGGB")) {           // 仅海康那类 Bayer 相机用
        cv::Mat raw(image_height_, image_width_, CV_8UC1, const_cast<std::uint8_t*>(p));
        const int code = (debayer_ == "BG") ? cv::COLOR_BayerBG2BGR : cv::COLOR_BayerRG2BGR;
        cv::cvtColor(raw, bgr, code);
      }
    }

    if (!bgr.empty()) {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      // ⚠️ `publish` 模式只用于**对照排查**，它会把真实采集延迟藏起来 —— 别拿它当默认。
      msg->header.stamp    = (timestamp_mode_ == "publish") ? this->now() : capture_stamp;
      msg->header.frame_id = frame_id_;
      msg->height = static_cast<uint32_t>(bgr.rows);
      msg->width  = static_cast<uint32_t>(bgr.cols);
      msg->encoding = "bgr8";
      msg->is_bigendian = false;
      msg->step = static_cast<uint32_t>(bgr.cols * 3);
      msg->data.assign(bgr.data, bgr.data + static_cast<size_t>(bgr.step) * bgr.rows);
      const auto publish_stamp = this->now();
      pub_image_->publish(std::move(msg));

      // ── 每帧上报：这两个数就是"画面到底旧不旧"的判据 ──
      const double cap_to_pub_ms =
          (publish_stamp - capture_stamp).seconds() * 1000.0;
      const double cap_vs_pub_ms =
          (capture_stamp - publish_stamp).seconds() * 1000.0;   // 采集时刻相对发布时刻（负=采集在过去）
      publish_diagnostics(cap_to_pub_ms, cap_vs_pub_ms, ++frames, drops);
    }

    if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "VIDIOC_QBUF 失败：%s", std::strerror(errno));
      ++drops;
    }
  }

  RCLCPP_INFO(this->get_logger(), "采集线程退出：共 %lu 帧，丢弃 %lu",
              static_cast<unsigned long>(frames), static_cast<unsigned long>(drops));
  close_device();
}

// ═══════════════════════════════════════════════════════════════════════════
// 诊断上报
// ═══════════════════════════════════════════════════════════════════════════
void V4l2CameraNode::publish_diagnostics(double capture_to_publish_ms,
                                         double capture_vs_publish_ms,
                                         uint64_t frames,
                                         uint64_t drops)
{
  // ⚠️ 限频：诊断不该比图像还贵。每 30 帧发一次。
  if (frames % 30 != 0) return;

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = camera_name_ + "_v4l2";
  st.hardware_id = video_device_;

  // 判据取自**实测**：usb_cam 同条件下这一项是 600~800ms。
  if (capture_to_publish_ms < 50.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "OK";
  } else if (capture_to_publish_ms < 200.0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = "CAPTURE_TO_PUBLISH_HIGH";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message = "CAPTURE_TO_PUBLISH_VERY_HIGH";
  }

  auto add = [&](const std::string& k, double v) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k; kv.value = std::to_string(v);
    st.values.push_back(kv);
  };
  add("capture_to_publish_ms", capture_to_publish_ms);   ///< 采集→发布：**本节点自己的开销**
  add("capture_vs_publish_ms", capture_vs_publish_ms);   ///< 采集时刻相对发布时刻（负值 = 画面是过去的）
  add("frames", static_cast<double>(frames));
  add("drops",  static_cast<double>(drops));

  arr.status.push_back(st);
  pub_diag_->publish(arr);
}

}  // namespace camera
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::V4l2CameraNode)
