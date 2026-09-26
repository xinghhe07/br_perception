#include "br_perception/camera/mvs_camera_node.hpp"

#include <MvCameraControl.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cstring>

namespace br_perception {
namespace camera {

// ── 纯函数（static public，见头文件；单测直接调）────────────────────────────
const char* MvsCameraNode::err_name(int ret)
{
  // ⚠️ **必须转成 unsigned 再 switch**：MVS 的错误码是 `0x80000000` 这一档，
  //    在 C++ 里是 `unsigned int` 字面量，直接拿 `int` 做 case 标签会触发
  //    `-Wnarrowing`（本仓库把告警当错误，会直接编不过 —— 本轮实测踩过）。
  // 只列最常撞的几个；其余直接打十六进制，配合 MvErrorDefine.h 查
  switch (static_cast<unsigned int>(ret)) {
    case MV_OK:                return "MV_OK";
    case MV_E_HANDLE:          return "MV_E_HANDLE(句柄无效)";
    case MV_E_SUPPORT:         return "MV_E_SUPPORT(不支持)";
    case MV_E_BUFOVER:         return "MV_E_BUFOVER(缓冲已满)";
    case MV_E_CALLORDER:       return "MV_E_CALLORDER(调用顺序错)";
    case MV_E_PARAMETER:       return "MV_E_PARAMETER(参数错)";
    case MV_E_RESOURCE:        return "MV_E_RESOURCE(资源申请失败)";
    case MV_E_NODATA:          return "MV_E_NODATA(无数据)";
    case MV_E_NOENOUGH_BUF:    return "MV_E_NOENOUGH_BUF(缓冲不足)";
    case MV_E_GC_GENERIC:      return "MV_E_GC_GENERIC(GenICam 通用错)";
    case MV_E_GC_ACCESS:       return "MV_E_GC_ACCESS(GenICam 访问错)";
    case MV_E_GC_TIMEOUT:      return "MV_E_GC_TIMEOUT(GenICam 超时)";
    case MV_E_GC_UNKNOW:       return "MV_E_GC_UNKNOW(GenICam 未知错)";
    case MV_E_NOT_IMPLEMENTED: return "MV_E_NOT_IMPLEMENTED(未实现)";
    case MV_E_ACCESS_DENIED:   return "MV_E_ACCESS_DENIED(无权限/被占用)";
    default:                   return "见 MvErrorDefine.h";
  }
}

std::string MvsCameraNode::mvs_str(const char* buf, int n)
{
  std::string s(buf, buf + n);
  const auto z = s.find('\0');
  return (z == std::string::npos) ? s : s.substr(0, z);
}

rclcpp::Time MvsCameraNode::stamp_from_host_ms(int64_t host_ms)
{
  // ⚠️⚠️ **毫秒 → 纳秒**。这个 ×1000000 就是全部内容，而**单位错了不报错、
  //    只是时间戳错**（差 1000 倍 = 差 1000 秒那种）⇒ 单测必须钉住量级。
  return rclcpp::Time(host_ms * 1000000LL, RCL_ROS_TIME);
}

// ═══════════════════════════════════════════════════════════════════════════
// 构造 / 析构
// ═══════════════════════════════════════════════════════════════════════════
MvsCameraNode::MvsCameraNode(const rclcpp::NodeOptions& options)
  : Node("mvs_camera", options)
{
  // ⚠️⚠️ **声明与读取必须一起兜异常**。
  //    参数类型不匹配时 rclcpp **抛异常**（不是回落默认值），而且**在
  //    `declare_parameter` 就抛** —— 命令行/launch 的覆盖是在"声明"那一刻应用的。
  //    ⇒ 只兜 `get_parameter` **没用**（本轮就先踩了这个）。
  //    这又是在构造函数里 ⇒ 未捕获 = **整个节点 terminate**。
  //    实测：`-p exposure_us:=5000`（整数打到 double 参数）直接
  //    `InvalidParameterTypeException` 崩掉。
  //    ⚠️ 对**每个小数参数**都成立 —— 手写 launch / 命令行人最容易踩。
  //    ⇒ 兜住 + **点名**，节点活着、错误看得见（合 §零「稳」轴：不许静默失效）。
  auto safe_param = [this](const char* name, auto& out) {
    using T = std::decay_t<decltype(out)>;
    try {
      this->declare_parameter<T>(name, out);
      this->get_parameter(name, out);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
        "参数 '%s' 类型不对 ⇒ **沿用默认值**。%s "
        "（小数参数要带小数点：写 5000.0 而不是 5000）", name, e.what());
    }
  };
  safe_param("serial_number", serial_number_);   safe_param("frame_id", frame_id_);
  safe_param("camera_name", camera_name_);       safe_param("exposure_us", exposure_us_);
  safe_param("gain_db", gain_db_);               safe_param("auto_exposure", auto_exposure_);
  safe_param("auto_white_balance", auto_white_balance_);
  safe_param("pixel_format", pixel_format_);     safe_param("debayer", debayer_);
  safe_param("timestamp_mode", timestamp_mode_); safe_param("packet_size", packet_size_);
  safe_param("grab_timeout_ms", grab_timeout_ms_);
  safe_param("frame_rate", frame_rate_);
  safe_param("output_width", output_width_);
  safe_param("output_height", output_height_);
  safe_param("output_offset_x", output_offset_x_);
  safe_param("output_offset_y", output_offset_y_);

  if (timestamp_mode_ != "capture" && timestamp_mode_ != "publish") {
    RCLCPP_WARN(this->get_logger(),
      "timestamp_mode='%s' 不认识（只认 capture/publish）⇒ 回落到 'capture'",
      timestamp_mode_.c_str());
    timestamp_mode_ = "capture";
  }

  // ── QoS：**best_effort + depth 1**（任务书 §3.0 硬约束第 2 条）──
  //    与 `v4l2_camera` **完全一致** —— 两条路对下游必须是同一种契约。
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.best_effort().durability_volatile();

  pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", image_qos);
  pub_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", image_qos);
  // ⚠️ 用**相对名** `capture_diagnostics`（不是 `~/capture_diagnostics`）。
  //    `~` 是**节点私有命名空间**，在 `namespace=/camera_tele` + 节点名 `mvs_camera`
  //    之下会落到 `/camera_tele/mvs_camera/capture_diagnostics` —— 比文档写的那条长一截，
  //    现场照着文档敲会"话题不存在"（2026-09-25 实测踩过）。
  pub_diag_  = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "capture_diagnostics", 10);

  RCLCPP_INFO(this->get_logger(),
    "MvsCameraNode 起 | serial='%s'(空=第一台) frame_id=%s fmt=%s debayer=%s ts=%s "
    "exp=%.0fµs gain=%.1fdB autoExp=%s autoWB=%s",
    serial_number_.c_str(), frame_id_.c_str(), pixel_format_.c_str(), debayer_.c_str(),
    timestamp_mode_.c_str(), exposure_us_, gain_db_,
    auto_exposure_.c_str(), auto_white_balance_.c_str());

  running_ = true;
  thread_ = std::thread(&MvsCameraNode::capture_loop, this);
}

MvsCameraNode::~MvsCameraNode()
{
  running_ = false;
  if (thread_.joinable()) thread_.join();
  close_device();
  MV_CC_Finalize();
}

// ═══════════════════════════════════════════════════════════════════════════
// 打开设备
// ═══════════════════════════════════════════════════════════════════════════
bool MvsCameraNode::open_device()
{
  MV_CC_Initialize();

  MV_CC_DEVICE_INFO_LIST dl;
  std::memset(&dl, 0, sizeof(dl));
  // GigE + USB3 都枚举（本节点主要给 GigE 用，但 USB3 的海康也走同一套 API）
  int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dl);
  if (ret != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_EnumDevices 失败：0x%08x %s", ret, err_name(ret));
    return false;
  }
  if (dl.nDeviceNum == 0) {
    RCLCPP_ERROR(this->get_logger(),
      "没枚举到相机。GigE 的排查看两处：① `ip -brief addr` 网口起来没（要 carrier=1）"
      "② 相机与主机是否同网段（`/opt/MVS/bin/Ip_Configurator`）");
    return false;
  }

  // ── 选相机：给了序列号就按序列号找，否则用第一台 ──
  MV_CC_DEVICE_INFO* chosen = nullptr;
  for (unsigned i = 0; i < dl.nDeviceNum; ++i) {
    auto* info = dl.pDeviceInfo[i];
    if (info == nullptr) continue;
    if (serial_number_.empty()) { chosen = info; break; }
    const bool gige = (info->nTLayerType == MV_GIGE_DEVICE);
    const std::string sn = gige
      ? mvs_str(reinterpret_cast<const char*>(info->SpecialInfo.stGigEInfo.chSerialNumber), 16)
      : mvs_str(reinterpret_cast<const char*>(info->SpecialInfo.stUsb3VInfo.chSerialNumber), 64);
    if (sn == serial_number_) { chosen = info; break; }
  }
  if (chosen == nullptr) {
    RCLCPP_ERROR(this->get_logger(),
      "枚举到 %u 台，但没有序列号='%s' 的那台 ⇒ 不猜，直接失败"
      "（猜错会把两台相机的角色/外参搞反）", dl.nDeviceNum, serial_number_.c_str());
    return false;
  }

  {
    const bool gige = (chosen->nTLayerType == MV_GIGE_DEVICE);
    if (gige) {
      const auto& g = chosen->SpecialInfo.stGigEInfo;
      RCLCPP_INFO(this->get_logger(), "选中 [GigE] %s  序列号=%s",
        mvs_str(reinterpret_cast<const char*>(g.chModelName), 32).c_str(),
        mvs_str(reinterpret_cast<const char*>(g.chSerialNumber), 16).c_str());
    }
  }

  ret = MV_CC_CreateHandle(&handle_, chosen);
  if (ret != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_CreateHandle 失败：0x%08x %s", ret, err_name(ret));
    handle_ = nullptr;
    return false;
  }

  ret = MV_CC_OpenDevice(handle_, MV_ACCESS_Exclusive, 0);
  if (ret != MV_OK) {
    RCLCPP_ERROR(this->get_logger(),
      "MV_CC_OpenDevice 失败：0x%08x %s（被别的进程占着？MVS 图形界面开着？）",
      ret, err_name(ret));
    return false;
  }

  // ── GigE：协商最大包长（不做吞吐上不去）──
  if (packet_size_ <= 0) {
    const int ps = MV_CC_GetOptimalPacketSize(handle_);
    packet_size_ = (ps > 0) ? ps : 1500;
  }
  ret = MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packet_size_);
  if (ret != MV_OK) {
    // ⚠️ 不致命但要喊 —— 包长没协商上会表现为"帧率低/丢帧"，很难反查
    RCLCPP_WARN(this->get_logger(), "设 GevSCPSPacketSize=%d 失败：0x%08x %s",
                packet_size_, ret, err_name(ret));
  } else {
    RCLCPP_INFO(this->get_logger(), "GigE 包长 = %d", packet_size_);
  }

  // ── 触发模式：连续采集 ──
  MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);

  // ── 输出尺寸 / ROI：减每帧的搬运量 ──
  // ⚠️ 本机实测（2026-09-25）：这台 **不支持 Binning / Decimation**
  //    （`BinningHorizontal` / `DecimationHorizontal` 都返回 `0x80000109` 不支持），
  //    所以减负只能用 **`Width`/`Height`**（可写，32~1624 / 32~1240）。
  // ⚠️⚠️ **改 Width/Height 会改内参**：这是**相机内裁剪**，`cx/cy` 会跟着偏移。
  //    ⇒ **标定时必须用同一个 ROI**，否则投影 ROI（那 40/80 分的判色链）会系统性偏。
  //    ⇒ 换尺寸 = 重标定，别当成纯性能开关。
  // ⚠️ 必须在设像素格式**之前**做：改尺寸会让格式设置失效，顺序反了就白设。
  if (output_width_ > 0 && output_height_ > 0) {
    // ── 先按相机的 min/max/**步进**把四个值吸附到合法档 ──
    //    ⚠️⚠️ **为什么必须吸附**: GenICam 的 `Width/Height/OffsetX/OffsetY` 各自
    //       带一个**步进** `nInc`（本机这台 Width 范围 32~1624、Height 32~1240，
    //       步进未实测）。不合步进时相机**很可能静默回退**而不是报错 ——
    //       而 ROI 是**内参的一部分**，回退就意味着"标定用的那块和配置写的那块
    //       不是同一块"，且不报错。
    //    ⚠️ 吸附会**改变数值**（最多 nInc−1 px，那点偏移直接进 `cx/cy`）
    //       ⇒ 下面**必须把最终值打出来**，只打请求值等于没说。
    //    ⚠️ 这也是 `output_width/height` 这条路径**第一次真被跑**：
    //       `camera_bringup.launch.py` 在此之前**从不传**这三个参数。
    auto snap = [&](const char* key, int& want) -> bool {
      MVCC_INTVALUE_EX iv;
      if (MV_CC_GetIntValueEx(handle_, key, &iv) != MV_OK) return false;
      const long lo  = static_cast<long>(iv.nMin);
      const long hi  = static_cast<long>(iv.nMax);
      const long inc = std::max<long>(1, static_cast<long>(iv.nInc));
      long v = std::max(lo, std::min(static_cast<long>(want), hi));
      if (inc > 1) {
        v = lo + ((v - lo + inc - 1) / inc) * inc;      // 向上取到步进整数倍
        if (v > hi) v = lo + ((hi - lo) / inc) * inc;   // 越界则退回最大的合法档
      }
      if (v != static_cast<long>(want)) {
        RCLCPP_WARN(this->get_logger(),
          "%s: 请求 %d 不是合法档（范围 %ld~%ld, 步进 %ld）⇒ 吸附到 %ld。"
          "⚠️ 若这是 OffsetX/OffsetY, 主点会跟着平移这么多像素 ⇒ **配置里的值要改成 %ld**",
          key, want, lo, hi, inc, v, v);
        want = static_cast<int>(v);
      }
      return true;
    };
    const bool ok_sx = snap("OffsetX", output_offset_x_);
    const bool ok_sy = snap("OffsetY", output_offset_y_);
    const bool ok_sw = snap("Width",   output_width_);
    const bool ok_sh = snap("Height",  output_height_);
    if (!ok_sx) output_offset_x_ = 0;      // 读不到描述子 ⇒ 宁可不动偏移（见下）
    if (!ok_sy) output_offset_y_ = 0;

    // ── ⚠️⚠️ Bayer 相位: 偏移**必须是偶数** ──
    //   本节点收 `BayerRG8` 然后在**主机侧**按**固定相位** debayer（`debayer_ = "RG"`）。
    //   Bayer 的 2×2 图案是 **RG / GB 逐行交替** —— 裁掉**奇数**行或列之后，
    //   交付图的第一行/第一列**不再是 R 行 / R 列** ⇒ 固定的 `COLOR_BayerRG2BGR`
    //   解出来**红蓝互换**，而它**不会报错**（跟 `debayer_` 写错是同一类故障）。
    //   ⇒ 强制拉平到偶数（最多挪 1 px，进 `cx/cy` 的量可忽略，但**必须打出来**）。
    auto force_even = [&](const char* key, int& v) {
      if (v % 2 != 0) {
        RCLCPP_WARN(this->get_logger(),
          "%s=%d 是**奇数** ⇒ Bayer 相位会翻转（固定相位 debayer 会红蓝互换且不报错）"
          " ⇒ 拉平到 %d。⚠️ 配置里改成 %d", key, v, v - 1, v - 1);
        v -= 1;
      }
    };
    force_even("OffsetX", output_offset_x_);
    force_even("OffsetY", output_offset_y_);

    if (!ok_sw || !ok_sh) {
      RCLCPP_WARN(this->get_logger(),
        "读不到 Width/Height 的合法范围 ⇒ **不设 ROI**（保持原生全幅）。"
        "此时内参按全幅标, 与配置的 resolution 不符 —— 见 check 的尺寸链");
    }

    // ⚠️⚠️ **偏移必须显式设, 而且必须在 Width/Height 之前**。
    //    GenICam 的 `OffsetX/OffsetY` 默认 **0** ⇒ 只设 Width/Height 得到的是
    //    **左上角**那块, 不是中间那块。后果不是"画面小一点", 是**光轴跑到右下**
    //    （FOV 偏轴、主点不在图心）—— 而它**不报错**。
    //    先定左上角再定宽高是 GenICam 的要求, 反了可能被判越界而回退。
    const bool want_roi = ok_sw && ok_sh;
    int rox = MV_OK, roy = MV_OK, rw = MV_OK, rh = MV_OK;
    if (want_roi) {
      rox = MV_CC_SetIntValueEx(handle_, "OffsetX", output_offset_x_);
      roy = MV_CC_SetIntValueEx(handle_, "OffsetY", output_offset_y_);
      rw  = MV_CC_SetIntValueEx(handle_, "Width",   output_width_);
      rh  = MV_CC_SetIntValueEx(handle_, "Height",  output_height_);
    }

    // ── 读回**四个**再对一遍 ──
    //    ⚠️ 这是**唯一**能知道"实际 ROI 到底是哪块"的手段: 相机可以对越界/不合
    //       步进的值**静默回退**(而不是报错), 也可能接受 Width/Height 却拒绝 Offset。
    //       内参是按 ROI 定的 ⇒ 这里读回来的才是"该拿哪份内参"的事实源。
    auto get_int = [&](const char* key, long& out) -> bool {
      MVCC_INTVALUE_EX iv;
      if (MV_CC_GetIntValueEx(handle_, key, &iv) != MV_OK) return false;
      out = static_cast<long>(iv.nCurValue);
      return true;
    };
    long gx = -1, gy = -1, gw = -1, gh = -1;
    const bool ok_x = get_int("OffsetX", gx);
    const bool ok_y = get_int("OffsetY", gy);
    const bool ok_w = get_int("Width",   gw);
    const bool ok_h = get_int("Height",  gh);

    if (want_roi && rw == MV_OK && rh == MV_OK && rox == MV_OK && roy == MV_OK
        && ok_x && ok_y && ok_w && ok_h
        && gx == output_offset_x_ && gy == output_offset_y_
        && gw == output_width_    && gh == output_height_) {
      RCLCPP_INFO(this->get_logger(),
        "输出 ROI = %ldx%ld @ 偏移(%ld, %ld)（搬运量 %.1f MB/帧）"
        " —— ⚠️ 内参必须按**这一块**标定",
        gw, gh, gx, gy, static_cast<double>(gw) * gh * 3.0 / 1e6);
    } else {
      // ⚠️ 三种失败的**内参后果不一样**, 分开报 —— 报混了会让人按错的那份去标:
      //    ① 压根没请求 ROI（描述子读不到）⇒ 全分辨率。图是完整的, 与配置不符。
      //    ② 偏移没设上、尺寸设上了  ⇒ **左上角裁**。最阴的一档: 尺寸与配置**对得上**,
      //       不报错, 只有光轴偏了 —— 投影会系统性偏（那 40/80 分）。
      //    ③ 都没设上 ⇒ 同 ①。
      if (!want_roi) {
        RCLCPP_WARN(this->get_logger(),
          "**未请求 ROI**（读不到 Width/Height 的合法范围）⇒ 保持原生全幅。"
          "此时内参按**全幅**标, 与 camera_hardware.yaml 的 resolution 不符 —— 先对齐再标");
      } else {
        RCLCPP_WARN(this->get_logger(),
          "设 ROI 失败或**被相机回退** —— 请求 %dx%d @ (%d, %d)，实际读到 %ldx%ld @ (%ld, %ld)。"
          "set 返回 Width/Height=0x%08x/0x%08x、OffsetX/Y=0x%08x/0x%08x %s。",
          output_width_, output_height_, output_offset_x_, output_offset_y_,
          gw, gh, gx, gy, rw, rh, rox, roy,
          err_name(rw != MV_OK ? rw : (rh != MV_OK ? rh : (rox != MV_OK ? rox : roy))));
        if (gw == output_width_ && gh == output_height_
            && (gx != output_offset_x_ || gy != output_offset_y_)) {
          RCLCPP_ERROR(this->get_logger(),
            "⇒⚠️⚠️ **尺寸对上了但偏移没对上** —— 这是**左上角裁切**，FOV 偏轴、"
            "光轴落在画幅右下。此时**内参必须是这一块（含偏移）标出来的**，"
            "拿居中裁/全幅的内参来用会系统性偏，且不会报错。");
        } else {
          RCLCPP_WARN(this->get_logger(),
            "⇒ 实际 ROI 与请求不符（可能仍是全分辨率，6MB/帧）。"
            "**config/camera_hardware.yaml 的 resolution/roi 与实际不符** —— 标定前先对齐。");
        }
      }
    }
  }

  // ── 帧率上限（0 = 不限，自由跑）──
  // ⚠️ 降帧 = **直接降发热**。本机实测海康自由跑 44.8fps，而比赛只要 30
  //    ⇒ 不限就是白烧电变热（用户反馈"太烫"）。
  if (frame_rate_ > 0.0) {
    // ⚠️ **不要先设 `AcquisitionFrameRateEnable`**：本机这台**不支持它**
    //    （实测返回 `0x80000109`）⇒ 先设它会把整个限帧路径带进失败分支，
    //    **限帧从来没生效过**（2026-09-25 实测踩到）。直接设 `AcquisitionFrameRate`
    //    就行 —— 那一个是 float、支持（本机当前值本就是 30.00）。
    ret = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate",
                              static_cast<float>(frame_rate_));
    if (ret == MV_OK) {
      RCLCPP_INFO(this->get_logger(), "帧率上限 = %.1f fps（降发热）", frame_rate_);
    } else {
      // ⚠️ 不致命但要喊 —— 限不上就还是自由跑，会被当成"已经降过发热了"
      RCLCPP_WARN(this->get_logger(),
        "设帧率上限 %.1f fps 失败：0x%08x %s ⇒ **仍会自由跑**，发热照旧",
        frame_rate_, ret, err_name(ret));
    }
  }

  // ── 像素格式 —— ⚠️ 这台**不支持机内转 BGR8/RGB8**（实测 0x80000004），
  //    所以默认就是 Bayer，主机 debayer。设失败不致命（就按当前格式解）。
  ret = MV_CC_SetEnumValueByString(handle_, "PixelFormat", pixel_format_.c_str());
  if (ret != MV_OK) {
    RCLCPP_WARN(this->get_logger(),
      "设 PixelFormat=%s 失败：0x%08x %s ⇒ 按相机当前格式走，若出花屏先查这里",
      pixel_format_.c_str(), ret, err_name(ret));
  }

  // ── 曝光 / 增益 / 白平衡 ──
  MV_CC_SetEnumValueByString(handle_, "ExposureAuto", auto_exposure_.c_str());
  MV_CC_SetEnumValueByString(handle_, "GainAuto", auto_exposure_.c_str());
  MV_CC_SetEnumValueByString(handle_, "BalanceWhiteAuto", auto_white_balance_.c_str());
  if (auto_exposure_ == "Off") {
    // ⚠️ MVS 的 ExposureTime 单位是 **µs**（不是 V4L2 的 100µs）
    ret = MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_us_));
    if (ret != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "设 ExposureTime=%.0fµs 失败：0x%08x %s",
                  exposure_us_, ret, err_name(ret));
    }
    MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_db_));
  }

  return true;
}

void MvsCameraNode::close_device()
{
  if (handle_ != nullptr) {
    MV_CC_StopGrabbing(handle_);     // 没开流时失败也无所谓
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// 采集线程
// ═══════════════════════════════════════════════════════════════════════════
void MvsCameraNode::capture_loop()
{
  if (!open_device()) {
    RCLCPP_ERROR(this->get_logger(),
      "相机起不来 —— 节点**继续存活**（不拖垮容器里其它相机），但本路不会有图。"
      "判据是 `ros2 topic hz <namespace>/image_raw`，不是「进程还在」。");
    close_device();
    return;
  }

  int ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "MV_CC_StartGrabbing 失败：0x%08x %s", ret, err_name(ret));
    close_device();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "流式采集已启动");

  uint64_t frames = 0, drops = 0;
  const int64_t timeout_ns = static_cast<int64_t>(grab_timeout_ms_) * 1000000LL;

  while (running_) {
    MV_FRAME_OUT frame;
    std::memset(&frame, 0, sizeof(frame));
    ret = MV_CC_GetImageBuffer(handle_, &frame, grab_timeout_ms_);
    if (ret != MV_OK) {
      if (ret != MV_E_NODATA && ret != MV_E_GC_TIMEOUT) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
          "MV_CC_GetImageBuffer 失败：0x%08x %s", ret, err_name(ret));
      }
      ++drops;
      continue;
    }

    const MV_FRAME_OUT_INFO_EX& fi = frame.stFrameInfo;

    // ── 时间戳：`nHostTimeStamp` = **Unix 纪元毫秒**（本机实测，见头文件）──
    //    ⚠️ 单位写死成毫秒是有实测依据的，别凭感觉改；单测钉住了它。
    const rclcpp::Time capture_stamp = stamp_from_host_ms(fi.nHostTimeStamp);

    // ── 转 BGR8 ──
    cv::Mat bgr;
    const auto w = static_cast<int>(fi.nWidth);
    const auto h = static_cast<int>(fi.nHeight);
    auto* raw = static_cast<std::uint8_t*>(frame.pBufAddr);

    if (fi.enPixelType == PixelType_Gvsp_BGR8_Packed) {
      bgr = cv::Mat(h, w, CV_8UC3, raw).clone();
    } else if (fi.enPixelType == PixelType_Gvsp_RGB8_Packed) {
      cv::Mat rgb(h, w, CV_8UC3, raw);
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    } else if (fi.enPixelType == PixelType_Gvsp_BayerRG8) {
      cv::Mat raw8(h, w, CV_8UC1, raw);
      // ⚠️ 相位已**实测验证**（CFA 平面法，§八 9.14 ①）—— 改错会红蓝互换且不报错
      const int code = (debayer_ == "BG") ? cv::COLOR_BayerBG2BGR : cv::COLOR_BayerRG2BGR;
      cv::cvtColor(raw8, bgr, code);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "像素格式 0x%08x 不认识（期望 BayerRG8/BGR8/RGB8）⇒ 本帧丢弃。"
        "改 pixel_format 参数或扩展本函数的分支", static_cast<unsigned>(fi.enPixelType));
      MV_CC_FreeImageBuffer(handle_, &frame);
      ++drops;
      continue;
    }

    if (!bgr.empty()) {
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      // ⚠️ `publish` 模式只用于对照排查 —— 它会把真实采集延迟藏起来，别当默认。
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

      publish_diagnostics((publish_stamp - capture_stamp).seconds() * 1000.0,
                          (capture_stamp - publish_stamp).seconds() * 1000.0,
                          ++frames, drops);
    }

    MV_CC_FreeImageBuffer(handle_, &frame);
    (void)timeout_ns;
  }

  RCLCPP_INFO(this->get_logger(), "采集线程退出：共 %lu 帧，丢弃 %lu",
              static_cast<unsigned long>(frames), static_cast<unsigned long>(drops));
  close_device();
}

// ═══════════════════════════════════════════════════════════════════════════
// 诊断上报 —— **与 v4l2_camera 同一口径**，两条路可直接对比
// ═══════════════════════════════════════════════════════════════════════════
void MvsCameraNode::publish_diagnostics(double capture_to_publish_ms,
                                        double capture_vs_publish_ms,
                                        uint64_t frames,
                                        uint64_t drops)
{
  if (frames % 30 != 0) return;   // 限频：诊断不该比图像还贵

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = camera_name_ + "_mvs";
  st.hardware_id = serial_number_.empty() ? "first" : serial_number_;

  // 阈值与 v4l2_camera 一致（那条实测 47ms；本条 GigE 实测底噪约 18ms）
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
  add("capture_to_publish_ms", capture_to_publish_ms);
  add("capture_vs_publish_ms", capture_vs_publish_ms);
  add("frames", static_cast<double>(frames));
  add("drops",  static_cast<double>(drops));

  arr.status.push_back(st);
  pub_diag_->publish(arr);
}

}  // namespace camera
}  // namespace br_perception

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::MvsCameraNode)
