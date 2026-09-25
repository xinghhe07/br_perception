/// ═══════════════════════════════════════════════════════════════════════════════
/// test_camera_drivers.cpp — 模块 30 两个自建相机驱动的**纯函数**单元测试
///
/// ⚠️ 只测 **static 纯函数** —— 采集循环要真设备（真相机、真 `/dev/video*`、真 MVS），
///    跑不了，**别把本文件的绿当成"驱动能跑"**。能跑那件事由 §八 9.15/9.16 的实机验收覆盖。
///
/// ── 为什么这几个函数值得单测 ──
///   这一路**所有的麻烦都出在时间戳上**：
///     · `usb_cam` 的时间戳比发布时刻老 **600~800 ms**，而我们排除了四种解释才定位到它（§八 9.15）；
///     · 海康的 `nHostTimeStamp` 官方文档**没写单位**，是我们实测出来的（13 位 = 毫秒）。
///   ⇒ **单位错了不会报错，只会让时间戳静静偏掉** —— 这正是必须用单测钉死的东西。
///   ⇒ 端到端延迟从 850ms 压到 36ms，靠的就是这两个换算函数是对的。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=V4l2PureTest.*`）。
/// ⚠️ 全部用例确定性：无设备、无节点、无时间依赖。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/v4l2_camera_node.hpp"
#include "br_perception/camera/mvs_camera_node.hpp"

#include <gtest/gtest.h>

// ⚠️ 拿**内核自己的常量**做对照 —— 这是唯一能钉死 fourcc 字节序的判据。
//    自己写个期望值 0x47504A4D 只是把我算的数再抄一遍，钉不住"和 V4L2 对不对得上"。
#include <linux/videodev2.h>
// ⚠️ 同理，MVS 的错误码也要**引它自己的头**（驱动头刻意不引 MVS，见其类注释）。
//    include 路径由 CMake 的 `target_include_directories(... ${MVCAM_INCLUDE_DIR})` 给。
#include <MvErrorDefine.h>

#include <cstdint>
#include <string>

namespace br_perception {
namespace camera {

using V4 = V4l2CameraNode;
using MV = MvsCameraNode;

// ═══════════════════════════════════════════════════════════════════════════
// fourcc —— 字节序写反了不报错，只会"格式设不上"然后驱动悄悄回退
// ═══════════════════════════════════════════════════════════════════════════

TEST(V4l2PureTest, fourcc与内核常量逐字一致)
{
  // ⚠️ **这条是这组里最值钱的**：如果字节序写反，它和内核的常量就对不上，
  //    而症状是"VIDIOC_S_FMT 失败/回退到别的格式 ⇒ 花屏"，不报错、很难反查。
  EXPECT_EQ(V4::fourcc("MJPG"), static_cast<std::uint32_t>(V4L2_PIX_FMT_MJPEG));
  EXPECT_EQ(V4::fourcc("YUYV"), static_cast<std::uint32_t>(V4L2_PIX_FMT_YUYV));
  EXPECT_EQ(V4::fourcc("RGGB"), static_cast<std::uint32_t>(V4L2_PIX_FMT_SRGGB8));
}

TEST(V4l2PureTest, fourcc是低字节在前)
{
  // 'M'=0x4D 在最低字节 ⇒ 写反了会变成 0x4D504A47，与内核常量对不上
  EXPECT_EQ(V4::fourcc("MJPG"), 0x47504A4Du);
}

// ═══════════════════════════════════════════════════════════════════════════
// stamp_from_monotonic —— **本驱动存在的理由**
// ═══════════════════════════════════════════════════════════════════════════

TEST(V4l2PureTest, 缓冲与当前同时刻时偏移为零)
{
  const rclcpp::Time ros_now(1000000000LL, RCL_ROS_TIME);   // 1e9 ns
  const auto got = V4::stamp_from_monotonic(5000LL, 5000LL, ros_now);
  EXPECT_EQ(got.nanoseconds(), ros_now.nanoseconds());
}

TEST(V4l2PureTest, 缓冲早于当前多少就回溯多少)
{
  // 单调钟现在 1e9，缓冲是 1e9−800e6 ⇒ 缓冲距今 800ms
  // ⇒ 结果必须 = ros_now − 800ms
  const rclcpp::Time ros_now(5000000000LL, RCL_ROS_TIME);
  const auto got = V4::stamp_from_monotonic(200000000LL, 1000000000LL, ros_now);
  EXPECT_EQ(got.nanoseconds(), ros_now.nanoseconds() - 800000000LL)
      << "回溯量错了 = 时间戳会静默偏掉，而那正是 usb_cam 的病";
}

TEST(V4l2PureTest, 每帧重取两次now必须得到不同结果)
{
  // ⚠️⚠️ **这条钉的是 usb_cam 那个坑的反面**：
  //    如果实现成"初始化时算一次偏移存起来"，那么无论传进来的 mono_now 是多少，
  //    结果都只随 ros_now 变 —— 系统时钟被 NTP 调过之后就**永久错**。
  //    正确实现下，mono_now 变了、ros_now 不变 ⇒ 结果必须跟着变。
  const rclcpp::Time ros_now(1000000000LL, RCL_ROS_TIME);
  const auto a = V4::stamp_from_monotonic(0LL, 100LL, ros_now);
  const auto b = V4::stamp_from_monotonic(0LL, 900LL, ros_now);
  EXPECT_EQ((a - b).nanoseconds(), 800LL)
      << "结果不随 mono_now 变 ⇒ 说明把偏移缓存起来了（usb_cam 的坑）";
}

TEST(V4l2PureTest, 缓冲在未来时如实返回未来时刻不钳零)
{
  // ⚠️ 合本仓库 SlotStatus.msg 的口径："时间是负的"本身就是要暴露的异常，
  //    **不钳到 0**。这里 buf 比 now 还新 100ms ⇒ 结果必须在未来。
  const rclcpp::Time ros_now(1000000000LL, RCL_ROS_TIME);
  const auto got = V4::stamp_from_monotonic(1100000000LL, 1000000000LL, ros_now);
  EXPECT_GT(got.nanoseconds(), ros_now.nanoseconds());
}

// ═══════════════════════════════════════════════════════════════════════════
// stamp_from_host_ms —— 海康 `nHostTimeStamp` 的单位
// ═══════════════════════════════════════════════════════════════════════════

TEST(MvsPureTest, 本机实测的那对输入输出逐位对上)
{
  // ⚠️ 这是 2026-09-25 现场抓到的那一对（§八 9.14 ②）：
  //      nHostTimeStamp = 1790316463495（13 位）
  //      host_realtime  = 1790316463512594070 ns（19 位）
  //    ⇒ 毫秒。**精确值钉死**，改单位这里立刻红。
  const auto got = MV::stamp_from_host_ms(1790316463495LL);
  EXPECT_EQ(got.nanoseconds(), 1790316463495000000LL);
}

TEST(MvsPureTest, 量级合理性改成年或微秒都能被抓住)
{
  // ⚠️ **这条才是防呆的重点**：精确值那条只钉了一个数，这条钉**量级**。
  //    2026 年的 Unix 纪元落在 1.7e18 ~ 1.9e18 ns。
  //    若有人把单位改成年（数值≈1.7e9）或微秒（≈1.7e15），结果会落到荒谬的纪元。
  const auto got = MV::stamp_from_host_ms(1790316463495LL).nanoseconds();
  EXPECT_GT(got, 1700000000000000000LL) << "偏小 ⇒ 单位可能写成了秒/微秒";
  EXPECT_LT(got, 1900000000000000000LL) << "偏大 ⇒ 单位可能写成了纳秒后又被放大了";
}

TEST(MvsPureTest, 与采集链路的实测底噪吻合)
{
  // 实测：nHostTimeStamp 只滞后真机时钟 **17.6 ms**（§八 9.14 ②）。
  // 换算后两者的差应当就在这个量级（用例里给 100ms 的松容差，
  // 因为这条是**量级校验**，精确值由上面那条钉）。
  const int64_t host_ms = 1790316463495LL;
  const int64_t real_ns = 1790316463512594070LL;
  const int64_t delta_ms = (real_ns - MV::stamp_from_host_ms(host_ms).nanoseconds()) / 1000000LL;
  EXPECT_GE(delta_ms, 0);
  EXPECT_LT(delta_ms, 100) << "滞后超过 100ms ⇒ 要么单位错了，要么链路真有积压";
}

// ═══════════════════════════════════════════════════════════════════════════
// mvs_str —— MVS 的定长 char[] 不保证带 NUL
// ═══════════════════════════════════════════════════════════════════════════

TEST(MvsPureTest, 带NUL时截到NUL)
{
  const char buf[16] = {'A', 'B', 'C', 0, 'X', 'Y'};
  EXPECT_EQ(MV::mvs_str(buf, 16), "ABC");
}

TEST(MvsPureTest, 没有NUL时取满n个不越界)
{
  // ⚠️ 这是真实场景：`chSerialNumber` 定长 16，序列号正好占满就没 NUL。
  //    若实现成 `std::string(buf)`，会一直读到下一个 NUL ⇒ 越界或带进垃圾。
  const char buf[4] = {'D', 'B', '2', '6'};
  EXPECT_EQ(MV::mvs_str(buf, 4), "DB26");
}

TEST(MvsPureTest, 首个字节就是NUL时为空串)
{
  const char buf[8] = {0, 'X', 'Y'};
  EXPECT_EQ(MV::mvs_str(buf, 8), "");
}

TEST(MvsPureTest, 本机那台的真实序列号)
{
  // 现场值是 DB2615603（§八 9.14）；MVS 的 chSerialNumber 是定长数组，
  // 后面跟着未初始化的字节 ⇒ 这段断言的就是"有没有被垃圾带进来"。
  char buf[16] = {};
  const char sn[] = "DB2615603";
  for (int i = 0; i < 9; ++i) buf[i] = sn[i];
  buf[9]  = '\0';
  buf[10] = static_cast<char>(0xAB);   // 模拟后面的垃圾
  EXPECT_EQ(MV::mvs_str(buf, 16), "DB2615603");
}

// ═══════════════════════════════════════════════════════════════════════════
// err_name —— 错误码在 0x80000000 这一档，按 int 比会编不过
// ═══════════════════════════════════════════════════════════════════════════

TEST(MvsPureTest, 认得的错误码给出名字)
{
  EXPECT_STREQ(MV::err_name(MV_OK), "MV_OK");
  // ⚠️ `MV_E_ACCESS_DENIED` 是本轮**实测撞到过**的那个：
  //    上一轮的节点没清干净、占着相机，新节点打开就报它（§八 9.17 ②）。
  EXPECT_NE(std::string(MV::err_name(MV_E_ACCESS_DENIED)).find("ACCESS_DENIED"),
            std::string::npos);
  EXPECT_NE(std::string(MV::err_name(MV_E_SUPPORT)).find("SUPPORT"), std::string::npos);
}

TEST(MvsPureTest, 不认得的码返回兜底而不是空指针)
{
  // ⚠️ 返回值**必须非空** —— 调用方是 `RCLCPP_ERROR("...%s", err_name(ret))`，
  //    返回 nullptr 会让 printf 系列直接 UB（很可能是段错误）。
  EXPECT_NE(MV::err_name(0x12345678), nullptr);
  EXPECT_NE(MV::err_name(-1), nullptr);
}

}  // namespace camera
}  // namespace br_perception
