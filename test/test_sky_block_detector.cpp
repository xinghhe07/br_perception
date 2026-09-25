/// ═══════════════════════════════════════════════════════════════════════════════
/// test_sky_block_detector.cpp — 模块 9「天空方块顶面颜色」单元测试（任务书 §3.3）
///
/// ⚠️ 本测试**不起 ROS 节点、不建 ROS 图**（只有 gtest + OpenCV + 消息结构体）：
///    被测的四样东西全是 **static 纯函数 / 纯状态机** ——
///      ① `judge_top_color`      判色核心（那 40/80 分的判据）
///      ② `top_face_roi`         兜底路的"上半区域"启发式
///      ③ `TemporalConfirmer`    时序确认（规则 3.5.8：对手会翻塔，一帧抖动不能翻归属）
///      ④ `project_with_transform` **投影几何**（2026-09-24 新加的投影主路）
///    ⇒ 这正是 2026-09-23 把判色核心做成 `static` 的原因（头文件原话：
///       "单测可以直接喂 cv::Mat 调它"）。④ 是 2026-09-24 为同样理由**专门从
///       `project_spot` 里摘出来的** —— 原来逐点调 TF，测试里喂不进去。
///
/// ⚠️ **TEST 名用中文**（贴合任务书，与 `test_coordinate_transformer.cpp` / `test_geometry_utils.cpp`
///    一致），但**测试组名（第一个参数）全是 ASCII** —— 过滤请用组名：
///    `--gtest_filter=ProjectGeometryTest.*`。别用中文 pattern（撞 locale）。
///
/// ⚠️ 全部用例**确定性**：图像由代码画出来、位姿手算给定，无随机、无 sleep、无时间依赖。
///
/// ⚠️⚠️ 本文件重点钉住**四处容易写错且静默出错**的地方（各自有专门用例）：
///    1. **机身系 → 光心系**（`body2opt`）—— 漏了它投影**整体错 90°**，
///       而图像上"看着还有东西"（只是位置全错），肉眼看 RViz 发现不了。
///       用"相机在塔顶正上方朝下看"钉死：顶面必须落在画面正中、边长手算得出。
///    2. **letterbox 是正向变换**（原图 → 收到的图），方向抄反会让 ROI 跑到别处。
///    3. **`3 (UNKNOWN)` 与 `0 (NONE)` 必须分开**（头文件边界①）：
///       "有方块但判不出色" ≠ "这里没有方块"。
///    4. **时序状态机进了 COASTING 必须能出来**（2026-09-23 Python 穷举抓到的真 bug：
///       退出只在 `kConfirmed` 时判 ⇒ 卡死在 COASTING 再也回不到 UNCONFIRMED）。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/sky_block_detector.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <string>

namespace br_perception {
namespace camera {
namespace {

/// 一片纯色 BGR 图
cv::Mat solid(int w, int h, const cv::Scalar& bgr)
{
  return cv::Mat(h, w, CV_8UC3, bgr);
}

/// 给 HSV 阈值造一组"宽到不会误伤"的范围（测试只关心**判据**，不关心阈值调得好不好）
vision::ColorThreshold make_th(const char* label,
                               const cv::Scalar& lo, const cv::Scalar& hi)
{
  vision::ColorThreshold t;
  t.label = label;
  t.groups.emplace_back(lo, hi);
  return t;
}

/// 红：HSV 里 H≈0（OpenCV 的 8U H ∈ [0,180)）；蓝：H≈120
vision::ColorThreshold red_th()  { return make_th("red",  {0, 80, 60},   {10, 255, 255}); }
vision::ColorThreshold blue_th() { return make_th("blue", {110, 80, 60}, {130, 255, 255}); }

SkyBlockDetector::ColorParams default_params()
{
  SkyBlockDetector::ColorParams p;
  p.decisive_ratio   = 0.70;
  p.close_lo         = 0.45;
  p.close_hi         = 0.55;
  p.min_valid_pixels = 80;
  p.min_valid_ratio  = 0.25;
  return p;
}

/// 摆一个"某位上有完整塔"的世界，返回 世界 → 相机**机身系** 的变换。
///
///@param cam_world        相机光心在世界系里的位置
///@param look_down         true = 朝下看（机头 +X 指世界 −Z，机身 +Z 指世界 +X）
///                         false = 水平看（机身系 ≡ 世界系朝向，+X 指世界 +X）
geometry_msgs::msg::TransformStamped make_tf(double cx, double cy, double cz,
                                             bool look_down)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "world";
  tf.child_frame_id  = "camera_wide_frame";

  if (look_down) {
    // 机身 X = 世界 −Z（镜头朝下）、机身 Y = 世界 +Y、机身 Z = 世界 +X
    // ⇒ 世界 → 机身 的旋转 = 绕 Y 轴 −90°，四元数 (w,x,y,z) = (cos45, 0, −sin45, 0)
    const double s = std::sqrt(0.5);
    tf.transform.rotation.w = s;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = -s;
    tf.transform.rotation.z = 0.0;
    // 平移 = −R·p_cam（把光心映射到机身原点）
    tf.transform.translation.x =  cz;    // R·(cx,cy,cz) = (−cz, cy, cx)
    tf.transform.translation.y = -cy;
    tf.transform.translation.z = -cx;
  } else {
    tf.transform.rotation.w = 1.0;       // 机身系朝向 = 世界系朝向（水平看）
    tf.transform.translation.x = -cx;
    tf.transform.translation.y = -cy;
    tf.transform.translation.z = -cz;
  }
  return tf;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// ① 判色核心
// ═══════════════════════════════════════════════════════════════════════════════
class JudgeColorTest : public ::testing::Test {};

TEST_F(JudgeColorTest, 全红判RED)
{
  const cv::Mat img = solid(100, 100, cv::Scalar(0, 0, 255));      // BGR: 红
  const auto v = SkyBlockDetector::judge_top_color(
      img, cv::Rect(0, 0, 100, 100), red_th(), blue_th(), default_params());
  EXPECT_EQ(v.color, SkyBlockDetector::kRed);
  EXPECT_EQ(v.reason, SkyBlockDetector::kReasonNone);
  EXPECT_EQ(v.valid_pixel_count, 100 * 100);
  EXPECT_NEAR(v.red_ratio, 1.0, 1e-9);
}

TEST_F(JudgeColorTest, 全蓝判BLUE)
{
  const cv::Mat img = solid(100, 100, cv::Scalar(255, 0, 0));      // BGR: 蓝
  const auto v = SkyBlockDetector::judge_top_color(
      img, cv::Rect(0, 0, 100, 100), red_th(), blue_th(), default_params());
  EXPECT_EQ(v.color, SkyBlockDetector::kBlue);
  EXPECT_NEAR(v.blue_ratio, 1.0, 1e-9);
}

TEST_F(JudgeColorTest, 红蓝各半报RATIO_AMBIGUOUS而不是NONE)
{
  // ⚠️⚠️ 这是**模型/实物错了**时的症状（见 field_geometry.yaml 的裁定）：
  //    顶面恒半红半蓝 ⇒ 恒 ≈50/50 ⇒ `RATIO_AMBIGUOUS`（"判不出"）。
  //    ⚠️ 它**不是** `NONE` —— "有方块但判不出色"与"这里没有方块"是两件事。
  cv::Mat img = solid(100, 100, cv::Scalar(0, 0, 255));
  img(cv::Rect(50, 0, 50, 100)).setTo(cv::Scalar(255, 0, 0));
  const auto v = SkyBlockDetector::judge_top_color(
      img, cv::Rect(0, 0, 100, 100), red_th(), blue_th(), default_params());
  EXPECT_EQ(v.color, SkyBlockDetector::kUnknown);
  EXPECT_EQ(v.reason, SkyBlockDetector::kReasonRatioAmbiguous);
  EXPECT_NEAR(v.red_ratio, 0.5, 0.02);
}

TEST_F(JudgeColorTest, 一个红蓝像素都没有判NONE)
{
  // 木色/地面 —— 说明**这里没有方块**（与 UNKNOWN 分开，头文件边界①）
  const cv::Mat img = solid(100, 100, cv::Scalar(128, 128, 128));
  const auto v = SkyBlockDetector::judge_top_color(
      img, cv::Rect(0, 0, 100, 100), red_th(), blue_th(), default_params());
  EXPECT_EQ(v.color, SkyBlockDetector::kNone);
  EXPECT_EQ(v.reason, SkyBlockDetector::kReasonNone);
}

TEST_F(JudgeColorTest, 有效像素太少判LOW_VALID_PIXELS)
{
  // 900 px 里只有 9 个红点 = 100% 红 —— 但**不该**与 600/900 同等可信（头文件边界②）
  cv::Mat img = solid(30, 30, cv::Scalar(128, 128, 128));
  img(cv::Rect(0, 0, 3, 3)).setTo(cv::Scalar(0, 0, 255));
  const auto v = SkyBlockDetector::judge_top_color(
      img, cv::Rect(0, 0, 30, 30), red_th(), blue_th(), default_params());
  EXPECT_EQ(v.color, SkyBlockDetector::kUnknown);
  EXPECT_EQ(v.reason, SkyBlockDetector::kReasonLowValidPixels);
}

TEST_F(JudgeColorTest, 六十比四十落进未定义段判UNKNOWN)
{
  // ⚠️ §3.3 只定义了 ">70% 定色" 与 "45~55% 判未知"，**中间那段没写** ——
  //    本模块取最保守读法 ⇒ **一律 UNKNOWN**。这条用例把那个取舍钉住：
  //    将来谁想把 decisive_ratio 调到 0.6，会在这里看到这条用例的红。
  cv::Mat img = solid(100, 100, cv::Scalar(0, 0, 255));
  img(cv::Rect(60, 0, 40, 100)).setTo(cv::Scalar(255, 0, 0));
  const auto v = SkyBlockDetector::judge_top_color(
      img, cv::Rect(0, 0, 100, 100), red_th(), blue_th(), default_params());
  EXPECT_EQ(v.color, SkyBlockDetector::kUnknown);
}

TEST_F(JudgeColorTest, ROI越界按交集算且note里说明)
{
  const cv::Mat img = solid(50, 50, cv::Scalar(0, 0, 255));
  const auto v = SkyBlockDetector::judge_top_color(
      img, cv::Rect(40, 40, 40, 40), red_th(), blue_th(), default_params());   // 部分出界
  EXPECT_EQ(v.color, SkyBlockDetector::kRed);                 // 交集里全是红
  EXPECT_NE(v.note.find("越界"), std::string::npos);          // ⚠️ 话要说出来
}

TEST_F(JudgeColorTest, 空ROI判ROI_TOO_SMALL而不是崩)
{
  const cv::Mat img = solid(50, 50, cv::Scalar(0, 0, 255));
  const auto v = SkyBlockDetector::judge_top_color(
      img, cv::Rect(200, 200, 10, 10), red_th(), blue_th(), default_params());
  EXPECT_EQ(v.color, SkyBlockDetector::kUnknown);
  EXPECT_EQ(v.reason, SkyBlockDetector::kReasonRoiTooSmall);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ② 兜底路的"上半区域"
// ═══════════════════════════════════════════════════════════════════════════════
TEST(TopFaceRoiTest, 取上半区域)
{
  const cv::Rect r = SkyBlockDetector::top_face_roi(cv::Rect(10, 20, 40, 60), 0.5);
  EXPECT_EQ(r, cv::Rect(10, 20, 40, 30));
}

TEST(TopFaceRoiTest, 比例1不切)
{
  const cv::Rect r = SkyBlockDetector::top_face_roi(cv::Rect(10, 20, 40, 60), 1.0);
  EXPECT_EQ(r, cv::Rect(10, 20, 40, 60));
}

TEST(TopFaceRoiTest, 比例越界被夹住)
{
  // ⚠️ 头文件写明"夹到 (0,1]：给了 0 或负数会让高度变 0 ⇒ 后面判成无交集，
  //    而症状是恒判 UNKNOWN" —— 这里钉住它。
  EXPECT_GT(SkyBlockDetector::top_face_roi(cv::Rect(0, 0, 40, 60), 0.0).height, 0);
  EXPECT_LE(SkyBlockDetector::top_face_roi(cv::Rect(0, 0, 40, 60), 9.9).height, 60);
}

// ═══════════════════════════════════════════════════════════════════════════════
// ③ 时序确认状态机
// ═══════════════════════════════════════════════════════════════════════════════
TEST(TemporalConfirmerTest, 连续满足到确认)
{
  SkyBlockDetector::TemporalConfirmer c;
  c.enter_frames = 2; c.exit_frames = 3;
  EXPECT_EQ(c.push(true),  SkyBlockDetector::TemporalConfirmer::kUnconfirmed);  // 第 1 帧不够
  EXPECT_EQ(c.push(true),  SkyBlockDetector::TemporalConfirmer::kConfirmed);
}

TEST(TemporalConfirmerTest, 掉帧先进COASTING再回UNCONFIRMED)
{
  SkyBlockDetector::TemporalConfirmer c;
  c.enter_frames = 2; c.exit_frames = 3;
  c.push(true); c.push(true);                                        // CONFIRMED
  EXPECT_EQ(c.push(false), SkyBlockDetector::TemporalConfirmer::kCoasting);
  EXPECT_EQ(c.push(false), SkyBlockDetector::TemporalConfirmer::kCoasting);
  EXPECT_EQ(c.push(false), SkyBlockDetector::TemporalConfirmer::kUnconfirmed);  // 到 exit
}

TEST(TemporalConfirmerTest, 卡死在COASTING是回归项)
{
  // ⚠️⚠️ **这条是给那个真 bug 立的碑**（2026-09-23 Python 穷举抓到）：
  //    退出原来只在 `state == kConfirmed` 时判 ⇒ 一旦进了 `kCoasting`
  //    **永远出不来**（状态卡死，再也不回 UNCONFIRMED）。改成两档一起判才修好。
  SkyBlockDetector::TemporalConfirmer c;
  c.enter_frames = 2; c.exit_frames = 2;
  c.push(true); c.push(true);                                  // CONFIRMED
  for (int i = 0; i < 10; ++i) c.push(false);                  // 一路掉
  EXPECT_EQ(c.state, SkyBlockDetector::TemporalConfirmer::kUnconfirmed);
}

TEST(TemporalConfirmerTest, reset清干净滞回)
{
  SkyBlockDetector::TemporalConfirmer c;
  c.enter_frames = 2; c.exit_frames = 3;
  c.push(true); c.push(true);
  c.reset();
  EXPECT_EQ(c.state, SkyBlockDetector::TemporalConfirmer::kUnconfirmed);
  EXPECT_EQ(c.push(true), SkyBlockDetector::TemporalConfirmer::kUnconfirmed);   // 重新数
}

// ═══════════════════════════════════════════════════════════════════════════════
// ④ 投影几何（2026-09-24 新加的主路）—— **手算得出答案**，所以能钉死约定
// ═══════════════════════════════════════════════════════════════════════════════
class ProjectGeometryTest : public ::testing::Test
{
protected:
  // 塔：位中心 (0,0)、平台高 0（塔直接立在地面 ⇒ 顶面 z=0.90）
  geometry_msgs::msg::Point spot;
  const double platform_z = 0.0;
  const double block      = 0.20;    // 天空方块 200mm
  const double tower      = 0.90;    // 完整塔高

  // 仿真相机内参（与 bringup_debug 的 SIM_INTRINSICS 同源：640×384、hfov=1.1）
  const double fx = 521.9, fy = 521.9, cx = 320.0, cy = 192.0;
  const int    img_w = 640, img_h = 384;

  void SetUp() override { spot.x = 0.0; spot.y = 0.0; spot.z = 0.0; }

  SkyBlockDetector::SpotProjection run(const geometry_msgs::msg::TransformStamped& tf,
                                       double scale = 1.0, int pad = 0)
  {
    return SkyBlockDetector::project_with_transform(
        tf, /*spot_id=*/3, spot, platform_z, block, tower,
        fx, fy, cx, cy, scale, pad, pad, img_w, img_h);
  }
};

TEST_F(ProjectGeometryTest, 相机在塔顶正上方朝下看_顶面落在画面正中)
{
  // 相机光心 (0,0,2.0) 朝下看 ⇒ 顶面（z=0.90）距相机 1.10m ⇒
  //   顶面边长 0.2m 投影 = fx · 0.2 / 1.10 = 521.9 × 0.18182 = 94.9 px
  //   且**中心必须正好在 (cx, cy)**  —— 这一条就是"机身系→光心系"那步的探针：
  //   漏了它，方框会整体错位/退化（而图像上"看着还有东西"，肉眼发现不了）。
  const auto tf = make_tf(0.0, 0.0, 2.0, /*look_down=*/true);
  const auto p  = run(tf);

  ASSERT_TRUE(p.valid) << p.note;
  EXPECT_NEAR(p.face_h_px, fx * block / 1.10, 1.0);
  EXPECT_NEAR(p.face_w_px, fx * block / 1.10, 1.0);
  EXPECT_NEAR(p.roi_lb.x + p.roi_lb.width  * 0.5, cx, 2.0);
  EXPECT_NEAR(p.roi_lb.y + p.roi_lb.height * 0.5, cy, 2.0);
  // ⚠️ 下面两条在 2026-09-24 抓出过一个**真错**：那时实现里 `grazing_deg` 的
  //    **方向**（写成了"相机→方块"）与**参考点**（写成了方块中心）都反了 ⇒
  //    正上方算出 180°、距离算出 1.20m（到方块中心），而门控整套跟着反。
  //    ⇒ 这两条是"门控语义"的探针，别把期望值改成"跑出来是几就写几"。
  EXPECT_NEAR(p.grazing_deg, 0.0, 1.0);        // 正上方看 ⇒ 夹角 0°（实测 180° 那次就是它报的）
  EXPECT_NEAR(p.distance_m, 1.10, 1e-6);       // 到**顶面中心**，不是到方块中心（后者是 1.20）
}

TEST_F(ProjectGeometryTest, 水平平视_顶面被压成一条线_门控该拦)
{
  // 相机与塔顶同高（z=0.90）、水平看 ⇒ 顶面四角都落在同一条水平线上
  //   ⇒ `face_h_px ≈ 0` ⇒ 调用方的掠射门控（min_face_h_px=6）**必须拦下**。
  //   ⚠️ 这条用例就是"门控值不值得信"的量化依据。
  //   ⚠️ 相机必须在塔的 **−X 侧**（机身 +X 指世界 +X ⇒ 朝 +X 看）；放 +X 侧就背对了。
  //   手算（本机 Python 复现同一公式）：face_h = 0.000 px、face_w = 116.0 px、
  //   夹角 = **90.00°**（相机与**顶面**同高 ⇒ 正好 90°）、距离 = 1.000 m —— 留档如下。
  //   ⚠️ 夹角与距离这两栏 2026-09-24 更正过：此前记的是 95.71° / 1.005 m，那是
  //      **方向取反 + 参考点取成"方块中心"**（比顶面低半块）的结果 —— 两处都改掉之后
  //      正好落在 90.00° / 1.000 m。**这道 90° 边界同时也是"门控判据"的一半**：
  //      相机与顶面同高时夹角正好顶到上限，与 `face_h_px≈0` 一起把它拦下。
  const auto tf = make_tf(-1.0, 0.0, 0.90, /*look_down=*/false);
  const auto p  = run(tf);

  ASSERT_TRUE(p.valid) << p.note;
  EXPECT_LT(p.face_h_px, 1.0);                 // 压成一条线（实测 0.000）
  EXPECT_GT(p.grazing_deg, 89.0);              // 夹角 > 上限 ⇒ 会被 max_grazing_deg 拦（实测 90.00）
  EXPECT_GT(p.face_w_px, 100.0);               // ⚠️ 宽度**几乎不变**（实测 116.0）——
                                               //    这正是"掠射用高度判、不用宽度判"的理由
}

TEST_F(ProjectGeometryTest, letterbox是正向变换_ROI跟着缩放与平移)
{
  // ⚠️ 内参描述的是**原图**，而 ROI 要落在**收到的（letterbox 之后的）图**上。
  //    方向抄反（像检测器那样"反算回原图"）会让 ROI 跑到别处 —— 这条用例钉住方向。
  const auto tf = make_tf(0.0, 0.0, 2.0, /*look_down=*/true);
  const auto p1 = run(tf, /*scale=*/0.5, /*pad=*/20);
  const auto p0 = run(tf);

  ASSERT_TRUE(p1.valid) << p1.note;
  EXPECT_NEAR(p1.face_h_px, p0.face_h_px * 0.5, 0.5);          // 缩放
  EXPECT_NEAR(p1.roi_lb.x + p1.roi_lb.width * 0.5,
              (p0.roi_lb.x + p0.roi_lb.width * 0.5) * 0.5 + 20.0, 2.0);  // 平移
}

TEST_F(ProjectGeometryTest, 相机背对塔_报不可用而不是给个假框)
{
  // 相机在塔的 **+X 侧**、机身 +X 仍指世界 +X（水平看，**背对**塔 ⇒ 塔在身后）
  //   ⚠️⚠️ 2026-09-24 复核修：这条用例原先只改四元数（`rotation.z=1, w=0`）而
  //      **没改平移**，于是变换不再自洽 —— `make_tf` 的平移是配着"单位旋转"算的
  //      （`t = −R·p_cam`），只转旋转 ⇒ 隐含的相机被搬到 `(+1, 0, 0.90)` **并转身
  //      面朝塔**（本机 Python 复算：机头 +X 对应世界 −X，塔正在正前方、opt.z>0）
  //      ⇒ `valid` 为 true，`EXPECT_FALSE(valid)` 恒挂。**背对必须整体一致地构造**：
  //      换一次 `make_tf` 调用，平移会跟着一起给对。
  const auto tf = make_tf(1.0, 0.0, 0.90, /*look_down=*/false);   // 平移 = −R·p_cam，一起给了
  const auto p  = run(tf);

  EXPECT_FALSE(p.valid);
  EXPECT_FALSE(p.note.empty());                // ⚠️ 失败必须有话说，不能静默
}

TEST_F(ProjectGeometryTest, 方块出画_与取不到位置分开报)
{
  // 相机朝下但在水平方向离塔很远（100m）⇒ 塔顶投影到画面外极远处
  //   （本机手算：u=320、v=47637 —— 远在 384 高的画面外）⇒ ROI 与画面**无交集**。
  //   ⚠️ 这条与"投影失败"必须是**两种不同的 note**：前者"塔在画外（靠近就能看见）"，
  //      后者"TF/内参出问题（靠近也没用）"—— 现场的动作完全不同。
  const auto tf = make_tf(100.0, 0.0, 2.0, /*look_down=*/true);
  const auto p  = run(tf);

  EXPECT_FALSE(p.valid);
  EXPECT_NE(p.note.find("出画"), std::string::npos) << p.note;
}

TEST_F(ProjectGeometryTest, 参数非法时明确拒绝)
{
  const auto tf = make_tf(0.0, 0.0, 2.0, true);
  // 方块边长 0 ⇒ 拒绝（不是算出一个退化框）
  const auto a = SkyBlockDetector::project_with_transform(
      tf, 3, spot, platform_z, /*block=*/0.0, tower,
      fx, fy, cx, cy, 1.0, 0, 0, img_w, img_h);
  EXPECT_FALSE(a.valid);
  EXPECT_FALSE(a.note.empty());

  // 图像尺寸 0 ⇒ 拒绝
  const auto b = SkyBlockDetector::project_with_transform(
      tf, 3, spot, platform_z, block, tower,
      fx, fy, cx, cy, 1.0, 0, 0, /*img_w=*/0, /*img_h=*/0);
  EXPECT_FALSE(b.valid);
  EXPECT_FALSE(b.note.empty());
}

}  // namespace camera
}  // namespace br_perception
