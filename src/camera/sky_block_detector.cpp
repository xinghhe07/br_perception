/// ═══════════════════════════════════════════════════════════════════════════════
/// sky_block_detector.cpp — 天空方块「顶面颜色」判定（任务书 §3.3）
///
/// **设计理由、三条边界、两个规格未定义段的取舍、以及那条"红蓝怎么分布"的
/// 未决前提，全部写在头文件里。** 本文件只实现，不重复论证。
///
/// ⚠️ 本文件**一行都没编译过**（本机无 ROS、无 OpenCV C++）。
///    ⚠️ 但**判色核心与时序状态机已在 Python 里穷举验过**（见下），
///       而且那轮验证抓出过一个真 bug（时序状态机卡死）。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/sky_block_detector.hpp"

// ⚠️ 这个头**必须显式 include**：漏了的话报错长得像语法错
//    （`expected constructor, destructor, or type conversion at end of input`，
//     指向文件最后一行）—— 2026-09-23 在另外两个节点上撞过，记在这里。
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

// ⚠️ letterbox 变换的**唯一实现**在这里，别在本文件里重写一份。
//    ⚠️ 注意方向：检测器那边是**反算**（原图←letterbox），本文件投影是**正算**
//       （原图→letterbox，用 `to_box_x/to_box_y`）。**别抄错方向。**
#include "br_perception/camera/image_preprocessor.hpp"

// ⚠️⚠️ **这一个不是可选的**（`mustika_pillar_check` 上正是漏它）：
//    `tf2_ros::Buffer::transform<PointStamped>()` 内部调 `tf2::doTransform`，
//    而 `geometry_msgs::msg::PointStamped` 的那个重载**声明在 tf2_geometry_msgs 里**。
//    CMake 的 `ament_target_dependencies` 只给编译/链接路径，**不会**把重载带进来。
//    （另：`tf2/exceptions.h` / `tf2/time.h` 也别指望传递包含。）
#include <tf2/exceptions.h>
#include <tf2/time.h>
// ⚠️ **不再需要 `tf2_geometry_msgs`**（2026-09-24 重构后删）：TF 只用来**查一次**
//    变换（`lookupTransform`），点变换自己做（四元数直接算）。
//    ⚠️ 那个包是给 `Buffer::transform<PointStamped>` 的重载用的 —— 谁把"逐点变换"
//       改回来（每个角点各查一次），就必须把它加回 include 与 CMake，
//       否则报 `no matching function for call to 'doTransform'`（`mustika_pillar_check` 栽过）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace br_perception {
namespace camera {

namespace {

inline std::chrono::steady_clock::time_point mono_now()
{
  return std::chrono::steady_clock::now();
}

inline double ms_since(const std::chrono::steady_clock::time_point& t0)
{
  return std::chrono::duration<double, std::milli>(mono_now() - t0).count();
}

const char* color_name(uint8_t c)
{
  switch (c) {
    case 0: return "NONE";
    case 1: return "RED";
    case 2: return "BLUE";
    case 3: return "UNKNOWN";
    default: return "?";
  }
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// 时序确认状态机
//
// ⚠️⚠️ 已在 Python 里穷举验过（抖动 / 翻转 / 翻回来），并抓出过一个真 bug：
//     退出那条**原先只在 `state == kConfirmed` 时判断** ⇒ 一旦进了 `kCoasting`
//     就**永远出不来**（状态卡死，再也回不到 UNCONFIRMED）。
//     ⇒ 下面把 `kConfirmed` 与 `kCoasting` **放在一起判**，就是修这个。
//     改这段之前先回去跑那三个序列，别凭读代码判断它对不对。
// ═══════════════════════════════════════════════════════════════════════════════
SkyBlockDetector::TemporalConfirmer::State
SkyBlockDetector::TemporalConfirmer::push(bool satisfied)
{
  if (satisfied) {
    in_count_++;
    out_count_ = 0;
    if (state != kConfirmed && in_count_ >= std::max(1, enter_frames)) {
      state = kConfirmed;
    }
  } else {
    out_count_++;
    in_count_ = 0;
    // ⚠️ **两档一起判** —— 只判 kConfirmed 会让状态卡在 kCoasting 出不来（见上）。
    if (state == kConfirmed || state == kCoasting) {
      state = (out_count_ < std::max(1, exit_frames)) ? kCoasting : kUnconfirmed;
    }
  }
  return state;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 判色核心（纯函数，可单测）
// ═══════════════════════════════════════════════════════════════════════════════
SkyBlockDetector::ColorVerdict SkyBlockDetector::judge_top_color(
    const cv::Mat& bgr,
    const cv::Rect& roi,
    const vision::ColorThreshold& red,
    const vision::ColorThreshold& blue,
    const ColorParams& p)
{
  ColorVerdict v;

  if (bgr.empty()) {
    v.color = kUnknown; v.reason = kReasonRoiTooSmall; v.note = "空图";
    return v;
  }
  if (bgr.type() != CV_8UC3) {
    // ⚠️ 不静默转：本模块的契约是 BGR8（`cv_bridge` 那侧已确保）。
    //    到这里说明上游改了编码 —— 正是 2026-09-20 踩过的那类坑
    //    （`enable_normalize` 一开，检测器 `toCvShare(BGR8)` 直接抛异常）。
    v.color = kUnknown; v.reason = kReasonOccluded;
    v.note = "图像不是 CV_8UC3（BGR8）—— 去查 image_preprocessor 的编码设置";
    return v;
  }

  const cv::Rect r = roi & cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (r.width <= 0 || r.height <= 0) {
    v.color = kUnknown; v.reason = kReasonRoiTooSmall;
    v.note = "ROI 与原图无交集（越界？）—— 没算";
    return v;
  }
  std::string pre;
  if (r != roi) {
    // ⚠️ 部分越界**要说出来**：不报的话，"位置填偏了一点"会表现成"颜色判错"，
    //    而现场会去调 HSV 阈值 —— 查错方向。
    pre = "⚠️ ROI 有部分越界，已按交集算 | ";
  }

  const cv::Mat sub = bgr(r);
  cv::Mat hsv;
  cv::cvtColor(sub, hsv, cv::COLOR_BGR2HSV);

  auto count_hits = [&hsv](const vision::ColorThreshold& th) -> int {
    if (th.groups.empty()) return 0;
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    for (const auto& g : th.groups) {
      cv::Mat m;
      cv::inRange(hsv, g.lower, g.upper, m);
      mask |= m;
    }
    return cv::countNonZero(mask);
  };
  const int red_px  = count_hits(red);
  const int blue_px = count_hits(blue);

  const int total = r.width * r.height;
  const int valid = red_px + blue_px;

  v.valid_pixel_count = valid;
  v.valid_ratio = (total > 0) ? static_cast<double>(valid) / total : 0.0;
  v.red_ratio   = (valid > 0) ? static_cast<double>(red_px) / valid : 0.0;
  v.blue_ratio  = (valid > 0) ? 1.0 - v.red_ratio : 0.0;
  v.color_margin = std::abs(v.red_ratio - v.blue_ratio);

  char buf[320];

  // ── ① 一个有效像素都没有 → **NONE**（头文件边界①）──
  //    "六个面都贴了贴纸" ⇒ 连一个红/蓝像素都没有，说明这里**没有方块**，
  //    而不是"没看清"。这与"看到了但判不出来"（UNKNOWN）必须是两个值。
  if (valid == 0) {
    v.color = kNone; v.reason = kReasonNone;
    std::snprintf(buf, sizeof(buf), "%s区域内一个红/蓝像素都没有 → NONE（这里没有方块）",
                  pre.c_str());
    v.note = buf;
    return v;
  }

  // ── ② **有效像素闸**（第一道，先于颜色比例）──
  //    ⚠️ 这一条是本模块补的（规格没写），理由见头文件边界②：
  //       24 个红像素 / 900 算出来也是 100% 红，但它**不该**与 600/900 同等可信。
  if (valid < std::max(1, p.min_valid_pixels) || v.valid_ratio < p.min_valid_ratio) {
    v.color = kUnknown; v.reason = kReasonLowValidPixels;
    std::snprintf(buf, sizeof(buf),
                  "%s有效像素 %d 个（占比 %.0f%%）没过闸（需 ≥%d 个 且 ≥%.0f%%）"
                  "→ UNKNOWN：**看到了东西，但太少，不敢说它是方块**。"
                  "动作: 靠近到能看清为止",
                  pre.c_str(), valid, v.valid_ratio * 100.0,
                  p.min_valid_pixels, p.min_valid_ratio * 100.0);
    v.note = buf;
    return v;
  }

  // ── ③ 颜色比例 ──
  const double decisive = std::max(0.5, std::min(p.decisive_ratio, 0.999));

  if (v.red_ratio >= decisive) {
    v.color = kRed; v.reason = kReasonNone;
    std::snprintf(buf, sizeof(buf), "%s红占 %.0f%%（> %.0f%%）→ RED | margin=%.2f 有效像素 %d",
                  pre.c_str(), v.red_ratio * 100.0, decisive * 100.0, v.color_margin, valid);
    v.note = buf;
    return v;
  }
  if (v.blue_ratio >= decisive) {
    v.color = kBlue; v.reason = kReasonNone;
    std::snprintf(buf, sizeof(buf), "%s蓝占 %.0f%%（> %.0f%%）→ BLUE | margin=%.2f 有效像素 %d",
                  pre.c_str(), v.blue_ratio * 100.0, decisive * 100.0, v.color_margin, valid);
    v.note = buf;
    return v;
  }

  // ── ④ 剩下的都是 UNKNOWN，但**原因分开写**（下游采取的动作不同）──
  v.color = kUnknown;
  if (v.red_ratio >= p.close_lo && v.red_ratio <= p.close_hi) {
    v.reason = kReasonRatioAmbiguous;
    std::snprintf(buf, sizeof(buf),
                  "%s红蓝接近（红 %.0f%% / 蓝 %.0f%%，margin=%.2f）→ UNKNOWN："
                  "**需靠近重新观察**（§3.3 原话）—— 别停在这",
                  pre.c_str(), v.red_ratio * 100.0, v.blue_ratio * 100.0, v.color_margin);
  } else {
    v.reason = kReasonRatioAmbiguous;
    std::snprintf(buf, sizeof(buf),
                  "%s红 %.0f%% 未过 %.0f%% 门槛 → UNKNOWN。"
                  "⚠️ 55~70%% 是**规格未定义段**（§3.3 只定义了 >70%% 与 45-55%%），"
                  "本模块取最保守读法：**只有 >70%% 才定色**",
                  pre.c_str(), v.red_ratio * 100.0, decisive * 100.0);
  }
  v.note = buf;
  return v;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 候选区域 —— "色面在哪"的全部几何假设都在这一个函数里
// ═══════════════════════════════════════════════════════════════════════════════
cv::Rect SkyBlockDetector::top_face_roi(const cv::Rect& bbox, double top_face_frac)
{
  if (bbox.width <= 0 || bbox.height <= 0) return cv::Rect();
  // ⚠️ 夹到 (0,1]：给了 0 或负数会让高度变 0 → 后面判成"无交集"，
  //    而症状是"恒判 UNKNOWN"，与"没配 ROI"长得一样 —— 别让它静默。
  const double f = std::max(0.01, std::min(top_face_frac, 1.0));
  const int h = std::max(1, static_cast<int>(bbox.height * f));
  return cv::Rect(bbox.x, bbox.y, bbox.width, h);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 雷达回调 —— 只做"记下来"，判定在相机帧到了才做
// ═══════════════════════════════════════════════════════════════════════════════
void SkyBlockDetector::on_lidar(const br_perception::msg::LidarResult::SharedPtr msg)
{
  if (!msg) return;
  std::lock_guard<std::mutex> lk(state_mutex_);
  spots_        = msg->building_spots;
  spots_header_ = msg->header;      // ⚠️ 坐标系与时间戳**都用雷达那帧的**，不是 now()
  has_spots_    = true;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 建筑位 → 图像 ROI（投影主路）
// ═══════════════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════════════
// 【纯几何】世界 → 图像 ROI（**可单测**：只吃一个 TransformStamped，不碰 TF/ROS）
//
// ⚠️⚠️ 2026-09-24 从 `project_spot` 里**摘出来**的，两个理由：
//    ① **单测**：原来逐点调 `tf2_ros::Buffer::transform()`，测试里没法喂（要起节点、
//       要真实 TF 树）。摘出来之后这个函数**只吃数学**，`test_sky_block_detector.cpp`
//       可以拿手搭的位姿直接验 —— 而"投影整体错 90°"正是最难靠肉眼发现的错。
//    ② **省**：一次 TF 查询代替 7 次（原来每个角点各查一次）。
// ⚠️ 顺带甩掉了 `tf2_geometry_msgs` 依赖（`doTransform<PointStamped>` 的重载在那个包里，
//    而逐点变换的路子已经不需要了 —— 那正是 `mustika_pillar_check` 栽过的坑）。
//
// 坐标系（两处约定都不变，单测会钉住它们）：
//   · 输入 `world_to_cam` 把**世界点**变换到 `camera_frames_[cam]`（**机身系**），
//     然后做 **body2opt**（机身 +X 前/+Y 左/+Z 上 → 光心 +X 右/+Y 下/+Z 前）。
//     ⚠️ 这一步漏了 = 整体错 90°，而**图像上看着还有东西**（只是位置全错）。
//   · 内参描述**原图**；输出 ROI 在 **letterbox 之后**的图里（正向 scale+pad 一次）。
// ═══════════════════════════════════════════════════════════════════════════════
SkyBlockDetector::SpotProjection SkyBlockDetector::project_with_transform(
    const geometry_msgs::msg::TransformStamped& world_to_cam,
    int32_t spot_id,
    const geometry_msgs::msg::Point& spot_position,
    double platform_z,
    double block_size_m, double tower_height_m,
    double fx, double fy, double cx, double cy,
    double scale, int32_t pad_x, int32_t pad_y,
    int img_w, int img_h)
{
  SpotProjection out;
  out.spot_id = spot_id;

  if (img_w <= 0 || img_h <= 0) { out.note = "图像尺寸非法"; return out; }
  if (block_size_m <= 0.0 || tower_height_m <= 0.0) { out.note = "方块/塔高非法"; return out; }

  const double half = block_size_m * 0.5;
  const double z_lo = platform_z + tower_height_m - block_size_m;   // 天空方块底面
  const double z_hi = platform_z + tower_height_m;                  // 天空方块顶面 = 塔顶

  // ── ① 世界点 → 机身系（四元数直接算，与 tf2::doTransform 同式）──
  const auto& t = world_to_cam.transform.translation;
  const auto& q = world_to_cam.transform.rotation;
  const double qw = q.w, qx = q.x, qy = q.y, qz = q.z;   // ⚠️ w 在前（ROS 消息也是）
  const double r00 = 1 - 2*(qy*qy + qz*qz), r01 = 2*(qx*qy - qz*qw), r02 = 2*(qx*qz + qy*qw);
  const double r10 = 2*(qx*qy + qz*qw),     r11 = 1 - 2*(qx*qx + qz*qz), r12 = 2*(qy*qz - qx*qw);
  const double r20 = 2*(qx*qz - qy*qw),     r21 = 2*(qy*qz + qx*qw),     r22 = 1 - 2*(qx*qx + qy*qy);

  //  ⚠️⚠️ **约定：`p_body = R·p_world + t`** —— 就是 `tf2::doTransform` 的算法
  //     （`t` 是"把光心映到机身原点"的那个平移 = `−R·p_cam`，**不是相机在世界系的位置**）。
  //     ⚠️ 写成 `R·(p − t)` 是**另一个变换**（等于把相机位置当原点直接相减）——
  //        2026-09-24 重构时真写错过一次，症状是**所有点都落到光心背后** ⇒ 投影恒失败。
  //        是靠 `test_sky_block_detector.cpp` 的几何用例（"相机在塔顶正上方朝下看，
  //        顶面中心必须落在画面正中"）在**本机用 Python 复现同一个公式**抓到的。
  auto to_cam = [&](double x, double y, double z) {
    const double bx = r00*x + r01*y + r02*z + t.x;
    const double by = r10*x + r11*y + r12*z + t.y;
    const double bz = r20*x + r21*y + r22*z + t.z;
    // ⚠️⚠️ 机身系 → 光心系（**漏了这步 = 整体错 90°，而图像上看着还有东西**）
    geometry_msgs::msg::Point p;
    p.x = -by; p.y = -bz; p.z = bx;
    return p;
  };

  auto project_pt = [&](const geometry_msgs::msg::Point& opt, double& u, double& v) -> bool {
    if (opt.z <= 1e-6) return false;                       // 在身后 / 与光心同面
    u = fx * opt.x / opt.z + cx;
    v = fy * opt.y / opt.z + cy;
    // ⚠️ 内参描述的是**原图**，而 `bgr` 是 letterbox 之后的图 ⇒ **正向**变换一次。
    //    （与检测器那边"反算回原图"方向相反 —— 那边用 letterbox_to_orig_*，别抄错。）
    u = u * scale + static_cast<double>(pad_x);
    v = v * scale + static_cast<double>(pad_y);
    return true;
  };

  // ── ② 顶面四角 + 方块八角 ──
  std::vector<geometry_msgs::msg::Point> top_opt, all_opt;
  top_opt.reserve(4);
  all_opt.reserve(8);
  for (double a : {-half, half}) {
    for (double b : {-half, half}) {
      auto o = to_cam(spot_position.x + a, spot_position.y + b, z_hi);
      top_opt.push_back(o);
      all_opt.push_back(o);
    }
  }
  for (double a : {-half, half}) {
    for (double b : {-half, half}) {
      all_opt.push_back(to_cam(spot_position.x + a, spot_position.y + b, z_lo));
    }
  }

  // ── ③ 掠射角：**顶面中心 → 相机** 的方向 与 **世界竖直** 的夹角 ──
  //    世界竖直在相机系里的表示：把「位中心」与「位中心上方 1m」两点各变换一次、相减。
  //    ⚠️ 这样就不需要知道世界系叫什么名字 —— 全在相机系里算完。
  //
  //    ⚠️⚠️ **方向与参考点两处都容易写反，两处原先就都反了**（2026-09-24 复核抓到）：
  //      · **方向必须是"方块 → 相机"**（= 把 `to_cam` 的结果**取反**）：
  //        `to_cam(P)` 给的是"**相机 → P**"（P 在光心系里的坐标）。不取反的话，
  //        相机在塔顶正上方（**最好的位置**）算出的是 **180°**，而门控
  //        `>= max_grazing_deg` 会把它**拦掉**；反过来相机在顶面下方（顶面根本
  //        看不见）算出的却 <90°、被**放行** —— 整套门控是反的，而症状是
  //        "判出来的颜色不是顶面的颜色"（比判不出来贵得多）。
  //      · **参考点必须是"顶面中心"（`z_hi`）**，不是方块中心 —— 语义是"相机在
  //        **顶面**下方"，而顶面正是本模块的判据本身（方块中心比顶面低半块）。
  //      ⇒ 语义定死: **0° = 相机在顶面正上方，90° = 与顶面同高，>90° = 在顶面下方**。
  {
    const auto c  = to_cam(spot_position.x, spot_position.y, platform_z);
    const auto c1 = to_cam(spot_position.x, spot_position.y, platform_z + 1.0);
    const double ux = c1.x - c.x, uy = c1.y - c.y, uz = c1.z - c.z;

    // 参考点 = **顶面中心**；方向 = 取反 ⇒ "方块 → 相机"
    const auto tc = to_cam(spot_position.x, spot_position.y, z_hi);
    const double vx = -tc.x, vy = -tc.y, vz = -tc.z;

    const double dot = vx*ux + vy*uy + vz*uz;
    const double nv = std::sqrt(vx*vx + vy*vy + vz*vz);
    const double nu = std::sqrt(ux*ux + uy*uy + uz*uz);
    if (nv < 1e-9 || nu < 1e-9) { out.note = "掠射角算不出来（零向量）"; return out; }
    double cosang = dot / (nv * nu);
    cosang = std::max(-1.0, std::min(1.0, cosang));
    out.grazing_deg = std::acos(cosang) * 180.0 / M_PI;   // 0=正上方, 90=平视, >90=在下方
    out.distance_m  = nv;                                 // 到**顶面中心**的距离
  }

  // ── ④ 投影 + 取包围盒 ──
  auto bbox_of = [&](const std::vector<geometry_msgs::msg::Point>& pts, cv::Rect& r,
                     double& w_px, double& h_px, std::string& why) -> bool {
    double u0 = 1e18, u1 = -1e18, v0 = 1e18, v1 = -1e18;
    for (const auto& p : pts) {
      double u, v;
      if (!project_pt(p, u, v)) { why = "有点落在光心后方（Z≤0）—— 相机没朝着它"; return false; }
      u0 = std::min(u0, u); u1 = std::max(u1, u);
      v0 = std::min(v0, v); v1 = std::max(v1, v);
    }
    w_px = u1 - u0;
    h_px = v1 - v0;
    r = cv::Rect(cv::Point(static_cast<int>(std::floor(u0)), static_cast<int>(std::floor(v0))),
                 cv::Point(static_cast<int>(std::ceil(u1)),  static_cast<int>(std::ceil(v1))));
    return true;
  };

  cv::Rect roi_all;
  std::string why;
  if (!bbox_of(all_opt, roi_all, out.face_w_px, out.face_h_px, why)) { out.note = why; return out; }
  // ⚠️ 全方块框目前只用来**先算出一次**（顶面那次会覆盖 `face_*`）；留着它是为了将来
  //    把"整个方块出画"与"顶面出画"分开报（现在两者都由调用方的门控兜底）。
  (void)roi_all;

  cv::Rect roi_top;
  double tw = 0, th = 0;
  if (!bbox_of(top_opt, roi_top, tw, th, why)) { out.note = why; return out; }
  out.face_w_px = tw;
  out.face_h_px = th;

  // ── ⑤ ROI 取**顶面**的投影框（加一点边距，够 Canny 找到边缘）──
  //    ⚠️ 这正是"有投影之后 `top_face_frac` 就不需要了"的地方：
  //       投影直接给出顶面四角，不用再从 bbox 上方按比例猜。
  const int pad = std::max(2, static_cast<int>(0.05 * std::max(tw, th)));
  out.roi_lb = cv::Rect(roi_top.x - pad, roi_top.y - pad,
                        roi_top.width + 2 * pad, roi_top.height + 2 * pad)
               & cv::Rect(0, 0, img_w, img_h);

  if (out.roi_lb.width <= 0 || out.roi_lb.height <= 0) {
    // 完全出画 —— 与"取不到位置"是两回事，note 里分开写
    out.note = "投影框与画面无交集（方块出画）";
    return out;
  }
  out.valid = true;
  return out;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 【薄壳】查一次 TF → 交给纯几何（`project_with_transform`）
// ═══════════════════════════════════════════════════════════════════════════════
SkyBlockDetector::SpotProjection SkyBlockDetector::project_spot(
    int32_t spot_id,
    const geometry_msgs::msg::Point& spot_position,
    double platform_z,
    uint8_t cam,
    const std_msgs::msg::Header& spot_header,
    const builtin_interfaces::msg::Time& query_stamp,
    double scale, int32_t pad_x, int32_t pad_y,
    int img_w, int img_h)
{
  SpotProjection out;
  out.spot_id = spot_id;

  if (cam >= kCamCount) { out.note = "相机下标非法"; return out; }
  if (spot_header.frame_id.empty()) {
    // ⚠️ 无从变换就**别猜**（同 `cloud_to_world` 那条）：frame_id 为空既可能"已在场系"
    //    也可能"根本没标"，按前者处理就是拿整条链去赌。
    out.note = "建筑位消息的 frame_id 为空 —— 无从投影（上游应继承输入点云的 frame_id）";
    return out;
  }
  if (!tf_buffer_) { out.note = "TF 缓冲区没建起来"; return out; }

  // ── TF：**一次**查询拿到 世界 → 相机机身系（⚠️ 时刻用图像那一帧的，见头文件）──
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    // ⚠️⚠️ `rclcpp::Time` 必须**带上本节点的时钟类型**（`get_clock_type()`）：
    //    仿真里节点用 `use_sim_time`，而这个 stamp 也是仿真时间 —— 默认构造的
    //    `rclcpp::Time(msg)` 是 **RCL_SYSTEM_TIME** ⇒ 与缓冲区时基不同源 ⇒
    //    **查 TF 恒失败**（症状：投影全失败，而 TF 树上明明有那条变换）。
    //    （同款写法见 `perception_pipeline.cpp` 的 `on_camera_frame`。）
    tf_msg = tf_buffer_->lookupTransform(camera_frames_[cam], spot_header.frame_id,
                                        rclcpp::Time(query_stamp, this->get_clock()->get_clock_type()),
                                        tf2::durationFromSec(tf_timeout_sec_));
  } catch (const tf2::TransformException& ex) {
    out.note = std::string("TF ") + spot_header.frame_id + " → " + camera_frames_[cam] +
               std::string(" 失败: ") + ex.what();
    return out;
  }

  return project_with_transform(tf_msg, spot_id, spot_position, platform_z,
                                block_size_m_, tower_height_m_,
                                fx_[cam], fy_[cam], cx_[cam], cy_[cam],
                                scale, pad_x, pad_y, img_w, img_h);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 构造
// ═══════════════════════════════════════════════════════════════════════════════
SkyBlockDetector::SkyBlockDetector(const rclcpp::NodeOptions& options)
  : Node("sky_block_detector", options)
{
  // ── 颜色阈值文件路径 ──
  // ⚠️ `camera_params.yaml` 里写的是**相对路径**，而 `color_utils.cpp` 是
  //    `std::ifstream(yaml_path)` —— 相对的是**启动时的 $PWD**，节点里没有回退解析。
  //    ⇒ launch **必须**用绝对路径覆盖它（见 launch/bringup_full.launch.py 的 color_yaml）。
  this->declare_parameter<std::string>("color_yaml_path", "config/color_thresholds.yaml");

  // ── 判色阈值 ──
  this->declare_parameter<double>("decisive_ratio",   color_params_.decisive_ratio);
  this->declare_parameter<double>("close_lo",         color_params_.close_lo);
  this->declare_parameter<double>("close_hi",         color_params_.close_hi);
  this->declare_parameter<int>("min_valid_pixels",    color_params_.min_valid_pixels);
  this->declare_parameter<double>("min_valid_ratio",  color_params_.min_valid_ratio);

  // ── 几何 ──
  this->declare_parameter<double>("top_face_frac",  top_face_frac_);
  this->declare_parameter<std::vector<double>>("debug_roi_ratio", debug_roi_ratio_);
  this->declare_parameter<int>("roi_min_area", roi_min_area_);

  // ── 时序确认 ──
  this->declare_parameter<int>("enter_confirm_frames", enter_confirm_frames_);
  this->declare_parameter<int>("exit_confirm_frames",  exit_confirm_frames_);

  // ── 投影主路（雷达建筑位 → 图像 ROI）──
  this->declare_parameter<std::string>("lidar_topic", lidar_topic_);
  // ⚠️ 这三个键与 `lidar_perception_node` 读的**同名同源**（field_geometry.yaml 兼容层）。
  //    长度必须都是 10 且一致，否则投影主路整组不可用（构造里会 WARN 点名）。
  this->declare_parameter<std::vector<double>>("building_spot_x", spot_x_);
  this->declare_parameter<std::vector<double>>("building_spot_y", spot_y_);
  this->declare_parameter<std::vector<double>>("building_spot_z", spot_z_);
  this->declare_parameter<double>("tower_height_m", tower_height_m_);
  this->declare_parameter<double>("block_size_m",   block_size_m_);
  this->declare_parameter<std::vector<double>>("fx", {fx_[0], fx_[1]});
  this->declare_parameter<std::vector<double>>("fy", {fy_[0], fy_[1]});
  this->declare_parameter<std::vector<double>>("cx", {cx_[0], cx_[1]});
  this->declare_parameter<std::vector<double>>("cy", {cy_[0], cy_[1]});
  this->declare_parameter<int>("orig_w", orig_w_);
  this->declare_parameter<int>("orig_h", orig_h_);
  this->declare_parameter<std::vector<std::string>>(
      "camera_frames", {camera_frames_[0], camera_frames_[1]});
  this->declare_parameter<double>("min_face_h_px",    min_face_h_px_);
  this->declare_parameter<double>("max_grazing_deg",  max_grazing_deg_);
  this->declare_parameter<double>("tf_timeout_sec",   tf_timeout_sec_);

  this->get_parameter("color_yaml_path", color_yaml_path_);
  this->get_parameter("decisive_ratio",   color_params_.decisive_ratio);
  this->get_parameter("close_lo",         color_params_.close_lo);
  this->get_parameter("close_hi",         color_params_.close_hi);
  this->get_parameter("min_valid_pixels", color_params_.min_valid_pixels);
  this->get_parameter("min_valid_ratio",  color_params_.min_valid_ratio);
  this->get_parameter("top_face_frac",    top_face_frac_);
  this->get_parameter("debug_roi_ratio",  debug_roi_ratio_);
  this->get_parameter("roi_min_area",     roi_min_area_);
  this->get_parameter("enter_confirm_frames", enter_confirm_frames_);
  this->get_parameter("exit_confirm_frames",  exit_confirm_frames_);

  // ── 投影主路 ──
  this->get_parameter("lidar_topic",       lidar_topic_);
  this->get_parameter("building_spot_x",   spot_x_);
  this->get_parameter("building_spot_y",   spot_y_);
  this->get_parameter("building_spot_z",   spot_z_);
  this->get_parameter("tower_height_m",    tower_height_m_);
  this->get_parameter("block_size_m",      block_size_m_);
  this->get_parameter("orig_w",            orig_w_);
  this->get_parameter("orig_h",            orig_h_);
  this->get_parameter("min_face_h_px",     min_face_h_px_);
  this->get_parameter("max_grazing_deg",   max_grazing_deg_);
  this->get_parameter("tf_timeout_sec",    tf_timeout_sec_);
  // ⚠️ 定长数组**不能**直接 `get_parameter` 进 `std::array` —— `rclcpp` 的
  //    `ParameterValue` 只对**标量**与 `std::vector` 开 `get<T>()` 重载，
  //    `get<std::array<T,N>>()` 一律走 `enable_if<false>` ⇒ **编译期硬错**
  //    （2026-09-25 首次全量编译实测，报错点在 rclcpp 头里、看着像库的问题，
  //     实际是本行）。⇒ 一律**读进 vector → 校长度 → `std::copy` 进定长数组**；
  //    长度不符就**整组忽略、沿用默认**（不猜，与下面 fx/fy/cx/cy 同一套口径）。
  {
    std::vector<double> v;
    auto arr = [&](const char* n, std::array<double, kCamCount>& dst) {
      this->get_parameter(n, v);
      if (v.size() == kCamCount) { std::copy(v.begin(), v.end(), dst.begin()); }
      else {
        RCLCPP_WARN(this->get_logger(),
          "参数 %s 长度 %zu ≠ 相机路数 %zu → **整组忽略**，沿用默认（不猜）",
          n, v.size(), kCamCount);
      }
    };
    arr("fx", fx_); arr("fy", fy_); arr("cx", cx_); arr("cy", cy_);
  }
  {
    std::vector<std::string> frames;
    this->get_parameter("camera_frames", frames);
    if (frames.size() == kCamCount) {
      std::copy(frames.begin(), frames.end(), camera_frames_.begin());
    } else {
      RCLCPP_WARN(this->get_logger(),
        "参数 camera_frames 长度 %zu ≠ 相机路数 %zu → **整组忽略**，沿用默认（不猜）；"
        "对不上 TF 树的话投影会**恒失败且不报错**",
        frames.size(), kCamCount);
    }
  }

  // ── 规范化 ──
  color_params_.decisive_ratio = std::max(0.5, std::min(color_params_.decisive_ratio, 0.999));
  color_params_.close_lo = std::max(0.0, std::min(color_params_.close_lo, 0.5));
  color_params_.close_hi = std::max(0.5, std::min(color_params_.close_hi, 1.0));
  if (color_params_.close_hi < color_params_.close_lo) {
    std::swap(color_params_.close_hi, color_params_.close_lo);
    RCLCPP_WARN(this->get_logger(),
      "close_lo/close_hi 反了（%.2f / %.2f）—— 已交换。去查 yaml",
      color_params_.close_lo, color_params_.close_hi);
  }
  color_params_.min_valid_pixels = std::max(1, color_params_.min_valid_pixels);
  color_params_.min_valid_ratio  = std::max(0.0, std::min(color_params_.min_valid_ratio, 1.0));
  top_face_frac_ = std::max(0.01, std::min(top_face_frac_, 1.0));
  roi_min_area_  = std::max(1, roi_min_area_);
  // ⚠️ 退出帧数**必须 ≥ 进入帧数**：反了的话"刚确认就马上掉出去"，
  //    滞回窗口形同虚设（规则 3.5.8 对手会翻，那种抖动最贵）。
  enter_confirm_frames_ = std::max(1, enter_confirm_frames_);
  exit_confirm_frames_  = std::max(enter_confirm_frames_, exit_confirm_frames_);
  for (auto& cam_slots : confirmer_) {
    for (auto& c : cam_slots) {       // ⚠️ **每个槽位**都要配上（每座塔一套滞回）
      c.enter_frames = enter_confirm_frames_;
      c.exit_frames  = exit_confirm_frames_;
      c.reset();
    }
  }

  // ── 投影主路的规范化 + **可用性校验** ──
  tower_height_m_ = std::max(1e-3, tower_height_m_);
  block_size_m_   = std::max(1e-3, block_size_m_);
  orig_w_ = std::max(1, orig_w_);
  orig_h_ = std::max(1, orig_h_);
  min_face_h_px_  = std::max(0.0, min_face_h_px_);
  // ⚠️ 上限夹到 90° 并**留一点** —— `≥90°` 意味着相机在顶面下方，顶面物理上看不见。
  //    夹到 89.9 是为了让"正好 90"这种边界值也走 `kReasonGrazingAngle` 而不是漏过。
  max_grazing_deg_ = std::max(1.0, std::min(max_grazing_deg_, 89.9));
  tf_timeout_sec_  = std::max(0.001, std::min(tf_timeout_sec_, 1.0));

  // ⚠️⚠️ 三个建筑位数组**必须同长**，否则投影主路整组不可用 —— 而且**要说出来**：
  //     静默留空的话，节点照常起、照常发消息、恒判 NONE，
  //     而 NONE 的意思是"这里没有方块"⇒ 融合层以为塔是空的。
  const size_t n = std::min({spot_x_.size(), spot_y_.size(), spot_z_.size()});
  if (n == 0 || spot_x_.size() != spot_y_.size() || spot_x_.size() != spot_z_.size()) {
    RCLCPP_WARN(this->get_logger(),
      "**建筑位数组不可用** —— building_spot_x/y/z 长度 %zu/%zu/%zu（需三者相等且 >0）。"
      "⇒ **投影主路关闭**，只剩手动 ROI。"
      "⚠️ 常见原因: launch 没把 `field_geometry.yaml` 传给本节点",
      spot_x_.size(), spot_y_.size(), spot_z_.size());
    spot_x_.clear(); spot_y_.clear(); spot_z_.clear();
  } else if (n > spot_x_.size() || n != spot_x_.size()) {
    // 不会到这（上面已判等），留个位置说明"这里刻意不截断"
  }
  if (!spot_x_.empty()) {
    RCLCPP_INFO(this->get_logger(),
      "投影主路已启用: %zu 个建筑位 | 塔高 %.2fm 方块 %.2fm | 内参 fx=[%.0f, %.0f] | "
      "掠射门控: 顶面高 ≥%.1fpx 且 夹角 ≤%.1f°",
      spot_x_.size(), tower_height_m_, block_size_m_, fx_[0], fx_[1],
      min_face_h_px_, max_grazing_deg_);
  }

  // ── 加载 HSV 阈值 ──
  // ⚠️ **加载失败要让它响**（抛）。`load_color_thresholds` 在文件打不开/格式错时
  //    抛 `cv::Exception` —— 这里**不接**，让它冒上去变成节点启动失败。
  //    理由：静默回落成"空阈值组"的话，节点会**正常跑、每帧都判 NONE**，
  //    而 NONE 的语义是"这里没有方块" ⇒ 融合层以为塔是空的。
  //    这正是本仓库最恨的"看着配好了，实际没接上"。
  {
    const auto all = vision::load_color_thresholds(color_yaml_path_);
    auto need = [&](const char* key) -> vision::ColorThreshold {
      const auto it = all.find(key);
      if (it == all.end()) {
        throw std::runtime_error(
            std::string("color_thresholds.yaml 里没有 '") + key +
            "' 这一节 —— 判色无法进行。文件: " + color_yaml_path_);
      }
      return it->second;
    };
    red_  = need("red_team");
    blue_ = need("blue_team");
  }

  // ── TF：**只读**（不广播 —— 理由见头文件 `tf_buffer_` 那段）──
  //    投影要把建筑位从雷达那个系转到相机系，走的就是这棵 TF 树。
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── 订阅 ──
  // 雷达的建筑位（投影主路的唯一输入）。⚠️ 与 `mustika_pillar_check` 同口径用
  // RELIABLE：发布端是 `lidar_perception_node` 的默认 QoS(10) = RELIABLE。
  sub_lidar_ = this->create_subscription<br_perception::msg::LidarResult>(
      lidar_topic_, rclcpp::QoS(10),
      std::bind(&SkyBlockDetector::on_lidar, this, std::placeholders::_1));

  // ⚠️ 与三个已有检测器**同口径**：BEST_EFFORT + 队列深度 1（任务书 §3.0 硬约束第 2 条）。
  //    发布端是 `image_preprocessor`，也是 depth=1 best_effort。
  //    **两侧必须同为 best_effort** —— RELIABLE 订阅接 BEST_EFFORT 发布 = 静默收不到。
  const auto stream_qos = rclcpp::QoS(1).best_effort();

  sub_wide_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
      "/perception/camera_wide/preprocessed", stream_qos,
      std::bind(&SkyBlockDetector::wide_callback, this, std::placeholders::_1));
  sub_tele_ = this->create_subscription<br_perception::msg::PreprocessedImage>(
      "/perception/camera_tele/preprocessed", stream_qos,
      std::bind(&SkyBlockDetector::tele_callback, this, std::placeholders::_1));

  // ── 发布 ──
  // ⚠️ 话题名带 `_wide` / `_tele` 后缀 —— 与 `mustika_wide` / `pillars_wide` /
  //    `building_spots_wide` 同体例。
  pub_wide_ = this->create_publisher<br_perception::msg::CameraSkyBlock>(
      "/perception/camera/sky_blocks_wide", stream_qos);
  pub_tele_ = this->create_publisher<br_perception::msg::CameraSkyBlock>(
      "/perception/camera/sky_blocks_tele", stream_qos);
  pub_diagnostics_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/perception/camera/sky_blocks_diagnostics", 10);

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SkyBlockDetector::on_parameter_change, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "SkyBlockDetector ready | 阈值=%s (红 %zu 组 / 蓝 %zu 组) | "
    "判据: 红/蓝占比>%.0f%% 定色 · 有效像素≥%d 且≥%.0f%% · 45~55%% 判未知 | "
    "时序: 连续 %d 帧确认 / %d 帧掉出（**按建筑位各自一套**）| top_face_frac=%.2f（仅手动路）",
    color_yaml_path_.c_str(), red_.groups.size(), blue_.groups.size(),
    color_params_.decisive_ratio * 100.0, color_params_.min_valid_pixels,
    color_params_.min_valid_ratio * 100.0,
    enter_confirm_frames_, exit_confirm_frames_, top_face_frac_);

  // ═══════════════════════════════════════════════════════════════════════════
  // ⚠️⚠️ **把"能不能产出"说在最显眼的地方**
  //
  //   2026-09-24 起 ROI 主路 = **雷达建筑位投影**（`build_requests`），
  //   而它**依赖三样外部东西**，缺任何一样都只会"安静地什么都不出"：
  //     ① 雷达的建筑位（`building_spot_x/y/z` 参数 —— 由 launch 传 `field_geometry.yaml`）
  //     ② TF（`world→camera_*`：需要里程计 + `perception_pipeline` 在广播）
  //     ③ **雷达得先量出"完整塔"**（`BuildingSpot.state == 3`）—— 没有塔就没得判
  //   而这个模块**只做判色、不做检测** ⇒ 它没有"自己去找方块"的能力，
  //   所以"要不要判"这件事**只能由雷达说**。
  // ═══════════════════════════════════════════════════════════════════════════
  if (spot_x_.empty()) {
    RCLCPP_WARN(this->get_logger(),
      "**建筑位数组为空 ⇒ 投影主路关闭**（只剩台架手动 ROI）。"
      "⚠️ 常见原因: launch 没把 `field_geometry.yaml` 传给本节点（三个数组在它的第 11 节）。"
      "那时本节点**照常起、照常发消息**，但每帧的 ROI 来源只有 `debug_roi_ratio` —— "
      "不配就是一条恒 UNKNOWN 的链（**症状与「场上真没塔」长得一样**）。");
  }
  if (debug_roi_ratio_.empty()) {
    RCLCPP_INFO(this->get_logger(),
      "手动 ROI 未配置（**这是比赛模式的正常状态**）—— 判色完全由投影主路决定。"
      "台架调阈值时才用 `debug_roi_ratio: [x, y, w, h]`（相对宽高的 0~1）指一块区域。");
  } else if (debug_roi_ratio_.size() != 4) {
    RCLCPP_ERROR(this->get_logger(),
      "debug_roi_ratio 长度是 %zu，**必须是 4**（x, y, w, h 相对比例）→ 整组忽略",
      debug_roi_ratio_.size());
    debug_roi_ratio_.clear();
  } else {
    RCLCPP_WARN(this->get_logger(),
      "**手动 ROI 已启用**（debug_roi_ratio=%zu 个数）—— 它只在**投影主路完全没产出**时兜底。"
      "⚠️ 比赛模式该把它留空：那时若雷达说「这儿有塔」、而手动框指的是别处，"
      "两边的结论会打架。",
      debug_roi_ratio_.size());
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 回调
// ═══════════════════════════════════════════════════════════════════════════════
void SkyBlockDetector::wide_callback(
    const br_perception::msg::PreprocessedImage::SharedPtr msg)
{
  process_and_publish(msg, 0, pub_wide_);
}

void SkyBlockDetector::tele_callback(
    const br_perception::msg::PreprocessedImage::SharedPtr msg)
{
  process_and_publish(msg, 1, pub_tele_);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 组 ROI 清单：**投影主路优先，手动兜底**
// ═══════════════════════════════════════════════════════════════════════════════
void SkyBlockDetector::build_requests(const br_perception::msg::PreprocessedImage& msg,
                                      uint8_t camera_id,
                                      const cv::Mat& bgr,
                                      std::vector<RoiRequest>& out)
{
  out.clear();
  if (bgr.cols <= 0 || bgr.rows <= 0) return;

  // ⚠️ 只在"有方块要判"时投影。`BuildingSpot.msg` 的 state 逐值：
  //    0=EMPTY 1=ONE_EARTH 2=TWO_EARTH 3=FULL_TOWER(完整塔) 4=UNKNOWN(没量成)
  //    · 完整塔 = 地球×2 + **天空×1**（`field_geometry.yaml: complete_tower.height 0.90`）
  //      —— 只有它的**顶层是天空方块**，判它的顶面颜色才有意义（40/80 分就看它）。
  //      ⚠️ 雷达那边对完整塔置的 `needs_visual_check` 就是同一个判据（它自己说"要看视觉"）。
  //    · `4 = UNKNOWN` **绝不当成"检查一下"** —— 那是"没量成"，投影没有依据。
  constexpr uint8_t kSpotStateCompleteTower = 3;

  // ── ① 快照雷达结果（**持锁取完就放** —— TF 查询不能在锁里做，见 `project_spot` 的注释）──
  std::vector<br_perception::msg::BuildingSpot> spots;
  std_msgs::msg::Header spots_hdr;
  bool have_spots = false;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    spots      = spots_;
    spots_hdr  = spots_header_;
    have_spots = has_spots_;
  }

  // ── ② 雷达结果不能太旧 ──
  //    ⚠️ 用**雷达那帧的时间戳**与**本图像的曝光时刻**比，不是用"收没收到"。
  //       太旧的位状态 = 拿"上一秒哪儿有塔"去给这一帧定 ROI ⇒ 塔可能已经被**翻转**
  //       （规则 3.5.8 对手会翻）或搬走了，而判出来的颜色会**挂到错的那座塔上**。
  //    ⚠️ 两个时间戳都为 0 时（仿真里有些源不发时间戳）跳过这项检查，**不猜**。
  const double kMaxLidarAgeSec = 0.5;
  bool radar_usable = have_spots && !spots.empty();
  if (radar_usable && msg.header.stamp.sec != 0 && spots_hdr.stamp.sec != 0) {
    const double age = (rclcpp::Time(msg.header.stamp) - rclcpp::Time(spots_hdr.stamp)).seconds();
    if (age > kMaxLidarAgeSec) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "雷达建筑位结果**太旧**（%.2fs > %.2fs）⇒ 这一帧不用投影主路。"
        "⚠️ 不是「没有塔」，是「不知道现在有没有塔」 —— 别拿旧状态去定 ROI",
        age, kMaxLidarAgeSec);
      radar_usable = false;
    }
  }

  if (radar_usable) {
    for (const auto& sp : spots) {
      if (sp.state != kSpotStateCompleteTower) continue;   // 只判"完整塔"的顶面

      RoiRequest rq;
      rq.source  = kRadarProjected;
      rq.spot_id = static_cast<int32_t>(sp.spot_id);

      // ⚠️ `BuildingSpot.position.z` = **该位的平台高**（雷达那边
      //    `m.position.z = s.platform_z`，L1=0.6 / L2=0.9），不是塔顶高度。
      //    塔顶 = 平台高 + 0.90（完整塔高）—— 那一步在 `project_spot` 里做。
      // ⚠️ 两个 header 各管一件事，别混：
      //    `spots_hdr`    → 位坐标**在哪个系**（frame_id）
      //    `msg.header.stamp` → **TF 查哪一时刻**（图像的曝光时刻 —— 相机位姿是那个时刻的）
      const SpotProjection proj = project_spot(
          rq.spot_id, sp.position, sp.position.z, camera_id,
          spots_hdr, msg.header.stamp, msg.scale, msg.pad_x, msg.pad_y,
          bgr.cols, bgr.rows);

      if (!proj.valid) {
        // ⚠️ **投影失败 ≠ 这里没有塔**。要发一条 UNKNOWN 出去（带原因），
        //    否则下游看到的是"关于这座塔什么都没有"，而不是"没看成"。
        rq.have_roi = false;
        // ⚠️ 用 `PROJECTION_FAILED`（**不是** `OCCLUDED`）：前者是"投影算不出来，
        //    靠近也没用，去查 TF/标定"，后者是"塔在画外，靠近就能看见"。
        //    （2026-09-24 之前这里拿 `OCCLUDED` 当替身 —— 那会把定位故障
        //     误报成"换个视角试试"。）
        rq.reason   = kReasonProjectionFailed;
        rq.note     = "**投影失败**（建筑位 " + std::to_string(rq.spot_id) + "）: " + proj.note +
                      " ⚠️ 这不是「塔不在这儿」，是「没法把塔投到画面上」";
        out.push_back(rq);
        continue;
      }

      // ── 掠射门控（**它为什么长在这里**见头文件 `project_spot` 那段）──
      if (proj.grazing_deg >= max_grazing_deg_ || proj.face_h_px < min_face_h_px_) {
        rq.have_roi         = false;
        rq.grazing_blocked  = true;
        rq.reason           = kReasonGrazingAngle;
        char buf[224];
        std::snprintf(buf, sizeof(buf),
          "**掠射角过大**（夹角 %.1f° 上限 %.1f° / 顶面投影高 %.1fpx 下限 %.1fpx）"
          "⇒ 顶面被压成窄条，判色会判到**侧面**上去 —— 这一帧的颜色**不可用**，"
          "动作: 换位姿/靠近",
          proj.grazing_deg, max_grazing_deg_, proj.face_h_px, min_face_h_px_);
        rq.note = buf;
        out.push_back(rq);
        continue;
      }

      // ── ROI 太小：判了也是拿几十个像素赌（判据与手动路同源: `roi_min_area_`）──
      if (proj.roi_lb.area() < roi_min_area_) {
        rq.have_roi = false;
        rq.reason   = kReasonRoiTooSmall;
        rq.note     = "投影出的顶面 ROI 太小（" + std::to_string(proj.roi_lb.area()) +
                      " px² < roi_min_area " + std::to_string(roi_min_area_) + "）—— 没算";
        out.push_back(rq);
        continue;
      }

      // ── 到这里才是真判 ──
      //    ⚠️ **不套 `top_face_roi`**：投影给出的就是**顶面四角**围成的框，
      //       再按比例切一刀反而会切掉顶面、留下侧面（那正是头文件里
      //       "红蓝怎么分布"那条未决几何的落点 —— 投影路**绕开了它**）。
      rq.roi_lb   = proj.roi_lb;
      rq.have_roi = true;
      char buf[160];
      std::snprintf(buf, sizeof(buf),
        "投影: %d×%d px @ %.2fm（顶面 %dx%d, 夹角 %.1f°）",
        proj.roi_lb.width, proj.roi_lb.height, proj.distance_m,
        static_cast<int>(proj.face_w_px), static_cast<int>(proj.face_h_px), proj.grazing_deg);
      rq.note = buf;
      out.push_back(rq);
    }
  }

  // ⚠️ 主路**产出过任何一条**（含"投影失败"/"掠射被拦"）就不再叠手动 ——
  //    否则同一帧里"雷达说这儿有塔"与"调试参数指的那块"一起发出去，下游分不清信谁。
  if (!out.empty()) return;

  // ── ③ 手动兜底（台架调阈值用；比赛模式该留空，见构造函数那条 WARN）──
  RoiRequest rq;
  rq.source  = static_cast<uint8_t>(kManual);
  rq.spot_id = -1;
  if (debug_roi_ratio_.size() == 4) {
    const int x = static_cast<int>(std::lround(debug_roi_ratio_[0] * bgr.cols));
    const int y = static_cast<int>(std::lround(debug_roi_ratio_[1] * bgr.rows));
    const int w = static_cast<int>(std::lround(debug_roi_ratio_[2] * bgr.cols));
    const int h = static_cast<int>(std::lround(debug_roi_ratio_[3] * bgr.rows));

    // ⚠️ 手动路**保留** `top_face_roi`（人工框的是整个方块，得自己收成上半区域）。
    //    这一步就是头文件里那条**未决几何**（红蓝怎么分布）的落点；投影路不经过它。
    const cv::Rect roi = top_face_roi(cv::Rect(x, y, w, h), top_face_frac_);
    if (roi.width * roi.height >= roi_min_area_) {
      rq.roi_lb   = roi;
      rq.have_roi = true;
      rq.note     = "手动 ROI（台架模式）";
    } else {
      rq.reason = kReasonRoiTooSmall;
      rq.note   = "ROI 太小（" + std::to_string(roi.width * roi.height) +
                  " px² < roi_min_area " + std::to_string(roi_min_area_) + "）—— 没算";
    }
  } else {
    // ⚠️ **这一帧什么都没看**（连"要看的对象"都没有）—— 与"投影失败/掠射/ROI 太小"
    //    那几条**分号**：那几条雷达确实报了完整塔（"有方块但没看清"），这条**没有塔**。
    rq.no_source = true;
    rq.reason = kReasonRoiTooSmall;
    rq.note   = "**没有 ROI 来源** —— 雷达没给「有完整塔」的位（或结果太旧），"
                "且 debug_roi_ratio 没配。⚠️ 这不是「没有方块」，是「没去看」";
  }
  out.push_back(rq);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 一帧的处理：定 ROI → 判色 → 时序确认 → 发消息
// ⚠️ **一帧可以发多条**（每个"有完整塔"的建筑位一条），见 `RoiRequest`
// ═══════════════════════════════════════════════════════════════════════════════
void SkyBlockDetector::process_and_publish(
    const br_perception::msg::PreprocessedImage::SharedPtr msg,
    uint8_t camera_id,
    const rclcpp::Publisher<br_perception::msg::CameraSkyBlock>::SharedPtr& pub)
{
  const auto t0 = mono_now();

  // ── 空消息：发一条"什么都没看见"（保留旧行为）──
  if (!msg) {
    br_perception::msg::CameraSkyBlock out;
    out.camera_id  = camera_id;
    out.roi_source = roi_source_;
    out.spot_id    = spot_id_;
    out.header.stamp = this->get_clock()->now();
    out.detected  = false;
    out.top_color = kNone;
    out.state = TemporalConfirmer::kUnconfirmed;
    pub->publish(out);
    return;
  }

  // ── 取图（零拷贝：msg->image 是内嵌字段，用带 tracked_object 的重载）──
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg->image, msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "cv_bridge 转换失败（cam%d）: %s —— ⚠️ 常见原因: 上游把编码改成了 float32",
      static_cast<int>(camera_id), e.what());
    br_perception::msg::CameraSkyBlock out;
    out.camera_id  = camera_id;
    out.roi_source = roi_source_;
    out.spot_id    = spot_id_;
    out.header     = msg->header;
    out.detected   = false;
    out.top_color  = kNone;
    out.state = TemporalConfirmer::kUnconfirmed;
    pub->publish(out);
    return;
  }

  const cv::Mat& bgr = cv_ptr->image;

  // ── ① 组 ROI 清单（投影主路 / 手动兜底）──
  std::vector<RoiRequest> reqs;
  build_requests(*msg, camera_id, bgr, reqs);

  // ── ② 逐条判色 + 发布 ──
  for (const auto& rq : reqs) {
    br_perception::msg::CameraSkyBlock out;
    out.camera_id  = camera_id;
    out.roi_source = rq.source;
    out.spot_id    = rq.spot_id;
    // ⚠️ **原样继承曝光时刻**，不是 now()。任务书 §四「时间戳约定」：
    //    取发出时刻会多算 ≈43ms 的位移，污染下游的外推。
    out.header = msg->header;

    ColorVerdict v;
    if (rq.have_roi) {
      v = judge_top_color(bgr, rq.roi_lb, red_, blue_, color_params_);
    } else {
      // ⚠️ "没 ROI"有**两种**，结论相反（2026-09-25 按 `no_source` 分开）：
      //    · 雷达**报了完整塔**、只是投影失败/掠射被拦/ROI 太小 ⇒ **`kUnknown`**
      //      （"有方块但判不出来"）。这里报 `kNone` = 让整座塔凭空消失（见头文件边界①）。
      //    · **雷达没报**（没塔 / 结果太旧）且没配手动 ROI ⇒ 这一帧**什么都没看**
      //      ⇒ 老老实实 `kNone`（`detected=false`）。"没去看"这件事装不进
      //      `detected` 这个二值字段，由 `unknown_reason` 与 `note` 说清。
      v.color  = rq.no_source ? kNone : kUnknown;
      v.reason = rq.reason;
      v.note   = rq.note;
    }
    if (rq.grazing_blocked) {
      // 掠射门控：**不去判一个必错的颜色**（顶面被压成窄条时会判到侧面上）
      v.color  = kUnknown;
      v.reason = kReasonGrazingAngle;
      v.note   = rq.note;
    } else if (!rq.note.empty() && rq.have_roi && v.note.empty()) {
      v.note = rq.note;      // 投影路附上几何量，方便现场对账
    }

    // ── 时序确认（**按位**：每座塔各自一套滞回）──
    // ⚠️ 喂给状态机的是"**这一帧判出了红或蓝**"（satisfied），不是"判色成功"：
    //    UNKNOWN 与 NONE 都算**不满足** —— 它们都不是可以拿去改塔归属的结论。
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      auto& c = confirmer_[camera_id < kCamCount ? camera_id : 0][confirmer_slot(rq.spot_id)];
      out.state = static_cast<uint8_t>(c.push(v.color == kRed || v.color == kBlue));
    }

    // ⚠️ `detected` 与 `top_color` 的关系（头文件边界①）：
    //    `NONE` ⇒ `detected=false`（这里没有方块）
    //    其余三档 ⇒ `detected=true` —— **`UNKNOWN` 也是"检测到了"**，
    //    只是颜色判不出来。把 UNKNOWN 也算成"没检测到"会让整座塔凭空消失。
    out.detected  = (v.color != kNone);
    out.top_color = static_cast<uint8_t>(v.color);
    out.unknown_reason = static_cast<uint8_t>(v.reason);

    out.red_ratio        = static_cast<float>(v.red_ratio);
    out.blue_ratio       = static_cast<float>(v.blue_ratio);
    out.color_margin     = static_cast<float>(v.color_margin);
    out.valid_ratio      = static_cast<float>(v.valid_ratio);
    out.valid_pixel_count = static_cast<uint32_t>(std::max(0, v.valid_pixel_count));

    // 置信度：**只反映"这次判色有多可信"，不是"这里有方块的概率"**。
    // ⚠️ 本模块不做检测，所以这个数只能这么定；定义写在这里，别当下游的检测概率用。
    //    判出颜色时 = 该颜色的占比；其余一律 0（结论不可信就该是 0）。
    out.confidence = (v.color == kRed)  ? static_cast<float>(v.red_ratio)
                   : (v.color == kBlue) ? static_cast<float>(v.blue_ratio)
                                        : 0.0f;

    if (rq.have_roi) {
      // ── 坐标反算回**原图** ──
      // ⚠️ 位置类与尺度类**分开调**（`letterbox_to_orig_xy` vs `letterbox_to_orig_size`）：
      //    角点要**减**填充偏移，宽高**不能减** —— 减了会出负宽度。
      //    这是整套反算最易错的一处（P0-3 当时踩过）。
      double x1 = rq.roi_lb.x, y1 = rq.roi_lb.y;
      double x2 = rq.roi_lb.x + rq.roi_lb.width, y2 = rq.roi_lb.y + rq.roi_lb.height;
      letterbox_to_orig_xy(msg->scale, msg->pad_x, msg->pad_y, x1, y1);
      letterbox_to_orig_xy(msg->scale, msg->pad_x, msg->pad_y, x2, y2);

      out.target_u  = static_cast<float>((x1 + x2) * 0.5);
      out.target_v  = static_cast<float>((y1 + y2) * 0.5);
      out.roi_width  = static_cast<float>(x2 - x1);
      out.roi_height = static_cast<float>(y2 - y1);
    } else {
      out.target_u = out.target_v = out.roi_width = out.roi_height = 0.0f;
    }

    pub->publish(out);

    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      (camera_id == 0 ? wide_count_ : tele_count_)++;
      (camera_id == 0 ? wide_frame_seq_ : tele_frame_seq_)++;
    }

    publish_diagnostics(camera_id, v, out, ms_since(t0));
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 诊断
// ═══════════════════════════════════════════════════════════════════════════════
void SkyBlockDetector::publish_diagnostics(
    uint8_t camera_id, const ColorVerdict& v,
    const br_perception::msg::CameraSkyBlock& out, double total_ms)
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = this->get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name        = std::string("sky_block_detector/cam") + std::to_string(camera_id);
  st.hardware_id = camera_id == 0 ? "camera_wide" : "camera_tele";

  // ⚠️ 等级只反映"这一帧判得顺不顺"，**不是节点健康**。
  //    UNKNOWN 不是故障 —— 它可能是"相机没看清"（正常，靠近重看即可）。
  st.level = (v.color == kRed || v.color == kBlue)
                 ? diagnostic_msgs::msg::DiagnosticStatus::OK
                 : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  st.message = v.note.empty() ? color_name(v.color) : v.note;

  auto kv = [](const std::string& k, const std::string& val) {
    diagnostic_msgs::msg::KeyValue p; p.key = k; p.value = val; return p;
  };
  char buf[192];
  st.values.push_back(kv("top_color", color_name(v.color)));
  st.values.push_back(kv("unknown_reason", std::to_string(static_cast<int>(v.reason))));
  // ⚠️ 来源名**从这条消息自己的 `roi_source` 取名**（2026-09-25 改）：
  //    原先印的是成员 `roi_source_name_`，而那个成员**从不被更新、恒是 "manual"**,
  //    主路实际是 `radar_projected` ⇒ 诊断会把人指到"手动 ROI 开着"上去。
  //    （本函数的参数注释早就写着"与发出去的那条**同源** —— 别在两处各算一次"。）
  {
    const uint8_t rs = out.roi_source;
    const char* nm = (rs == static_cast<uint8_t>(kRadarProjected)) ? "radar_projected"
                   : (rs == static_cast<uint8_t>(kYolo))           ? "yolo"
                   : (rs == static_cast<uint8_t>(kManual))         ? "manual"
                                                                   : "?";
    st.values.push_back(kv("roi_source", nm));
  }
  std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(out.spot_id));
  st.values.push_back(kv("spot_id", buf));

  std::snprintf(buf, sizeof(buf), "%.3f / %.3f (margin %.3f)",
                v.red_ratio, v.blue_ratio, v.color_margin);
  st.values.push_back(kv("red/blue ratio", buf));
  std::snprintf(buf, sizeof(buf), "%d px (%.1f%%)",
                v.valid_pixel_count, v.valid_ratio * 100.0);
  st.values.push_back(kv("valid pixels", buf));

  const char* st_name[] = {"UNCONFIRMED", "CONFIRMED", "COASTING"};
  const int si = static_cast<int>(out.state);
  st.values.push_back(kv("state", (si >= 0 && si < 3) ? st_name[si] : "?"));
  std::snprintf(buf, sizeof(buf), "%d / %d", enter_confirm_frames_, exit_confirm_frames_);
  st.values.push_back(kv("enter/exit frames", buf));

  std::snprintf(buf, sizeof(buf), "%.2f / [%.2f, %.2f]",
                color_params_.decisive_ratio, color_params_.close_lo, color_params_.close_hi);
  st.values.push_back(kv("decisive / close_band", buf));
  st.values.push_back(kv("color_yaml", color_yaml_path_));
  std::snprintf(buf, sizeof(buf), "%.2f", total_ms);
  st.values.push_back(kv("total_ms", buf));

  arr.status.push_back(st);
  pub_diagnostics_->publish(arr);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 动态参数
// ═══════════════════════════════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult SkyBlockDetector::on_parameter_change(
    const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult res;
  res.successful = true;

  for (const auto& p : params) {
    const auto& n = p.get_name();
    if (n == "decisive_ratio") {
      const double x = p.as_double();
      if (x < 0.5 || x > 0.999) {
        res.successful = false;
        res.reason = "decisive_ratio 必须在 [0.5, 0.999]";
        return res;
      }
      color_params_.decisive_ratio = x;
    } else if (n == "close_lo") {
      color_params_.close_lo = std::max(0.0, std::min(p.as_double(), 0.5));
    } else if (n == "close_hi") {
      color_params_.close_hi = std::max(0.5, std::min(p.as_double(), 1.0));
    } else if (n == "min_valid_pixels") {
      color_params_.min_valid_pixels = std::max(1, static_cast<int>(p.as_int()));
    } else if (n == "min_valid_ratio") {
      color_params_.min_valid_ratio = std::max(0.0, std::min(p.as_double(), 1.0));
    } else if (n == "top_face_frac") {
      top_face_frac_ = std::max(0.01, std::min(p.as_double(), 1.0));
    } else if (n == "debug_roi_ratio") {
      const auto arr = p.as_double_array();
      if (arr.size() == 4) {
        debug_roi_ratio_ = arr;
      } else {
        res.successful = false;
        res.reason = "debug_roi_ratio 必须是 4 个数（x, y, w, h 相对比例）";
        return res;
      }
    } else if (n == "roi_min_area") {
      roi_min_area_ = std::max(1, static_cast<int>(p.as_int()));
    } else if (n == "enter_confirm_frames") {
      enter_confirm_frames_ = std::max(1, static_cast<int>(p.as_int()));
      exit_confirm_frames_  = std::max(enter_confirm_frames_, exit_confirm_frames_);
      for (auto& cam_slots : confirmer_) {
        for (auto& c : cam_slots) {
          c.enter_frames = enter_confirm_frames_;
          c.exit_frames  = exit_confirm_frames_;
        }
      }
    } else if (n == "exit_confirm_frames") {
      exit_confirm_frames_ = std::max(enter_confirm_frames_, static_cast<int>(p.as_int()));
      for (auto& cam_slots : confirmer_) {
        for (auto& c : cam_slots) {
          c.exit_frames = exit_confirm_frames_;
        }
      }
    }
    // ⚠️ `color_yaml_path` **热改无效**：阈值是构造时加载的，改路径不会重新加载。
    //    这条的处置是**不改**（要换阈值文件就重启）—— 静默忽略比假装支持好。
  }
  return res;
}

}  // namespace camera
}  // namespace br_perception

// ═══════════════════════════════════════════════════════════════════════════════
// 组件注册（容器化时被容器加载; 也生成同名独立可执行）
// ═══════════════════════════════════════════════════════════════════════════════
RCLCPP_COMPONENTS_REGISTER_NODE(br_perception::camera::SkyBlockDetector)
