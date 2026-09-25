/// ═══════════════════════════════════════════════════════════════════════════════
/// test_camera_perception_node.cpp — 模块 10 聚合薄壳：**槽状态判定**单元测试
///
/// ⚠️ 只测 **`judge_slot`**（static 纯函数）—— **不起节点、不组帧、不发布**。
///    ⚠️ **别把本文件的绿当成"聚合薄壳能跑"**：组帧、时间对齐、诊断、回调都不在这里。
///
/// ── 为什么这几行值得单测（它出过一次错）──
///   它是**整个相机层的健康读数**（`/perception/camera/frame` 的 `slot_status`）。
///   而**原判据是 `age > 0`** —— 曝光时刻永远早于组帧时刻 ⇒ `age` 恒为正
///   ⇒ `state` **永远到不了 FRESH** ⇒ **那一档等于不存在**，
///   与 `msg/SlotStatus.msg` 的字段定义和验收判据**直接打架**。
///   ⇒ 本文件最贵的一条是 `FRESH这一档必须真的可达`（把那个历史错钉死）。
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=SlotJudgeTest.*`）。
/// ⚠️ 全部用例确定性：纯算术，无时间、无 ROS。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/camera/camera_perception_node.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace br_perception {
namespace camera {

using CP = CameraPerceptionNode;

namespace {
constexpr std::uint8_t kNever = br_perception::msg::SlotStatus::S_NEVER;
constexpr std::uint8_t kFresh = br_perception::msg::SlotStatus::S_FRESH;
constexpr std::uint8_t kStale = br_perception::msg::SlotStatus::S_STALE;
}  // namespace

TEST(SlotJudgeTest, 没数据是NEVER)
{
  // ⚠️ NEVER 与 STALE 是**两回事**（见 msg/SlotStatus.msg）：
  //    NEVER = "从未收到过"（话题名对不上 / 发布端没起 / 这路相机没装）；
  //    STALE = "跑过，但慢了"。**现场排查方向完全不同。**
  const auto v = CP::judge_slot(false, 0.0, 0.5);
  EXPECT_EQ(v.state, kNever);
  EXPECT_FLOAT_EQ(v.stale_sec, 0.0f) << "NEVER 时本字段无意义（msg 里写明的）";
}

TEST(SlotJudgeTest, 容差内是FRESH)
{
  const auto v = CP::judge_slot(true, 0.10, 0.5);
  EXPECT_EQ(v.state, kFresh);
  EXPECT_FLOAT_EQ(v.stale_sec, 0.10f);
}

TEST(SlotJudgeTest, 超容差是STALE)
{
  const auto v = CP::judge_slot(true, 0.80, 0.5);
  EXPECT_EQ(v.state, kStale);
  EXPECT_FLOAT_EQ(v.stale_sec, 0.80f);
}

TEST(SlotJudgeTest, 边界恰好等于容差算FRESH)
{
  // ⚠️ 判据是**严格 `>`** ⇒ 恰好等于容差**算 FRESH**。
  const auto v = CP::judge_slot(true, 0.5, 0.5);
  EXPECT_EQ(v.state, kFresh) << "恰好等于容差该算 FRESH（判据是严格 >）";
}

// ═══════════════════════════════════════════════════════════════════════════
// ★★ 最贵的一条：FRESH 必须真的可达
// ═══════════════════════════════════════════════════════════════════════════

TEST(SlotJudgeTest, FRESH这一档必须真的可达)
{
  // ⚠️⚠️ **本文件最贵的一条**。历史上这条判据写成过 `age > 0` ——
  //    而**曝光时刻永远早于组帧时刻** ⇒ `age` 恒为正
  //    ⇒ **每个有数据的槽都报 STALE，FRESH 这一档等于不存在**。
  //    ⚠️ 这个错**不报错**，只是让"健康"与"不健康"在读数上**完全没有区别**。
  //    ⇒ 用例：拿一串**真实世界里会出现的正 age**（都是正的！），
  //      必须**至少有一个 FRESH**。
  bool saw_fresh = false, saw_stale = false;
  for (double age : {0.001, 0.01, 0.1, 0.3, 0.49, 0.51, 1.0, 2.0}) {
    const auto v = CP::judge_slot(true, age, 0.5);
    if (v.state == kFresh) saw_fresh = true;
    if (v.state == kStale) saw_stale = true;
  }
  EXPECT_TRUE(saw_fresh)
      << "**所有 age 都是正的、却没有一个 FRESH** —— 判据八成写成了 `age > 0`";
  EXPECT_TRUE(saw_stale) << "大的 age 也该判出 STALE（两档都要出现）";
}

TEST(SlotJudgeTest, 负age是FRESH不是STALE)
{
  // ⚠️ 负 age = 该槽时间戳在**未来**（时钟跳变 / 驱动异常）。
  //    它**不是"数据旧"** —— 是"**钟不对**"。⇒ 算 FRESH，
  //    该异常由 `stale_sec` 的负值**本身**暴露。
  const auto v = CP::judge_slot(true, -0.3, 0.5);
  EXPECT_EQ(v.state, kFresh) << "负 age 是钟的问题，不是数据旧";
  EXPECT_FLOAT_EQ(v.stale_sec, -0.3f) << "**如实报负值，不钳到 0**（钳了就掩盖了那个异常）";
}

TEST(SlotJudgeTest, 容差为零时任何正age都是STALE)
{
  // ⚠️ 语义边界：容差 0 ⇒ 只有 age <= 0 才算 FRESH。
  //    钉住它免得有人把"容差 0"当成"不判陈旧"。
  EXPECT_EQ(CP::judge_slot(true, 0.0, 0.0).state, kFresh) << "恰好 0 算 FRESH（非严格）";
  EXPECT_EQ(CP::judge_slot(true, 0.001, 0.0).state, kStale);
}

TEST(SlotJudgeTest, note字段跟着状态走)
{
  // note 是给人看的短说明（RViz/日志里直接显示）
  EXPECT_STREQ(CP::judge_slot(false, 0.0, 0.5).note, "never");
  EXPECT_STREQ(CP::judge_slot(true, 0.1, 0.5).note, "ok");
  EXPECT_STREQ(CP::judge_slot(true, 1.0, 0.5).note, "stale");
  EXPECT_NE(CP::judge_slot(true, 1.0, 0.5).note, nullptr);   // ⚠️ 不能是空指针
}

TEST(SlotJudgeTest, 三档互不相同)
{
  // ⚠️ 三档的**数值**不能撞（撞了 = 下游分不出"没装上"和"慢了"）
  EXPECT_NE(kNever, kFresh);
  EXPECT_NE(kNever, kStale);
  EXPECT_NE(kFresh, kStale);
}

}  // namespace camera
}  // namespace br_perception
