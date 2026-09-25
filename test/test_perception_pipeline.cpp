/// ═══════════════════════════════════════════════════════════════════════════════
/// test_perception_pipeline.cpp — 模块 27 编排薄壳：**天空方块选色决策**单元测试
///
/// ⚠️ 本文件**只测 `choose_sky_color` 这一个 static 纯函数**（不起 ROS 节点）：
///    它是"两路相机的判色结论 → 到底用哪个颜色发给主控"的**全部判据所在**。
///    ⚠️ 没测的部分（组帧 / TF / 事件补帧 / 降级）需要真实 ROS 图，留给集成测试。
///
/// ── 为什么这个函数值得单测（它只值 4 行判据）──
///   它决定**写入 `BuildingSpot.top_color` 的那个值**，而规则 3.5.6/3.5.7 里那
///   40/80 分的归属**只由顶面颜色决定** ⇒ 这个值错了 = **把塔送给对手**。
///   最贵的一条是「**两路相机打架时不许下结论**」—— 宁可让主控看到"未知"，
///   也不拿一个可能就是错色去覆盖。**这条要是被人"优化"掉，现场没人看得出来。**
///
/// ⚠️ TEST 名用中文、**组名用 ASCII**（`--gtest_filter=ChooseSkyColorTest.*`）。
/// ⚠️ 全部用例确定性：输入是手造的 struct，无 ROS、无时间、无随机。
/// ═══════════════════════════════════════════════════════════════════════════════

#include "br_perception/core/perception_pipeline.hpp"

#include <gtest/gtest.h>

#include <string>

namespace br_perception {
namespace core {

using In = PerceptionPipeline::SkyColorInput;

namespace {

constexpr uint8_t kNone    = 0;
constexpr uint8_t kRed     = 1;
constexpr uint8_t kBlue    = 2;
constexpr uint8_t kUnknown = 3;

constexpr uint8_t kUnconfirmed = 0;
constexpr uint8_t kConfirmed   = 1;
constexpr uint8_t kCoasting    = 2;

constexpr double kTimeout = 0.5;

/// 造一路"新鲜且已确认"的输入
In fresh(uint8_t color, uint8_t state = kConfirmed, double age = 0.0)
{
  In in;
  in.present    = true;
  in.color      = color;
  in.state      = state;
  in.age_known  = true;
  in.age_sec    = age;
  return in;
}

/// 没收到过这一路
In absent()
{
  return In{};
}

}  // namespace

class ChooseSkyColorTest : public ::testing::Test
{
protected:
  uint8_t     color{0};
  std::string why;

  bool run(const In& wide, const In& tele, double timeout = kTimeout)
  {
    color = 0;
    why.clear();
    return PerceptionPipeline::choose_sky_color(wide, tele, timeout, color, why);
  }
};

// ═══════════════════════════════════════════════════════════════════════════════
// 基本：取哪一路
// ═══════════════════════════════════════════════════════════════════════════════
TEST_F(ChooseSkyColorTest, 只有长焦确认就用长焦)
{
  ASSERT_TRUE(run(absent(), fresh(kRed)));
  EXPECT_EQ(color, kRed);
}

TEST_F(ChooseSkyColorTest, 只有宽角确认就用宽角)
{
  ASSERT_TRUE(run(fresh(kBlue), absent()));
  EXPECT_EQ(color, kBlue);
}

TEST_F(ChooseSkyColorTest, 两路都确认且一致_长焦优先)
{
  // ⚠️ 长焦是判色主力（200mm 方块在 5m 处：宽角约 18px、长焦 50px+）——
  //    一致时优先取它，不是随便取。
  ASSERT_TRUE(run(fresh(kRed), fresh(kRed)));
  EXPECT_EQ(color, kRed);
  EXPECT_NE(why.find("长焦"), std::string::npos) << why;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ⚠️⚠️ 最贵的一条：两路打架 ⇒ **不下结论**
// ═══════════════════════════════════════════════════════════════════════════════
TEST_F(ChooseSkyColorTest, 两路结论相反时必须报未知而不是二选一)
{
  EXPECT_FALSE(run(fresh(kRed), fresh(kBlue)));
  EXPECT_FALSE(run(fresh(kBlue), fresh(kRed)));
  EXPECT_FALSE(why.empty());                       // ⚠️ 必须说得出理由
}

TEST_F(ChooseSkyColorTest, 打架时why里带上两个值便于现场对账)
{
  ASSERT_FALSE(run(fresh(kRed), fresh(kBlue)));
  EXPECT_NE(why.find("1"), std::string::npos) << why;   // 宽角=红(1)
  EXPECT_NE(why.find("2"), std::string::npos) << why;   // 长焦=蓝(2)
}

// ═══════════════════════════════════════════════════════════════════════════════
// 三条否决判据（逐条都有用例，免得被人"顺手放宽"）
// ═══════════════════════════════════════════════════════════════════════════════
TEST_F(ChooseSkyColorTest, 太旧的颜色不采信)
{
  // 规则 3.5.8 对手会翻塔 ⇒ 拿旧颜色报"现在塔顶什么色"就是把旧状态当现状
  EXPECT_FALSE(run(fresh(kRed, kConfirmed, kTimeout + 0.01), absent()));
}

TEST_F(ChooseSkyColorTest, 恰好等于超时阈值仍然采信)
{
  // ⚠️ 边界钉死：判据是 `age > timeout` 才算旧（不是 `>=`）。
  //    改动这个不等号的人，会在这里看到红 —— 而现场这种错**根本看不出来**。
  ASSERT_TRUE(run(fresh(kRed, kConfirmed, kTimeout), absent()));
  EXPECT_EQ(color, kRed);
}

TEST_F(ChooseSkyColorTest, COASTING不采信)
{
  // `CameraSkyBlock.state` 三档的原话：COASTING = "曾确认过、最近几帧没满足，
  //  **不要据此翻状态**"。拿它当结论 = 拿一个正在掉帧的观测去改塔的归属。
  EXPECT_FALSE(run(fresh(kRed, kCoasting), absent()));
  EXPECT_FALSE(run(fresh(kRed, kUnconfirmed), absent()));
}

TEST_F(ChooseSkyColorTest, 没时间戳就不采信)
{
  // 判不了新鲜度 ⇒ 不采信（**不猜**）。同 `project_spot` 对空 frame_id 的处置。
  In in = fresh(kRed);
  in.age_known = false;
  EXPECT_FALSE(run(in, absent()));
}

TEST_F(ChooseSkyColorTest, NONE与UNKNOWN都不是可用颜色)
{
  // ⚠️ `NONE` = "这里没有方块"，`UNKNOWN` = "有方块但判不出色" ——
  //    两者都**不能**拿去覆盖雷达填的值（否则等于用"没看见"去否定雷达的"有塔"）。
  EXPECT_FALSE(run(fresh(kNone),    absent()));
  EXPECT_FALSE(run(fresh(kUnknown), absent()));
}

TEST_F(ChooseSkyColorTest, 一路不可用时另一路照样能用)
{
  // ⚠️ 这是"冗余通道"的意义：长焦被遮挡/太重（COASTING），宽角还能顶上
  ASSERT_TRUE(run(fresh(kRed), fresh(kBlue, kCoasting)));
  EXPECT_EQ(color, kRed);
}

TEST_F(ChooseSkyColorTest, 两路都没有就别下结论)
{
  EXPECT_FALSE(run(absent(), absent()));
  EXPECT_FALSE(why.empty());
}

}  // namespace core
}  // namespace br_perception
