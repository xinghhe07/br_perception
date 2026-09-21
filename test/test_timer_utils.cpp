/// ═══════════════════════════════════════════════════════════════════════════
/// test_timer_utils.cpp — timer_utils 单元测试 (任务书 §11.1)
///
/// ⚠ 本测试**不起 ROS** —— timer_utils 是纯算法库 (设计文档 §4 硬性规则第 1 条
///   「算法类不碰 ROS」), 这正是它能被 gtest 直接单测的原因。
///
/// ⚠ 全部用例**不依赖真实等待时长**:
///   - Timer / ScopedTimer 用忙等 (busy_wait_ms), 断言留 5~10 倍余量
///   - FpsCounter 注入假时间戳 (at_ms), 完全确定性
///   因此结果与机器负载无关。
/// ═══════════════════════════════════════════════════════════════════════════

#include "br_perception/utils/timer_utils.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

namespace br_perception {
namespace utils {
namespace {

/// 忙等 ms 毫秒。
/// 刻意**不用 std::this_thread::sleep_for** —— 避免给纯算法测试引入
/// <thread> / pthread 的链接依赖。
void busy_wait_ms(double ms)
{
  const SteadyClock::time_point deadline =
      SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
                               std::chrono::duration<double, std::milli>(ms));
  while (SteadyClock::now() < deadline) {
    // spin
  }
}

/// 构造假时间戳 (基准时刻 + ms 毫秒), 供 FpsCounter 注入用
SteadyClock::time_point at_ms(double ms)
{
  return SteadyClock::time_point{} +
         std::chrono::duration_cast<SteadyClock::duration>(
             std::chrono::duration<double, std::milli>(ms));
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════
// Timer
// ═══════════════════════════════════════════════════════════════════════════

TEST(TimerTest, 未开始计时时返回零且不报错)
{
  Timer t;
  EXPECT_FALSE(t.running());
  EXPECT_DOUBLE_EQ(0.0, t.stop());
  EXPECT_DOUBLE_EQ(0.0, t.elapsed_ms());
}

TEST(TimerTest, stop幂等且重复调用返回同一结果)
{
  Timer t;
  t.start();
  busy_wait_ms(2.0);
  const double first = t.stop();
  EXPECT_GE(first, 1.0);
  EXPECT_FALSE(t.running());
  EXPECT_DOUBLE_EQ(first, t.stop());
}

TEST(TimerTest, 计时中elapsed不改变running状态)
{
  Timer t;
  t.start();
  busy_wait_ms(2.0);
  const double mid = t.elapsed_ms();
  EXPECT_TRUE(t.running());          // elapsed_ms 只读, 不停止计时
  EXPECT_GE(mid, 1.0);
  EXPECT_GE(t.stop(), mid);          // 单调时钟 → 最终值只会更大
}

TEST(TimerTest, 重新start丢弃上一轮)
{
  Timer t;
  t.start();
  busy_wait_ms(20.0);
  t.start();                         // 未 stop 就重启
  EXPECT_LT(t.elapsed_ms(), 10.0);   // 若上一轮的 20ms 被保留, 这里必超
  busy_wait_ms(2.0);
  EXPECT_GE(t.stop(), 2.0);
}

TEST(TimerTest, reset回到初始状态)
{
  Timer t;
  t.start();
  busy_wait_ms(2.0);
  (void)t.stop();   // 只为停表, 返回值此处无意义 → 显式丢弃 (stop 是 [[nodiscard]])
  t.reset();
  EXPECT_FALSE(t.running());
  EXPECT_DOUBLE_EQ(0.0, t.stop());
}

// ═══════════════════════════════════════════════════════════════════════════
// ScopedTimer
// ═══════════════════════════════════════════════════════════════════════════

TEST(ScopedTimerTest, 析构时写出耗时)
{
  double ms = -1.0;
  {
    ScopedTimer st(ms);
    busy_wait_ms(2.0);
  }
  EXPECT_GE(ms, 1.0);
}

TEST(ScopedTimerTest, 显式stop后析构不再二次写入)
{
  double ms = -1.0;
  {
    ScopedTimer st(ms);
    busy_wait_ms(2.0);
    EXPECT_GE(st.stop(), 1.0);
    ms = -12345.0;                   // 占位: 析构若再写一次就会覆盖掉它
  }
  EXPECT_DOUBLE_EQ(-12345.0, ms);
}

TEST(ScopedTimerTest, 重复stop返回第一次的结果)
{
  double ms = 0.0;
  ScopedTimer st(ms);
  busy_wait_ms(2.0);
  const double first = st.stop();
  busy_wait_ms(2.0);                 // 再等一会儿
  EXPECT_DOUBLE_EQ(first, st.stop());
  EXPECT_DOUBLE_EQ(first, ms);
}

// ═══════════════════════════════════════════════════════════════════════════
// StageProfiler
// ═══════════════════════════════════════════════════════════════════════════

TEST(StageProfilerTest, 初始无阶段且下标全非法)
{
  StageProfiler p;
  EXPECT_EQ(0u, p.size());
  EXPECT_FALSE(p.valid_stage(0));
  EXPECT_EQ(0u, p.stats(0).count);
  EXPECT_TRUE(p.name(0).empty());
}

TEST(StageProfilerTest, declare返回递增下标)
{
  StageProfiler p;
  EXPECT_EQ(0u, p.declare("preprocess"));
  EXPECT_EQ(1u, p.declare("ground_segment"));
  EXPECT_EQ(2u, p.declare("cluster_extract"));
  EXPECT_EQ(3u, p.size());
  EXPECT_EQ("ground_segment", p.name(1));
  EXPECT_TRUE(p.valid_stage(2));
  EXPECT_FALSE(p.valid_stage(3));
}

TEST(StageProfilerTest, 同名不去重)
{
  StageProfiler p;
  const std::size_t a = p.declare("dup");
  const std::size_t b = p.declare("dup");
  EXPECT_NE(a, b);                   // 有意行为: 不共享统计
  EXPECT_EQ(2u, p.size());
}

TEST(StageProfilerTest, 超出容量返回哨兵且不越界)
{
  StageProfiler p;
  for (std::size_t i = 0; i < StageProfiler::kMaxStages; ++i) {
    EXPECT_EQ(i, p.declare("s" + std::to_string(i)));
  }
  EXPECT_EQ(StageProfiler::kInvalidStage, p.declare("overflow"));
  EXPECT_EQ(StageProfiler::kMaxStages, p.size());
}

TEST(StageProfilerTest, record累计平均最值)
{
  StageProfiler p;
  const std::size_t s = p.declare("stage");
  p.record(s, 10.0);
  p.record(s, 30.0);
  p.record(s, 20.0);

  const LatencyStats st = p.stats(s);
  EXPECT_EQ(3u, st.count);
  EXPECT_DOUBLE_EQ(10.0, st.min_ms);
  EXPECT_DOUBLE_EQ(30.0, st.max_ms);
  EXPECT_DOUBLE_EQ(20.0, st.mean_ms);   // (10 + 30 + 20) / 3
  EXPECT_DOUBLE_EQ(20.0, st.last_ms);
}

TEST(StageProfilerTest, 脏数据被丢弃且不计入count)
{
  StageProfiler p;
  const std::size_t s = p.declare("stage");
  p.record(s, 10.0);
  p.record(s, -1.0);
  p.record(s, std::nan(""));
  p.record(s, std::numeric_limits<double>::infinity());
  p.record(s, -std::numeric_limits<double>::infinity());
  p.record(StageProfiler::kInvalidStage, 5.0);
  p.record(999u, 5.0);

  const LatencyStats st = p.stats(s);
  EXPECT_EQ(1u, st.count);              // 只有 10.0 生效
  EXPECT_DOUBLE_EQ(10.0, st.min_ms);
  EXPECT_DOUBLE_EQ(10.0, st.max_ms);
  EXPECT_DOUBLE_EQ(10.0, st.mean_ms);
}

TEST(StageProfilerTest, 单个NaN不污染最值)
{
  StageProfiler p;
  const std::size_t s = p.declare("stage");
  p.record(s, 10.0);
  p.record(s, std::nan(""));
  p.record(s, 30.0);

  const LatencyStats st = p.stats(s);
  EXPECT_EQ(2u, st.count);
  EXPECT_DOUBLE_EQ(10.0, st.min_ms);
  EXPECT_DOUBLE_EQ(30.0, st.max_ms);
  EXPECT_DOUBLE_EQ(20.0, st.mean_ms);
}

TEST(StageProfilerTest, 当前帧与累计统计互不干扰)
{
  StageProfiler p;
  const std::size_t s = p.declare("stage");

  p.begin_frame();
  p.record(s, 10.0);
  EXPECT_FLOAT_EQ(10.0f, p.frame_ms()[s]);

  p.begin_frame();                       // 新一帧: 当前帧清零
  EXPECT_FLOAT_EQ(0.0f, p.frame_ms()[s]);
  EXPECT_EQ(1u, p.stats(s).count);       // 累计**不**被 begin_frame 清掉

  p.record(s, 30.0);
  EXPECT_FLOAT_EQ(30.0f, p.frame_ms()[s]);
  EXPECT_EQ(2u, p.stats(s).count);
  EXPECT_DOUBLE_EQ(20.0, p.stats(s).mean_ms);
}

TEST(StageProfilerTest, 统计保持double精度而当前帧按设计转float)
{
  StageProfiler p;
  const std::size_t s = p.declare("precision");

  // 取位数超过 float 可表示范围的值 (float 只有 ~7 位十进制有效数字):
  // 若实现里把统计量存成 float, 下面第一条断言必然失败。
  constexpr double v1 = 123.456789012345;
  constexpr double v2 = 0.000123456789;

  p.begin_frame();
  p.record(s, v1);

  const LatencyStats first = p.stats(s);
  EXPECT_DOUBLE_EQ(v1, first.last_ms);
  EXPECT_DOUBLE_EQ(v1, first.mean_ms);
  EXPECT_FLOAT_EQ(static_cast<float>(v1), p.frame_ms()[s]);   // 当前帧按设计是 float

  p.record(s, v2);
  const LatencyStats second = p.stats(s);
  EXPECT_EQ(2u, second.count);
  EXPECT_DOUBLE_EQ((v1 + v2) / 2.0, second.mean_ms);
  EXPECT_DOUBLE_EQ(v2, second.last_ms);
  EXPECT_FLOAT_EQ(static_cast<float>(v2), p.frame_ms()[s]);
}

TEST(StageProfilerTest, frame数组未使用的元素保持为零)
{
  StageProfiler p;
  const std::size_t s = p.declare("stage");
  p.begin_frame();
  p.record(s, 5.0);

  const std::array<float, StageProfiler::kMaxStages>& f = p.frame_ms();
  EXPECT_FLOAT_EQ(5.0f, f[s]);
  for (std::size_t i = 1; i < StageProfiler::kMaxStages; ++i) {
    EXPECT_FLOAT_EQ(0.0f, f[i]);
  }
}

TEST(StageProfilerTest, reset_stats保留阶段声明)
{
  StageProfiler p;
  const std::size_t s = p.declare("stage");
  p.record(s, 10.0);
  p.reset_stats();

  EXPECT_EQ(0u, p.stats(s).count);
  EXPECT_EQ(1u, p.size());
  EXPECT_EQ("stage", p.name(s));
  EXPECT_TRUE(p.valid_stage(s));
}

// ═══════════════════════════════════════════════════════════════════════════
// FpsCounter
// ═══════════════════════════════════════════════════════════════════════════

TEST(FpsCounterTest, 不足两帧时为零)
{
  FpsCounter c;
  EXPECT_DOUBLE_EQ(0.0, c.fps());
  c.tick(at_ms(0.0));                // 首帧只记时间戳
  EXPECT_EQ(0u, c.sample_count());
  EXPECT_DOUBLE_EQ(0.0, c.fps());
  EXPECT_DOUBLE_EQ(0.0, c.last_interval_ms());
}

TEST(FpsCounterTest, 由假时钟算出帧率)
{
  FpsCounter c(10);
  double t = 0.0;
  for (int i = 0; i < 11; ++i) {     // 11 帧 → 10 个 20ms 间隔 → 50 fps
    c.tick(at_ms(t));
    t += 20.0;
  }
  EXPECT_EQ(10u, c.sample_count());
  EXPECT_DOUBLE_EQ(50.0, c.fps());
  EXPECT_DOUBLE_EQ(20.0, c.last_interval_ms());
}

TEST(FpsCounterTest, 滑动窗口只保留最近N个间隔)
{
  FpsCounter c(3);
  // 帧间隔依次为 10 / 20 / 30 / 40 / 50 ms
  c.tick(at_ms(0.0));
  c.tick(at_ms(10.0));
  c.tick(at_ms(30.0));
  c.tick(at_ms(60.0));
  c.tick(at_ms(100.0));
  c.tick(at_ms(150.0));

  EXPECT_EQ(3u, c.sample_count());   // 窗口容量
  // 只剩 30 / 40 / 50 → 均值 40ms → 25 fps。
  // 若挤出样本时没同步从累加和里减掉, 这里会算成 20 fps。
  EXPECT_DOUBLE_EQ(25.0, c.fps());
  EXPECT_DOUBLE_EQ(50.0, c.last_interval_ms());
}

TEST(FpsCounterTest, 零间隔被忽略)
{
  FpsCounter c;
  c.tick(at_ms(0.0));
  c.tick(at_ms(0.0));                // 同一时刻 → 间隔为 0, 丢弃
  EXPECT_EQ(0u, c.sample_count());
  EXPECT_DOUBLE_EQ(0.0, c.fps());
}

TEST(FpsCounterTest, 时间回拨的负间隔被忽略且时间戳继续更新)
{
  FpsCounter c;
  c.tick(at_ms(100.0));
  c.tick(at_ms(50.0));               // 回拨 → 丢弃
  EXPECT_EQ(0u, c.sample_count());

  c.tick(at_ms(120.0));              // 相对上一时间戳(50) 的间隔 = 70ms
  EXPECT_EQ(1u, c.sample_count());
  EXPECT_DOUBLE_EQ(70.0, c.last_interval_ms());
}

TEST(FpsCounterTest, reset清空且下一帧重新当首帧)
{
  FpsCounter c;
  c.tick(at_ms(0.0));
  c.tick(at_ms(20.0));
  EXPECT_EQ(1u, c.sample_count());

  c.reset();
  EXPECT_EQ(0u, c.sample_count());
  EXPECT_DOUBLE_EQ(0.0, c.fps());
  EXPECT_DOUBLE_EQ(0.0, c.last_interval_ms());

  c.tick(at_ms(1000.0));             // reset 后第一帧不产生间隔
  EXPECT_EQ(0u, c.sample_count());
  EXPECT_DOUBLE_EQ(0.0, c.fps());
}

TEST(FpsCounterTest, 窗口为零按一处理)
{
  FpsCounter c(0);
  c.tick(at_ms(0.0));
  c.tick(at_ms(20.0));
  c.tick(at_ms(60.0));               // 间隔 20 / 40 → 窗口 1 → 只剩 40
  EXPECT_EQ(1u, c.sample_count());
  EXPECT_DOUBLE_EQ(25.0, c.fps());
}

// ═══════════════════════════════════════════════════════════════════════════
// classify_latency / to_string
// ═══════════════════════════════════════════════════════════════════════════

TEST(ClassifyLatencyTest, 默认阈值三档边界)
{
  const LatencyThresholds th;        // warn 56 / error 80 (§15.3 / §7.3)
  EXPECT_EQ(LatencyLevel::OK, classify_latency(0.0, th));
  EXPECT_EQ(LatencyLevel::OK, classify_latency(56.0, th));    // 边界算 OK
  EXPECT_EQ(LatencyLevel::WARN, classify_latency(56.1, th));
  EXPECT_EQ(LatencyLevel::WARN, classify_latency(80.0, th));  // 边界算 WARN
  EXPECT_EQ(LatencyLevel::ERROR, classify_latency(80.1, th));
}

TEST(ClassifyLatencyTest, 省略阈值时用默认值)
{
  EXPECT_EQ(LatencyLevel::OK, classify_latency(10.0));
  EXPECT_EQ(LatencyLevel::ERROR, classify_latency(100.0));
}

TEST(ClassifyLatencyTest, 非有限值一律ERROR)
{
  EXPECT_EQ(LatencyLevel::ERROR, classify_latency(std::nan("")));
  EXPECT_EQ(LatencyLevel::ERROR,
            classify_latency(std::numeric_limits<double>::infinity()));
  EXPECT_EQ(LatencyLevel::ERROR,
            classify_latency(-std::numeric_limits<double>::infinity()));
}

TEST(ClassifyLatencyTest, 阈值配反时退化为单一门限)
{
  LatencyThresholds th;
  th.warn_ms = 90.0;
  th.error_ms = 60.0;                // 配反了
  EXPECT_EQ(LatencyLevel::OK, classify_latency(60.0, th));
  EXPECT_EQ(LatencyLevel::ERROR, classify_latency(60.1, th));
  EXPECT_EQ(LatencyLevel::ERROR, classify_latency(1000.0, th));
}

TEST(ClassifyLatencyTest, 等级转字符串覆盖全部取值)
{
  EXPECT_STREQ("OK", to_string(LatencyLevel::OK));
  EXPECT_STREQ("WARN", to_string(LatencyLevel::WARN));
  EXPECT_STREQ("ERROR", to_string(LatencyLevel::ERROR));
}

}  // namespace utils
}  // namespace br_perception
