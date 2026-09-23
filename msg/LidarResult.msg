# 雷达感知结果 —— 容器 A(雷达流水线) → 容器 C(融合) 的正式接口 (设计文档 §5.1)
#
# 替代原先"拿 diagnostic_msgs/DiagnosticArray 当数据接口"的做法: 那是给人看的。
# MarkerArray / debug_cloud 保留, 但从此只作可视化, 不再承担数据职责。

# ⚠️ frame_id **不是 "world"** —— 2026-09-22 接线时核实：雷达节点里**没有任何坐标变换**
#    （全文件搜不到 transform），所以那些坐标实际在**输入点云的坐标系**里
#    （实机 = 驱动给的 livox 系；仿真 = Gazebo 给的系）。
#    发布端**原样继承**输入点云的 frame_id，因此本字段说的就是**真的那个系**。
#    硬写 "world" 等于虚构一个本层没产出的坐标系，下游照它做 TF 会失败且**错得没提示**。
#    转 world 是**融合层**的活（它有 TF、有外参、有里程计）—— 那是本消息的消费者要做的。
#
# 时间戳：**曝光/采集时刻**（继承输入点云），不是处理完的时刻（任务书 §四 时间戳约定）。
std_msgs/Header header

# ── 检测结果 ──
Obstacle[] obstacles
BuildingSpot[] building_spots
PillarStatus mustika_pillar
PillarStatus core_pillar

# ── 诊断 ──
# 各阶段耗时 [ms], 下标与 StageProfiler::declare 的顺序一一对应 (timer_utils)
float32[] stage_elapsed_ms
# 雷达流水线在线 (替代原先塞在 DiagnosticArray 里的 watchdog 位)
bool watchdog_ok
