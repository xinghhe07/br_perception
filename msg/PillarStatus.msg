# 柱子状态消息
# 描述场地中柱子的几何信息和状态（穆斯蒂卡柱或核心支柱）

# 柱子类型
# 0 = 穆斯蒂卡柱 (MUSTIKA_PILLAR)  — 穆斯蒂卡初始放置的柱子
# 1 = 核心支柱 (CORE_PILLAR)        — 场地核心支柱
uint8 type

# 柱子中心位置（世界坐标系，地面投影点）
geometry_msgs/Point position

# ⚠️⚠️ **"这根柱子没检出"的哨兵约定（2026-09-22 写入本文件，别再靠口头传）**
#
#   本消息**没有 `detected` 字段**，所以"没检出"与"检出但高度为 0"在消息层面
#   **长得一模一样**。当前的约定是给**全零**（本消息的默认构造值）：
#
#       height == 0.0 && diameter == 0.0   ⇒   **未检出**
#
#   ⚠️ 这是一个**跨模块的隐性契约**，两头都要守住：
#     · 发布端（`lidar_perception_node` 的 `to_msg_pillar`）：未检出时返回默认构造
#     · 消费端（`perception_pipeline` 的 `transform_lidar_to_world`）：全零**不参与 TF**
#       （拿 (0,0,0) 去变换会得到"世界系原点附近"的**假柱子位置**，比不报更糟）
#
#   ⚠️ 它**脆弱在哪**：只要发布端改成"未检出也填个预期高度/直径"（看起来更友好），
#      消费端就会**静默地**开始 TF 一根并不存在的柱子 —— **没有任何编译期或运行期提示**。
#      真要改，正确做法是给本消息加一个显式的 `bool detected`（加法改动，
#      **不影响线上协议** —— `protocol_encoder` 只写 occupied 位，不读 height/diameter）。
#
# 柱子高度 [m]
float32 height

# 柱子直径 [m]
float32 diameter

# 柱子顶部是否检测到物体（穆斯蒂卡或方块）
bool has_object_on_top

# 检测置信度 [0.0, 1.0]
float32 confidence
