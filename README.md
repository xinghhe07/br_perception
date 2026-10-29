# br_perception — ROBOCON 2027 BR 感知系统

> **目标机器人**：BR（Builder Robot，**完全自主**）+ TR（Transport Robot，可手动/自动）配合的 180 秒建塔任务。
> **本包职责**：把 Livox Mid-360 雷达 + 两路工业相机 + 深度相机，变成一份**可信的 `PerceptionFrame`**（自位姿、建筑位/塔状态、天空方块顶面颜色、穆斯蒂卡状态、障碍物、诊断），经串口交给 STM32 主控。

- 框架：**ROS 2 Humble** · **C++17** · `ament_cmake` · `rclcpp_components`
- 平台：GMKtec AMD 迷你主机（Zen 4，8C16T，Ubuntu）+ STM32 主控（串口）
- 传感器：
  - **Livox Mid-360** ×1（雷达，`CustomMsg`）
  - **宽角 = JR（UVC / V4L2）** —— 自建 `v4l2_camera` 驱动（替换 `usb_cam`，其时间戳老 ~760ms）
  - **长焦 = 海康 GigE（MVS SDK）** —— 自建 `mvs_camera` 驱动，原生 1624×1240 **居中裁到 1624×976** 输出
  - **Intel RealSense D435i** ×1 —— 近距深度 + **近距判色**，装在**机械臂端**（eye-in-hand，动态 TF）

> ⚠️ **进度以 [`doc/模块进度.md`](doc/模块进度.md) 为唯一事实源**，不要看 `doc/任务_v1.7.docx` §15.1（已过期）。

---

## 1. 系统架构

```
                      ┌─────────────────────────── 传感器层 ───────────────────────────┐
                      │ Livox Mid-360     JR(wide, UVC)   海康 GigE(tele)   D435i(arm) │
                      │ /livox/lidar      /camera_wide     /camera_tele      depth+Rgb │
                      └──────────┬───────────────────┬───────────────────┬────────────┘
                                 │                   │                   │
                  ┌──────────────▼─────────┐  ┌──────▼──────────┐  ┌─────▼──────────┐
                  │ 雷达层 lidar/          │  │ 相机层 camera/   │  │ camera_depth   │
                  │ preprocess→ground      │  │ preprocess→mustika│ │ (D435i 深度/判色)│
                  │ →cylinder→cluster      │  │ →pillar→building  │ └─────┬──────────┘
                  │ →building_spot         │  │ →sky_block→组帧   │       │
                  └──────────┬─────────────┘  └──────┬───────────┘       │
                             │ /perception/lidar/*   │ /perception/camera/*
                             └──────────────┬────────┴───────────────────┘
                                            ▼
                        ┌──────────────── 融合层 fusion/ ────────────────┐
                        │ coordinate_transformer (TF + 里程计可靠性)      │
                        │ kalman_tracker ← target_associator             │
                        │ mustika_pillar_check | fusion_decisions | score│
                        │ fusion_node → /perception/fusion/state         │
                        └───────────────────┬───────────────────────────┘
                                            ▼
                        ┌──────────── 编排层 core/ ────────────┐
                        │ perception_pipeline → /perception/output
                        │  · 组 PerceptionFrame               │
                        │  · 降级状态机 + 紧急停止 + 事件补帧   │
                        │  · 相机确认色覆盖雷达 UNKNOWN         │
                        │ health_monitor → /perception/health   │
                        └───────────────────┬────────────────┘
                                            ▼
                        ┌──────── 通信层 communication/ ────────┐
                        │ comm_node → protocol_encoder → CRC16  │
                        │ → serial_manager (termios 460800)     │
                        │ protocol_decoder ← ACK / 0x10 BR_RESET│
                        └───────────────────────────────────────┘

    里程计：Point-LIO（纯里程计）→ /aft_mapped_to_init → coordinate_transformer
```

**TF 树**（单一权威，禁止同帧双父）：

```
world ── odom ── base_link ──┬── livox_frame
  ▲         ▲         ▲       ├── camera_wide_frame
  │         │         │       ├── camera_tele_frame   (均朝前，非前后相机)
场地系   里程计    Point-LIO  └── camera_depth_frame (arm 端，动态 TF)
```

- `world`：场地系，原点 = 场地西南角，X 东 / Y 北 / Z 上，单位 m。
- `odom ← base_link`：Point-LIO 转播；`world ← odom`：`coordinate_transformer` 广播（含 BR_RESET 位姿注入，窗口 5 s）。
- ⚠️ D435i 的 TF 是**机械臂正运动学（动态）**，**不能**与静态 `SENSOR_TF` 同时广播。

---

## 2. 目录结构

```
br_perception/
├── config/                     # 全部参数（改参数只改这里，不碰代码）
│   ├── field_geometry.yaml     # 场地几何：平台/柱子/建筑位/区域/棋盘/配色
│   ├── lidar_params.yaml       # 雷达预处理/地面/圆柱/聚类/分层高度
│   ├── camera_params.yaml      # 相机节点参数（内参/ROI/模型/判色门控）
│   ├── camera_hardware.yaml    # 硬件规格（wide=JR / tele=海康 / D435i）；驱动与标定读
│   ├── color_thresholds.yaml   # HSV 阈值（现场必须重标）
│   ├── fusion_params.yaml      # 融合/坐标系/里程计可靠性/静态外参/TF
│   ├── perception_params.yaml  # 编排层：话题/超时/降级/事件补帧/延迟阈值
│   ├── comm_params.yaml        # 串口：波特率/CRC/帧格式/频率/诊断周期
│   ├── debug_flags.yaml        # 调试开关（比赛全关）
│   ├── pointlio_gazebo.yaml    # 仿真用 Point-LIO 覆盖参数
│   └── *.rviz                  # lidar_perception / camera_debug / gazebo_sim
├── launch/                     # 8 个启动脚本（见 §4）
├── msg/                        # 16 个自定义消息（见 §5）
├── src/
│   ├── vision/                 # color_utils（HSV 工具层）
│   ├── utils/                  # geometry_utils / timer_utils / Livox 双向桥接
│   ├── lidar/                  # 预处理→地面→圆柱→聚类→建筑位→单体节点
│   ├── camera/                 # 预处理/检测器/组帧/深度/自建驱动(v4l2, mvs)
│   ├── fusion/                 # 坐标/卡尔曼/关联/互补融合/决策/计分/融合节点
│   ├── communication/          # 编码/解码/串口/通信节点
│   └── core/                   # perception_pipeline / health_monitor
├── test/                       # 20+ 单测文件（250 例，10/10 目标全绿）
├── scripts/                    # 标定/调参/仿真/体检/回放/伪下位机（19 个）
├── doc/                        # 模块进度(★唯一事实源)/任务书/设计/调优/规则书
├── CMakeLists.txt
└── package.xml
```

---

## 3. 模块与实现状态

| 层 | 模块 | 状态 |
|---|---|---|
| 工具 | `color_utils`(src/vision) · `geometry_utils` · `timer_utils` | 🧱 / 🟢 / 🟢 |
| 雷达 | `lidar_preprocessor` · `ground_segmenter` · `cylinder_detector` · `cluster_extractor` · `building_spot_analyzer` · `lidar_perception_node` | 🧱×5 · 🟢(cluster) |
| 相机 | `image_preprocessor` · `mustika_detector` · `pillar_detector` · `building_spot_detector` · `sky_block_detector` · `camera_perception_node` · `camera_depth_node` · `v4l2_camera`/`mvs_camera` | 🧱 / 🟢(sky) / 🟢(驱动) |
| 融合 | `coordinate_transformer` · `kalman_tracker` · `target_associator` · `mustika_pillar_check` · `fusion_decisions` · `fusion_node` · `score_estimator` | 🟢×4 · 🔨 |
| 通信 | `protocol_encoder` · `protocol_decoder` · `serial_manager` · `comm_node` | 🟢 / 🟢 / 🧱 / 🧱 |
| 调度 | `perception_pipeline` · `health_monitor` | 🟢 / 🔨 |
| 未实现 | `earth_block_detector`（YOLO26n + HSV 后处理） | ⬜ |
| 非主线 | `extrinsic_monitor`（已降级，默认关闭） | ⛔ |

**状态定义**：`⬜ 未开始` / `🔨 进行中` / `📄 代码完成待编译` / `🧱 colcon build 通过` / `🟢 build 通过 **且** 单测通过` / `⛔ 卡住`。
> ⚠️ 当前总计（2026-09-25）：🟢 14 / 🧱 13 / 🔨 2 / ⬜ 1 / ⛔ 1。逐行明细、每个 🟢 实际覆盖到哪，见台账。

---

## 4. 构建与启动

### 环境要求

| 项 | 说明 |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble（`rclcpp` / `rclcpp_components` / `rosidl`） |
| 编译器 | g++ ≥ 11（C++17） |
| 第三方库 | PCL、Eigen3、OpenCV、`cv_bridge`、`pcl_conversions`、ONNX Runtime（YOLO）、海康 MVS SDK |
| 外部工作区 | **`livox_ros_driver2`** · **`point_lio`**（均非 rosdep 包，需自行编译并 source） |

```bash
source /opt/ros/humble/setup.bash
source ~/point_ws/install/setup.bash      # livox_ros_driver2
source ~/lio_ws/install/setup.bash        # point_lio

cd ~/br_perception_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select br_perception
source install/setup.bash
```

### 启动脚本一览

| launch | 用途 | 关键参数 |
|---|---|---|
| `bringup_full.launch.py` | **比赛模式全量**：雷达 + Point-LIO + 桥接 + 静态 TF + 感知管线 + 相机层 + 通信 | `perception_rviz` `lio` `comm` |
| `bringup_lidar_only.launch.py` | **降级模式**（相机故障）：仅雷达链路 | `livox` `lio` `use_sim_time` |
| `bringup_debug.launch.py` | 调试：全模块 + RViz + 动态参数 + 全调试话题 | — |
| `camera_bringup.launch.py` | 相机驱动（`v4l2` = JR/UVC，`mvs` = 海康 GigE，含 ROI 下发） | 设备/分辨率/ROI |
| `bringup_gazebo.launch.py` | **Gazebo 仿真**：场地 + sensor_rig + Point-LIO + 巡游 | `gui` `motion` |
| `pointlio_mapping.launch.py` | 实机建图（Ctrl-C 落盘 PCD） | `pipeline` |
| `lidar_perception.launch.py` | 仅雷达感知（台架；**自带 `map→livox_frame` 静态 TF**） | `test_mode` `rviz` |
| `record_test_bag.launch.py` | 录制调试 bag（话题白名单） | — |

```bash
ros2 launch br_perception bringup_full.launch.py
ros2 launch br_perception bringup_lidar_only.launch.py lio:=false   # 降级
ros2 launch br_perception bringup_gazebo.launch.py                  # 仿真
```

> ⚠️ **已知坑**
> - `bringup_lidar_only` 不起 `perception_pipeline` ⇒ **没有 `world` 帧广播** ⇒ 雷达节点 `cloud_to_world` 失败、建筑位全报 `UNKNOWN`（台架上属预期）。
> - 仿真必须 `use_sim_time:=true` + `livox:=false`，否则 TF 时间源不同 → 全帧 `UNKNOWN`。
> - `debug_flags.yaml` 在参数列表里**必须放最后**（后者覆盖前者）。
> - `perception_pipeline` 要**同时**加载 `perception_params.yaml` + `fusion_params.yaml`（坐标系/里程计键只在后者）。

---

## 5. 主要话题与消息

| 方向 | 话题 | 类型 |
|---|---|---|
| 雷达原始 | `/livox/lidar` | `livox_ros_driver2/CustomMsg` |
| 雷达点云 | `/livox/lidar/pointcloud` | `PointCloud2` |
| 雷达结果 | `/perception/lidar/result` | `LidarResult` |
| 雷达中间量 | `/perception/lidar/{filtered,ground,non_ground,l0..l2_surface,obstacles,building_spots,pillars}` | `PointCloud2` / `MarkerArray` |
| 相机原始 | `/camera_wide/image_raw` · `/camera_tele/image_raw` · `/camera/color/*` · `/camera/depth/*` | `Image` |
| 预处理 | `/perception/camera_*/preprocessed` | `PreprocessedImage` |
| 相机结果 | `/perception/camera/frame` | `CameraFrame`（含 `SlotStatus`） |
| 相机判色 | `/perception/camera/sky_blocks_{wide,tele}` | `CameraSkyBlock` |
| 融合 | `/perception/fusion/state` · `/perception/fusion/diagnostics` · `/perception/fusion/mustika_state` | `FusionState` / `DiagnosticArray` / `UInt8` |
| **下行出口** | **`/perception/output`** | **`PerceptionFrame`** |
| 降级/健康 | `/system/degradation_state` · `/perception/health` · `/perception/alerts` | `DiagnosticArray` |
| 紧急停止 | `/perception/emergency_stop` | `Bool` |
| 里程计 | `/aft_mapped_to_init` | `Odometry` |
| 仿真真值 | `/ground_truth/odom` · `/clock` | — |

### 消息（`msg/`，共 16 个）

`PerceptionFrame`（最终输出）· `LidarResult` · `CameraFrame` · `CameraSkyBlock` · `CameraMustika` · `CameraPillar` · `CameraBuildingSpot` · `CameraDepth` · `PreprocessedImage` · `SlotStatus` · `BuildingSpot` · `Obstacle` · `PillarStatus` · `Track` · `FusionState` · `MasterCommand`

> `msg/MustikaStatus.msg` **已删除**（状态内嵌进 `PerceptionFrame.mustika_state`）。

---

## 6. 降级与保护策略（核心设计）

> 贯穿原则：**不能静默失效**；某一路死了要**如实降级并点名**，不拖垮别的路。
> **"没量到" ≠ "量到是空的"**；**"未知"不许用 0 冒充**（0 在协议里是合法值，会被当成真实断言）。

### 6.1 编排层（`perception_pipeline`）— 降级状态机

- **按路独立判活**：`lidar / camera / fusion / odom / comm` 五路，各自 `NEVER / OK / STALE / FAILED`。
- **超时按该路自己的周期给**（不能一律同一个值，否则必然误报）：

  | 路 | 超时 | 依据 |
  |---|---|---|
  | lidar | 0.5 s | 10 Hz → 约 5 帧 |
  | camera | 0.5 s | 30 Hz，取齐雷达便于现场记 |
  | fusion | 2.0 s | 模块 21 未完成，现恒 NEVER |
  | odom | 0.5 s | 实际判定在 `fusion_params.odometry.timeout_sec` |
  | comm | 3.0 s | 诊断周期 1 s → 3 个周期，避免抖动误报 |

- **"从未见过"不算故障**：启动初期与未接入模块都是 `NEVER`，**只作信息列出，不报 WARN/ERROR**。
- **全部断连** → `kEmergency` → 发 `/perception/emergency_stop`（2 s 重复一次，防下游中途才连上）。
- **降级可视化**：`/system/degradation_state`（1 Hz，给人看），逐路状态 + `pose_available` + **按原因分开**的组帧失败计数。
- **事件补帧**：障碍数变化 / 近距障碍进入或移动 >0.2 m / **建筑位状态翻转** / **柱顶有无变化** / 降级模式翻转 → 立即补发一帧，最小间隔 0.05 s（上限 20 Hz）。
- **延迟阈值**：warn 56 ms、error 80 ms（端到端预算）。

### 6.2 传感器级降级

| 故障 | 表现 | 策略 |
|---|---|---|
| 相机断（一路或全部） | 视觉状态/判色不可得 | `bringup_lidar_only` 起**仅雷达链路**；`top_color` 恒 `UNKNOWN`，状态仍由雷达给 |
| 雷达断 | 建筑位/塔高不可得 | 相机仍可判色，但**无锚点**；主控应停塔相关动作 |
| 两路相机判色**结论相反** | — | **保守报未知**（不二选一）——错色会把塔归属判反 |
| 判色数据过期 | 超过 `sky_block_timeout_sec = 0.5 s` | 不采信（塔会被对手翻，旧色≠现状） |
| 判色跟踪态非 `CONFIRMED` | `COASTING` | **不采信**（不能用预测态翻状态） |
| 判色几何不可见 | 掠射：顶面投影 <6 px 或相机在顶面下方 ≥89° | 报 `GRAZING_ANGLE` + 动作建议（换位姿/靠近），**不判必错的颜色** |
| 判色采到腰线以下 | 相机距塔面 +0.70~0.80 m | **会采到下半 = 反色（判反）** ⇒ 门控拦（≥+0.90 看顶面；+0.80~+0.90 看腰线以上=顶色） |
| D435i 失效 | 判色**唯一可见通路**（车体相机在塔顶之下，物理上看不见） | **单点故障**：那 40/80 分没了；判色帧须在臂近静止时取（卷帘形变会挪 ROI 落点） |

### 6.3 定位/里程计保护（`coordinate_transformer`）

| 状态 | 判据 | 后果 |
|---|---|---|
| `OK` | — | `odometry_reliable = true` |
| `DEGENERATE` | Point-LIO 退化标志 | `odometry_reliable=false`，世界坐标置信度 ×`confidence_decay(0.5)` |
| `IMU_VIBRATION` | 加速度模长方差 > 阈值（默认 0.25 g²；坡道/碰撞） | 同上 |
| `TIMEOUT` | 里程计 >0.5 s 无数据 | 同上；位姿查询返回 `nullopt` |

- 退化时可选**相对定位模式**（以最后可靠位姿为局部原点、帧间增量代替绝对 world），避免 TF 整体偏移把建筑位/柱/障碍集体带偏。
- TF 查询失败/超时 → `tf_unreliable()` 置位；**换不成 `world` 的建筑位一律报 `UNKNOWN(state=4)`，绝不回落成 `EMPTY`**。
- **BR_RESET**：收到复位后 5 s 窗口内注入启动区/重试区位姿先验（里程计未就绪则拒绝注入）。

### 6.4 热管理保护（`health_monitor`）

- **降级**：CPU 温度 > **85 °C** → 关后相机推理 / 前相机分辨率降 480×288 / 暂停调试点云与图像。
- **恢复**：回落 < **75 °C 且连续 30 s** 才恢复（10 °C 死区 + 时长滞回；否则阈值附近抖动会反复开关相机管线，比一直降级更糟）。
- **温度源**：按名字选 **`k10temp`**、优先 `Tctl`；⚠️ `acpitz` 会读成 **20 °C 占位值** ⇒ 天真实现"读第一个 hwmon"会导致**热保护永不触发**。
- 告警级别 `OK/WARNING/ERROR/FATAL`（**取最坏**），指标含 CPU / 延迟 / 相机帧率 / CRC 错率。

### 6.5 通信保护（`communication/`）

- 帧头同步 + **CRC16**（参数化，默认 CCITT-FALSE）+ 帧号/长度；流式状态机处理**分包/粘包**。
- 串口 **460800 8N1**；独立收发线程 + 双缓冲队列（只发最新帧）；断开 **1 s 自动重连**；心跳保活。
- 诊断上报（含 `connected` 键）→ 编排层据此判 `comm` 路 STALE。
- `/perception/emergency_stop` 已由 `comm_node` 订阅；⚠️ **协议侧 `ERROR 0x03` / `degraded` 位尚未冻结落地**——不做这一步，急停"只是给 RViz 和日志看的"。

### 6.6 静默失效防护（工具级）

- `scripts/check_param_consumers.py`：查 `yaml 键 ↔ declare_parameter ↔ 消费方`（抓"看着配好了其实没接上"）。
- `scripts/check_topic_consumers.py`：话题发布/订阅对账（抓**孤儿话题**：发了没人订 / 有人订没人发）。
- **QoS 是兼容性硬约束、不是调参项**：`RELIABLE 订阅 ← BEST_EFFORT 发布 = 静默收不到`（不报错），反向兼容。订阅端一律取发布端实际使用的那一档。
- 参数三方一致：`yaml 键名 == declare_parameter 名 == Params 字段名`。

### 6.7 尚未落地的降级（明确列为缺口）

1. **急停落线**：`comm_node` 已订阅，但 `ERROR 0x03` / `degraded` 位未与下位机冻结 → 主控收不到"感知算不出坐标"与"链路断"的区别（两者应对相反）。
2. **"全丢"→"部分丢"**：当前一个点 TF 失败就整帧不发；协议加 `pose_valid` 位后应改为部分丢。
3. **`mustika_state` 无发布者**：`mustika_pillar_check` 的结论尚未接到 `/perception/fusion/mustika_state`，该路恒 NEVER、`mustika_state` 仍是 `0=未知`。
4. **`sanctuary_mandate_fulfilled` / `held_object_count` 不由感知提供**：前者归融合/主控，后者只有主控知道夹爪状态；**填 0 是断言不是未知**（`sanctuary=0` 会让 TR 白丢 250 分）。

---

## 7. 仿真（无硬件开发）

```bash
python3 scripts/make_gazebo_world.py      # 依 config 生成场地世界（一次即可）

ros2 launch br_perception bringup_gazebo.launch.py                 # GUI + 自动巡游
ros2 launch br_perception bringup_gazebo.launch.py gui:=false motion:=false
```

- 世界文件：`scripts/out/gazebo/worlds/br_field.world`（安装到 `share/br_perception/gazebo`）
- `sensor_rig`：非 static、带 inertial，`planar_move` 驱动（`/cmd_vel` → TF `odom→base_link` + `/ground_truth/odom`）
- 雷达点云经 `pointcloud2_to_livox_custom` 伪装成 CustomMsg → Point-LIO，**与实机同一条配置路径**
- 巡游：`scripts/sim_patrol.py`（闭环方框，评估漂移）

> ⚠️ 仿真 QoS 全为 **Best Effort**（`config/gazebo_sim.rviz` 已按此配置）；相机渲染在 VM 上较慢，`gzserver` 无图形也能跑雷达/IMU。

---

## 8. 测试

```bash
colcon test --packages-select br_perception
colcon test-result --verbose
```

- 20+ 测试文件 / **250 例** / 10 个测试目标全绿。
- 重点：协议编解码（golden 帧）、坐标变换、卡尔曼（恒速/静止收敛）、关联（**落区认领优先于就近指派**）、融合判定（圣所使命）、计分、判色核心、letterbox 反算、热管理滞回、相机槽状态判定。
- ⚠️ 多数 🟢 **只覆盖 static 纯函数**，节点本体/回调/订阅未覆盖 —— **🟢 ≠ 能跑**，详见台账。

---

## 9. 常用工具脚本（`scripts/`，19 个）

| 脚本 | 用途 |
|---|---|
| `check_param_consumers.py` | 参数对账（抓"配了没接上"） |
| `check_topic_consumers.py` | 话题对账（抓孤儿话题） |
| `fake_master.py` | 伪下位机（socat 虚拟串口对端） |
| `replay_bag_debug.py` | bag 回放体检（缺路/路径错 → 分层退出码） |
| `hsv_tuner.py` | 现场 HSV 实时调参 → 写回 `color_thresholds.yaml` |
| `calibrate_camera.py` / `calibrate_lidar_camera.py` / `calib_common.py` | 内参 / LiDAR-Camera 外参标定 |
| `make_gazebo_world.py` / `make_field_from_config.py` / `render_map*.py` | 场地世界与俯视图生成 |
| `field_model_spec.py` / `field_model_generator.py` | 场地参数单一事实源与生成器 |
| `sim_patrol.py` | 仿真巡游轨迹 |
| `analyze_stl.py` / `parse_field_assembly.py` | 网格体检 / 场地图纸解析 |
| `make_v1_7_docx.py` | 由 v1.5 生成 v1.7 任务书 |

---

## 10. 文档索引（`doc/`）

| 文档 | 内容 |
|---|---|
| **`模块进度.md`** | ★ **唯一进度事实源**：逐模块状态、每轮交付、踩坑记录 |
| `任务_v1.7.docx` | 系统方案（模块规格、协议、字段表；§15.1 已过期） |
| `odometry_tuning.md` | Point-LIO 里程计调优 |
| `容器化重构设计_20260914.md` / `容器化重构_任务清单_20260914.md` | 组件容器化重构 |
| `自主探索_多层costmap与在线规划.md` | 多层代价地图与在线规划 |
| `强化学习决策与导航方案.md` | 基于结构化状态的分层 PPO 决策与局部导航推荐方案 |
| `模型训练指南.md` | YOLO 数据集 / 训练 / 导出 ONNX |
| `待编译验证清单_20260920.md` | Ubuntu 侧编译验收清单 |
| `Robocon_2027_Rulebook_v1.0 _CN.pdf` + 四份算法综述 | 规则与选型依据 |

---

## 11. 开发约定（重要）

1. **进度只认 `doc/模块进度.md`**；改完模块必须回填状态。
2. **参数三方一致**，用 `check_param_consumers.py` 自动核对。
3. **单一事实源**：外参只在 `fusion_params.yaml`；坐标系/里程计键也只在那里；不要在 launch 里硬编码。
4. **"未知"不用 0 冒充**，协议用哨兵/`*_valid` 位。
5. **不做同帧双父**：每条 TF 边只允许一个广播者（仿真 / 实机 / D435i 动态 TF 三选一）。
6. **Windows 开发机无 ROS**：本机只做纯 C++ 静态检查 + Python/YAML 校验，**ROS 部分必须在 Ubuntu 上 `colcon build` 才算过**。
7. **提交节奏**：攒到"要接相机/雷达内外参"时总提交；改了通信层 / QoS / 话题名 / 定时器必须上真机验证。

---

## 12. 已知限制

- **外参仍是占位值**：`fusion_params.yaml` 的 `static_tf.*`、`camera_hardware.yaml` 的内外参未标定（长焦 `fx=2380` 是"与视场自洽"的估计，非实测）。
- **协议未冻结**：CRC16 多项式、波特率、`seq` 语义、`CMD_VEL` 下发通道、`ERROR 0x03` / `degraded` 位待与下位机确认。
- **`earth_block_detector`（模块 8）未实现**。
- **`fusion_node` 进行中**：BR_RESET 联动 / 共享区 dwell / `score_estimator` 挂载未完成。
- **时间同步未做**（PTP / 硬触发）——"有效感知延迟"的唯一空白。
- **判色单点故障**：车体相机在塔顶之下看不见，D435i 挂了那 40/80 分就没了。
- **仿真场地 mesh 52 万三角面**：碰撞建议用解析体替代。
- **Windows 开发机上无 ROS**：相关代码未经本机编译验证。

---

## 13. 许可

Apache-2.0（见 `package.xml`）。
