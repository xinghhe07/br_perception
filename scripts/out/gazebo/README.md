# Gazebo 仿真世界 (BR 感知)

位置: `scripts/out/gazebo/worlds/br_field.world`
依赖: **ROS2 Humble + Gazebo Classic 11** (`gazebo_ros` 插件集).
      若你用 Ignition/Garden, 传感器插件需换成 `gz-sim` 系列, 本文件未适配。

## 启动

```bash
# 把 out/gazebo/models 加入 Gazebo 模型路径
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:.../scripts/out/gazebo/models

gazebo --verbose .../scripts/out/gazebo/worlds/br_field.world
# 或(带 gzserver + 节点)
```

## 传感器话题
| 传感器 | 话题 |
|---|---|
| 3D 雷达(GPU ray, 10Hz) | `/livox/lidar/pointcloud` |
| 宽角相机(30Hz, hfov 1.10) | `/camera_wide/image_raw` |
| 长焦相机(30Hz, hfov 0.40) | `/camera_tele/image_raw` |
| IMU(100Hz) | `/imu/data` |

这些话题名与 `src/lidar/lidar_preprocessor.cpp` 订阅的 `/livox/lidar/pointcloud`、
`src/camera/*` 订阅的 `/camera_*_/image_raw` 一致, 可直接跑感知管线(无需真机)。
*/livox/lidar/pointcloud 是雷达原始话题, 仿真里用 GPU ray 近似 Mid-360 的 360°×16 线扫描。

## 组成
- `models/br_field/` — 11×11 m 场地, **全部由 `config/field_geometry.yaml` 生成**
  (见 `scripts/make_field_from_config.py`): 地面/双色地面板、L1+L2 平台、双侧楼梯(3 级)+双侧 3.5m 坡道
  (楔形网格)、双侧转运区、外圈围栏、中轴隔墙(南北留口)、L1 周界屏障(转运区处留口)、
  两根柱(穆斯蒂卡柱在 5.5,9.75 / 核心柱在 L2 中央)、10 个建筑位(含 L2 的 4 个, per-spot 高度)、
  12 个 200mm 天空方块、各区域色板。
- ~~`models/sensor_rig/`~~ — 已于 2026-09-11 按用户要求**删除**("他没有用")。
  现在世界里没有雷达/相机/IMU 话题; 需要时把 `make_gazebo_world.py` 的
  `WITH_SENSOR_RIG` 设回 `True` 重新生成。

## 重新生成
```bash
python scripts/make_field_from_config.py   # 只重生成场地+世界 (数据源 = config/field_geometry.yaml)
python scripts/make_gazebo_world.py        # 场地+世界(委托上面那个) + 传感器平台 + README
```

## 已知限制 / 待办
- **`rc1_field_zup.stl` 已不再使用**(那是旧 spec 造的, 几何与现状不符), 保留在 meshes/ 仅为存档。
- 天空方块的**色序**仍是按规则 4.1.4 推出的先验(几何已确认) —— 见 config 文末 PENDING。
- 柱顶凹槽 (Ø180 深 100) 未建模, 用整根圆柱近似; 若要做"安放穆斯蒂卡"的仿真需补。
- 场地是**静态碰撞**; 方块也可改为可抓取(dynamic+FP), 需另配夹爪模型。
- `gpu_ray` 是普通线束雷达近似, 非 Livox 非重复扫描; 若要更接近可换 `livox_ros2` 插件(不常见)。
