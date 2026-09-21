# -*- coding: utf-8 -*-
"""
make_gazebo_world.py — 基于 rc1_field_zup.stl 生成 Gazebo Classic 仿真世界
===========================================================================
把团队的 11×11 场地 STL(已转 Z-up)包裹成 Gazebo 静态模型, 加上:
  - 地面平面 + 光照
  - 2 根柱(穆斯蒂卡柱 500mm / 核心支柱 800mm, 均在场地中心 5.5,5.5)
  - 12 个天空方块(200mm, 按 240mm 棋盘 5×5 布局, 绝对中心占位)
  - 一个"传感器平台"(代表 BR 感知安装位): 3D 雷达 + 前后双相机 + IMU
    -> 发布到 /livox/lidar/pointcloud, /camera_wide/image_raw, /camera_tele/image_raw, /imu/data

输出:
  scripts/out/gazebo/worlds/br_field.world
  scripts/out/gazebo/models/br_field/model.config + model.sdf   (场地 mesh)
  scripts/out/gazebo/models/sensor_rig/...                        (传感器平台)
  scripts/out/gazebo/README.md

注意: 除场地 mesh 外, 方块/柱坐标取自 field_model_spec.py(占位), 绝对中心等官方坐标待图2回填。
环境: ROS2 Humble + Gazebo Classic 11 (gazebo_ros 插件)。
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import field_model_spec as S  # noqa: E402

DB = os.path.join(HERE, "out", "gazebo")
WORLD_DIR = os.path.join(DB, "worlds")
FIELD_MODEL_DIR = os.path.join(DB, "models", "br_field")
SENSOR_MODEL_DIR = os.path.join(DB, "models", "sensor_rig")
STL_SRC = os.path.join(DB, "rc1_field_zup.stl")
os.makedirs(WORLD_DIR, exist_ok=True)
os.makedirs(FIELD_MODEL_DIR, exist_ok=True)
# SENSOR_MODEL_DIR 只在 WITH_SENSOR_RIG 时创建 (见下), 否则会留下一个空目录

SCALE = 1.0  # STL 已是米制(m)

# 是否在场地模型里放置 12 个天空方块(开局真实状态)。想省事/排除方块干扰检测 -> 设 False
WITH_SKY_BLOCKS = True


def f(v):
    return ("%.3f" % v).rstrip("0").rstrip(".")


def sky_positions():
    """用确认的 240mm 棋盘, 位置来自 spec(占位)。返回 [(x,y,color)] 世界坐标(米)。"""
    out = []
    for r, c, col in S.sky_grid_positions():
        x = S.SKY_GRID_ORIGIN[0] + c * S.SKY_GRID_STEP
        y = S.SKY_GRID_ORIGIN[1] + r * S.SKY_GRID_STEP
        out.append((round(x, 3), round(y, 3), col))
    return out


def gen_field_model():
    """[已废弃 2026-09-11] 旧路线: 用团队 STL + field_model_spec 的 DRAFT 坐标建场地。
    问题: 柱子坐标写死 CORE_PILLAR_POS(两根柱都在 5.5,5.5), 方块/建筑位全是 DRAFT 占位,
          与 config/field_geometry.yaml 相差极大。
    现由 make_field_from_config.py 从 config 重建 —— 见 gen_field_model_delegated()。"""
    raise RuntimeError('gen_field_model 已废弃, 请用 make_field_from_config.gen_field()')
    sky = sky_positions()
    lines = []
    A = lines.append
    A('<?xml version="1.0"?>')
    A('<sdf version="1.6">')
    A('  <model name="br_field">')
    A('    <static>true</static>')
    A('    <link name="field_mesh">')
    A('      <visual name="vis">')
    A('        <geometry><mesh><uri>model://br_field/meshes/rc1_field_zup.stl</uri>')
    A('          <scale>%s %s %s</scale></mesh></geometry>' % (f(SCALE), f(SCALE), f(SCALE)))
    A('      </visual>')
    A('      <collision name="col">')
    A('        <geometry><mesh><uri>model://br_field/meshes/rc1_field_zup.stl</uri></mesh></geometry>')
    A('      </collision>')
    A('    </link>')
    # 柱子
    for name, z, r, h, color in (
        ("mustika_pillar", S.GROUND_Z, S.MUSTIKA_PILLAR_D / 2, S.MUSTIKA_PILLAR_H, "0.39 0.24 0.0"),
        ("core_pillar", S.L2_HEIGHT, S.CORE_PILLAR_D / 2, S.CORE_PILLAR_H, "0.39 0.24 0.0"),
    ):
        A('    <link name="%s">' % name)
        A('      <pose>%s %s %s 0 0 0</pose>' % (f(S.CORE_PILLAR_POS[0]), f(S.CORE_PILLAR_POS[1]),
                                                  f(z + h / 2)))
        A('      <visual name="v"><geometry><cylinder><radius>%s</radius><height>%s</height></cylinder></geometry>'
          '<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>' % (f(r), f(h), color, color))
        A('      <collision name="c"><geometry><cylinder><radius>%s</radius><height>%s</height></cylinder></geometry></collision>'
          % (f(r), f(h)))
        A('    </link>')
    # 天空方块(可由 WITH_SKY_BLOCKS 开关)
    if WITH_SKY_BLOCKS:
        for i, (x, y, col) in enumerate(sky):
            ccol = "0.87 0.13 0.13" if col == 1 else "0.2 0.0 1.0"
            A('    <link name="sky_%02d">' % i)
            A('      <pose>%s %s %s 0 0 0</pose>' % (f(x), f(y), f(0.2 / 2)))
            A('      <visual name="v"><geometry><box><size>0.2 0.2 0.2</size></box></geometry>'
              '<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>' % (ccol, ccol))
            A('      <collision name="c"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></collision>')
            A('    </link>')
    A('  </model>')
    A('</sdf>')
    model_sdf = "\n".join(lines)

    with open(os.path.join(FIELD_MODEL_DIR, "model.sdf"), "w", encoding="utf-8") as fd:
        fd.write(model_sdf + "\n")
    with open(os.path.join(FIELD_MODEL_DIR, "model.config"), "w", encoding="utf-8") as fd:
        fd.write('<?xml version="1.0"?>\n<model>\n  <name>br_field</name>\n'
                 '  <version>1.0</version>\n  <sdf version="1.6">model.sdf</sdf>\n'
                 '  <author><name>br_perception</name></author>\n  <description>ROBOCON 2027 field (team STL)</description>\n</model>\n')
    # 把 STL 拷进模型目录
    meshes = os.path.join(FIELD_MODEL_DIR, "meshes")
    os.makedirs(meshes, exist_ok=True)
    with open(STL_SRC, "rb") as srcf:
        with open(os.path.join(meshes, "rc1_field_zup.stl"), "wb") as dtf:
            dtf.write(srcf.read())
    return model_sdf


# 传感器平台: 用户 2026-09-11 明确 **没用, 删掉**, 2026-09-17 再次确认删掉 ——
# 默认不生成也不放进世界。世界里那根"接地的柱"就是它的机身 (0.5×0.5×1.2 from z=0)。
# 代价: 仿真里没有 /livox/lidar/pointcloud + /camera_wide|tele/image_raw + /imu/data,
#       launch/bringup_gazebo.launch.py 与 scripts/sim_patrol.py 会收不到数据。
# 想恢复(比如需要仿真里有雷达/相机话题)就设 True, 会重新生成 models/sensor_rig。
WITH_SENSOR_RIG = False

# 整台平台的摆放位姿。z=1.3 是传感器头的世界高度, 机身从地面长上来 (见 gen_sensor_rig)
SENSOR_RIG_POSE = {"x": 5.5, "y": 2.0, "z": 1.3, "yaw": 0.0}


def gen_sensor_rig():
    os.makedirs(SENSOR_MODEL_DIR, exist_ok=True)
    lines = []
    A = lines.append
    A('<?xml version="1.0"?>')
    A('<sdf version="1.6">')
    A('  <model name="sensor_rig">')
    A('    <static>true</static>')
    # 机身: 从地面(局 z=-1.3, 因为整个 model 摆在 z=1.3) 一直长到传感器头, 否则
    # 整台平台会飘在半空 —— 就是用户说的"悬空的杂物"
    A('    <link name="body">')
    A('      <pose>0 0 -0.7 0 0 0</pose>')
    A('      <visual name="v"><geometry><box><size>0.5 0.5 1.2</size></box></geometry>'
      '<material><ambient>0.35 0.35 0.38 1</ambient><diffuse>0.35 0.35 0.38 1</diffuse></material></visual>')
    A('      <collision name="c"><geometry><box><size>0.5 0.5 1.2</size></box></geometry></collision>')
    A('    </link>')
    A('    <link name="base">')
    A('      <pose>0 0 0 0 0 0</pose>')
    A('      <visual name="v"><geometry><box><size>0.5 0.5 0.2</size></box></geometry>'
      '<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>')
    A('      <collision name="c"><geometry><box><size>0.5 0.5 0.2</size></box></geometry></collision>')
    A('    </link>')
    # 3D 雷达 (GPU ray)
    A('    <link name="lidar">')
    A('      <pose>0 0 0.4 0 0 0</pose>')
    # 用 CPU <ray> 而非 <gpu_ray>: Gazebo Classic 的 gpu_ray **水平 FOV 上限 180°**
    # (GpuRaySensor.cc:192 会警告并截断), 而 Mid-360 是 360°. CPU ray 支持全 360°,
    # 静态场地 + 360×16 @10Hz 的开销可以接受.
    A('      <sensor name="lidar" type="ray">')
    A('        <always_on>true</always_on><update_rate>10</update_rate>')
    A('        <ray>')
    A('          <scan><horizontal><samples>360</samples><min_angle>-3.1416</min_angle><max_angle>3.1416</max_angle></horizontal>'
      '<vertical><samples>16</samples><min_angle>-0.26</min_angle><max_angle>0.26</max_angle></vertical></scan>')
    A('          <range><min>0.05</min><max>15.0</max><resolution>0.01</resolution></range>')
    A('        </ray>')
    A('        <plugin name="gazebo_ros_ray_sensor" filename="libgazebo_ros_ray_sensor.so">')
    A('          <ros><remapping>~/out:=/livox/lidar/pointcloud</remapping></ros>')
    A('          <output_type>sensor_msgs/PointCloud2</output_type>')
    A('          <frame_name>livox_frame</frame_name>')
    A('        </plugin>')
    A('      </sensor>')
    A('    </link>')
    # 宽角 + 长焦 —— ✅ 2026-09-20 按实际装机改: 原来这里是"前/后"两颗(yaw 0 与 π),
    #   但实机是 **JR H200S 一宽一长焦, 都朝前**(见任务书 §0 传感器行)。
    #   仿真用不同 horizontal_fov 体现"一宽一长焦", 朝向都是 +x。
    for name, hfov, topic in (("cam_wide", 1.10, "/camera_wide/image_raw"),
                              ("cam_tele", 0.40, "/camera_tele/image_raw")):
        A('    <link name="%s">' % name)
        A('      <pose>0 0 0.4 0 0 0</pose>')          # 都朝 +x
        A('      <sensor name="%s_s" type="camera">' % name)
        A('        <always_on>true</always_on><update_rate>30</update_rate>')
        A('        <camera><horizontal_fov>%s</horizontal_fov>' % hfov)
        # Gazebo Classic 的 <format> 只认 R8G8B8 / B8G8R8 / L8 等; 写 RGB8 会报
        # "Error parsing image format" 并回落. gazebo_ros_camera 输出 BGR8,
        # 与 camera_params.yaml 的 pixel_format: "BGR8" 一致, 无需再改.
        A('          <image><width>640</width><height>384</height><format>R8G8B8</format></image>')
        A('          <clip><near>0.05</near><far>30.0</far></clip></camera>')
        # 插件名必须唯一, 否则报 "Found multiple nodes with same name: /gazebo_ros_camera"
        A('        <plugin name="gazebo_ros_camera_%s" filename="libgazebo_ros_camera.so">' % name)
        A('          <ros><remapping>~/image_raw:=%s</remapping></ros>' % topic)
        A('        </plugin>')
        A('      </sensor>')
        A('    </link>')
    # IMU
    A('    <link name="imu">')
    A('      <pose>0 0 0.4 0 0 0</pose>')
    A('      <sensor name="imu_s" type="imu">')
    A('        <always_on>true</always_on><update_rate>100</update_rate>')
    # ROS 2 (Humble) 的 IMU 插件叫 libgazebo_ros_imu_sensor.so
    # (libgazebo_ros_imu.so 是 ROS 1 的名字, 会报 cannot open shared object file);
    # 其默认输出话题是 ~/out 而不是 ~/data.
    A('        <plugin name="gazebo_ros_imu" filename="libgazebo_ros_imu_sensor.so">')
    A('          <ros><remapping>~/out:=/imu/data</remapping></ros>')
    A('        </plugin>')
    A('      </sensor>')
    A('    </link>')
    A('  </model>')
    A('</sdf>')
    sdf = "\n".join(lines)
    with open(os.path.join(SENSOR_MODEL_DIR, "model.sdf"), "w", encoding="utf-8") as fd:
        fd.write(sdf + "\n")
    with open(os.path.join(SENSOR_MODEL_DIR, "model.config"), "w", encoding="utf-8") as fd:
        fd.write('<?xml version="1.0"?>\n<model>\n  <name>sensor_rig</name>\n  <version>1.0</version>\n'
                 '  <sdf version="1.6">model.sdf</sdf>\n  <author>br_perception</author>\n'
                 '  <description>BR perception sensor rig (lidar+cameras+imu)</description>\n</model>\n')
    return sdf


def gen_world():
    sensor = SENSOR_RIG_POSE
    lines = []
    A = lines.append
    A('<?xml version="1.0"?>')
    A('<sdf version="1.6">')
    A('  <world name="br_field">')
    A('    <physics type="ode"><max_step_size>0.005</max_step_size></physics>')
    A('    <light type="directional" name="sun"><pose>0 0 6 0 0 0</pose>'
      '<diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>'
      '<direction>-0.5 0.2 -1</direction></light>')
    A('    <model name="ground">')
    A('      <static>true</static>')
    A('      <link name="ground_link"><visual name="v"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>'
      '<material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>'
      '<collision name="c"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision></link>')
    A('    </model>')
    A('    <include><uri>model://br_field</uri></include>')
    if WITH_SENSOR_RIG:
        A('    <include>')
        A('      <uri>model://sensor_rig</uri>')
        A('      <pose>%s %s %s 0 0 %s</pose>' % (sensor["x"], sensor["y"], sensor["z"], sensor["yaw"]))
        A('    </include>')
    A('  </world>')
    A('</sdf>')
    world = "\n".join(lines)
    with open(os.path.join(WORLD_DIR, "br_field.world"), "w", encoding="utf-8") as fd:
        fd.write(world + "\n")
    return world


def gen_readme():
    txt = f"""# Gazebo 仿真世界 (BR 感知)

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
| 宽角(30Hz) | `/camera_wide/image_raw` |
| 长焦(30Hz) | `/camera_tele/image_raw` |
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
"""
    with open(os.path.join(DB, "README.md"), "w", encoding="utf-8") as fd:
        fd.write(txt)
    return txt


def main():
    # 场地与世界一律交给 make_field_from_config (数据源 = config/field_geometry.yaml)
    import make_field_from_config as F
    F.WITH_SENSOR_RIG = WITH_SENSOR_RIG      # world 由 F.gen_world() 生成, 开关必须传过去
    F.SENSOR_RIG_POSE = (SENSOR_RIG_POSE["x"], SENSOR_RIG_POSE["y"],
                         SENSOR_RIG_POSE["z"], SENSOR_RIG_POSE["yaw"])
    F.gen_field()
    F.gen_world()
    if WITH_SENSOR_RIG:
        gen_sensor_rig()
    gen_readme()
    print("wrote:")
    for p in (os.path.join(FIELD_MODEL_DIR, "model.sdf"),
              os.path.join(SENSOR_MODEL_DIR, "model.sdf"),
              os.path.join(WORLD_DIR, "br_field.world"),
              os.path.join(DB, "README.md")):
        print("  " + p)


if __name__ == "__main__":
    main()
