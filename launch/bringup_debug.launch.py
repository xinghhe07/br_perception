#!/usr/bin/env python3
"""
相机层端到端联调 — ROBOCON 2027 BR 感知系统 (任务书 §10.4, P0-6)

**第一次端到端联调只能靠它** —— 在此之前相机层 6 个节点从来没有一起跑过。

启动内容（全部相机层节点）:
  1. image_preprocessor      双相机预处理 + letterbox
  2. mustika_detector        穆斯蒂卡（Mikasa V300W 排球）
  3. pillar_detector         深绿穆斯蒂卡柱 + 棕色核心支柱
  4. building_spot_detector  建筑位绿色方格
  5. camera_depth            D435i 深度辅助（独立回调，不阻塞主链）
  6. camera_perception       聚合薄壳 → /perception/camera/frame   ← P0-2 新增

⚠️ 本 launch **不起工业相机驱动** —— 因为仓库里还没有：
   /camera_wide/image_raw 与 /camera_tele/image_raw 这两路的生产者目前是 **Gazebo**
   （scripts/make_gazebo_world.py:180）；实机驱动还没接进来
   （模块进度.md 的 P2-14「卡接口确认: GigE / USB3 / 厂商 SDK?」）。
   → 实机联调时，那两路话题得**另外起**（厂商 SDK，或 ros2 bag play）。
     本 launch 只负责相机层节点本身，不假装知道工业相机怎么起。

D435i 是**可选**的（默认不起）—— 它在本层只承担**深度**一路。

前置:
  source /opt/ros/humble/setup.bash
  source ~/point_ws/install/setup.bash          # 相机层用不到, 但整个包的 CMake 需要它
  source ~/br_perception/install/setup.bash

用法:
  # 主要场景: 相机层 + 另起的图像源（bag 回放 / 仿真 / 厂商 SDK）
  ros2 launch br_perception bringup_debug.launch.py

  # 连 D435i 一起起
  ros2 launch br_perception bringup_debug.launch.py realsense:=true

  # 进程级隔离（联调期定位崩溃/卡死时用, 代价是图像走序列化）
  ros2 launch br_perception bringup_debug.launch.py intra_process:=false

  # 看结果
  ros2 topic hz   /perception/camera/frame
  ros2 topic echo /perception/camera/frame --once

⚠️ 首次联调**必看的三件事**（否则很容易白折腾半天）:
   ① 深度槽永远是 IDLE —— camera_depth 的设计是"没收到 ROI 就不测量"，
      而 ROI 话题 /camera/depth/target_roi **在本 launch 里没有任何发布者**。
      所以首次联调看到深度无测量值是**正常的**，不是节点坏了。要测 MEASURE 通路:
        ros2 topic pub --once /camera/depth/target_roi sensor_msgs/msg/RegionOfInterest \
          "{x_offset: 600, y_offset: 300, width: 80, height: 80, do_rectify: false}"
   ② 六路检测话题任何一路没人发，聚合帧**照发**（这是设计 —— 见口径③），
      但对应槽的 slot_status[].state 会是 NEVER。**看槽状态，别看帧有没有到**。
   ③ RViz 里若什么都看不见，先确认用的是本文件的 config/camera_debug.rviz ——
      相机话题是 BEST_EFFORT，而 RViz 默认 Reliable，**不兼容就完全不显示**。

⚠️ 本文件**没有在本机验证过**（本机无 ROS）—— 按 doc/待编译验证清单_20260920.md 核对。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
# ⚠️ 2026-09-25 新增：**double 数组参数必须显式标类型** —— 见 `_sim_intrinsics_params()`。
from launch_ros.parameter_descriptions import ParameterValue
from rcl_interfaces.msg import ParameterType


def _resolve_launch(share_dir, launch_file):
    """在 launch/ 与 launch_ROS2/ 下找 launch 文件（livox 用后者，保持同款容错）"""
    for sub in ('launch', 'launch_ROS2', ''):
        candidate = os.path.join(share_dir, sub, launch_file) if sub \
            else os.path.join(share_dir, launch_file)
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(f"在 {share_dir} 下找不到 {launch_file}")


def _include(pkg_name, launch_file, launch_arguments=None, source_hint="", condition=None):
    """外部包 launch 的安全 include — 包不存在时给出可读的中文提示"""
    try:
        share = get_package_share_directory(pkg_name)
    except Exception as exc:  # PackageNotFoundError
        raise RuntimeError(
            f"找不到包 '{pkg_name}' — 请先 source 对应工作区。{source_hint}"
        ) from exc
    kwargs = {'condition': condition} if condition is not None else {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_resolve_launch(share, launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
        **kwargs,
    )


# ═══════════════════════════════════════════════════════════════════════════════
# 相机层的 6 个节点 —— **一处定义，两种跑法**（容器内 / 各自独立进程）
# ═══════════════════════════════════════════════════════════════════════════════
# 每项 = (节点名, plugin 全名, 可执行名, 是否用颜色阈值)
# ⚠️ 节点名必须与 config/camera_params.yaml 的参数块名**逐字一致** ——
#    否则参数匹配不上，节点静默用代码默认值（2026-09-20 踩过的坑）。
NODE_SPECS = [
    # name,                     plugin,                                            exe,                      uses_color
    ('image_preprocessor',      'br_perception::camera::ImagePreprocessor',        'image_preprocessor',     False),
    ('mustika_detector',        'br_perception::camera::MustikaDetector',          'mustika_detector',       True),
    ('pillar_detector',         'br_perception::camera::PillarDetector',           'pillar_detector',        True),
    ('building_spot_detector',  'br_perception::camera::BuildingSpotDetector',     'building_spot_detector', True),
    # ⚠️ 2026-09-23 新增（模块 9）。**用颜色** ⇒ uses_color=True，
    #    launch 会把 color_yaml_path 覆盖成 install 空间的**绝对路径**
    #    （`camera_params.yaml` 里写的是相对 $PWD 的字符串，不覆盖就找不到阈值文件）。
    #    ⚠️ 它**只做判色、不做检测**：得分场景的 ROI 走**雷达建筑位投影**
    #       （2026-09-24 落地）—— 但那条路要求 launch 额外给两样东西，本函数已按需补上：
    #         · `field_geometry.yaml`（建筑位坐标/平台高）
    #         · `sim_camera:=true` 时的**仿真相机内参**（实机内参是待标定的占位值）
    ('sky_block_detector',      'br_perception::camera::SkyBlockDetector',         'sky_block_detector',     True),
    ('camera_depth',            'br_perception::camera::CameraDepthNode',          'camera_depth',           False),
    ('camera_perception',       'br_perception::camera::CameraPerceptionNode',     'camera_perception_node', False),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('br_perception')
    camera_params = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    # ⚠️⚠️ 颜色阈值文件必须给**绝对路径**。config/camera_params.yaml 里写的是
    #     相对路径 "config/color_thresholds.yaml"，而 color_utils.cpp:89 是
    #     `std::ifstream(yaml_path)` —— **相对启动时的 $PWD**，且节点里没有任何
    #     回退解析。launch 可以从任意目录起 → 检测器会找不到颜色文件
    #     （load_color_thresholds 抛异常）。所以这里按 install 空间解析成绝对路径覆盖。
    #     ⚠️ **只有 3 个节点用颜色**（mustika / pillar / building_spot）——
    #        见 NODE_SPECS 的 uses_color 列。给另外 3 个传是无用的未声明覆盖。
    color_yaml = os.path.join(pkg_share, 'config', 'color_thresholds.yaml')

    # ── launch 参数 ──
    intra_arg = DeclareLaunchArgument(
        'intra_process', default_value='true', choices=['true', 'false'],
        description='true = 6 个节点装进一个容器（图像零拷贝）; '
                    'false = 拆成 6 个独立进程（真隔离，代价是图像走序列化）')
    rs_arg = DeclareLaunchArgument(
        'realsense', default_value='false', choices=['true', 'false'],
        description='启动 D435i 驱动。默认 false —— 首次联调通常没有 D435i，'
                    '默认开着会刷一屏驱动错误日志')
    perception_rviz_arg = DeclareLaunchArgument(
        'perception_rviz', default_value='true', choices=['true', 'false'],
        description='启动 RViz2')
    # ⚠️ 这个 launch 参数与本文件里的**节点参数**同名，是有意的 ——
    #    这里不会冲突，因为**没有任何 include 会转发它**（本文件的 include 只有
    #    realsense 那一条，且只显式透传了本参数，不转发整个 launch context）。
    #    ⚠️ bringup_full.launch.py 踩过同类坑（rviz 参数被 include 污染），
    #       将来若在本文件里加 include，**先确认它不会把 use_sim_time 吃掉**。
    sim_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'],
        description='用仿真时钟（接 gazebo/bag 回放时设 true）')

    # ⚠️⚠️ `sim_camera` 与 `use_sim_time` **是两件事**，别合并成一个开关：
    #    · `use_sim_time` = 时间戳来源（bag 回放也要开）
    #    · `sim_camera`   = **图像来自 Gazebo 那台 640×384 的虚拟相机** ⇒ 内参要按
    #      SDF 的 `<horizontal_fov>` 覆盖（实机是 1280×720 + 待标定内参）
    #    合并的话，"回放实拍 bag"会被当成仿真而套上错的内参 —— 而**投影错**这件事
    #    在看图像时完全看不出来（框照样画在画面上）。
    sim_camera_arg = DeclareLaunchArgument(
        'sim_camera', default_value='false', choices=['true', 'false'],
        description='图像来自 Gazebo 虚拟相机 ⇒ 用 SDF 的 hfov 覆盖内参 + 戴上 '
                    'field_geometry.yaml（建筑位坐标）。⚠️ 接实机/回放实拍时必须 false')
    # ⚠️ **这里刻意不取 `sim_camera = LaunchConfiguration(...)`**（2026-09-25 删）。
    #    取了它就会有人写成 `if sim_camera:` —— 而那**恒为真**（见 `make_params` 的 ⚠️），
    #    后果是"仿真内参被无条件用到实机上"且 launch 直接崩。
    #    正确读法只有一种：在 `_build_camera_layer(context)` 里
    #    `context.launch_configurations.get('sim_camera', ...)`（执行期，拿到的是字符串）。

    intra = LaunchConfiguration('intra_process')
    use_sim = LaunchConfiguration('use_sim_time')

    # ═══════════════════════════════════════════════════════════════════════════
    # 仿真相机内参 —— ⚠️ **数值由 SDF 推导, 不是抄的**, 推导过程留在这里
    # ═══════════════════════════════════════════════════════════════════════════
    #   `models/fishbot/model.sdf`（仓库里那份在 scripts/out/gazebo/ 下）：
    #     cam_wide_s: <horizontal_fov>1.1</horizontal_fov>  <image>640×384
    #     cam_tele_s: <horizontal_fov>0.40</horizontal_fov> <image>640×384
    #   Gazebo 的相机模型：fx = fy = (宽/2) / tan(hfov/2), cx = 宽/2, cy = 高/2
    #     wide: 320 / tan(0.55) = 521.9     tele: 320 / tan(0.20) = 1578.6
    #   ⚠️ 宽高比 640/384 ≠ 16:9（Gazebo 用 hfov 定 x 方向，y 由像素比推）
    #      ⇒ **别拿"1280×720 的内参除 2"** 去凑（那套的视场是实机镜头的, 不是这台仿真相机的）。
    #   ⚠️ 改 SDF 的 fov 或分辨率 ⇒ **这两个数组要一起重算**。
    SIM_INTRINSICS = {
        'fx': [521.9, 1578.6], 'fy': [521.9, 1578.6],
        'cx': [320.0, 320.0],  'cy': [192.0, 192.0],
        # letterbox **之前**那张图的尺寸（= 仿真相机分辨率）
        'orig_w': 640, 'orig_h': 384,
    }
    # 建筑位坐标 / 平台高 / 塔高 / 方块边长 —— **事实源是 field_geometry.yaml 第 11 节**
    # （与 `lidar_perception_node` 读的是同一组键；那份文件的参数块是 `/**` 通配,
    #  所以传给任何节点都能匹配上）。
    field_geometry = os.path.join(pkg_share, 'config', 'field_geometry.yaml')

    def _sim_intrinsics_params():
        """仿真相机内参 → 可直接交给节点的参数字典。

        ⚠️⚠️ **float 数组必须包 `ParameterValue` 并显式标 `DOUBLE_ARRAY`（2026-09-25 修）。**
        直接传 Python 的 `[521.9, 1578.6]` 会被 launch 当成**替换列表**去解析
        （`normalize_to_list_of_substitutions` 只认 str / Substitution）⇒ 抛
            `TypeError: Failed to normalize given item of type '<class 'float'>'`
        ⇒ **整个 launch 起不来**。int 的 `orig_w` / `orig_h` 不需要包（它们是标量，
        走的是另一条分支）—— 原注释说"包了会变成字符串"对**标量**成立，对**数组**不成立。
        """
        return {
            'fx': ParameterValue(SIM_INTRINSICS['fx'],
                                 value_type=ParameterType.PARAMETER_DOUBLE_ARRAY),
            'fy': ParameterValue(SIM_INTRINSICS['fy'],
                                 value_type=ParameterType.PARAMETER_DOUBLE_ARRAY),
            'cx': ParameterValue(SIM_INTRINSICS['cx'],
                                 value_type=ParameterType.PARAMETER_DOUBLE_ARRAY),
            'cy': ParameterValue(SIM_INTRINSICS['cy'],
                                 value_type=ParameterType.PARAMETER_DOUBLE_ARRAY),
            # letterbox **之前**那张图的尺寸（= 仿真相机分辨率）
            'orig_w': SIM_INTRINSICS['orig_w'],
            'orig_h': SIM_INTRINSICS['orig_h'],
        }

    def make_params(name, uses_color, sim_camera_on):
        """一个节点的参数列表。⚠️ 顺序有意义: 后面的**覆盖**前面的。

        ⚠️ `sim_camera_on` 是**已求值的 bool**，不是 `LaunchConfiguration` ——
           必须在 `OpaqueFunction` 里用 `context.launch_configurations` 取。
           原实现写的是 `if sim_camera:`，而 `sim_camera` 是 `LaunchConfiguration`
           对象 ⇒ **恒为真**（Python 里没有 `__bool__` 的对象都是真）⇒
           **仿真相机内参被无条件写进 sky_block_detector**。
           后果不是崩溃而是**静默错参**：接实机跑时用的还是 Gazebo 那台
           640×384 / fx=521.9 的内参，投影出来的 ROI 全错 ——
           而那正是 40/80 分判色链的输入。见 §八。
        """
        ps = [camera_params,
              # ⚠️ use_sim_time 不是节点自己的参数，但每个节点都得有一份，
              #    否则仿真下各节点时间戳不同源 —— 聚合薄壳的 stale_sec 全靠
              #    header.stamp 相减，不同源会算出天文数字甚至抛异常。
              {'use_sim_time': use_sim}]
        if uses_color:
            ps.append({'color_yaml_path': color_yaml})
        if name == 'sky_block_detector':
            # ⚠️ 投影主路要**建筑位坐标**（在 field_geometry.yaml 里）——
            #    漏传 ⇒ 三个数组为空 ⇒ 主路关闭（节点会 WARN 点名, 不会静默）
            ps.append(field_geometry)
            if sim_camera_on:
                ps.append(_sim_intrinsics_params())
        return ps

    # ── 跑法一: 单容器（默认）──
    # ⚠️ 相机层节点之间传的是**图像**（640×384×3 ≈ 700KB/帧 × 30Hz）。
    #    容器内启用 intra-process 后这些按指针传递、不再序列化 ——
    #    这正是"薄壳聚合比单体多一跳"那个代价的**具体消除手段**。
    #
    # ⚠️ 启用 intra-process 的**生命周期后果**（写在这里，将来换 loaned message 时要用）:
    #    消息按指针传递 → 下游持有 SharedPtr 就会**延长上游图像的生命周期**。
    #    聚合薄壳的 SlotData 正是长期持有（存到下一帧来为止）。
    #    将来若改用 **loaned message**（零拷贝的更进一步），**跨回调持有是禁止的** ——
    #    那时聚合薄壳必须改成"组帧时拷贝"，不能存指针。
    # ═══════════════════════════════════════════════════════════════════════════
    # 节点构造搬到**执行期**（OpaqueFunction）—— 这不是风格选择，是必需：
    #   `sim_camera` / `intra_process` 都是 `LaunchConfiguration`，
    #   **生成期它们只是对象、不是值**。原实现在生成期写 `if sim_camera:`，
    #   那**恒为真**（Python 里没定义 `__bool__` 的对象都判真）⇒
    #   仿真相机内参被**无条件**塞进 sky_block_detector，且 **launch 直接崩**
    #   （float 数组不能当字面量参数传 —— 见 `_sim_intrinsics_params`）。
    #   ⇒ 两个症状、一个根因：**在生成期读了一个只有执行期才有值的量**。
    # ⚠️ 两种跑法**节点、参数、名字完全相同**，只有部署拓扑不同：
    #    这样在"隔离模式"里复现的问题，容器模式下也一定复现（反之亦然）。
    # ═══════════════════════════════════════════════════════════════════════════
    def _build_camera_layer(context):
        def _flag(key, default):
            return context.launch_configurations.get(key, default).lower() \
                in ('true', '1', 'yes')

        sim_camera_on = _flag('sim_camera', 'false')
        want_standalone = not _flag('intra_process', 'true')

        if want_standalone:
            # ── 跑法二: 6 个独立进程 ──
            # ⚠️ 这才是**真正的进程级隔离**：一个节点段错误只死自己，不带走整层。
            #    （容器跑法下 6 个节点同进程，一个崩全崩 —— 这是取舍，不是 bug。）
            #    代价: 节点之间的图像走 DDS 序列化/反序列化。
            return [
                Node(package='br_perception', executable=exe, name=name,
                     output='screen',
                     parameters=make_params(name, uses_color, sim_camera_on))
                for name, _plugin, exe, uses_color in NODE_SPECS
            ]

        # ── 跑法一: 单容器（默认）──
        return [_build_container(sim_camera_on)]

    def _build_container(sim_camera_on):
        return ComposableNodeContainer(
            name='camera_layer_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',   # mt = 多线程执行器
            # ⚠️ 用多线程的理由: **节点之间**并行 —— 三个检测器各跑各的 OpenCV，
            #    深度走独立回调（camera_depth_node.hpp 的硬约束是"独立回调，
            #    不阻塞主感知链"）。单线程会让它们互相排队，直接违反那条约束。
            #    ⚠️ **但多线程 ≠ 节点内并行**: 同一节点里 wide/tele 两个回调默认同属
            #       一个 MutuallyExclusive callback group，**仍然串行**。
            #       若将来发现 image_preprocessor 的宽/长焦成了瓶颈，得在**节点内部**
            #       加 callback group —— 换个容器执行器解决不了。
            #    ⚠️ 检测器内部有 state_mutex_ / diag_mutex_，聚合薄壳有 slot_mutex_，
            #       多线程下安全; **将来新增节点前先确认它也有锁**。
            composable_node_descriptions=[
                ComposableNode(
                    package='br_perception', plugin=plugin, name=name,
                    parameters=make_params(name, uses_color, sim_camera_on),
                    extra_arguments=[{'use_intra_process_comms': intra}])
                for name, plugin, _exe, uses_color in NODE_SPECS
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim}],
        )

    topology = OpaqueFunction(function=_build_camera_layer)

    # ⚠️ 用 OpaqueFunction 延迟解析: _include 内部要查包，而它在
    #    generate_launch_description() 阶段就执行 —— 早于 Condition 求值。
    #    直接写 condition=IfCondition(...) 的话，没装 realsense 时
    #    即使 realsense:=false 也会在生成阶段抛异常，条件形同虚设。
    def _realsense_if_enabled(context):
        if context.launch_configurations.get('realsense', 'false').lower() in ('false', '0', 'no'):
            return []
        # ⚠️ use_sim_time **必须一起透传**给驱动 —— 否则驱动的深度图时间戳用系统时钟，
        #    与本层其它节点不同源；聚合薄壳的 stale_sec 全靠 header.stamp 相减，
        #    轻则算出天文数字，重则 rclcpp::Time 混用不同 clock_type **直接抛异常**。
        #    （实机跑时它本来就是 false，这里透传是为了仿真/回放场景不踩坑。）
        return [_include(
            'realsense2_camera', 'rs_launch.py',
            launch_arguments={
                'camera_name': 'camera',        # → /camera/depth/image_rect_raw
                'enable_color': 'false',        # ⚠️ 本层用不到 D435i 的彩色
                'enable_depth': 'true',
                # ⚠️ 带点号的参数名在不同 realsense2_camera 版本间有过变化 ——
                #    已记入 doc/待编译验证清单_20260920.md，起不来先核对这个键名。
                'align_depth.enable': 'true',
                'use_sim_time': context.launch_configurations.get('use_sim_time', 'false'),
            },
            source_hint='D435i 驱动: sudo apt install ros-humble-realsense2-camera')]

    realsense = OpaqueFunction(function=_realsense_if_enabled)

    # ⚠️ 相机话题是 BEST_EFFORT，RViz 默认 Reliable **不兼容 = 什么都不显示** ——
    #    所以必须用 config/camera_debug.rviz（那份的显示项全部 Best Effort），
    #    **不能**复用 lidar_perception.rviz（那个连 Reliability Policy 都没写 = Reliable）。
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'camera_debug.rviz')],
        parameters=[{'use_sim_time': use_sim}],
        condition=IfCondition(LaunchConfiguration('perception_rviz')))

    return LaunchDescription([
        intra_arg,
        rs_arg,
        perception_rviz_arg,
        sim_arg,
        sim_camera_arg,
        LogInfo(msg="[br_perception] 相机层联调启动 — §10.4 (P0-6)"),
        LogInfo(msg="  期望话题: /perception/camera/frame  (聚合薄壳输出)"),
        LogInfo(msg="  ⚠️ 深度槽默认恒为 IDLE —— ROI 话题 /camera/depth/target_roi 无发布者，"
                    "这是正常的；要测 MEASURE 见本文件 docstring 的 ros2 topic pub 示例"),
        topology,
        realsense,
        rviz_node,
    ])
