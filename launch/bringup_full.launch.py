#!/usr/bin/env python3
"""
比赛模式全量启动 — ROBOCON 2027 BR 感知系统 (任务_v1.7 §10.1)

启动内容:
  1. Livox Mid-360 雷达驱动          (CustomMsg — Point-LIO 要求)
  2. Point-LIO 里程计节点            (ROS2 移植版 — §十.2 定案: 仅作纯里程计)
  3. Livox CustomMsg → PointCloud2 桥接 (感知管线要 PointCloud2)
  4. 静态外参 TF (base_link ← 传感器)  ⚠ 占位值, 标定后回填 fusion_params.yaml
  5. 雷达感知管线                    (br_perception: lidar_perception_node)
  6. 相机层(**最小集** 3 个组件, 一个容器)  🆕 2026-09-23
                                      image_preprocessor → mustika_detector
                                                        → camera_perception
                                      ⚠️ **图像源不在本仓库**(工业相机驱动 / Gazebo 提供),
                                         没接上时这 3 个节点起得来但没数据 —— 预期行为
  7. 感知编排与降级                   (br_perception: perception_pipeline, 模块 27)
                                      → 组装 /perception/output (通信层唯一上行入口)
  8. 最小融合                        (br_perception: mustika_pillar_check, §15.5)
                                      → 相机+雷达交叉确认「穆斯蒂卡在柱上吗」
  9. 通信节点                        (br_perception: comm_node, 模块 25)
                                      → /perception/output → 串口 → STM32
 10. 融合 / 健康监控                  ⬜ 待实现 (§4.5 fusion_node / §13)

前置 (必须先 source, 否则找不到驱动与里程计包):
  source /opt/ros/humble/setup.bash
  source ~/ws_livox/install/setup.bash        # livox_ros_driver2
  source ~/lio_ws/install/setup.bash          # point_lio
  source ~/br_perception/install/setup.bash

用法:
  ros2 launch br_perception bringup_full.launch.py
  ros2 launch br_perception bringup_full.launch.py perception_rviz:=false lio:=false
  ros2 launch br_perception bringup_full.launch.py comm:=false     # 台架(无下位机/无串口)
  ros2 launch br_perception bringup_full.launch.py camera:=false   # 没有相机驱动时闭嘴
  ⚠️ `comm` 默认 **true**：关掉它 = 感知算完的东西**到不了 STM32**，比赛模式别关。
  ⚠️ `camera` 默认 **true**：关掉它 = 「穆斯蒂卡在柱上」那 250 分**没有来源**。
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


def _resolve_launch(share_dir, launch_file):
    """在 launch/ 与 launch_ROS2/ (livox_ros_driver2 用后者) 下找 launch 文件"""
    for sub in ('launch', 'launch_ROS2', ''):
        candidate = os.path.join(share_dir, sub, launch_file) if sub \
            else os.path.join(share_dir, launch_file)
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(
        f"在 {share_dir} 下找不到 {launch_file} (已试 launch/ 与 launch_ROS2/)")


def _include(pkg_name, launch_file, launch_arguments=None, source_hint="", condition=None):
    """外部包 launch 的安全 include — 包不存在时给出可读的中文提示"""
    try:
        share = get_package_share_directory(pkg_name)
    except Exception as exc:  # PackageNotFoundError
        raise RuntimeError(
            f"找不到包 '{pkg_name}' — 请先 source 对应工作区。{source_hint}"
        ) from exc
    kwargs = {}
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_resolve_launch(share, launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
        **kwargs,
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('br_perception')

    # ⚠ 参数名不用 'rviz': include 的 launch_arguments 会写进共享 launch context,
    #   给 point_lio 传 rviz:=false 会把同名参数一起污染, 导致本文件的 rviz 静默不启动
    rviz_arg = DeclareLaunchArgument(
        'perception_rviz', default_value='true', choices=['true', 'false'],
        description='启动 RViz2 (感知调试视图)')
    lio_arg = DeclareLaunchArgument(
        'lio', default_value='true', choices=['true', 'false'],
        description='启动 Point-LIO 里程计节点 (与雷达驱动分开控制)')
    # ⚠️ 默认 **true** —— 比赛模式必须有它（否则 STM32 什么都收不到）。
    #    台架/无下位机时设 comm:=false 可少一屏 device_unavailable 日志。
    comm_arg = DeclareLaunchArgument(
        'comm', default_value='true', choices=['true', 'false'],
        description='启动通信节点 comm_node (串口 → STM32)。'
                    '⚠️ **比赛模式必须 true** —— 关掉等于主控收不到任何东西')
    # ⚠️ 默认 **true** —— 相机是"穆斯蒂卡在柱上"那 250 分的**唯一来源**（见第 5b 节）。
    #    关掉它只在"没有相机驱动、又不想看那几个节点报 NEVER"时用。
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='true', choices=['true', 'false'],
        description='启动相机层最小集 (image_preprocessor + mustika_detector + '
                    'camera_perception)。⚠️ **比赛模式必须 true**')
    rviz = LaunchConfiguration('perception_rviz')
    lio = LaunchConfiguration('lio')

    # ── 1. Livox Mid-360 雷达驱动 (CustomMsg — Point-LIO 的 lidar_type:1 要求) ──
    livox_driver = _include(
        'livox_ros_driver2', 'msg_MID360_launch.py',
        source_hint='雷达驱动在 ~/ws_livox, 见 ~/livox_lab_up.sh')

    # ── 2. Point-LIO 里程计节点 (§10.1 / §十.2: 降级为纯里程计, 绝对位姿靠场地先验) ──
    #    复用 lio_ws 的 launch 作为单一事实来源; 此处关掉其自带 rviz
    #    输出话题: /aft_mapped_to_init, 由 coordinate_transformer (§4.1) 消费
    # ⚠ 必须用 OpaqueFunction 延迟到"启动执行期"再解析: _include() 内部要调
    #   get_package_share_directory('point_lio'), 而它在 generate_launch_description()
    #   阶段就执行 —— 早于任何 condition 求值。若写成 _include(..., condition=IfCondition(lio)),
    #   没装 point_lio 时即使 lio:=false 也会在生成阶段抛 RuntimeError, 条件形同虚设。
    def _point_lio_if_enabled(context):
        if context.launch_configurations.get('lio', 'true').lower() in ('false', '0', 'no'):
            return []
        return [_include(
            'point_lio', 'mapping_mid360.launch.py',
            launch_arguments={'rviz': 'false'},
            source_hint='Point-LIO 在 ~/lio_ws (9000_point_lio_ros2_Mid-360)')]

    point_lio = OpaqueFunction(function=_point_lio_if_enabled)

    # ── 3. CustomMsg → PointCloud2 桥接 (感知管线订阅 /livox/lidar/pointcloud) ──
    custom_to_pcl2 = Node(
        package='br_perception', executable='livox_custom_to_pcl2',
        name='livox_custom_to_pcl2', output='screen',
        parameters=[{'input_topic': '/livox/lidar',
                     'output_topic': '/livox/lidar/pointcloud',
                     'frame_id': 'livox_frame'}])

    # ═══════════════════════════════════════════════════════════════════════════
    # 4. 静态外参 TF —— **值从 `config/fusion_params.yaml` 的 `static_tf:` 块读**
    #
    #    TF 树 (§4.1): world ← odom ← base_link ← {livox_frame, camera_*}
    #    world←odom 与 odom←base_link 由 coordinate_transformer 广播。
    #
    #    ⚠️⚠️ 2026-09-23 **由"写死数值"改成"读纵表"**，一次解决两笔债：
    #      ① `fusion_params.yaml` 的 `static_tf:` 块**此前没有任何人读**（台账 §七 记为
    #         "死配置"）—— 现在它是**唯一事实源**，那块配置活了。
    #      ② **"标定产物 → 运行期无自动传递"**：以前标定完要**手工改本文件的数值**，
    #         改了 yaml 不生效（而 yaml 里那份看起来才像配置）。现在
    #         **标定写到 `fusion_params.yaml` 就自动生效**，人工零处。
    #    ⚠️ 代价：本文件多了一段 yaml 读取。**读失败时直接抛**，不静默回落 ——
    #       外参错了会让融合算出错位置且**不报错**，那比起不来糟得多。
    # ═══════════════════════════════════════════════════════════════════════════
    def _load_static_tfs():
        """从 config/fusion_params.yaml 读 `static_tf:` 块 → [(frame, x,y,z, r,p,y), ...]"""
        import yaml
        path = os.path.join(pkg_share, 'config', 'fusion_params.yaml')
        with open(path, encoding='utf-8') as fh:
            doc = yaml.safe_load(fh)
        # ⚠️ 三个 KeyError 都是**故意**的（见本节头："读失败时直接抛，不静默回落"）——
        #    但裸 `KeyError: 'static_tf'` 看不出是哪个文件、该往哪改，所以包一层。
        try:
            block = doc['/**']['ros__parameters']['static_tf']
        except (KeyError, TypeError) as exc:
            raise RuntimeError(
                '{} 里找不到 `/**` → `ros__parameters` → `static_tf` 块（{}）。'
                '本 launch 的静态外参**唯一事实源**就是它 —— 改文件名/挪作用域都会让'
                '这条链没有 base_link←传感器 的 TF。'.format(path, exc)) from exc
        out = []
        for frame, spec in block.items():
            try:
                t = spec['translation']
                r = spec['rotation_rpy']
            except (KeyError, TypeError) as exc:
                raise RuntimeError(
                    '{} 的 static_tf.{} 缺 translation / rotation_rpy（{}）'.format(
                        path, frame, exc)) from exc
            out.append((frame, t[0], t[1], t[2], r[0], r[1], r[2]))

        # ⚠️⚠️ **空块必须当错误报出来**（2026-09-23 本机实测）：
        #    上面三种写法都会**抛异常、launch 起不来** —— 那是好的。
        #    唯独 `static_tf: {}`（空块）**无异常**：`*static_tf_nodes` 展开成空，
        #    launch **正常起、日志干净**，而 TF 树里**一条 base_link←传感器 都没有** ⇒
        #    `perception_pipeline` 的 transform_to_world 全失败 → 一帧都不发，
        #    `mustika_pillar_check` 两侧 TF 全失败 → 恒"未知"。
        #    **零报错、零输出**，现场只能从"完全不输出"倒推。
        #    这正是 §零 第一宗旨"稳"要防的形态，所以在这里硬拦一道。
        if not out:
            raise RuntimeError(
                '{} 的 `static_tf` 块是**空的** → 本 launch 会广播 0 个静态 TF，'
                '整条感知链将静默不出帧（且没有任何其它报错）。'
                '请在 `/**`: `ros__parameters`: `static_tf` 下至少写一个传感器 frame。'.format(path))
        return out

    # ⚠️ `parent_frame` 写死 `base_link`：`static_tf:` 块里只有子帧名，
    #    而 §4.1 的树是 base_link ← 各传感器。将来若出现别的主链，这里要跟着改。
    static_tf_nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name=f'static_tf_base_to_{frame}',
             arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                        '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                        '--frame-id', 'base_link', '--child-frame-id', frame])
        for (frame, x, y, z, roll, pitch, yaw) in _load_static_tfs()
    ]

    # ── 5. 雷达感知管线 (现有实现: 预处理→地面→圆柱→聚类→建筑位) ──
    #    直接起主节点, 不复用 lidar_perception.launch.py — 那个 launch 会发
    #    map→livox_frame 静态 TF, 与本 bringup 的 base_link 树冲突 (同帧双父)
    lidar_pipeline = Node(
        package='br_perception', executable='lidar_perception_node',
        name='lidar_perception_node', output='screen',
        # ⚠️ `debug_flags.yaml` **必须排在最后** —— ROS 2 参数**后者覆盖前者**，
        #    而它里面的 `debug_enabled` 要盖掉 `lidar_params.yaml:86` 的 `true`。
        #    比赛模式要求"调试全部关闭"（任务书 §1.7），那个 true 是给台架用的。
        #    ⚠️ 顺序写反 = 调试点云在比赛时照发，且**不会报任何错**。
        parameters=[os.path.join(pkg_share, 'config', 'lidar_params.yaml'),
                    os.path.join(pkg_share, 'config', 'field_geometry.yaml'),
                    os.path.join(pkg_share, 'config', 'debug_flags.yaml')])

    # ── 6. 感知编排与降级 (模块 27) ──
    #    ⚠️ **必须传两份 yaml**: fusion_params.yaml 提供 TF 的坐标系名与里程计参数
    #       (CoordinateTransformer 的 load_from_node 读它们)。缺了这份 → TF 取默认名
    #       → 与雷达结果的实际坐标系对不上 → **一帧都发不出去**(且不报错, 只报 TF 失败)。
    #    ⚠️ 本节点内部会起一个 CoordinateTransformer, 因此**它广播 odom→base_link /
    #       world→odom**。目前 fusion_node(模块21)不存在, 没人广播 → 这里正好补上;
    #       **fusion_node 落地后两个广播器会打架, 必须二选一**(见 pkg 的 perception_pipeline.hpp)。
    perception_pipeline = Node(
        package='br_perception', executable='perception_pipeline',
        name='perception_pipeline', output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'perception_params.yaml'),
                    os.path.join(pkg_share, 'config', 'fusion_params.yaml')])

    # ═══════════════════════════════════════════════════════════════════════════
    # 5b. 相机层 —— **最小集**（2026-09-23 补：此前比赛链路里一个相机节点都没有）
    #
    #  ⚠️⚠️ 为什么非补不可：`mustika_pillar_check`（下面 6b）的输入
    #     `/perception/camera/mustika_{wide,tele}` **正是这里产出的**。缺这一段，
    #     那条链起完**一个发布者都没有** —— 节点恒判"未知"，诊断里写的是
    #     "这一路还没有数据"，**看着像检测器没检到球，实际是相机层压根没起**。
    #
    #  ⚠️ 只起**得分链上必需的 3 个**：
    #       image_preprocessor → mustika_detector → camera_perception
    #     · mustika_detector   出 CameraMustika，喂 mustika_pillar_check
    #     · camera_perception  聚合出 `/perception/camera/frame`，喂 perception_pipeline
    #                          的相机健康判定（不起它，pipeline 里相机一路恒 NEVER）
    #     · **不起** pillar_detector / building_spot_detector / camera_depth ——
    #       它们不在"穆斯蒂卡在柱上"这条得分链上，比赛模式没理由再多占 CPU
    #       （6 核已分完）。要调它们用 `launch/bringup_debug.launch.py`。
    #
    #  ⚠️ **图像源不在本仓库**：`/camera_wide/image_raw` / `/camera_tele/image_raw`
    #     由工业相机驱动（或 Gazebo）提供。驱动没接上时这 3 个节点**起得来但没数据**
    #     —— 那是预期的，诊断里那两槽报 NEVER，**不是本 launch 的问题**。
    #     接口确认见 `doc/模块进度.md` 的 P2-14。
    #
    #  ⚠️ 用**容器 + intra-process**（与 `bringup_debug` 的默认跑法同型）：
    #     节点之间传的是**图像**（≈700KB/帧 × 30Hz），intra-process 下按指针传递、
    #     不序列化 —— 这是"薄壳聚合比单体多一跳"那个代价的具体消除手段。
    #     代价：3 个节点同进程，一个崩全崩（取舍，不是 bug）。
    # ═══════════════════════════════════════════════════════════════════════════
    camera_params = os.path.join(pkg_share, 'config', 'camera_params.yaml')
    # ⚠️⚠️ 颜色阈值文件**必须给绝对路径**：`camera_params.yaml` 里写的是相对路径
    #     "config/color_thresholds.yaml"，而 `color_utils.cpp` 是 `std::ifstream(yaml_path)`
    #     —— 相对的是**启动时的 $PWD**，节点里没有任何回退解析。launch 可从任意目录起
    #     ⇒ 不给绝对路径，mustika_detector 会找不到颜色文件而**抛异常**。
    #     （同 `bringup_debug.launch.py` 里那段注释，是同一个坑。）
    color_yaml = os.path.join(pkg_share, 'config', 'color_thresholds.yaml')

    # name, plugin, 是否用颜色（三个里只有 mustika_detector 需要）
    CAMERA_MIN_SPECS = [
        ('image_preprocessor', 'br_perception::camera::ImagePreprocessor',
         'image_preprocessor', False),
        ('mustika_detector',   'br_perception::camera::MustikaDetector',
         'mustika_detector',   True),
        ('camera_perception',  'br_perception::camera::CameraPerceptionNode',
         'camera_perception_node', False),
    ]
    camera_layer = ComposableNodeContainer(
        name='camera_layer_container', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        # ⚠️ 用多线程的理由同 bringup_debug：**节点之间**并行（两个检测器各跑各的
        #    OpenCV）。⚠️ 多线程 ≠ 节点内并行 —— 同一节点里 wide/tele 两个回调仍串行。
        composable_node_descriptions=[
            # ⚠️ 参数**顺序有意义，后面的覆盖前面的**（同 bringup_debug 的 make_params）
            ComposableNode(
                package='br_perception', plugin=plugin, name=name,
                parameters=[camera_params] + ([{'color_yaml_path': color_yaml}]
                                              if uses_color else []),
                extra_arguments=[{'use_intra_process_comms': True}])
            for name, plugin, _exe, uses_color in CAMERA_MIN_SPECS
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('camera')))

    # ── 6b. 最小融合：穆斯蒂卡在柱上吗 (模块 20 第一刀, 任务书 §15.5) ──
    #  **相机与雷达的第一次交叉确认** —— 它回答"雷达看到的柱顶那个凸起，是不是球"。
    #  值多少分：规则 4.5.1 —— TR 取五色石的前置是圣所使命达成，而"柱上有没有球"
    #  是 §3.0.1 表里点名的得分动作（**250 分**）。
    #
    #  ⚠️ 依赖 TF（要把相机点转到 world）—— 而 `world←odom` 由
    #     `perception_pipeline` 内部那个 `CoordinateTransformer` 广播
    #     ⇒ **本节点必须排在它后面**（不过严格说这是运行期依赖，不是启动顺序依赖）。
    #  ⚠️ 现在**还没有发布者替它**：它的输出 `/perception/fusion/mustika_state`
    #     由 `perception_pipeline` 订阅（已接），所以链条是通的。
    #  ⚠️⚠️ 它带一道**人工闸** `extrinsics_calibrated`（默认 **false**）：
    #     未标定时**只发残差诊断，状态恒发 0=未知** —— 用占位外参算出的"一致"
    #     是巧合，而这个节点最危险的地方就是**它可能碰巧对、然后被当成标定过了**。
    #     标定完成后改 `config/fusion_params.yaml` 里那个键。
    mustika_check = Node(
        package='br_perception', executable='mustika_pillar_check',
        name='mustika_pillar_check', output='screen',
        # ⚠️ 它的参数块 `mustika_pillar_check:` 就在 `fusion_params.yaml` 里
        #    （与上面 `/**` 块是两个作用域）。漏传 → 全吃代码默认值，
        #    而那里面有**占位内参**，且**不会报错**。
        parameters=[os.path.join(pkg_share, 'config', 'fusion_params.yaml')])

    perception_rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'lidar_perception.rviz')],
        condition=IfCondition(rviz))

    # ── 7. 通信节点 (模块 25) ──
    #  **比赛模式的最后一环** —— 没有它，感知算完的东西**到不了 STM32**。
    #  ⚠️⚠️ 2026-09-23 补：此前本 launch **根本没起 comm_node**（第 7 节注释停在
    #     "通信 ⬜ 待实现"，而 `comm_node` 其实早就写完了）—— 与 `perception_pipeline`
    #     那个"零发布者"是同一类毛病：**launch 没跟上模块落地的进度**。
    #     ⇒ 修好之前，`bringup_full` 起完是**一条完整的、什么都不输出的链**。
    #
    #  ⚠️ 串口打不开时**它不会崩**（`comm_node.cpp:158`：节点继续跑、只报
    #     `device_unavailable`）—— 所以台架/无下位机时起它也是安全的，只是诊断会提示。
    comm = Node(
        package='br_perception', executable='comm_node',
        name='comm_node', output='screen',
        # ⚠️ **必须带 debug_flags.yaml**：`debug_mode`（串口 0x7F 调试帧）的
        #    唯一事实源在那里。漏传的话它会吃自己的默认值 `false` —— **症状上碰巧是对的**，
        #    于是把"总闸没接上"这件事盖掉（见 config/debug_flags.yaml 的接线状态一节）。
        parameters=[os.path.join(pkg_share, 'config', 'comm_params.yaml'),
                    os.path.join(pkg_share, 'config', 'debug_flags.yaml')],
        # ⚠️ 无下位机 / 台架联调时可以 comm:=false 关掉；**比赛模式必须开**。
        condition=IfCondition(LaunchConfiguration('comm')))

    # ── 8. 融合 / 健康监控 ⬜ 待实现 ──
    #  §4.5 fusion_node          : 调用 coordinate_transformer → ... → PerceptionFrame
    #  §13  health_monitor        : 热管理 OnThermalThrottle / 看门狗
    # TODO(模块 21): fusion_node 实现后在此启动, 并加载 config/fusion_params.yaml:
    #   Node(package='br_perception', executable='fusion_node',
    #        parameters=[os.path.join(pkg_share, 'config', 'fusion_params.yaml')])
    # TODO(§13.3): 实时性 — 隔离核心 + 实时调度 (§10.1 任务清单最后两条):
    #   prefix='taskset -c 3 chrt -f 50'   # perception_pipeline 绑定隔离核

    return LaunchDescription([
        rviz_arg,
        lio_arg,
        comm_arg,
        camera_arg,
        LogInfo(msg="[br_perception] 比赛模式启动 — §10.1 (里程计: Point-LIO 纯里程计 + 场地先验)"),
        # ⚠️ 展开：个数由 `fusion_params.yaml` 的 `static_tf:` 块决定，
        #    不再与本文件写死的三个节点绑定（加减传感器只改 yaml）
        *static_tf_nodes,
        livox_driver,
        point_lio,
        custom_to_pcl2,
        lidar_pipeline,
        camera_layer,
        perception_pipeline,
        mustika_check,
        comm,
        perception_rviz_node,
    ])
