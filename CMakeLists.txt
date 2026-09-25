cmake_minimum_required(VERSION 3.8)
project(br_perception)

# ── C++17 ──
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# ═══════════════════════════════════════════════════════════════════════════
# 依赖查找
# ═══════════════════════════════════════════════════════════════════════════
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search features)
find_package(Eigen3 REQUIRED)
# serial_manager 用 std::thread —— Linux 上必须显式链 pthread
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 外部包: Livox 驱动 (桥接节点用它的 CustomMsg 头)。
# ⚠ 在 ~/point_ws, 构建前必须 `source ~/point_ws/install/setup.bash`, 否则 configure 直接报错。
find_package(livox_ros_driver2 REQUIRED)

# ═══════════════════════════════════════════════════════════════════════════
# 自定义消息生成
# ═══════════════════════════════════════════════════════════════════════════
rosidl_generate_interfaces(${PROJECT_NAME}
  # 2026-09-25 新增: 融合层的追踪目标 (模块 18/19/21 的**公共契约**)
  #   ⚠️ 为什么先建它: 那三个模块的规格都写着"输入/输出**追踪列表**", 而仓库里
  #      **没有任何消息能表达它** —— 与 `fusion_node` 那条"前置: 消息接口必须先落"
  #      是同一类问题(台账 §五)。消息接口不先落, 后面三个模块都写不了。
  "msg/Track.msg"
  # 2026-09-25 新增: 融合层输出 (模块 21)。⚠️ **不是 /perception/output** ——
  #   用户拍板 pipeline 保有上行权, 21 只发融合结果, 由 pipeline 汇总进 PerceptionFrame。
  "msg/FusionState.msg"
  "msg/PillarStatus.msg"
  "msg/Obstacle.msg"
  "msg/BuildingSpot.msg"
  "msg/PerceptionFrame.msg"
  "msg/LidarResult.msg"
  "msg/MasterCommand.msg"
  "msg/CameraMustika.msg"
  "msg/CameraBuildingSpot.msg"
  "msg/CameraPillar.msg"
  # 2026-09-20 新增: 预处理图 + letterbox 变换 (检测框反算用)
  "msg/PreprocessedImage.msg"
  # 2026-09-20 新增: D435i 深度辅助 (§3.0 三相机并行)
  "msg/CameraDepth.msg"
  # 2026-09-21 新增: 相机层聚合帧 (§15.1 模块 10, P0-2)
  #   ⚠️ 下面两条的**先后顺序不影响构建** —— rosidl 按 IDL 依赖图解析同包引用,
  #      不看本列表的实参顺序(本仓库现有的同包引用恰好都是"被引用者在前",
  #      所以这里没有能证明该点的先例; 真遇到解析失败, 再考虑调序)。
  "msg/SlotStatus.msg"
  "msg/CameraFrame.msg"
  # 2026-09-23 新增: 天空方块顶面颜色 (§15.1 模块 9)
  "msg/CameraSkyBlock.msg"
  # std_msgs: PerceptionFrame 与 LidarResult 都用了 std_msgs/Header
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces diagnostic_msgs sensor_msgs
)

# 获取 C++ typesupport 链接目标（供组件库链接）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# 公共 include 目录
include_directories(
  src/lidar/include
  src/camera/include
  src/vision/include
  src/fusion/include
  src/utils/include
  src/communication/include
  src/core/include          # 2026-09-22: perception_pipeline (模块 27)
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ═══════════════════════════════════════════════════════════════════════════
# 宏: 创建 ROS2 组件库
#   ARGN 自动捕获所有剩余参数 → 作为 ROS2 依赖包名列表传递给 ament
# ═══════════════════════════════════════════════════════════════════════════
macro(add_lidar_component target_name source_file plugin_class executable_name)
  add_library(${target_name} SHARED ${source_file})
  ament_target_dependencies(${target_name} ${ARGN})
  target_link_libraries(${target_name}
    ${PCL_LIBRARIES}
    ${cpp_typesupport_target})
  rclcpp_components_register_node(${target_name}
    PLUGIN "${plugin_class}"
    EXECUTABLE ${executable_name})
endmacro()

# ═══════════════════════════════════════════════════════════════════════════
# 1. LidarPreprocessor — 预处理
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(lidar_preprocessor_component
  src/lidar/lidar_preprocessor.cpp
  "br_perception::lidar::LidarPreprocessor"
  lidar_preprocessor
  rclcpp rclcpp_components sensor_msgs geometry_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 2. GroundSegmenter — 地面分割
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(ground_segmenter_component
  src/lidar/ground_segmenter.cpp
  "br_perception::lidar::GroundSegmenter"
  ground_segmenter
  rclcpp rclcpp_components sensor_msgs std_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 3. CylinderDetector — 圆柱检测
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(cylinder_detector_component
  src/lidar/cylinder_detector.cpp
  "br_perception::lidar::CylinderDetector"
  cylinder_detector
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 4. ClusterExtractor — 障碍物聚类
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(cluster_extractor_component
  src/lidar/cluster_extractor.cpp
  "br_perception::lidar::ClusterExtractor"
  cluster_extractor
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 5. BuildingSpotAnalyzer — 建筑位分析
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(building_spot_analyzer_component
  src/lidar/building_spot_analyzer.cpp
  "br_perception::lidar::BuildingSpotAnalyzer"
  building_spot_analyzer
  # ⚠️ `tf2 tf2_ros`（2026-09-24 加坐标系修复时才需要的）—— 它要**只读 TF**
  #    把输入点云搬到场地系。⚠️ 只用 `lookupTransform` + `pcl::transformPointCloud`
  #    ⇒ **不需要 `tf2_geometry_msgs`**（那个包是给 `Buffer::transform<PointStamped>`
  #    的重载用的，逐点变换才要它）。
  #    ⚠️ 漏了这两个 ⇒ 编译期死在 `#include <tf2_ros/buffer.h>`。
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions
  tf2 tf2_ros)

# ═══════════════════════════════════════════════════════════════════════════
# 6. LidarPerceptionNode — 主节点 (全管线编排)
#    额外链接 Eigen3::Eigen (非 ament 包, 单独处理)
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(lidar_perception_node_component
  src/lidar/lidar_perception_node.cpp
  "br_perception::lidar::LidarPerceptionNode"
  lidar_perception_node
  # ⚠️ `tf2 tf2_ros`：同 `building_spot_analyzer` 那条 —— 2026-09-24 的坐标系修复
  #    要用它们把点云搬到场地系（**不需要** `tf2_geometry_msgs`）。
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs std_msgs pcl_conversions
  tf2 tf2_ros)
target_link_libraries(lidar_perception_node_component Eigen3::Eigen)

# ═══════════════════════════════════════════════════════════════════════════
# 7. ImagePreprocessor — 双相机图像预处理
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(image_preprocessor_component
  src/camera/image_preprocessor.cpp
  "br_perception::camera::ImagePreprocessor"
  image_preprocessor
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(image_preprocessor_component ${OpenCV_LIBRARIES})

# ═══════════════════════════════════════════════════════════════════════════
# 8. VisionUtils — 传统 CV 颜色工具库 (被 MustikaDetector 等链接)
# ═══════════════════════════════════════════════════════════════════════════
add_library(vision_utils SHARED src/vision/color_utils.cpp)
ament_target_dependencies(vision_utils
  rclcpp rclcpp_components sensor_msgs cv_bridge)
target_link_libraries(vision_utils ${OpenCV_LIBRARIES})

# ═══════════════════════════════════════════════════════════════════════════
# 9. MustikaDetector — 穆斯蒂卡相机检测
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(mustika_detector_component
  src/camera/mustika_detector.cpp
  "br_perception::camera::MustikaDetector"
  mustika_detector
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(mustika_detector_component
  ${OpenCV_LIBRARIES}
  vision_utils
  ${cpp_typesupport_target})

# ═══════════════════════════════════════════════════════════════════════════
# 10. BuildingSpotDetector — 建筑位绿色方格检测
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(building_spot_detector_component
  src/camera/building_spot_detector.cpp
  "br_perception::camera::BuildingSpotDetector"
  building_spot_detector
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(building_spot_detector_component
  ${OpenCV_LIBRARIES}
  vision_utils
  ${cpp_typesupport_target})

# ═══════════════════════════════════════════════════════════════════════════
# 11. PillarDetector — 柱子检测 (深绿穆斯蒂卡柱 + 棕色核心支柱)
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(pillar_detector_component
  src/camera/pillar_detector.cpp
  "br_perception::camera::PillarDetector"
  pillar_detector
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(pillar_detector_component
  ${OpenCV_LIBRARIES}
  vision_utils
  ${cpp_typesupport_target})

# ═══════════════════════════════════════════════════════════════════════════
# 11a2. SkyBlockDetector — 天空方块「顶面颜色」判定 (任务书 §3.3, 模块 9)
#
#      值多少分: 规则 3.5.6 —— Sky Block 按**顶面颜色**计分; 3.5.7 说明
#      "颜色朝上的那队算完成这座塔（**哪怕底下的地球块是对手的**）";
#      3.5.8 允许对手**翻转**它偷塔 ⇒ 这是 40/80 分归属的唯一来源,
#      而雷达对颜色无能为力 (`building_spot_analyzer` 只置 needs_visual_check)。
#
#      ⚠️ 本目标**只做判色，不做检测**（检测要 YOLO26n ONNX, 那套基础设施
#         仓库里一样都没有, 见该节点头文件的"只做判色"一节）。
#
#      ⚠️ 刻意**不走 `add_lidar_component` 宏** —— 那个宏无条件链 PCL, 而本节点
#         只碰 OpenCV 图像, **不碰点云**（同 `mustika_pillar_check` 与 11c 节的
#         相机聚合薄壳; 容器化时 PCL 是个真负担）。
#      ⚠️ 依赖列表**照抄 `mustika_detector_component`**（同层的检测器, 已验证的
#         组合）: 少了 `cv_bridge` 会报找不到 cv_bridge 头; 少了 `vision_utils`
#         会在链接期报 color_utils 的符号未定义。
# ═══════════════════════════════════════════════════════════════════════════
add_library(sky_block_detector_component SHARED
  src/camera/sky_block_detector.cpp)
# ⚠️ `tf2 tf2_ros` 是加投影主路时才需要的 —— 它要**只读 TF** 把建筑位投到图像上。
# ⚠️ 2026-09-24 重构后**不再需要 `tf2_geometry_msgs`**：投影改成"**查一次**变换 +
#    自己算点变换"（`project_with_transform`，纯数学、可单测），不再走
#    `Buffer::transform<PointStamped>` 那条逐点重载。
# ⚠️ 谁把逐点变换改回来，**必须把它加回来**，否则编译期报
#    `no matching function for call to 'doTransform'`（`mustika_pillar_check` 栽过一次）。
ament_target_dependencies(sky_block_detector_component
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge
  geometry_msgs std_msgs tf2 tf2_ros)
target_link_libraries(sky_block_detector_component
  ${OpenCV_LIBRARIES}
  vision_utils
  ${cpp_typesupport_target})
rclcpp_components_register_node(sky_block_detector_component
  PLUGIN "br_perception::camera::SkyBlockDetector"
  EXECUTABLE sky_block_detector)

# ═══════════════════════════════════════════════════════════════════════════
# 11b. CameraDepthNode — D435i 深度辅助 (§3.0 三相机并行)
#      ⚠️ 只出深度, 不跑 YOLO。独立进程/独立回调, 不阻塞主感知链。
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(camera_depth_component
  src/camera/camera_depth_node.cpp
  "br_perception::camera::CameraDepthNode"
  camera_depth
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs cv_bridge)
target_link_libraries(camera_depth_component
  ${OpenCV_LIBRARIES}
  ${cpp_typesupport_target})

# ═══════════════════════════════════════════════════════════════════════════
# 11c. CameraPerceptionNode — 相机层聚合薄壳 (§15.1 模块 10, P0-2)
#      把 3 个检测器 × 两路相机 + 深度聚合成一帧。**只做时间对齐 + 打包**。
#
#      ⚠️ 刻意不走 add_lidar_component 宏 —— 那个宏无条件链 PCL, 而本节点
#         只订阅已有话题、只做消息拷贝, **不碰点云也不碰图像**(与 comm_node 同理)。
#         走宏会白白把它挂上 PCL 依赖, 容器化时那是个真的负担。
# ═══════════════════════════════════════════════════════════════════════════
add_library(camera_perception_component SHARED
  src/camera/camera_perception_node.cpp)
ament_target_dependencies(camera_perception_component
  rclcpp rclcpp_components std_msgs diagnostic_msgs)
target_link_libraries(camera_perception_component
  ${cpp_typesupport_target})
rclcpp_components_register_node(camera_perception_component
  PLUGIN "br_perception::camera::CameraPerceptionNode"
  EXECUTABLE camera_perception_node)

# ═══════════════════════════════════════════════════════════════════════════
# 13. V4l2CameraNode — **本仓库自己的相机驱动**（2026-09-25 新增）
#
#     为什么要有它：实测 `usb_cam` 写进 `header.stamp` 的时间戳比发布时刻**老约 600~800ms**
#     （同相机同探针对照：usb_cam 810ms vs 自建发布器 170ms），而且它**发 RELIABLE**、
#     与仓库六路检测器收的 best-effort 不匹配。详见 §八 9.15。
#
#     ⚠️ 需要 `<linux/videodev2.h>` + `mmap` ⇒ 只能跑在 Linux（本仓库的部署环境就是）。
#     ⚠️ 要 OpenCV 做 MJPG→BGR8 解码 ⇒ 显式链 `${OpenCV_LIBRARIES}`，
#        不走 `add_lidar_component` 宏（那个宏无条件链 PCL，本节点不碰点云）。
# ═══════════════════════════════════════════════════════════════════════════
add_library(v4l2_camera_component SHARED
  src/camera/v4l2_camera_node.cpp)
ament_target_dependencies(v4l2_camera_component
  rclcpp rclcpp_components sensor_msgs diagnostic_msgs)
target_link_libraries(v4l2_camera_component
  ${cpp_typesupport_target}
  ${OpenCV_LIBRARIES})
rclcpp_components_register_node(v4l2_camera_component
  PLUGIN "br_perception::camera::V4l2CameraNode"
  EXECUTABLE v4l2_camera)

# ═══════════════════════════════════════════════════════════════════════════
# 14. MvsCameraNode — **海康 GigE 相机驱动**（走海康 MVS SDK，2026-09-25 新增）
#
#     为什么单开一个：海康 `MV-CS020-10GC` 是 **GigE**，**不产生 `/dev/video*`**
#     （`v4l2-ctl --list-devices` 里没有它）⇒ `v4l2_camera` 那条 V4L2 路走不通。
#
#     ⚠️ **MVS 是外部 SDK，不在 ROS 生态里**（`/opt/MVS`，非 rosdep key）
#        ⇒ 找不到时**只警告、不构建**，绝不能让没有 MVS 的机器整个包编不过。
#        （本机已装：`/opt/MVS/include` + `/opt/MVS/lib/64/libMvCameraControl.so`。）
# ═══════════════════════════════════════════════════════════════════════════
find_path(MVCAM_INCLUDE_DIR MvCameraControl.h PATHS /opt/MVS/include NO_DEFAULT_PATH)
find_library(MVCAM_LIBRARY MvCameraControl PATHS /opt/MVS/lib/64 /opt/MVS/lib/32 NO_DEFAULT_PATH)

if(MVCAM_INCLUDE_DIR AND MVCAM_LIBRARY)
  add_library(mvs_camera_component SHARED src/camera/mvs_camera_node.cpp)
  target_include_directories(mvs_camera_component PRIVATE ${MVCAM_INCLUDE_DIR})
  ament_target_dependencies(mvs_camera_component
    rclcpp rclcpp_components sensor_msgs diagnostic_msgs)
  target_link_libraries(mvs_camera_component
    ${cpp_typesupport_target}
    ${MVCAM_LIBRARY}
    ${OpenCV_LIBRARIES})
  rclcpp_components_register_node(mvs_camera_component
    PLUGIN "br_perception::camera::MvsCameraNode"
    EXECUTABLE mvs_camera)

  install(TARGETS mvs_camera DESTINATION lib/${PROJECT_NAME})
  install(TARGETS mvs_camera_component
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

  message(STATUS "海康 MVS SDK 找到 (${MVCAM_LIBRARY}) ⇒ 构建 mvs_camera")
else()
  message(WARNING
    "未找到海康 MVS SDK（找的是 /opt/MVS）⇒ **不构建 mvs_camera**。"
    "要接海康 GigE 相机需先装 MVS SDK；只跑 JR/UVC 那类相机则不受影响。")
endif()

# ═══════════════════════════════════════════════════════════════════════════
# 12. CoordinateTransformer — 坐标变换 / 里程计可靠性 (模块 17)
#     辅助库 (构造需 rclcpp::Node*), 供后续 fusion_node 与 test 链接
# ═══════════════════════════════════════════════════════════════════════════
add_library(coordinate_transformer SHARED
  src/fusion/coordinate_transformer.cpp)
ament_target_dependencies(coordinate_transformer
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs)

# ═══════════════════════════════════════════════════════════════════════════
# 12c. TargetAssociator — 观测↔追踪 关联（模块 19，2026-09-25 新增）
#
#      ⚠️ **不做匈牙利算法** —— 用户 2026-09-19 拍板收窄：本项目落点**全是已知位置**，
#         "观测↔目标"是**查表/落区判定 + 就近指派**。理由与影响写在头文件顶部。
#      ⚠️ **纯 C++，不依赖 ROS / Eigen / PCL** ⇒ 头文件路径按目标声明（与 common_utils
#         同款理由：让"不依赖 ROS"这条设计约束在 CMake 层面成立，不只是注释里的承诺）。
#      ⚠️ 它是 `kalman_tracker`(18) 与 `fusion_node`(21) 的前置 —— 那两个要调它。
# ═══════════════════════════════════════════════════════════════════════════
add_library(target_associator SHARED
  src/fusion/target_associator.cpp)
target_include_directories(target_associator PUBLIC src/fusion/include)

# ═══════════════════════════════════════════════════════════════════════════
# 12d. KalmanTracker — 多目标追踪（模块 18，2026-09-25 新增）
#
#      ⚠️ **纯 C++ 库，不是节点** —— 规格里那句「发布 /perception/tracks」归模块 21：
#         21 是融合层唯一的节点（用户 2026-09-25 拍板），由它统一发布。
#      ⚠️ **Eigen 出现在公开签名里**（`Vec6`/`Mat6`）⇒ 按 PUBLIC 传播，
#         与 `common_utils` 处理 `geometry_utils` 同款理由。
#      ⚠️ `target_associator`(19) 与它是**平级**的：19 出关联、18 出滤波后的目标，
#         两者都由 21 调。
# ═══════════════════════════════════════════════════════════════════════════
add_library(kalman_tracker SHARED
  src/fusion/kalman_tracker.cpp)
target_include_directories(kalman_tracker PUBLIC src/fusion/include)
target_link_libraries(kalman_tracker Eigen3::Eigen)

# ═══════════════════════════════════════════════════════════════════════════
# 12e. FusionDecisions — 融合层判定规则（模块 21 第一刀，2026-09-25 新增）
#
#      ⚠️ **只有规则，没有节点** —— 订阅/发布/调 18/19 由 `fusion_node` 做（下一刀）。
#         理由同 18/19：规则能用闭式解手算，节点不能。
#      ⚠️ **本文件里最值钱的是 `sanctuary_mandate_fulfilled` —— 它值 250 分**
#         （规则 4.5.1：TR 取五色石的前置是圣所使命达成）。
#      ⚠️ **纯 C++，不依赖 ROS / Eigen / PCL**（不读 yaml，场地映射由 21 填）。
# ═══════════════════════════════════════════════════════════════════════════
add_library(fusion_decisions SHARED
  src/fusion/fusion_decisions.cpp)
target_include_directories(fusion_decisions PUBLIC src/fusion/include)

# ═══════════════════════════════════════════════════════════════════════════
# 12g. ScoreEstimator — 实时得分估计（模块 29，2026-09-25 新增）
#
#      ⚠️ 规格原文：「**仅作场下/赛中策略参考，不上报裁判**」⇒ 算错不影响比赛进程。
#         但那也是它值得写对的原因：**拿错的分数去决定"还追不追分"是自欺**。
#      ⚠️ **按【颜色】算分，不按"我方/对方"** —— 仓库里没有"我们是哪一队"的配置，
#         在这里猜一次就会和别处的猜测打架（与模块 27 的 `ownership` 同一条纪律）。
#      ⚠️ **纯 C++，不依赖 ROS / Eigen / PCL**。
# ═══════════════════════════════════════════════════════════════════════════
add_library(score_estimator SHARED
  src/fusion/score_estimator.cpp)
target_include_directories(score_estimator PUBLIC src/fusion/include)

# ═══════════════════════════════════════════════════════════════════════════
# 12h. HealthMonitor — 系统健康 + 热管理（模块 26，2026-09-25 新增）
#
#      ⚠️ 本目标只含**判定逻辑**（热管理状态机 / 告警级别 / 指标阈值）；
#         节点外壳（订诊断、读 hwmon、发话题）另算。
#      ⚠️ **纯 C++，不依赖 ROS / Eigen / PCL** ⇒ 可被 gtest 直接单测。
# ═══════════════════════════════════════════════════════════════════════════
add_library(health_monitor SHARED
  src/core/health_monitor.cpp)
target_include_directories(health_monitor PUBLIC src/core/include)

# ═══════════════════════════════════════════════════════════════════════════
# 12f. FusionNode — 融合层主节点（模块 21 第二刀，2026-09-25 新增）
#
#      ⚠️ **它不发 `/perception/output`** —— 用户 2026-09-25 拍板 pipeline 保有上行权。
#         它发 `/perception/fusion/state`，由 pipeline 汇总进 `PerceptionFrame`。
#      ⚠️ 链接 18/19/decisions 三个库 + 自己的消息。
# ═══════════════════════════════════════════════════════════════════════════
add_library(fusion_node_component SHARED
  src/fusion/fusion_node.cpp)
ament_target_dependencies(fusion_node_component
  rclcpp rclcpp_components std_msgs diagnostic_msgs
  geometry_msgs)
target_link_libraries(fusion_node_component
  ${cpp_typesupport_target}
  fusion_decisions
  kalman_tracker
  target_associator)
rclcpp_components_register_node(fusion_node_component
  PLUGIN "br_perception::fusion::FusionNode"
  EXECUTABLE fusion_node)

# ═══════════════════════════════════════════════════════════════════════════
# 12b. MustikaPillarCheck — 最小融合「穆斯蒂卡在柱上吗」(任务书 §15.5, 模块 20 第一刀)
#
#      相机检测到球 → TF 投影到 world → 与雷达的柱位比对 → 确认"在柱上"。
#      值多少分: 规则 4.5.1 —— TR 取五色石的前置是圣所使命达成, 而"柱上有没有球"
#      是 §3.0.1 表里点名的得分动作 (**250 分**)。
#
#      ⚠️ 它**只用只读 TF**（`tf2_ros::Buffer` + `TransformListener`），
#         **不起 `CoordinateTransformer`** —— 那个类构造时会广播 TF,
#         而 perception_pipeline 已经起了一个, 再来一个就是第三个广播同一条变换
#         ⇒ tf2 刷 TF_REPEATED_DATA。详见该节点头文件的成员注释。
#
#      ⚠️ **补这个目标是 2026-09-23 的一处真缺口**：
#         `.cpp` 写了、`bringup_full` 也挂了, 但 CMake 里**零命中** ⇒
#         不编译、不注册、不 install ⇒ 一 launch 就报 "executable not found",
#         而那是**运行期假故障**, 会把真正的编译错误信号盖住。
#         （教训：**"文件写完"和"接进构建"是两件事** —— 与"写完即断链"同类。）
# ═══════════════════════════════════════════════════════════════════════════
# ⚠️ 不走 `add_lidar_component` 宏（它无条件链 PCL，而本节点不碰点云）。
#    ⚠️ 依赖列表**照抄 `perception_pipeline_component` 那套已验证的组合**
#       （含 tf2/tf2_ros/tf2_geometry_msgs）—— 本节点直接 include tf2_ros 的头，
#       少一个就可能报"找不到 tf2_ros/buffer.h"。
add_library(mustika_pillar_check_component SHARED
  src/fusion/mustika_pillar_check.cpp)
ament_target_dependencies(mustika_pillar_check_component
  rclcpp rclcpp_components std_msgs diagnostic_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(mustika_pillar_check_component
  ${cpp_typesupport_target})
rclcpp_components_register_node(mustika_pillar_check_component
  PLUGIN "br_perception::fusion::MustikaPillarCheck"
  EXECUTABLE mustika_pillar_check)

# ═══════════════════════════════════════════════════════════════════════════
# 13. Livox CustomMsg ↔ PointCloud2 桥接节点
#     仿真: Gazebo PointCloud2 → CustomMsg → Point-LIO (与实机同走 lidar_type=1)
#     实机: 驱动 CustomMsg → PointCloud2 → 感知管线 (§3)
# ═══════════════════════════════════════════════════════════════════════════
# livox_ros_driver2 是接口包: 显式链接其 C++ typesupport (与本仓库自有消息同做法)
#
# ⚠ 上面那句 find_package(... REQUIRED) 保证的是"**包**存在", 而**不保证**它导出了
#   下面这个名字的**目标** —— 包找得到但目标不存在是常见情况 (取决于对方怎么构建/安装)。
#   所以判 TARGET 不是多余的死代码, 是两件不同的事。
if(TARGET livox_ros_driver2__rosidl_typesupport_cpp)
  set(livox_cpp_typesupport livox_ros_driver2__rosidl_typesupport_cpp)
else()
  message(WARNING "livox_ros_driver2 已找到, 但它未导出 C++ typesupport 目标 "
                  "(livox_ros_driver2__rosidl_typesupport_cpp) — "
                  "桥接节点可能在链接期报 CustomMsg 符号未定义")
  set(livox_cpp_typesupport "")
endif()

add_executable(livox_custom_to_pcl2 src/utils/livox_custom_to_pcl2.cpp)
ament_target_dependencies(livox_custom_to_pcl2
  rclcpp sensor_msgs pcl_conversions livox_ros_driver2)
target_link_libraries(livox_custom_to_pcl2
  ${PCL_LIBRARIES}
  ${livox_cpp_typesupport})

add_executable(pointcloud2_to_livox_custom
  src/utils/pointcloud2_to_livox_custom.cpp)
ament_target_dependencies(pointcloud2_to_livox_custom
  rclcpp sensor_msgs pcl_conversions livox_ros_driver2)
target_link_libraries(pointcloud2_to_livox_custom
  ${PCL_LIBRARIES}
  ${livox_cpp_typesupport})

# ═══════════════════════════════════════════════════════════════════════════
# 14. CommonUtils — 通用工具库 (任务书 §七, 模块 2/3/…)
#     ⚠ **刻意不挂 ament_target_dependencies** —— 不依赖 ROS / PCL / OpenCV,
#     只依赖 Eigen3 (仅头文件库, 已在文件上方 find_package 过)。
#     这是设计文档 §4 硬性规则第 1 条「算法类不碰 ROS」的直接体现, 也使得
#     该库可被 gtest 直接单测 (§11.1), 不需要起 ROS。
#     ⚠ 后续新增的 src/utils/ 模块加进本目标的源文件列表即可。
# ═══════════════════════════════════════════════════════════════════════════
add_library(common_utils SHARED
  src/utils/timer_utils.cpp
  src/utils/geometry_utils.cpp)
# 头文件路径**按目标声明**, 不走全局 include_directories ——
# 于是"本库不依赖 PCL/OpenCV"这条设计约束在 CMake 层面就成立, 不只是注释里的承诺。
# PUBLIC: 链接 common_utils 的目标 (两个单测) 自动继承该路径, 无需各自再声明。
target_include_directories(common_utils PUBLIC src/utils/include)
# Eigen 出现在 geometry_utils.hpp 的公开签名里 (Point3/Isometry3d), 故按 PUBLIC 传播
target_link_libraries(common_utils Eigen3::Eigen)

# ═══════════════════════════════════════════════════════════════════════════
# 14b. PerceptionPipeline — 感知编排与降级薄壳 (任务书 §6.1 模块 27)
#
#      把各层结果组装成"要发给主控的那一帧", 并在某层死掉时如实降级。
#      **补的是断链**: `/perception/output` 是 comm_node 唯一的上行入口,
#      而在此之前全仓库没有发布者 —— 下位机一帧都收不到。
#
#      ⚠️ **刻意不走 add_lidar_component 宏** —— 那个宏无条件链 PCL, 而本节点
#         不碰点云也不碰图像(它只转坐标 + 打包小消息)。走宏会白白挂上 PCL 依赖。
#         (与 camera_perception_component / comm_node_component 同一处理。)
#
#      ⚠️ **必须排在 coordinate_transformer 与 common_utils 之后** ——
#         本目标链接这两个库, CMake 要求被链目标先定义。
#         依赖关系(rclcpp 之外):
#           · coordinate_transformer —— TF 到 world + 里程计可靠性(模块 17)
#           · common_utils           —— timer_utils 的 FpsCounter / StageProfiler /
#                                       classify_latency (**复用, 不重造**)
#
#      ⚠️ 本目标**不依赖 PCL / OpenCV / cv_bridge**。将来若有人往本节点里塞图像
#         处理, 链接会失败 —— 那是**故意的**: 它是个编排薄壳, 不该长成算法节点。
# ═══════════════════════════════════════════════════════════════════════════
add_library(perception_pipeline_component SHARED
  src/core/perception_pipeline.cpp)
ament_target_dependencies(perception_pipeline_component
  rclcpp rclcpp_components std_msgs diagnostic_msgs geometry_msgs)
target_link_libraries(perception_pipeline_component
  coordinate_transformer
  common_utils
  ${cpp_typesupport_target})
rclcpp_components_register_node(perception_pipeline_component
  PLUGIN "br_perception::core::PerceptionPipeline"
  EXECUTABLE perception_pipeline)

# ═══════════════════════════════════════════════════════════════════════════
# 15. Communication — 协议编解码 + 串口传输 (任务书 §5.1 / §5.2 / §5.3)
#     · encoder/decoder 共用同一份 Crc16 与帧格式定义 (decoder 头文件直接 include
#       encoder 头文件), 拆开只会多一层没必要的依赖边
#     · serial_manager 内部持有 ProtocolDecoder (拼帧只实现一处), 故同属一个库
#     用 ROS 消息类型作输入契约, 但**不含节点/订阅** —— 可直接单测, 不需要 spin。
#     ⚠ serial_manager.cpp 是 **Linux/POSIX termios**, Windows 下编不过。
# ═══════════════════════════════════════════════════════════════════════════
add_library(communication SHARED
  src/communication/protocol_encoder.cpp
  src/communication/protocol_decoder.cpp
  src/communication/serial_manager.cpp)
ament_target_dependencies(communication
  rclcpp std_msgs geometry_msgs diagnostic_msgs)
target_link_libraries(communication
  ${cpp_typesupport_target}
  Threads::Threads)

# ═══════════════════════════════════════════════════════════════════════════
# 16. CommNode — 通信薄壳 (任务书 §5.4)
#     做成**组件**而不是单体可执行: 容器化时直接挂进容器 C, 不必改代码。
#     刻意不走 add_lidar_component 宏 —— 那个宏无条件链 PCL, 通信层不需要。
# ═══════════════════════════════════════════════════════════════════════════
add_library(comm_node_component SHARED
  src/communication/comm_node.cpp)
ament_target_dependencies(comm_node_component
  rclcpp rclcpp_components diagnostic_msgs std_msgs geometry_msgs)
target_link_libraries(comm_node_component
  communication
  ${cpp_typesupport_target})
rclcpp_components_register_node(comm_node_component
  PLUGIN "br_perception::communication::CommNode"
  EXECUTABLE comm_node)

# ═══════════════════════════════════════════════════════════════════════════
# 测试工具: 模拟点云发布者 (无雷达时用于开发测试)
# ═══════════════════════════════════════════════════════════════════════════
add_executable(test_cloud_publisher
  test/test_cloud_publisher.cpp)
ament_target_dependencies(test_cloud_publisher
  rclcpp sensor_msgs pcl_conversions)
target_link_libraries(test_cloud_publisher ${PCL_LIBRARIES})

# ═══════════════════════════════════════════════════════════════════════════
# 单元测试 (§11) — colcon test / 或 ros2 run 直接跑
# ═══════════════════════════════════════════════════════════════════════════
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_coordinate_transformer
    test/test_coordinate_transformer.cpp)
  target_link_libraries(test_coordinate_transformer coordinate_transformer)
  ament_target_dependencies(test_coordinate_transformer
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)

  # 纯算法库 — 无 ROS 依赖, 故无 ament_target_dependencies
  ament_add_gtest(test_timer_utils test/test_timer_utils.cpp)
  target_link_libraries(test_timer_utils common_utils)

  ament_add_gtest(test_geometry_utils test/test_geometry_utils.cpp)
  target_link_libraries(test_geometry_utils common_utils)

  # 用消息类型作输入但不起节点 —— 故只需 ament_target_dependencies, 不需要 spin
  ament_add_gtest(test_protocol_encoder test/test_protocol_encoder.cpp)
  target_link_libraries(test_protocol_encoder communication)
  ament_target_dependencies(test_protocol_encoder
    rclcpp std_msgs geometry_msgs diagnostic_msgs)

  # ── 模块 9 天空方块判色（2026-09-24 建；台账里它是"❌ 未建"那条）──
  # ⚠️ **只测 static 纯函数**（判色核心 / 上半区域 / 时序状态机 / 投影几何）——
  #    所以它**不起 ROS 节点、不建 ROS 图**：直接链组件库拿符号即可。
  #    ⚠️ 投影几何那部分是 2026-09-24 专门从 `project_spot` 里摘出来的
  #       （原来逐点调 `tf2_ros::Buffer`，测试里喂不进去）—— 而它**已经立过功**：
  #       摘出来写用例时，本机用 Python 复现同一个公式发现**平移约定写反了**
  #       （`R·(p−t)` vs tf 的 `R·p+t`），否则所有投影都会落到光心背后。
  ament_add_gtest(test_sky_block_detector test/test_sky_block_detector.cpp)
  target_link_libraries(test_sky_block_detector
    sky_block_detector_component
    vision_utils
    ${OpenCV_LIBRARIES})
  ament_target_dependencies(test_sky_block_detector
    rclcpp geometry_msgs std_msgs diagnostic_msgs cv_bridge)

  # ── 模块 27：天空方块选色决策（2026-09-24 建）──
  # ⚠️ 只测 `choose_sky_color`（static 纯函数）—— 不起 ROS 节点。
  #    它决定**写进 `BuildingSpot.top_color` 的那个值**，而那 40/80 分的归属只由它决定；
  #    最贵的一条是「两路相机打架 ⇒ 不许下结论」。见 `test/test_perception_pipeline.cpp`。
  ament_add_gtest(test_perception_pipeline test/test_perception_pipeline.cpp)
  target_link_libraries(test_perception_pipeline
    perception_pipeline_component
    coordinate_transformer
    common_utils)
  ament_target_dependencies(test_perception_pipeline
    rclcpp std_msgs geometry_msgs diagnostic_msgs)

  # ── 模块 20：穆斯蒂卡在柱上吗（**250 分**；2026-09-24 之前一个单测都没有）──
  # ⚠️ 只测两个 static 纯函数（球径反推距离 / 像元反投影）—— 不起 ROS 节点、不用 TF。
  #    它们正好是两类最贵的错所在：距离公式错 ⇒ 残差全对不上（会让人反复标外参）；
  #    反投影约定错 ⇒ **整体差 90° 且不报错**（2026-09-23 复核抓到过一次）。
  ament_add_gtest(test_mustika_pillar_check test/test_mustika_pillar_check.cpp)
  target_link_libraries(test_mustika_pillar_check
    mustika_pillar_check_component
    coordinate_transformer
    common_utils)
  ament_target_dependencies(test_mustika_pillar_check
    rclcpp geometry_msgs std_msgs diagnostic_msgs)

  # ── 模块 14：障碍物聚类 + 过滤 + 分类（2026-09-25 建）──
  # ⚠️ 只测 **static 纯函数**（欧式聚类 / 包围盒·质心·PCA / 四条过滤 / 尺寸分类 / 圆柱掩膜）
  #    —— 不起 ROS 节点。对应 `doc/容器化重构_任务清单_20260914.md` 任务 5 的三条要求：
  #    不同距离下的聚类精度 / 尺寸过滤正确性 / **分类标签正确性**。
  # ⚠️ **必须链 `${PCL_LIBRARIES}`**：`extract_clusters` 用 `pcl::EuclideanClusterExtraction`
  #    与 `pcl::search::KdTree`，`PointCloudPtr` 的符号也在 PCL 里。
  #    （`add_lidar_component` 宏给组件库链了 PCL，但**不会**传播到测试目标。）
  # ⚠️ 不需要 `Eigen3::Eigen`：公开签名里没有 Eigen 类型（用自有的 `Obstacle` POD）。
  ament_add_gtest(test_cluster_extractor test/test_cluster_extractor.cpp)
  target_link_libraries(test_cluster_extractor
    cluster_extractor_component
    ${PCL_LIBRARIES})
  ament_target_dependencies(test_cluster_extractor
    rclcpp sensor_msgs visualization_msgs diagnostic_msgs)

  # ── 模块 30：两个自建相机驱动的纯函数（2026-09-25 建）──
  # ⚠️ 只测 **static 纯函数**（fourcc 字节序 / 两种时间戳换算 / mvs_str / err_name）。
  #    采集循环要真设备，跑不了 —— **别把这条的绿当成"驱动能跑"**。
  #    ⚠️ 这一路所有的麻烦都在时间戳上，所以最贵的用例是那几条换算。
  # ⚠️ **必须条件化**：`mvs_camera_component` 只在找到 MVS SDK 时才存在
  #    （见上面的 `find_path(MVCAM_INCLUDE_DIR …)`）⇒ 无条件链会在没装 MVS 的机器上配置失败。
  if(MVCAM_INCLUDE_DIR AND MVCAM_LIBRARY)
    # ── 模块 19：观测↔追踪 关联（2026-09-25 建）──
  # ⚠️ 只测 **static 纯函数** —— 不起节点。最贵的一条是「**落区优先于就近**」，
  #    那条改错会**静默**让建筑位归属串位（下游连着 40/80 分的 top_color 归属）。
  # ⚠️ 纯 C++ 库，**无 ament_target_dependencies**（它不引任何 ROS 头）。
  ament_add_gtest(test_target_associator test/test_target_associator.cpp)
  target_link_libraries(test_target_associator target_associator)

  # ── 模块 18：多目标追踪（2026-09-25 建）──
  # ⚠️ 只测**纯函数 + 纯数据管理**（滤波器矩阵 / 状态机 / 推移检测 / 增删）。
  #    最贵的几条是**能用闭式解手算**的：恒速运动的新息必须趋近 0、静止目标速度必须收敛到 0。
  #    ⚠️ 卡尔曼错了**不报错**，只表现为"跟得不对" —— 那些在 RViz 里都是平滑轨迹。
  # ⚠️ 纯 C++ 库（Eigen 由 `common_utils` 的 find_package 提供），**无 ament_target_dependencies**。
  ament_add_gtest(test_kalman_tracker test/test_kalman_tracker.cpp)
  target_link_libraries(test_kalman_tracker kalman_tracker)

  # ── 模块 21 判定规则（2026-09-25 建）──
  # ⚠️ 只测**纯规则**（不起节点）。最贵的是 `sanctuary_mandate_fulfilled` ——
  #    **它值 250 分**，判错的两种后果都是"白丢分"或"做规则不允许的动作"。
  # ⚠️ 纯 C++ 库，**无 ament_target_dependencies**。
  ament_add_gtest(test_fusion_decisions test/test_fusion_decisions.cpp)
  target_link_libraries(test_fusion_decisions fusion_decisions)

  # ── 模块 21 的节点：只测**时间同步缓冲**（节点里唯一纯的东西）──
  # ⚠️ 节点本体（订阅/发布/调 18/19/组 FusionState）要真 ROS 图，这一刀不覆盖。
  # ── 模块 29：实时得分估计（2026-09-25 建）──
  # ⚠️ 最贵的三条：`state==4(UNKNOWN)` 一分不计、**穆斯蒂卡 250 分需圣所使命达成**、
  #    L1/L2 分值差一倍（拿 `state` 当唯一输入会少算一半）。
  ament_add_gtest(test_score_estimator test/test_score_estimator.cpp)
  target_link_libraries(test_score_estimator score_estimator)

  # ── 模块 26：热管理与健康判定（2026-09-25 建）──
  # ⚠️ 最贵的是那两条"**不该动作**"：滞回死区里不恢复 / 死区打断连续低温计时。
  #    没有滞回 ⇒ 温度在阈值附近抖 ⇒ 相机管线反复开关，**比一直降级更糟**。
  ament_add_gtest(test_health_monitor test/test_health_monitor.cpp)
  target_link_libraries(test_health_monitor health_monitor)

  # ── 模块 12：地面分割的纯函数（2026-09-25 建，P1-9 的第 1 个）──
  # ⚠️ 只测 `validate_plane_normal` —— 它是**地面分割的第一道闸**，
  #    而它有三处会静默出错的地方（不取 |c| / 零法向量出 NaN / values 越界）。
  # ⚠️ 要 `${PCL_LIBRARIES}`：签名里有 `pcl::ModelCoefficients`。
  # ── 模块 13：圆柱检测的纯函数（2026-09-25 建，P1-9 的第 2 个）──
  # ⚠️ 最贵的是「**空档里不分类**」那条 —— 填上它 = 给每个噪声团贴标签。
  # ── 模块 5：穆斯蒂卡球检测的纯函数（2026-09-25 建，P1-9 的第 3 个）──
  # ⚠️ 它出的是**那 250 分**那条链的输入；而三个阈值 9-20 改过值、台账写着「需重编重测」。
  # ⚠️ 最贵的一条是「半径判据是**两套来源取交集**」—— 头文件早就标注了这个陷阱。
  # ── 模块 4：letterbox 反算（2026-09-25 建，相机层单测的第 1 个）──
  # ⚠️ 下游**每一个检测器**都用它反算检测框；算错了**每个框的坐标都是错的**，
  #    且在"原图与输入同尺寸"时**完全看不出来**。
  # ⚠️ 最贵的是「尺度类反算不能减填充偏移」—— 头文件点名"整套反算里最易错的一处"。
  # ⚠️ 被测的是 header 里的 inline 纯函数，但**那个 header 自己引了 rclcpp**
  #    ⇒ 仍要挂 ament 依赖拿 include 路径（不是"不需要任何依赖"）。
  # ── 模块 10：聚合薄壳的槽状态判定（2026-09-25 建，相机层第 2 个）──
  # ⚠️ 最贵的是「**FRESH 这一档必须真的可达**」—— 历史上判据写成过 `age > 0`，
  #    而 age 恒为正 ⇒ 每个有数据的槽都报 STALE ⇒ **那一档等于不存在**（不报错）。
  # ── 模块 6：建筑位的四边形几何验证（2026-09-25 建，相机层第 3 个）──
  # ⚠️ 建筑位是那 40/80 分判色的**投影锚点**；这两条判据判"这团绿色是不是一个建筑位"。
  # ⚠️ 其中一条**量出这套判据有多松**（明显不规则的四边形仍通过）—— 那是给人看的。
  # ── 模块 7：柱子候选验证（2026-09-25 建，相机层第 4 个 / 最后一个）──
  # ⚠️ 柱位是那 **250 分**（球在不在柱上）的基准点。
  # ⚠️ 最贵的一条钉的是「**一段死代码没有效果**」—— 尺寸检查算完就丢。
  ament_add_gtest(test_pillar_detector test/test_pillar_detector.cpp)
  target_link_libraries(test_pillar_detector
    pillar_detector_component
    vision_utils
    ${OpenCV_LIBRARIES})
  ament_target_dependencies(test_pillar_detector
    rclcpp sensor_msgs diagnostic_msgs cv_bridge)

  ament_add_gtest(test_building_spot_detector test/test_building_spot_detector.cpp)
  target_link_libraries(test_building_spot_detector
    building_spot_detector_component
    vision_utils
    ${OpenCV_LIBRARIES})
  ament_target_dependencies(test_building_spot_detector
    rclcpp sensor_msgs diagnostic_msgs cv_bridge)

  ament_add_gtest(test_camera_perception_node test/test_camera_perception_node.cpp)
  target_link_libraries(test_camera_perception_node
    camera_perception_component)
  ament_target_dependencies(test_camera_perception_node
    rclcpp std_msgs diagnostic_msgs)

  ament_add_gtest(test_image_preprocessor test/test_image_preprocessor.cpp)
  target_link_libraries(test_image_preprocessor
    image_preprocessor_component
    vision_utils
    ${OpenCV_LIBRARIES})
  ament_target_dependencies(test_image_preprocessor
    rclcpp sensor_msgs diagnostic_msgs cv_bridge)

  ament_add_gtest(test_mustika_detector test/test_mustika_detector.cpp)
  target_link_libraries(test_mustika_detector
    mustika_detector_component
    vision_utils
    ${OpenCV_LIBRARIES})
  ament_target_dependencies(test_mustika_detector
    rclcpp sensor_msgs diagnostic_msgs cv_bridge)

  ament_add_gtest(test_cylinder_detector test/test_cylinder_detector.cpp)
  target_link_libraries(test_cylinder_detector
    cylinder_detector_component
    ${PCL_LIBRARIES})
  ament_target_dependencies(test_cylinder_detector
    rclcpp sensor_msgs visualization_msgs diagnostic_msgs)

  ament_add_gtest(test_ground_segmenter test/test_ground_segmenter.cpp)
  target_link_libraries(test_ground_segmenter
    ground_segmenter_component
    ${PCL_LIBRARIES})
  ament_target_dependencies(test_ground_segmenter
    rclcpp sensor_msgs diagnostic_msgs)

  ament_add_gtest(test_fusion_node test/test_fusion_node.cpp)
  target_link_libraries(test_fusion_node
    fusion_node_component
    fusion_decisions
    kalman_tracker
    target_associator)
  ament_target_dependencies(test_fusion_node
    rclcpp std_msgs diagnostic_msgs)

  ament_add_gtest(test_camera_drivers test/test_camera_drivers.cpp)
    target_link_libraries(test_camera_drivers
      v4l2_camera_component
  fusion_decisions
  kalman_tracker
  target_associator
  score_estimator
  health_monitor
  fusion_node_component
      mvs_camera_component
      ${MVCAM_LIBRARY})
    target_include_directories(test_camera_drivers PRIVATE ${MVCAM_INCLUDE_DIR})
    ament_target_dependencies(test_camera_drivers
      rclcpp sensor_msgs diagnostic_msgs)
  else()
    message(STATUS "未找到 MVS SDK ⇒ 跳过 test_camera_drivers（它链接 mvs_camera_component）")
  endif()

  ament_add_gtest(test_protocol_decoder test/test_protocol_decoder.cpp)
  target_link_libraries(test_protocol_decoder communication)
  ament_target_dependencies(test_protocol_decoder
    rclcpp std_msgs geometry_msgs diagnostic_msgs)
endif()

# ═══════════════════════════════════════════════════════════════════════════
# 安装
# ═══════════════════════════════════════════════════════════════════════════
install(TARGETS test_cloud_publisher
  DESTINATION lib/${PROJECT_NAME})

# 桥接节点 — launch 以 package+executable 引用, 必须落在 lib/<pkg>
install(TARGETS
  livox_custom_to_pcl2
  pointcloud2_to_livox_custom
  DESTINATION lib/${PROJECT_NAME})

# rclcpp_components_register_node(... EXECUTABLE <名>) 生成的独立可执行。
# launch 里 executable='lidar_perception_node' / 'mustika_detector' 这类名字引用的就是
# 它们 → 必须在 install 空间里可被 ros2 run / ros2 launch 找到。
# 显式列出 (与 ROS 2 demos 同做法), 不依赖宏内部是否自带 install 规则。
install(TARGETS
  lidar_preprocessor
  ground_segmenter
  cylinder_detector
  cluster_extractor
  building_spot_analyzer
  lidar_perception_node
  image_preprocessor
  mustika_detector
  building_spot_detector
  pillar_detector
  sky_block_detector
  camera_depth
  camera_perception_node
  v4l2_camera
  fusion_node
  perception_pipeline
  mustika_pillar_check
  comm_node
  DESTINATION lib/${PROJECT_NAME})

# 组件库 (.so), rclcpp_components_register_node 只注册不安装
install(TARGETS
  lidar_preprocessor_component
  ground_segmenter_component
  cylinder_detector_component
  cluster_extractor_component
  building_spot_analyzer_component
  lidar_perception_node_component
  image_preprocessor_component
  vision_utils
  mustika_detector_component
  building_spot_detector_component
  pillar_detector_component
  sky_block_detector_component
  camera_depth_component
  camera_perception_component
  v4l2_camera_component
  # 2026-09-25 新增: 融合层（模块 18/19/21）
  fusion_decisions
  kalman_tracker
  target_associator
  score_estimator
  health_monitor
  fusion_node_component
  perception_pipeline_component
  mustika_pillar_check_component
  coordinate_transformer
  common_utils
  communication
  comm_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config launch msg
  DESTINATION share/${PROJECT_NAME})

# ── 仿真资产 (bringup_gazebo.launch.py 需要) ──
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/sim_patrol.py")
  install(FILES scripts/sim_patrol.py
    DESTINATION share/${PROJECT_NAME}/scripts)
else()
  message(WARNING "找不到 scripts/sim_patrol.py — bringup_gazebo 的自动巡游会起不来")
endif()

# ── 台架工具: 伪下位机 (无硬件时通信层的唯一对端) ──
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/fake_master.py")
  install(FILES scripts/fake_master.py
    DESTINATION share/${PROJECT_NAME}/scripts)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/out/gazebo")
  install(DIRECTORY scripts/out/gazebo/
    DESTINATION share/${PROJECT_NAME}/gazebo)
else()
  message(WARNING
    "找不到 scripts/out/gazebo — 先跑 `python3 scripts/make_gazebo_world.py`, "
    "否则 bringup_gazebo.launch.py 找不到世界文件")
endif()

ament_package()
