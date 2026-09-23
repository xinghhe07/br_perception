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
# ⚠ 在 ~/ws_livox, 构建前必须 `source ~/ws_livox/install/setup.bash`, 否则 configure 直接报错。
find_package(livox_ros_driver2 REQUIRED)

# ═══════════════════════════════════════════════════════════════════════════
# 自定义消息生成
# ═══════════════════════════════════════════════════════════════════════════
rosidl_generate_interfaces(${PROJECT_NAME}
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
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs pcl_conversions)

# ═══════════════════════════════════════════════════════════════════════════
# 6. LidarPerceptionNode — 主节点 (全管线编排)
#    额外链接 Eigen3::Eigen (非 ament 包, 单独处理)
# ═══════════════════════════════════════════════════════════════════════════
add_lidar_component(lidar_perception_node_component
  src/lidar/lidar_perception_node.cpp
  "br_perception::lidar::LidarPerceptionNode"
  lidar_perception_node
  rclcpp rclcpp_components sensor_msgs visualization_msgs diagnostic_msgs std_msgs pcl_conversions)
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
# 12. CoordinateTransformer — 坐标变换 / 里程计可靠性 (模块 17)
#     辅助库 (构造需 rclcpp::Node*), 供后续 fusion_node 与 test 链接
# ═══════════════════════════════════════════════════════════════════════════
add_library(coordinate_transformer SHARED
  src/fusion/coordinate_transformer.cpp)
ament_target_dependencies(coordinate_transformer
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  tf2 tf2_ros tf2_geometry_msgs)

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
  camera_depth
  camera_perception_node
  perception_pipeline
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
  camera_depth_component
  camera_perception_component
  perception_pipeline_component
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
