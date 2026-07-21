cmake_minimum_required(VERSION 3.16)

project(mask_pc_roi_extractor VERSION 1.0.0)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 精简编译选项（保留关键警告，移除冗余）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wno-anon-structs)

# ===== 性能优化编译选项 =====
# 链接期优化（LTO）：跨编译单元内联，降低调用开销，对 aarch64/x86 均安全
add_compile_options(-flto)
add_link_options(-flto)

# 仅当交叉编译目标为 aarch64（如 Horizon RDK X5, Cortex-A55）时启用针对该 CPU 的
# 微架构优化（NEON 自动向量化、调度调优）。
# 注意：严禁使用 -march=native / -mavx2，那会针对构建宿主机（x86）生成指令，
# 在 ARM 边缘端会直接非法指令崩溃。
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|aarch64-linux|arm64")
  message(STATUS "mask_pc_roi_extractor: targeting aarch64 -> -mcpu=cortex-a55")
  add_compile_options(-mcpu=cortex-a55)
else()
  message(STATUS "mask_pc_roi_extractor: non-aarch64 build, skipping -mcpu (host-neutral)")
endif()

# ROS2核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(ai_msgs REQUIRED)  # 自定义消息依赖
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 第三方库依赖
# 仅链接实际使用的 PCL 组件（common）。io/filters 代码未使用，移除以减小二进制与链接时间
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ========== 自定义消息生成 ==========
set(msg_files
  "msg/ROIPointCloud.msg"
  "msg/ROIPointClouds.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs sensor_msgs
)

# ========== 构建组件库 ==========
add_library(${PROJECT_NAME}_component SHARED
  src/mask_pc_roi_extractor.cpp
  src/callback.cpp
  src/pipeline.cpp
  src/parsing.cpp
  src/depth_utils.cpp
  src/publish.cpp
  src/params.cpp
  src/time_stamp.cpp
)

# 目标包含目录（仅保留必要路径）
target_include_directories(${PROJECT_NAME}_component
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  SYSTEM  # 系统头文件，抑制第三方库警告
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ========== 依赖链接（精简版） ==========
# 先链接ROS2 ament依赖
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  std_msgs
  cv_bridge
  rclcpp_components
  sensor_msgs
  pcl_conversions
  message_filters
  ai_msgs
  ament_index_cpp
)

# 链接第三方库
target_link_libraries(${PROJECT_NAME}_component
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  Eigen3::Eigen
)

# 链接自定义消息
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_component ${cpp_typesupport_target})

# ========== 注册组件（核心功能保留） ==========
rclcpp_components_register_nodes(${PROJECT_NAME}_component
  PLUGIN "robot::mask_pc_roi_extractor::MaskPcRoiExtractor"
#   EXECUTABLE ${PROJECT_NAME}
)

# ========== 安装规则 ==========
install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# 仅保留必要的导出和包配置
ament_export_dependencies(ament_cmake rclcpp ai_msgs PCL OpenCV)
ament_package()