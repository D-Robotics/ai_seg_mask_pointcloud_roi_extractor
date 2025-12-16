cmake_minimum_required(VERSION 3.16)

project(ai_seg_mask_pointcloud_roi_extractor VERSION 1.0.0)

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

# ROS2核心依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(ai_msgs REQUIRED)  # 自定义消息依赖

# 第三方库依赖
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(SOURCE_FILES1 
  src/callback.cpp
  src/publish.cpp
  src/set_dynamic_para.cpp
  src/time_stamp.cpp
  src/read_param.cpp
  src/time_sync_detect.cpp
)

# ========== 构建组件库 ==========
add_library(${PROJECT_NAME}_component SHARED
  src/ai_seg_mask_pointcloud_roi_extractor.cpp
  ${SOURCE_FILES1}
)

# 目标包含目录（仅保留必要路径）
target_include_directories(${PROJECT_NAME}_component
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  SYSTEM  # 系统头文件，抑制第三方库警告
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ========== 依赖链接（精简版） ==========
# 先链接ROS2 ament依赖
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  cv_bridge
  rclcpp_components
  sensor_msgs
  pcl_conversions
  message_filters
  ai_msgs
)

# 链接第三方库
target_link_libraries(${PROJECT_NAME}_component
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  Eigen3::Eigen
)

# ========== 注册组件（核心功能保留） ==========
rclcpp_components_register_nodes(${PROJECT_NAME}_component
  PLUGIN "seg_mask_roi_extractor::AISegMaskPointCloudROIExtractor"
#   EXECUTABLE ${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

# ========== 安装规则 ==========
# install(TARGETS 
#   ${PROJECT_NAME}_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(TARGETS 
  ${PROJECT_NAME}_component
  DESTINATION lib/
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY 
  launch 
  config
  DESTINATION share/${PROJECT_NAME}/
)


# ament_export_dependencies(ament_cmake rclcpp ai_msgs PCL OpenCV)
# ament_export_libraries(${PROJECT_NAME}_component)

ament_package()