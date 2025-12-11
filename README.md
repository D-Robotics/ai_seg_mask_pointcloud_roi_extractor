# AI Segmentation Mask PointCloud ROI Extractor

[English Version](README_en.md)  |  [中文版本](README.md)

## 1. 项目概述

AI Segmentation Mask PointCloud ROI Extractor是一个基于ROS2 Humble的功能包，专门用于从深度图中提取或过滤感兴趣区域（ROI）。它利用AI分割掩码技术，能够精确地从深度图或者点云中提取或者过滤出目标对象，为后续的机器人感知和决策提供感兴趣的点云或者深度图信息。

## 2. 核心功能

该功能包实现了以下核心功能：

- 使用标准ROS2节点实现
- 订阅深度图和AI检测信息，确保时间戳精确对齐
- 根据检测类别和置信度阈值从点云中提取ROI
- 发布过滤后的点云、深度图和掩码图像
- 支持动态参数调整
- 提供丰富的日志和调试信息

## 2. 目录结构

```
ai_seg_mask_pointcloud_roi_extractor/
├── include/
│   └── ai_seg_mask_pointcloud_roi_extractor/
│       └── ai_seg_mask_pointcloud_roi_extractor.hpp  # 组件头文件
├── src/
│   ├── ai_seg_mask_pointcloud_roi_extractor.cpp  # 组件主实现
│   ├── callback.cpp                              # 回调函数实现
│   ├── publish.cpp                               # 发布函数实现
│   ├── read_param.cpp                            # 参数读取实现
│   ├── set_dynamic_para.cpp                      # 动态参数处理
│   └── time_stamp.cpp                            # 时间戳转换工具
|   |___ time_sync_detect.cpp                     # 时间同步话题异常检测
├── launch/
│   ├── ai_seg_mask_pointcloud_roi_extractor.py   # 组件启动文件
│   └── parser_config_params.py                   # 配置解析器
├── config/
│   ├── descriptions/
│   │   └── ai_seg_mask_pointcloud_roi_extractor.yaml  # 参数配置描述
│   └── model_classes_config.yaml                 # 类别置信度阈值配置
├── package.xml                         # 包定义文件
├── CMakeLists.txt                      # CMake构建文件
├── README.md                           # 中文文档
└── README_en.md                        # 英文文档
```

## 3. 依赖项

- **ROS2 Humble**：机器人操作系统
- **rclcpp**：ROS2 C++客户端库
- **rclcpp_components**：ROS2组件支持
- **sensor_msgs**：传感器消息类型
- **std_msgs**：标准消息类型
- **cv_bridge**：OpenCV与ROS图像转换
- **image_transport**：图像传输
- **OpenCV**：计算机视觉库
- **PCL (Point Cloud Library)**：点云处理库
- **ai_msgs**：自定义AI消息类型

## 4. 编译说明

### 4.1 创建工作空间

```bash
mkdir -p ./perception/src/
cd ./perception/src/
```

### 4.2 克隆代码库

```bash
git clone https://github.com/D-Robotics/ai_seg_mask_pointcloud_roi_extractor.git
git clone https://github.com/D-Robotics/ai_msgs.git
```

### 4.3 编译

源码编译依赖于地瓜提供的交叉编译工具链，详细流程参考链接：[5.1.3 源码安装 | RDK DOC](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_start/cross_compile)

```bash
# 先编译ai_msgs
bash ./robot_dev_config/build.sh -p X5 -s ai_msgs
# 再编译功能包
bash ./robot_dev_config/build.sh -p X5 -s ai_seg_mask_pointcloud_roi_extractor
```

## 5. 运行说明

### 5.1 使用Launch文件运行

```bash
# source 工作空间
source install/setup.bash

# 使用默认参数运行
ros2 launch ai_seg_mask_pointcloud_roi_extractor ai_seg_mask_pointcloud_roi_extractor.py

# 自定义参数运行
ros2 launch ai_seg_mask_pointcloud_roi_extractor ai_seg_mask_pointcloud_roi_extractor.py debug:=true log_level:=debug
```
NODE : 启动之前需要先启动双目以及yolov8-seg节点，确保深度图和检测信息能够被正常订阅。


## 6. 参数配置

### 6.1 配置文件说明

项目使用YAML文件进行参数配置，主要配置文件包括：

- **`config/descriptions/ai_seg_mask_pointcloud_roi_extractor.yaml`**：包含所有参数的详细描述、类型、默认值和单位
- **`config/model_classes_config.yaml`**：配置各类别的置信度阈值

### 6.2 参数说明

### 6.2 核心参数说明

| 参数名称 | 类型 | 默认值 | 描述 | 单位 |
|---------|------|-------|------|------|
| debug | bool | false | 是否启用调试模式，启用后会输出更详细的日志信息 | - |
| topic_check | bool | false | 订阅话题存在性检测 | - |
| depth_image_topic | string | "/StereoNetNode/stereonet_depth" | 深度图订阅话题 | - |
| detect_info_topic | string | "/hobot_dnn_detection" | AI分割检测信息订阅话题 | - |
| class_info_topic | string | "/hobot_dnn_detection_info" | 类别信息订阅话题 | - |
| camera_info_topic | string | "/StereoNetNode/stereonet_depth/camera_info" | 相机信息订阅话题 | - |
| color_image_topic | string | "/StereoNetNode/origin_left_image" | 彩色图订阅话题 | - |
| filtered_mask_topic | string | "/filtered_depth_mask" | 过滤后的掩码发布话题 | - |
| filtered_depth_topic | string | "/filtered_depth_img" | 过滤后的深度图发布话题 | - |
| filtered_cloud_topic | string | "/filtered_depth_cloud" | 过滤后的点云发布话题 | - |

### 6.3 配置参数说明

| 参数名称 | 类型 | 默认值 | 描述 | 单位 |
|---------|------|-------|------|------|
| confidence_threshold_file_path | string | "model_classes_config.yaml" | 类别置信度阈值文件路径 | - |
| queue_size | int | 10 | 消息同步队列大小，影响消息处理的实时性和稳定性 | - |
| allow_timestamp_deviation | double | 0.5 | 允许的时间戳偏差，超过此值的消息会被丢弃 | 秒 |
| min_depth | double | 0.0 | 最小深度过滤值，小于此值的点会被过滤 | 米 |
| max_depth | double | 5.0 | 最大深度过滤值，大于此值的点会被过滤 | 米 |
| dilate_iter_num | int | 1 | 掩码膨胀迭代次数，影响ROI区域的大小 | - |
| camera_width | int | 640 | 相机图像宽度，必须与实际输入图像一致 | 像素 |
| camera_height | int | 352 | 相机图像高度，必须与实际输入图像一致 | 像素 |
| log_level | string | "info" | 日志级别，可选值：debug、info、warn、error、critical | - |

### 6.4 动态参数

支持在运行时动态调整的参数：

- **debug**：启用/禁用调试模式
- **topic_check**：订阅话题存在性检测
- **log_level**：设置日志级别（debug/info/warn/error/critical）

```bash
# 动态调整日志级别
ros2 param set /seg_mask log_level debug

# 话题检测
ros2 param set /seg_mask topic_check true

# 动态启用调试模式
ros2 param set /seg_mask debug true
```

## 7. 消息格式

### 7.1 订阅消息

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| depth_image_topic | `sensor_msgs/msg/Image` | 深度图输入，编码：MONO16或TYPE_16UC1，单位：毫米 |
| detect_info_topic | `ai_msgs::msg::PerceptionTargets` | AI分割检测信息，包含目标类别、置信度、边界框和掩码数据 |
| class_info_topic | `ai_msgs::msg::PerceptionInfo` | 检测类别信息，包含所有可能的检测类别 |
| camera_info_topic | `sensor_msgs/msg/CameraInfo` | 相机参数信息，包含内参、畸变系数等 |
| color_image_topic | `sensor_msgs/msg/Image` | 彩色图像输入 |

### 7.2 发布消息

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| filtered_cloud_topic | `sensor_msgs/msg/PointCloud2` | 过滤后的点云，仅包含ROI区域的点云数据 |
| filtered_depth_topic【最终发布的话题】 | `sensor_msgs/msg/Image` | 过滤后的深度图，仅包含ROI区域的深度信息 |
| filtered_mask_topic | `sensor_msgs/msg/Image` | 过滤后的掩码，二值图像，1表示ROI区域，0表示非ROI区域 |

## 8. 工作流程

### 8.1 初始化阶段
- 创建ROS2节点和参数对象
- 声明和获取配置参数
- 初始化发布者和订阅者
- 通过read_param.cpp解析YAML配置文件中的类别置信度阈值

### 8.2 数据接收阶段
- 订阅相机信息和类别信息
- 使用message_filters实现深度图和检测信息的时间同步

### 8.3 参数配置解析
- 根据订阅的类别信息和配置文件设置目标检测参数
- 支持动态调整的参数配置

### 8.4 数据处理阶段
- 将ROS消息转换为OpenCV图像格式
- 解析检测信息生成分割掩码
- 对掩码进行膨胀处理
- 根据掩码过滤深度图
- 将深度图转换为点云

### 8.5 结果发布阶段
- 发布过滤后的深度图
- 发布过滤后的掩码图像
- 发布过滤后的点云

## 9. 注意事项

1. **时间同步**：深度图和检测信息必须时间戳对齐，否则会被跳过处理
2. **图像尺寸**：深度图和分割掩码的尺寸必须与配置的camera_width和camera_height一致
3. **深度单位**：默认假设深度图单位为毫米，转换为米进行处理
4. **类别配置**：model_classes_config.yaml文件需要根据实际的AI模型类别进行配置
5. **性能考虑**：处理大分辨率图像时可能需要调整队列大小和时间戳偏差参数

## 10. 性能优化

1. **调整队列大小**：根据系统性能和消息频率调整queue_size参数
2. **优化深度范围**：根据实际应用场景调整min_depth和max_depth参数，减少不必要的计算
3. **调整膨胀迭代次数**：根据分割掩码的质量调整dilate_iter_num参数

## 11. 调试方法

### 11.1 设置日志级别

```bash
# 运行时设置日志级别
ros2 launch ai_seg_mask_pointcloud_roi_extractor ai_seg_mask_pointcloud_roi_extractor.py log_level:=debug

# 动态调整日志级别
ros2 param set /seg_mask log_level debug
```

### 11.2 检查参数

```bash
# 查看所有参数
ros2 param list /seg_mask

# 查看特定参数值
ros2 param get /seg_mask debug

# 设置参数值
ros2 param set /seg_mask debug true
```

## 12. 许可证

本项目基于Apache License 2.0开源。

## 13. 联系方式

如有问题或建议，请联系项目维护人员。