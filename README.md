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
└── image                               # 图片文件夹
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
| time_debug | bool | false | 是否启用查看各个模块的运行时间，启用后会输出更详细的日志信息 | - |
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

# 查看各个模块的运行时间
ros2 param set /seg_mask time_debug true
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

## 12. 资源占用

| 开发板型号 |  CPU频率 | CPU占用 | 话题发布平均帧率 | Time-Delay|
|---------|---------|---------|---------|---------|
| RDK-X5 CPU (8核) | 1500 | Approximately-28% | Approximately-13HZ| Approximately-7ms|


## 13. 许可证

本项目基于Apache License 2.0开源。

## 14. 联系方式

如有问题或建议，请联系项目维护人员。

## 15. 重构版新增功能

本版本将重构后继版（mask_pc_roi_extractor）的全部功能并回本仓库，保留原有文件结构与命名。相对旧版的新增功能如下。

### 15.1 逐实例 ROI 点云输出（semantic_map 联动）

- 新增自定义消息 `msg/ROIPointCloud.msg` / `msg/ROIPointClouds.msg`；
- 新话题 `/roi_pointclouds`（参数 `roi_cloud_topic`）发布逐实例 ROI 点云（含实例 id 与 confidence），供下游 semantic_map 节点构建语义地图；
- 新增 ROI 可视化叠加话题 `/roi_visual_depth_seg`（参数 `roi_visual_topic`，bgr8 深度+分割叠加图；置为空字符串可禁用该发布者）。

### 15.2 双类别白名单

- `config/model_classes_config.yaml`：深度过滤管线类别置信度阈值；
- `config/model_classes_roi_config.yaml`：ROI 提取管线类别置信度阈值；
- 解析阶段将两份配置合并为通用阈值（冲突取较低置信度），之后各管线再分别用专属阈值二次过滤。

### 15.3 按类腐蚀与实例级深度门控

- 全局：21×21 椭圆核腐蚀类别 ID 分割掩码，去除边界边缘伪影；
- 按类附加腐蚀：`erode_extra_class_names`（默认 `chair`）+ `erode_extra_iter_num`（默认 2），仅作用于 ROI 点云/语义地图链路，不影响障碍物管线；
- 实例级自适应百分位深度门控 `instance_depth_gate_tau`（默认 0.3 m，0=禁用）：保留带 = [p10 − m, p90 + m]，m 随实例自身深度跨度扩大，既保留大物体的远端边缘，又能整段切除背景渗透形成的径向尾状噪声；
- 深度连续性检查 `depth_continuity_check`（默认关闭，调试开关）：比较像素深度与同类 3×3 邻域中位数，过滤物体边界处深度突变的背景像素。

### 15.4 多线程帧处理

- 常驻线程池（`worker_threads`，1–3，默认 2），同步回调投递任务后立即返回，executor 持续接收帧；
- 待处理帧数超过 `max_pending_frames`（默认 2）时丢弃最旧帧；
- 过滤深度输出与 ROI 点云提取两个阶段在工作线程上并行执行。

### 15.5 性能优化

- `toCvShare` 零拷贝共享深度图消息缓冲；
- MatPool 复用大图像缓冲，消除每帧堆分配；
- 掩码单遍生成、投影倒数乘法、点云 `reserve`；
- 编译优化：LTO、aarch64 上 `-mcpu=cortex-a55`（NEON 自动向量化）。

### 15.6 独立可执行与调试支持

- 除组件库外构建独立可执行文件 `${PROJECT_NAME}_node`（`src/main.cpp`）；
- launch 支持 `use_gdb:=True`（GDB 包裹启动节点，崩溃自动打印 backtrace）；
- `debug` 模式输出详细日志 + 每 100 帧平均单帧耗时；
- 恢复时间同步看门狗（time_sync_detect）：无同步数据或帧间隔停滞时告警；停滞阈值 `sync_time_delta_` 由旧版 0.5 s 调整为 5.0 s，以适配低帧率深度源（如 0.43 Hz 的 stereonet_depth_filtered）。

### 15.7 新增参数一览

| 参数 | 类型 | 默认值 | 描述 |
|------|------|-------|------|
| roi_cloud_topic | string | "/roi_pointclouds" | ROI 点云发布话题 |
| roi_visual_topic | string | "/roi_visual_depth_seg" | ROI 可视化叠加话题（空字符串禁用） |
| confidence_threshold_roi_file_path | string | "model_classes_roi_config.yaml" | ROI 管线类别置信度配置文件 |
| erode_iter_num_roi | int | 1 | ROI 掩码腐蚀迭代次数 |
| erode_extra_class_names | string | "chair" | 需附加腐蚀的类名列表（逗号分隔） |
| erode_extra_iter_num | int | 2 | 附加腐蚀迭代次数 |
| depth_continuity_check | bool | false | 深度连续性检查（调试开关，默认关闭） |
| max_depth_diff | double | 0.3 | 连续性检查允许的邻域中位数最大深度差（米） |
| instance_depth_gate_tau | double | 0.3 | 实例级百分位深度门控最小裕量（米，0=禁用） |
| worker_threads | int | 2 | 常驻工作线程数（上限 3） |
| max_pending_frames | int | 2 | 线程池待处理帧上限，超出丢弃最旧帧 |

> 完整参数说明见 `config/descriptions/ai_seg_mask_pointcloud_roi_extractor.yaml`。