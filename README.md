# AI Segmentation Mask PointCloud ROI Extractor

[English Version](README_en.md)  |  [中文版本](README.md)

## 1. 项目概述

AI Segmentation Mask PointCloud ROI Extractor 是一个 ROS2 Humble 功能包。它接收 AI 检测结果（分割掩码与边界框）和深度图，通过双管线并行架构，对深度图进行 ROI 过滤或提取，同时生成逐实例 ROI 点云（含 id 与 confidence），为下游机器人感知与决策提供感兴趣的点云或深度图信息。

## 2. 核心功能

- 基于 ROS2 组件（`component_container`）加载，支持进程内零拷贝
- 订阅深度图与 AI 分割检测信息，使用 `ApproximateTime` 时间同步（`allow_timestamp_deviation` 控制最大时间间隔），避免时间戳漂移时静默丢帧
- 根据检测类别与置信度阈值，从深度图提取逐实例 ROI 点云（含 id 与 confidence），并发布过滤后的深度图、掩码图与调试点云
- 接收与计算解耦：同步回调仅做轻量预处理，随后将两阶段任务（过滤发布 / ROI 点云生成）投递到常驻线程池并行执行，提升帧率
- 深度图解析采用 `cv_bridge::toCvShare` 零拷贝；大尺寸图像缓冲通过可复用缓冲池（MatPool）避免每帧堆分配
- 支持动态参数调整（debug、log_level），debug 模式下自动输出各阶段耗时统计

## 3. 目录结构

```
mask_pc_roi_extractor/
├── include/
│   └── mask_pc_roi_extractor/
│       ├── mask_pc_roi_extractor.hpp  # 组件头文件
│       ├── params.hpp                 # 参数结构体定义
│       └── internal_types.hpp         # 内部类型（BoxInfo, MatPool, ThreadPool, FrameJob）
├── src/
│   ├── mask_pc_roi_extractor.cpp  # 组件主实现（初始化、参数、订阅/发布、同步器）
│   ├── callback.cpp                # 同步回调、相机/类别信息回调
│   ├── pipeline.cpp                # 过滤发布管线与 ROI 点云提取管线
│   ├── parsing.cpp                 # 深度/检测信息解析
│   ├── depth_utils.cpp             # 深度图→点云、掩码运算
│   ├── publish.cpp                 # 图像/点云发布
│   ├── params.cpp                  # 参数管理、动态参数回调、类别阈值解析
│   └── time_stamp.cpp              # 时间戳转换工具
├── launch/
│   ├── mask_pc_roi_extractor.py         # 组件启动文件
│   └── seg_mask_parser_config_params.py # 配置解析器
├── msg/
│   ├── ROIPointCloud.msg           # 单 ROI 点云消息定义
│   └── ROIPointClouds.msg          # ROI 点云集合消息定义
├── config/
│   ├── descriptions/
│   │   └── mask_pc_roi_extractor.yaml       # 参数配置描述
│   ├── model_classes_config.yaml            # 深度管线类别置信度阈值
│   └── model_classes_roi_config.yaml        # ROI 管线类别置信度阈值
├── package.xml
├── CMakeLists.txt
├── README.md
└── README_en.md
```

## 4. 依赖项

- **ROS2 Humble**
- **rclcpp**：ROS2 C++ 客户端库
- **rclcpp_components**：ROS2 组件支持
- **sensor_msgs / std_msgs**：标准消息类型
- **cv_bridge**：OpenCV 与 ROS 图像转换
- **message_filters**：时间同步（ApproximateTime）
- **OpenCV 4**：计算机视觉库
- **PCL (Point Cloud Library)**：点云处理库（仅 common 组件）
- **ai_msgs**：自定义 AI 消息类型
- **pcl_conversions / yaml-cpp / Eigen3**：点云转换、配置解析、线性代数
- **ament_index_cpp**：ROS2 包文件路径解析

## 5. 编译说明

### 5.1 创建工作空间

```bash
mkdir -p ./perception/src/
cd ./perception/src/
```

### 5.2 克隆代码库

```bash
git clone https://github.com/D-Robotics/mask_pc_roi_extractor.git
git clone https://github.com/D-Robotics/ai_msgs.git
```

### 5.3 编译

源码编译依赖地瓜交叉编译工具链，详细流程参考：[5.1.3 源码安装 | RDK DOC](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_start/cross_compile)

```bash
# 先编译依赖包 ai_msgs，再编译功能包
bash ./robot_dev_config/build.sh -p X5 -s ai_msgs mask_pc_roi_extractor
```

> **编译优化说明**：aarch64（Horizon RDK X5, Cortex-A55）交叉编译时，自动追加 `-mcpu=cortex-a55` 与 `-flto` 以启用 NEON 自动向量化与链接期优化。

## 6. 运行说明

### 6.1 前置条件

启动前需确保以下节点已在运行：
- 双目深度节点（如 StereoNetNode，提供深度图和 camera_info）
- AI 检测节点（如 yolov8-seg，提供检测结果和 class_info）

### 6.2 启动

```bash
# source 工作空间
source install/setup.bash

# 默认参数运行
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py

# 启用 debug 模式（详细日志 + 每 100 帧耗时统计）
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py debug:=true log_level:=debug

# GDB 调试模式（崩溃时自动打印 backtrace）
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py use_gdb:=True
```

## 7. 参数配置

### 7.1 配置文件说明

- **`config/descriptions/mask_pc_roi_extractor.yaml`**：所有参数的详细描述、类型、默认值和单位
- **`config/model_classes_config.yaml`**：深度管线类别置信度阈值
- **`config/model_classes_roi_config.yaml`**：ROI 管线类别置信度阈值

解析阶段两份配置文件合并为通用阈值（冲突取较低值），各管线再分别用专属阈值做二次过滤。

### 7.2 核心参数

| 参数名称 | 类型 | 默认值 | 描述 | 单位 |
|---------|------|-------|------|------|
| debug | bool | false | 调试模式：输出详细日志 + 每 100 帧平均耗时 | - |
| depth_image_topic | string | "/StereoNetNode/stereonet_depth" | 深度图订阅话题 | - |
| detect_info_topic | string | "/hobot_dnn_detection" | AI 分割检测信息订阅话题 | - |
| class_info_topic | string | "/hobot_dnn_detection_info" | 类别信息订阅话题 | - |
| camera_info_topic | string | "/StereoNetNode/stereonet_depth/camera_info" | 相机信息订阅话题 | - |
| color_image_topic | string | "/StereoNetNode/origin_left_image" | 彩色图话题（预留参数，当前未订阅） | - |
| filtered_mask_topic | string | "/filtered_depth_mask" | 过滤后掩码发布话题（MONO8） | - |
| filtered_depth_topic | string | "/filtered_depth_img" | 过滤后深度图发布话题（MONO16） | - |
| filtered_cloud_topic | string | "/filtered_depth_cloud" | 过滤后点云发布话题（仅 debug 模式） | - |
| roi_cloud_topic | string | "/roi_pointclouds" | ROI 点云发布话题 | - |

### 7.3 高级参数

| 参数名称 | 类型 | 默认值 | 描述 | 单位 |
|---------|------|-------|------|------|
| confidence_threshold_file_path | string | "model_classes_config.yaml" | 深度管线类别置信度阈值文件路径 | - |
| confidence_threshold_roi_file_path | string | "model_classes_roi_config.yaml" | ROI 管线类别置信度阈值文件路径 | - |
| queue_size | int | 10 | 消息同步队列大小 | - |
| allow_timestamp_deviation | double | 0.5 | ApproximateTime 同步的最大时间间隔 | 秒 |
| min_depth | double | 0.0 | 最小深度过滤值 | 米 |
| max_depth | double | 5.0 | 最大深度过滤值 | 米 |
| dilate_iter_num | int | 1 | 形态学操作迭代次数（过滤模式膨胀，提取模式腐蚀） | - |
| camera_width | int | 640 | 相机图像宽度（须与实际输入一致） | 像素 |
| camera_height | int | 352 | 相机图像高度（须与实际输入一致） | 像素 |
| log_level | string | "info" | 日志级别：debug、info、warn、error | - |
| use_extractor | bool | false | 深度管线模式：true=提取（腐蚀 mask），false=过滤（膨胀 mask） | - |
| worker_threads | int | 2 | 常驻工作线程数（上限 3） | - |
| max_pending_frames | int | 2 | 线程池待处理帧上限，超出丢弃最旧帧 | - |

### 7.4 动态参数

以下参数支持运行时动态调整：

```bash
ros2 param set /seg_mask log_level debug    # 切换日志级别
ros2 param set /seg_mask debug true         # 开启调试模式
```

## 8. 发布的话题与消息格式

### 8.1 订阅消息

| 话题 | 消息类型 | 编码 | 描述 |
|------|---------|------|------|
| depth_image_topic | `sensor_msgs/msg/Image` | `MONO16` | 深度图，单位：毫米 |
| detect_info_topic | `ai_msgs/msg/PerceptionTargets` | - | AI 分割检测结果（类别、置信度、边界框、掩码） |
| class_info_topic | `ai_msgs/msg/PerceptionInfo` | - | 检测类别列表 |
| camera_info_topic | `sensor_msgs/msg/CameraInfo` | - | 相机内参 |

### 8.2 发布消息

| 话题（默认值） | 消息类型 | 编码 | 描述 |
|--------------|---------|------|------|
| `/filtered_depth_img` | `sensor_msgs/msg/Image` | `MONO16` | 过滤后深度图，ROI 外深度置零，单位毫米 |
| `/filtered_depth_mask` | `sensor_msgs/msg/Image` | `MONO8` | 过滤后掩码（255=ROI，0=非 ROI） |
| `/filtered_depth_cloud` | `sensor_msgs/msg/PointCloud2` | - | 过滤后点云（仅 debug 模式发布） |
| `/roi_pointclouds` | `mask_pc_roi_extractor/msg/ROIPointClouds` | - | 逐实例 ROI 点云（含 id 和 confidence） |

## 9. 工作流程

### 9.1 初始化阶段
- 创建 ROS2 节点与参数对象
- 声明和获取配置参数
- 初始化发布者与订阅者
- 解析 YAML 配置文件中的类别置信度阈值
- 创建线程池并启动常驻工作线程
- 使用 `message_filters` 建立深度图与检测信息的 ApproximateTime 时间同步

### 9.2 数据接收阶段
- 订阅 camera_info 和 class_info（收到第一帧后各自动取消订阅）
- 深度图与检测结果通过 ApproximateTime 配对后触发回调

### 9.3 数据处理阶段
- 深度图通过 `toCvShare` 零拷贝共享消息缓冲（只读）
- 解析检测信息：合并两份配置文件的通用阈值，一次遍历生成二值掩码和类别 ID 掩码
- 同步回调将两阶段任务投递到线程池后立即返回；工作线程并行执行：
  - **过滤深度图**：用深度专属类别 ID 过滤掩码 → 膨胀/腐蚀 → 置零 ROI 外深度
  - **提取 ROI 点云**：用 ROI 专属置信度二次过滤检测框 → 逐实例点云提取

### 9.4 结果发布阶段
- 发布过滤后的深度图（MONO16）和掩码图（MONO8）
- 发布逐实例 ROI 点云（含 id / confidence）
- debug 模式下额外发布调试点云

## 10. 性能优化

1. **并行流水**：常驻线程池解耦接收与计算，两个阶段真正并行执行
2. **零拷贝**：深度图通过 `toCvShare` 共享消息缓冲，过滤阶段仅克隆一次
3. **缓冲复用**：大尺寸图像通过 MatPool 复用，消除每帧堆分配
4. **微观优化**：掩码单遍生成、投影倒数乘法、点云 `reserve`、NEON 自动向量化（`-mcpu=cortex-a55`）、LTO 链接期优化
5. **形态学操作**：根据分割掩码质量调整 `dilate_iter_num`，过滤模式膨胀扩大覆盖，提取模式腐蚀精确提取

## 11. 调试指南

### 11.1 查看处理耗时（Timing）

debug 模式下节点每 100 帧自动输出一次各阶段平均耗时：

```bash
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py debug:=true
```

日志中会看到：
```
[seg_mask] [timing] last 100 frames avg: filter=2.35 ms (100 samples), roi=1.82 ms (100 samples)
```

- **filter**：过滤管线平均耗时（深度置零 + 掩码生成 + 发布）
- **roi**：ROI 点云提取管线平均耗时

debug 模式下还会额外输出每条消息的发布-订阅延迟（毫秒级）和 depth-detect 时间差。

### 11.2 查看话题帧率与延迟

```bash
# 查看各话题发布频率
ros2 topic hz /filtered_depth_mask
ros2 topic hz /filtered_depth_img
ros2 topic hz /roi_pointclouds

# 查看端到端延迟
ros2 topic delay /filtered_depth_mask
```

### 11.3 检查话题 QoS 匹配

```bash
ros2 topic info /filtered_depth_mask --verbose
```

若 RViz2 收不到消息，最常见原因：
- **QoS 不兼容**（RELIABLE ↔ BEST_EFFORT）→ DDS 无法建立匹配
- **frame_id 不一致** → RViz2 Fixed Frame 需与消息中的 frame_id 一致

### 11.4 在 RViz2 中查看

```bash
rviz2
```

操作步骤：
1. `Add` → `By topic` → `/filtered_depth_mask` → `Image`
2. `Add` → `By topic` → `/filtered_depth_img` → `Image`
3. `Add` → `By topic` → `/filtered_depth_cloud` → `PointCloud2`（需 debug 模式）

> `/filtered_depth_img` 是 MONO16 编码（毫米），RViz2 默认归一化可能导致画面全白。在 Image 插件设置中打开 **Normalize Range**，Min/Max 设为 `0` / `5000`（对应 0-5 米深度范围）。

### 11.5 查看话题原始数据

```bash
# 查看 Image 消息头（不含 data 数组）
ros2 topic echo /filtered_depth_mask --no-arr --once

# 查看特定字段
ros2 topic echo /filtered_depth_mask --once --field header.frame_id --field encoding --field height --field width
```

### 11.6 使用 GDB 调试

```bash
# 启动时自带 GDB（崩溃自动打 backtrace）
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py use_gdb:=True

# attach 到运行中进程
ps aux | grep component_container
gdb -p <PID>
```

### 11.7 查看所有参数

```bash
ros2 param list /seg_mask          # 列出所有参数
ros2 param describe /seg_mask debug # 查看参数描述
ros2 param get /seg_mask min_depth  # 查看当前值
```

## 12. 常见问题

### 12.1 启动后无数据输出

1. 确认双目节点和 AI 检测节点已启动
2. 检查话题是否存在：`ros2 topic list | grep -E "depth|detect|filtered"`
3. 检查话题是否有数据：`ros2 topic hz /StereoNetNode/stereonet_depth`
4. 查看日志中 `Required topics found` 是否成功、`class_names_id` 是否非空

### 12.2 RViz2 收不到消息

1. 检查 QoS 匹配：`ros2 topic info /filtered_depth_mask --verbose`
2. 确认 RViz2 Fixed Frame 设为消息中的 frame_id（通常为 `camera_depth_frame`）
3. 确认话题有数据：`ros2 topic hz /filtered_depth_mask`

### 12.3 时间同步丢帧

增大 `allow_timestamp_deviation` 以容忍更大的时间戳偏差：
```bash
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py allow_timestamp_deviation:=1.0
```

## 13. 许可证

本项目基于 Apache License 2.0 开源。
