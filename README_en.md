# AI Segmentation Mask PointCloud ROI Extractor

## 1. Project Overview

AI Segmentation Mask PointCloud ROI Extractor is a ROS2 Humble-based package specifically designed to extract or filter Regions of Interest (ROI) from depth images. It utilizes AI segmentation mask technology to accurately separate target objects from point clouds, providing high-quality point cloud data for subsequent robot perception and decision-making.

## 2. Core Features

The package implements the following core features:

- Standard ROS2 node implementation
- Subscribes to depth images and AI detection information with precise timestamp synchronization
- Extracts or filters ROI from point clouds based on detection categories and confidence thresholds
- Depth-filter chain (filtered point cloud, depth image, mask): always on, output feeds obstacle avoidance / navigation
- ROI pointcloud chain (standard PointCloud2 with per-point class_id / confidence / instance_id; ROI overlay image): gated by enable_roi_extraction (bring_up derives it from run_semantic_map) — runs only once semantic mapping is enabled (semantic_map subscribes to roi_cloud_topic)
- Persistent thread pool for parallel frame processing (worker_threads); oldest frame dropped when the queue exceeds max_pending_frames to bound latency
- Supports dynamic parameter adjustment
- Provides comprehensive logging and debugging information

## 2. Directory Structure

```
ai_seg_mask_pointcloud_roi_extractor/
├── include/
│   └── ai_seg_mask_pointcloud_roi_extractor/
│       ├── ai_seg_mask_pointcloud_roi_extractor.hpp  # Component header file
│       ├── params.hpp                                # Parameter structs and declare/get utilities
│       ├── internal_types.hpp                        # Frame jobs, thread pool, drop-oldest queue
│       └── depth_continuity.h                        # Depth continuity check
├── src/
│   ├── ai_seg_mask_pointcloud_roi_extractor.cpp  # Main component implementation
│   ├── callback.cpp                              # Sync callback and task dispatch
│   ├── pipeline.cpp                              # Filter / ROI / overlay-render processing pipeline
│   ├── depth_continuity.cpp                      # Depth continuity check implementation
│   ├── publish.cpp                               # Publishing function implementation
│   ├── read_param.cpp                            # Parameter and class-confidence reading
│   ├── set_dynamic_para.cpp                      # Dynamic parameter handling
│   ├── time_stamp.cpp                            # Timestamp conversion utilities
│   └── time_sync_detect.cpp                      # Time Synchronization Topic Anomaly Detection
├── launch/
│   ├── ai_seg_mask_pointcloud_roi_extractor.py   # Component launch file
│   ├── container.py                              # Component container loading
│   └── seg_mask_parser_config_params.py          # Configuration parser
├── config/
│   ├── descriptions/
│   │   └── ai_seg_mask_pointcloud_roi_extractor.yaml  # Parameter configuration description
│   ├── model_classes_config.yaml                 # Depth-filtering pipeline class confidence thresholds
│   └── model_classes_roi_config.yaml             # ROI-extraction pipeline class confidence thresholds
├── package.xml                         # Package definition file
├── CMakeLists.txt                      # CMake build configuration
├── README.md                           # Chinese documentation
└── README_en.md                        # English documentation
└── image                               # Picture Folder
```

## 3. Dependencies

- **ROS2 Humble**：Robot Operating System
- **rclcpp**：ROS2 C++ client library
- **rclcpp_components**：ROS2 component support
- **sensor_msgs**：Sensor message types
- **std_msgs**：Standard message types
- **cv_bridge**：OpenCV to ROS image conversion
- **image_transport**：Image transport
- **OpenCV**：Computer vision library
- **PCL (Point Cloud Library)**：Point cloud processing library
- **ai_msgs**：Custom AI message types

## 4. Compilation Instructions

### 4.1 Create Workspace

```bash
mkdir -p ./perception/src/
cd ./perception/src/
```

### 4.2 Clone Repository

```bash
git clone https://github.com/D-Robotics/ai_seg_mask_pointcloud_roi_extractor.git
git clone https://github.com/D-Robotics/ai_msgs.git
```

### 4.3 Compile

Source code compilation requires the cross-compilation toolchain provided by DiGua. For detailed instructions, refer to: [5.1.3 Source Code Installation | RDK DOC](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_start/cross_compile)

```bash
# First compile ai_msgs
bash ./robot_dev_config/build.sh -p X5 -s ai_msgs
# Then compile the package
bash ./robot_dev_config/build.sh -p X5 -s ai_seg_mask_pointcloud_roi_extractor
```

## 5. Running Instructions

### 5.1 Run Using Launch File

```bash
# Source the workspace
source install/setup.bash

# Run with default parameters
ros2 launch ai_seg_mask_pointcloud_roi_extractor ai_seg_mask_pointcloud_roi_extractor.py

# Run with custom parameters
ros2 launch ai_seg_mask_pointcloud_roi_extractor ai_seg_mask_pointcloud_roi_extractor.py debug:=true log_level:=debug
```
**NOTE**: Before launching, you need to start the stereo camera and yolov8-seg nodes to ensure that depth images and detection information can be properly subscribed.

## 6. Parameter Configuration

### 6.1 Configuration File Description

The project uses YAML files for parameter configuration. The main configuration files are:

- **`config/descriptions/ai_seg_mask_pointcloud_roi_extractor.yaml`**: Contains detailed descriptions, types, default values, and units for all parameters
- **`config/model_classes_config.yaml`**: Class confidence thresholds for the depth-filtering pipeline
- **`config/model_classes_roi_config.yaml`**: Class confidence thresholds for the ROI-extraction pipeline (class names come from YOLO's 80 COCO classes, see tros_nav_workflow/bring_up/params/config/coco.list); the semantic_map class whitelist shares this file

The parsing stage merges both class configs into a common threshold map (lower confidence wins on conflict); each pipeline then applies its own thresholds as a second filter.

### 6.2 Parameter Description

### 6.2 Core Parameters

| Parameter Name | Type | Default Value | Description | Unit |
|---------------|------|---------------|-------------|------|
| debug | bool | false | Enable debug mode to output more detailed log information | - |
| time_debug | bool | false | Enable debug mode to output more detailed time information | - |
| topic_check | bool | false | Subscribe topic existence detection | - |
| depth_image_topic | string | "/StereoNetNode/stereonet_depth" | Depth image subscription topic | - |
| detect_info_topic | string | "/hobot_dnn_detection" | AI segmentation detection information subscription topic | - |
| class_info_topic | string | "/hobot_dnn_detection_info" | Class information subscription topic | - |
| camera_info_topic | string | "/StereoNetNode/stereonet_depth/camera_info" | Camera information subscription topic | - |
| color_image_topic | string | "/StereoNetNode/origin_left_image" | Color image subscription topic | - |
| filtered_mask_topic | string | "/filtered_depth_mask" | Filtered mask publishing topic | - |
| filtered_depth_topic | string | "/filtered_depth_img" | Filtered depth image publishing topic | - |
| filtered_cloud_topic | string | "/filtered_depth_cloud" | Filtered point cloud publishing topic | - |
| roi_cloud_topic | string | "/roi/pointclouds" | ROI pointcloud publish topic (standard PointCloud2 with per-point class_id/confidence/instance_id fields; semantic-map chain only) | - |
| roi_visual_topic | string | "/roi/visual_depth_seg" | ROI segment overlay depth render topic (bgr8; jpeg-encoded to web channel 2; empty string disables) | - |

### 6.3 Configuration Parameters

| Parameter Name | Type | Default Value | Description | Unit |
|---------------|------|---------------|-------------|------|
| confidence_threshold_file_path | string | "model_classes_config.yaml" | Class confidence threshold file path | - |
| queue_size | int | 10 | Message synchronization queue size, affecting real-time performance and stability | - |
| allow_timestamp_deviation | double | 0.5 | Allowed timestamp deviation, messages exceeding this value will be discarded | seconds |
| min_depth | double | 0.0 | Minimum depth filter value, points below this value will be filtered | meters |
| max_depth | double | 5.0 | Maximum depth filter value, points above this value will be filtered | meters |
| dilate_iter_num | int | 1 | Mask dilation iteration count, affecting the size of ROI area | - |
| camera_width | int | 640 | Camera image width, must match actual input image | pixels |
| camera_height | int | 352 | Camera image height, must match actual input image | pixels |
| log_level | string | "info" | Log level, available options: debug, info, warn, error, critical | - |
| confidence_threshold_roi_file_path | string | "model_classes_roi_config.yaml" | ROI-extraction pipeline class-confidence threshold file path | - |
| erode_iter_num_roi | int | 1 | Erosion iterations on the ROI-dedicated segmentation mask before ROI pointcloud extraction (0=disabled; ROI/semantic-map chain only, the depth-filter chain is unaffected) | - |
| erode_extra_class_names | string | "chair" | Comma-separated class names receiving extra erosion (ROI/semantic-map chain only) | - |
| erode_extra_iter_num | int | 2 | Extra erosion iterations for erode_extra_class_names (0=disabled) | - |
| depth_continuity_check | bool | false | ROI pointcloud depth continuity check (debug switch, off by default) | - |
| max_depth_diff | double | 0.3 | Max allowed depth difference from neighborhood median in the continuity check | meters |
| instance_depth_gate_tau | double | 0.3 | Instance-level adaptive percentile depth gate minimum margin (0=disabled) | meters |
| use_extractor | bool | false | Whether to perform ROI extraction; true:=extraction, false:=filtering | - |
| enable_roi_extraction | bool | true | Master switch for the ROI/semantic-map chain; false=skip ROI extraction and render tasks (~135 ms worker time per frame saved), keeping only the depth-filter chain. bring_up derives it from run_semantic_map | - |
| worker_threads | int | 2 | Persistent worker threads (max 3 to avoid starving BPU cores) | - |
| max_pending_frames | int | 4 | Thread-pool pending frame cap; oldest frame dropped when exceeded (must be >= 3 tasks per frame) | - |

### 6.4 Dynamic Parameters

Parameters that support runtime dynamic adjustment:

- **debug**: Enable/disable debug mode
- **log_level**: Set log level (debug/info/warn/error/critical)

```bash
# Dynamically adjust log level
ros2 param set /seg_mask log_level debug

# Topic ckeck
ros2 param set /seg_mask topic_check true

# Dynamically enable debug mode
ros2 param set /seg_mask debug true


```

## 7. Message Format

### 7.1 Subscribed Messages

| Topic Name | Message Type | Description |
|-----------|-------------|-------------|
| depth_image_topic | `sensor_msgs/msg/Image` | Depth image input (encoding: MONO16 or TYPE_16UC1, unit: millimeters) |
| detect_info_topic | `ai_msgs::msg::PerceptionTargets` | AI segmentation detection information (target class, confidence, bounding box, and mask data) |
| class_info_topic | `ai_msgs::msg::PerceptionInfo` | Detection class information (all possible detection classes) |
| camera_info_topic | `sensor_msgs/msg/CameraInfo` | Camera parameter information (intrinsic parameters, distortion coefficients, etc.) |
| color_image_topic | `sensor_msgs/msg/Image` | Color image input |

### 7.2 Published Messages

| Topic Name | Message Type | Description |
|-----------|-------------|-------------|
| filtered_cloud_topic | `sensor_msgs/msg/PointCloud2` | Filtered point cloud containing only ROI area data |
| filtered_depth_topic | `sensor_msgs/msg/Image` | Filtered depth image containing only ROI area information |
| filtered_mask_topic | `sensor_msgs/msg/Image` | Filtered binary mask (1 = ROI area, 0 = non-ROI area) |
| roi_cloud_topic | `sensor_msgs/msg/PointCloud2` | ROI pointcloud with per-point fields: x/y/z (float32), class_id (int32), confidence (float32), instance_id (int32, per-frame box index), consumed by semantic_map |
| roi_visual_topic | `sensor_msgs/msg/Image` | ROI segment overlay depth render (bgr8: JET depth rendering + ROI mask overlay) |

## 8. Workflow

### 8.1 Initialization Phase
- Create ROS2 node and parameter objects
- Declare and retrieve configuration parameters
- Initialize publishers and subscribers
- Parse class confidence thresholds from YAML configuration files through read_param.cpp

### 8.2 Data Receiving Phase
- Subscribe to camera information and class information
- Use message_filters for timestamp synchronization of depth images and detection information

### 8.3 Parameter Configuration Parsing
- Set target detection parameters based on subscribed class information and configuration files
- Support dynamically adjustable parameter configuration

### 8.4 Data Processing Phase
- Convert ROS messages to OpenCV image format
- Parse detection information to generate segmentation masks
- Perform dilation processing on masks
- Filter depth images based on masks
- Convert depth images to point clouds
- A persistent thread pool runs three tasks per frame in parallel (depth filtering, ROI pointcloud extraction, ROI overlay rendering); oldest frames are dropped when the queue overflows
- ROI chain: per-class extra erosion and the instance-level adaptive percentile depth gate remove boundary artifacts and radial tail streaks
- Zero-copy sharing of the depth message buffer, MatPool reuse of large image buffers, single-pass mask generation

### 8.5 Result Publishing Phase
- Publish filtered depth images
- Publish filtered mask images
- Publish filtered point clouds
- Publish the ROI pointcloud (standard PointCloud2) and the ROI overlay render (when roi_visual_topic is non-empty)

## 9. Notes

1. **Time Synchronization**: Depth images and detection information must be timestamp-aligned; otherwise, processing will be skipped
2. **Image Size**: The size of depth images and segmentation masks must match the configured camera_width and camera_height
3. **Depth Unit**: Depth images are assumed to be in millimeters, which are converted to meters for processing
4. **Class Configuration**: model_classes_config.yaml (depth filtering) and model_classes_roi_config.yaml (ROI extraction) must be configured according to the actual AI model classes (YOLO's 80 COCO classes, see tros_nav_workflow/bring_up/params/config/coco.list)
5. **Performance Considerations**: When processing high-resolution images, adjust queue_size and allow_timestamp_deviation parameters as needed

## 10. Performance Optimization

1. **Adjust Queue Size**: Modify the queue_size parameter based on system performance and message frequency
2. **Optimize Depth Range**: Adjust min_depth and max_depth parameters to match application requirements and reduce unnecessary calculations
3. **Adjust Dilation Iterations**: Tune the dilate_iter_num parameter based on segmentation mask quality

## 11. Debugging Methods

### 11.1 Set Log Level

```bash
# Set log level at runtime
ros2 launch ai_seg_mask_pointcloud_roi_extractor ai_seg_mask_pointcloud_roi_extractor.py log_level:=debug

# Dynamically adjust log level
ros2 param set /seg_mask log_level debug

# Dynamically check time
ros2 param set /seg_mask time_debug true
```

### 11.3 Visualize the ROI Pointcloud (foxglove / RViz)

`roi_cloud_topic` publishes a standard `sensor_msgs/msg/PointCloud2` that foxglove 3D panels and RViz render natively — no conversion script needed. In foxglove, color by the `class_id` field to tell classes apart. The output is in the camera optical frame — enable /tf and /tf_static in foxglove, or set the panel's fixed frame to the message frame.

![ROI pointcloud rendered in the foxglove 3D panel: the rainbow blob is a bottle instance cloud (colored by per-point class_id), the grid is the semantic_map label map, all in the map frame](image/roi_pointcloud_foxglove.png)


### 11.2 Check Parameters

```bash
# View all parameters
ros2 param list /seg_mask

# View specific parameter values
ros2 param get /seg_mask debug

# Set parameter values
ros2 param set/seg_mask debug true
```

## 12. Resource Occupancy

| Development Board Model |  CPU Frequency | CPU Usage | Average Frame Rate of Topic Publishing | Time-Delay|
|---------|---------|---------|---------|---------|
| RDK-X5 CPU (8核) | 1500 | Approximately-28% | Approximately-13HZ| Approximately-7ms|

## 13. License

This project is open source under the Apache License 2.0.

## 14. Contact Information

For questions or suggestions, please contact the project maintainers.

