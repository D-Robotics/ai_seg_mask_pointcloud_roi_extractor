# AI Segmentation Mask PointCloud ROI Extractor

[English Version](README_en.md)  |  [中文版本](README.md)

## 1. Project Overview

AI Segmentation Mask PointCloud ROI Extractor is a ROS2 Humble package. It ingests AI detection results (segmentation masks and bounding boxes) together with depth images, runs a dual-pipeline parallel architecture to filter or extract ROIs on the depth map, and simultaneously produces per-instance ROI point clouds (with id and confidence) for downstream robot perception and decision-making.

## 2. Core Features

- ROS2 component (`component_container`) with intra-process zero-copy
- Subscribes to depth images and AI segmentation detection info, synchronized via `ApproximateTime` policy (max interval controlled by `allow_timestamp_deviation`), avoiding silent frame loss under timestamp drift
- Extracts per-instance ROI point clouds (with id and confidence) based on detection classes and confidence thresholds; publishes filtered depth images, mask images, and debug point clouds
- Decoupled reception and computation: the synchronized callback performs lightweight preprocessing only, then dispatches two-stage tasks (filter publish / ROI point cloud generation) to a persistent thread pool for parallel execution
- Zero-copy depth image parsing via `cv_bridge::toCvShare`; large image buffers reused through a pool (MatPool) to eliminate per-frame heap allocation
- Dynamic parameter adjustment (debug, log_level) with automatic per-stage timing reports in debug mode

## 3. Directory Structure

```
mask_pc_roi_extractor/
├── include/
│   └── mask_pc_roi_extractor/
│       ├── mask_pc_roi_extractor.hpp  # Component header
│       ├── params.hpp                 # Parameter struct definition
│       └── internal_types.hpp         # Internal types (BoxInfo, MatPool, ThreadPool, FrameJob)
├── src/
│   ├── mask_pc_roi_extractor.cpp  # Main component (init, params, subscribers, publishers, synchronizer)
│   ├── callback.cpp                # Synchronized callback, camera/class info callbacks
│   ├── pipeline.cpp                # Filter publish pipeline & ROI point cloud extraction pipeline
│   ├── parsing.cpp                 # Depth/detection info parsing
│   ├── depth_utils.cpp             # Depth→point cloud conversion, mask operations
│   ├── publish.cpp                 # Image/point cloud publishing
│   ├── params.cpp                  # Parameter management, dynamic callbacks, class threshold parsing
│   └── time_stamp.cpp              # Timestamp conversion utilities
├── launch/
│   ├── mask_pc_roi_extractor.py         # Component launch file
│   └── seg_mask_parser_config_params.py # Configuration parser
├── msg/
│   ├── ROIPointCloud.msg           # Single ROI point cloud message
│   └── ROIPointClouds.msg          # Aggregate ROI point clouds message
├── config/
│   ├── descriptions/
│   │   └── mask_pc_roi_extractor.yaml       # Parameter configuration description
│   ├── model_classes_config.yaml            # Depth pipeline class confidence thresholds
│   └── model_classes_roi_config.yaml        # ROI pipeline class confidence thresholds
├── package.xml
├── CMakeLists.txt
├── README.md
└── README_en.md
```

## 4. Dependencies

- **ROS2 Humble**
- **rclcpp**: ROS2 C++ client library
- **rclcpp_components**: ROS2 component support
- **sensor_msgs / std_msgs**: Standard message types
- **cv_bridge**: OpenCV ↔ ROS image conversion
- **message_filters**: Time synchronization (ApproximateTime)
- **OpenCV 4**: Computer vision library
- **PCL (Point Cloud Library)**: Point cloud processing (common component only)
- **ai_msgs**: Custom AI message types
- **pcl_conversions / yaml-cpp / Eigen3**: Point cloud conversion, config parsing, linear algebra
- **ament_index_cpp**: ROS2 package file path resolution

## 5. Compilation

### 5.1 Create Workspace

```bash
mkdir -p ./perception/src/
cd ./perception/src/
```

### 5.2 Clone Repositories

```bash
git clone https://github.com/D-Robotics/mask_pc_roi_extractor.git
git clone https://github.com/D-Robotics/ai_msgs.git
```

### 5.3 Build

Source compilation requires the D-Robotics cross-compilation toolchain. For detailed instructions, refer to: [5.1.3 Source Installation | RDK DOC](https://developer.d-robotics.cc/rdk_doc/Robot_development/quick_start/cross_compile)

```bash
# Build dependency (ai_msgs) first, then the package
bash ./robot_dev_config/build.sh -p X5 -s ai_msgs mask_pc_roi_extractor
```

> **Build optimization**: When cross-compiling for aarch64 (Horizon RDK X5, Cortex-A55), CMake automatically appends `-mcpu=cortex-a55` and `-flto` to enable NEON auto-vectorization and link-time optimization.

## 6. Running

### 6.1 Prerequisites

The following nodes must be running before launching:
- Stereo depth node (e.g. StereoNetNode, providing depth images and camera_info)
- AI detection node (e.g. yolov8-seg, providing detection results and class_info)

### 6.2 Launch

```bash
# Source the workspace
source install/setup.bash

# Run with default parameters
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py

# Enable debug mode (detailed logs + per-100-frame timing report)
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py debug:=true log_level:=debug

# Run under GDB (auto-prints backtrace on crash)
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py use_gdb:=True
```

## 7. Parameter Configuration

### 7.1 Configuration Files

- **`config/descriptions/mask_pc_roi_extractor.yaml`**: Detailed descriptions, types, defaults, and units for all parameters
- **`config/model_classes_config.yaml`**: Depth pipeline class confidence thresholds
- **`config/model_classes_roi_config.yaml`**: ROI pipeline class confidence thresholds

The two config files are merged at the parsing stage into a common threshold map (union; min confidence on conflict). Each pipeline then applies its own threshold for secondary filtering.

### 7.2 Core Parameters

| Parameter | Type | Default | Description | Unit |
|-----------|------|---------|-------------|------|
| debug | bool | false | Debug mode: detailed logs + per-100-frame timing stats | - |
| depth_image_topic | string | "/StereoNetNode/stereonet_depth" | Depth image subscription topic | - |
| detect_info_topic | string | "/hobot_dnn_detection" | AI detection info subscription topic | - |
| class_info_topic | string | "/hobot_dnn_detection_info" | Class info subscription topic | - |
| camera_info_topic | string | "/StereoNetNode/stereonet_depth/camera_info" | Camera info subscription topic | - |
| color_image_topic | string | "/StereoNetNode/origin_left_image" | Color image topic (reserved, not currently subscribed) | - |
| filtered_mask_topic | string | "/filtered_depth_mask" | Filtered mask publish topic (MONO8) | - |
| filtered_depth_topic | string | "/filtered_depth_img" | Filtered depth image publish topic (MONO16) | - |
| filtered_cloud_topic | string | "/filtered_depth_cloud" | Filtered point cloud publish topic (debug mode only) | - |
| roi_cloud_topic | string | "/roi_pointclouds" | ROI point clouds publish topic | - |

### 7.3 Advanced Parameters

| Parameter | Type | Default | Description | Unit |
|-----------|------|---------|-------------|------|
| confidence_threshold_file_path | string | "model_classes_config.yaml" | Depth pipeline class confidence threshold file | - |
| confidence_threshold_roi_file_path | string | "model_classes_roi_config.yaml" | ROI pipeline class confidence threshold file | - |
| queue_size | int | 10 | Message sync queue size | - |
| allow_timestamp_deviation | double | 0.5 | Max interval for ApproximateTime sync | sec |
| min_depth | double | 0.0 | Minimum depth filter value | m |
| max_depth | double | 5.0 | Maximum depth filter value | m |
| dilate_iter_num | int | 1 | Morphological iterations (dilate in filter mode, erode in extract mode) | - |
| camera_width | int | 640 | Camera image width (must match actual input) | px |
| camera_height | int | 352 | Camera image height (must match actual input) | px |
| log_level | string | "info" | Log level: debug, info, warn, error | - |
| use_extractor | bool | false | Depth pipeline mode: true=extract (erode mask), false=filter (dilate mask) | - |
| worker_threads | int | 2 | Persistent worker thread count (max 3) | - |
| max_pending_frames | int | 2 | Max pending frames in thread pool; oldest dropped when exceeded | - |

### 7.4 Dynamic Parameters

The following parameters can be changed at runtime:

```bash
ros2 param set /seg_mask log_level debug    # Change log level
ros2 param set /seg_mask debug true         # Enable debug mode
```

## 8. Published Topics & Message Format

### 8.1 Subscribed Messages

| Topic | Message Type | Encoding | Description |
|-------|-------------|----------|-------------|
| depth_image_topic | `sensor_msgs/msg/Image` | `MONO16` | Depth image, unit: mm |
| detect_info_topic | `ai_msgs/msg/PerceptionTargets` | - | AI detection results (classes, confidence, boxes, mask) |
| class_info_topic | `ai_msgs/msg/PerceptionInfo` | - | Detection class list |
| camera_info_topic | `sensor_msgs/msg/CameraInfo` | - | Camera intrinsic parameters |

### 8.2 Published Messages

| Topic (default) | Message Type | Encoding | Description |
|-----------------|-------------|----------|-------------|
| `/filtered_depth_img` | `sensor_msgs/msg/Image` | `MONO16` | Filtered depth image, non-ROI depth set to zero, unit: mm |
| `/filtered_depth_mask` | `sensor_msgs/msg/Image` | `MONO8` | Filtered binary mask (255 = ROI, 0 = non-ROI) |
| `/filtered_depth_cloud` | `sensor_msgs/msg/PointCloud2` | - | Filtered point cloud (debug mode only) |
| `/roi_pointclouds` | `mask_pc_roi_extractor/msg/ROIPointClouds` | - | Per-instance ROI point clouds with id and confidence |

## 9. Workflow

### 9.1 Initialization
- Create ROS2 node and parameter object
- Declare and retrieve configuration parameters
- Initialize publishers and subscribers
- Parse class confidence thresholds from YAML config files
- Create thread pool and start persistent worker threads
- Set up `message_filters` ApproximateTime synchronizer for depth + detection

### 9.2 Data Reception
- Subscribe to camera_info and class_info (auto-unsubscribe after first message received)
- Depth and detection messages paired via ApproximateTime trigger the callback

### 9.3 Data Processing
- Depth image shared via `toCvShare` zero-copy (read-only)
- Parse detection info: merge config files into common thresholds, single pass for binary mask + class-ID mask
- Callback dispatches two-stage tasks to thread pool and returns immediately; worker threads execute in parallel:
  - **Filter depth**: per-pixel class filter → dilate/erode → zero out depth outside ROI
  - **Extract ROI point clouds**: secondary confidence filter → per-instance point cloud extraction

### 9.4 Result Publishing
- Publish filtered depth image (MONO16) and mask image (MONO8)
- Publish per-instance ROI point clouds (with id / confidence)
- Debug point cloud published only in debug mode

## 10. Performance Optimization

1. **Parallel pipeline**: Persistent thread pool decouples reception from computation; two stages execute truly in parallel
2. **Zero-copy**: Depth image via `toCvShare` shares message buffer; filter stage clones only once
3. **Buffer reuse**: Large images reused through MatPool, eliminating per-frame heap allocation
4. **Micro-optimizations**: Single-pass mask generation, projection reciprocal multiplication, point cloud `reserve`, NEON auto-vectorization (`-mcpu=cortex-a55`), LTO
5. **Morphological operations**: Tune `dilate_iter_num` based on segmentation mask quality; filter mode dilates for coverage, extract mode erodes for precision

## 11. Debugging Guide

### 11.1 View Processing Time (Timing Report)

In debug mode, the node automatically prints per-stage average timing every 100 frames:

```bash
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py debug:=true
```

Example output:
```
[seg_mask] [timing] last 100 frames avg: filter=2.35 ms (100 samples), roi=1.82 ms (100 samples)
```

- **filter**: Average time for the filter pipeline (depth zeroing + mask generation + publishing)
- **roi**: Average time for the ROI point cloud extraction pipeline

Debug mode also prints per-message publish-subscribe latency and depth-detect time delta.

### 11.2 Check Topic Frame Rate & Latency

```bash
# Check publishing frequency for each topic
ros2 topic hz /filtered_depth_mask
ros2 topic hz /filtered_depth_img
ros2 topic hz /roi_pointclouds

# Check end-to-end latency
ros2 topic delay /filtered_depth_mask
```

### 11.3 Check QoS Compatibility

```bash
ros2 topic info /filtered_depth_mask --verbose
```

Common reasons RViz2 cannot receive messages:
- **QoS mismatch** (RELIABLE ↔ BEST_EFFORT) → DDS cannot match
- **frame_id mismatch** → RViz2 Fixed Frame must match the message's frame_id

### 11.4 Visualizing in RViz2

```bash
rviz2
```

Steps:
1. `Add` → `By topic` → `/filtered_depth_mask` → `Image`
2. `Add` → `By topic` → `/filtered_depth_img` → `Image`
3. `Add` → `By topic` → `/filtered_depth_cloud` → `PointCloud2` (debug mode only)

> `/filtered_depth_img` uses MONO16 encoding (millimeters). RViz2 default normalization may wash out the image. In the Image display settings, enable **Normalize Range** and set Min/Max to `0` / `5000` (0–5 m depth range).

### 11.5 Inspect Raw Message Data

```bash
# View Image message header (excluding data array)
ros2 topic echo /filtered_depth_mask --no-arr --once

# View specific fields
ros2 topic echo /filtered_depth_mask --once --field header.frame_id --field encoding --field height --field width
```

### 11.6 Debug with GDB

```bash
# Launch with GDB (auto backtrace on crash)
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py use_gdb:=True

# Attach to a running process
ps aux | grep component_container
gdb -p <PID>
```

### 11.7 Inspect All Parameters

```bash
ros2 param list /seg_mask          # List all parameters
ros2 param describe /seg_mask debug # View parameter description
ros2 param get /seg_mask min_depth  # Get current value
```

## 12. Troubleshooting

### 12.1 No Data After Launch

1. Verify the stereo node and AI detection node are running
2. Check that topics exist: `ros2 topic list | grep -E "depth|detect|filtered"`
3. Check that topics have data: `ros2 topic hz /StereoNetNode/stereonet_depth`
4. Confirm in logs that "Required topics found" succeeded and class_names_id is non-empty

### 12.2 RViz2 Cannot Receive Messages

1. Check QoS compatibility: `ros2 topic info /filtered_depth_mask --verbose`
2. Set RViz2 Fixed Frame to match the message's frame_id (usually `camera_depth_frame`)
3. Confirm the topic has data: `ros2 topic hz /filtered_depth_mask`

### 12.3 Time Sync Dropping Frames

Increase `allow_timestamp_deviation` to tolerate larger timestamp drift:
```bash
ros2 launch mask_pc_roi_extractor mask_pc_roi_extractor.py allow_timestamp_deviation:=1.0
```

## 13. License

This project is open source under the Apache License 2.0.
