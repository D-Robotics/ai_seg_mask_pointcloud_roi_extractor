# AI Segmentation Mask PointCloud ROI Extractor

## 1. Project Overview

AI Segmentation Mask PointCloud ROI Extractor is a ROS2 Humble-based package specifically designed to extract or filter Regions of Interest (ROI) from depth images. It utilizes AI segmentation mask technology to accurately separate target objects from point clouds, providing high-quality point cloud data for subsequent robot perception and decision-making.

## 2. Core Features

The package implements the following core features:

- Standard ROS2 node implementation
- Subscribes to depth images and AI detection information with precise timestamp synchronization
- Extracts ROI from point clouds based on detection categories and confidence thresholds
- Publishes filtered point clouds, depth images, and mask images
- Supports dynamic parameter adjustment
- Provides comprehensive logging and debugging information

## 2. Directory Structure

```
ai_seg_mask_pointcloud_roi_extractor/
├── include/
│   └── ai_seg_mask_pointcloud_roi_extractor/
│       └── ai_seg_mask_pointcloud_roi_extractor.hpp  # Component header file
├── src/
│   ├── ai_seg_mask_pointcloud_roi_extractor.cpp  # Main component implementation
│   ├── callback.cpp                              # Callback function implementation
│   ├── publish.cpp                               # Publishing function implementation
│   ├── read_param.cpp                            # Parameter reading implementation
│   ├── set_dynamic_para.cpp                      # Dynamic parameter handling
│   └── time_stamp.cpp                            # Timestamp conversion utilities
├── launch/
│   ├── ai_seg_mask_pointcloud_roi_extractor.py   # Component launch file
│   └── parser_config_params.py                   # Configuration parser
├── config/
│   ├── descriptions/
│   │   └── ai_seg_mask_pointcloud_roi_extractor.yaml  # Parameter configuration description
│   └── model_classes_config.yaml                 # Class confidence threshold configuration
├── package.xml                         # Package definition file
├── CMakeLists.txt                      # CMake build configuration
├── README.md                           # Chinese documentation
└── README_en.md                        # English documentation
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
- **`config/model_classes_config.yaml`**: Configures confidence thresholds for each class

### 6.2 Parameter Description

### 6.2 Core Parameters

| Parameter Name | Type | Default Value | Description | Unit |
|---------------|------|---------------|-------------|------|
| debug | bool | false | Enable debug mode to output more detailed log information | - |
| depth_image_topic | string | "/StereoNetNode/stereonet_depth" | Depth image subscription topic | - |
| detect_info_topic | string | "/hobot_dnn_detection" | AI segmentation detection information subscription topic | - |
| class_info_topic | string | "/hobot_dnn_detection_info" | Class information subscription topic | - |
| camera_info_topic | string | "/StereoNetNode/stereonet_depth/camera_info" | Camera information subscription topic | - |
| color_image_topic | string | "/StereoNetNode/origin_left_image" | Color image subscription topic | - |
| filtered_mask_topic | string | "/filtered_depth_mask" | Filtered mask publishing topic | - |
| filtered_depth_topic | string | "/filtered_depth_img" | Filtered depth image publishing topic | - |
| filtered_cloud_topic | string | "/filtered_depth_cloud" | Filtered point cloud publishing topic | - |

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

### 6.4 Dynamic Parameters

Parameters that support runtime dynamic adjustment:

- **debug**: Enable/disable debug mode
- **log_level**: Set log level (debug/info/warn/error/critical)

```bash
# Dynamically adjust log level
ros2 param set /depth_mask_extractor_node log_level debug

# Dynamically enable debug mode
ros2 param set /depth_mask_extractor_node debug true
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

### 8.5 Result Publishing Phase
- Publish filtered depth images
- Publish filtered mask images
- Publish filtered point clouds

## 9. Notes

1. **Time Synchronization**: Depth images and detection information must be timestamp-aligned; otherwise, processing will be skipped
2. **Image Size**: The size of depth images and segmentation masks must match the configured camera_width and camera_height
3. **Depth Unit**: Depth images are assumed to be in millimeters, which are converted to meters for processing
4. **Class Configuration**: The model_classes_config.yaml file must be configured according to the actual AI model classes
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
ros2 param set /depth_mask_extractor_node log_level debug
```

### 11.2 Check Parameters

```bash
# View all parameters
ros2 param list /depth_mask_extractor_node

# View specific parameter values
ros2 param get /depth_mask_extractor_node debug

# Set parameter values
ros2 param set /depth_mask_extractor_node debug true
```

## 12. License

This project is open source under the Apache License 2.0.

## 13. Contact Information

For questions or suggestions, please contact the project maintainers.