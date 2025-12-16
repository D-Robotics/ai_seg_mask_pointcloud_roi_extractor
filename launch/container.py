# Copyright 2025 perception
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction
)

from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression
)


def get_launch_file_path(pkg_name: str, launch_file_name: str):
    try:
        pkg_share_dir = get_package_share_directory(pkg_name)
    except PackageNotFoundError as e:
        print(f"[ERROR] Failed to find package {pkg_name}:{e}", file=sys.stderr)
        sys.exit(1)
    
    launch_file_path = os.path.join(pkg_share_dir, 'launch', launch_file_name)

    if not os.path.exists(launch_file_path):
        print(f"[ERROR] Failed to find launch file {launch_file_name} in package {pkg_name}:{launch_file_path}", file=sys.stderr)
        sys.exit(1)
    
    LogInfo(msg=f"[INFO] Found launch file {launch_file_name} in package {pkg_name}:{launch_file_path}")
    return launch_file_path

def generate_launch_description():

    # Whether to launch the ai_seg_mask_pointcloud_roi_extractor module
    declare_run_mask_depth_cmd = DeclareLaunchArgument(
        "run_mask_depth",
        default_value="True",
        description="Whether to launch the ai_seg_mask_pointcloud_roi_extractor module"
    )

    # Name of the component container (shared with other modules)
    declare_container_name_cmd = DeclareLaunchArgument(
        "container_name",
        default_value="perception_container",
        description="Name of the component container (shared with other modules)"
    )

    # Namespace for the component container and nodes
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the component container and nodes"
    )

    run_mask_depth = LaunchConfiguration("run_mask_depth")
    container_name = LaunchConfiguration("container_name")
    namespace = LaunchConfiguration("namespace")

    # Log the launch arguments
    log_args = LogInfo(msg=PythonExpression([
        '"run_mask_depth=", "', run_mask_depth, 
        '", container_name=", "', container_name,
        '", namespace=", "', namespace, '"'
    ]))

    # Start the component container
    main_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name=container_name,
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # Path to the ai_seg_mask_pointcloud_roi_extractor module
    target_launch_path = get_launch_file_path(
        pkg_name="ai_seg_mask_pointcloud_roi_extractor",
        launch_file_name="ai_seg_mask_pointcloud_roi_extractor.py"
    )

    # Include the ai_seg_mask_pointcloud_roi_extractor launch file
    include_target_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(target_launch_path),
        condition=IfCondition(LaunchConfiguration('run_mask_depth')),
        launch_arguments={
            'autostart': 'True',
            'launch_container': 'False',
            'container_name': container_name,
            'namespace': namespace,
            'camera_info_topic': '/StereoNetNode/stereonet_depth/camera_info',
            'detect_info_topic': '/hobot_dnn_detection',
            'class_info_topic': '/hobot_dnn_detection_info',
            'filtered_depth_topic': '/StereoNetNode/stereonet_depth_filtered',
            'log_level': 'info'
        }.items()
    )

    return LaunchDescription([
        declare_run_mask_depth_cmd,
        declare_container_name_cmd,
        declare_namespace,
        main_container,
        include_target_launch
    ])


