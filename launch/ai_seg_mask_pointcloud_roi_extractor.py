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
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, 
    GroupAction, TimerAction, LogInfo
)

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
sys.path.append(current_dir)
print(f"parent_dir= {parent_dir}")

from seg_mask_parser_config_params import AutoLaunchArguments

def generate_launch_description():
    # declare launch arguments
    declare_container_name_cmd = DeclareLaunchArgument(
                    "container_name",
                    default_value="perception_container",
                    description="Name of the container in which the nodes will be launched")
    declare_launch_container_cmd = DeclareLaunchArgument(
                    "launch_container",
                    default_value="True",
                    description="Whether to launch the container")

    container_name = LaunchConfiguration("container_name")
    launch_container = LaunchConfiguration("launch_container")
    # print(f"container_name= {container_name}, launch_container= {launch_container}")

    descriptions_dir = os.path.join(parent_dir, 'config', 'descriptions')
    print(f"descriptions_dir = {descriptions_dir}")
    auto_launch_arguments = AutoLaunchArguments(descriptions_dir)
    declare_arguments = auto_launch_arguments.get_declare_arguments()
    launch_parameters = auto_launch_arguments.get_config_parameters()

    log_level = auto_launch_arguments.find_parameter("log_level")
    if log_level is None:
        raise ValueError("[log_level] parameter not found")
    print(f"log_level= {log_level}")
    
    container_node = Node(
        condition=IfCondition(launch_container),
        package="rclcpp_components",
        executable="component_container",
        name=container_name,
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        respawn=False,
        respawn_delay=2.0)
    
    # Load composable nodes
    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="ai_seg_mask_pointcloud_roi_extractor",
                plugin="seg_mask_roi_extractor::AISegMaskPointCloudROIExtractor",
                name="seg_mask",
                parameters=launch_parameters,
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    # launch description
    launch_description = []
    launch_description.append(declare_container_name_cmd)
    launch_description.append(declare_launch_container_cmd)
    launch_description.extend(declare_arguments)
    launch_description.append(container_node)
    launch_description.append(load_composable_nodes)

    return LaunchDescription(launch_description)
