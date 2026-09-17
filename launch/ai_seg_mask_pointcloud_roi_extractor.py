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

from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from seg_mask_parser_config_params import AutoLaunchArguments

def generate_launch_description():
    pkg_dir = get_package_share_directory('ai_seg_mask_pointcloud_roi_extractor')

    # declare launch arguments
    declare_container_name_cmd = DeclareLaunchArgument(
                    "container_name",
                    default_value="perception_container",
                    description="Name of the container in which the nodes will be launched")
    declare_launch_container_cmd = DeclareLaunchArgument(
                    "launch_container",
                    default_value="True",
                    description="Whether to launch the container")
    declare_use_gdb_cmd = DeclareLaunchArgument(
                    "use_gdb",
                    default_value="False",
                    description="Run component_container under GDB for debugging")

    container_name = LaunchConfiguration("container_name")
    launch_container = LaunchConfiguration("launch_container")
    use_gdb = LaunchConfiguration("use_gdb")

    def _on_launch(context, *args):
        """
        Print resolved launch configuration values at runtime.
        """
        print(f"container_name= {context.launch_configurations.get('container_name', 'N/A')}, "
              f"launch_container= {context.launch_configurations.get('launch_container', 'N/A')}")
        if context.launch_configurations.get('launch_container', 'False') == 'True':
            print("Launching container")
        else:
            print("Not launching container")
        if context.launch_configurations.get('use_gdb', 'False') == 'True':
            print("Running under GDB")
        else:
            print("Not running under GDB")

    descriptions_dir = os.path.join(pkg_dir, 'config', 'descriptions')
    print(f"descriptions_dir = {descriptions_dir}")
    auto_launch_arguments = AutoLaunchArguments(descriptions_dir)
    
    declare_arguments = auto_launch_arguments.get_declare_arguments()
    launch_parameters = auto_launch_arguments.get_config_parameters()

    log_level = auto_launch_arguments.find_parameter("log_level")
    if log_level is None:
        raise ValueError("[log_level] parameter not found")
    print(f"log_level= {log_level}")

    # Conditions: mutually exclusive
    cond_launch_gdb = PythonExpression(
        ["'", use_gdb, "' == 'True' and '", launch_container, "' == 'True'"])
    cond_launch_normal = PythonExpression(
        ["'", use_gdb, "' == 'False' and '", launch_container, "' == 'True'"])
    # Standalone: no container, run as independent executable (GDB-friendly)
    cond_standalone_gdb = PythonExpression(
        ["'", use_gdb, "' == 'True' and '", launch_container, "' == 'False'"])
    cond_standalone_normal = PythonExpression(
        ["'", use_gdb, "' == 'False' and '", launch_container, "' == 'False'"])

    gdb_prefix = 'gdb -batch -ex run -ex "bt" -ex "bt full" -ex "info threads" -ex quit --args'
    # Container node with GDB
    container_node_gdb = Node(
        condition=IfCondition(cond_launch_gdb),
        package="rclcpp_components",
        executable="component_container",
        name=container_name,
        output="screen",
        emulate_tty=True,   # Simulated terminal, GDB requires interaction
        prefix=gdb_prefix,
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ])

    # Container node normal
    container_node = Node(
        condition=IfCondition(cond_launch_normal),
        package="rclcpp_components",
        executable="component_container",
        name=container_name,
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ])

    # Standalone node (no container, GDB-friendly)
    node_standalone_gdb = Node(
        condition=IfCondition(cond_standalone_gdb),
        package="ai_seg_mask_pointcloud_roi_extractor",
        executable="ai_seg_mask_pointcloud_roi_extractor_node",
        name="seg_mask",
        output="screen",
        emulate_tty=True,
        prefix='gdb -batch -ex run -ex "bt" -ex "bt full" -ex "info threads" -ex quit --args',
        parameters=launch_parameters,
        arguments=["--ros-args", "--log-level", log_level])

    node_standalone = Node(
        condition=IfCondition(cond_standalone_normal),
        package="ai_seg_mask_pointcloud_roi_extractor",
        executable="ai_seg_mask_pointcloud_roi_extractor_node",
        name="seg_mask",
        output="screen",
        parameters=launch_parameters,
        arguments=["--ros-args", "--log-level", log_level])

    # Load composable nodes (container mode only)
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(cond_launch_normal),  # only when container is running without GDB
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="ai_seg_mask_pointcloud_roi_extractor",
                plugin="seg_mask_roi_extractor::AISegMaskPointCloudROIExtractor",
                name="seg_mask",
                parameters=launch_parameters,
                extra_arguments=[{"use_intra_process_comms": False}],
            ),
        ],
    )
    load_composable_nodes_gdb = LoadComposableNodes(
        condition=IfCondition(cond_launch_gdb),  # only when container is running with GDB
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="ai_seg_mask_pointcloud_roi_extractor",
                plugin="seg_mask_roi_extractor::AISegMaskPointCloudROIExtractor",
                name="seg_mask",
                parameters=launch_parameters,
                extra_arguments=[{"use_intra_process_comms": False}],
            ),
        ],
    )

    # launch description
    launch_description = []
    launch_description.append(declare_container_name_cmd)
    launch_description.append(declare_launch_container_cmd)
    launch_description.append(declare_use_gdb_cmd)
    launch_description.extend(declare_arguments)
    launch_description.append(OpaqueFunction(function=_on_launch))
    launch_description.append(container_node)
    launch_description.append(container_node_gdb)
    launch_description.append(node_standalone)
    launch_description.append(node_standalone_gdb)
    launch_description.append(load_composable_nodes)
    launch_description.append(load_composable_nodes_gdb)

    return LaunchDescription(launch_description)
