# Copyright 2020 Tier IV, Inc. All rights reserved.
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


import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    # set concat filter as a component
    crop_box_filter = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        # remappings=[("output", "concatenated/pointcloud"), ("input", "hesai/pandar")],
        remappings=[("output", "crop_box_filtered/pointcloud"), ("input", "hesai/pandar")],
        parameters=[
            {
                "input_frame": "3d_lidar",
                "min_x": -0.4,
                "max_x": 0.4,
                "min_y": -0.2,
                "max_y": 0.8,
                "min_z": -1.0,
                "max_z": 0.5,
                "negative": True,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    distortion_corrector = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
        name="distortion_corrector",
        # remappings=[("~/output/pointcloud", "concatenated/pointcloud"), ("~/input/pointcloud", "crop_box_filtered/pointcloud"), ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance")],
        remappings=[("~/output/pointcloud", "distortion_corrector_filtered/pointcloud"), ("~/input/pointcloud", "crop_box_filtered/pointcloud"), ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance")],
        parameters=[
            {
                "timestamp_field_name": "timestamp",
                "use_imu": False,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    voxel_grid_outlier_filter = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::VoxelGridOutlierFilterComponent",
        name="voxel_grid_outlier_filter",
        # remappings=[("output", "concatenated/pointcloud"), ("input", "crop_box_filtered/pointcloud")],
        remappings=[("output", "concatenated/pointcloud"), ("input", "distortion_corrector_filtered/pointcloud")],
        parameters=[
            {
                "voxel_size_x": 0.3,
                "voxel_size_y": 0.3,
                "voxel_size_z": 0.3,
                "voxel_points_threshold": 5,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[],
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        composable_node_descriptions=[crop_box_filter, distortion_corrector, voxel_grid_outlier_filter],
        # composable_node_descriptions=[crop_box_filter],
        target_container=target_container,
        condition=IfCondition(LaunchConfiguration("use_concat_filter")),
    )

    return [container, concat_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("container_name", "pointcloud_preprocessor_container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )