from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


prenamespace = "monitor3"
index = 3

def generate_launch_description():
    avt_vimba_camera_pkg_prefix = get_package_share_directory('avt_vimba_camera')

    avt_vimba_camera_params_file = os.path.join(
        avt_vimba_camera_pkg_prefix, 'config/params.yaml')
    avt_vimba_camera_calibration_file = os.path.join(
        avt_vimba_camera_pkg_prefix, 'calibrations/calibration_example.yaml')

    avt_vimba_camera_params = DeclareLaunchArgument(
        'avt_vimba_camera_params_file',
        default_value = avt_vimba_camera_params_file,
        description = 'Path to config file for avt_vimba_camera'
    )
    avt_vimba_camera_calibration = DeclareLaunchArgument(
        'avt_vimba_camera_calibration_file',
        default_value = avt_vimba_camera_calibration_file,
        description = 'Path to calibration file for avt_vimba_camera'
    )

    sensor = Node(
        package = 'avt_vimba_camera',
        namespace = prenamespace,
        executable = 'mono_camera_exec',
        name = 'camera',
        output = 'screen',
        parameters = [
            LaunchConfiguration('avt_vimba_camera_params_file'),
            {"name": "camera"},
            {"frame_id": "camera"},
            {"ip": "192.168.2.89"},
            {"guid": ""},
            {"use_measurement_time": True},
            {"ptp_offset": 0},
            {"node_index": index},
            {"camera_info_url": avt_vimba_camera_calibration_file},
            {"use_image_transport": False},
            {"image_crop": False},
            {"use_benchmark": True},
            {"number_of_nodes": 3},
            {"camera_fps": 30.},
            {"inference_fps": 5.},
            {"inference_model_path": "/home/avees/engine/yolov7.engine"},
            {"inference_cfg_path": "/home/avees/ros2_ws/weights/yolov7.cfg"},
            {"inference_weight_path": "/home/avees/ros2_ws/weights/yolov7.weights"},
            {"use_can": True},
            {"can_id": index + 101},
            {"time_interval": 1000},
            {"pcan_benchmark": False},
            {"pcan_benchmark_start_stamp": 10180.0},
            {"pcan_benchmark_stamp_interval": 14.0},
            {"max_camera_cycle_time": 33.5},
            {"min_camera_cycle_time": 33.0},
            {"convert_frame": 50}, 
        ]
    )
    
    return LaunchDescription([
        avt_vimba_camera_params,
        avt_vimba_camera_calibration,
        sensor
    ])
