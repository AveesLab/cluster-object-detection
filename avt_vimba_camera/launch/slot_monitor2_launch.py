from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


prenamespace = "node2"
index = 2

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
            {"ip": "192.168.2.89"},
            {"guid": ""},
            {"camera_info_url": avt_vimba_camera_calibration_file},
            {"frame_id": "camera"},
            {"ptp_offset": 0},
            {"node_index": index},
            {"number_of_nodes": 4},
            {"local_inference_fps": 7.5},
            {"timestamp_margin_milisecond_": 0.05},
            {"convert_frame": 50},
            {"dnn_cfg_path": "/home/avees/object_detection/src/cluster-object-detection/avt_vimba_camera/include/objectdetection/darknet/cfg/yolov4.cfg"},
            {"dnn_weight_path": "/home/avees/weights/yolov4.weights"},
            {"can_send_time_interval_microsecond": 1000},
        ]
    )
    
    return LaunchDescription([
        avt_vimba_camera_params,
        avt_vimba_camera_calibration,
        sensor
    ])
