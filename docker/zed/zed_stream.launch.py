"""ZED X Mini from the Jetson's network stream, with the lab's topic names.

The Jetson streams the camera with the ZED SDK; this receiver computes depth on
the PC's GPU and publishes the simulation's /zed_x_mini/* interface. Messages
use the wrapper's zed_x_mini_* frames, which the real robot URDF places on the
hand, so the wrapper publishes no TF or camera description of its own.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

NODE = '/zed_x_mini/zed_node'
# Wrapper topic -> lab topic; each image's camera_info follows its image topic.
TOPICS = {
    'rgb/color/rect/image': '/zed_x_mini/rgbd/image',
    'depth/depth_registered': '/zed_x_mini/rgbd/depth_image',
    'point_cloud/cloud_registered': '/zed_x_mini/rgbd/points',
    'right/color/rect/image': '/zed_x_mini/right/image',
    'imu/data': '/zed_x_mini/imu',
}


def generate_launch_description():
    wrapper = Path(get_package_share_directory('zed_wrapper')) / 'launch/zed_camera.launch.py'
    return LaunchDescription([GroupAction([
        *[SetRemap(f'{NODE}/{src}', dst) for src, dst in TOPICS.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(wrapper)), launch_arguments={
            'camera_model': 'zedxm',
            'camera_name': 'zed_x_mini',
            'stream_address': os.environ.get('ZED_STREAM_ADDRESS', '192.168.55.1'),
            'stream_port': os.environ.get('ZED_STREAM_PORT', '30000'),
            'publish_urdf': 'false',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'ros_params_override_path': str(Path(__file__).with_name('zed_stream.yaml')),
        }.items()),
    ])])
