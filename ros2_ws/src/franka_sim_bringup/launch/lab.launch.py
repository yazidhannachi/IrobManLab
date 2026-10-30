"""Start one lab scene and robot interfaces; never start a student task."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument,
    IncludeLaunchDescription, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def generate_launch_description():
    pkg = Path(get_package_share_directory('franka_sim_bringup'))
    desc = Path(get_package_share_directory('franka_description'))
    gz_launch = str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')
    scene = Node(package='franka_sim_bringup', executable='spawn_scene.py', parameters=[{
        'environment': LaunchConfiguration('environment'),
        'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
        'cube_count': ParameterValue(LaunchConfiguration('cube_count'), value_type=int),
    }])

    def config(name):
        return yaml.safe_load((pkg / 'config' / name).read_text())

    robot = {'robot_description': ParameterValue(
        Command(['xacro ', str(pkg / 'urdf/franka_sim.urdf.xacro')]), value_type=str)}
    semantic = {'robot_description_semantic': ParameterValue(
        Command(['xacro ', str(desc / 'robots/fr3/fr3.srdf.xacro')]), value_type=str)}
    kinematics = {'robot_description_kinematics': config('kinematics.yaml')}
    planning = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath'],
            'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect'}},
            'fr3_arm': {'planner_configs': ['RRTConnect']},
        },
        'robot_description_planning': config('joint_limits.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': config('moveit_controllers.yaml'),
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    # Only sensor data is bridged. Robot joint states come from ros2_control.
    bridges = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for topic, ros, gz in [
        ('/zed_x_mini/left/rgbd/image', 'Image', 'Image'),
        ('/zed_x_mini/left/rgbd/depth_image', 'Image', 'Image'),
        ('/zed_x_mini/left/rgbd/camera_info', 'CameraInfo', 'CameraInfo'),
        ('/zed_x_mini/right/image', 'Image', 'Image'),
        ('/zed_x_mini/right/camera_info', 'CameraInfo', 'CameraInfo'),
        ('/zed_x_mini/imu', 'Imu', 'IMU'),
    ]:
        bridges.append(f'{topic}@sensor_msgs/msg/{ros}[gz.msgs.{gz}')

    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='cubes', choices=['cubes', 'drawer']),
        DeclareLaunchArgument('seed', default_value='1', description='Initial object layout seed'),
        DeclareLaunchArgument('cube_count', default_value='6', description='1–12 cubes; cubes scene only'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        SetParameter(name='use_sim_time', value=True),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  f'{pkg / "models"}:{desc.parent}:{pkg.parent}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': [
                '-r -s --headless-rendering ', str(pkg / 'worlds/franka_table.world')],
                'on_exit_shutdown': 'true'}.items()),
        # Load the complete scene before the GUI requests its initial scene snapshot.
        RegisterEventHandler(OnProcessExit(target_action=scene, on_exit=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={'gz_args': '-g --render-engine-gui ogre2',
                                  'on_exit_shutdown': 'true'}.items(),
                condition=IfCondition(LaunchConfiguration('gui')))])),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot]),
        # Gazebo identifies stereo sides by "left"/"right" in sensor topics.
        # Keep the lab's RGB-D ROS interface independent of those internal names.
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridges,
             remappings=[(f'/zed_x_mini/left/rgbd/{suffix}', f'/zed_x_mini/rgbd/{suffix}')
                         for suffix in ('image', 'depth_image', 'camera_info')]),
        # Gazebo's own point cloud is mislabelled and distorted; build it from depth.
        Node(package='franka_sim_bringup', executable='rgbd_points.py'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'fr3', '-topic', 'robot_description']),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'fr3_arm_controller', 'fr3_gripper',
                        '--controller-manager-timeout', '90']),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[robot, semantic, kinematics, planning]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(pkg / 'rviz/lab.rviz')],
             parameters=[robot, semantic, kinematics], condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='franka_sim_bringup', executable='table_collision.py'),
        scene,
    ])
