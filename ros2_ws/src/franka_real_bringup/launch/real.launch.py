"""Bring up the real FR3 with the same ROS interfaces as the lab simulation.

Starting this file connects to the robot and sets its payload and collision
thresholds (config/robot_settings.yaml). By default (arm:=inactive) it only
reads robot state; with arm:=active the arm trajectory controller is then
activated and holds the current pose under torque control. Arm trajectory goals
pass through trajectory_guard.py. Start it only while someone watches the robot
with the external activation/stop device in hand.
use_fake_hardware:=true runs everything against mock hardware instead.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
FINGERS = ['fr3_finger_joint1', 'fr3_finger_joint2']
# Arm controller inputs that could move the arm without trajectory_guard.py's checks;
# they are moved below this hidden namespace.
UNCHECKED = '/fr3_arm_controller/_unchecked'
ARM_INPUTS = ['follow_joint_trajectory', 'joint_trajectory', 'speed_scaling_input']


def robot_nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    fake = arg('use_fake_hardware') == 'true'
    gripper = arg('load_gripper') == 'true'
    robot_ip = arg('robot_ip')
    pkg = Path(get_package_share_directory('franka_real_bringup'))
    sim = Path(get_package_share_directory('franka_sim_bringup'))
    desc = Path(get_package_share_directory('franka_description'))

    def config(path):
        return yaml.safe_load(path.read_text())

    robot = {'robot_description': xacro.process_file(
        str(pkg / 'urdf/franka_real.urdf.xacro'),
        mappings={'robot_ip': robot_ip, 'use_fake_hardware': str(fake).lower()}).toxml()}
    semantic = {'robot_description_semantic': xacro.process_file(
        str(desc / 'robots/fr3/fr3.srdf.xacro')).toxml()}
    kinematics = {'robot_description_kinematics': config(sim / 'config/kinematics.yaml')}
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
        'robot_description_planning': config(pkg / 'config/joint_limits.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': config(sim / 'config/moveit_controllers.yaml'),
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
    }
    controllers = pkg / ('config/controllers_fake.yaml' if fake else 'config/controllers.yaml')

    def stop_on_failure(node):
        # The controller manager keeps running when the driver cannot connect, so a
        # failed startup step stops the bringup instead of leaving RViz and MoveIt
        # running on joint_state_publisher's default joint values.
        return [node, RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=lambda event, _: [Shutdown(reason=f'{event.action.name} failed')]
            if event.returncode else []))]

    def spawner(*arguments):
        return stop_on_failure(Node(package='controller_manager', executable='spawner',
                                    arguments=[*arguments, '--controller-manager-timeout', '60']))

    nodes = [
        LogInfo(msg='MOCK HARDWARE: no robot is connected.' if fake else
                f'REAL ROBOT at {robot_ip}: keep the external activation/stop device in hand.'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(controllers)],
             remappings=[('~/robot_description', '/robot_description')],
             output='screen', on_exit=Shutdown(reason='controller manager exited')),
        # Arm states are merged with the separately published finger states.
        *spawner('joint_state_broadcaster', '--controller-ros-args',
                 '--remap joint_states:=franka/joint_states'),
        # The arm controller starts inactive; robot_settings.py applies the payload and
        # collision thresholds, then activates it if arm:=active. Its inputs move to
        # hidden names, so goals reach it only through trajectory_guard.py.
        *spawner('fr3_arm_controller', '--inactive', '--controller-ros-args', ' '.join(
            f'-r /fr3_arm_controller/{name}:={UNCHECKED}/{name}' for name in ARM_INPUTS)),
        *stop_on_failure(Node(
            package='franka_real_bringup', executable='robot_settings.py', output='screen',
            parameters=[str(pkg / 'config/robot_settings.yaml'),
                        {'fake': fake, 'activate_arm': arg('arm') == 'active'}])),
        Node(package='franka_real_bringup', executable='trajectory_guard.py', output='screen',
             parameters=[{'controller_action': f'{UNCHECKED}/follow_joint_trajectory'}],
             on_exit=Shutdown(reason='trajectory guard exited')),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
                          'rate': 30}]),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[robot, semantic, kinematics, planning], output='screen'),
        Node(package='franka_real_bringup', executable='workspace_collision.py'),
    ]
    if not fake:
        nodes += spawner('franka_robot_state_broadcaster')
    if gripper:
        nodes.append(Node(package='franka_real_bringup', executable='fr3_gripper_bridge.py',
                          parameters=[str(pkg / 'config/gripper_bridge.yaml'), {'fake': fake}]))
        if not fake:
            gripper_config = Path(get_package_share_directory('franka_gripper')) / \
                'config/franka_gripper_node.yaml'
            # Exits at once when FCI is refused; stop the bringup with it.
            nodes.append(Node(package='franka_gripper', executable='franka_gripper_node',
                              name='franka_gripper', output='screen',
                              parameters=[{'robot_ip': robot_ip, 'joint_names': FINGERS},
                                          str(gripper_config)],
                              on_exit=Shutdown(reason='franka_gripper_node exited')))
    if arg('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', str(sim / 'rviz/lab.rviz')],
                          parameters=[robot, semantic, kinematics]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='10.10.10.10'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('arm', default_value='inactive', choices=['active', 'inactive'],
                              description='active: hold the pose and accept goals; '
                                          'inactive: connect and read state without commanding'),
        DeclareLaunchArgument('load_gripper', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=robot_nodes),
    ])
