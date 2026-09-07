import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

    default_model = os.path.join(pkg_share, 'urdf', 'kitchen_scene.urdf.xacro')
    default_rviz = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    default_initial = os.path.join(pkg_share, 'config', 'initial_positions.yaml')

    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    gui = LaunchConfiguration('gui')
    initial_positions = LaunchConfiguration('initial_positions')

    # value_type=str keeps the URDF text a plain string; without it the
    # parameter machinery tries to YAML-parse the XML and the node fails.
    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to the robot URDF/xacro file',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz,
            description='Absolute path to the RViz config file',
        ),
        DeclareLaunchArgument(
            'initial_positions',
            default_value=default_initial,
            description='YAML file of startup joint positions (zeros parameter)',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start joint_state_publisher_gui to drive the joints',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[initial_positions],
            condition=IfCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
