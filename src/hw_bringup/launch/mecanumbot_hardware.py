'''
Author: JFG 73979026+bodaoye@users.noreply.github.com
Date: 2022-11-21 21:58:00
LastEditors: JFG 73979026+bodaoye@users.noreply.github.com
LastEditTime: 2022-11-21 23:16:31
FilePath: \ros2-mecanum-bot-main\mecanumbot_bringup\launch\mecanumbot_hardware.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Get the robot description
    urdf_path = os.path.join(get_package_share_directory('hw_description'), 'urdf', 'mecanumbot.urdf')
    urdf_doc = xacro.parse(open(urdf_path, 'r'))
    xacro.process_doc(urdf_doc)
    robot_description = urdf_doc.toxml()

    robot_controller_config = os.path.join(get_package_share_directory('hw_description'), 'config', 'robot_controller_config.yaml')

    return LaunchDescription([
        Node(
            package='hw_control',
            executable='hw_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                robot_controller_config
            ]
        )
    ])