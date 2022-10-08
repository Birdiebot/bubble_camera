'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-10 18:59:25
FilePath: /bubble/src/bubble_camera/launch/camera_launch.py
LastEditors: Ligcox
LastEditTime: 2022-07-16 02:54:09
E-mail: robomaster@birdiebot.top
'''

import os
import launch_ros
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.launch_context import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process.process_exited import ProcessExited
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # log_level = 'info'
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription([
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('bubble_camera'), 'config', 'config.yaml'),
                description='Full path to camera parameters file'
            ),

            launch_ros.actions.Node(
                package='bubble_camera',
                node_name="camera",
                node_executable='cam',
                parameters=[LaunchConfiguration('params_file')],
                output='screen',
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(on_exit=on_exit_restart))
        ])
    else:
        return LaunchDescription([
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('bubble_camera'), 'config', 'config.yaml'),
                description='Full path to camera parameters file'
            ),

            launch_ros.actions.Node(
                package='bubble_camera',
                name="camera",
                executable='cam',
                parameters=[LaunchConfiguration('params_file')],
                output='screen',
            ),
        ])


def on_exit_restart(event: ProcessExited, context: LaunchContext):

    import time
    print(
        "\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(event.action.name, event.pid, event.returncode))
    if event.returncode == -6:
        # 相机掉线重启
        time.sleep(3)
        os.system("ros2 launch birdiebot_vinput vinput_launch.py")
