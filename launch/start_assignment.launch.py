from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 1. Spawn robot
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bme_gazebo_sensors'),
                'launch',
                'spawn_robot.launch.py'
            )
        )
    )

    # 2. Aruco tracker
    aruco_tracker_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('aruco_opencv'),
                'launch',
                'aruco_tracker.launch.xml'
            )
        )
    )

    # 3. Load map / SLAM
    loading_map = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_navigation'),
                'launch',
                'mapping.launch.py'
            )
        )
    )

    # 4. Navigation stack
    navigation_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_navigation'),
                'launch',
                'navigation.launch.py'
            )
        )
    )

    # 5. Generate plan
    generating_plan_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('assignment2_exp'),
                'launch',
                'distributed_actions.launch.py'
            )
        )
    )

    # 6. Start scanning / execution in new terminal
    execute_plan = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'ros2', 'run', 'assignment2_exp', 'get_plan_and_execute'
        ],
        output='screen'
    )

    # Sequenza con timer per garantire che ogni launch parta dopo il precedente
    sequence = TimerAction(
        period=0.5,  # piccolo delay iniziale
        actions=[
            spawn_robot_launch,
            LogInfo(msg='spawn_robot launched, scheduling aruco_tracker in 5s'),
            TimerAction(
                period=5.0,
                actions=[
                    aruco_tracker_launch,
                    LogInfo(msg='aruco_tracker launched, scheduling load_map in 2s'),
                    TimerAction(
                        period=2.0,
                        actions=[
                            loading_map,
                            LogInfo(msg='map loading launched, scheduling navigation in 2s'),
                            TimerAction(
                                period=2.0,
                                actions=[
                                    navigation_launch,
                                    LogInfo(msg='navigation launched, scheduling plan generation in 4s'),
                                    TimerAction(
                                        period=4.0,
                                        actions=[
                                            generating_plan_launch,
                                            LogInfo(msg='plan generation launched, scheduling execution in 3s'),
                                            TimerAction(
                                                period=3.0,
                                                actions=[execute_plan]
                                            )
                                        ]
                                    )
                                ]
                            )
                        ]
                    )
                ]
            )
        ]
    )

    return LaunchDescription([sequence])
