from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    store_markers_service = Node(
        package='marker_service_pkg',
        executable='store_markers_service',
        name='store_markers_service',
        output='screen'
    )
    """
    generating_plan = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'ros2', 'run', 'marker_service_pkg', 'store_markers_service'
        ],
        output='screen'
    )
    """

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
                'localization.launch.py'
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
    generating_plan = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'ros2', 'launch', 'assignment2_exp', 'distributed_actions.launch.py'
        ],
        output='screen'
    )

    # 6. Start scanning / execution in new terminal
    execute_plan = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--',
            'ros2', 'run', 'assignment2_exp', 'get_plan_and_execute'
        ],
        output='screen'
    )

    return LaunchDescription([
        store_markers_service,
        aruco_tracker_launch,
        spawn_robot_launch,
        LogInfo(msg='store_markers_service, aruco_tracker and spawn robot'),

        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='Starting localization'),
                loading_map
            ]
        ),

        # after 11s start navigation (give time to localization)
        TimerAction(
            period=7.0,
            actions=[
                LogInfo(msg='Starting navigation'),
                navigation_launch
            ]
        ),

        # after 15s start plan generation
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='Starting plan generation'),
                generating_plan
            ]
        ),

        # after 18s execute the plan (choose one of the options below)
        TimerAction(
            period=14.0,
            actions=[
                LogInfo(msg='Executing plan (node)'),
                execute_plan,   # preferito: lo esegue come node normale
                # execute_plan_new_term,  # alternativa: esegui in nuovo terminale se preferisci
            ]
        ),
    ])
