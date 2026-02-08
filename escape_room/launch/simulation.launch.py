"""
Launch file unificato per l'escape room

Questo launch file avvia tutti i componenti necessari per completare la missione:
1. Keypad controller (in attesa del PIN)
2. Door controller (in attesa della conferma PIN)
3. Navigazione Nav2 con mappa + RViz
4. ArUco marker detection
5. Bridge ArUco -> PIN
6. Script follow_waypoints per guidare fra2mo verso la porta
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    escape_room_dir = FindPackageShare('escape_room')
    aruco_ros_dir = FindPackageShare('aruco_ros')

    # Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # ============================================================
    # 1. KEYPAD CONTROLLER (per armando - in attesa del PIN)
    # ============================================================
    keypad_controller_node = Node(
        package='escape_room',
        executable='keypad_controller.py',
        name='keypad_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ============================================================
    # 2. DOOR CONTROLLER (in attesa della conferma PIN)
    # ============================================================
    door_controller_node = Node(
        package='escape_room',
        executable='door_controller.py',
        name='door_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ============================================================
    # 3. NAVIGAZIONE Nav2 (include mappa + RViz)
    # ============================================================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([escape_room_dir, 'launch', 'fra2mo_navigation.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ============================================================
    # 4. ARUCO MARKER DETECTION
    # ============================================================
    aruco_marker_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([aruco_ros_dir, 'launch', 'marker_publisher.launch.py'])
        ),
        launch_arguments={
            'marker_size': '0.06',
            'camera_image': '/fra2mo/camera',
            'camera_info': '/fra2mo/camera_info',
            'camera_frame': 'camera_optical_link',
            'image_is_rectified': 'False',
            'reference_frame': 'base_link',
        }.items()
    )

    # ============================================================
    # 5. ARUCO TO PIN BRIDGE
    # ============================================================
    aruco_to_pin_bridge_node = Node(
        package='escape_room',
        executable='aruco_to_pin_bridge.py',
        name='aruco_to_pin_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'required_consecutive_frames': 5},
            {'markers_topic': '/marker_publisher/markers'},
            {'pin_topic': '/pin_code'}
        ]
    )

    # ============================================================
    # 6. FOLLOW WAYPOINTS (guida fra2mo verso la porta)
    # ============================================================
    # Ritardato di 10 secondi per dare tempo a Nav2 di inizializzarsi
    reach_goal_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='escape_room',
                executable='reach_goal.py',
                name='reach_goal',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,

        # Prima i nodi "listener" che stanno in attesa
        keypad_controller_node,
        door_controller_node,

        # Poi la navigazione
        navigation_launch,

        # ArUco detection e bridge
        aruco_marker_publisher,
        aruco_to_pin_bridge_node,

        # Infine reach_goal (ritardato per dare tempo a Nav2)
        reach_goal_node,
    ])
