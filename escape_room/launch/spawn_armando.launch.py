import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def spawn_robot(name, namespace, x_pose, y_pose, z_pose, xacro_file, yaml_file):
    """Genera i nodi necessari per spawnare e controllare un robot in Gazebo"""
    
    pkg_share = FindPackageShare('escape_room')
    yaml_path = PathJoinSubstitution([pkg_share, 'config', yaml_file])
    
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', xacro_file]),
        ' namespace:=', namespace,
        ' yaml_file:=', yaml_path
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', name,
            '-topic', f'/{namespace}/robot_description',
            '-x', str(x_pose),
            '-y', str(y_pose),
            '-z', str(z_pose)
        ],
        output='screen'
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'position_controller',
            '-c', f'/{namespace}/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    position_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[position_spawner]
        )
    )

    return [
        robot_state_publisher,
        spawn_entity,
        joint_state_spawner,
        position_after_jsb
    ]


def generate_launch_description():
    
    # ✅ NON avviare Gazebo qui - si assume sia già in esecuzione
    
    # Robot 1
    robot1_nodes = spawn_robot(
        name='arm',
        namespace='arm',
        x_pose=1.84,
        y_pose=0.89,
        z_pose=0.00,
        xacro_file='armando/arm.urdf.xacro',
        yaml_file='armando/armando_controllers.yaml'
    )
 

    # Bridge per la camera del robot 1
    bridge_camera1 = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/arm/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/arm/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='arm_bridge',
        arguments=[
            # Comandi velocità base mobile
            '/arm/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            # Odometria
            '/arm/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # TF
            '/arm/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
        ],
        remappings=[
            ('/arm/odom', '/arm/odom'),
            ('/arm/cmd_vel', '/arm/cmd_vel'),
        ],
        output='screen'
    )




    
    return LaunchDescription([
        *robot1_nodes,
        bridge_camera1,
        bridge,
    ])