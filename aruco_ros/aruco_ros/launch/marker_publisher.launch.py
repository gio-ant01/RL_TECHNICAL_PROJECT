from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    aruco_marker_publisher_params = {
        'image_is_rectified': LaunchConfiguration('image_is_rectified'),
        'marker_size': LaunchConfiguration('marker_size'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': LaunchConfiguration('camera_frame'),
    }

    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        parameters=[aruco_marker_publisher_params],
        remappings=[('/camera_info', LaunchConfiguration('camera_info')),
                    ('/image', LaunchConfiguration('camera_image'))],
    )

    return [aruco_marker_publisher]


def generate_launch_description():

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.06',
        description='Marker size in m. '
    )

    camera_image_arg = DeclareLaunchArgument(
        'camera_image', default_value='/fra2mo/camera',
        description='Image topic published by the camera.'
    )

    camera_info_arg = DeclareLaunchArgument(
        'camera_info', default_value='/fra2mo/camera_info',
        description='Camera info topic published by the camera.'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame', default_value='camera_optical_link',
        description='Camera optical frame name.'
    )

    image_is_rectified_arg = DeclareLaunchArgument(
        'image_is_rectified', default_value='False',
        description='Set to True if images are already rectified (camera_info present and rectified).'
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='base',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_size_arg)
    ld.add_action(camera_image_arg)
    ld.add_action(camera_info_arg)
    ld.add_action(camera_frame_arg)
    ld.add_action(image_is_rectified_arg)
    ld.add_action(reference_frame)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld