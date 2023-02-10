from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_entities: list[LaunchDescriptionEntity] = []
    composable_nodes: list[ComposableNode] = []

    namespace = LaunchConfiguration('namespace')
    width = LaunchConfiguration('camera_width')
    height = LaunchConfiguration('camera_height')
    framerate = LaunchConfiguration('camera_framerate')
    flip_method = LaunchConfiguration('camera_flip_method')
    optical_frame = LaunchConfiguration('camera_optical_frame')

    launch_entities.append(DeclareLaunchArgument('namespace', default_value='csi_camera'))
    launch_entities.append(DeclareLaunchArgument('camera_width', default_value='1280'))
    launch_entities.append(DeclareLaunchArgument('camera_height', default_value='720'))
    launch_entities.append(DeclareLaunchArgument('camera_framerate', default_value='30'))
    launch_entities.append(DeclareLaunchArgument('camera_flip_method', default_value='0'))
    launch_entities.append(DeclareLaunchArgument('camera_optical_frame', default_value='csi_camera_optical_frame'))

    composable_nodes.append(
            ComposableNode(
                    package='image_proc',
                    plugin='image_proc::DebayerNode',
                    name='debayer_node',
                    namespace=namespace,
                    extra_arguments=[
                        {'use_intra_process_comms': True}
                    ]
            )
    )
    composable_nodes.append(
            ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_mono_node',
                    namespace=namespace,
                    extra_arguments=[
                        {'use_intra_process_comms': True}
                    ],
                    remappings=[
                        ('image', 'image_mono'),
                        ('image_rect', 'image_rect_mono')
                    ]
            )
    )
    composable_nodes.append(
            ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_color_node',
                    namespace=namespace,
                    extra_arguments=[
                        {'use_intra_process_comms': True}
                    ],
                    remappings=[
                        ('image', 'image_color'),
                        ('image_rect', 'image_rect_color')
                    ]
            )
    )
    composable_nodes.append(
            ComposableNode(
                    package='csi_driver',
                    plugin='csi_driver::CSIDriverNode',
                    name="csi_driver",
                    namespace=namespace,
                    extra_arguments=[
                        {'use_intra_process_comms': True}
                    ],
                    parameters=[
                        {
                            'capture.width': width,
                            'capture.height': height,
                            'capture.framerate': framerate,
                            'capture.flip_method': flip_method,
                            "optical_frame": optical_frame
                        }
                    ]
            )
    )

    launch_entities.append(
            ComposableNodeContainer(
                    name='image_proc_container',
                    namespace=namespace,
                    package='rclcpp_components',
                    executable='component_container',
                    output='screen',
                    composable_node_descriptions=composable_nodes
            )
    )

    return LaunchDescription(launch_entities)
