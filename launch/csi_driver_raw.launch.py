from launch_ros.actions import Node

from launch import LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

FLIP_METHOD_DOCS_URL = "https://gstreamer.freedesktop.org/documentation/videofilter/videoflip.html#GstVideoFlipMethod"


def generate_launch_description() -> LaunchDescription:
    launch_entities: list[LaunchDescriptionEntity] = []

    namespace = LaunchConfiguration('namespace')
    info_file = LaunchConfiguration('info_file')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    framerate = LaunchConfiguration('framerate')
    flip_method = LaunchConfiguration('flip_method')
    optical_frame = LaunchConfiguration('optical_frame')

    launch_entities.append(DeclareLaunchArgument('namespace', default_value='csi_camera'))
    launch_entities.append(DeclareLaunchArgument('info_file', default_value='',
                                                 description='Path to the camera info file'))
    launch_entities.append(DeclareLaunchArgument('width', default_value='1920'))
    launch_entities.append(DeclareLaunchArgument('height', default_value='1080'))
    launch_entities.append(DeclareLaunchArgument('framerate', default_value='30'))
    launch_entities.append(DeclareLaunchArgument('flip_method', default_value='0',
                                                 description="Flip Method: " + FLIP_METHOD_DOCS_URL))
    launch_entities.append(DeclareLaunchArgument('optical_frame', default_value='csi_camera_optical_frame',
                                                 description='The tf2 optical frame id'))

    launch_entities.append(
            Node(
                    package='csi_driver',
                    executable='csi_driver_node',
                    name="csi_driver",
                    namespace=namespace,
                    parameters=[
                        {
                            'info_file': info_file,
                            'width': width,
                            'height': height,
                            'framerate': framerate,
                            'flip_method': flip_method,
                            "optical_frame": optical_frame
                        }
                    ]
            )
    )

    return LaunchDescription(launch_entities)
