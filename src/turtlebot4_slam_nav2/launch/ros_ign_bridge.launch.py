from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true','false']),
    DeclareLaunchArgument('low_resource', default_value='true', choices=['true','false']),
    DeclareLaunchArgument('camera_enabled', default_value='false', choices=['true','false']),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4'),
    DeclareLaunchArgument('dock_name', default_value='standard_dock'),
    DeclareLaunchArgument('namespace', default_value=''),
    DeclareLaunchArgument('world', default_value='maze'),
    DeclareLaunchArgument('model', default_value='lite', choices=['lite']),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    namespace = LaunchConfiguration('namespace')
    base = get_package_share_directory('irobot_create_ignition_bringup')
    create3_path = PathJoinSubstitution([base, 'launch', 'create3_ros_ignition_bridge.launch.py'])
    create3_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(create3_path),
        launch_arguments={'robot_name': robot_name, 'dock_name': LaunchConfiguration('dock_name'),
                          'namespace': namespace, 'world': world}.items(),
        condition=UnlessCondition(LaunchConfiguration('low_resource')))
    lidar_topic = ['/world/', world, '/model/', robot_name, '/link/rplidar_link/sensor/rplidar/scan']
    camera_topic = ['/world/', world, '/model/', robot_name,
                    '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image']
    info_topic = ['/world/', world, '/model/', robot_name,
                  '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info']
    lidar = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='lidar_bridge', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[lidar_topic + ['@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                   '--ros-args', '-r', '/world/maze/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan:=/scan'])
    camera = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[camera_topic + ['@sensor_msgs/msg/Image[ignition.msgs.Image'],
                   info_topic + ['@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
                   '--ros-args', '-r', '/world/maze/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image:=/oakd/rgb/preview/image_raw',
                   '-r', '/world/maze/model/turtlebot4/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info:=/oakd/rgb/preview/camera_info'],
        condition=IfCondition(LaunchConfiguration('camera_enabled')))
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(create3_bridge)
    ld.add_action(lidar)
    ld.add_action(camera)
    return ld
