from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def include(package, filename, **arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory(package)) / "launch" / filename)
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    root = Path(get_package_share_directory("turtlebot4_voice_nav"))
    support = Path(get_package_share_directory("turtlebot4_slam_nav2"))
    sim = "true"
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument(
                "results_dir", default_value=str(Path.home() / ".ros/voice_navigation")
            ),
            SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1"),
            SetEnvironmentVariable("LP_NUM_THREADS", "2"),
            SetEnvironmentVariable("OGRE_RTT_MODE", "Copy"),
            SetEnvironmentVariable("OPENBLAS_NUM_THREADS", "1"),
            SetEnvironmentVariable("OMP_NUM_THREADS", "1"),
            SetParameter(name="bond_timeout", value=60.0),
            include(
                "turtlebot4_slam_nav2",
                "ignition.launch.py",
                world=str(root / "worlds/maze"),
                model="lite",
                use_sim_time=sim,
                gui=LaunchConfiguration("gui"),
            ),
            include(
                "turtlebot4_slam_nav2",
                "turtlebot4_spawn.launch.py",
                world="maze",
                model="lite",
                use_sim_time=sim,
                rviz="false",
                camera_enabled="false",
                slam="false",
                localization="false",
                nav2="false",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="voice_nav_validation_bridge",
                arguments=[
                    "/model/turtlebot4/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"
                ],
                remappings=[("/model/turtlebot4/pose", "/voice_nav/ground_truth")],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    include(
                        "turtlebot4_navigation",
                        "localization.launch.py",
                        map=str(root / "maps/navigation_map.yaml"),
                        use_sim_time=sim,
                        params=str(support / "config/localization_low_resource.yaml"),
                    ),
                    include(
                        "turtlebot4_navigation",
                        "nav2.launch.py",
                        use_sim_time=sim,
                        params_file=str(root / "config/nav2_voice.yaml"),
                    ),
                    Node(
                        package="turtlebot4_voice_nav",
                        executable="voice_navigation",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "initialize_pose": True,
                                "require_ground_truth": True,
                                "regions_file": str(root / "maps/regions.json"),
                                "results_dir": LaunchConfiguration("results_dir"),
                                "goal_timeout_sec": 600.0,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
