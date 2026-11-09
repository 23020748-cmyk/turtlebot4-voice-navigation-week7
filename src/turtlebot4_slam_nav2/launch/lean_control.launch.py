"""Spawn the upstream Create3 wheel controllers after sensor renderer startup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _controller_spawner():
    # IgnitionROS2ControlPlugin loads the upstream controller parameters.
    # Humble's spawner configures and activates these controllers in order.
    return Node(
        package='controller_manager', executable='spawner',
        namespace=LaunchConfiguration('namespace'),
        arguments=[
            'joint_state_broadcaster', 'diffdrive_controller',
            '-c', 'controller_manager',
            '--controller-manager-timeout', '120',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
        ],
        output='screen',
    )


def generate_launch_description():
    initial_spawner = _controller_spawner()
    retry_spawner = _controller_spawner()

    def retry_on_failure(event, context):
        if event.returncode == 0:
            return []
        # Retry once after rendering has had more time to initialize. Target
        # only the initial process, so the retry cannot schedule another retry.
        return [
            LogInfo(msg='Controller spawner failed; retrying once in 10 seconds.'),
            TimerAction(period=10.0, actions=[retry_spawner]),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        RegisterEventHandler(OnProcessExit(
            target_action=initial_spawner, on_exit=retry_on_failure,
        )),
        # The software LiDAR renderer initializes in about 20 seconds on this
        # host. Starting earlier can hit Humble's fixed load-controller timeout.
        TimerAction(period=30.0, actions=[initial_spawner]),
    ])
