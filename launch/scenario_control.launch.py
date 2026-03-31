from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "nav2_action_name",
            default_value="navigate_to_pose",
            description="Nav2 NavigateToPose 액션 이름",
        ),
        DeclareLaunchArgument(
            "entering_check_timeout_sec",
            default_value="0.5",
            description="entering_check 대기 시간(초)",
        ),
        DeclareLaunchArgument(
            "entering_check_max_retries",
            default_value="10",
            description="entering_check 미수신 시 entering_start 최대 재전송 횟수",
        ),
        Node(
            package="onemin_action",
            executable="scenario_controller_node",
            name="scenario_controller_node",
            output="screen",
            parameters=[{
                "nav2_action_name": LaunchConfiguration("nav2_action_name"),
                "entering_check_timeout_sec": LaunchConfiguration("entering_check_timeout_sec"),
                "entering_check_max_retries": LaunchConfiguration("entering_check_max_retries"),
            }],
        ),
    ])
