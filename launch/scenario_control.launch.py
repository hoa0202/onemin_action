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
        DeclareLaunchArgument(
            "use_waypoint_graph",
            default_value="true",
            description="true면 waypoint_graph.yaml+line_goal_links 로 다중 Nav2 목표",
        ),
        DeclareLaunchArgument(
            "waypoint_graph_file",
            default_value="waypoint_graph.yaml",
            description="data 디렉터리 기준 그래프 파일명",
        ),
        DeclareLaunchArgument(
            "graph_start_node_id",
            default_value="",
            description="비우면 현재 pose에서 그래프 최단 진입; 지정 시 항상 그 노드에서만 출발(예: wp_1)",
        ),
        DeclareLaunchArgument(
            "pose_topic",
            default_value="/liorf/mapping/odometry",
            description="그래프 자동 시작용 로봇 pose 토픽",
        ),
        DeclareLaunchArgument(
            "pose_type",
            default_value="odom",
            description="odom | amcl | pose_stamped",
        ),
        DeclareLaunchArgument(
            "graph_auto_start_policy",
            default_value="nearest",
            description="후보 순서: nearest(가까운노드부터) | min_total(유클대그래프합 작은순). 둘 다 역행 검사 통과한 경로만 채택",
        ),
        DeclareLaunchArgument(
            "graph_path_dot_min",
            default_value="0.0",
            description="간선·첫구간 진행방향·목표벡터 dot 최소(0=뒷걸음 금지, 음수면 약간 허용)",
        ),
        DeclareLaunchArgument(
            "line_command_topic",
            default_value="/harv_robot/line_move",
            description="라인/홈 std_msgs/String: 1~N 또는 warehouse",
        ),
        DeclareLaunchArgument(
            "line_number_max",
            default_value="10",
            description="허용 라인 번호 상한 (시나리오 파라미터)",
        ),
        DeclareLaunchArgument(
            "docking_topic",
            default_value="/harv_robot/docking_move",
            description="도킹 명령 std_msgs/String (docking_positions.yaml 키와 동일)",
        ),
        DeclareLaunchArgument(
            "docking_positions_file",
            default_value="docking_positions.yaml",
            description="data 디렉터리 기준 도킹 목표 포즈 파일",
        ),
        DeclareLaunchArgument(
            "graph_segment_relax_yaw",
            default_value="true",
            description="그래프 경유 중간 NavigateToPose 구간만 controller yaw_goal_tolerance 완화",
        ),
        DeclareLaunchArgument(
            "graph_segment_yaw_tolerance",
            default_value="6.283185307179586",
            description="중간 구간에 쓸 yaw 허용(라디안), 사실상 임의 방향 허용",
        ),
        DeclareLaunchArgument(
            "final_goal_yaw_tolerance",
            default_value="0.25",
            description="라인/도킹 마지막 세그먼트 전에 되돌릴 yaw_goal_tolerance",
        ),
        DeclareLaunchArgument(
            "nav2_controller_node",
            default_value="controller_server",
            description="goal_checker.yaw_goal_tolerance 가 올라가는 Nav2 노드 이름",
        ),
        DeclareLaunchArgument(
            "nav2_yaw_goal_tolerance_param",
            default_value="",
            description="비우면 후보 키 순차 시도. ros2 param list 로 확인한 전체 파라미터 이름",
        ),
        DeclareLaunchArgument(
            "nav_goal_output_frame",
            default_value="",
            description="비우면 그대로 전송. Nav2 map 과 맞출 때 map (grid_map 이 map 이면 필수에 가까움)",
        ),
        DeclareLaunchArgument(
            "nav_goal_tf_timeout_sec",
            default_value="1.0",
            description="goal 변환 lookup_transform 타임아웃(초)",
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
                "use_waypoint_graph": LaunchConfiguration("use_waypoint_graph"),
                "waypoint_graph_file": LaunchConfiguration("waypoint_graph_file"),
                "graph_start_node_id": LaunchConfiguration("graph_start_node_id"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "pose_type": LaunchConfiguration("pose_type"),
                "graph_auto_start_policy": LaunchConfiguration("graph_auto_start_policy"),
                "graph_path_dot_min": LaunchConfiguration("graph_path_dot_min"),
                "line_command_topic": LaunchConfiguration("line_command_topic"),
                "line_number_max": LaunchConfiguration("line_number_max"),
                "docking_topic": LaunchConfiguration("docking_topic"),
                "docking_positions_file": LaunchConfiguration("docking_positions_file"),
                "graph_segment_relax_yaw": LaunchConfiguration("graph_segment_relax_yaw"),
                "graph_segment_yaw_tolerance": LaunchConfiguration(
                    "graph_segment_yaw_tolerance"
                ),
                "final_goal_yaw_tolerance": LaunchConfiguration("final_goal_yaw_tolerance"),
                "nav2_controller_node": LaunchConfiguration("nav2_controller_node"),
                "nav2_yaw_goal_tolerance_param": LaunchConfiguration(
                    "nav2_yaw_goal_tolerance_param"
                ),
                "nav_goal_output_frame": LaunchConfiguration("nav_goal_output_frame"),
                "nav_goal_tf_timeout_sec": LaunchConfiguration("nav_goal_tf_timeout_sec"),
            }],
        ),
    ])
