#!/usr/bin/env python3
"""
코드2: 전체 시나리오 제어.

carry_01 (운반): line_move 비활성(기본). /carry_robot/docking_move 만 사용.
  move_to_docking_station → docking_positions + docking_goal_links
  move_to_return → warehouse(라인 홈) 그래프 이동; 실패 시 YAML move_to_return 폴백

harv (수확): line_move + 도킹. move_to_return → 저장 라인 복귀 후 entering_start~ 수확 시나리오.
"""
import enum
import math
from typing import Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.task import Future
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

from onemin_action.line_pose_loader import (
    get_data_dir,
    load_line_pose,
    load_pose_by_key,
    warehouse_pose_and_graph_link_key,
)
from onemin_action.waypoint_graph_planner import (
    plan_path_to_line,
    plan_path_to_line_goal_link_key,
    plan_path_to_named_goal,
)

DOCK_STATION_CMD = "move_to_docking_station"
DOCK_RETURN_CMD = "move_to_return"

# Nav2 배포/플러그인마다 goal_checker 파라미터 전체 이름이 다름. 잘못된 이름을 먼저 쓰면 controller 에 WARN 이 찍힘.
_YAW_GOAL_TOL_PARAM_CANDIDATES = (
    "goal_checker.SimpleGoalChecker.yaw_goal_tolerance",
    "goal_checker.yaw_goal_tolerance",
    "general_goal_checker.yaw_goal_tolerance",
    "FollowPath.goal_checker.yaw_goal_tolerance",
)


def _param_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes")
    return bool(value)


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _apply_deadband(v: float, vmin: float, vmax: float) -> float:
    """0이 아니면 최소속도 보장(정지마찰 극복), 부호 유지, 최대속도 포화."""
    if v == 0.0:
        return 0.0
    s = 1.0 if v > 0.0 else -1.0
    m = abs(v)
    m = _clamp(m, vmin, vmax)
    return s * m


class State(enum.Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    DOCKING = "docking"
    SEND_ENTERING_START = "send_entering_start"
    WAIT_ENTERING_CHECK = "wait_entering_check"
    WAIT_GOAL_FINISH = "wait_goal_finish"
    WAIT_GOAL_RETURN_FINISH = "wait_goal_return_finish"


class ScenarioControllerNode(Node):
    def __init__(self):
        super().__init__("scenario_controller_node")

        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("entering_check_timeout_sec", 0.5)
        self.declare_parameter("entering_check_max_retries", 10)
        self.declare_parameter("use_waypoint_graph", True)
        self.declare_parameter("waypoint_graph_file", "waypoint_graph.yaml")
        self.declare_parameter("graph_start_node_id", "")
        self.declare_parameter("pose_topic", "/liorf/mapping/odometry")
        self.declare_parameter("pose_type", "odom")
        self.declare_parameter("graph_auto_start_policy", "nearest")
        self.declare_parameter("graph_path_dot_min", 0.0)
        self.declare_parameter("carry_robot", True)
        self.declare_parameter("line_command_topic", "")
        self.declare_parameter("line_number_max", 10)
        self.declare_parameter("docking_topic", "/carry_robot/docking_move")
        self.declare_parameter("docking_positions_file", "docking_positions.yaml")
        # 그래프 경유 세그먼트만 yaw_goal_tolerance 완화 (Nav2 controller_server)
        self.declare_parameter("graph_segment_relax_yaw", True)
        self.declare_parameter("graph_segment_yaw_tolerance", 6.283185307179586)
        self.declare_parameter("final_goal_yaw_tolerance", 0.25)
        self.declare_parameter("nav2_controller_node", "controller_server")
        # 비우면 _YAW_GOAL_TOL_PARAM_CANDIDATES 순서 시도. ros2 param list 로 확인한 전체 이름 지정 가능.
        self.declare_parameter("nav2_yaw_goal_tolerance_param", "")
        # Nav2 global_costmap 과 동일 프레임(보통 map). YAML 이 odom_1 등이면 여기에 map 두고 TF 로 goal 변환.
        self.declare_parameter("nav_goal_output_frame", "")
        self.declare_parameter("nav_goal_tf_timeout_sec", 1.0)

        # ── 최종 도킹(차동구동, TF 기반 후방 standoff 정렬) ──────────────────
        self.declare_parameter("enable_final_docking", True)
        self.declare_parameter("dock_target_frame", "base_link_1")   # 도킹 대상 로봇
        self.declare_parameter("dock_robot_frame", "base_link_2")    # carry 로봇
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        # 대상 로봇 후방(-x_target)으로 이만큼 떨어진 지점에 carry 가 정렬·정지
        self.declare_parameter("dock_rear_offset", 1.0)
        self.declare_parameter("dock_control_rate_hz", 20.0)
        self.declare_parameter("dock_k_lin", 0.5)
        self.declare_parameter("dock_k_ang", 1.2)
        self.declare_parameter("dock_max_lin", 0.25)
        self.declare_parameter("dock_min_lin", 0.03)
        self.declare_parameter("dock_max_ang", 0.6)
        self.declare_parameter("dock_min_ang", 0.05)
        # 목표점 방위 오차가 이보다 크면 직진 멈추고 제자리 선회부터
        self.declare_parameter("dock_coarse_ang", 0.6)
        self.declare_parameter("dock_pos_tol", 0.08)
        self.declare_parameter("dock_yaw_tol", 0.05)
        self.declare_parameter("dock_settle_cycles", 5)
        self.declare_parameter("dock_tf_timeout_sec", 0.2)
        self.declare_parameter("dock_lost_timeout_sec", 3.0)
        self.declare_parameter("dock_status_value", "docking_end")

        self._nav2_action_name = self.get_parameter("nav2_action_name").value
        self._entering_check_timeout = self.get_parameter("entering_check_timeout_sec").value
        self._entering_check_max_retries = self.get_parameter("entering_check_max_retries").value
        self._use_graph = _param_bool(self.get_parameter("use_waypoint_graph").value)
        self._graph_file = self.get_parameter("waypoint_graph_file").value
        self._graph_start_node_id = str(self.get_parameter("graph_start_node_id").value or "").strip()
        self._pose_topic = self.get_parameter("pose_topic").value
        self._pose_type = self.get_parameter("pose_type").value
        self._graph_auto_start_policy = str(
            self.get_parameter("graph_auto_start_policy").value or "nearest"
        ).strip()
        try:
            self._graph_path_dot_min = float(self.get_parameter("graph_path_dot_min").value)
        except (TypeError, ValueError):
            self._graph_path_dot_min = 0.0
        self._carry_robot = _param_bool(self.get_parameter("carry_robot").value)
        self._line_command_topic = str(
            self.get_parameter("line_command_topic").value or ""
        ).strip()
        try:
            self._line_number_max = int(self.get_parameter("line_number_max").value)
        except (TypeError, ValueError):
            self._line_number_max = 10
        if self._line_number_max < 1:
            self._line_number_max = 10
        self._docking_topic = str(self.get_parameter("docking_topic").value or "")
        self._docking_positions_file = str(
            self.get_parameter("docking_positions_file").value or "docking_positions.yaml"
        )
        self._graph_segment_relax_yaw = _param_bool(
            self.get_parameter("graph_segment_relax_yaw").value
        )
        try:
            self._graph_relaxed_yaw_tolerance = float(
                self.get_parameter("graph_segment_yaw_tolerance").value
            )
        except (TypeError, ValueError):
            self._graph_relaxed_yaw_tolerance = 6.283185307179586
        try:
            self._final_goal_yaw_tolerance = float(
                self.get_parameter("final_goal_yaw_tolerance").value
            )
        except (TypeError, ValueError):
            self._final_goal_yaw_tolerance = 0.25
        self._nav2_controller_node = str(
            self.get_parameter("nav2_controller_node").value or "controller_server"
        ).strip()
        _yaw_param_override = str(
            self.get_parameter("nav2_yaw_goal_tolerance_param").value or ""
        ).strip()
        self._nav2_yaw_tol_param_names: Tuple[str, ...] = (
            (_yaw_param_override,) if _yaw_param_override else _YAW_GOAL_TOL_PARAM_CANDIDATES
        )
        self._nav_goal_output_frame = str(
            self.get_parameter("nav_goal_output_frame").value or ""
        ).strip()
        try:
            self._nav_goal_tf_timeout_sec = float(
                self.get_parameter("nav_goal_tf_timeout_sec").value
            )
        except (TypeError, ValueError):
            self._nav_goal_tf_timeout_sec = 1.0
        # 도킹 파라미터 로드
        self._enable_final_docking = _param_bool(
            self.get_parameter("enable_final_docking").value
        )
        self._dock_target_frame = str(self.get_parameter("dock_target_frame").value or "base_link_1")
        self._dock_robot_frame = str(self.get_parameter("dock_robot_frame").value or "base_link_2")
        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value or "/cmd_vel")

        def _pf(name: str, default: float) -> float:
            try:
                return float(self.get_parameter(name).value)
            except (TypeError, ValueError):
                return default

        self._dock_rear_offset = _pf("dock_rear_offset", 1.0)
        self._dock_rate_hz = max(1.0, _pf("dock_control_rate_hz", 20.0))
        self._dock_k_lin = _pf("dock_k_lin", 0.5)
        self._dock_k_ang = _pf("dock_k_ang", 1.2)
        self._dock_max_lin = _pf("dock_max_lin", 0.25)
        self._dock_min_lin = _pf("dock_min_lin", 0.03)
        self._dock_max_ang = _pf("dock_max_ang", 0.6)
        self._dock_min_ang = _pf("dock_min_ang", 0.05)
        self._dock_coarse_ang = _pf("dock_coarse_ang", 0.6)
        self._dock_pos_tol = _pf("dock_pos_tol", 0.08)
        self._dock_yaw_tol = _pf("dock_yaw_tol", 0.05)
        try:
            self._dock_settle_cycles = max(1, int(self.get_parameter("dock_settle_cycles").value))
        except (TypeError, ValueError):
            self._dock_settle_cycles = 5
        self._dock_tf_timeout = _pf("dock_tf_timeout_sec", 0.2)
        self._dock_lost_timeout = _pf("dock_lost_timeout_sec", 3.0)
        self._dock_status_value = str(self.get_parameter("dock_status_value").value or "docking_end")

        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        if self._nav_goal_output_frame or self._enable_final_docking:
            self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
            self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # 도킹 런타임 상태
        self._after_nav_is_dock_station: bool = False
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._dock_status_pub = (
            self.create_publisher(String, self._docking_topic, 10)
            if self._docking_topic
            else None
        )
        self._dock_timer: Optional[Node.Timer] = None
        self._dock_settle_count = 0
        self._dock_last_tf_ok_ns: int = 0

        self._data_dir = get_data_dir()

        self._last_robot_pose: Optional[PoseStamped] = None

        self._nav_targets: list = []
        self._nav_target_idx: int = 0
        self._after_nav_is_docking: bool = False
        self._after_nav_is_warehouse_home: bool = False
        self._line_before_dock: Optional[int] = None
        self._from_dock_return: bool = False

        self._state = State.IDLE
        self._line_number: Optional[int] = None
        self._entering_check_received = False
        self._entering_check_retry_count = 0
        self._goal_finish_received = False
        self._goal_return_finish_received = False
        self._entering_check_timer: Optional[Node.Timer] = None
        self._nav_goal_handle = None
        self._nav_result_future: Optional[Future] = None
        self._param_cb_group = ReentrantCallbackGroup()
        self._nav2_param_client: Optional[AsyncParameterClient] = None
        if self._graph_segment_relax_yaw:
            self._nav2_param_client = AsyncParameterClient(
                self,
                self._nav2_controller_node,
                callback_group=self._param_cb_group,
            )
        self._last_set_yaw_goal_tolerance: Optional[float] = None
        self._resolved_yaw_tol_param_name: Optional[str] = None
        self._warned_nav2_param_unavailable = False
        self._warned_yaw_tol_all_names_failed = False

        self._action_pub = self.create_publisher(String, "/action", 10)
        self._action_check_sub = self.create_subscription(
            String, "/action_check", self._cb_action_check, 10
        )
        if self._line_command_topic:
            self.create_subscription(
                String,
                self._line_command_topic,
                self._cb_line_move,
                10,
            )
        if self._docking_topic:
            self.create_subscription(
                String, self._docking_topic, self._cb_docking_move, 10
            )
        if self._pose_type == "pose_stamped":
            self.create_subscription(
                PoseStamped, self._pose_topic, self._cb_pose_stamped, 10
            )
        elif self._pose_type == "amcl":
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._pose_topic,
                self._cb_pose_amcl,
                10,
            )
        else:
            self.create_subscription(Odometry, self._pose_topic, self._cb_pose_odom, 10)

        self._nav_client = ActionClient(self, NavigateToPose, self._nav2_action_name)

        _line_desc = (
            f"{self._line_command_topic!r} (1~{self._line_number_max}|warehouse)"
            if self._line_command_topic
            else "비활성(운반)"
        )
        self.get_logger().info(
            f"시나리오: line_move={_line_desc} | 도킹: {self._docking_topic!r} "
            f"| carry_robot={self._carry_robot}"
        )
        _gstart = (
            repr(self._graph_start_node_id)
            if self._graph_start_node_id
            else "자동(현재 pose→최단 그래프 진입)"
        )
        self.get_logger().info(
            f"Nav2 액션: {self._nav2_action_name} | data: {self._data_dir} | "
            f"use_waypoint_graph={self._use_graph} graph_start_node_id={_gstart} | "
            f"graph_auto_start_policy={self._graph_auto_start_policy!r} | "
            f"path_dot_min={self._graph_path_dot_min} | "
            f"pose: {self._pose_topic} ({self._pose_type}) | "
            f"graph_relax_yaw={self._graph_segment_relax_yaw} | "
            f"nav_goal_frame={self._nav_goal_output_frame or '원본'}"
        )
        if self._enable_final_docking:
            self.get_logger().info(
                f"최종 도킹 ON: {self._dock_robot_frame}→{self._dock_target_frame} "
                f"후방 {self._dock_rear_offset}m, cmd_vel={self._cmd_vel_topic}, "
                f"pos_tol={self._dock_pos_tol} yaw_tol={self._dock_yaw_tol}"
            )
        else:
            self.get_logger().info("최종 도킹 OFF (도킹 스테이션 도착 시 바로 IDLE).")

    def _nav_goal_pose_in_output_frame(self, pose: PoseStamped) -> PoseStamped:
        target = self._nav_goal_output_frame
        if not target or not self._tf_buffer or pose.header.frame_id == target:
            return pose
        try:
            tf = self._tf_buffer.lookup_transform(
                target,
                pose.header.frame_id,
                Time(),
                timeout=Duration(seconds=self._nav_goal_tf_timeout_sec),
            )
            out = do_transform_pose(pose, tf)
            out.header.frame_id = target
            out.header.stamp = self.get_clock().now().to_msg()
            return out
        except Exception as e:
            self.get_logger().warn(
                f"Nav goal TF {pose.header.frame_id!r}→{target!r} 실패, 원본 전송: {e}"
            )
            return pose

    def _fill_pose_stamped(self, pose: PoseStamped, frame_id, position, orientation) -> None:
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = position
        pose.pose.orientation = orientation

    def _cb_pose_odom(self, msg: Odometry) -> None:
        if self._last_robot_pose is None:
            self._last_robot_pose = PoseStamped()
        self._fill_pose_stamped(
            self._last_robot_pose,
            msg.header.frame_id,
            msg.pose.pose.position,
            msg.pose.pose.orientation,
        )

    def _cb_pose_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        if self._last_robot_pose is None:
            self._last_robot_pose = PoseStamped()
        self._fill_pose_stamped(
            self._last_robot_pose,
            msg.header.frame_id,
            msg.pose.pose.position,
            msg.pose.pose.orientation,
        )

    def _cb_pose_stamped(self, msg: PoseStamped) -> None:
        self._last_robot_pose = msg

    def _navigate_to_line(self, n: int, stamp) -> bool:
        """line_positions + line_goal_links 로만 이동 (도킹 YAML 안 탐). 성공 시 Nav 발사까지."""
        pose = load_line_pose(self._data_dir, n, stamp=stamp)
        if pose is None:
            self.get_logger().warn(
                f"라인 {n} 없음 (line_positions.yaml). 다음 입력 대기."
            )
            print(f"라인 {n} 없음.", flush=True)
            return False

        if self._use_graph:
            cur = self._last_robot_pose
            if cur is None:
                self.get_logger().error(
                    "그래프 모드: pose 미수신 (pose_topic / pose_type)."
                )
                return False
            planned = plan_path_to_line(
                self._data_dir,
                self._graph_file,
                n,
                self._graph_start_node_id,
                pose,
                stamp=stamp,
                current_pose=cur,
                auto_start_policy=self._graph_auto_start_policy,
                path_dot_min=self._graph_path_dot_min,
            )
            if not planned:
                self.get_logger().warn(
                    f"그래프 경로 실패 (라인 {n}). line_goal_links·간선·역행 검사 확인."
                )
                print(f"그래프 경로 실패 라인 {n}.", flush=True)
                return False
            poses, eff_start = planned
            self._nav_targets = poses
            self._nav_target_idx = 0
            self.get_logger().info(
                f"라인 {n} 경로 {len(poses)}포인트, 시작노드={eff_start}"
            )
        else:
            self._nav_targets = [pose]
            self._nav_target_idx = 0
            self.get_logger().info(f"라인 {n} 직선 이동.")

        self._line_number = n
        self._after_nav_is_docking = False
        self._after_nav_is_warehouse_home = False
        self._state = State.NAVIGATING
        self._send_current_nav_goal()
        return True

    def _cb_docking_move(self, msg: String) -> None:
        if self._state != State.IDLE:
            self.get_logger().warn(
                f"도킹 명령 무시 (상태 {self._state.value}). IDLE 일 때만 가능."
            )
            return
        cmd = (msg.data or "").strip()
        if not cmd:
            return
        # 자체 발행하는 상태 문자열은 명령으로 처리하지 않음(토픽 공유로 인한 자기수신 방지)
        if cmd in ("docking_end", "move_finish", self._dock_status_value):
            return
        stamp = self.get_clock().now().to_msg()

        if cmd == DOCK_RETURN_CMD:
            if self._carry_robot:
                if self._navigate_to_warehouse_home(stamp):
                    return
                self.get_logger().warn(
                    "warehouse 복귀 실패(line_positions·line_goal_links 등) → "
                    "docking_positions move_to_return 폴백"
                )
            else:
                n_mem = self._line_before_dock
                if n_mem is not None and n_mem >= 1:
                    self._from_dock_return = True
                    self.get_logger().info(
                        f"도킹 복귀 → 재기억 라인 {n_mem} (수확 시나리오)"
                    )
                    if not self._navigate_to_line(n_mem, stamp):
                        self._from_dock_return = False
                    return
                self.get_logger().warn(
                    "복귀할 저장 라인 없음 → docking_positions 의 move_to_return 폴백"
                )

        if cmd not in (DOCK_STATION_CMD, DOCK_RETURN_CMD):
            self.get_logger().warn(f"알 수 없는 도킹 명령: {cmd!r}")
            return

        self._after_nav_is_warehouse_home = False

        if cmd == DOCK_STATION_CMD:
            if self._carry_robot:
                self._line_before_dock = None
            elif self._line_number is not None and self._line_number >= 1:
                self._line_before_dock = self._line_number
                self.get_logger().info(
                    f"도킹 이동 전 라인 {self._line_before_dock} 저장 (복귀 시 사용)"
                )
            else:
                self._line_before_dock = None
                self.get_logger().warn(
                    "저장할 라인 없음 → line_before_dock 초기화. 복귀는 폴백(move_to_return pose)만."
                )

        dock_pose = load_pose_by_key(
            self._data_dir, self._docking_positions_file, cmd, stamp=stamp
        )
        if dock_pose is None:
            self.get_logger().warn(
                f"도킹 목표 없음: 키 {cmd!r} in {self._docking_positions_file}"
            )
            return

        cur = self._last_robot_pose
        if self._use_graph:
            if cur is None:
                self.get_logger().error("도킹 그래프 경로: pose 미수신.")
                return
            planned = plan_path_to_named_goal(
                self._data_dir,
                self._graph_file,
                cmd,
                self._graph_start_node_id,
                dock_pose,
                stamp=stamp,
                current_pose=cur,
                auto_start_policy=self._graph_auto_start_policy,
                path_dot_min=self._graph_path_dot_min,
            )
            if not planned:
                self.get_logger().warn(
                    f"도킹 그래프 경로 실패: {cmd} (docking_goal_links / 간선 확인)"
                )
                return
            poses, eff_start = planned
            self._nav_targets = poses
            self._nav_target_idx = 0
            self._after_nav_is_docking = True
            self._after_nav_is_dock_station = cmd == DOCK_STATION_CMD
            self.get_logger().info(
                f"도킹 그래프 {cmd!r} {len(poses)}포인트, 시작={eff_start}"
            )
        else:
            self._nav_targets = [dock_pose]
            self._nav_target_idx = 0
            self._after_nav_is_docking = True
            self._after_nav_is_dock_station = cmd == DOCK_STATION_CMD
            self.get_logger().info(f"도킹 직선 {cmd!r}")

        self._state = State.NAVIGATING
        self._send_current_nav_goal()

    def _navigate_to_warehouse_home(self, stamp) -> bool:
        """line_positions 의 ``warehouse`` 또는 최대 line_N 목표. 그래프는 ``line_goal_links`` 의 동일 키."""
        resolved = warehouse_pose_and_graph_link_key(self._data_dir, stamp=stamp)
        if resolved is None:
            self.get_logger().warn(
                "warehouse: line_positions 에 warehouse 또는 line_N 없음."
            )
            return False
        pose, link_key = resolved

        if self._use_graph:
            cur = self._last_robot_pose
            if cur is None:
                self.get_logger().error("warehouse: 그래프 모드인데 pose 미수신.")
                return False
            planned = plan_path_to_line_goal_link_key(
                self._data_dir,
                self._graph_file,
                link_key,
                self._graph_start_node_id,
                pose,
                stamp=stamp,
                current_pose=cur,
                auto_start_policy=self._graph_auto_start_policy,
                path_dot_min=self._graph_path_dot_min,
            )
            if not planned:
                self.get_logger().warn(
                    f"warehouse 경로 실패 (line_goal_links.{link_key!r}·간선·역행 검사)."
                )
                return False
            poses, eff_start = planned
            self._nav_targets = poses
            self._nav_target_idx = 0
            self.get_logger().info(
                f"warehouse → link {link_key!r}: {len(poses)}포인트, 시작노드={eff_start}"
            )
        else:
            self._nav_targets = [pose]
            self._nav_target_idx = 0
            self.get_logger().info(
                f"warehouse → link {link_key!r} 직선 (그래프 미사용)."
            )

        self._line_number = None
        self._after_nav_is_docking = False
        self._after_nav_is_warehouse_home = True
        self._state = State.NAVIGATING
        self._send_current_nav_goal()
        return True

    def _cb_line_move(self, msg: String) -> None:
        if self._state != State.IDLE:
            self.get_logger().warn(f"현재 상태 {self._state.value} 에서는 line_move 무시.")
            return
        raw = (msg.data or "").strip()
        if not raw:
            return
        self._line_before_dock = None
        self._from_dock_return = False
        stamp = self.get_clock().now().to_msg()
        if raw.lower() == "warehouse":
            self._navigate_to_warehouse_home(stamp)
            return
        try:
            n = int(raw, 10)
        except ValueError:
            self.get_logger().warn(f"line_move 알 수 없는 값: {raw!r} (숫자 또는 warehouse).")
            return
        if n < 1 or n > self._line_number_max:
            self.get_logger().warn(
                f"라인 {n} 은 허용 범위 밖 (1~{self._line_number_max}). 무시."
            )
            return
        self._navigate_to_line(n, stamp)

    def _desired_nav_yaw_tolerance(self) -> Optional[float]:
        """그래프 경유 중간만 완화. None 이면 controller 파라미터 건드리지 않음."""
        if not self._graph_segment_relax_yaw:
            return None
        n = len(self._nav_targets)
        idx = self._nav_target_idx
        if n > 1 and idx < n - 1:
            return self._graph_relaxed_yaw_tolerance
        return self._final_goal_yaw_tolerance

    def _dispatch_nav_goal_pose(self, pose: PoseStamped) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._nav_goal_pose_in_output_frame(pose)
        self._nav_client.wait_for_server(timeout_sec=5.0)
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._cb_nav_goal_done)

    def _cb_yaw_tolerance_set(
        self,
        future: Future,
        pose: PoseStamped,
        desired: float,
        tried_name: str,
        names: Tuple[str, ...],
        name_index: int,
    ) -> None:
        ok = False
        try:
            resp = future.result()
            ok = all(r.successful for r in resp.results)
            if not ok:
                for r in resp.results:
                    if not r.successful:
                        self.get_logger().debug(
                            f"yaw_goal_tolerance {tried_name!r}: {r.reason}"
                        )
        except Exception as e:
            self.get_logger().debug(f"yaw_goal_tolerance 설정 예외 {tried_name!r}: {e}")

        if ok:
            self._resolved_yaw_tol_param_name = tried_name
            self._last_set_yaw_goal_tolerance = desired
            self._dispatch_nav_goal_pose(pose)
            return

        nxt = name_index + 1
        if nxt < len(names):
            self._set_yaw_tolerance_try_name(pose, desired, names, nxt)
            return

        if not self._warned_yaw_tol_all_names_failed:
            self.get_logger().warn(
                "yaw_goal_tolerance 파라미터를 설정하지 못함. "
                f"시도한 이름: {list(names)}. "
                f"`ros2 param list /{self._nav2_controller_node} | grep yaw` 로 실제 키 확인 후 "
                "nav2_yaw_goal_tolerance_param 으로 지정. (그래프 중간 구간 yaw 완화만 영향)"
            )
            self._warned_yaw_tol_all_names_failed = True
        self._dispatch_nav_goal_pose(pose)

    def _set_yaw_tolerance_try_name(
        self,
        pose: PoseStamped,
        desired: float,
        names: Tuple[str, ...],
        name_index: int,
    ) -> None:
        client = self._nav2_param_client
        if client is None:
            self._dispatch_nav_goal_pose(pose)
            return
        if name_index >= len(names):
            if not self._warned_yaw_tol_all_names_failed:
                self.get_logger().warn(
                    "yaw_goal_tolerance 이름 목록 소진. nav2_yaw_goal_tolerance_param 확인."
                )
                self._warned_yaw_tol_all_names_failed = True
            self._dispatch_nav_goal_pose(pose)
            return
        pname = names[name_index]
        param = Parameter(pname, Parameter.Type.DOUBLE, desired)
        fut = client.set_parameters([param])
        fut.add_done_callback(
            lambda f: self._cb_yaw_tolerance_set(
                f, pose, desired, pname, names, name_index
            )
        )

    def _send_current_nav_goal(self) -> None:
        if self._nav_target_idx >= len(self._nav_targets):
            self.get_logger().error("내비 목표 인덱스 오류.")
            self._after_nav_is_docking = False
            self._after_nav_is_warehouse_home = False
            self._from_dock_return = False
            self._state = State.IDLE
            return
        pose = self._nav_targets[self._nav_target_idx]
        desired_tol = self._desired_nav_yaw_tolerance()
        if desired_tol is None:
            self._dispatch_nav_goal_pose(pose)
            return
        client = self._nav2_param_client
        if client is None or not client.services_are_ready():
            if not self._warned_nav2_param_unavailable:
                self.get_logger().warn(
                    f"Nav2 파라미터 클라이언트 준비 안 됨 ({self._nav2_controller_node}); "
                    "그래프 경유도 yaw_goal_tolerance 미조정 → 중간 노드에서 방향 정렬 시도될 수 있음."
                )
                self._warned_nav2_param_unavailable = True
            self._dispatch_nav_goal_pose(pose)
            return
        if (
            self._last_set_yaw_goal_tolerance is not None
            and abs(self._last_set_yaw_goal_tolerance - desired_tol) < 1e-5
        ):
            self._dispatch_nav_goal_pose(pose)
            return
        names = (
            (self._resolved_yaw_tol_param_name,)
            if self._resolved_yaw_tol_param_name
            else self._nav2_yaw_tol_param_names
        )
        self._set_yaw_tolerance_try_name(pose, desired_tol, names, 0)

    def _cb_nav_goal_done(self, future: Future) -> None:
        try:
            self._nav_goal_handle = future.result()
            if not self._nav_goal_handle.accepted:
                self.get_logger().error("Nav2 목표 거부.")
                self._nav_targets = []
                self._nav_target_idx = 0
                self._after_nav_is_docking = False
                self._after_nav_is_warehouse_home = False
                self._from_dock_return = False
                self._state = State.IDLE
                return
            self._nav_result_future = self._nav_goal_handle.get_result_async()
            self._nav_result_future.add_done_callback(self._cb_nav_result_done)
        except Exception as e:
            self.get_logger().error(f"Nav2 goal 전송 실패: {e}")
            self._nav_targets = []
            self._nav_target_idx = 0
            self._after_nav_is_docking = False
            self._after_nav_is_warehouse_home = False
            self._from_dock_return = False
            self._state = State.IDLE

    def _cb_nav_result_done(self, future: Future) -> None:
        try:
            future.result().result
            status = future.result().status
            if status != 4:  # SUCCEEDED = 4
                self.get_logger().warn(f"Nav2 이동 완료 (성공 아님): status={status}")
                self._nav_targets = []
                self._nav_target_idx = 0
                self._after_nav_is_docking = False
                self._after_nav_is_warehouse_home = False
                self._from_dock_return = False
                self._state = State.IDLE
                return
            self.get_logger().info(
                f"Nav2 세그먼트 완료 ({self._nav_target_idx + 1}/{len(self._nav_targets)})."
            )
            self._nav_target_idx += 1
            if self._nav_target_idx < len(self._nav_targets):
                self._send_current_nav_goal()
                return
            self._nav_targets = []
            self._nav_target_idx = 0
            if self._after_nav_is_docking:
                self._after_nav_is_docking = False
                self._line_number = None
                is_station = self._after_nav_is_dock_station
                self._after_nav_is_dock_station = False
                if is_station and self._enable_final_docking:
                    self.get_logger().info(
                        "도킹 스테이션 도착 → 최종 도킹 시작 "
                        f"(대상 {self._dock_target_frame} 후방 {self._dock_rear_offset}m)"
                    )
                    self._start_docking()
                    return
                self._state = State.IDLE
                self.get_logger().info(
                    "도킹 스테이션 도착 → IDLE (entering_start 생략). 복귀: move_to_return"
                )
                return
            if self._after_nav_is_warehouse_home:
                self._after_nav_is_warehouse_home = False
                self._line_number = None
                self._state = State.IDLE
                self.get_logger().info("warehouse 홈 도착 → IDLE (entering_start 생략).")
                return
            self._state = State.SEND_ENTERING_START
            self._do_entering_start()
        except Exception as e:
            self.get_logger().error(f"Nav2 result 수신 오류: {e}")
            self._nav_targets = []
            self._nav_target_idx = 0
            self._after_nav_is_docking = False
            self._after_nav_is_warehouse_home = False
            self._from_dock_return = False
            self._state = State.IDLE

    def _do_entering_start(self) -> None:
        msg = String()
        msg.data = "entering_start"
        self._action_pub.publish(msg)
        self.get_logger().info("/action 발행: entering_start")
        self._entering_check_received = False
        self._entering_check_retry_count = 0
        self._state = State.WAIT_ENTERING_CHECK
        self._entering_check_timer = self.create_timer(
            self._entering_check_timeout, self._cb_entering_check_timeout
        )

    def _cb_entering_check_timeout(self) -> None:
        if self._state != State.WAIT_ENTERING_CHECK:
            return
        self._entering_check_timer.cancel()
        self._entering_check_timer = None
        if self._entering_check_received:
            self._state = State.WAIT_GOAL_FINISH
            self.get_logger().info("entering_check 수신됨 → goal_finish 대기.")
            return
        self._entering_check_retry_count += 1
        if self._entering_check_retry_count >= self._entering_check_max_retries:
            self.get_logger().warn("entering_check 미수신, 최대 재전송 횟수 도달 → 다음 단계로.")
            self._state = State.WAIT_GOAL_FINISH
            return
        msg = String()
        msg.data = "entering_start"
        self._action_pub.publish(msg)
        self.get_logger().info("entering_check 미수신 → entering_start 재전송.")
        self._entering_check_timer = self.create_timer(
            self._entering_check_timeout, self._cb_entering_check_timeout
        )

    def _cb_action_check(self, msg: String) -> None:
        data = msg.data
        if data == "entering_check" and self._state == State.WAIT_ENTERING_CHECK:
            self._entering_check_received = True
            if self._entering_check_timer is not None:
                self._entering_check_timer.cancel()
                self._entering_check_timer = None
            self._state = State.WAIT_GOAL_FINISH
            self.get_logger().info("entering_check 수신됨 → goal_finish 대기.")
        elif data == "goal_finish":
            self.get_logger().info("[goal_finish] 수신")
            print("[goal_finish] 수신", flush=True)
            if self._state == State.WAIT_GOAL_FINISH:
                self._state = State.WAIT_GOAL_RETURN_FINISH
                self.get_logger().info("goal_return_finish 대기 중...")
        elif data == "goal_return_finish":
            self.get_logger().info("[goal_return_finish] 수신 → 다시 라인 입력 대기.")
            print("[goal_return_finish] 수신 → 다시 라인 입력 대기.", flush=True)
            msg = String()
            msg.data = "entering_end2"
            self._action_pub.publish(msg)
            self.get_logger().info("/action 발행: entering_end2")
            if self._from_dock_return:
                self._from_dock_return = False
                self._line_before_dock = None
                self.get_logger().info("도킹 복귀 라인 사이클 완료 → 저장 라인 초기화")
            self._state = State.IDLE

    # =========================
    # 최종 도킹 (차동구동, TF 폐루프)
    # =========================
    def _publish_cmd(self, lin: float, ang: float) -> None:
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self._cmd_vel_pub.publish(t)

    def _stop_robot(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _start_docking(self) -> None:
        if self._tf_buffer is None:
            self.get_logger().error(
                "도킹 불가: TF 버퍼 없음 (enable_final_docking 확인). IDLE 로."
            )
            self._state = State.IDLE
            return
        self._state = State.DOCKING
        self._dock_settle_count = 0
        self._dock_last_tf_ok_ns = self.get_clock().now().nanoseconds
        period = 1.0 / self._dock_rate_hz
        self._dock_timer = self.create_timer(period, self._cb_dock_control)
        self.get_logger().info(
            f"도킹 제어 시작: {self._dock_robot_frame}→{self._dock_target_frame}, "
            f"cmd_vel={self._cmd_vel_topic}, rate={self._dock_rate_hz}Hz"
        )

    def _cancel_dock_timer(self) -> None:
        if self._dock_timer is not None:
            try:
                self._dock_timer.cancel()
            except Exception:
                pass
            self._dock_timer = None

    def _finish_docking(self) -> None:
        self._cancel_dock_timer()
        self._stop_robot()
        if self._dock_status_pub is not None:
            # 도킹 명령 토픽으로 상태 통지(debug_topic 등 수신). 자기수신은 콜백에서 무시.
            msg = String()
            msg.data = self._dock_status_value
            self._dock_status_pub.publish(msg)
        self.get_logger().info(f"도킹 완료 → '{self._dock_status_value}' 발행, IDLE.")
        print(f"[도킹 완료] {self._dock_status_value}", flush=True)
        self._line_number = None
        self._state = State.IDLE

    def _abort_docking(self, reason: str) -> None:
        self._cancel_dock_timer()
        self._stop_robot()
        self.get_logger().warn(f"도킹 중단: {reason} → IDLE.")
        print(f"[도킹 중단] {reason}", flush=True)
        self._state = State.IDLE

    def _cb_dock_control(self) -> None:
        if self._state != State.DOCKING:
            self._cancel_dock_timer()
            return

        now_ns = self.get_clock().now().nanoseconds
        try:
            tf = self._tf_buffer.lookup_transform(
                self._dock_robot_frame,
                self._dock_target_frame,
                Time(),
                timeout=Duration(seconds=self._dock_tf_timeout),
            )
        except Exception as e:
            self._stop_robot()
            lost_s = (now_ns - self._dock_last_tf_ok_ns) / 1e9
            if lost_s >= self._dock_lost_timeout:
                self._abort_docking(
                    f"TF {self._dock_robot_frame}←{self._dock_target_frame} {lost_s:.1f}s 미수신 ({e})"
                )
            return
        self._dock_last_tf_ok_ns = now_ns

        # 대상 로봇 원점(robot 프레임 기준)
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw_t = _yaw_from_quat(q.x, q.y, q.z, q.w)  # 대상 헤딩(robot 프레임)

        # 대상 후방(-x_target)으로 offset 만큼 떨어진 도킹 지점 (robot 프레임)
        dx = tx - self._dock_rear_offset * math.cos(yaw_t)
        dy = ty - self._dock_rear_offset * math.sin(yaw_t)

        rho = math.hypot(dx, dy)
        bearing = _norm_angle(math.atan2(dy, dx))   # 도킹 지점 방위
        heading_err = _norm_angle(yaw_t)            # 최종 정렬 오차(대상과 같은 헤딩)

        # 완료 판정: 위치·헤딩 동시에 임계값 안 + 연속 settle
        if rho <= self._dock_pos_tol and abs(heading_err) <= self._dock_yaw_tol:
            self._dock_settle_count += 1
            self._stop_robot()
            if self._dock_settle_count >= self._dock_settle_cycles:
                self._finish_docking()
            return
        self._dock_settle_count = 0

        if rho > self._dock_pos_tol:
            # 접근 단계
            if abs(bearing) > self._dock_coarse_ang:
                # 목표점이 옆/뒤 → 제자리 선회 우선
                lin = 0.0
                ang = self._dock_k_ang * bearing
            else:
                lin = self._dock_k_lin * rho
                ang = self._dock_k_ang * bearing
        else:
            # 위치 도달, 최종 헤딩 정렬(제자리 선회)
            lin = 0.0
            ang = self._dock_k_ang * heading_err

        lin = _apply_deadband(lin, self._dock_min_lin, self._dock_max_lin) if lin != 0.0 else 0.0
        ang = _apply_deadband(ang, self._dock_min_ang, self._dock_max_ang) if ang != 0.0 else 0.0
        self._publish_cmd(lin, ang)

    def destroy_node(self, *args, **kwargs):
        if getattr(self, "_entering_check_timer", None) is not None:
            try:
                self._entering_check_timer.cancel()
            except Exception:
                pass
        self._cancel_dock_timer()
        try:
            self._stop_robot()
        except Exception:
            pass
        super().destroy_node(*args, **kwargs)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
