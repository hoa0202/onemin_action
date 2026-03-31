#!/usr/bin/env python3
"""
코드2: 전체 시나리오 제어.
진입 라인 번호 수신 → YAML에서 좌표 로드 → Nav2 이동 → entering_start → entering_check 확인 → goal_finish 로그 → goal_return_finish 대기 → 반복.

라인 입력: /line_to_enter (std_msgs/Int32) 로 번호 전송. (1, 2, 3…, 0 미사용)
"""
import enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from onemin_action.line_pose_loader import get_data_dir, load_line_pose


class State(enum.Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
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

        self._nav2_action_name = self.get_parameter("nav2_action_name").value
        self._entering_check_timeout = self.get_parameter("entering_check_timeout_sec").value
        self._entering_check_max_retries = self.get_parameter("entering_check_max_retries").value
        self._data_dir = get_data_dir()

        self._state = State.IDLE
        self._line_number: Optional[int] = None
        self._entering_check_received = False
        self._entering_check_retry_count = 0
        self._goal_finish_received = False
        self._goal_return_finish_received = False
        self._entering_check_timer: Optional[Node.Timer] = None
        self._nav_goal_handle = None
        self._nav_result_future: Optional[Future] = None

        self._action_pub = self.create_publisher(String, "/action", 10)
        self._action_check_sub = self.create_subscription(
            String, "/action_check", self._cb_action_check, 10
        )
        self._line_sub = self.create_subscription(
            Int32, "/line_to_enter", self._cb_line_to_enter, 10
        )
        self._nav_client = ActionClient(self, NavigateToPose, self._nav2_action_name)

        self.get_logger().info(
            f"시나리오 제어 대기. /line_to_enter (Int32) 로 진입할 라인 번호(1,2,3…) 전송."
        )
        self.get_logger().info(f"Nav2 액션: {self._nav2_action_name} | data: {self._data_dir}")

    def _cb_line_to_enter(self, msg: Int32) -> None:
        if self._state != State.IDLE:
            self.get_logger().warn(f"현재 상태 {self._state.value} 에서는 라인 입력 무시.")
            return
        n = msg.data
        if n < 1:
            self.get_logger().warn("라인 번호는 1 이상만 사용 (0 미사용). 무시.")
            return
        pose = load_line_pose(
            self._data_dir, n, stamp=self.get_clock().now().to_msg()
        )
        if pose is None:
            self.get_logger().warn(f"라인 {n} 이(가) 없습니다. (YAML에 해당 번호 없음). 다음 라인 입력 대기.")
            print(f"라인 {n} 없음. 다음 입력 대기.", flush=True)
            return
        self._line_number = n
        self._state = State.NAVIGATING
        self.get_logger().info(f"라인 {n} 이동 시작.")
        self._send_nav_goal(pose)

    def _send_nav_goal(self, pose: PoseStamped) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._nav_client.wait_for_server(timeout_sec=5.0)
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._cb_nav_goal_done)

    def _cb_nav_goal_done(self, future: Future) -> None:
        try:
            self._nav_goal_handle = future.result()
            if not self._nav_goal_handle.accepted:
                self.get_logger().error("Nav2 목표 거부.")
                self._state = State.IDLE
                return
            self._nav_result_future = self._nav_goal_handle.get_result_async()
            self._nav_result_future.add_done_callback(self._cb_nav_result_done)
        except Exception as e:
            self.get_logger().error(f"Nav2 goal 전송 실패: {e}")
            self._state = State.IDLE

    def _cb_nav_result_done(self, future: Future) -> None:
        try:
            result = future.result().result
            # Nav2 result has no direct success; status in result wrapper
            status = future.result().status
            if status != 4:  # SUCCEEDED = 4
                self.get_logger().warn(f"Nav2 이동 완료 (성공 아님): status={status}")
            else:
                self.get_logger().info("Nav2 이동 완료.")
            self._state = State.SEND_ENTERING_START
            self._do_entering_start()
        except Exception as e:
            self.get_logger().error(f"Nav2 result 수신 오류: {e}")
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
            self._state = State.IDLE

    def destroy_node(self, *args, **kwargs):
        if getattr(self, "_entering_check_timer", None) is not None:
            try:
                self._entering_check_timer.cancel()
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
