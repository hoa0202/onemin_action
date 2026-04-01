#!/usr/bin/env python3
"""
코드3: 제3의 명령 터미널.
  /action:       entering_next, entering_end
  /action_check: entering_check, goal_finish, goal_return_finish

실행:
  ros2 run onemin_action manual_action_command_node                    # 대화형
  ros2 run onemin_action manual_action_command_node entering_next      # 한 번만 전송
  ros2 run onemin_action manual_action_command_node entering_end
  ros2 run onemin_action manual_action_command_node entering_check
  ros2 run onemin_action manual_action_command_node goal_finish
  ros2 run onemin_action manual_action_command_node goal_return_finish
  ros2 run onemin_action manual_action_command_node 1   # std_msgs/String → line_move_topic (기본 /harv_robot/line_move)
  ros2 run onemin_action manual_action_command_node 11  # line_number_max 이하 숫자 문자열
  ros2 run onemin_action manual_action_command_node warehouse
  ros2 run onemin_action manual_action_command_node move_to_docking_station
  ros2 run onemin_action manual_action_command_node move_to_return

파라미터 예:
  --ros-args -p line_move_topic:=/harv_robot/line_move -p line_number_max:=12 -p docking_topic:=/harv_robot/docking_move
"""
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DOCK_CMDS = frozenset({"move_to_docking_station", "move_to_return"})


class ManualActionCommandNode(Node):
    def __init__(self):
        super().__init__("manual_action_command_node")
        self.declare_parameter("docking_topic", "/harv_robot/docking_move")
        self._docking_topic = str(self.get_parameter("docking_topic").value or "")

        self._pub_action = self.create_publisher(String, "/action", 10)
        self._pub_action_check = self.create_publisher(String, "/action_check", 10)
        self.declare_parameter("line_move_topic", "/harv_robot/line_move")
        _line_topic = str(self.get_parameter("line_move_topic").value or "").strip()
        self._pub_line = self.create_publisher(String, _line_topic or "/harv_robot/line_move", 10)
        self.declare_parameter("line_number_max", 10)
        try:
            self._line_number_max = int(self.get_parameter("line_number_max").value)
        except (TypeError, ValueError):
            self._line_number_max = 10
        if self._line_number_max < 1:
            self._line_number_max = 10
        if self._docking_topic:
            self._pub_docking = self.create_publisher(String, self._docking_topic, 10)
        else:
            self._pub_docking = None

    def send_line_move(self, data: str) -> None:
        msg = String()
        msg.data = data.strip()
        self._pub_line.publish(msg)
        self.get_logger().info(f"line_move 발행: {msg.data!r}")
        print(f"line_move 발행: {msg.data!r}", flush=True)

    def send(self, data: str) -> None:
        msg = String()
        msg.data = data
        self._pub_action.publish(msg)
        self.get_logger().info(f"/action 발행: '{data}'")
        print(f"/action 발행: '{data}'", flush=True)

    def send_check(self, data: str) -> None:
        msg = String()
        msg.data = data
        self._pub_action_check.publish(msg)
        self.get_logger().info(f"/action_check 발행: '{data}'")
        print(f"/action_check 발행: '{data}'", flush=True)

    def send_docking(self, data: str) -> None:
        if not self._pub_docking:
            self.get_logger().warn("docking_topic 비어 있음")
            return
        msg = String()
        msg.data = data
        self._pub_docking.publish(msg)
        self.get_logger().info(f"{self._docking_topic} 발행: '{data}'")
        print(f"{self._docking_topic} 발행: '{data}'", flush=True)


def _stdin_ready():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def _get_key():
    if _stdin_ready():
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = ManualActionCommandNode()

    # 인자로 명령이 오면 한 번 발행 후 종료
    if len(sys.argv) > 1:
        cmd = sys.argv[1].strip()
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        if cmd in ("entering_next", "entering_end"):
            node.send(cmd)
        elif cmd in ("entering_check", "goal_finish", "goal_return_finish"):
            node.send_check(cmd)
        elif cmd.lower() == "warehouse":
            node.send_line_move("warehouse")
        elif cmd.isdigit():
            n = int(cmd, 10)
            if 1 <= n <= node._line_number_max:
                node.send_line_move(cmd)
            else:
                print(
                    f"라인 번호 1~{node._line_number_max} 만 허용: {cmd}",
                    file=sys.stderr,
                )
        elif cmd in DOCK_CMDS:
            node.send_docking(cmd)
        else:
            print(
                f"알 수 없는 명령: {cmd} (entering_* | goal_* | 1~{node._line_number_max} | warehouse | move_to_* )",
                file=sys.stderr,
            )
        node.destroy_node()
        rclpy.shutdown()
        return

    # 대화형: TTY 1~9·0(=10, line_number_max>=10일 때) = line_move; w=warehouse; 10 초과는 파이프/줄 입력으로 숫자
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    _mx = node._line_number_max
    _zero_hint = ", 0→10" if _mx >= 10 else ""
    print(
        f"라인 1~{_mx} (TTY: 1~9{_zero_hint}; 10+는 줄 입력 '10' 등) | w: warehouse | "
        "n/e/c/f/r | d/b 도킹 | q 종료",
        flush=True,
    )

    if sys.stdin.isatty():
        old_attr = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = _get_key()
                if key is None:
                    continue
                if key in "123456789":
                    n = int(key)
                    if n <= node._line_number_max:
                        node.send_line_move(key)
                    else:
                        print(f"라인 상한 {node._line_number_max}", flush=True)
                elif key == "0":
                    if node._line_number_max >= 10:
                        node.send_line_move("10")
                    else:
                        print(f"0 키(라인 10) 비활성: line_number_max={node._line_number_max}", flush=True)
                elif key == "w":
                    node.send_line_move("warehouse")
                elif key == "n":
                    node.send("entering_next")
                elif key == "e":
                    node.send("entering_end")
                elif key == "c":
                    node.send_check("entering_check")
                elif key == "f":
                    node.send_check("goal_finish")
                elif key == "r":
                    node.send_check("goal_return_finish")
                elif key == "d":
                    node.send_docking("move_to_docking_station")
                elif key == "b":
                    node.send_docking("move_to_return")
                elif key == "q" or key == "\x03":
                    break
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    else:
        try:
            while True:
                line = input().strip()
                low = line.lower()
                if low == "warehouse":
                    node.send_line_move("warehouse")
                elif line.isdigit():
                    n = int(line, 10)
                    if 1 <= n <= node._line_number_max:
                        node.send_line_move(line)
                    else:
                        print(f"라인 1~{node._line_number_max} 만 허용.", flush=True)
                elif low in ("n", "entering_next"):
                    node.send("entering_next")
                elif low in ("e", "entering_end"):
                    node.send("entering_end")
                elif low in ("c", "entering_check"):
                    node.send_check("entering_check")
                elif low in ("f", "goal_finish"):
                    node.send_check("goal_finish")
                elif low in ("r", "goal_return_finish"):
                    node.send_check("goal_return_finish")
                elif low in ("d", "move_to_docking_station", "dock"):
                    node.send_docking("move_to_docking_station")
                elif low in ("b", "move_to_return", "return"):
                    node.send_docking("move_to_return")
                elif low in ("q", "quit"):
                    break
        except (EOFError, KeyboardInterrupt):
            pass

    node.destroy_node()
    rclpy.shutdown()
    print("종료")


if __name__ == "__main__":
    main()
