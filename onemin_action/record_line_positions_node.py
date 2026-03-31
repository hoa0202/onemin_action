#!/usr/bin/env python3
"""
코드1: 로봇을 수동으로 각 라인 입구 출발지점에 세운 뒤,
현재 위치·방향을 패키지 내 data/line_positions.yaml 에 1번부터 저장.
저장 형식: RViz2 Nav2 Goal과 동일 (geometry_msgs/PoseStamped).

실행: ros2 run onemin_action record_line_positions_node
  k : 현재 위치 저장 (1번부터 자동 증가)
  s 또는 Ctrl+C : 종료
"""
import os
import select
import sys
import termios
import threading
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import yaml


def _get_package_data_dir() -> str:
    """소스 패키지의 data 디렉터리 (colcon 시 workspace/src/<패키지명>/data)."""
    path = os.path.abspath(__file__)
    # .../workspace/build/onemin_action/onemin_action/record_... or .../install/.../onemin_action/record_...
    pkg_dir = os.path.dirname(os.path.dirname(path))
    parent = os.path.dirname(pkg_dir)
    if os.path.basename(parent) in ("build", "install"):
        workspace = os.path.dirname(parent)
        src_data = os.path.join(workspace, "src", "onemin_action", "data")
        if os.path.isdir(os.path.join(workspace, "src", "onemin_action")):
            return src_data
    return os.path.join(pkg_dir, "data")


def _pose_to_dict(frame_id: str, position, orientation) -> dict:
    """Nav2 goal(PoseStamped) 형식: position x,y,z + orientation quaternion."""
    return {
        "frame_id": frame_id,
        "position": {
            "x": round(position.x, 6),
            "y": round(position.y, 6),
            "z": round(position.z, 6),
        },
        "orientation": {
            "x": round(orientation.x, 6),
            "y": round(orientation.y, 6),
            "z": round(orientation.z, 6),
            "w": round(orientation.w, 6),
        },
    }


class RecordLinePositionsNode(Node):
    def __init__(self):
        super().__init__("record_line_positions_node")

        self.declare_parameter("pose_topic", "/liorf/mapping/odometry")
        self.declare_parameter("pose_type", "odom")  # odom | amcl | pose_stamped

        self._pose_topic = self.get_parameter("pose_topic").value
        self._pose_type = self.get_parameter("pose_type").value
        self._data_dir = _get_package_data_dir()
        self._output_file = os.path.join(self._data_dir, "line_positions.yaml")
        os.makedirs(self._data_dir, exist_ok=True)

        self._last_pose: Optional[dict] = None
        self._next_line_num = self._load_next_line_number()
        self._record_lock = threading.Lock()

        if self._pose_type == "pose_stamped":
            self._pose_sub = self.create_subscription(
                PoseStamped,
                self._pose_topic,
                self._cb_pose_stamped,
                10,
            )
        elif self._pose_type == "amcl":
            self._pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self._pose_topic,
                self._cb_pose_amcl,
                10,
            )
        else:
            self._pose_sub = self.create_subscription(
                Odometry,
                self._pose_topic,
                self._cb_pose_odom,
                10,
            )

        self._record_sub = self.create_subscription(
            Empty,
            "/record_line_position",
            self._cb_record_trigger,
            10,
        )

        self.get_logger().info(
            f"pose_topic={self._pose_topic} | pose_type={self._pose_type} | 저장: {self._output_file}"
        )
        if sys.stdin.isatty():
            hint = "k: 저장, s 또는 Ctrl+C: 종료"
        else:
            hint = (
                "저장: 다른 터미널에서 아래 한 줄 실행 (k 입력 불가, launch는 stdin 미연결)\n"
                "  ros2 topic pub --once /record_line_position std_msgs/Empty '{}'\n"
                "  또는 k/s 쓰려면: ros2 run onemin_action record_line_positions_node"
            )
        self.get_logger().info(hint)
        print(hint, flush=True)

    def _load_next_line_number(self) -> int:
        path = os.path.join(self._data_dir, "line_positions.yaml")
        if not os.path.isfile(path):
            return 1
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            keys = [k for k in data if isinstance(k, str) and k.startswith("line_")]
            if not keys:
                return 1
            nums = []
            for k in keys:
                try:
                    nums.append(int(k.replace("line_", "")))
                except ValueError:
                    continue
            return max(nums) + 1 if nums else 1
        except Exception:
            return 1

    def _cb_pose_odom(self, msg: Odometry) -> None:
        self._last_pose = _pose_to_dict(
            msg.header.frame_id,
            msg.pose.pose.position,
            msg.pose.pose.orientation,
        )

    def _cb_pose_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_pose = _pose_to_dict(
            msg.header.frame_id,
            msg.pose.pose.position,
            msg.pose.pose.orientation,
        )

    def _cb_pose_stamped(self, msg: PoseStamped) -> None:
        self._last_pose = _pose_to_dict(
            msg.header.frame_id,
            msg.pose.position,
            msg.pose.orientation,
        )

    def _cb_record_trigger(self, _msg: Empty) -> None:
        self._save_current_position()

    def _save_current_position(self) -> None:
        with self._record_lock:
            line_num = self._next_line_num
            if self._last_pose is None:
                msg = (
                    f"저장 실패: pose 수신 없음. "
                    f"'{self._pose_topic}' 토픽이 발행 중인지 확인 후 k 다시 입력."
                )
                self.get_logger().warn(msg)
                print(msg, flush=True)
                return
            key = f"line_{line_num}"
            path = self._output_file
            existing: dict = {}
            if os.path.isfile(path):
                with open(path, "r", encoding="utf-8") as f:
                    existing = yaml.safe_load(f) or {}
            existing[key] = self._last_pose
            with open(path, "w", encoding="utf-8") as f:
                yaml.dump(
                    existing, f, default_flow_style=False, allow_unicode=True, sort_keys=False
                )
            p = self._last_pose["position"]
            o = self._last_pose["orientation"]
            msg = (
                f"[{line_num}번째 저장 완료] {key} | frame_id={self._last_pose['frame_id']} | "
                f"x={p['x']}, y={p['y']}, z={p['z']} | qx={o['x']}, qy={o['y']}, qz={o['z']}, qw={o['w']} | "
                f"파일: {path}"
            )
            self.get_logger().info(msg)
            print(msg, flush=True)
            self._next_line_num += 1


def _stdin_ready():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def _get_key():
    if _stdin_ready():
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = RecordLinePositionsNode()

    if sys.stdin.isatty():
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        old_attr = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = _get_key()
                if key == "k":
                    node._save_current_position()
                elif key == "s" or key == "\x03":  # s or Ctrl+C
                    break
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    print("종료 (완료)")


if __name__ == "__main__":
    main()
