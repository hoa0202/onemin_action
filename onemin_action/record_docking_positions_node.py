#!/usr/bin/env python3
"""
도킹 목표 포즈를 data/docking_positions.yaml 에 키 단위로 저장 (시나리오 load_pose_by_key 와 동일 스키마).

실행: 도킹 스테이션 자리에서 (복귀 move_to_return 은 시나리오가 저장 라인으로 처리, 여기서 안 찍음)
  ros2 run onemin_action record_docking_positions_node \\
    --ros-args -p save_key:=move_to_docking_station

  k : 현재 pose를 save_key 아래에 저장(기존 키 덮어씀)
  s / Ctrl+C : 종료
"""
import os
import select
import sys
import termios
import threading
import tty
from typing import Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty


def _get_package_data_dir() -> str:
    path = os.path.abspath(__file__)
    pkg_dir = os.path.dirname(os.path.dirname(path))
    parent = os.path.dirname(pkg_dir)
    if os.path.basename(parent) in ("build", "install"):
        workspace = os.path.dirname(parent)
        src_data = os.path.join(workspace, "src", "onemin_action", "data")
        if os.path.isdir(os.path.join(workspace, "src", "onemin_action")):
            return src_data
    return os.path.join(pkg_dir, "data")


def _pose_to_dict(frame_id: str, position, orientation) -> dict:
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


class RecordDockingPositionsNode(Node):
    def __init__(self):
        super().__init__("record_docking_positions_node")

        self.declare_parameter("pose_topic", "/liorf/mapping/odometry")
        self.declare_parameter("pose_type", "odom")
        self.declare_parameter("output_file", "docking_positions.yaml")
        self.declare_parameter("save_key", "move_to_docking_station")

        self._pose_topic = self.get_parameter("pose_topic").value
        self._pose_type = self.get_parameter("pose_type").value
        self._data_dir = _get_package_data_dir()
        self._basename = self.get_parameter("output_file").value
        self._save_key = str(self.get_parameter("save_key").value or "").strip()
        if not self._save_key:
            self._save_key = "move_to_docking_station"

        self._output_file = os.path.join(self._data_dir, self._basename)
        os.makedirs(self._data_dir, exist_ok=True)

        self._last_pose: Optional[dict] = None
        self._record_lock = threading.Lock()

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
            self.create_subscription(
                Odometry, self._pose_topic, self._cb_pose_odom, 10
            )

        self.create_subscription(
            Empty, "/record_docking_position", self._cb_record_trigger, 10
        )

        self.get_logger().info(
            f"save_key={self._save_key} | {self._output_file} | "
            f"{self._pose_topic} ({self._pose_type})"
        )
        if sys.stdin.isatty():
            hint = "k: 저장, s 또는 Ctrl+C: 종료"
        else:
            hint = (
                "저장: ros2 topic pub --once /record_docking_position std_msgs/msg/Empty '{}'\n"
                "  또는 터미널에서 ros2 run ... record_docking_positions_node"
            )
        self.get_logger().info(hint)
        print(hint, flush=True)

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
            msg.header.frame_id, msg.pose.position, msg.pose.orientation
        )

    def _cb_record_trigger(self, _msg: Empty) -> None:
        self._save()

    def _save(self) -> None:
        with self._record_lock:
            if self._last_pose is None:
                self.get_logger().warn(f"pose 없음: {self._pose_topic}")
                return
            existing: dict = {}
            if os.path.isfile(self._output_file):
                with open(self._output_file, "r", encoding="utf-8") as f:
                    existing = yaml.safe_load(f) or {}
            existing[self._save_key] = self._last_pose
            with open(self._output_file, "w", encoding="utf-8") as f:
                yaml.dump(
                    existing, f, default_flow_style=False, allow_unicode=True, sort_keys=False
                )
            p = self._last_pose["position"]
            self.get_logger().info(
                f"[{self._save_key}] 저장 완료 x={p['x']}, y={p['y']} | {self._output_file}"
            )
            print(f"[{self._save_key}] 저장 완료 | {self._output_file}", flush=True)


def _stdin_ready():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def _get_key():
    if _stdin_ready():
        return sys.stdin.read(1)
    return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecordDockingPositionsNode()

    if sys.stdin.isatty():
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        old_attr = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = _get_key()
                if key == "k":
                    node._save()
                elif key == "s" or key == "\x03":
                    break
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_attr)
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
    print("종료 (도킹 포즈 기록)", flush=True)


if __name__ == "__main__":
    main()
