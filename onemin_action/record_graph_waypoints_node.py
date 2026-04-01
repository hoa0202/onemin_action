#!/usr/bin/env python3
"""
중간 웨이포인트를 data/waypoint_graph.yaml 의 nodes/edges 로 저장.
Dijkstra_example 의 setup_graph 와 같이 간선을 만들려면 edge_build_mode:=chain_bidirectional
(연속 wp_N 양방향 체인 + graph_build.extra_bidirectional_pairs).
라인 목표는 record_line_positions_node 로 별도; line_goal_links 는 YAML에서 수동.

실행: ros2 run onemin_action record_graph_waypoints_node
  k : 현재 pose로 노드 저장 (wp_1, wp_2, … 자동 id)
  s 또는 Ctrl+C : 종료
토픽: ros2 topic pub --once /record_graph_waypoint std_msgs/msg/Empty '{}'
"""
import os
import select
import sys
import termios
import threading
import tty
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty

from onemin_action.waypoint_graph_builder import apply_graph_build


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


def _pose_to_entry(frame_id: str, position, orientation) -> dict:
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


class RecordGraphWaypointsNode(Node):
    def __init__(self):
        super().__init__("record_graph_waypoints_node")

        self.declare_parameter("pose_topic", "/liorf/mapping/odometry")
        self.declare_parameter("pose_type", "odom")
        self.declare_parameter("node_id_prefix", "wp")
        self.declare_parameter("waypoint_graph_file", "waypoint_graph.yaml")
        self.declare_parameter("auto_edge_from_previous", True)
        self.declare_parameter("auto_edge_bidirectional", False)
        # incremental: 찍은 순서대로 간선만 추가 | chain_bidirectional: Dijkstra_example 식 전체 체인 재생성
        self.declare_parameter("edge_build_mode", "chain_bidirectional")

        self._pose_topic = self.get_parameter("pose_topic").value
        self._pose_type = self.get_parameter("pose_type").value
        self._prefix = self.get_parameter("node_id_prefix").value
        self._graph_file = self.get_parameter("waypoint_graph_file").value
        self._auto_edge = self.get_parameter("auto_edge_from_previous").value
        self._auto_edge_bidi = self.get_parameter("auto_edge_bidirectional").value
        self._edge_build_mode = str(
            self.get_parameter("edge_build_mode").value or "incremental"
        ).strip()

        self._data_dir = _get_package_data_dir()
        os.makedirs(self._data_dir, exist_ok=True)
        self._output_path = os.path.join(self._data_dir, self._graph_file)

        self._last_pose: Optional[dict] = None
        self._prev_node_id: Optional[str] = None
        self._record_lock = threading.Lock()

        if self._pose_type == "pose_stamped":
            self._pose_sub = self.create_subscription(
                PoseStamped, self._pose_topic, self._cb_pose_stamped, 10
            )
        elif self._pose_type == "amcl":
            self._pose_sub = self.create_subscription(
                PoseWithCovarianceStamped, self._pose_topic, self._cb_pose_amcl, 10
            )
        else:
            self._pose_sub = self.create_subscription(
                Odometry, self._pose_topic, self._cb_pose_odom, 10
            )

        self._record_sub = self.create_subscription(
            Empty, "/record_graph_waypoint", self._cb_record_trigger, 10
        )

        self.get_logger().info(
            f"pose_topic={self._pose_topic} | pose_type={self._pose_type} | "
            f"저장: {self._output_path} | prefix={self._prefix} | "
            f"edge_build_mode={self._edge_build_mode}"
        )
        if sys.stdin.isatty():
            hint = "k: 노드 저장, s 또는 Ctrl+C: 종료"
        else:
            hint = (
                "저장: ros2 topic pub --once /record_graph_waypoint std_msgs/msg/Empty '{}'\n"
                "  또는 터미널에서 직접 실행 시 k/s 사용"
            )
        self.get_logger().info(hint)
        print(hint, flush=True)

    def _next_node_id(self, nodes: Dict[str, Any]) -> str:
        prefix = f"{self._prefix}_"
        nums: List[int] = []
        for k in nodes:
            if isinstance(k, str) and k.startswith(prefix):
                try:
                    nums.append(int(k[len(prefix) :]))
                except ValueError:
                    continue
        n = max(nums) + 1 if nums else 1
        return f"{prefix}{n}"

    def _load_doc(self) -> dict:
        if not os.path.isfile(self._output_path):
            return {"nodes": {}, "edges": [], "line_goal_links": {}}
        with open(self._output_path, "r", encoding="utf-8") as f:
            doc = yaml.safe_load(f) or {}
        if "nodes" not in doc:
            doc["nodes"] = {}
        if "edges" not in doc:
            doc["edges"] = []
        if "line_goal_links" not in doc:
            doc["line_goal_links"] = {}
        if "graph_build" not in doc:
            doc["graph_build"] = {}
        return doc

    def _cb_pose_odom(self, msg: Odometry) -> None:
        self._last_pose = _pose_to_entry(
            msg.header.frame_id,
            msg.pose.pose.position,
            msg.pose.pose.orientation,
        )

    def _cb_pose_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_pose = _pose_to_entry(
            msg.header.frame_id,
            msg.pose.pose.position,
            msg.pose.pose.orientation,
        )

    def _cb_pose_stamped(self, msg: PoseStamped) -> None:
        self._last_pose = _pose_to_entry(
            msg.header.frame_id, msg.pose.position, msg.pose.orientation
        )

    def _cb_record_trigger(self, _msg: Empty) -> None:
        self._save_node()

    def _save_node(self) -> None:
        with self._record_lock:
            if self._last_pose is None:
                msg = f"저장 실패: pose 없음. '{self._pose_topic}' 확인."
                self.get_logger().warn(msg)
                print(msg, flush=True)
                return
            doc = self._load_doc()
            nodes: Dict[str, Any] = doc["nodes"]
            edges: List[Any] = doc["edges"]
            new_id = self._next_node_id(nodes)
            nodes[new_id] = self._last_pose

            doc["nodes"] = nodes
            doc["edges"] = edges

            if self._edge_build_mode.lower() == "chain_bidirectional":
                doc = apply_graph_build(doc, prefix=self._prefix, with_cost=True)
                edges = doc["edges"]
            elif self._auto_edge and self._prev_node_id is not None:
                edges.append({"from": self._prev_node_id, "to": new_id})
                if self._auto_edge_bidi:
                    edges.append({"from": new_id, "to": self._prev_node_id})
                doc["edges"] = edges
            with open(self._output_path, "w", encoding="utf-8") as f:
                yaml.dump(doc, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

            p = self._last_pose["position"]
            extra = ""
            if self._edge_build_mode.lower() == "chain_bidirectional":
                extra = f" | edges 재생성(chain_bidirectional), count={len(edges)}"
            elif self._auto_edge and self._prev_node_id is not None:
                extra = f" | edge {self._prev_node_id} -> {new_id}"
                if self._auto_edge_bidi:
                    extra += f", {new_id} -> {self._prev_node_id}"
            msg = (
                f"[그래프 노드 저장] {new_id} | "
                f"x={p['x']}, y={p['y']} | {self._output_path}{extra}"
            )
            self.get_logger().info(msg)
            print(msg, flush=True)
            self._prev_node_id = new_id


def _stdin_ready():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def _get_key():
    if _stdin_ready():
        return sys.stdin.read(1)
    return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecordGraphWaypointsNode()

    if sys.stdin.isatty():
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        old_attr = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                key = _get_key()
                if key == "k":
                    node._save_node()
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
    print("종료 (그래프 기록)", flush=True)


if __name__ == "__main__":
    main()
