#!/usr/bin/env python3
"""
waypoint_graph.yaml 을 읽어 nodes + graph_build 규칙으로 edges 를 재생성 (Dijkstra_example setup_graph 와 동일 패턴).

  ros2 run onemin_action rebuild_waypoint_graph_edges
  ros2 run onemin_action rebuild_waypoint_graph_edges --ros-args \
    -p waypoint_graph_file:=my_graph.yaml -p with_edge_cost:=false
"""
import os
import sys

import rclpy
import yaml
from rclpy.node import Node

from onemin_action.line_pose_loader import get_data_dir
from onemin_action.waypoint_graph_builder import apply_graph_build


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("rebuild_waypoint_graph_edges")
    node.declare_parameter("waypoint_graph_file", "waypoint_graph.yaml")
    node.declare_parameter("with_edge_cost", True)
    node.declare_parameter("node_id_prefix", "wp")

    basename = node.get_parameter("waypoint_graph_file").value
    _w = node.get_parameter("with_edge_cost").value
    with_cost = _w if isinstance(_w, bool) else str(_w).lower() in ("1", "true", "yes")
    prefix = str(node.get_parameter("node_id_prefix").value or "wp")

    data_dir = get_data_dir()
    os.makedirs(data_dir, exist_ok=True)
    path = os.path.join(data_dir, basename)

    if not os.path.isfile(path):
        node.get_logger().error(f"파일 없음: {path}")
        rclpy.shutdown()
        sys.exit(1)

    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}

    if not doc.get("nodes"):
        node.get_logger().error("nodes 가 비어 있음")
        rclpy.shutdown()
        sys.exit(1)

    new_doc = apply_graph_build(doc, prefix=prefix, with_cost=with_cost)
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(new_doc, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    n_e = len(new_doc.get("edges") or [])
    node.get_logger().info(f"갱신 완료: {path} | edges={n_e}")
    print(f"edges 재생성 → {path} ({n_e} edges)", flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
