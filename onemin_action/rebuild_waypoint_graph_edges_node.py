#!/usr/bin/env python3
"""
waypoint_graph.yaml 을 읽어 nodes + graph_build 규칙으로 edges 를 재생성 (Dijkstra_example setup_graph 와 동일 패턴).

  ros2 run onemin_action rebuild_waypoint_graph_edges
  ros2 run onemin_action rebuild_waypoint_graph_edges --ros-args \
    -p waypoint_graph_file:=my_graph.yaml -p with_edge_cost:=false -p autofill_empty_docking_links:=false

  edges 갱신 후 line_goal_links / docking_goal_links 가 없으면 {} 로 넣고,
  autofill_empty_docking_links 기본 true 일 때 비어 있는 도킹 키는 체인 마지막 wp_* 로 채움.
  YAML 맨 아래에 링크 필드 설명 주석을 한 번만 덧붙임(마커로 중복 방지).
"""
import os
import sys

import rclpy
import yaml
from rclpy.node import Node

from onemin_action.line_pose_loader import get_data_dir
from onemin_action.waypoint_graph_builder import (
    GRAPH_LINK_FIELD_GUIDE,
    GRAPH_LINK_FIELD_GUIDE_MARKER,
    apply_graph_build,
    autofill_empty_docking_goal_links,
)


def _append_graph_link_guide(path: str) -> None:
    with open(path, "r", encoding="utf-8") as f:
        body = f.read()
    if GRAPH_LINK_FIELD_GUIDE_MARKER in body:
        return
    with open(path, "a", encoding="utf-8") as f:
        f.write("\n" + GRAPH_LINK_FIELD_GUIDE_MARKER + GRAPH_LINK_FIELD_GUIDE)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("rebuild_waypoint_graph_edges")
    node.declare_parameter("waypoint_graph_file", "waypoint_graph.yaml")
    node.declare_parameter("with_edge_cost", True)
    node.declare_parameter("node_id_prefix", "wp")
    node.declare_parameter(
        "autofill_empty_docking_links",
        True,
    )

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

    _af = node.get_parameter("autofill_empty_docking_links").value
    autofill_on = _af if isinstance(_af, bool) else str(_af).lower() in ("1", "true", "yes")
    if autofill_on:
        for line in autofill_empty_docking_goal_links(new_doc, prefix=prefix):
            node.get_logger().info(line)
            print(line, flush=True)

    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(new_doc, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    _append_graph_link_guide(path)

    n_e = len(new_doc.get("edges") or [])
    node.get_logger().info(f"갱신 완료: {path} | edges={n_e}")
    print(f"edges 재생성 → {path} ({n_e} edges)", flush=True)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
