"""
Dijkstra_example.py 의 setup_graph 패턴을 맵/odom 웨이포인트 그래프에 맞게 적용.

- 연속 노드(wp_1, wp_2, … 정렬 순) 사이 양방향 간선 + 좌표 기준 거리 비용(옵션)
- extra_bidirectional_pairs: 예제의 special_nodes 처럼 임의 쌍 양방향 연결
- excluded_node_ids: 해당 노드 및 인접 간선 제거 (장애물 근처 노드 배제용)
"""
from __future__ import annotations

import math
import re
from typing import Any, Dict, List, Optional, Sequence, Tuple


def euclidean_xy_nodes(nodes: Dict[str, Any], u: str, v: str) -> float:
    pu = nodes[u].get("position", {})
    pv = nodes[v].get("position", {})
    dx = float(pu.get("x", 0.0)) - float(pv.get("x", 0.0))
    dy = float(pu.get("y", 0.0)) - float(pv.get("y", 0.0))
    return math.hypot(dx, dy)


def sorted_waypoint_keys(node_ids: Sequence[str], prefix: str) -> List[str]:
    """wp_1, wp_2, … wp_10 순으로 정렬. prefix_숫자 형식 아니면 맨 뒤로."""
    esc = re.escape(prefix)
    pat = re.compile(rf"^{esc}_(\d+)$")
    scored: List[Tuple[int, str]] = []
    for k in node_ids:
        m = pat.match(k) if isinstance(k, str) else None
        if m:
            scored.append((int(m.group(1)), k))
        else:
            scored.append((10**12, str(k)))
    scored.sort(key=lambda t: (t[0], t[1]))
    return [k for _, k in scored]


def build_chain_edges_bidirectional(
    nodes: Dict[str, Any],
    *,
    ordered_ids: Optional[List[str]] = None,
    prefix: str = "wp",
    with_cost: bool = True,
) -> List[Dict[str, Any]]:
    """
    예제: for i in 1..n-1: (i)<->(i+1) 동일 가중치.
    비용 생략 시 플래너가 좌표로 다시 계산.
    """
    keys = ordered_ids if ordered_ids is not None else sorted_waypoint_keys(list(nodes.keys()), prefix)
    keys = [k for k in keys if k in nodes]
    edges: List[Dict[str, Any]] = []
    for i in range(len(keys) - 1):
        u, v = keys[i], keys[i + 1]
        for a, b in ((u, v), (v, u)):
            e: Dict[str, Any] = {"from": a, "to": b}
            if with_cost:
                e["cost"] = round(euclidean_xy_nodes(nodes, a, b), 6)
            edges.append(e)
    return edges


def edges_for_extra_pairs(
    nodes: Dict[str, Any],
    pairs: Sequence[Sequence[Any]],
    *,
    with_cost: bool = True,
) -> List[Dict[str, Any]]:
    """[[wp_1, wp_3], ...] → 각 쌍 양방향."""
    edges: List[Dict[str, Any]] = []
    for pair in pairs:
        if len(pair) != 2:
            continue
        u, v = str(pair[0]), str(pair[1])
        if u not in nodes or v not in nodes or u == v:
            continue
        for a, b in ((u, v), (v, u)):
            e: Dict[str, Any] = {"from": a, "to": b}
            if with_cost:
                e["cost"] = round(euclidean_xy_nodes(nodes, a, b), 6)
            edges.append(e)
    return edges


def remove_excluded_from_graph(
    nodes: Dict[str, Any],
    edges: List[Any],
    excluded_ids: Sequence[str],
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """노드 dict 복사본에서 제거 + 인접 간선 삭제."""
    ex = set(str(x) for x in excluded_ids)
    new_nodes = {k: v for k, v in nodes.items() if k not in ex}
    new_edges: List[Dict[str, Any]] = []
    for e in edges:
        if not isinstance(e, dict):
            continue
        u, v = e.get("from"), e.get("to")
        if u in ex or v in ex:
            continue
        new_edges.append(dict(e))
    return new_nodes, new_edges


def _get_graph_build_section(doc: Dict[str, Any]) -> Dict[str, Any]:
    gb = doc.get("graph_build")
    return gb if isinstance(gb, dict) else {}


def apply_graph_build(
    doc: Dict[str, Any],
    *,
    prefix: str = "wp",
    with_cost: bool = True,
) -> Dict[str, Any]:
    """
    doc: nodes, (optional) graph_build, line_goal_links, …
    edges 를 예제 스타일로 재생성. graph_build 가 없어도 chain 만 적용 가능.
    """
    out = dict(doc)
    nodes: Dict[str, Any] = dict(out.get("nodes") or {})
    gb = _get_graph_build_section(out)

    excluded = gb.get("excluded_node_ids") or gb.get("excluded_nodes") or []
    if excluded:
        # 먼저 노드/간선에서 제외 (기존 edges 도 정리)
        old_edges: List[Any] = list(out.get("edges") or [])
        nodes, _ = remove_excluded_from_graph(nodes, old_edges, excluded)

    extra = gb.get("extra_bidirectional_pairs") or gb.get("special_pairs") or []

    use_prefix = str(gb.get("node_id_prefix") or prefix)
    only_ids = gb.get("chain_node_order")
    if only_ids is not None and isinstance(only_ids, list):
        ordered = [str(x) for x in only_ids if str(x) in nodes]
    else:
        ordered = None

    edges = build_chain_edges_bidirectional(
        nodes, ordered_ids=ordered, prefix=use_prefix, with_cost=with_cost
    )
    edges.extend(edges_for_extra_pairs(nodes, extra, with_cost=with_cost))

    out["nodes"] = nodes
    out["edges"] = edges
    if "graph_build" not in out:
        out["graph_build"] = {}
    elif not isinstance(out["graph_build"], dict):
        out["graph_build"] = {}
    return out
