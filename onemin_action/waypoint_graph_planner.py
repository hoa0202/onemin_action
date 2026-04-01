"""
waypoint_graph.yaml 로드 → (선택) line_positions.yaml 목표와 연결 → 다익스트라로 포즈 시퀀스 생성.
간선은 YAML에 정의된 방향 그래프; line_goal_links로 그래프 노드에서 라인 목표까지 마지막 간선을 연결.

참고 (저장소 루트 `Dijkstra_example.py` 와의 대응):
- 예제의 ``distance()`` / Nó 좌표 간 거리 → 여기서는 맵·odom 평면이므로 간선 가중치에
  ``_euclidean_xy`` 사용 (예제가 LL이면 하버사인, 동일한 ‘코털 간 거리=비용’ 개념).
- 예제 ``YamlWaypointParser`` / YAML에서 점 나열 → ``nodes`` + ``edges`` 로 명시적 그래프
  (최단경로는 예제처럼 나열이 아니라 다익스트라로 순서 결정).
- 예제 ``NavigateThroughPoses`` 로 포즈를 순차 주행 → 실행은 ``scenario_controller_node`` 가
  ``NavigateToPose`` 를 경로 순서대로 호출 (동일하게 ‘계획된 포인트를 순서대로 이음’).
- 그래프 **경유** pose 의 yaw는 YAML recorded orientation 을 쓰지 않고, 경로상 **다음 점을 향한 접선**
  으로 둬서 역주행(4→3→2…) 시에도 노드마다 180° 정렬이 나지 않게 함.
"""
from __future__ import annotations

import heapq
import math
import os
import re
from typing import Any, Dict, List, Optional, Tuple

import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped


def _euclidean_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _graph_node_pose_tangent_to_next(
    entry: Dict[str, Any],
    stamp: Optional[Time],
    tx: float,
    ty: float,
    *,
    eps: float = 1e-6,
) -> PoseStamped:
    """
    노드 entry 의 위치 + (현재 노드 xy → (tx,ty)) 방향 yaw.
    경로가 역방향이어도 ‘가야 할 쪽’을 바라보도록 해 recorded orientation 과 무관.
    """
    p = _pose_from_node_entry(entry, stamp=stamp)
    sx = p.pose.position.x
    sy = p.pose.position.y
    dx = tx - sx
    dy = ty - sy
    if dx * dx + dy * dy < eps * eps:
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        return p
    yaw = math.atan2(dy, dx)
    qx, qy, qz, qw = _quaternion_from_yaw(yaw)
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


def _pose_from_node_entry(entry: Dict[str, Any], stamp: Optional[Time] = None) -> PoseStamped:
    frame_id = entry.get("frame_id", "map")
    pos = entry.get("position", {})
    ori = entry.get("orientation", {})
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    pose.pose.position.x = float(pos.get("x", 0.0))
    pose.pose.position.y = float(pos.get("y", 0.0))
    pose.pose.position.z = float(pos.get("z", 0.0))
    pose.pose.orientation.x = float(ori.get("x", 0.0))
    pose.pose.orientation.y = float(ori.get("y", 0.0))
    pose.pose.orientation.z = float(ori.get("z", 0.0))
    pose.pose.orientation.w = float(ori.get("w", 1.0))
    return pose


def _load_graph_yaml(path: str) -> Dict[str, Any]:
    if not os.path.isfile(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _build_adjacency_with_synthetic_goal(
    nodes: Dict[str, Any],
    edges: List[Any],
    goal_pose: PoseStamped,
    goal_id: str,
    link_nodes: Optional[Any],
) -> Tuple[Dict[str, List[Tuple[str, float]]], str]:
    """간선 + (link_nodes → synthetic goal_id) 유클리드 마지막 구간."""
    adj: Dict[str, List[Tuple[str, float]]] = {nid: [] for nid in nodes}

    for e in edges:
        if not isinstance(e, dict):
            continue
        u = e.get("from")
        v = e.get("to")
        if u not in nodes or v not in nodes:
            continue
        cost = e.get("cost")
        if cost is None:
            pu = nodes[u].get("position", {})
            pv = nodes[v].get("position", {})
            cost = _euclidean_xy(
                float(pu.get("x", 0.0)),
                float(pu.get("y", 0.0)),
                float(pv.get("x", 0.0)),
                float(pv.get("y", 0.0)),
            )
        adj[u].append((v, float(cost)))

    adj[goal_id] = []

    if not link_nodes:
        return adj, goal_id

    if isinstance(link_nodes, str):
        link_nodes = [link_nodes]
    gx = goal_pose.pose.position.x
    gy = goal_pose.pose.position.y

    for nid in link_nodes:
        if nid not in nodes:
            continue
        pu = nodes[nid].get("position", {})
        dist = _euclidean_xy(float(pu.get("x", 0.0)), float(pu.get("y", 0.0)), gx, gy)
        adj[nid].append((goal_id, dist))

    return adj, goal_id


def _build_adjacency(
    nodes: Dict[str, Any],
    edges: List[Any],
    line_number: int,
    goal_pose: PoseStamped,
    line_goal_links: Dict[str, Any],
) -> Tuple[Dict[str, List[Tuple[str, float]]], str]:
    goal_id = f"__line_{line_number}__"
    key = str(line_number)
    link_nodes = line_goal_links.get(key)
    return _build_adjacency_with_synthetic_goal(
        nodes, edges, goal_pose, goal_id, link_nodes
    )


def _safe_named_goal_id(goal_key: str) -> str:
    safe = "".join(c if c.isalnum() else "_" for c in goal_key)
    return f"__named_goal_{safe}__"


def _distances_to_goal(
    adj: Dict[str, List[Tuple[str, float]]], goal_id: str
) -> Dict[str, float]:
    """역방향 그래프에서 goal에서 출발하는 다익스트라 → 원래 그래프에서 각 정점→goal 최단거리."""
    radj: Dict[str, List[Tuple[str, float]]] = {n: [] for n in adj}
    for u, nbs in adj.items():
        for v, w in nbs:
            radj[v].append((u, w))
    inf = float("inf")
    dist: Dict[str, float] = {n: inf for n in adj}
    dist[goal_id] = 0.0
    pq: List[Tuple[float, str]] = [(0.0, goal_id)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, inf):
            continue
        for v, w in radj.get(u, []):
            nd = d + w
            if nd < dist.get(v, inf):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    return dist


def _node_xy(nodes: Dict[str, Any], nid: str) -> Tuple[float, float]:
    pu = nodes[nid].get("position", {})
    return float(pu.get("x", 0.0)), float(pu.get("y", 0.0))


def _goal_xy(line_pose: PoseStamped) -> Tuple[float, float]:
    return line_pose.pose.position.x, line_pose.pose.position.y


def _dot_from_a_to_b_toward_c(ax: float, ay: float, bx: float, by: float, cx: float, cy: float) -> float:
    """벡터 (a→b) · (a→c). b가 c의 ‘뒤’쪽이면 음수."""
    return (bx - ax) * (cx - ax) + (by - ay) * (cy - ay)


def _xy_on_path(
    node_path: List[str],
    idx: int,
    nodes: Dict[str, Any],
    goal_id: str,
    goal_pose: PoseStamped,
) -> Tuple[float, float]:
    if idx >= len(node_path):
        return _goal_xy(goal_pose)
    nid = node_path[idx]
    if nid == goal_id:
        return _goal_xy(goal_pose)
    return _node_xy(nodes, nid)


def _path_forward_along_path(
    node_path: List[str],
    nodes: Dict[str, Any],
    goal_id: str,
    goal_pose: PoseStamped,
    cx: float,
    cy: float,
    dot_min: float,
) -> bool:
    """
    경로 상에서 다음 구간과의 접선이 뒷걸음이 아닌지 검사.
    예전 (간선)·(현재→최종목표) 검사는 도킹처럼 ‘목표가 그래프 반대편’일 때
    긴 체인 경로가 깨질 수 있음. 또 dock 이 마지막 WP와 직선으로 닿는데 체인 접선과 다르면
    (wp_2→wp_1 대 wp_1→dock) 마지막 그래프 간선만 검사 생략.
    """
    if len(node_path) < 2:
        return False

    def xy(i: int) -> Tuple[float, float]:
        return _xy_on_path(node_path, i, nodes, goal_id, goal_pose)

    fx, fy = xy(0)
    if _euclidean_xy(cx, cy, fx, fy) > 1e-6:
        sx, sy = xy(1)
        if _dot_from_a_to_b_toward_c(cx, cy, fx, fy, sx, sy) < dot_min:
            return False

    for i in range(len(node_path) - 1):
        if node_path[i + 1] == goal_id:
            continue
        # 마지막 그래프 노드 직전 간선: 이후 목표(dock/라인)로 급전환하는 경우가 많아
        # (wp_1→dock 이 체인 방향과 거의 반대일 수 있음). 그래프 '역행' 검사 제외.
        if i + 2 < len(node_path) and node_path[i + 2] == goal_id:
            continue
        ax, ay = xy(i)
        bx, by = xy(i + 1)
        tx, ty = xy(i + 2)
        if _euclidean_xy(ax, ay, bx, by) < 1e-9:
            continue
        if _dot_from_a_to_b_toward_c(ax, ay, bx, by, tx, ty) < dot_min:
            return False
    return True


def _sorted_start_candidates(
    nodes: Dict[str, Any],
    dist_to_goal: Dict[str, float],
    cx: float,
    cy: float,
    policy: str,
) -> List[str]:
    """목표까지 도달 가능한 노드를 후보 순서로 정렬 (가까운 것부터 시도)."""
    inf = float("inf")
    reachable = [u for u in nodes if u in dist_to_goal and dist_to_goal[u] < inf]
    pol = policy.strip().lower()

    def key_nearest(u: str) -> Tuple[float, str]:
        ux, uy = _node_xy(nodes, u)
        return (_euclidean_xy(cx, cy, ux, uy), u)

    def key_min_total(u: str) -> Tuple[float, str]:
        ux, uy = _node_xy(nodes, u)
        d = _euclidean_xy(cx, cy, ux, uy) + dist_to_goal[u]
        return (d, u)

    if pol == "min_total":
        return sorted(reachable, key=key_min_total)
    return sorted(reachable, key=key_nearest)


def _dijkstra(
    adj: Dict[str, List[Tuple[str, float]]], start: str, goal: str
) -> Optional[List[str]]:
    inf = float("inf")
    dist: Dict[str, float] = {n: inf for n in adj}
    prev: Dict[str, Optional[str]] = {n: None for n in adj}
    if start not in adj or goal not in adj:
        return None
    dist[start] = 0.0
    pq: List[Tuple[float, str]] = [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == goal:
            break
        if d > dist.get(u, inf):
            continue
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, inf):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if dist.get(goal, inf) == inf:
        return None
    path: List[str] = []
    cur: Optional[str] = goal
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path


def plan_path_to_line(
    data_dir: str,
    graph_basename: str,
    line_number: int,
    start_node_id: str,
    line_pose: PoseStamped,
    stamp: Optional[Time] = None,
    current_pose: Optional[PoseStamped] = None,
    auto_start_policy: str = "nearest",
    path_dot_min: float = 0.0,
) -> Optional[Tuple[List[PoseStamped], str]]:
    """
    line_pose: line_positions에서 읽은 라인 N 목표 (마지막 웨이포인트에 사용).

    start_node_id 비움 + current_pose: 가까운 노드 순으로 후보 시도 → 다익스트라 경로가
    그래프 순서상 ‘다음 구간’과 역행(dot_min)하지 않을 때만 채택.
    start_node_id 지정: 해당 노드만 (current_pose 있으면 같은 검사 적용).
    """
    path = os.path.join(data_dir, graph_basename)
    raw = _load_graph_yaml(path)
    nodes = raw.get("nodes") or {}
    edges = raw.get("edges") or []
    lgl = raw.get("line_goal_links") or {}
    line_goal_links = {str(k): v for k, v in lgl.items()}

    if not nodes:
        return None

    adj, goal_id = _build_adjacency(nodes, edges, line_number, line_pose, line_goal_links)
    dist_to_goal = _distances_to_goal(adj, goal_id)

    fixed = (start_node_id or "").strip()
    cx: Optional[float] = None
    cy: Optional[float] = None
    if current_pose is not None:
        cx = current_pose.pose.position.x
        cy = current_pose.pose.position.y

    if fixed:
        if fixed not in nodes:
            return None
        candidates = [fixed]
    else:
        if cx is None or cy is None:
            return None
        candidates = _sorted_start_candidates(
            nodes, dist_to_goal, cx, cy, auto_start_policy
        )
        if not candidates:
            return None

    node_path: Optional[List[str]] = None
    start: Optional[str] = None
    for u in candidates:
        p = _dijkstra(adj, u, goal_id)
        if not p:
            continue
        if cx is not None and cy is not None:
            if not _path_forward_along_path(p, nodes, goal_id, line_pose, cx, cy, path_dot_min):
                continue
        node_path = p
        start = u
        break

    if not node_path or start is None:
        return None

    out: List[PoseStamped] = []
    for i, nid in enumerate(node_path):
        if nid == goal_id:
            final = PoseStamped()
            final.header.frame_id = line_pose.header.frame_id
            final.header.stamp = stamp if stamp is not None else line_pose.header.stamp
            final.pose = line_pose.pose
            out.append(final)
        else:
            nnext = node_path[i + 1]
            if nnext == goal_id:
                tx = float(line_pose.pose.position.x)
                ty = float(line_pose.pose.position.y)
            else:
                pn = nodes[nnext].get("position", {})
                tx = float(pn.get("x", 0.0))
                ty = float(pn.get("y", 0.0))
            out.append(
                _graph_node_pose_tangent_to_next(nodes[nid], stamp, tx, ty)
            )
    return out, start


def highest_numbered_wp_node_id(nodes: Dict[str, Any]) -> Optional[str]:
    """노드 키 ``wp_<정수>`` 중 번호가 가장 큰 id (warehouse 홈 등)."""
    best_n = -1
    best_id: Optional[str] = None
    pat = re.compile(r"^wp_(\d+)$")
    for nid in nodes:
        m = pat.match(str(nid))
        if not m:
            continue
        n = int(m.group(1))
        if n > best_n:
            best_n = n
            best_id = str(nid)
    return best_id


def _incoming_link_nodes_for_target(
    edges: List[Any], nodes: Dict[str, Any], target_id: str
) -> List[str]:
    """``to == target_id`` 인 간선의 ``from`` 목록. 없으면 호출부에서 ``[target_id]`` 로 폴백."""
    seen: List[str] = []
    found = set()
    for e in edges:
        if not isinstance(e, dict):
            continue
        if e.get("to") != target_id:
            continue
        u = e.get("from")
        if u in nodes and u not in found:
            found.add(str(u))
            seen.append(str(u))
    return seen


def get_warehouse_home_pose(
    data_dir: str,
    graph_basename: str,
    stamp: Optional[Time] = None,
) -> Optional[PoseStamped]:
    """가장 큰 번호의 ``wp_*`` 노드 포즈 (그래프 미경유 단일 목표용)."""
    path = os.path.join(data_dir, graph_basename)
    raw = _load_graph_yaml(path)
    nodes = raw.get("nodes") or {}
    home_id = highest_numbered_wp_node_id(nodes)
    if not home_id or home_id not in nodes:
        return None
    return _pose_from_node_entry(nodes[home_id], stamp=stamp)


def plan_path_to_warehouse_home(
    data_dir: str,
    graph_basename: str,
    start_node_id: str,
    stamp: Optional[Time] = None,
    current_pose: Optional[PoseStamped] = None,
    auto_start_policy: str = "nearest",
    path_dot_min: float = 0.0,
) -> Optional[Tuple[List[PoseStamped], str]]:
    """
    ``wp_*`` 중 최대 번호 노드를 목표(홈/warehouse)로 하는 그래프 경로.
    합성 목표 간선은 ``line_goal_links``/도킹과 동일하게 ``link_nodes → goal`` 유클리드 연결.
    """
    path = os.path.join(data_dir, graph_basename)
    raw = _load_graph_yaml(path)
    nodes = raw.get("nodes") or {}
    edges = raw.get("edges") or []

    if not nodes:
        return None

    home_id = highest_numbered_wp_node_id(nodes)
    if not home_id or home_id not in nodes:
        return None

    goal_pose = _pose_from_node_entry(nodes[home_id], stamp=stamp)
    link_nodes = _incoming_link_nodes_for_target(edges, nodes, home_id)
    if not link_nodes:
        link_nodes = [home_id]

    goal_id = "__warehouse_home__"
    adj, goal_id = _build_adjacency_with_synthetic_goal(
        nodes, edges, goal_pose, goal_id, link_nodes
    )
    dist_to_goal = _distances_to_goal(adj, goal_id)

    fixed = (start_node_id or "").strip()
    cx: Optional[float] = None
    cy: Optional[float] = None
    if current_pose is not None:
        cx = current_pose.pose.position.x
        cy = current_pose.pose.position.y

    if fixed:
        if fixed not in nodes:
            return None
        candidates = [fixed]
    else:
        if cx is None or cy is None:
            return None
        candidates = _sorted_start_candidates(
            nodes, dist_to_goal, cx, cy, auto_start_policy
        )
        if not candidates:
            return None

    node_path: Optional[List[str]] = None
    start: Optional[str] = None
    for u in candidates:
        p = _dijkstra(adj, u, goal_id)
        if not p:
            continue
        if cx is not None and cy is not None:
            if not _path_forward_along_path(
                p, nodes, goal_id, goal_pose, cx, cy, path_dot_min
            ):
                continue
        node_path = p
        start = u
        break

    if not node_path or start is None:
        return None

    out: List[PoseStamped] = []
    for i, nid in enumerate(node_path):
        if nid == goal_id:
            final = PoseStamped()
            final.header.frame_id = goal_pose.header.frame_id
            final.header.stamp = stamp if stamp is not None else goal_pose.header.stamp
            final.pose = goal_pose.pose
            out.append(final)
        else:
            nnext = node_path[i + 1]
            if nnext == goal_id:
                tx = float(goal_pose.pose.position.x)
                ty = float(goal_pose.pose.position.y)
            else:
                pn = nodes[nnext].get("position", {})
                tx = float(pn.get("x", 0.0))
                ty = float(pn.get("y", 0.0))
            out.append(
                _graph_node_pose_tangent_to_next(nodes[nid], stamp, tx, ty)
            )
    return out, start


def plan_path_to_named_goal(
    data_dir: str,
    graph_basename: str,
    goal_key: str,
    start_node_id: str,
    goal_pose: PoseStamped,
    stamp: Optional[Time] = None,
    current_pose: Optional[PoseStamped] = None,
    auto_start_policy: str = "nearest",
    path_dot_min: float = 0.0,
) -> Optional[Tuple[List[PoseStamped], str]]:
    """
    waypoint_graph.yaml 의 docking_goal_links[goal_key] 로 그래프와 도킹/복귀 목표 포즈를 연결.
    goal_key 예: move_to_docking_station, move_to_return (토픽 String.data 와 동일).
    """
    path = os.path.join(data_dir, graph_basename)
    raw = _load_graph_yaml(path)
    nodes = raw.get("nodes") or {}
    edges = raw.get("edges") or []
    dgl_raw = raw.get("docking_goal_links") or {}
    docking_goal_links = {str(k): v for k, v in dgl_raw.items()}

    if not nodes or not goal_key:
        return None

    gk = goal_key.strip()
    link_nodes = docking_goal_links.get(gk)
    if not link_nodes:
        return None

    goal_id = _safe_named_goal_id(gk)
    adj, goal_id = _build_adjacency_with_synthetic_goal(
        nodes, edges, goal_pose, goal_id, link_nodes
    )
    dist_to_goal = _distances_to_goal(adj, goal_id)

    fixed = (start_node_id or "").strip()
    cx: Optional[float] = None
    cy: Optional[float] = None
    if current_pose is not None:
        cx = current_pose.pose.position.x
        cy = current_pose.pose.position.y

    if fixed:
        if fixed not in nodes:
            return None
        candidates = [fixed]
    else:
        if cx is None or cy is None:
            return None
        candidates = _sorted_start_candidates(
            nodes, dist_to_goal, cx, cy, auto_start_policy
        )
        if not candidates:
            return None

    node_path: Optional[List[str]] = None
    start: Optional[str] = None
    for u in candidates:
        p = _dijkstra(adj, u, goal_id)
        if not p:
            continue
        if cx is not None and cy is not None:
            if not _path_forward_along_path(p, nodes, goal_id, goal_pose, cx, cy, path_dot_min):
                continue
        node_path = p
        start = u
        break

    if not node_path or start is None:
        return None

    out: List[PoseStamped] = []
    for i, nid in enumerate(node_path):
        if nid == goal_id:
            final = PoseStamped()
            final.header.frame_id = goal_pose.header.frame_id
            final.header.stamp = stamp if stamp is not None else goal_pose.header.stamp
            final.pose = goal_pose.pose
            out.append(final)
        else:
            nnext = node_path[i + 1]
            if nnext == goal_id:
                tx = float(goal_pose.pose.position.x)
                ty = float(goal_pose.pose.position.y)
            else:
                pn = nodes[nnext].get("position", {})
                tx = float(pn.get("x", 0.0))
                ty = float(pn.get("y", 0.0))
            out.append(
                _graph_node_pose_tangent_to_next(nodes[nid], stamp, tx, ty)
            )
    return out, start
