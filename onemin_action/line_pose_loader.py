"""
data/line_positions.yaml 경로(소스 패키지 기준) 및 line_N → PoseStamped 변환.
코드1과 동일 경로 규칙 사용.
"""
import os
import re
from typing import Optional, Tuple

import yaml
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


def get_data_dir() -> str:
    """소스 패키지의 data 디렉터리 (colcon 시 workspace/src/<패키지명>/data)."""
    path = os.path.abspath(__file__)
    pkg_dir = os.path.dirname(os.path.dirname(path))
    parent = os.path.dirname(pkg_dir)
    if os.path.basename(parent) in ("build", "install"):
        workspace = os.path.dirname(parent)
        src_data = os.path.join(workspace, "src", "onemin_action", "data")
        if os.path.isdir(os.path.join(workspace, "src", "onemin_action")):
            return src_data
    return os.path.join(pkg_dir, "data")


_LINE_KEY_RE = re.compile(r"^line_(\d+)$")


def warehouse_pose_and_graph_link_key(
    data_dir: str, stamp: Optional[Time] = None
) -> Optional[Tuple[PoseStamped, str]]:
    """
    warehouse 이동 목표 포즈와 ``waypoint_graph.yaml`` 의 ``line_goal_links`` 에 쓸 키.

    우선순위:
    1) ``line_positions.yaml`` 최상위 키 ``warehouse`` (line_N 과 동일 스키마) → 링크 키 ``\"warehouse\"``
    2) 없으면 최대 번호 ``line_N`` pose → 링크 키 ``\"N\"`` (문자열)

    그래프 모드에서는 ``line_goal_links`` 에 해당 키가 있어야 함 (explicit warehouse 면 ``warehouse: [wp_…]``).
    """
    wpose = load_pose_by_key(data_dir, "line_positions.yaml", "warehouse", stamp)
    if wpose is not None:
        return wpose, "warehouse"
    n_last = max_line_number_in_positions(data_dir)
    if n_last is None:
        return None
    pose = load_line_pose(data_dir, n_last, stamp=stamp)
    if pose is None:
        return None
    return pose, str(n_last)


def max_line_number_in_positions(data_dir: str) -> Optional[int]:
    """
    line_positions.yaml 최상위 키 ``line_N`` 중 최대 N. 파일 없거나 키 없으면 None.
    warehouse 폴백(명시 키 없을 때)에 사용.
    """
    path = os.path.join(data_dir, "line_positions.yaml")
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    best = -1
    for k in data:
        m = _LINE_KEY_RE.match(str(k))
        if m:
            n = int(m.group(1))
            if n > best:
                best = n
    return best if best >= 1 else None


def load_line_pose(data_dir: str, line_number: int, stamp: Optional[Time] = None) -> Optional[PoseStamped]:
    """
    data/line_positions.yaml에서 line_N 좌표를 읽어 PoseStamped로 반환.
    line_number는 1 이상 (0 미사용). 없으면 None.
    """
    if line_number < 1:
        return None
    path = os.path.join(data_dir, "line_positions.yaml")
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    key = f"line_{line_number}"
    if key not in data:
        return None
    entry = data[key]
    frame_id = entry.get("frame_id", "map")
    pos = entry.get("position", {})
    ori = entry.get("orientation", {})
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp:
        pose.header.stamp = stamp
    pose.pose.position.x = float(pos.get("x", 0))
    pose.pose.position.y = float(pos.get("y", 0))
    pose.pose.position.z = float(pos.get("z", 0))
    pose.pose.orientation.x = float(ori.get("x", 0))
    pose.pose.orientation.y = float(ori.get("y", 0))
    pose.pose.orientation.z = float(ori.get("z", 0))
    pose.pose.orientation.w = float(ori.get("w", 1))
    return pose


def load_pose_by_key(
    data_dir: str,
    yaml_basename: str,
    key: str,
    stamp: Optional[Time] = None,
) -> Optional[PoseStamped]:
    """
    data/<yaml_basename> 에서 최상위 키 key의 pose 엔트리 로드 (line_N 과 동일 스키마).
    도킹: docking_positions.yaml + 키 move_to_docking_station, move_to_return 등.
    """
    if not key:
        return None
    path = os.path.join(data_dir, yaml_basename)
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if key not in data:
        return None
    entry = data[key]
    frame_id = entry.get("frame_id", "map")
    pos = entry.get("position", {})
    ori = entry.get("orientation", {})
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp:
        pose.header.stamp = stamp
    pose.pose.position.x = float(pos.get("x", 0))
    pose.pose.position.y = float(pos.get("y", 0))
    pose.pose.position.z = float(pos.get("z", 0))
    pose.pose.orientation.x = float(ori.get("x", 0))
    pose.pose.orientation.y = float(ori.get("y", 0))
    pose.pose.orientation.z = float(ori.get("z", 0))
    pose.pose.orientation.w = float(ori.get("w", 1))
    return pose
