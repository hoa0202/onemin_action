"""
data/line_positions.yaml 경로(소스 패키지 기준) 및 line_N → PoseStamped 변환.
코드1과 동일 경로 규칙 사용.
"""
import os
from typing import Optional

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
