# onemin_action

라인 입구 출발 위치 기록 및 시나리오 제어용 ROS2 패키지.

## 코드1: 라인 위치 기록 (record_line_positions_node)

로봇을 수동으로 각 라인 입구 출발 지점에 두고, 현재 위치·방향을 YAML에 1번부터 저장한다.  
저장 형식은 Nav2 Goal과 동일한 `geometry_msgs/PoseStamped` (position + orientation quaternion).

### 실행

```bash
ros2 run onemin_action record_line_positions_node
```

- **k** : 현재 pose를 다음 번호(1, 2, 3, …)로 저장
- **s** 또는 **Ctrl+C** : 종료

### 저장 위치

- `data/line_positions.yaml` (이 저장소·패키지 루트 기준; colcon 워크스페이스에 둘 때는 `src/onemin_action/data/line_positions.yaml`)

### 파라미터

| 파라미터      | 기본값                     | 설명                    |
|---------------|----------------------------|-------------------------|
| `pose_topic`  | `/liorf/mapping/odometry`  | Pose 구독 토픽          |
| `pose_type`   | `odom`                     | `odom` / `amcl` / `pose_stamped` |

### 토픽

- 구독: `pose_topic` (Odometry / PoseWithCovarianceStamped / PoseStamped)
- 구독: `/record_line_position` (`std_msgs/Empty`) — 한 번 publish할 때마다 현재 위치 저장 (터미널 없이 사용 시)

### YAML 형식 예시

```yaml
line_1:
  frame_id: "odom"
  position: { x: -0.21, y: -0.02, z: 0.0 }
  orientation: { x: 0.0, y: 0.02, z: -0.005, w: -0.999 }
line_2:
  ...
```

---

## 코드3: 수동 액션 명령 (manual_action_command_node)

사람이 육안으로 확인한 뒤 `/action`, `/action_check`, **라인/홈(`line_move_topic`)**, **도킹(`docking_topic`)** 으로 수동 명령을 보낸다.

### 실행

**대화형** (터미널에서 키 입력; TTY일 때만 raw 키 모드):

```bash
ros2 run onemin_action manual_action_command_node
```

- **1~9** : `line_move_topic`에 해당 라인 번호 문자열 (`"1"` … `"9"`)
- **0** : 라인 **10** (`line_number_max` ≥ 10일 때만)
- **w** : `warehouse` 발행 → 시나리오에서 그래프 **최대 번호 `wp_*` 노드**(홈)로 이동
- **n** : `/action` → `entering_next`
- **e** : `/action` → `entering_end`
- **c** : `/action_check` → `entering_check`
- **f** : `/action_check` → `goal_finish`
- **r** : `/action_check` → `goal_return_finish`
- **d** : 도킹 스테이션 (`move_to_docking_station`)
- **b** : 도킹 복그 (`move_to_return`)
- **q** 또는 **Ctrl+C** : 노드 종료

라인 번호가 **10 초과**이면 TTY 한 글자로는 부족하므로, **표준 입력 한 줄 모드**(stdin이 TTY가 아닐 때)나 **한 번만 전송** 모드에서 `11` 같은 문자열을 넘긴다.

**한 번만 전송 후 종료**:

```bash
ros2 run onemin_action manual_action_command_node 1
ros2 run onemin_action manual_action_command_node warehouse
ros2 run onemin_action manual_action_command_node entering_next
ros2 run onemin_action manual_action_command_node entering_end
ros2 run onemin_action manual_action_command_node entering_check
ros2 run onemin_action manual_action_command_node goal_finish
ros2 run onemin_action manual_action_command_node goal_return_finish
ros2 run onemin_action manual_action_command_node move_to_docking_station
ros2 run onemin_action manual_action_command_node move_to_return
```

**파라미터 예** (시나리오와 동일하게 맞추기):

```bash
ros2 run onemin_action manual_action_command_node --ros-args \
  -p line_move_topic:=/harv_robot/line_move \
  -p line_number_max:=10 \
  -p docking_topic:=/harv_robot/docking_move
```

| 파라미터           | 기본값                      | 설명 |
|--------------------|-----------------------------|------|
| `line_move_topic`  | `/harv_robot/line_move`     | 라인·홈 명령 발행 (`std_msgs/String`) |
| `line_number_max`  | `10`                        | 허용 라인 번호 상한 (`scenario_controller_node` 와 동일하게 두는 것을 권장) |
| `docking_topic`    | `/harv_robot/docking_move`  | 도킹 명령 발행 (`std_msgs/String`) |

### 토픽

- 발행: `line_move_topic` (`std_msgs/String`) — `"1"` … `"N"` (`line_number_max` 이하), 또는 **`warehouse`**
- 발행: `/action` (`std_msgs/String`) — `entering_next`, `entering_end`
- 발행: `/action_check` (`std_msgs/String`) — `entering_check`, `goal_finish`, `goal_return_finish`
- 발행: `docking_topic` (`std_msgs/String`) — `move_to_docking_station`, `move_to_return`

---

## 코드2: 시나리오 제어 (scenario_controller_node)

라인 번호(**String**) 또는 **warehouse** 수신 → YAML·웨이포인트 그래프로 Nav2 이동 → (라인일 때만) `entering_start` 발행 → `entering_check` 확인 → `goal_finish` 로그 → `goal_return_finish` 대기 후 다시 입력 대기.  
**warehouse** 로 홈에 도착하면 **수확 시나리오 없이** IDLE로 돌아간다.

### 실행

```bash
ros2 launch onemin_action scenario_control.launch.py
```

라인/홈 입력은 **다른 터미널**에서:

```bash
ros2 topic pub --once /harv_robot/line_move std_msgs/String "{data: '1'}"
ros2 topic pub --once /harv_robot/line_move std_msgs/String "{data: 'warehouse'}"
```

### 파라미터 (일부)

| 파라미터                       | 기본값               | 설명 |
|--------------------------------|----------------------|------|
| `nav2_action_name`             | `navigate_to_pose`   | Nav2 NavigateToPose 액션 이름 |
| `entering_check_timeout_sec`   | `0.5`                | entering_check 대기 시간(초) |
| `entering_check_max_retries`  | `10`                 | entering_check 미수신 시 entering_start 최대 재전송 횟수 |
| `line_command_topic`           | `/harv_robot/line_move` | 라인·홈 구독 (`std_msgs/String`) |
| `line_number_max`              | `10`                 | 허용 라인 번호 상한 (`"11"` 등은 거부) |
| `use_waypoint_graph`           | `true`               | `waypoint_graph.yaml` 경유 |
| `waypoint_graph_file`          | `waypoint_graph.yaml` | 데이터 디렉터리 기준 그래프 파일 |
| `docking_topic`                | `/harv_robot/docking_move` | 도킹 명령 (`std_msgs/String`) |

`scenario_control.launch.py` 에서 `line_command_topic`, `line_number_max` 를 넘길 수 있다.

### 토픽

- 구독: `line_command_topic` (`std_msgs/String`) — `"1"`…`"N"` 진입 라인, 또는 **`warehouse`** (홈 = 그래프에서 번호가 가장 큰 `wp_*` 노드)
- 구독: `docking_topic` (설정 시) — 도킹/복그
- 구독: `/action_check` (`std_msgs/String`) — `entering_check`, `goal_finish`, `goal_return_finish`
- 발행: `/action` (`std_msgs/String`) — `entering_start` (라인 도착 후; warehouse·도킹 직행은 생략)

### 데이터

- `data/line_positions.yaml` — 라인 목표 (`line_N`).
- `data/waypoint_graph.yaml` — 노드·간선·`line_goal_links` 등 (그래프 모드).

---

## 빌드

```bash
cd /path/to/onemin_action
colcon build --packages-select onemin_action
source install/setup.bash
```
