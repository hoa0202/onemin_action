# onemin_action

라인 입구 출발 위치 기록 및 시나리오 제어용 ROS2 패키지.

## 빠른 사용 흐름

1. **데이터 준비** (패키지 `data/` 또는 워크스페이스 `src/onemin_action/data/`)
   - `line_positions.yaml` — `line_1` … 및 (선택) `warehouse`
   - `waypoint_graph.yaml` — 그래프 모드 시 `nodes`, `edges`, `line_goal_links`
2. **빌드**

   ```bash
   cd /path/to/workspace
   colcon build --packages-select onemin_action
   source install/setup.bash
   ```

3. **시나리오 노드**

   ```bash
   ros2 launch onemin_action scenario_control.launch.py
   ```

4. **라인 / 창고 이동** (별 터미널)

   ```bash
   ros2 topic pub --once /harv_robot/line_move std_msgs/String "{data: '2'}"
   ros2 topic pub --once /harv_robot/line_move std_msgs/String "{data: 'warehouse'}"
   ```

5. **수확 시퀀스** (라인 도착 후)  
   다른 노드가 `/action_check` 등으로 `entering_check` → `goal_finish` → `goal_return_finish` 를 보내야 다음 라인 입력으로 돌아감.  
   **`WAIT_GOAL_FINISH` / `WAIT_GOAL_RETURN_FINISH` 상태에서는 `line_move`가 무시된다** — 로그에 `line_move 무시`가 나오면 수확 플로우가 끝날 때까지 기다리거나 `entering_check`/`goal_*`를 맞춰 줄 것.

---

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
  frame_id: "odom_1"
  position: { x: 0.0, y: 0.0, z: 0.0 }
  orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
line_2:
  ...
# (선택) 수확 창고/홈 목표. 있으면 토픽 warehouse 는 이 pose 로 간다. 없으면 최대 번호 line_N 폴백.
warehouse:
  frame_id: "odom_1"
  position: { x: 0.0, y: 0.0, z: 0.0 }
  orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
```

**그래프 모드**에서는 반드시 `waypoint_graph.yaml` 의 `line_goal_links` 에 **`warehouse`** 키를 넣고, 목표 pose로 이어질 **그래프 노드(wp_*)** 목록을 적는다.

- 한 노드만 두면 Nav2 global costmap 상 그 구간이 막혀 `"no valid path"` 가 날 수 있다. **같은 창고 목표에 대해 여러 wp 를 나열**하면 (예: `wp_2`, `wp_1`) 다익스트라가 비용이 작은 쪽으로 골라, 비어 있는 통로 쪽에서만 마지막까지 가게 할 수 있다.
- `line_goal_links` 의 키는 문자열로 통일된다 (`1`, `2`, `warehouse` 등).

예 (`config/waypoint_graph.example.yaml` 하단 레퍼런스 주석 포함 — `data/` 는 종종 gitignore):

```yaml
line_goal_links:
  "1": [wp_5]
  "2": [wp_6]
  warehouse:
    - wp_2
    - wp_1
```

**`graph_build`** (선택): `rebuild_waypoint_graph_edges` / 그래프 기록 노드가 `edges` 를 다시 만들 때 쓴다. `edges` 를 수동 유지하면 `{}` 로 둬도 된다.

```yaml
graph_build:
  node_id_prefix: wp
  extra_bidirectional_pairs:
    - [wp_1, wp_4]
  excluded_node_ids: []
  # chain_node_order: [wp_1, wp_2, wp_3]

nodes:
  wp_1:
    frame_id: odom_1
    position:
      x: 1.291922
      y: -0.881204
      z: 0.0
    orientation:
      x: -0.000968
      y: 0.001286
      z: 0.011866
      w: 0.999928
  wp_2:
    frame_id: odom_1
    position:
      x: 1.362242
      y: 0.128558
      z: -0.0
    orientation:
      x: 0.002169
      y: -0.001405
      z: 0.009473
      w: 0.999952
  wp_3:
    frame_id: odom_1
    position:
      x: 2.621927
      y: 0.147455
      z: 0.0
    orientation:
      x: 0.013911
      y: -0.006187
      z: -0.669609
      w: 0.742558
edges:
- from: wp_1
  to: wp_2
  cost: 1.012208
- from: wp_2
  to: wp_1
  cost: 1.012208
- from: wp_2
  to: wp_3
  cost: 1.259827
- from: wp_3
  to: wp_2
  cost: 1.259827
line_goal_links: 
  warehouse: 
  - wp_1
  - wp_2
docking_goal_links:
  move_to_docking_station:
  - wp_3
  move_to_return:
  - wp_1
  - wp_2
graph_build: {}
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
- **w** : `warehouse` 발행 → `line_positions` 의 **`warehouse`** 항목(없으면 최대 `line_N`)
- **n** : `/action` → `entering_next`
- **e** : `/action` → `entering_end`
- **c** : `/action_check` → `entering_check`
- **f** : `/action_check` → `goal_finish`
- **r** : `/action_check` → `goal_return_finish`
- **d** : 도킹 스테이션 (`move_to_docking_station`)
- **b** : 도킹 복귀 (`move_to_return`)
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
**warehouse** 로 도착하면 **수확 시나리오 없이** IDLE로 돌아간다.

### 실행

```bash
ros2 launch onemin_action scenario_control.launch.py
```

라인/홈 입력은 **다른 터미널**에서:

```bash
ros2 topic pub --once /harv_robot/line_move std_msgs/String "{data: '1'}"
ros2 topic pub --once /harv_robot/line_move std_msgs/String "{data: 'warehouse'}"
```

Nav2 `nav_goal_output_frame` 을 map 으로 맞춰야 할 환경이면 launch 인자로 넘긴다 (README는 패키지 기본; 로봇 쪽 launch에서 override).

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
| `graph_path_dot_min`           | `0.0`                | 역행 검사(음수면 약간 허용). 막히면 튜닝 |
| `docking_topic`                | `/harv_robot/docking_move` | 도킹 명령 (`std_msgs/String`) |

`scenario_control.launch.py` 에서 `line_command_topic`, `line_number_max` 등을 넘길 수 있다.

### 토픽

- 구독: `line_command_topic` (`std_msgs/String`) — `"1"`…`"N"` 진입 라인, 또는 **`warehouse`**
- 구독: `docking_topic` (설정 시) — 도킹/복귀
- 구독: `/action_check` (`std_msgs/String`) — `entering_check`, `goal_finish`, `goal_return_finish`
- 발행: `/action` (`std_msgs/String`) — `entering_start` (라인 도착 후; warehouse·도킹 직행은 생략)

### 데이터

- `data/line_positions.yaml` — 라인 목표 (`line_N`), (선택) `warehouse`
- `data/waypoint_graph.yaml` — 노드·간선·`line_goal_links` 등 (그래프 모드)

저장소 `.gitignore` 에는 운영용 `data/line_positions.yaml` · `waypoint_graph.yaml` 이 제외될 수 있음 — 팀원은 `config/*.example.yaml` 을 참고해 로컬 `data/` 에 복사해 사용한다. **`line_goal_links` / `graph_build` 필드 채움 예시는 `config/waypoint_graph.example.yaml` 맨 아래 주석 블록**을 보면 된다.

---

## 트러블슈팅 (요약)

| 증상 | 점검 |
|------|------|
| `warehouse 경로 실패 (line_goal_links.'warehouse'…)` | `waypoint_graph.yaml` 에 `line_goal_links.warehouse` 및 최소 한 개 이상 `wp_*` 링크 |
| Nav2 `no valid path found` (그래프상 연결은 있는데) | global costmap 에 막힌 영역; `warehouse` 링크에 **여러 wp** 추가해 우회, 또는 맵/인플레이션 조정 |
| `wait_goal_finish 에서는 line_move 무시` | 수확 플로우 끝까지 `/action_check` 시퀀스 대기 |
| `entering_check` 타임아웃 반복 | 상대 노드가 `entering_check` 를 보내는지, 토픽 이름·메시지 문자열 일치 여부 |

---

## 빌드

```bash
cd /path/to/workspace
colcon build --packages-select onemin_action
source install/setup.bash
```
