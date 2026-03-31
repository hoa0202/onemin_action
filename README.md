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

사람이 육안으로 확인한 뒤 `/action` 또는 `/action_check`로 수동 명령을 보낸다.

### 실행

**대화형** (터미널에서 키 입력):

```bash
ros2 run onemin_action manual_action_command_node
```

- **1~9** : `/line_to_enter`에 해당 라인 번호 발행 (코드2 시나리오에 진입 라인 요청)
- **n** : `/action`에 `entering_next` (다음 단계로)
- **e** : `/action`에 `entering_end` (시나리오 종료)
- **c** : `/action_check`에 `entering_check` (진입 확인 수신 알림)
- **f** : `/action_check`에 `goal_finish` (목표 완료 알림)
- **r** : `/action_check`에 `goal_return_finish` (복귀 완료 알림)
- **q** 또는 **Ctrl+C** : 노드 종료

**한 번만 전송 후 종료**:

```bash
ros2 run onemin_action manual_action_command_node 1              # 라인 1 진입 요청
ros2 run onemin_action manual_action_command_node entering_next
ros2 run onemin_action manual_action_command_node entering_end
ros2 run onemin_action manual_action_command_node entering_check
ros2 run onemin_action manual_action_command_node goal_finish
ros2 run onemin_action manual_action_command_node goal_return_finish
```

### 토픽

- 발행: `/line_to_enter` (`std_msgs/Int32`) — 진입할 라인 번호(1, 2, 3…)
- 발행: `/action` (`std_msgs/String`) — `entering_next`, `entering_end`
- 발행: `/action_check` (`std_msgs/String`) — `entering_check`, `goal_finish`, `goal_return_finish`

---

## 코드2: 시나리오 제어 (scenario_controller_node)

진입할 라인 번호 수신 → YAML에서 좌표 로드 → Nav2 이동 → `entering_start` 발행 → `entering_check` 확인 → `goal_finish` 로그 → `goal_return_finish` 대기 후 다시 라인 입력 대기 (반복).

### 실행

```bash
ros2 launch onemin_action scenario_control.launch.py
```

라인 입력은 **다른 터미널**에서 토픽으로 전송:

```bash
ros2 topic pub --once /line_to_enter std_msgs/Int32 "{data: 1}"
```

(1 = 1번 라인, 2 = 2번 라인, … 0 미사용)

### 파라미터

| 파라미터                       | 기본값               | 설명 |
|--------------------------------|----------------------|------|
| `nav2_action_name`             | `navigate_to_pose`   | Nav2 NavigateToPose 액션 이름 |
| `entering_check_timeout_sec`   | `0.5`                | entering_check 대기 시간(초) |
| `entering_check_max_retries`  | `10`                 | entering_check 미수신 시 entering_start 최대 재전송 횟수 |

### 토픽

- 구독: `/line_to_enter` (`std_msgs/Int32`) — 진입할 라인 번호(1, 2, 3…)
- 구독: `/action_check` (`std_msgs/String`) — `entering_check`, `goal_finish`, `goal_return_finish`
- 발행: `/action` (`std_msgs/String`) — `entering_start`

### 데이터

- `data/line_positions.yaml` (코드1과 동일 경로)에서 `line_N` 좌표 로드.

---

## 빌드

```bash
cd /path/to/onemin_action
colcon build --packages-select onemin_action
source install/setup.bash
```
