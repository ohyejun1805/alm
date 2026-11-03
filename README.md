# 스마트팜 2륜 레일카: 레일 입구 ArUco 정렬 데모

ROS 1 Noetic과 Gazebo Classic 11 기반 데모입니다. 2륜 차동구동 대차가 전방 하단 카메라로 레일 입구 좌우의
ArUco 마커 두 개를 읽어 위치와 방향을 보정하고, 진입선 앞 목표 지점에 정지한 뒤 레일에 진입해 끝까지 주행합니다.
두 바퀴는 왼쪽 레일 바로 왼쪽, 오른쪽 레일 바로 오른쪽을 지나갑니다.

- **정렬 구간:** 제어 입력은 카메라로 계산한 오차뿐입니다.
- **레일 구간:** 입구를 지나면 마커가 화각 밖으로 나가므로, 마지막 카메라 헤딩을 기준으로 바퀴 엔코더 오도메트리(`/odom`)로 직진합니다.
- **Gazebo 실제 자세:** 결과 검증 도구(`run_recorder.py`, `verify_*.py`)만 읽으며, 이 도구들은 제어 노드로 아무것도 보내지 않습니다.

동작 원리, 장면 사진, 파일 구성은 [docs/PROJECT_OVERVIEW.md](docs/PROJECT_OVERVIEW.md)에 정리했습니다.

## 1. 환경과 설치

확인한 환경에서는 추가 설치가 필요 없었습니다.

| 항목 | 확인 결과 |
|---|---|
| OS | Ubuntu 20.04.6 LTS |
| ROS | Noetic |
| 시뮬레이터 | Gazebo Classic 11.15.1, `gazebo_ros_pkgs` 2.9.3 |
| Python OpenCV | `cv2` 4.13 (`~/.local`, `cv2.aruco` 지원). 시스템 `cv2` 4.2도 지원 |
| 기존 모델 | 재사용 가능한 차동구동 Gazebo 모델이 없어 새로 작성 |

다른 PC에서 의존성이 없다면 다음을 설치하고 확인합니다.

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins ros-noetic-cv-bridge \
                 ros-noetic-xacro ros-noetic-rqt-image-view python3-opencv python3-yaml
rospack find gazebo_ros && gazebo --version
python3 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
```

## 2. 빌드

`~/.bashrc`가 다른 워크스페이스를 source하므로, 이 워크스페이스가 그 위에 겹치지 않도록 Noetic부터 다시 source합니다.

```bash
cd /home/inji2/alm
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 3. 실행

```bash
roslaunch rail_align_demo demo.launch start:=left    # 왼쪽 시작
roslaunch rail_align_demo demo.launch start:=right   # 오른쪽 시작
roslaunch rail_align_demo demo.launch start:=side_left    # 레일 옆 왼쪽에서 90° 돌아간 상태로 시작
roslaunch rail_align_demo demo.launch start:=side_right   # 레일 옆 오른쪽에서 90° 돌아간 상태로 시작
```

Gazebo 창이 다른 창 뒤에 열릴 수 있습니다. 보이지 않으면 Alt+Tab으로 찾으세요.

Gazebo 창과 인식 결과가 그려진 카메라 창이 열립니다. 정렬이 끝나면 터미널에 `ALIGNED`가 출력되고 3초 뒤 레일에 진입하며,
주행이 끝나면 `RAIL_END`가 출력됩니다. 결과를 기록한 뒤에도 시뮬레이션은 켜진 채 정지 상태를 유지합니다. 종료는 `Ctrl+C`입니다.

| launch 인자 | 기본값 | 설명 |
|---|---|---|
| `start` | `left` | `start_poses`의 키(`left`, `right`, `left_inward`, `right_outward`, `side_left`, `side_right`, `blind`) 또는 `custom` |
| `x`, `y`, `yaw_deg` | 없음 | 시작 자세 직접 지정. 예: `start:=custom x:=-2.0 y:=0.15 yaw_deg:=-20` |
| `config` | `config/demo.yaml` | 설정 파일 경로 |
| `gui` | `true` | Gazebo 창 표시 |
| `view` | `gui` 값 | 카메라 창 표시 |
| `control` | `true` | `false`면 인식만 실행하고 차량은 움직이지 않음 |
| `rail_drive` | `true` | `false`면 `ALIGNED`에서 계속 정지 |
| `video` | `false` | `true`면 전경과 ArUco 인식 화면을 합친 MP4를 `results_dir/videos`에 저장 |
| `record` | `true` | CSV와 요약 기록 |
| `exit_on_finish` | `false` | 결과 기록 후 launch 전체 종료 |
| `results_dir` | `<워크스페이스>/results` | 결과 저장 폴더 |

모니터가 없는 환경에서 실행하려면 다음과 같이 합니다.

```bash
xvfb-run -a -s "-screen 0 1280x1024x24" roslaunch rail_align_demo demo.launch gui:=false exit_on_finish:=true
```

## 4. 설정 변경

모든 값은 `src/rail_align_demo/config/demo.yaml` 한 곳에 있습니다. launch가 시작될 때 이 파일로 월드와 마커 텍스처를
생성하고 URDF를 만들며, 인식과 제어 노드도 같은 값을 읽습니다. 값을 바꾸면 다시 빌드할 필요 없이 launch만 다시 실행하면 됩니다.

| 바꾸려는 것 | YAML 위치 |
|---|---|
| 마커 ID, 사전, 크기, 간격, 높이 | `marker.*` |
| 카메라 위치, 높이, 각도, 해상도, 화각 | `camera.*` |
| 시작 자세(왼쪽, 오른쪽 등) | `start_poses.*` |
| 목표 정지 위치 | `target.standoff_m` |
| 성공 기준 | `tolerance.*` |
| 영상, 마커 끊김 판단 시간 | `safety.*` |
| 속도와 제어 이득 | `control.*` |
| 레일 주행 속도, 거리, 진입 전 대기 | `rail_drive.*` |
| 레일 간격, 선 폭, 길이 | `rail.*` |
| 차체, 바퀴 치수 | `vehicle.*` |

다른 값의 예시는 `config/alt_example.yaml`입니다.

```bash
roslaunch rail_align_demo demo.launch config:=$(rospack find rail_align_demo)/config/alt_example.yaml
```

### 시뮬레이션 가정값

**실측값이 없어, 치수는 모두 시뮬레이션 가정값입니다.** 실제 차량과 레일에 적용하기 전에 측정값으로 바꿔야 합니다.

| 항목 | 가정값 |
|---|---|
| 마커 | DICT_4X4_50, 왼쪽 ID 10, 오른쪽 ID 20, 한 변 0.20 m |
| 마커 배치 | 중심 간격 1.10 m(차체와 바퀴가 사이로 통과), 중심 높이 0.30 m, 진입선 위 |
| 레일 | 긴 선 2개. 중심 간격 0.50 m, 폭 3 cm, 길이 6 m. 시각 모델만 있고 충돌 없음 |
| 목표 정지점 | 차축 중심이 진입선 1.40 m 앞 |
| 차체 | 길이 0.80 m, 폭 0.58 m, 높이 0.25 m |
| 바퀴 | 반지름 0.10 m, 폭 4 cm, 중심 간격 0.65 m. 안쪽 면이 레일 바깥 면에서 4 cm 떨어짐 |
| 옆면 시작 | 중심선에서 ±1.50 m, 진입선 5.0 m 앞, 레일과 수직. 시작 시 마커가 화각 밖 |
| 레일 주행 | 0.25 m/s로 5.5 m 이동. 차축 기준 약 x = +4.1 m에서 정지 |
| 카메라 | 차축 앞 0.43 m, 높이 0.15 m, 위로 5°, 1280×720, 수평 화각 80°, 20 Hz, 왜곡 없음 |
| 지면 | 완전한 평면 |

마커 간격을 넓히면 접근 중 마커가 화각 밖으로 나가기 쉽습니다. 치수를 바꾼 뒤에는 단위 테스트
`test_markers_stay_in_view_for_all_start_poses`로 전체 궤적의 화각 여유를 먼저 확인하세요.

## 5. 좌표계와 부호 규칙

레일 좌표계는 Gazebo 월드 좌표계와 같습니다. 원점은 중심선과 진입선의 교점이고, 차량 기준점은 차축 중심(`base_link`)입니다.

```
                 +Y (레일 쪽을 볼 때 왼쪽)
                  ^
                  ■ ID 10  왼쪽 마커 (y = +spacing/2)
                  |
                  |======================  왼쪽 레일 (y = +gauge/2)
 ---- 목표점 ------+---------------------> +X (레일 안쪽)
   x = -standoff  |======================  오른쪽 레일 (y = -gauge/2)
                  | 진입선 x = 0
                  ■ ID 20  오른쪽 마커 (y = -spacing/2)
```

| 오차 | 정의 | 양수의 의미 |
|---|---|---|
| 횡방향 `lateral_error_m` | `y` | 차량이 중심선 **왼쪽**에 있음 |
| 헤딩 `heading_error_deg` | `yaw`, (-180°, 180°] | 차량 코가 레일 방향보다 **왼쪽(반시계)** 으로 돌아감 |
| 종방향 `longitudinal_error_m` | `x_target − x` | 목표점이 아직 **앞**에 있음. 음수면 지나침 |

`T_rail_base = T_rail_optical · T_optical_base`로 계산합니다. `T_rail_optical`은 두 마커의 코너 8개를 알려진 배치에 맞춰
푼 PnP 결과이고, `T_optical_base`는 설정의 카메라 장착값으로 만듭니다.

마커별 자세도 따로 구해 `/rail_align/errors`로 내보냅니다. 두 마커 사이 거리가 설정과 다르거나, 재투영 오차가 크거나,
차량 높이와 기울기가 비현실적이면 해당 프레임은 무효로 처리합니다.

## 6. 제어 흐름

```
WAIT_VISION (↔ SEARCH) → ALIGN_HEADING → APPROACH → FINAL_HEADING → HOLD → ALIGNED → RAIL_DRIVE → RAIL_END
                    ^            │ 중심에서 벗어나 도착           │ 범위 이탈
                    └── RETREAT ←┘                               └→ FINAL_HEADING / TRIM_LONGITUDINAL / RETREAT
```

| 상태 | 동작 |
|---|---|
| `WAIT_VISION` | 유효한 추정이 `start_delay_s` 동안 이어질 때까지 대기 |
| `SEARCH` | `search_direction`이 지정된 시작 자세에서만, 마커 쌍을 처음 찾을 때까지 제자리 회전. `search.max_angle_deg`를 넘기면 `FAILED` |
| `ALIGN_HEADING` | 제자리 회전으로 헤딩 기준값 `-atan(k·lateral)`에 맞춤. 기준값 크기는 두 마커가 화각 안에 남는 각도로 제한 |
| `APPROACH` | 전진과 선회를 조합해 중심선으로 접근하고 목표점에서 멈춤 |
| `RETREAT` | 중심에서 벗어난 채 도착하면 후진 후 재접근(최대 `max_retreats`회) |
| `FINAL_HEADING` | 제자리 회전으로 레일 방향에 맞춤 |
| `HOLD` | 정지 상태로 성공 기준을 `hold_s` 동안 연속 유지하는지 확인 |
| `ALIGNED` | `ALIGNED` 출력 후 `pause_after_aligned_s` 동안 정지. `rail_drive:=false`면 계속 정지 |
| `RAIL_DRIVE` | `/odom` 이동 거리로 `travel_m`까지 직진. 마지막 카메라 헤딩을 기준으로 방향 유지 |
| `RAIL_END` | `RAIL_END` 출력 후 정지 유지 |
| `FAILED` | 후진 횟수를 넘겨도 맞추지 못하면 정지 |
| `SAFETY_STOP` | 영상이나 마커 쌍을 잃으면 속도 0 출력. 복구되면 이전 상태로 복귀 |

명령은 `/cmd_vel`의 `linear.x`와 `angular.z`만 사용합니다. 감시 루프는 벽시계 기준으로 돌기 때문에 시뮬레이션 시계가 멈춰도 0 명령을 계속 보냅니다.

정지 조건은 구간마다 다릅니다.

| 조건 | 첫 검출 전 | 정렬 구간 | 레일 구간 |
|---|---|---|---|
| 마커 쌍 없음/소실 | 정지. `search_direction`이 있으면 탐색 회전 | 정지 | 정상 상황이므로 계속 주행 |
| 영상 끊김 | 정지 | 정지 | 정지 |
| `/odom` 끊김 | 영향 없음 | 영향 없음 | 정지 |

**탐색의 예외:** `SEARCH`는 마커 없이 움직이는 유일한 정렬 전 동작입니다. 첫 검출 전에만, 영상이 들어올 때만, 정해진 각도 안에서만 회전합니다.

**인식 거리 한계:** 마커 0.20 m, 1280×720 기준으로 약 5.6 m 이상 떨어지면 PnP가 잘못된 해를 내는 것을 확인했습니다(실제 횡방향 1.5 m를 0.89 m로 추정).
그래서 `estimator.max_marker_distance_m`(5.5 m)보다 멀면 무효 처리합니다. 시작 위치를 더 멀리 잡으려면 마커나 해상도를 키워야 합니다.

**레일 구간의 한계:** 레일은 시각 모델이라 차량을 물리적으로 안내하지 않고, 오도메트리에는 레일 대비 위치 보정이 없습니다.
실제 차량에서는 바퀴 미끄러짐이 그대로 누적되므로 레일 가이드나 추가 센서로 보정해야 합니다.
또한 레일 진입 중 장애물 감지는 이 데모의 범위가 아닙니다.

## 7. 카메라와 오차 로그 확인

```bash
rqt_image_view /rail_align/debug_image    # 검출 결과와 오차가 표시된 영상
rqt_image_view /rail_cam/image_raw        # 원본 영상
rostopic echo /rail_align/errors          # 프레임별 오차, 검출 ID, 무효 사유
rostopic echo /rail_align/state           # 제어 상태
rostopic echo /cmd_vel                    # 속도 명령
```

| 파일 | 내용 |
|---|---|
| `results/<scenario>_<시각>.csv` | 프레임별 카메라 오차, Gazebo 오차, 명령, 상태 |
| `results/summary.csv` | 실행별 시작 오차, 종료 오차, 소요 시간, 성공 여부 |
| `results/images/<run>_{start,aligned,end}.png` | 시작, 정렬 완료, 종료 시점 카메라 영상 |
| `results/videos/<scenario>_<시각>.mp4` | `video:=true`로 저장한 합성 영상 |
| `results/sign_check.csv` | 오차 부호 검증 결과 |
| `results/safety_check.txt` | 안전 정지 검증 결과 |

요약 표는 `rosrun rail_align_demo summarize_results.py results/summary.csv`로 볼 수 있습니다.

## 8. 영상 녹화

```bash
roslaunch rail_align_demo demo.launch start:=left video:=true
```

`results/videos/<scenario>_<시각>.mp4`로 저장됩니다. 왼쪽에는 Gazebo 전경과 차량 추적 화면이, 오른쪽에는 RGB 카메라의 ArUco 인식 결과가
나옵니다. 인식 화면의 초록 윤곽은 검출한 마커, 빨간 점은 코너 0, 빨강/초록/파랑 축은 마커별 추정 자세입니다. 위쪽 띠에는 오차 또는 무효 사유가 표시됩니다.
레일 구간에서 표시되는 `INVALID: missing_marker_10`은 마커가 화각을 벗어났다는 뜻이며 고장이 아닙니다.

추적 카메라와 전경 카메라는 `video:=true`일 때만 추가됩니다. 재생 속도는 시뮬레이션 시간에 맞춰지며, 현재 시간은 화면 하단의 `t`입니다.
영상은 Gazebo 내부 카메라 토픽으로 만들기 때문에 `gui:=false`로도 찍을 수 있고 Gazebo 창이 가려져도 영향을 받지 않습니다.

## 9. 검증 재현

```bash
# 인식, 부호, 좌우 시나리오, 안전 정지를 순서대로 모두 실행
rosrun rail_align_demo run_all_checks.sh /home/inji2/alm/results 3

# ROS 없이 실행하는 단위 테스트
cd src/rail_align_demo && python3 -m unittest discover -s tests
```

단계별로 따로 확인하려면 터미널 두 개를 사용합니다.

```bash
# 터미널 1
roslaunch rail_align_demo demo.launch gui:=false control:=false record:=false
# 터미널 2: 차량을 옮겨 가며 부호 확인
rosrun rail_align_demo verify_perception.py --results-dir results

# 터미널 1
roslaunch rail_align_demo demo.launch gui:=false record:=false
# 터미널 2: occlude | pause | railpause | nofeed (blind는 터미널 1에 start:=blind)
rosrun rail_align_demo verify_safety.py occlude --results-dir results
```

## 10. 검증 결과

직접 실행한 결과입니다. 값은 `횡방향(m) / 헤딩(°) / 종방향(m)` 순서입니다. 기준값은 카메라 추정과 Gazebo 실제 자세 모두에 적용했습니다.

### 인식과 오차 부호 (제어 연결 전)

Gazebo 카메라 영상에서 ID 10, 20을 10개 자세 모두에서 검출했습니다. 차량을 좌우로 옮기거나 회전시켜도 부호가 규칙과 일치했고,
실제 자세와의 차이도 성공 기준 안이었습니다. 자세한 값은 `results/sign_check.csv`에 있습니다.

### 시나리오

값은 `횡방향(m) / 헤딩(°) / 종방향(m)` 순서입니다. `side_*`는 탐색 후 첫 유효 추정 시점의 값이며, 시간은 탐색을 포함합니다.

| 실행 | 스폰 자세 (y m / yaw ° / x m) | 첫 유효 추정 (Gazebo) | 정렬 종료 (카메라) | 정렬 종료 (Gazebo) | 정렬 시간 (s) | 정렬 성공 | 레일 끝 x (m) | 레일 최대 횡편차 (mm) | 바퀴가 레일 바깥 유지 |
|---|---|---|---|---|---|---|---|---|---|
| left | +0.30 / +12 / x=-3.0 | +0.300 / +12.00 / +1.600 | +0.002 / -0.15 / +0.007 | +0.002 / -0.13 / +0.007 | 16.5 | 성공 | 4.097 | 5.6 | 예 |
| right | -0.25 / +15 / x=-3.0 | -0.250 / +15.00 / +1.600 | +0.000 / -0.06 / +0.007 | +0.002 / -0.06 / +0.007 | 15.3 | 성공 | 4.097 | 2.0 | 예 |
| left | +0.30 / +12 / x=-3.0 | +0.300 / +12.00 / +1.600 | +0.001 / -0.14 / +0.008 | +0.002 / -0.12 / +0.008 | 16.6 | 성공 | 4.096 | 4.7 | 예 |
| right | -0.25 / +15 / x=-3.0 | -0.250 / +15.00 / +1.600 | +0.001 / -0.04 / +0.007 | +0.002 / -0.03 / +0.007 | 15.3 | 성공 | 4.098 | 4.5 | 예 |
| left | +0.30 / +12 / x=-3.0 | +0.300 / +12.01 / +1.600 | +0.001 / -0.10 / +0.007 | +0.002 / -0.10 / +0.008 | 16.6 | 성공 | 4.097 | 3.2 | 예 |
| right | -0.25 / +15 / x=-3.0 | -0.250 / +15.00 / +1.600 | +0.000 / -0.04 / +0.007 | +0.002 / -0.06 / +0.007 | 15.3 | 성공 | 4.081 | 3.9 | 예 |
| left_inward | +0.20 / -15 / x=-2.8 | +0.200 / -15.00 / +1.400 | +0.001 / -0.10 / +0.008 | +0.002 / -0.10 / +0.008 | 13.7 | 성공 | 4.097 | 3.5 | 예 |
| right_outward | -0.30 / -12 / x=-3.0 | -0.300 / -12.00 / +1.600 | -0.000 / +0.01 / +0.007 | +0.001 / +0.01 / +0.007 | 16.7 | 성공 | 4.099 | 2.1 | 예 |
| side_left | +1.50 / -90 / x=-5.0 | +1.500 / -46.18 / +3.600 | +0.001 / -0.07 / +0.006 | +0.002 / -0.07 / +0.006 | 36.1 | 성공 | 4.100 | 3.0 | 예 |
| side_right | -1.50 / +90 / x=-5.0 | -1.500 / +45.88 / +3.600 | +0.000 / -0.09 / +0.007 | +0.002 / -0.10 / +0.007 | 36.2 | 성공 | 4.099 | 1.9 | 예 |
| custom | +0.15 / -20 / x=-2.6 | +0.150 / -20.00 / +1.200 | +0.002 / -0.17 / +0.007 | +0.002 / -0.14 / +0.007 | 12.1 | 성공 | 4.098 | 6.9 | 예 |
| side_left | +1.50 / -90 / x=-5.0 | +1.500 / -46.19 / +3.600 | +0.000 / -0.05 / +0.006 | +0.002 / -0.05 / +0.006 | 36.0 | 성공 | 4.100 | 4.1 | 예 |
| side_right | -1.50 / +90 / x=-5.0 | -1.500 / +45.85 / +3.600 | +0.000 / -0.04 / +0.007 | +0.002 / -0.05 / +0.007 | 36.1 | 성공 | 4.099 | 2.5 | 예 |

### 안전 정지

| 조건 | 결과 | 측정값 |
|---|---|---|
| 접근 중 왼쪽 마커 가림 | 통과 | 0.44 s 안에 0 명령, 3 s 유지, 이동 4.8 mm, 가림 해제 후 재개 |
| 접근 중 영상 정지 (`pause_physics`) | 통과 | 0.30 s 안에 0 명령, 해제 후 재개 |
| 레일 주행 중 영상과 오도메트리 정지 | 통과 | 0.49 s 안에 0 명령, 해제 후 재개 |
| 접근 중 추정 노드 종료 | 통과 | 0.34 s 안에 0 명령, 이동 3.3 mm |
| 처음부터 마커 미검출 (`start:=blind`) | 통과 | 8 s 동안 명령 160개 모두 0, 이동 1.3 mm |

### 기타 확인

| 항목 | 결과 |
|---|---|
| 단위 테스트 22개 | 시스템 OpenCV 4.2와 `~/.local`의 4.13 양쪽에서 통과 |
| `config/alt_example.yaml` (ID, 사전, 치수, 기준을 모두 변경) | `RAIL_END` 도달, 정렬 기준 충족 |
| 시스템 OpenCV 4.2 (`PYTHONNOUSERSITE=1`) | `RAIL_END` 도달, 정렬 기준 충족 |
| `xvfb-run` 무모니터 실행 | `RAIL_END` 도달, 정렬 기준 충족 |
| `rail_drive:=false` | `ALIGNED`에서 정지 유지 |
| GUI 실행과 `Ctrl+C` 종료 | 잔여 프로세스 없음 |

### 미검증

| 항목 | 이유 | 이어서 할 일 |
|---|---|---|
| 실제 차량과 레일 | 실측 치수와 하드웨어 없음 | 설정을 실측값으로 바꾸고 카메라 보정 후 시험 |
| 조명 변화, 마커 오염, 렌즈 왜곡 | 시뮬레이션이 이상적 조건 | 실제 환경 영상으로 인식률 측정 |
| 탐색 각도 초과(`FAILED`) | 단위 테스트로만 확인 | `start:=blind`에 `search_direction`을 지정해 시험 |
| `/odom`만 단독으로 끊긴 경우 | Gazebo에서 단독 재현을 하지 않음. 단위 테스트로만 확인 | 필요 시 `/odom`을 중계 노드로 끊어 시험 |
