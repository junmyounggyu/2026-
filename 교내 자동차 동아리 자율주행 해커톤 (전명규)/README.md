# 2025 자율주행 해커톤: FSDS(Formula Student Driverless Simulator) 경로 추종 제어

2025 자율주행 해커톤에서 진행한 팀 프로젝트입니다. 대회는 2D 시뮬레이터 파트와 FSDS(Formula Student Driverless Simulator) 파트로 나뉘어 있었고, 본인은 그중 **FSDS 파트의 주행 제어 코드 구현**을 담당했습니다.

- 대회: 2025 자율주행 해커톤
- 담당 역할: FSDS 파트 — Stanley 기반 경로 추종 제어 코드(`control.py`) 구현
- 최종 결과: **종합 3위**

## 1. 개발 환경

| 항목 | 내용 |
|---|---|
| OS | Ubuntu |
| 미들웨어 | ROS1 (rospy, catkin workspace) |
| 시뮬레이터 | FSDS (Formula Student Driverless Simulator, Unreal Engine 기반) |
| 언어/라이브러리 | Python 3, rospy, NumPy, tf |
| 사용 메시지 | `fs_msgs/Track`, `fs_msgs/Cone`, `fs_msgs/ControlCommand`, `nav_msgs/Odometry`, `nav_msgs/Path`, `geometry_msgs/TwistWithCovarianceStamped` |

### 실행 방법

```bash
# 1. 시뮬레이터 실행
cd ~/fsds
./FSDS.sh
# 시뮬레이터 창에서 "Run Simulation" 버튼 클릭

# 2. FSDS-ROS 브릿지 실행
cd ~/Formula-Student-Driverless-Simulator/ros
source devel/setup.bash
roslaunch fsds_ros_bridge fsds_ros_bridge.launch

# 3. 주행 제어 노드 실행
cd ~/hack_ws
source devel/setup.bash
roslaunch formula control.launch
```

대회 규정상 `/Formula/formula/src/control.py` 파일만 수정하여 제어를 구현해야 했으며(추가 파일 생성은 허용, 그 외 기존 코드 수정은 불가), 사용 가능한 정보는 차량의 위치(x, y), yaw(x축 기준 반시계방향 +), 속도, 그리고 라바콘 정보로 생성한 경로(`calculate_midpoints()`에서 수정 가능)로 제한되었습니다.

## 2. 이론적 배경

### Stanley Control (횡방향 제어)
1번 프로젝트(모바일시스템제어 텀프로젝트)와 동일하게 Stanley 제어 기법을 채택했습니다. 전륜 차축 중심을 기준점으로 삼아, 헤딩 오차(경로 접선 방향과 차량 진행 방향의 차이)와 횡방향 오차(전륜 중심에서 경로까지의 최단 거리, cross-track error)를 동시에 보정합니다.

`δ = ψ_e + arctan(k·e / (v + ε))`

이 프로젝트에서는 라바콘(파란색/노란색) 위치 정보만 주어지므로, 별도의 사전 제작 경로 파일 없이 코드 내에서 실시간으로 파란색 콘과 노란색 콘을 페어링해 그 중점을 잇는 중간선(midline)을 생성하고, 이를 Stanley 제어기의 추종 목표 경로로 사용했습니다.

### 종방향 제어
목표 속도 대비 현재 속도 오차에 비례하는 단순 P 제어로 throttle/brake를 계산했으며, 조향각이 일정 수준(약 17.5도) 이상으로 커지는 코너 구간에서는 목표 속도를 70%로 낮춰 코너 통과 안정성을 확보했습니다.

## 3. 구현, 검증, 개선

**경로 생성 (`calculate_midpoints`)**
`/fsds/testing_only/track` 토픽으로 수신한 콘들을 색상별(BLUE/YELLOW)로 분류한 뒤, 두 리스트를 인덱스 순서로 페어링해 각 쌍의 (x, y, z) 평균 위치를 중간점으로 계산했습니다. 이렇게 생성한 `mid_points`를 Stanley 제어기의 추종 경로로 사용하고, RViz 확인을 위해 `Path`, `MarkerArray` 메시지로도 발행했습니다.

**차량 상태 추정**
`/fsds/testing_only/odom`으로 위치·자세를, `/fsds/gss`로 속도를 각각 구독했습니다. 쿼터니언 자세값은 `tf.transformations.euler_from_quaternion`으로 yaw만 추출해 사용했고, 매 콜백마다 차량 pose를 TF로 브로드캐스트해 RViz에서 실시간으로 차량 위치를 확인할 수 있도록 했습니다.

**제어 구현**
Stanley 제어로 조향각(steering)을, 목표 속도 대비 오차 기반 P 제어로 throttle/brake를 계산해 `ControlCommand` 메시지로 30Hz 주기로 발행했습니다. 무게중심(CoG) 좌표를 축거(wheelbase=1.55m)의 절반만큼 전방으로 투영해 전륜 중심 좌표를 구하고, 이 지점 기준으로 최근접 경로점과 헤딩 오차·횡방향 오차를 계산하는 방식은 1번 프로젝트의 Stanley 구현과 동일한 접근입니다.

**검증**
시뮬레이터 로그에서 콘 충돌 횟수와 랩타임을 확인하며 게인(k=0.85)과 목표 속도(target_speed=10.0 m/s), 코너 감속 임계값을 조정했습니다.

## 4. 평가 방식 및 결과

- 주행 방식: 5바퀴 주행 후 시간 측정 (주황색 콘 사이 진입 시 타이머 시작, 랩타임 기록)
- 페널티: 콘 충돌 1회당 4초 가산, 트랙 5m 이상 이탈 시 DNF(주행 실패)
- 차량 사양: 축거(Wheelbase) 1.55m, 최고속도 27m/s, Odometry 기준점은 차량 무게중심
- 총점 산정: 2D 시뮬레이터 파트 60% + FSDS 파트 40%로 종합 점수 산정
- **최종 결과: 종합 3위**

## 5. 파일 구성

- `control.py` — FSDS 트랙(라바콘)을 인식해 중간선을 생성하고, Stanley 횡방향 제어 + P 종방향 제어로 주행하는 최종 제어 코드

## 6. 상장
![Uploading 자율주행해커톤_상장_3등.jpg…]()

