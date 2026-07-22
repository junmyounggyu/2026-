# driving/ 사용법 (원본 뼈대 + 튜닝 P제어)

## 폴더 구조
```
driving/
├── AutoLab_lib.py                # 원본 레포에서 그대로 복사 (배너/크레딧, 경고 로그 off)
├── config.py                     # 모터주소/YOLO/튜닝 파라미터
├── yolo_utils.py                 # YOLOv3-tiny 후처리
├── image_processor.py            # BEV -> DPU 추론 -> 각도
├── motor_controller.py           # 모터제어 (자율=P제어 / 수동=원본)
├── driving_system_controller.py  # 모드 전환 + 키 입력 루프
└── main.py                       # ★ 유일한 실행 파일
```
`../dpu/`(dpu.bit 등), `../xmodel/`(lane_class.txt, tiny-yolov3_256.xmodel)은 형제 폴더에 둔다.
※ `AutoLab_lib.py`는 원본 레포 driving 폴더의 파일을 그대로 복사해 넣을 것.

## 실행
```bash
cd driving
python main.py        # 권한 문제 시: sudo python3 main.py
```

## 조작 (원본과 동일)
- 실행하면 먼저 **모드 선택**: `1`(자율) 또는 `2`(수동)
- **Space**: 주행 시작/정지 (토글)
- **1 / 2**: 자율 ↔ 수동 모드 전환
- **수동 모드**: `W`/`S` 전진·후진, `A`/`D` 좌·우 조향, `R` 긴급정지
- **Q**: 프로그램 종료 (⚠️ 대회 중 누르지 말 것)

## 원본과 달라진 점 (자율주행 제어만 교체)
- `motor_controller.control_motors(control_mode=1)`: 3단계 on/off → **연속 P제어**
  (e=목표-실제, u=Kp·e, duty 상하한 clamp, 데드밴드)
- `map_angle_to_range`: 단순 ±7 → **DeadZone→LPF→스케일→Saturation→변화율제한**
- 차선 미검출: 기본값 90 → **직전 각도 유지 후 임계 초과 시 강제 선회**
- 자율 시작 시 `align_steering_straight()`로 앞바퀴 직진 정렬
- 수동 모드(W/A/S/D/R), 모드 전환, Space/Q, 파일 시그니처는 **원본 그대로 유지**

## 튜닝은 config.py에서
`RESISTANCE_MOST_LEFT/RIGHT`(가변저항 좌우 끝값), `STEER_KP`, `STEER_DUTY_MIN/MAX`,
`ALPHA`, `MAX_STEP_DELTA` 등. 대회 당일 가변저항 좌우값은 반드시 재측정해 갱신할 것.

---

## 대회 당일 시나리오 (상세)

### 배경: 왜 "USB-C + Jupyter" 방식으로만 실행해야 하는가
대회 당일에는 **디버깅용 PC(노트북)를 트랙에서 제거한 뒤 블루투스 키보드로만 차를 조작**해야 한다.
따라서 PC와 보드를 잇는 연결을 뽑아도 `main.py`가 죽지 않아야 한다.

⚠️ **중요(실측으로 확인된 사실): JTAG/UART Pod + PuTTY 시리얼 연결은 이 용도로 쓰면 안 된다.**
- 처음엔 "세션이 끊겨도 안 죽게 `nohup ... &`로 백그라운드 실행하면 되겠지"라고 생각했지만,
  **PuTTY(JTAG) 쪽은 `nohup`+`&`로 띄운 뒤에도 케이블을 뽑으면 `main.py`가 즉시 종료됨을 실측으로 확인함.**
- 원인으로 추정되는 것: JTAG 헤더(8핀)에는 데이터 신호(TCK/TMS/TDI/TDO) 외에
  **`PS_SRST_N`(시스템 리셋), `PS_POR_N`(전원 리셋) 신호선이 함께 물려 있다** (Ultra96-V2-HW-User-Guide-v1_3.pdf, 6.2 JTAG Configuration and Debug, Figure 18).
  즉 JTAG 변환기(Pod)를 뽑는 순간은 단순히 "터미널 세션 하나가 끊기는 것"이 아니라
  **보드 자체가 리셋될 수 있는 상황**이라, 세션 분리(`nohup`)로는 막을 수 없는 문제다.
- 반면 **보드의 USB-C(USB 3.0 Type Micro-B) 단자는 일반 데이터 케이블**이라 이런 리셋 신호선이 없다.
  보드를 하나의 USB 네트워크 장치로 인식시키는 용도일 뿐이므로, 뽑아도 보드 자체는 안전하게 계속 동작한다.

**결론**
- JTAG/PuTTY 시리얼 = **개발·디버깅 전용** (콘솔 로그 확인, Vivado 디버깅). 대회 중 뽑을 케이블로 쓰지 말 것.
- 대회 당일 실주행 실행 및 케이블 분리는 반드시 **USB-C 케이블로 연결 → 브라우저로 Jupyter 접속 → 그 안에서 실행**하는 방식으로 진행한다.
  - Jupyter 서버 자체가 보드 안에서 이미 독립적으로 계속 돌고 있는 프로세스이기 때문에,
    Jupyter(노트북 셀 또는 Jupyter 내 [터미널])에서 실행한 `main.py`는 그 서버의 자식 프로세스로 붙어서 돌아가고,
    USB-C 케이블(=단순 데이터/네트워크 연결)을 뽑아도 보드 안에서 계속 실행된다.
  - 안전을 위해 이 경우에도 `nohup ... &`를 함께 써서 실행하는 것을 권장한다(필수는 아니지만 손해볼 것 없음).

### 접속 방법
- **(대회 당일 실행용) USB-C 케이블 + Jupyter**
  - 보드의 USB-C(Micro-B) 단자와 노트북을 케이블로 연결 → 보드가 USB 네트워크 장치로 잡힘
  - 브라우저에서 `http://<IP>:9090/lab` 접속 (PYNQ 기본값은 보통 `http://192.168.3.1:9090/lab`)
  - Jupyter 안의 [터미널] 열어서 아래 실행 절차 진행
- **(개발/디버깅 전용) JTAG/UART Pod + PuTTY 시리얼**
  - IP·Wi-Fi·브라우저 없이 콘솔 로그를 직접 볼 때만 사용
  - PuTTY → `Connection type: Serial`, `Serial line: COMx`, `Speed: 115200` → Open
  - 로그인 계정: `xilinx@pynq`
  - ⚠️ 이 경로로 `main.py`를 띄운 뒤 **이 케이블을 뽑으면 안 됨** (위 배경 설명 참고)

### 실행 절차
```bash
# 1) Jupyter 터미널에서 폴더 이동
cd /home/xilinx/jupyter_notebooks/<대회용 코드 폴더>/driving

# 2) 백그라운드 분리 실행 (USB-C+Jupyter 환경이라 케이블 분리에는 필수는 아니지만, 안전하게 계속 사용 권장)
sudo nohup python3 main.py &
#    └ print 출력은 화면에 안 뜨고 현재 폴더의 nohup.out 파일에 쌓인다.
#      로그 위치를 지정하고 싶으면:
#      sudo nohup python3 main.py > /tmp/driving.log 2>&1 &

# 3) 프로그램이 실제로 떠 있는지 확인
ps aux | grep main.py
#    └ "python3 main.py" 가 포함된 줄이 보이면 실행 중.
#      (grep 자신이 잡힌 "grep main.py" 줄은 무시)

# 4) (선택) 현재 상태 확인 — "주행 모드를 선택하세요" 대기 중인지 로그로 확인
cat nohup.out
#    └ 로그 경로를 지정했다면: cat /tmp/driving.log
```

### 조작 순서 (블루투스 키보드)
```
1. (프로그램이 "모드 선택 대기" 상태)  →  키보드 2  : 수동주행 모드
2. 수동모드 W/A/S/D 로 차를 출발선에 정렬  (또는 직접 들어서 배치)
3. 키보드 1  : 자율주행 모드로 전환
   └ 전환 시 자동으로 정지 상태가 되고 "Space를 눌러 시작" 안내가 뜬다
4. 출발 신호에 맞춰  →  키보드 Space  : 자율주행 출발
   └ 출발 직전 앞바퀴가 자동으로 직진 정렬된 뒤 주행 시작
5. 주행 종료  →  키보드 Space  : 정지  (다시 Space로 재출발 가능)
6. 완전 종료  →  키보드 Q     : 프로그램 종료
```

### PC(연결선) 제거 타이밍
```
실행(2단계) → ps/로그로 확인(3~4단계) → 키보드로 모드 선택까지 마친 뒤
→ USB-C 연결선 제거 → 트랙에서 키보드로 Space 출발
```
USB-C는 단순 데이터 케이블이라 뽑아도 `main.py`는 보드에서 계속 실행된다.
⚠️ **JTAG/UART Pod 케이블은 이 타이밍에 뽑지 말 것** — 리셋 신호선이 물려 있어 보드가 리셋될 수 있다(위 배경 설명 참고).
⚠️ 또한 **연결선(데이터용 USB)과 보드 전원은 별개여야 한다.** 전원까지 그 케이블로
공급받는 구조라면 뽑는 순간 보드가 꺼지므로, 보드는 별도 전원으로 켜져 있어야 함.

### 비상 종료 (q가 안 먹힐 때만)
```bash
# 프로세스 번호(PID) 확인 후 강제 종료
ps aux | grep main.py          # 둘째 열의 숫자가 PID (예: 12345)
sudo kill 12345
# 또는 이름으로 한 번에
sudo pkill -f main.py
```

### 대회 전 반드시 리허설할 것
- [ ] USB-C 케이블 + Jupyter로 `sudo nohup python3 main.py &` 실행 후 **USB-C 연결선을 실제로 뽑았을 때** 프로그램이 유지되는가
- [ ] (참고용) JTAG/PuTTY로 같은 방식 실행 후 케이블을 뽑으면 **프로그램이 종료되는지 재확인** — 종료된다면 대회 당일 이 경로는 절대 사용하지 말 것
- [ ] 연결선 제거 후에도 **블루투스 키보드 페어링이 유지**되어 `1`/`2`/`Space`/`Q`가 먹히는가
- [ ] 연결선(데이터)과 **보드 전원이 분리**되어 있어, 선을 뽑아도 보드가 안 꺼지는가
- [ ] `config.py`의 **가변저항 좌/우 끝값**을 대회 트랙에서 재측정해 갱신했는가
- [ ] USB-C 연결 시 브라우저로 Jupyter(`http://<IP>:9090/lab`)가 정상적으로 잡히는가
- [ ] 조교 확인: **USB-C+Jupyter 실행 방식이 대회 규정상 허용되는지**
