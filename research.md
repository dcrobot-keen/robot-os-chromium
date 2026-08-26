# ROS → Chromium 포팅 리서치

## 배경

ROS(Robot Operating System) 관련 기능을 Chromium 기반으로 포팅하는 방향을 탐색한 리서치 기록. 논의 과정에서 방향이 한 번 크게 전환됐다 — 처음에는 "ROS 런타임 자체를 브라우저에서 돌린다"는 전제로 시작했지만, 최종적으로는 "Chromium의 네이티브 하드웨어 접근 API로 ROS의 드라이버 레이어를 대체하고, 하드 리얼타임은 처음부터 브라우저 밖(펌웨어)에 둔다"는 방향으로 정리됐다.

---

## 1. 첫 번째 방향 — ROS 런타임을 브라우저로 포팅

### ROS2WASM
QUT(Queensland University of Technology) + RWTH Aachen이 2024년 발표한 프로젝트. RoboStack 기반으로 ROS 2 패키지 전체를 Emscripten으로 WebAssembly 크로스컴파일하고, `rmw-wasm`이라는 자체 미들웨어로 ROS 2 노드 그래프(pub/sub)를 브라우저 안에서 재현한다.

- 지연시간 오버헤드 ~20%, 이벤트 처리율 14kHz 이상
- LEGO BOOST Vernie를 Bluetooth로 제어하는 실물 로봇 데모 존재
- [arXiv:2409.09941](https://arxiv.org/abs/2409.09941) / [IEEE](https://ieeexplore.ieee.org/document/11127821/)

### 브릿지형 접근 (업계 표준 패턴)
ROS는 그대로 네이티브로 두고 웹은 브릿지로만 연결하는, 훨씬 성숙한 기존 패턴들:

- **rosbridge_suite + roslibjs/ros2djs/ros3djs** (Robot Web Tools) — WebSocket으로 ROS 토픽/서비스를 JSON으로 노출
- **Foxglove Studio** — Webviz(Cruise)의 포크, ROS/ROS2 데이터 시각화·디버깅
- **ROSboard** — 로봇을 경량 웹서버로 만들어 대시보드 제공
- **CEF(Chromium Embedded Framework)** — ROS 전용은 아니지만 로봇 HMI를 웹기술로 만들고 임베드하는 범용 패턴

### 이 방향을 접은 이유
ROS2WASM은 "ROS 소프트웨어 스택(노드 그래프, pub/sub)을 브라우저에 재현하는" 문제를 풀 뿐, "하드웨어와 얼마나 낮은 지연으로 결정론적으로 통신하는가"라는 문제는 다루지 않는다. WASM은 브라우저 JS 엔진 위 샌드박스 실행 환경일 뿐이라 하드웨어 접근은 결국 동일한 Web API를 거쳐야 하고, 커널 드라이버·DMA·메모리 매핑 I/O에 우회 접근할 권한이 생기지 않는다. 오히려 WASM 계층을 하나 더 얹으면 지연이 늘어난다(논문 자체가 ~20% 오버헤드를 보고). 브라우저의 JS 이벤트 루프, GC, OS 스케줄러 모두 결정론적 타이밍을 보장하지 않으므로, 이 한계는 WASM 유무와 무관한 "브라우저"라는 실행 환경 자체의 구조적 한계다.

---

## 2. 두 번째 방향 — Chromium 하드웨어 API + 실시간은 펌웨어에 위임

### Chromium 하드웨어 접근 API (Project Fugu)
Project Fugu는 웹앱에 네이티브급 기능을 부여하는 Chromium 주도 프로젝트. 그중 하드웨어 직접 접근 API 4가지:

| API | 대상 | 비고 |
|---|---|---|
| WebSerial | UART/시리얼 장치 | 아두이노, 모터 드라이버 보드 등 |
| WebUSB | 비표준 USB 장치 전반 | |
| WebHID | HID 장치 | 조이스틱, 레거시 HID 장비 |
| Web Bluetooth | BLE GATT 장치 | W3C 정식 표준 아님, 커뮤니티 그룹 스펙 |

공통점: 크로미움 계열 브라우저에서만 지원, 드라이버 설치 불필요, HTTPS(보안 컨텍스트) 필수, 최초 연결에 사용자 제스처 필요.

### 실사례
- **HuggingFace LeRobot.js / LeLab** — 가장 근접한 실사례. Python LeRobot(로봇 팔 teleoperation/모방학습)의 JS 포트. WebSerial/WebUSB로 브라우저에서 직접 SO-100 로봇 팔에 연결, 캘리브레이션·teleoperation을 브라우저 GUI만으로 수행. ([소개](https://huggingface.co/blog/NERDDISCO/lerobotjs), [LeLab](https://www.hackster.io/news/lelab-is-hugging-face-s-new-browser-based-gui-for-the-lerobot-ecosystem-d73ff19088f6))
- **Arduino Cloud Editor** — WebSerial로 브라우저에서 스케치 업로드
- **Microsoft MakeCode** — WebSerial로 마이크로컨트롤러 플래싱
- **BBC micro:bit** — WebUSB 기반 연결/디버깅

### 한계
크로미움 계열 전용(Firefox/Safari 미지원), 커널 레벨 리얼타임 보장 없음, DMA/메모리 매핑 I/O 불가. 하드 리얼타임이 필요한 모터 제어 루프에는 그대로 쓰기 어려움.

### 리얼타임 문제의 실제 해법
ROS 자체도 순정 상태에서는 하드 리얼타임을 보장하지 않는다. 진짜 마이크로초 단위 PID 루프는 보통 ROS 노드가 아니라 별도 마이크로컨트롤러 펌웨어(Zephyr, FreeRTOS)나 EtherCAT/CAN 같은 전용 필드버스에 위임되고, ROS는 그 위에서 목표값만 던져주는 상위 감독 레이어 역할을 한다. 이 구조를 그대로 가져오면 브라우저는 WebSerial로 목표값만 마이크로컨트롤러에 전달하고, 실제 리얼타임 루프는 펌웨어 안에 그대로 남는다. LeRobot.js가 정확히 이 패턴.

---

## 3. 아키텍처 선례 리서치

새 스택의 각 레이어를 설계하며 참고한 기존 사례:

- **Firmata / Johnny-Five** — 마이크로컨트롤러에 범용 펌웨어(Firmata)를 얹고, 호스트(원래 Node.js)가 시리얼로 핀 상태를 읽고 명령을 내리는 host-client 모델. "실시간은 펌웨어, 로직은 호스트"라는 레이어 분리의 기존 증명 사례.
- **Viam** — ROS를 의도적으로 쓰지 않고 처음부터 다시 설계한 상용 로보틱스 플랫폼. ROS의 토픽 기반 pub/sub(DDS) 대신 **gRPC + WebRTC**를 통신 계층으로 채택. 로컬은 유닉스 도메인 소켓 위 gRPC, 원격은 시그널링 서버를 거친 WebRTC P2P. WebRTC가 브라우저 네이티브 기술이라 브라우저 클라이언트가 이 아키텍처에서 1급 시민.
- **W3C Web of Things (WoT)** — 장치를 "속성(property)·동작(action)·이벤트(event)"로 추상화하고 실제 프로토콜은 바인딩으로 감추는 표준. ROS의 메시지 타입 정의(.msg)와 드라이버 노드 역할을 대체할 수 있는 표준 스펙.
- **BroadcastChannel / SharedWorker** — 같은 origin 내 탭/워커 간 pub/sub(BroadcastChannel)과 여러 탭이 공유하는 단일 인스턴스(SharedWorker)를 위한 브라우저 네이티브 API. Slack, Figma 등이 탭 간 상태 동기화에 실제로 사용.

---

## 4. 최종 아키텍처 (요약)

7개 레이어로 구성된 스택. 상세 설계, 와이어 프로토콜, TypeScript 인터페이스, 시퀀스 다이어그램은 별도 아키텍처 문서(Artifact) 참고: [Chromium Robotics Stack](https://claude.ai/code/artifact/801cdb9e-e663-48a5-92bc-8e43a08d4c46)

1. **펌웨어** — 실시간 제어 루프, 워치독/E-STOP. 샌드박스 경계 아래.
2. **브라우저 하드웨어 브리지** — WebSerial/WebUSB/WebHID/Web Bluetooth 래퍼
3. **장치 추상화** — WoT 스타일 속성/동작/이벤트
4. **애플리케이션 노드** — Web Worker 단위 로직(teleop, odometry, safety monitor)
5. **통신** — 로컬은 BroadcastChannel+SharedWorker, 원격은 WebRTC
6. **HMI/시각화** — 같은 브라우저에서 3D 뷰·텔레메트리·비디오
7. **시그널링/플릿** — 원격 세션에서만 필요, 데이터 평면에는 관여 안 함

핵심 안전 모델: 펌웨어가 워치독과 E-STOP을 무조건적으로 소유한다. 하트비트 300ms 타임아웃 시 브라우저와 무관하게 모터를 정지시킨다. 탭 프리징, GC 정지, USB 케이블 분리, 브라우저 크래시가 전부 동일하게 처리된다.

## 5. 기존 ROS 패키지 매핑

구현 시 실제로는 **2개 레포(펌웨어 레포 + 웹 모노레포) / 8개 컴포넌트**로 나뉜다. 각 컴포넌트를 기존 ROS 개념에 대응시키면:

| 컴포넌트 | ROS 대응물 | 비고 |
|---|---|---|
| 펌웨어 레포 | rosserial(ROS1) / micro-ROS(ROS2) | micro-ROS는 MCU에 ROS 그래프 자체를 얹지만, 이 설계는 순수 커맨드 프로토콜만 둠(Firmata에 더 가까움) |
| transport | `serial_driver`, `rosserial_python`/`rosserial_server`, `usb_cam` | 디바이스 핸들을 열고 원시 프로토콜을 읽고 쓰는 역할 |
| device-abstraction | `ros2_control`의 `hardware_interface` + `.msg`/`.srv` 정의 | 하드웨어 종류 무관 통일 인터페이스 |
| bus | DDS/RTPS(ROS2 rmw) 또는 ROS1 TCPROS pub/sub | ROS 통신 미들웨어 그 자체 |
| nodes | `rclcpp`/`rclpy` Node + `teleop_twist_joy`, `robot_localization`, `diagnostic_updater` | 프레임워크 + 구체 노드 둘 다 대응 |
| rtc | 대응물 없음(굳이 찾으면 `rosbridge_suite`) | DDS는 LAN 멀티캐스트 전제라 NAT 통과 P2P 개념이 ROS 생태계엔 약함 — 매핑이 가장 어색한 지점 |
| dashboard 앱 | `rviz2`/`rqt`, 웹 쪽은 `rosboard`/Foxglove Studio/webviz | |
| signaling-server 앱 | ROS1의 `roscore` | roscore도 실데이터는 안 나르고 등록·연결 협상만 함 — 구조적으로 유사. ROS2는 이 역할이 DDS 디스커버리에 흡수되어 별도 프로세스가 없음 |

가장 깔끔한 대응: device-abstraction ↔ hardware_interface, bus ↔ DDS/rmw. 가장 안 맞는 지점: rtc — ROS는 "로컬 네트워크 안에 다 있다"는 전제로 설계되어 브라우저의 NAT 통과형 P2P 문제를 다뤄본 적이 없다.
