# 구현 계획

## 목표

이 프로젝트는 ROS를 그대로 브라우저에 옮기는 대신, ROS가 맡던 역할 중 하드웨어 드라이버와 노드 간 통신 계층만 Chromium의 네이티브 API로 다시 구현하고, 마이크로초 단위 결정론이 필요한 실시간 제어는 처음부터 펌웨어 쪽에 남겨두는 것을 목표로 한다. 브라우저는 WebSerial, WebUSB, WebHID, Web Bluetooth로 하드웨어에 직접 접근하고, 노드 간 통신은 로컬에서는 BroadcastChannel과 SharedWorker로, 원격에서는 WebRTC로 처리한다. 이렇게 하면 드라이버 설치나 udev 권한 설정 없이 브라우저만 있으면 로봇을 제어할 수 있으면서도, 실시간 안전 장치는 브라우저의 상태와 무관하게 항상 동작한다.

## 저장소 구조

레포는 두 개로 나눈다. 하나는 펌웨어 레포로, C/C++ 또는 Rust와 FreeRTOS 기반 툴체인을 쓰기 때문에 나머지 코드와 언어도 빌드 시스템도 완전히 달라 처음부터 독립된 저장소로 둔다. 나머지는 npm 워크스페이스로 관리하는 하나의 웹 모노레포에 전부 넣는데, 여기 들어가는 컴포넌트들은 서로 자주 같이 바뀌는 관계라 굳이 쪼갤 이유가 없기 때문이다. (처음엔 pnpm+Turborepo로 적어뒀었는데, 실제로는 Turborepo를 설치한 적도 없고 워크스페이스 링크도 plain `npm install`로 되어 있어서 문서를 실제 상태에 맞게 정정했다 — 패키지 개수가 적은 지금 단계에서는 npm 워크스페이스만으로 충분하고, Turborepo 같은 빌드 오케스트레이션은 실제로 필요해지면 그때 추가해도 늦지 않는다.)

웹 모노레포 안에는 일곱 개의 컴포넌트가 들어간다. transport 패키지는 WebSerial, WebUSB, WebHID, Web Bluetooth를 하나의 `HardwareTransport` 인터페이스 뒤로 감추는 역할을 하고, device-abstraction 패키지는 그 위에서 원시 프레임을 속성·동작·이벤트로 감싸는 매니페스트 스키마와 로더를 담는다. bus 패키지는 같은 브라우저 안 노드끼리 통신하는 BroadcastChannel과 SharedWorker 래퍼를 제공하고, nodes 패키지는 노드 베이스 클래스와 teleop, odometry, safety monitor 같은 레퍼런스 노드를 담는다. rtc 패키지는 원격 세션을 위한 WebRTC 데이터채널과 시그널링 클라이언트를 감싸고, 여기에 실제로 배포되는 두 개의 애플리케이션인 dashboard(브라우저 HMI)와 signaling-server(Node.js WebSocket 서버)가 더해진다. 매니페스트 파일들은 코드가 아니라 데이터이므로 별도 디렉터리에 로봇별로 보관한다.

## 단계별 진행

첫 단계는 프로토콜과 매니페스트를 확정하는 것부터 시작한다. 펌웨어와 브라우저가 주고받을 프레임 포맷(SOF, LEN, CMD, PAYLOAD, CRC16, EOF)과 커맨드 어휘(SET_VELOCITY, GET_ENCODER, GET_IMU, GET_BATTERY, HEARTBEAT, ESTOP)를 이 시점에 고정해야 이후 레이어들이 흔들리지 않는다. 이 단계는 코드보다 스펙 문서 리뷰가 핵심이고, 한번 정하면 버전을 올리지 않는 한 바꾸지 않는다.

두 번째 단계에서는 전송 계층과 워치독을 붙인다. 펌웨어 레포에 실제로 1kHz 제어 루프를 도는 FreeRTOS 태스크를 올리고, 웹 모노레포에는 WebSerialTransport 하나만 구현해서 이 둘을 연결한다. 이 단계의 통과 기준은 명확한데, 로봇이 움직이는 도중 USB 케이블을 뽑았을 때 브라우저 쪽 코드가 전혀 실행되지 않아도 300밀리초 안에 양쪽 모터가 멈춰야 한다. 이게 통과되지 않으면 이후 어떤 레이어를 얹어도 안전 모델 자체가 성립하지 않으므로, 이 단계는 다음으로 넘어가기 전에 반드시 검증한다.

세 번째 단계에서는 device-abstraction, bus, nodes, dashboard 앱을 한꺼번에 붙여서 단일 탭 teleoperation을 완성한다. 레퍼런스 매니페스트를 작성해서 모터와 IMU를 속성·동작·이벤트로 노출하고, TeleopNode가 게임패드 입력을 읽어 목표 속도를 로컬 버스에 publish하면 transport를 거쳐 펌웨어까지 전달되는 흐름 전체를 연결한다. 통과 기준은 게임패드로 로봇을 실제로 몰 수 있고, 로봇마다 달라지는 부분이 매니페스트 파일 하나뿐이어야 한다는 것이다.

네 번째 단계는 여러 탭이 동시에 열려도 문제가 없도록 하드웨어 연결 자체를 SharedWorker 안으로 옮기는 작업이다. 지금까지는 탭 하나가 시리얼 포트를 독점하고 있었는데, 대시보드를 두 개 열면 둘 중 하나가 연결에 실패하는 문제가 생긴다. 이 단계에서는 새로운 컴포넌트를 추가하지 않고 transport와 bus 내부 구조만 재배치하며, 통과 기준은 대시보드 탭 두 개를 동시에 열어도 어느 쪽도 포트를 뺏기지 않는 것이다.

다섯 번째 단계에서 비로소 rtc 패키지와 signaling-server 앱이 들어온다. 원격에 있는 두 번째 브라우저가 시그널링 서버를 통해 SDP와 ICE 후보를 교환한 뒤, 실제 명령과 텔레메트리는 WebRTC 데이터채널로 피어투피어로 흐르게 만든다. 이 단계의 통과 기준은 원격 머신에서 조작했을 때의 지연시간을 실측해서 teleoperation이 가능한 수준인지 확인하는 것이고, 여기서부터 여덟 개 컴포넌트가 전부 갖춰진다.

마지막 단계는 새로운 컴포넌트 없이 signaling-server에 로봇 레지스트리 기능만 얹어 여러 대의 로봇을 한 콘솔에서 다루는 플릿 대시보드를 완성하는 것이다. 통과 기준은 매니페스트만 다른 두 번째 로봇을 등록했을 때 대시보드 코드를 전혀 건드리지 않고도 화면에 나타나는 것이다.

## 안전 모델

이 계획 전체를 관통하는 원칙은 하나다. 워치독과 E-STOP은 펌웨어가 무조건적으로 소유하고, 그 위의 어떤 레이어도 정지를 요청할 수는 있지만 보장할 수는 없다. 하트비트가 300밀리초 동안 도착하지 않으면 펌웨어가 스스로 모터를 0으로 낙지시키는데, 이 하트비트가 끊기는 이유가 탭이 얼어붙어서든, 가비지 컬렉션이 오래 걸려서든, USB 케이블이 빠져서든, 브라우저 자체가 죽어서든 펌웨어 입장에서는 전부 같은 신호로 취급된다. 그래서 2단계 이후의 모든 작업은 이 경계를 침범하지 않는 범위 안에서만 진행한다.

## Phase 1 검증 기록

두 번째 단계의 통과 기준(케이블이 뽑혀도 브라우저 코드 없이 300ms 안에 정지)을 실제 하드웨어 없이 먼저 검증했다. 실제 보드가 아직 정해지지 않았고 WebSerial도 실제 시리얼 포트나 가상 COM 포트 페어가 있어야 동작하기 때문에, 펌웨어 자리에는 Node.js로 짠 시뮬레이터(`firmware/sim`)를, 브라우저 자리에는 같은 프로토콜을 쓰는 TCP 클라이언트(`web/scripts/prototype-client.mjs`)를 두고 검증했다.

검증은 Windows 11에서 처음 실행했지만, 여기 쓰인 코드는 `node:net`, `Uint8Array`, `DataView` 같은 Node.js 표준 API만 쓰고 OS별 분기나 경로 가정이 전혀 없어서 macOS나 Linux에서도 동일한 명령으로 동일한 결과가 나와야 한다. 유일한 전제조건은 Node.js 18 이상이 설치되어 있는 것인데, ESM(`"type": "module"`)과 `node:` 접두사가 붙은 임포트를 쓰기 때문이다.

### macOS(또는 Linux)에서 재현하는 절차

두 레포(`robot-base`, `robot-os-chromium`)를 같은 부모 디렉터리 아래 clone한다. 레포를 둘로 나눈 이유 자체가 "각자 독립적으로 서 있어야 한다"는 것이었으므로 서로 상대 경로를 참조하지 않는다.

터미널 하나를 열어 펌웨어 시뮬레이터를 띄운다.
```
cd robot-base/sim
node src/index.js
```

다른 터미널을 열어 클라이언트를 실행한다.
```
cd robot-os-chromium
node scripts/prototype-client.mjs
```

포트는 기본 8765를 쓰고, 겹치면 `SIM_PORT` 환경변수로 양쪽 다 동일하게 바꿔주면 된다(`SIM_PORT=9000 node src/index.js`처럼).

### 관찰된 결과

Windows에서 실행했을 때 펌웨어 시뮬레이터 로그는 다음과 같은 순서로 찍혔다. 서버가 뜨자마자 아직 아무 연결도 없는 상태에서 300ms가 지나 첫 ESTOP이 찍혔는데(콜드 부팅 시 기본값이 안전 상태라는 뜻이라 의도된 동작), 이후 클라이언트가 붙어 워치독이 재무장되고, 속도 명령이 반영되고, 클라이언트가 인사 없이 연결을 끊은 뒤, 클라이언트 프로세스가 완전히 종료된 시점보다 뒤늦게(약 320ms 후) 두 번째 ESTOP이 독자적으로 찍혔다. macOS에서 같은 절차를 실행했을 때도 타임스탬프 값만 다를 뿐 이 순서와 간격(300ms 근방)은 동일하게 나와야 하고, 만약 다르게 나온다면 그 자체를 재현성이 깨진 버그로 취급해야 한다.

### 이번 검증으로 실제로 증명된 것

와이어 프로토콜(SOF/LEN/CMD/PAYLOAD/CRC16/EOF) 인코딩과 디코딩이 TCP 스트림처럼 바이트가 조각나서 도착하는 상황에서도 정확히 프레임 단위로 복원됐고, 하트비트 에코와 SET_VELOCITY 파싱도 정확했다. 가장 중요하게는, 워치독이 연결 상태와 완전히 무관하게 독립적으로 동작해서 클라이언트 프로세스가 이미 죽고 없는데도 스스로 ESTOP을 발동한다는 것, 즉 "펌웨어가 워치독과 ESTOP을 무조건적으로 소유한다"는 안전 모델의 핵심 가정이 최소한 이 단순화된 형태에서는 성립한다는 것을 확인했다.

### 아직 검증되지 않은 것

실제 WebSerial API는 이번에 전혀 쓰지 않았다. macOS에서도 실제 USB-시리얼 어댑터나 `socat` 등으로 만든 가상 tty 페어가 있어야 WebSerial 자체를 검증할 수 있다. 진짜 마이크로컨트롤러의 1kHz 제어 루프 타이밍, USB-CDC 드라이버 지연, 모터가 명령을 받고 실제로 멈추기까지의 물리적 반응 시간도 이번 범위 밖이다. 브라우저 탭이 실제로 프리징되거나 가비지 컬렉션이 길어질 때 하트비트 전송이 어떻게 열화되는지도 아직 확인하지 못했다. 즉 이번 검증은 "프로토콜과 워치독 로직이 논리적으로 올바른가"까지만 증명했고, "실제 브라우저와 실제 하드웨어 조합에서도 똑같이 안전한가"는 보드가 정해지고 WebSerial 연결이 실제로 붙는 다음 단계에서 다시 확인해야 한다.

## Phase 2 진행 — 실제 브라우저를 흐름에 넣기

Phase 1 검증까지는 전부 Node.js끼리의 통신이었고 실제 브라우저는 한 번도 등장하지 않았다. 이 스택의 핵심 전제(크로미움이 직접 하드웨어를 제어한다)를 증명하려면 device-abstraction이나 bus 같은 상위 레이어를 쌓기 전에 실제 브라우저 탭을 흐름에 넣는 게 먼저라고 판단해서, transport 계층부터 다시 손댔다.

가장 먼저 한 일은 시뮬레이터를 raw TCP에서 WebSocket으로 바꾼 것이다. 브라우저는 raw TCP 소켓을 아예 열 수 없기 때문에, 지금까지 검증한 프로토콜과 워치독 로직은 그대로 두고 그 위에 얹힌 전송 방식만 바꿨다. 그다음 `HardwareTransport` 인터페이스의 첫 실제 구현체인 `WebSocketTransport`를 만들었는데, 브라우저와 Node.js(22 이상) 양쪽에 다 있는 전역 `WebSocket` 클래스만 써서 만들었기 때문에 이 클래스 하나가 수정 없이 두 환경 모두에서 동작한다. Node 클라이언트도 이 인터페이스를 쓰도록 다시 짜서 기존 하트비트/ESTOP 검증이 여전히 통과하는 걸 재확인했고, 마지막으로 실제 크로미움 브라우저에서 열 수 있는 최소 테스트 페이지(`apps/dashboard/index.html`)를 추가했다. 정적 파일 서버(`scripts/serve-dashboard.mjs`)로 서빙해야 하는데, HTML을 `file://`로 직접 열면 모듈 스크립트 임포트가 CORS로 막히기 때문이다.

이 페이지는 아직 Phase 3의 정식 dashboard가 아니라 순수한 연결성 테스트용이다. 사용자가 직접 실제 브라우저로 열어서 Connect를 누르고, 연결된 상태에서 탭을 닫아보는 것으로 안전 모델이 진짜 브라우저 환경에서도 성립하는지 처음으로 확인하는 게 이 단계의 남은 할 일이다 — 이건 코드로 자동 검증할 수 없고 사람이 직접 브라우저를 조작해봐야 하는 부분이라 여기 남겨둔다.

### 첫 실사용 테스트에서 나온 예상 밖의 결과

macOS 크롬에서 처음 열어본 세션의 로그는 다음과 같은 순서였다.

```
connection opened — watchdog (re)armed
velocity set: left=0.2 right=-0.3
ESTOP — motors zeroed (no heartbeat for >300ms)
ignoring SET_VELOCITY — currently estopped   (x3)
connection closed — watchdog keeps running regardless
```

처음 설계했던 테스트는 "탭을 닫아서 크래시를 흉내내는" 시나리오였는데, 실제로 일어난 건 그게 아니었다. `velocity set`과 `ESTOP` 사이에 23초 가까이 간격이 있었고, 그 사이에도 연결(`connection closed`)은 끊기지 않은 상태였다 — 즉 WebSocket 연결은 살아있는데 자동으로 100ms마다 돌아야 할 하트비트 전송만 조용히 멈춘 것이다. 처음에는 브라우저가 백그라운드 탭의 타이머를 스로틀링하는 현상으로 추정했지만, 사용자가 그 시간 동안 계속 버튼을 클릭하고 있었다고 확인해줘서(탭이 포그라운드였고 입력도 처리되고 있었다는 뜻) 이 추정은 틀렸다. 정확한 원인은 아직 특정하지 못했다.

다만 이 원인 불명의 정지 자체가 오히려 의미 있는 결과이기도 하다. 이유가 무엇이었든 실제 브라우저 환경에서 하트비트가 예고 없이 끊겼고, 펌웨어 시뮬레이터는 그 이유를 몰라도 정확히 300ms 근방에서 스스로 ESTOP을 발동시켰다. 게다가 ESTOP 이후 재연결 없이 같은 연결로 들어온 `SET_VELOCITY` 세 번이 전부 정확히 거부됐다는 것까지 로그로 확인됐다 — "정지는 한 번으로 끝나지 않고 재무장 전까지는 계속 거부한다"는 방어 로직도 실사용 중에 처음 검증된 것이다.

원인을 특정하기 위해 `apps/dashboard/index.html`에 진단 로직을 추가했다. `Connect` 버튼 중복 클릭 가드, 매 하트비트 틱마다 실제 전송 간격을 측정해서 150ms를 넘으면 즉시 화면에 경고를 띄우는 로직, `transport.send()` 실패를 조용히 삼키지 않고 로그에 남기는 처리를 넣었다.

### 재시도 — 의도했던 시나리오로 재현 성공

진단 로직을 추가한 뒤 같은 macOS 크롬에서 다시 열어 처음 설계했던 그대로(탭을 직접 닫는 것) 테스트했다. 이번에는 경고가 한 번도 뜨지 않았고, 로그도 깨끗했다.

```
velocity set: left=0.699999988079071 right=0.6000000238418579
connection closed — watchdog keeps running regardless
ESTOP — motors zeroed (no heartbeat for >300ms)
```

`connection closed`와 그 뒤 `ESTOP` 사이 간격은 311ms로, Phase 1에서 Node 클라이언트로 검증했던 것과 같은 범위였다. 탭을 닫는 것으로 연결이 끊기고, 그 이후 어떤 브라우저 코드도 실행되지 않는 상태에서 펌웨어(시뮬레이터)가 독자적으로 정지시켰다는 걸 처음으로 실제 브라우저에서 확인했다 — Phase 1/2가 원래 세우려던 통과 기준을 실제 크로미움 탭으로 재현한 것이다.

앞서 나온 23초짜리 하트비트 정지는 이번 재시도에서 재현되지 않았다. 진단 로직도 아무 경고를 띄우지 않은 걸로 봐서 원인이 대시보드 코드 쪽의 상시적인 버그는 아닌 것으로 보이고, 특정 세션에서만 발생한 일시적 현상(맥OS 쪽 이벤트, 우연한 타이밍 등)이었을 가능성이 높다. 다만 재현되지 않았다고 원인이 규명된 건 아니므로, 진단 로직은 그대로 남겨두고 다시 나타나면 그때 클라이언트 쪽 타임스탬프로 좁혀본다.

## Phase 3 진행 — device-abstraction, bus, nodes

세 번째 단계를 시작했다. device-abstraction, bus, nodes 패키지를 채우고, 연결성 테스트였던 대시보드를 실제 게임패드 teleop으로 발전시켰다.

구현하면서 원래 아키텍처 문서를 하나 고쳤다. 처음 설계에서는 `motors.left`/`motors.right`를 독립된 액션으로 뒀는데, 실제 와이어 프로토콜의 `SET_VELOCITY`는 좌우 바퀴 목표값을 한 프레임에 같이 보낸다. 독립 액션 두 개로는 하나를 호출할 때 다른 쪽 목표값을 알 수 없는 문제가 생겨서, `drive.setVelocity(left, right)` 하나로 합쳤다 — 디퍼렌셜 드라이브 로봇은 원래 좌우 바퀴가 하나의 명령으로 같이 움직이는 게 물리적으로도 맞으므로, 원래 설계보다 더 정확한 모델이 됐다. 매니페스트(`manifests/rover.manifest.json`)도 이 구조로 다시 썼다.

구현 중 실제 버그도 하나 잡았다. `LocalBus`는 `BroadcastChannel`을 감싼 것인데, `BroadcastChannel`은 스펙상 자기 자신이 보낸 메시지를 자기 자신에게 전달하지 않는다 — 대시보드에서 `TeleopNode`와 구독 로직이 같은 `LocalBus` 인스턴스를 공유하도록 짜다 보니, 발행한 주행 명령이 그 구독자에게 영원히 도달하지 않는 상황이 될 뻔했다. Node로 직접 재현해서 확인한 뒤 `publish()`가 같은 인스턴스의 구독자도 직접 호출하도록 고쳤다. 자세한 내용은 `source-explained.md`에 남겼다.

Node로 검증 가능한 부분은 다 검증했다: `LocalBus`가 같은 인스턴스/다른 인스턴스 양쪽에서 정상 전달되는지, 그리고 대시보드와 똑같은 배선(transport → device-abstraction → bus → 구독)이 실제 펌웨어 시뮬레이터까지 정상적으로 이어지는지(속도 명령이 정확히 반영되고, 연결이 끊기면 여전히 300ms 근방에서 워치독이 독자적으로 동작하는지)를 스크립트로 직접 실행해서 확인했다.

`TeleopNode`는 `navigator.getGamepads()`와 브라우저의 Gamepad API에 의존하기 때문에 Node에서는 검증할 수 없다 — 실제 게임패드를 붙인 실제 브라우저에서 사용자가 직접 확인해야 하는 이 스택의 첫 부분이다. 통과 기준(plan.md 원문)은 "게임패드로 로봇을 실제로 몰 수 있고, 로봇마다 달라지는 부분이 매니페스트 파일 하나뿐이어야 한다"였는데, 후자(매니페스트 하나로 로봇이 바뀐다)는 코드 구조상 이미 성립하고, 전자(게임패드로 실제로 몬다)는 아직 사람이 직접 확인해야 한다.

### 실사용 테스트 — 정정: 게임패드가 아니라 수동 슬라이더였다

macOS 크롬에서 받은 로그.

```
connection opened — watchdog (re)armed
velocity set: left=0.6000000238418579 right=-0.10000000149011612
velocity set: left=0.6000000238418579 right=-0.10000000149011612
velocity set: left=-0.5 right=-0.5
velocity set: left=-0.5 right=-0.5
connection closed — watchdog keeps running regardless
ESTOP — motors zeroed (no heartbeat for >300ms)
```

처음엔 이걸 게임패드 조작 결과로 해석하고, `TeleopNode`가 50ms마다 무조건 발행하는데도 값이 500ms 넘는 간격으로만 찍히는 걸 이상 현상으로 보고 원인을 조사했다(아래 "원인 조사" 절 — 삽질 아님, 다음 절 참고). 그런데 실제로는 **이 시점에 게임패드가 아예 연결되어 있지 않았다** — 테스트한 사람이 게임패드를 집에 두고 나온 상태였다. `TeleopNode._tick()`은 `connectedGamepad()`가 `null`이면 바로 `return`하고 아무것도 발행하지 않으므로, 이 로그의 속도 변화는 전부 화면의 수동 슬라이더 + Send 버튼에서 나온 것이었다. 그렇게 보면 로그 패턴이 정확히 맞아떨어진다 — 사람이 슬라이더를 맞추고 Send를 몇 번 눌렀을 때 나오는 간격과 반복이지, 50ms 자동 폴링의 흔적이 아니다.

즉 이번 테스트가 실제로 검증한 건 "수동 조작 → bus → device-abstraction → transport → 펌웨어" 경로와, 이 새 대시보드에서도 탭을 닫으면 여전히 안전하게 정지한다는 것(240ms)이다. `TeleopNode`/Gamepad API 경로는 아직 한 번도 실제로 검증되지 않았다 — Phase 3의 게임패드 통과 기준은 여전히 열려 있다.

### 원인 조사 (결과: 버그 아님, 전제가 틀렸음)

게임패드가 없었다는 걸 알기 전에, `navigator.getGamepads()`를 목(mock)으로 바꿔서 `TeleopNode`의 틱 빈도를 Node로 직접 재봤다. 50ms 인터벌 기준 1초에 약 15번(기대치 20번과 비슷한 범위)이 나와서 인터벌/발행 로직 자체엔 문제가 없다는 걸 확인했는데, 결과적으로 이건 애초에 있지도 않았던 문제를 조사한 셈이었다. 다만 이 과정에서 만든 진단 도구는 버리지 않고 남겨뒀다 — `TeleopNode`에 `onTick(pad)` 콜백을 추가해서 게임패드 유무와 무관하게 매 폴링마다 호출되게 했고, 대시보드에는 1초마다 리셋되는 `teleop ticks/s`, `gamepad seen/s`, 원시 축 값(`raw axes`) 카운터를 추가했다. 실제 게임패드로 테스트할 때 이 숫자들로 폴링이 제대로 도는지 바로 확인할 수 있다.

### 실제 게임패드로 재시도 — 통과

실제 게임패드를 연결하고 다시 테스트한 로그를 받았다. 이번엔 패턴이 완전히 다르다.

- `velocity set`이 약 48~52ms 간격으로 17초 넘게 꾸준히 찍혔다 — 설계한 50ms 폴링과 정확히 일치.
- 값이 `0.732219398021698`, `-0.9004349112510681`처럼 연속적인 아날로그 값이다 — 0.1 단위 슬라이더로는 나올 수 없는 값들이라 실제 스틱 입력이 맞다.
- 스틱을 중립으로 놓은 구간(예: `12:52:15.251`~`15.940`, `12:52:19.440`~`21.591`, `12:52:27.199`~`29.742`)에서는 데드존대로 `left=0 right=0`이 정확히 찍혔다.
- `connection closed` 후 237ms 만에 `ESTOP`이 독자적으로 발동해서, 실제 게임패드로 조작하는 중에도 안전 모델이 성립한다는 걸 다시 확인했다.

앞서 원인 불명이라고 남겼던 현상은 정말로 게임패드 미연결이 전부였다는 게 이걸로 확정됐다 — `TeleopNode`의 폴링 로직도, 크롬의 게임패드 상태 갱신도 둘 다 문제없이 동작한다. Phase 3의 통과 기준("게임패드로 로봇을 실제로 몰 수 있고, 로봇마다 달라지는 부분이 매니페스트 파일 하나뿐")이 이제 완전히 충족됐다.

## Phase 4 진행 — SharedWorker로 연결 공유

네 번째 단계를 시작했다. 목표는 대시보드 탭을 여러 개 열어도 하드웨어 연결을 뺏고 뺏기는 일이 없게 만드는 것이다. 지금 쓰는 WebSocket 기반 시뮬레이터는 동시 연결을 여러 개 받아도 아무 문제가 없어서(진짜 시리얼 포트처럼 배타적으로 하나만 열리는 게 아니다) 이 문제가 지금 당장 눈에 보이지는 않지만, 나중에 진짜 `WebSerialTransport`로 바꿨을 때는 OS 레벨에서 포트 하나에 핸들 하나만 허용되는 게 보통이라 그때 가서 고치기보다 지금 구조를 잡아두는 쪽을 택했다.

새 컴포넌트는 추가하지 않고 `packages/bus` 안에 두 파일만 더했다. `hardware-bridge-worker.js`는 `SharedWorker`로 로드되는 스크립트 자체로, 오리진 전체에서 단 하나만 떠서 진짜 `WebSocketTransport` 하나를 독점 소유한다. 몇 개의 탭이 열려 있든 첫 번째 탭이 연결을 요청할 때만 실제로 연결하고 하트비트를 시작하며, 이후 탭들은 이미 연결된 상태를 그대로 전달받는다. `hardware-bridge-client.js`의 `HardwareBridgeClient`는 각 탭이 이 워커에 붙는 프록시인데, `WebSocketTransport`와 똑같은 모양(`connect`/`send`/`onFrame`/`onDisconnect`)을 구현해서 `createDriveDevice`를 비롯한 위쪽 코드는 전혀 손대지 않았다 — `HardwareTransport` 인터페이스를 처음부터 이렇게 두길 잘했다는 게 이번에 실제로 증명된 셈이다.

이 단계는 이 스택에서 내가(Claude) 직접 검증할 수 없는 첫 부분이다. `SharedWorker`는 Node에 아예 없는 API라 문법 검사(`node --check`) 이상은 여기서 할 수가 없다. 통과 기준은 사람이 실제 브라우저로 직접 확인해야 한다.

### 확인해주셔야 하는 것

1. 같은 대시보드 페이지를 탭 두 개로 연다. 각 탭에서 Connect를 누른다.
2. 펌웨어 시뮬레이터 로그를 본다 — `connection opened`가 **한 번만** 찍혀야 한다(Phase 3까지는 탭마다 한 번씩, 즉 두 번 찍혔을 것).
3. 한쪽 탭에서 게임패드나 슬라이더로 조작해본다 — 다른 탭의 "heartbeats sent/acked" 숫자도 같이 올라가는지 확인한다(하나의 공유 연결이므로 두 탭 다 같은 숫자를 봐야 한다).
4. 탭 하나만 닫는다 — 나머지 탭은 계속 정상 동작해야 한다(연결이 끊기면 안 된다).
5. 남은 탭도 마저 닫는다(또는 브라우저를 완전히 종료한다) — 펌웨어 시뮬레이터 로그에 ~300ms 뒤 `ESTOP`이 독자적으로 찍혀야 한다. 탭이 하나 있든 여러 개 있든 "아무도 조작하지 않게 되면 멈춘다"는 안전 모델이 그대로 유지되는지가 핵심이다.

### 실사용 테스트 — 통과

macOS 크롬에서 탭 두 개로 테스트했다. `connection opened`는 한 번만 찍혔고(2번 통과), 탭 하나를 닫아도 나머지 탭은 멀쩡했고(4번 통과), 세션 내내(약 86초) 하트비트가 끊기지 않다가 마지막 탭까지 닫은 뒤에야 `connection closed` 후 254ms 만에 `ESTOP`이 독자적으로 발동했다(5번 통과).

3번(다른 탭의 하트비트 카운트도 같이 올라가는지)은 처음엔 한쪽 탭만 안 올라가는 것처럼 보였는데, 원인은 버그가 아니라 그 탭에서 Connect를 안 눌렀던 것이었다. `HardwareBridgeClient`는 탭에서 `new HardwareBridgeClient(...)`를 실제로 생성해야(즉 Connect를 눌러야) 그 시점에 워커로 향하는 `MessagePort`가 열리고 `ports` 집합에 등록된다 — 다른 탭이 이미 연결돼 있다고 자동으로 끼워지는 게 아니다. Connect를 양쪽 탭에서 다 누르니 정상적으로 두 탭의 카운트가 같이 올라갔다.

이건 의도적으로 그대로 두기로 했다. 지금은 WebSocket이라 사용자 제스처 없이도 연결할 수 있지만, 나중에 진짜 `WebSerialTransport`로 바뀌면 `navigator.serial.requestPort()` 자체가 사용자 클릭 없이는 열리지 않기 때문에, "탭마다 Connect를 눌러야 한다"는 지금의 흐름이 그때 가서도 그대로 맞는 UI다.

Phase 4의 통과 기준이 전부 충족됐다.

## Phase 5 진행 — 원격 세션 (WebRTC), 시뮬레이터로 먼저

다섯 번째 단계를 시작했다. 실제 두 번째 머신이나 하드웨어 없이, 지금까지 쓰던
펌웨어 시뮬레이터를 그대로 두고(한 줄도 안 바꿨다 — WebRTC 계층은 전적으로
브라우저 ↔ 브라우저 사이에 들어간다) 한 대에서 창 두 개로 전 구간을 붙였다.

### 새로 만든 것

- `apps/signaling-server` — SDP/ICE만 중계하는 최소 WebSocket 서버. robot id 하나당
  "방" 하나를 두고, host 하나 + operator 여럿이 붙는다. `signal` 메시지는 방의
  다른 피어에게 그대로 전달만 하고 내용은 보지 않는다. `list`는 Phase 6 플릿
  레지스트리의 씨앗으로 넣어뒀고 아직 아무 데서도 안 쓴다. 포트 `SIGNALING_PORT`(9770).
- `packages/rtc` — 세 조각이다.
  - `SignalingClient` — 시그널링 서버의 피어 쪽 절반. 브라우저와 Node(22+ 전역
    `WebSocket`) 양쪽에서 그대로 돈다.
  - `RtcTransport` — operator 쪽. `WebSocketTransport`와 **똑같은 모양**
    (`connect`/`send`/`onFrame`/`onDisconnect`)을 구현해서, 대시보드가 원격 로봇을
    로컬 로봇과 완전히 같은 코드로 몬다 — transport 생성자만 바뀐다. Phase 4의
    `HardwareBridgeClient`와 같은 수법을 한 홉 더 밖으로 민 것이다.
  - `RtcHostBridge` — host 쪽. operator의 데이터채널과 펌웨어로 향하는 WebSocket
    사이를 거의 그대로 지나가는 바이트 파이프. **하트비트를 절대 보내지 않고**,
    ESTOP도 명령 검사도 안 한다. 펌웨어가 authority이고 이건 중계일 뿐이다.
- `apps/dashboard/host.html` — host 브리지 콘솔. 조작 UI 없이 연결된 operator 목록과
  양방향 바이트 카운트, 이벤트 로그만 보여준다.
- `apps/dashboard/index.html`에 모드 선택 추가 — `local (shared worker)` /
  `operator (remote WebRTC)`. transport 위쪽(`createDriveDevice`, `LocalBus`,
  `TeleopNode`)은 두 모드가 동일하고, 하트비트 배선만 갈린다(아래).
- `packages/transport/src/websocket-transport.js`에 `close()` 추가 — 인터페이스의
  선택적 부분. `prototype-client.mjs`는 여전히 크래시 흉내를 위해 일부러 안 부르고,
  host 브리지는 operator 세션 하나를 깔끔히 끝낼 때 쓴다.

### 하트비트는 operator가 소유한다 (설계 결정)

Phase 4에서는 SharedWorker가 하트비트를 보냈다. Phase 5 원격에서 그 모델을 그대로
두면, operator의 랩탑이 얼거나 네트워크가 끊겨도 host가 대신 하트비트를 계속 보내서
로봇이 마지막 속도로 계속 굴러가는 — 안전 모델을 정면으로 어기는 — 상황이 된다.
그래서 원격에서는 하트비트가 반드시 operator 링크를 타야 한다. `RtcHostBridge`는
하트비트를 전혀 안 보내고 순수 중계만 하며, operator 대시보드가 (Phase 3 이전처럼)
`startHeartbeat(rtcTransport)`를 직접 돈다. operator가 끊기면 host로 하트비트가 안
오고, host는 아무것도 전달 안 하고, 펌웨어 워치독이 ~300ms 뒤 스스로 모터를 0으로
만든다 — USB 케이블을 뽑은 것과 같은 보장이, 이번엔 WebRTC 링크 너머에서.

부수 효과: operator가 아무도 없으면 시뮬레이터는 하트비트를 못 받아 estop 상태로
가만히 있는다. "아무도 몰지 않으면 멈춰 있다"가 맞으므로 의도된 동작이다.

### operator 세션 하나당 펌웨어 WebSocket 하나

시뮬레이터는 콜드 부팅 / ESTOP 이후의 안전 상태에서 **새 연결이 들어올 때만**
빠져나온다(하트비트만으로는 재무장 안 함 — Phase 1에서 확인된 동작). host가 펌웨어로
향하는 WebSocket 하나를 세션 내내 붙들고 있으면, 한 번 estop된 뒤 나중에 operator가
붙어 하트비트를 보내도 시뮬레이터는 계속 estop 상태다. 그래서 `RtcHostBridge`는
operator의 데이터채널이 열릴 때마다 펌웨어로 **새 WebSocket**을 열고, operator가
떠나면 닫는다. "operator가 연결돼 있다" ↔ "펌웨어로 향하는 소켓이 열려 있다"를
1:1로 묶어서 연결 → 주행 → 해제 → 재연결이 올바르게 돈다.

실제 로봇 위의 host가 이렇게 동작해야 하는지(아니면 명시적 재무장 명령을 둬야
하는지)는 아직 정하지 않은 항목으로 남긴다.

### 지금까지 검증된 것 / 안 된 것

- `scripts/signaling-smoke.mjs` — 시그널링 서버 + `SignalingClient`를 브라우저 없이
  돌리는 스모크 테스트. 자체적으로 시그널링 서버를 띄우고, hello→ready,
  peer-joined/peer-left, host↔operator 양방향 `signal` 중계(순서 보존 포함), 같은
  robot에 두 번째 host 거부, `list` 응답을 확인한다. 8개 체크 전부 통과.
  `RTCPeerConnection`은 Node에 없어서 여기서는 안 건드린다.
- `RtcTransport` / `RtcHostBridge` / 두 HTML은 `node --check` 문법 검사만 했다.
  실제 WebRTC 협상, 데이터채널, 그리고 Phase 5의 진짜 통과 기준(원격 조작 시
  지연시간 실측, operator가 끊겼을 때 300ms ESTOP)은 사람이 실제 브라우저로
  확인해야 한다.

### 사람이 확인해줘야 하는 것

한 대에서:

1. 터미널 세 개 — `firmware/sim`(`npm start`), `web` 시그널링 서버
   (`node apps/signaling-server/src/index.js`), `web` 정적 서버
   (`node scripts/serve-dashboard.mjs`).
2. `http://localhost:5173/apps/dashboard/host.html`를 열고 Start bridge.
   시뮬레이터 로그에는 아직 아무 `connection opened`가 없어야 한다(host는 아직
   펌웨어에 연결 안 함).
3. 다른 창에서 `http://localhost:5173/apps/dashboard/index.html`를 열고 모드를
   operator로, Connect. 이제 시뮬레이터 로그에 `connection opened`가 한 번 찍히고,
   host 페이지의 바이트 카운터가 양방향으로 올라가야 한다.
4. 슬라이더나 게임패드로 몬다 — 시뮬레이터 로그에 `velocity set`이 찍혀야 한다
   (operator → 데이터채널 → host → 펌웨어).
5. operator 창을 **닫는다** — 시뮬레이터 로그에 `connection closed` 후 ~300ms 뒤
   `ESTOP`이 독자적으로 찍혀야 한다. 이게 Phase 5의 핵심: 조작하던 쪽이 사라지면
   WebRTC 홉을 하나 거쳤든 아니든 펌웨어가 스스로 멈춘다.
6. operator 창을 다시 열고 Connect — 다시 `connection opened`가 찍히고(새 세션 =
   새 펌웨어 소켓 = 재무장) 정상 주행돼야 한다.

지연시간과 NAT 통과는 이 한 대짜리 테스트로는 의미 있게 못 잰다(둘 다 host
candidate, RTT ~0). 실제 두 번째 머신이 붙는 시점에 다시 확인한다.

## 타겟 하드웨어 확정 — ROAS Former 2.0

레퍼런스 하드웨어가 **ROAS Former 2.0**(https://roas.co.kr/former/)으로 정해졌다.
회사 보유 유닛이고 OS는 Debian(기본은 Ubuntu)이다. 자세한 배경과 펌웨어 경계가
어떻게 바뀌는지는 `firmware/plan.md`의 "타겟 하드웨어 확정" 절에 적었고, 여기서는
`web` 레포에 미치는 영향만 정리한다.

- **온보드 컴퓨터가 완전한 x86-64 PC다** (i5-8265U, 8GB). MCU가 아니다. 즉
  Chromium을 **로봇 위에서 직접** 돌릴 수 있다. Phase 5의 `host.html` 브리지를
  Former의 Debian에서 띄우고, operator 랩탑이 WebRTC로 진짜 원격 접속하는 구성이
  자연스럽다.
- **베이스 연결은 RS232 @ 115200** (기본형; 업그레이드형은 Ethernet + USB). 지금까지
  "실제 보드가 정해지면"으로 미뤄둔 `WebSerialTransport`의 대상이 바로 이 RS232
  링크다. Debian Chromium이 `/dev/ttyUSB*`(또는 `/dev/ttyS*`)를 WebSerial로 연다.
  전제: 유저가 `dialout` 그룹, **apt Chromium**(snap은 confinement가 `/dev/tty*`를
  막아서 안 됨), secure context(localhost는 OK), origin별 포트 권한 영속화.
- **와이어 프로토콜을 우리 것 → ROAS 것으로 교체한다.** 지금 `packages/transport`의
  `frame.js`/`commands.js`(SOF/LEN/CMD/CRC16, `SET_VELOCITY`/`HEARTBEAT`/`ESTOP`)는
  처음부터 placeholder였다. `former_hardware_interface` 소스에서 실제 프로토콜을
  역추출해 `../former-motor-protocol.md`(루트 스크래치 레포)에 정리했다 — 결론은
  **Former 구동부가 Roboteq 모터 컨트롤러**이고, 115200 8N1 라인 프로토콜(`\r` 종결,
  `_` 다중명령 구분)에 Roboteq ASCII 명령(`!G 1 n_!G 2 n`, `!MG`, `!EX`, `?C`, `?V`
  …)을 그대로 말하면 된다. `packages/transport`는 이 Roboteq 코덱으로,
  `firmware/sim`은 Roboteq 에뮬레이터로 다시 쓴다. `HardwareTransport` 인터페이스
  위쪽(device-abstraction, bus, nodes, rtc, dashboard)은 프로토콜이 바뀌어도 안
  건드리는 게 이 구조를 둔 이유다 — 이번이 그 가정의 첫 실전 시험이 된다.
- **워치독은 이미 존재한다.** Roboteq 내장 시리얼 워치독(`RWD`, 기본 1000ms)이
  시리얼 침묵 시 브라우저와 무관하게 모터를 세운다 — 안전 모델의 "펌웨어 워치독"이
  이것이다. 우리 heartbeat(100ms 주기)는 이 1000ms 안에서 뭔가를 계속 보내기만 하면
  된다. E-STOP = `!EX`. (실기에서 `~RWD`가 0이 아닌지 확인 필요 —
  `../former-motor-protocol.md` 참고.)

### 코드 반영 완료

`packages/transport/src/{frame,commands}.js`(바이너리 프레임)를 삭제하고
`roboteq.js`(코덱: `encodeCommand` + `cmd` 빌더 + `RoboteqDecoder`)를 넣었다.
`WebSocketTransport`는 `onMessage`(파싱된 응답) + `onRaw`(중계용 미가공 바이트)로
바뀌었고, `startHeartbeat`는 `!B 3 1`을 보낸다. `createDriveDevice`는 `enable()`
(`!MG`) / `estop()`(`!EX`) / `setVelocity(l, r)`(정규화 ±1 → `!G ±1000`)을 노출한다.
`HardwareTransport` 인터페이스 위쪽(`bus`, `nodes`, `rtc`, 대시보드 배선)은
`onFrame`→`onMessage` 이름만 따라 바뀌고 로직은 그대로 — 프로토콜 전면 교체가
인터페이스 경계 안에서 끝났다(이 구조를 둔 목적의 첫 실전 시험, 통과).

매니페스트는 `manifests/former.manifest.json`(robot `former-01`), 대시보드/호스트
페이지도 그에 맞춰 갱신. `RtcHostBridge`는 이제 `onRaw`로 양방향 미가공 중계만
한다. 브라우저 없이 도는 검증: `scripts/roboteq-smoke.mjs`(7/7),
`scripts/signaling-smoke.mjs`(8/8), `prototype-client.mjs` 크래시 회귀 모두 통과.
WebRTC 실동작은 여전히 사람이 브라우저로 확인해야 한다(Phase 5 절 "사람이
확인해줘야 하는 것", robot id만 `former-01`로).

### WebSerialTransport + host.html 하드웨어 모드

`WebSerialTransport`(`navigator.serial` @ 115200, `/dev/ttyMOTOR`, `WebSocketTransport`와
동일 인터페이스) 구현. Chromium 전용이라 `node --check`만, 실검증은 로봇에서.
`RtcHostBridge` 생성자를 `firmwareUrl` 문자열 → `makeTransport` 팩토리 +
`initCommands`로 바꿔서 같은 브리지가 시뮬레이터(WebSocket)와 실기(WebSerial)를 다
앞단다. `host.html`에 Controller 라디오(simulator / hardware) 추가 — hardware면
WebSerial 팩토리 + 매니페스트의 `drive.commands.init`(`^ECHOF 1`/`!R 2`/`!AC`/`!DC`)을
컨트롤러 연결 직후 100ms 간격으로 전송(bring-up만, 주행 아님). init 시퀀스는
시뮬레이터에서도 무해(전부 `+` ack) — `roboteq-smoke.mjs`에 체크 추가(이제 8/8).

이걸로 "로봇에 설치" 경로의 코드 공백은 메워졌다.

### 오프라인 설치 번들 (`deploy/`)

회사망에서 로봇이 외부 접근이 안 돼서, 노트북에서 번들을 만들어 LAN(scp)이나 USB로
옮겨 설치하는 방식으로 갔다. 스택 자체는 빌드가 없고 외부 런타임 의존성이 `ws`
하나뿐(zero transitive)이라 번들의 무거운 부분은 Node 런타임과 Chromium `.deb`뿐이다.

- `deploy/make-offline-bundle.sh` (노트북) — 스택 소스 복사 + `ws` 벤더링 + Node
  linux tarball 다운로드 + `debian:<suite>` 컨테이너로 Chromium `.deb` 클로저 받기
  → `web/dist/former-webstack-offline-<날짜>.tar.gz`. `--suite`/`--arch`로 로봇
  Debian에 맞추고, Docker 없으면 `--skip-chromium`.
- `deploy/bundle/install.sh` (로봇, 오프라인) — `/opt/former-webstack`에 설치,
  `.deb` dpkg, `dialout` 그룹, udev 규칙, Chromium managed policy
  (`SerialAllowUsbDevicesForUrls` — 재부팅마다 포트 피커 안 뜨게), 데스크톱 kiosk
  autostart + 서버용 systemd 유닛(옵션).
- `deploy/bundle/kiosk-launch.sh` — signaling + 정적 서버 띄우고 Chromium을
  `host.html`(hardware 모드)로 kiosk 실행.
- 로봇에서 `signaling-server`도 같이 돈다(로봇↔노트북 LAN 전제). operator는
  노트북에서 `http://<robot-ip>:5173/.../index.html` operator 모드로 붙는다 —
  데이터채널 전용 WebRTC는 plain http에서도 동작.

스테이징 로직과 스택이 스테이지 복사본에서 정상 기동하는지는 Windows에서 검증했다.
Node/Chromium 실제 다운로드와 로봇 설치는 로봇 Debian 버전이 확정되면 마무리.

남은 건 실기 검증(`?FID` 모델, `~RWD` ≠ 0, 채널 좌우/부호)과 ROS `former_bringup`
정지뿐.

### 매니페스트 주도로 전환

`createDriveDevice`에서 Roboteq 하드코딩(`!G`/`!MG`/`!EX`, 스케일)을 걷어내고
매니페스트의 `drive.commands`(템플릿 문자열) + `drive.channels` + `drive.scale`에서
읽도록 했다. `setVelocity` 템플릿은 `${ch.left}`/`${v.right}` 같은 자리를 채우는
방식(`"!G ${ch.left} ${v.left}_!G ${ch.right} ${v.right}"`). 결과적으로 **같은 와이어
프로토콜을 쓰는 다른 디퍼렌셜 베이스 = 새 매니페스트 파일 하나, `drive-device.js`
무수정.** 코드에 남긴 건 인코딩(코덱 소관)과 [-1,1] 정규화 규약뿐. 두 Node
스크립트도 실제 매니페스트 파일을 읽도록 바꿔 dogfooding. 다른 *와이어 프로토콜*이
생기면 `manifest.transport.kind`로 코덱을 고르는 지점만 추가하면 된다.
- **`GET_ENCODER` 리드백**(지금 매니페스트에 매핑만 있고 미구현)은 Former가 실제로
  주는 엔코더/오도메트리 데이터로 채운다.
- LIDAR(SICK TiM571, Ethernet/CoLa)와 RealSense D435(USB/UVC)는 브라우저로 붙이기
  훨씬 어려워 당분간 범위 밖. 디퍼렌셜 베이스가 첫 교체 대상이다.

### `npm test` 배선 + RWD 타이밍 검사를 회귀 테스트로 강화

`roboteq-smoke.mjs`/`signaling-smoke.mjs`는 계속 잘 동작했지만 둘 다 `node scripts/...mjs`로 사람이 직접 실행해야만 돌아갔다 — `npm test` 자체가 정의되어 있지 않아서, CI에 연결하려 해도 걸 곳이 없었다. 루트 `package.json`에 `"test": "node scripts/roboteq-smoke.mjs && node scripts/signaling-smoke.mjs"`를 추가해 표준 진입점으로 만들었다.

RWD 워치독 체크도 다시 봤다: 기존 코드는 `!G`를 마지막으로 보낸 뒤 `RWD_MS + 300ms`(400+300=700ms)를 무조건 기다렸다가 로그에 `"RWD: no serial command"`가 있는지 문자열 포함 여부만 확인했다. 이 방식의 문제는, 워치독이 원래보다 훨씬 느려지는 회귀(예: 400ms짜리가 690ms로 늘어남)가 생겨도 여전히 700ms 창 안에서는 로그에 그 문자열이 찍히므로 테스트가 계속 PASS를 낸다는 것 — "언젠가 멈췄는가"만 검증하고 "제때 멈췄는가"는 검증하지 못했다.

지금은 침묵을 시작한 시각(`silenceStart`)부터 20ms 간격으로 폴링하며 그 로그 줄이 새로 찍히는 순간까지의 실제 경과 시간(`rwdLatencyMs`)을 측정하고, `RWD_MS`~`RWD_MS+150ms`(150ms는 Node 타이머 지터 + 시뮬레이터 자체 20ms tick + 폴링 간격을 감안한 여유) 범위 안에 드는지를 어서션한다. 측정값을 PASS/FAIL 메시지에 항상 출력해서(`측정 448ms`처럼) 통과하더라도 수치 추이를 눈으로 볼 수 있게 했다. 로컬에서 5회 연속 실행 결과 447~452ms로 매우 안정적이었다(오차 5ms 이내) — 150ms 여유는 실제 지터 대비 넉넉하면서도, 2배 이상 느려지는 진짜 회귀는 확실히 잡아낸다.

체크 개수는 그대로 8/8(로직만 교체, 새 체크 추가 아님). 이 테스트가 검증하는 건 여전히 "시뮬레이터가 문서화된 RWD 사양대로 동작하고 JS 쪽 연동(코덱/transport/createDriveDevice)이 정확한가"까지다 — 실제 Former 2.0의 Roboteq 컨트롤러가 진짜로 이 타이밍을 지키는지는 실기 검증 전까지는 알 수 없다는 한계는 그대로 남아 있다.

## 아직 정하지 않은 것

정확한 RS232 커맨드 어휘와 Former 베이스의 명령 타임아웃(워치독) 동작은
`former_hardware_interface` 소스를 읽어 확정한다. 매니페스트 스키마의 버전 관리 방식과, rtc 계층에서 WebRTC가 NAT 통과에 실패했을 때의 폴백(로컬 네트워크 안에서는 signaling-server를 거치는 일반 WebSocket 릴레이로 대체하는 방안을 고려 중)도 아직 세부 설계가 남아 있다.

Phase 5에서 새로 생긴 미결 항목: (1) 실제 로봇 위의 host 브리지가 시뮬레이터처럼 "operator 세션마다 펌웨어 연결을 새로 여는" 모델로 가야 하는지, 아니면 펌웨어에 명시적 재무장(arm) 명령을 두고 host는 연결을 계속 유지하는 모델로 가야 하는지. (2) operator가 여럿 붙었을 때 누가 하트비트를 소유하고 누가 조작 권한을 갖는지(지금은 operator 1명 전제) — 조작권 중재(handover) 설계. (3) TURN 서버 운영 방식.
