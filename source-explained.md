# 소스 코드 설명

이 레포에 실제로 존재하는 코드 파일들을 하나씩 설명한다. 아키텍처 구상이 아니라 "지금 여기 있는 코드가 정확히 무엇을 하는가"를 남기는 문서다. Phase 5까지 와서 이제 빈 자리는 없다 — `transport`, `bus`, `device-abstraction`, `nodes`, `rtc`, `apps/dashboard`, `apps/signaling-server`, `scripts/`가 전부 실제 코드다. 다만 `rtc`의 `RTCPeerConnection` 의존 부분(`RtcTransport`, `RtcHostBridge`)과 두 개의 새 HTML은 아직 `node --check` + 사람 검증 단계이고, `signaling-server`와 `SignalingClient`는 `scripts/signaling-smoke.mjs`로 자동 검증된다.

## package.json

루트에 있는 npm 워크스페이스 정의(`workspaces: ["packages/*", "apps/*"]`). 원래 plan.md엔 pnpm+Turborepo라고 적어뒀었는데 실제로 쓰인 건 plain `npm install`이었어서 문서를 정정했다 — `node_modules/@ros-chromium/*`에 각 패키지로의 심볼릭 링크가 생겨 있다(`.gitignore`로 제외, `package-lock.json`은 커밋). 다만 지금까지 만든 코드는 전부 상대 경로(`../../packages/transport/src/...`)로 서로를 불러오지, `@ros-chromium/transport` 같은 패키지 이름으로 import하는 곳은 아직 없다 — 그래서 이 워크스페이스 링크가 없어도 지금까지의 모든 코드는 동일하게 동작한다.

## packages/transport/src/roboteq.js

Former 2.0 베이스(Roboteq 모터 컨트롤러)의 실제 시리얼 프로토콜 코덱. 처음 프로토타입이 쓰던 `frame.js`(SOF/LEN/CMD/CRC16 바이너리) + `commands.js`(CMD enum)를 대체한다 — 그 둘은 타겟 로봇이 정해지기 전 placeholder였고 삭제됐다. 프로토콜 전체는 루트의 `former-motor-protocol.md`에 정리돼 있다.

- `encodeCommand(line)` — 끝에 `\r` 없는 명령 문자열을 바이트로.
- `cmd` — 명령/쿼리 문자열 빌더 모음(`motorGo`/`estop`/`keepAlive`/`motorCommand`/`queryRuntime` 등). 호출부가 프로토콜 문서처럼 읽히게 하는 얇은 템플릿.
- `RoboteqDecoder` — 스트리밍 라인 디코더. `push(bytes)`마다 `\r` 단위로 잘라 `{type:'ack',ok}`(`+`/`-`), `{type:'reply',key,values,raw}`(`KEY=v:v`), `{type:'line',raw}`(그 외)로 파싱한 배열을 낸다.

`firmware/sim/src/roboteq.js`와 바이트 단위로 동일해야 한다(의도된 복제 — `former-motor-protocol.md`가 공통 소스). 상세는 펌웨어 레포 `source-explained.md`에도 같은 내용으로 있다.

## packages/transport/src/websocket-transport.js

시뮬레이터 상대 테스트용 `HardwareTransport` 구현체(실제 보드는 같은 모양의 `WebSerialTransport`가 맡을 자리). 전역 `WebSocket`(브라우저 + Node 22+)만 써서 브라우저 탭에서도 Node 스크립트에서도 수정 없이 돈다 — `apps/dashboard/index.html`과 `scripts/prototype-client.mjs`가 같은 클래스를 쓴다.

`RoboteqDecoder` 하나를 들고 있다가, `onmessage`로 바이트가 들어오면 (1) 먼저 raw 바이트 그대로 `onRaw` 콜백들에 넘기고 (2) 디코더에 밀어넣어 나온 파싱 메시지를 `onMessage` 콜백들에 넘긴다. `onRaw`는 파싱하지 않고 바이트만 중계해야 하는 곳(Phase 5 `RtcHostBridge`)을 위한 것이다.

`connect()`는 `onopen`에서 resolve, `onclose`에서 disconnect 콜백. `send()`는 `ws.send(frame)`. `close()`는 인터페이스의 선택적 부분 — 정상 종료 핸드셰이크를 보내는 의도적 종료다. "크래시처럼 인사 없이 끊기"를 재현하려면 `close()`를 부르지 않고 프로세스/페이지를 죽여야 하고(`prototype-client.mjs`가 그렇게 한다), `RtcHostBridge`는 operator 세션을 깔끔히 끝낼 때 쓴다.

## packages/transport/src/heartbeat.js

`startHeartbeat(transport, options)`. 100ms마다 `!B 3 1`(Roboteq keepalive bool — 레퍼런스 ROS 드라이버가 매 제어 사이클 보내고, 온보드 안전 스크립트도 이걸 본다)을 보낸다. Former의 Roboteq는 시리얼이 ~1초 조용하면 RWD 워치독이 모터를 세우므로, 그 안에서 뭐라도 계속 보내는 게 이 루프의 역할이다. 매 틱마다 직전 전송과의 간격(`performance.now()`)을 재서 `gapWarnMs`(기본 150ms)를 넘으면 `onGap` — 원인 불명의 23초 정지(plan.md "Phase 2 진행") 이후 남긴 진단. `onSend`/`onSendError`, `stop()`.

## packages/transport/src/index.js

`roboteq.js`, `websocket-transport.js`, `heartbeat.js`를 재수출한다. 파일 맨 위 주석에 `HardwareTransport` 인터페이스 모양(`connect`/`send`/`onMessage`/`onDisconnect`, 선택적 `onRaw`/`close`)과 와이어 프로토콜이 Roboteq라는 것, 다음 TODO가 `navigator.serial` 기반 `WebSerialTransport`(`/dev/ttyMOTOR` @ 115200)라는 걸 적어뒀다. 그 위 코드(device-abstraction, nodes, dashboard)는 transport 구현을 갈아끼워도 안 건드리는 게 이 인터페이스를 둔 이유다.

## packages/bus/src/local-bus.js

`LocalBus` — `BroadcastChannel` 위에 토픽 이름으로 필터링하는 pub/sub을 얹은 것. 구현하면서 실제로 걸려 넘어진 버그가 하나 있어서 그대로 적어둔다.

`BroadcastChannel`은 스펙상 **자기 자신이 보낸 메시지를 자기 자신에게는 전달하지 않는다** — 같은 채널 이름을 쓰는 *다른* `BroadcastChannel` 인스턴스에만 전달된다(같은 페이지 안에서 만든 다른 인스턴스여도 상관없이). 처음 짠 버전은 `publish()`가 `postMessage()`만 호출했는데, `apps/dashboard/index.html`에서 `TeleopNode`와 구독 로직이 **같은 `LocalBus` 인스턴스**를 공유하도록 짜다 보니 발행한 명령이 자기 자신의 구독자에게 영원히 전달되지 않는 상황이 됐다. Node로 직접 재현해서 확인했다(`bc.postMessage()` 후 같은 인스턴스의 `onmessage`가 500ms 동안 안 옴 → 확정). 지금 버전은 `publish()`가 `postMessage()`와 함께 `_dispatch()`를 직접 호출해서 같은 인스턴스의 구독자에게도 즉시 전달하도록 고쳤다 — 그 결과 같은 인스턴스를 공유하든(대시보드처럼) 노드마다 별도 인스턴스를 만들든(원래 의도했던 구조) 둘 다 정상 동작한다. 두 경우 다 Node 스크립트로 실제 검증했다.

## packages/bus/src/hardware-bridge-worker.js, hardware-bridge-client.js

Phase 4(plan.md)에서 추가한, 하드웨어 연결을 탭 여러 개가 공유하는 부분. `hardware-bridge-worker.js`는 일반 모듈이 아니라 `new SharedWorker(url, { type: 'module' })`로 로드되는 워커 스크립트 자체다 — 이 스크립트 하나가 오리진 전체에서 단 하나만 실행되고, 탭이 몇 개든 전부 이 하나의 인스턴스에 `MessagePort`로 연결된다.

워커 안에는 진짜 `WebSocketTransport` 인스턴스가 딱 하나만 존재한다. 첫 번째 탭이 `{type:'connect'}`를 보내면 그때 실제로 `transport.connect()`를 호출하고 `startHeartbeat()`도 그때 한 번만 시작한다(`ensureConnected()`가 진행 중인 연결 시도를 프라미스로 캐싱해서 여러 탭이 동시에 연결을 요청해도 실제 연결 시도는 한 번만 일어나게 막는다). 이후 새로 열리는 탭은 이미 연결되어 있으면 그 상태를 바로 돌려받는다. 컨트롤러에서 오는 파싱된 메시지(`{type:'message', msg}`)와 연결 상태, keepalive 진단(간격 경고, 전송 실패)은 전부 `broadcast()`로 연결된 모든 포트에 똑같이 전달된다.

`hardware-bridge-client.js`의 `HardwareBridgeClient`는 탭 쪽에서 이 워커에 붙는 프록시다. 의도적으로 `WebSocketTransport`와 같은 모양(`connect()`, `send()`, `onMessage()`, `onDisconnect()`)을 그대로 구현했다 — `packages/transport`의 `HardwareTransport` 인터페이스를 처음 만들 때 "구현체를 바꿔 끼워도 위 레이어는 안 건드린다"는 게 목적이었는데, 이번이 그 목적이 실제로 쓰인 첫 사례다. `createDriveDevice(transport, manifest)`는 `WebSocketTransport`를 받든 `HardwareBridgeClient`를 받든 코드 한 줄도 안 바뀐다.

한 가지 의도적인 단순화: `send()`는 워커에 메시지를 posting만 하고 바로 resolve되는 fire-and-forget이다. 매 명령(특히 100ms마다 나가는 keepalive)마다 워커의 응답을 기다리는 왕복을 만들지 않기 위한 선택이고, 대신 전송이 실패하면 `onSendError` 콜백으로 비동기 통지된다.

이 두 파일은 `SharedWorker`, `self.onconnect` 같은 브라우저 전용 API에 의존해서 Node에서는 실행할 수 없다(Node에 `SharedWorker`가 없다). `node --check`로 문법만 확인했고, 실제 동작(탭 두 개를 열었을 때 펌웨어 로그에 `connection opened`가 한 번만 찍히는지, 탭을 하나씩 닫아도 나머지 탭은 멀쩡한지, 모든 탭이 닫히고 나서야 워치독이 독자적으로 정지시키는지)은 이 스택에서 처음으로 사람이 실제 브라우저로 직접 검증해야 하는 부분이다.

## packages/device-abstraction/src/manifest.js, drive-device.js

`loadManifest(url)`은 `fetch()`로 매니페스트 JSON을 받아오는 것뿐이다. 스키마 검증은 없고, fetch 자체가 실패하면 에러를 던진다. 브라우저 전용이라(HTTP fetch를 가정) Node에서 직접 쓸 일은 없다.

`createDriveDevice(transport, manifest)`는 좌우 바퀴를 단일 "drive" 액션으로 모델링한다 — Roboteq `!G`가 한 명령에 두 채널을 같이 싣고, 디퍼렌셜 드라이브는 애초에 좌우가 하나의 명령으로 움직이는 게 물리적으로 맞다. 세 가지를 노출한다: `enable()`(`!MG` — 모터는 연결/ESTOP/워치독 이후 항상 비활성으로 시작하므로 연결 후 한 번 불러야 함), `estop()`(`!EX`), `setVelocity(left, right)`. `left`/`right`는 정규화된 [-1, 1](TeleopNode와 수동 슬라이더가 내는 값)이고, `±1`을 `±1000` Roboteq 단위(= ±200 바퀴 RPM)로 매핑해 `!G <chL> n_!G <chR> n`을 만들어 보낸다. 채널 번호는 매니페스트의 `drive.channels`에서 온다. 실제 단위(바퀴 rad/s → RPM ×60/2π → ÷200×1000)로 가는 경로는 `former-motor-protocol.md`에 있고, 기존 teleop 파이프라인과 맞추려고 지금은 정규화 값을 그대로 쓴다. 속도 리드백(`drive.readback.encoder = "?C"`)은 `?C` 카운트 델타로 바퀴 속도를 뽑는 폴 루프가 아직 없어서 TODO.

## packages/nodes/src/teleop-node.js

`TeleopNode` — Gamepad API를 폴링해서 버스에 주행 명령을 발행한다. Gamepad API에는 "축 값이 바뀌었다" 이벤트가 아예 없어서(연결/해제 이벤트만 있음), `navigator.getGamepads()`를 타이머로 주기적으로 읽는 것 말고는 방법이 없다. 기본 50ms 간격.

왼쪽 스틱의 Y축(위로 밀면 음수가 나오므로 부호를 뒤집는다)을 전진/후진으로, X축을 회전으로 써서 아케이드 믹싱(`left = forward + turn`, `right = forward - turn`)으로 좌우 바퀴 목표 속도를 만든다. 스틱을 살짝만 건드려도 값이 남는 걸 막기 위해 ±0.08 미만은 0으로 죽이는 데드존을 뒀다.

`navigator.getGamepads()`를 직접 목(mock)으로 바꿔서(Node 22의 전역 `navigator`는 읽기 전용이라 `globalThis.navigator.getGamepads = ...`로 메서드만 얹는 식) Node로 틱 빈도를 확인했더니 50ms 인터벌 기준 1초에 약 15번(기대치 20번과 비슷한 범위) 정상적으로 돌았다 — 로직 자체는 문제없다는 뜻이다.

이후 실제 크롬 테스트에서 같은 값이 50ms가 아니라 500ms 이상 간격으로만 로그에 찍히는 현상이 나와서 원인을 조사했지만, 결과적으로 그 테스트는 애초에 게임패드가 연결되지 않은 상태에서 진행된 것이었다 — `connectedGamepad()`가 `null`을 반환하니 `_tick()`이 매번 조기 리턴했을 뿐, 로그에 찍힌 값은 화면 수동 슬라이더에서 온 것이었다. 즉 버그가 아니라 전제가 틀린 조사였다. 그래도 조사 과정에서 만든 `onTick(pad)` 콜백(매 폴링마다, 게임패드가 없어도 호출)과 대시보드의 `teleop ticks/s`/`gamepad seen/s`/`raw axes` 카운터는 남겨뒀는데, 실제로 도움이 됐다 — 진짜 게임패드로 재시도했을 때는 `velocity set`이 정확히 ~50ms 간격으로, 스틱을 중립에 두면 데드존대로 `left=0 right=0`이 찍히는 걸로 폴링과 발행 로직이 설계대로 동작한다는 게 확인됐다(`plan.md` "실제 게임패드로 재시도 — 통과" 참고). Phase 3의 게임패드 통과 기준은 이걸로 충족됐다.

## apps/signaling-server/src/index.js

WebRTC 시그널링만 하는 최소 WebSocket 서버. robot id 하나가 "방" 하나이고, 방마다 host 하나 + operator 여럿이 붙는다. 클라이언트가 보내는 메시지는 세 가지뿐이다 — `hello`(role + robot id, host면 manifest도 옵션), `signal`(상대에게 그대로 넘길 불투명한 `data` 블롭), `list`(알려진 로봇 목록, Phase 6 씨앗). 서버는 `signal`의 `data` 안을 절대 들여다보지 않는다. `hello`에 대한 응답 `ready`에는 이미 방에 있던 피어 목록을 실어줘서, 새로 들어온 쪽이 WebRTC offer를 지금 보낼지 상대가 올 때까지 기다릴지 판단하게 한다. 같은 robot에 두 번째 host가 붙으면 거부한다. 포트는 `SIGNALING_PORT`(기본 9770). 로봇 명령·텔레메트리는 여기를 절대 지나가지 않는다 — 핸드셰이크가 끝나면 데이터채널로 P2P로 흐른다.

## packages/rtc/src/signaling-client.js

`apps/signaling-server`의 피어 쪽 절반. WebSocket 하나를 감싸서 서버의 JSON 메시지를 콜백(`onPeerJoined`/`onPeerLeft`/`onSignal`/`onClose`)으로 바꾼다. 전역 `WebSocket`만 써서 브라우저와 Node(22+) 양쪽에서 그대로 돈다 — 브라우저 대시보드와 `scripts/signaling-smoke.mjs`가 같은 클래스를 쓴다. `connect()`는 `ready`가 올 때 `{ peerId, peers }`로 resolve한다. WebRTC가 뭔지는 전혀 모르고, SDP/ICE를 나르는 `data`는 불투명하게 전달만 한다.

## packages/rtc/src/rtc-transport.js

원격 세션의 operator 쪽. `WebSocketTransport`와 똑같은 `connect`/`send`/`onMessage`/`onDisconnect` 모양을 구현해서, 대시보드가 원격 로봇을 로컬 로봇과 같은 코드로 몬다 — transport 생성자만 바뀐다(`HardwareBridgeClient`가 Phase 4에서 한 것과 같은 수법, 한 홉 더 밖). operator가 능동적인 쪽이라 `RTCPeerConnection`과 데이터채널을 만들고 offer를 보낸다(host는 answer만; 역할이 고정이라 perfect-negotiation 안 함). host가 이미 방에 있으면 바로 offer하고, 없으면 `peer-joined`를 기다린다. 들어오는 데이터채널 메시지는 `RoboteqDecoder`를 거쳐 파싱된 메시지로 나온다(`WebSocketTransport`와 같은 디코딩 경로 재사용).

이 transport는 **자체 keepalive가 없다**. operator 대시보드가 `startHeartbeat(rtcTransport)`를 직접 돈다 — keepalive(`!B 3 1`)가 operator 자신의 링크를 타야, operator가 얼거나 끊겼을 때 Roboteq RWD 워치독이 ~1초 뒤 모터를 0으로 만든다. host가 대신 보내면 이 보장이 깨진다. ICE 서버는 지금 공용 STUN 하나뿐이고, 크로스-NAT용 TURN과 LAN WebSocket 릴레이 폴백은 아직 미결(`plan.md` "아직 정하지 않은 것").

## packages/rtc/src/rtc-host-bridge.js

원격 세션의 host 쪽. 펌웨어에 닿을 수 있는 머신(여기서는 시뮬레이터에 닿는 머신)에서 돈다. 방에 나타나는 operator마다 WebRTC offer에 answer하고, 그 operator의 데이터채널과 펌웨어로 향하는 WebSocket 사이를 **양방향 모두 raw 바이트 그대로** 지나가는 파이프가 된다 — `transport.onRaw`를 쓰지 파싱된 `onMessage`를 안 쓴다. **keepalive를 절대 안 보내고**, ESTOP도 트래픽 파싱도 안 한다. 유효성 판정은 펌웨어의 디코더가 authority.

operator 세션 하나당 펌웨어 WebSocket을 새로 연다. 펌웨어는 새 연결이 들어올 때(또는 operator가 보낸 `!MG`)만 정지 상태에서 재무장하기 때문에, "operator 연결됨" ↔ "펌웨어 소켓 열림"을 1:1로 묶어야 연결→주행→해제→재연결이 올바르게 돈다. operator가 떠나면 그 세션의 펌웨어 소켓을 `close()`로 깔끔히 닫는다. 실제 로봇 host가 이렇게 가야 하는지는 미결 항목.

`RTCPeerConnection` 의존이라 Node에서는 못 돌린다 — `node --check`만 했고 실제 동작은 사람이 브라우저로 검증(`plan.md` Phase 5 "사람이 확인해줘야 하는 것").

## packages/rtc/src/index.js

`signaling-client.js`, `rtc-transport.js`, `rtc-host-bridge.js`를 재수출하고, 파일 맨 위 주석에 세 조각의 역할과 "무엇이 Node에서 돌고 무엇이 브라우저 전용인지"를 적어뒀다.

## apps/dashboard/host.html

host 브리지 콘솔. 조작 UI가 없다 — 시그널링/펌웨어/robot id 입력 필드, Start bridge 버튼, 연결된 operator 목록(각 행에 펌웨어 링크 상태와 양방향 바이트 카운트), 이벤트 로그가 전부다. `SignalingClient({role:'host'})` + `RtcHostBridge`를 만들어 `start()`할 뿐이고, 실제 중계 로직은 전부 `RtcHostBridge` 안에 있다. 페이지 상단에 "이 페이지는 로봇을 몰지 않고 keepalive도 안 보낸다"를 명시해뒀다.

## manifests/former.manifest.json

Former 2.0을 기술하는 매니페스트. `base: "roboteq"`, `transport`(kind/baud/device), `drive`(채널 매핑 `{left:1, right:2}`, `maxWheelRpm`, `countsPerRev`, 바퀴 반지름/축거, 그리고 `readback`에 `?C`/`?V 2`/`?A`/`?T 1`/`?FF`/`?DI` 쿼리 매핑). `drive.channels`만 `createDriveDevice`가 실제로 읽고, 나머지는 아직 문서/향후 용도. 이전 이름은 `rover.manifest.json`(가상의 rover-01)이었다.

## scripts/prototype-client.mjs

`WebSocketTransport` + `startHeartbeat` + `createDriveDevice`를 실제로 가져다 쓰는(dogfooding) Node.js 클라이언트. 브라우저 탭 하나가 하는 걸 그대로 한다.

1. `WebSocketTransport`로 시뮬레이터에 연결.
2. `?FID` 한 번 보내 컨트롤러 응답 로그, `drive.enable()`(`!MG`).
3. `startHeartbeat`로 100ms 간격 `!B 3 1` keepalive 시작.
4. 연결 150ms 시점에 `drive.setVelocity(0.5, 0.5)` → `!G 1 500_!G 2 500`.
5. 연결 350ms 시점에 `heartbeat.stop()` 후 `transport.close()`를 **부르지 않고** `process.exit(0)` — 크래시 흉내.

`close()`를 일부러 안 부르는 게 핵심이다. 정상 종료는 핸드셰이크가 있지만 크래시는 그럴 겨를이 없다. 이 스크립트는 정지를 스스로 확인하지 않는다(프로세스가 죽어버리니까) — 시뮬레이터 로그에서 이 프로세스가 죽고 `SIM_RWD_MS`(기본 1초) 뒤 찍히는 `motors zeroed — RWD: ...`로 확인한다. 빠르게 보려면 시뮬레이터를 `SIM_RWD_MS=300`으로 띄운다.

## scripts/roboteq-smoke.mjs

Roboteq 라인 프로토콜 전 구간을 브라우저 없이 검증하는 스모크 테스트 — `WebSocketTransport` + `roboteq.js` 코덱 + 시뮬레이터의 Roboteq 에뮬레이터 + `createDriveDevice`, 그리고 load-bearing한 RWD 워치독. 자체적으로 시뮬레이터를 짧은 `SIM_RWD_MS`로 자식 프로세스로 띄우고 7개 체크(`?FID` 응답, `!MG` 전엔 `!G` 무시, `!MG` 후 엔코더 증가, `+` ack, `!EX` 후 `FF=16`/`DI=0` 래치, 침묵 시 RWD 정지)를 돌린 뒤 PASS/FAIL, 실패 시 non-zero 종료. `node scripts/roboteq-smoke.mjs`.

## scripts/signaling-smoke.mjs

`apps/signaling-server`와 `SignalingClient`를 브라우저 없이 검증하는 스모크 테스트. `RTCPeerConnection`은 Node에 없으므로 여기서는 안 건드리고, "랑데부"만 본다 — hello→ready, peer-joined/peer-left, 한 피어의 `signal` 블롭이 다른 피어에게 그대로(순서 보존 포함) 나오는지. SDP/ICE 페이로드는 가짜 문자열이다(서버가 안 들여다보므로). 자체적으로 던져버릴 포트에 시그널링 서버를 자식 프로세스로 띄우고, 8개 체크를 돌린 뒤 PASS/FAIL을 찍고 실패 시 non-zero로 종료한다. `node scripts/signaling-smoke.mjs`.

## scripts/serve-dashboard.mjs

Node 내장 `http`/`fs`만 쓰는, 의존성 없는 정적 파일 서버. `apps/dashboard/index.html`이 `<script type="module">`로 `packages/transport/src/*.js`를 상대 경로로 불러오는데, HTML 파일을 `file://`로 그냥 열면 크로미움이 모듈 스크립트 로딩을 CORS로 막아버리기 때문에 `http://`로 서빙해야 한다. `web/` 디렉터리 전체를 루트로 서빙하고, 확장자에 따라 `Content-Type`을 최소한으로만 맞춰준다(`.js`/`.mjs`는 `text/javascript`로 지정 — 이게 틀리면 브라우저가 모듈로 인식하지 않는다). 기본 포트는 5173, `DASHBOARD_PORT` 환경변수로 바꿀 수 있다.

## apps/dashboard/index.html

Phase 1~2에서는 연결성 테스트용 최소 페이지였는데, 이번 Phase 3에서 실제 게임패드 teleop 대시보드로 바뀌었다. 연결 흐름과 하트비트 진단 로직(중복 Connect 가드, 전송 간격 경고, 전송 실패 로그)은 이전 버전에서 그대로 가져왔고, `startHeartbeat` 헬퍼로 옮겨서 코드는 오히려 줄었다.

Connect를 누르면 순서대로: `loadManifest()`로 `/manifests/former.manifest.json`을 받아오고 → transport를 열고 → `createDriveDevice(transport, manifest)`로 주행 디바이스를 만들고 → 연결 후 `drive.enable()`(`!MG`)로 모터를 활성화하고 → `LocalBus`를 하나 만들어서 `former-01/drive/cmd_vel` 토픽을 구독해 들어오는 `{left, right}`를 그대로 `drive.setVelocity()`에 넘기고 → `TeleopNode`를 그 버스에 붙여 시작한다. 화면의 수동 슬라이더도 같은 토픽에 `publish`하는 것 말고는 하지 않는다 — 즉 게임패드든 수동 조작이든 마지막에 `drive.setVelocity()`를 호출하는 코드는 딱 한 군데(버스 구독 콜백)뿐이다.

Phase 4에서는 `WebSocketTransport`를 직접 여는 대신 `HardwareBridgeClient`를 연다는 점만 바뀌었다. `createDriveDevice`는 두 경우 모두 같은 방식으로 호출되므로(둘 다 `HardwareTransport` 모양을 구현하기 때문에) 이 위의 배선 설명은 그대로 유효하다. 달라진 건 하트비트 관련 UI 갱신 방식이다 — 이전엔 이 탭이 직접 `startHeartbeat`를 불렀지만, 지금은 그 호출 자체가 워커 안으로 옮겨갔기 때문에 이 탭은 `transport.onHeartbeatSent`/`onHeartbeatGap`/`onHeartbeatError`로 워커가 보내주는 소식을 받아 화면 숫자만 갱신한다.

게임패드 연결 상태는 `window`의 `gamepadconnected`/`gamepaddisconnected` 이벤트로 표시한다 — 참고로 Gamepad API 스펙상 실제로 게임패드의 버튼이나 스틱을 한 번 조작해야 브라우저가 그 게임패드를 "연결됨"으로 인식하는 브라우저가 많다(연결만 해두고 가만히 있으면 안 뜰 수 있음).

Phase 5에서 모드 선택(`local` / `operator`)이 추가됐다. `local`은 지금까지대로 `HardwareBridgeClient`(SharedWorker)를 쓰고, `operator`는 `SignalingClient` + `RtcTransport`를 써서 원격 host 브리지에 WebRTC로 붙는다. transport 위쪽 배선(`createDriveDevice`, `drive.enable()`, `LocalBus`, `TeleopNode`, 버스 구독)은 두 모드가 완전히 동일하고, 갈리는 건 keepalive뿐이다 — `local`은 워커가 keepalive를 소유하므로 이 탭은 `onHeartbeatSent`/`onHeartbeatGap`/`onHeartbeatError`를 듣기만 하고, `operator`는 `RtcTransport`에 keepalive가 없으므로 이 탭이 (Phase 4 이전처럼) `startHeartbeat`를 직접 돈다. 이게 load-bearing한 선택인 이유는 `plan.md` Phase 5 "하트비트는 operator가 소유한다" 절에 적어뒀다. 컨트롤러가 각 명령에 보내는 `+` ack는 `transport.onMessage`에서 세어 "acked" 카운터로 쓴다(keepalive가 100ms마다 하나씩 `+`를 받으므로 왕복 liveness 지표). `teardown()`은 두 모드 공통 정리(keepalive·teleop·bus 정지, transport null)를 한군데로 모은 것이다.

이 페이지가 존재하는 이유는 단 하나, 지금까지의 모든 검증이 Node.js끼리의 통신이었고 실제 브라우저 탭은 이 흐름에 한 번도 들어온 적이 없었기 때문이다. `scripts/serve-dashboard.mjs`로 서빙한 뒤 `http://localhost:5173/apps/dashboard/index.html`로 열어야 한다.

### 실사용 테스트 히스토리

첫 실사용 테스트에서 탭이 포그라운드에 있고 사용자가 계속 버튼을 누르고 있었는데도 하트비트가 23초 가까이 끊겨 ESTOP이 발동하는 원인 불명의 현상이 있었다. 진단 로직(중복 Connect 가드, 전송 간격 경고, 전송 실패 로그, sent/ack 카운터 분리)을 추가한 뒤 같은 방식(탭을 직접 닫는 것)으로 재시도했을 때는 경고 없이 깨끗하게 재현됐다 — `connection closed` 후 311ms 만에 ESTOP. 처음 현상은 재현되지 않아 일회성 현상으로 기록해뒀다.

두 번째 실사용 테스트는 처음엔 게임패드 조작으로 오해했지만, 실제로는 게임패드가 연결되지 않은 상태에서 수동 슬라이더만으로 진행된 것이었다 — `TeleopNode`는 정확히 설계대로 아무것도 발행하지 않았을 뿐이다. 원인 조사 과정에서 만든 `teleop ticks/s`, `gamepad seen/s`, `raw axes` 실시간 카운터(1초마다 리셋)는 결과적으로 필요 없었던 조사의 부산물이지만, 실제 게임패드 테스트 때 그대로 쓸 수 있어서 남겨뒀다. `teleop-node.js`의 `onTick` 콜백이 매 폴링마다 불리는 걸 그대로 세는 것뿐이라, 이 숫자가 낮게 나오면 `_tick()` 자체가 안 도는 것이고, 숫자는 정상인데 `raw axes`가 안 바뀌면 크롬의 게임패드 상태 갱신이 문제라는 걸 구분할 수 있다.

전체 타임라인과 로그는 `plan.md`의 "Phase 2 진행"과 "실사용 테스트 — 게임패드 조작 확인" 절에 있다.
