# 소스 코드 설명

이 레포에 실제로 존재하는 코드 파일들을 하나씩 설명한다. 아키텍처 구상이 아니라 "지금 여기 있는 코드가 정확히 무엇을 하는가"를 남기는 문서다. `packages/rtc`와 `apps/signaling-server`만 아직 `package.json`뿐인 빈 자리이고, 나머지(`transport`, `bus`, `device-abstraction`, `nodes`, `scripts/`, `apps/dashboard`)는 전부 실제로 동작하는 코드다.

## package.json

루트에 있는 npm 워크스페이스 정의(`workspaces: ["packages/*", "apps/*"]`). 원래 plan.md엔 pnpm+Turborepo라고 적어뒀었는데 실제로 쓰인 건 plain `npm install`이었어서 문서를 정정했다 — `node_modules/@ros-chromium/*`에 각 패키지로의 심볼릭 링크가 생겨 있다(`.gitignore`로 제외, `package-lock.json`은 커밋). 다만 지금까지 만든 코드는 전부 상대 경로(`../../packages/transport/src/...`)로 서로를 불러오지, `@ros-chromium/transport` 같은 패키지 이름으로 import하는 곳은 아직 없다 — 그래서 이 워크스페이스 링크가 없어도 지금까지의 모든 코드는 동일하게 동작한다.

## packages/transport/src/commands.js

펌웨어 레포의 `firmware/sim/src/commands.js`와 바이트 하나 다르지 않게 동일한 파일이다. `HEARTBEAT`(0x01), `SET_VELOCITY`(0x02), `ESTOP`(0x03) 세 개만 실제로 쓰이고, `GET_ENCODER`/`GET_IMU`/`GET_BATTERY`는 번호만 예약되어 있다. 두 레포가 서로의 파일을 참조하지 않기 때문에 일부러 복제해뒀고, 프로토콜이 바뀌면 두 파일을 사람이 직접 맞춰야 한다.

## packages/transport/src/frame.js

이것도 `firmware/sim/src/frame.js`와 동일한 코드다. `SOF|LEN|CMD|PAYLOAD|CRC16|EOF` 프레임을 만드는 `encodeFrame()`과, 바이트가 조각나서 들어와도 완성된 프레임 단위로 복원해주는 `FrameDecoder`가 들어있다. 상세한 동작(재동기화 로직, CRC 알고리즘)은 펌웨어 레포의 `source-explained.md`에 이미 적어뒀고, 여기서는 완전히 같은 내용이라 반복하지 않는다.

## packages/transport/src/websocket-transport.js

`HardwareTransport` 인터페이스의 첫 실제 구현체. 브라우저와 Node.js(22 이상) 양쪽에 다 있는 전역 `WebSocket` 클래스 하나만 써서 만들었기 때문에, 이 파일은 수정 없이 실제 브라우저 탭 안에서도, Node 스크립트 안에서도 똑같이 동작한다 — `apps/dashboard/index.html`과 `scripts/prototype-client.mjs`가 정확히 같은 이 클래스를 가져다 쓴다.

내부적으로 `FrameDecoder` 인스턴스를 하나 들고 있다가, `onmessage`로 바이너리 메시지(`ArrayBuffer`)가 들어올 때마다 `Uint8Array`로 바꿔서 디코더에 밀어넣고, 완성된 `{cmd, payload}` 프레임이 나오면 등록된 콜백들에 전달한다. WebSocket은 이미 메시지 단위로 배달해주기 때문에 TCP 때처럼 바이트가 쪼개져 오는 걱정은 없지만, 프레임 포맷 자체(CRC 검증 등)는 어차피 나중에 WebSerial(진짜 바이트 스트림)에서도 그대로 써야 하므로 굳이 다른 디코딩 경로를 따로 만들지 않고 `FrameDecoder`를 그대로 재사용했다.

`connect()`는 `WebSocket`을 열고 `onopen`에서 resolve하는 프라미스를 반환하고, `onclose`가 오면 등록된 disconnect 콜백들을 부른다. `send()`는 그냥 `ws.send(frame)`이다 — 표준 `close()` 메서드는 항상 정상 종료 핸드셰이크를 보내기 때문에, "크래시처럼 인사 없이 끊기"를 재현하려면 `close()`를 부르지 않고 프로세스/페이지 자체를 죽여야 한다(아래 `prototype-client.mjs` 설명 참고).

## packages/transport/src/heartbeat.js

`startHeartbeat(transport, options)`. 원래는 `prototype-client.mjs`와 `apps/dashboard/index.html`에 거의 똑같은 하트비트 루프가 두 벌 있었는데, 실제 teleop 대시보드가 세 번째 자리가 되는 시점에 그냥 하나로 뽑아냈다. 100ms마다 하트비트 프레임을 보내고, 매 틱마다 직전 전송 이후 얼마나 지났는지(`performance.now()` 차이)를 재서 `gapWarnMs`(기본 150ms)를 넘으면 `onGap` 콜백을 부른다 — 이게 지난번 원인 불명의 23초 하트비트 정지 이후 추가한 진단 로직이다. `onSend`는 전송이 성공할 때마다, `onSendError`는 실패할 때마다 불린다. `stop()`으로 멈춘다.

## packages/transport/src/index.js

`frame.js`, `commands.js`, `websocket-transport.js`, `heartbeat.js`를 재수출(`export *`)한다. 파일 맨 위 주석에 `HardwareTransport` 인터페이스의 모양을 적어뒀고, 지금은 `WebSocketTransport`가 그 모양을 구현한 첫 번째이자 유일한 구현체다. `WebSerialTransport`, `WebUSBTransport` 등은 여전히 TODO로 남아있다 — 실제 보드가 정해지면 `WebSocketTransport`와 똑같은 모양으로 하나 더 추가하면 되고, 그 위의 코드(device-abstraction, nodes, dashboard)는 건드릴 필요가 없는 게 이 인터페이스를 둔 이유다.

## packages/bus/src/local-bus.js

`LocalBus` — `BroadcastChannel` 위에 토픽 이름으로 필터링하는 pub/sub을 얹은 것. 구현하면서 실제로 걸려 넘어진 버그가 하나 있어서 그대로 적어둔다.

`BroadcastChannel`은 스펙상 **자기 자신이 보낸 메시지를 자기 자신에게는 전달하지 않는다** — 같은 채널 이름을 쓰는 *다른* `BroadcastChannel` 인스턴스에만 전달된다(같은 페이지 안에서 만든 다른 인스턴스여도 상관없이). 처음 짠 버전은 `publish()`가 `postMessage()`만 호출했는데, `apps/dashboard/index.html`에서 `TeleopNode`와 구독 로직이 **같은 `LocalBus` 인스턴스**를 공유하도록 짜다 보니 발행한 명령이 자기 자신의 구독자에게 영원히 전달되지 않는 상황이 됐다. Node로 직접 재현해서 확인했다(`bc.postMessage()` 후 같은 인스턴스의 `onmessage`가 500ms 동안 안 옴 → 확정). 지금 버전은 `publish()`가 `postMessage()`와 함께 `_dispatch()`를 직접 호출해서 같은 인스턴스의 구독자에게도 즉시 전달하도록 고쳤다 — 그 결과 같은 인스턴스를 공유하든(대시보드처럼) 노드마다 별도 인스턴스를 만들든(원래 의도했던 구조) 둘 다 정상 동작한다. 두 경우 다 Node 스크립트로 실제 검증했다.

## packages/bus/src/hardware-bridge-worker.js, hardware-bridge-client.js

Phase 4(plan.md)에서 추가한, 하드웨어 연결을 탭 여러 개가 공유하는 부분. `hardware-bridge-worker.js`는 일반 모듈이 아니라 `new SharedWorker(url, { type: 'module' })`로 로드되는 워커 스크립트 자체다 — 이 스크립트 하나가 오리진 전체에서 단 하나만 실행되고, 탭이 몇 개든 전부 이 하나의 인스턴스에 `MessagePort`로 연결된다.

워커 안에는 진짜 `WebSocketTransport` 인스턴스가 딱 하나만 존재한다. 첫 번째 탭이 `{type:'connect'}`를 보내면 그때 실제로 `transport.connect()`를 호출하고 `startHeartbeat()`도 그때 한 번만 시작한다(`ensureConnected()`가 진행 중인 연결 시도를 프라미스로 캐싱해서 여러 탭이 동시에 연결을 요청해도 실제 연결 시도는 한 번만 일어나게 막는다). 이후 새로 열리는 탭은 이미 연결되어 있으면 그 상태를 바로 돌려받는다. 펌웨어에서 오는 프레임(하트비트 에코 포함)과 연결 상태, 하트비트 진단(간격 경고, 전송 실패)은 전부 `broadcast()`로 연결된 모든 포트에 똑같이 전달된다.

`hardware-bridge-client.js`의 `HardwareBridgeClient`는 탭 쪽에서 이 워커에 붙는 프록시다. 의도적으로 `WebSocketTransport`와 같은 모양(`connect()`, `send()`, `onFrame()`, `onDisconnect()`)을 그대로 구현했다 — `packages/transport`의 `HardwareTransport` 인터페이스를 처음 만들 때 "구현체를 바꿔 끼워도 위 레이어는 안 건드린다"는 게 목적이었는데, 이번이 그 목적이 실제로 쓰인 첫 사례다. `createDriveDevice(transport, manifest)`는 `WebSocketTransport`를 받든 `HardwareBridgeClient`를 받든 코드 한 줄도 안 바뀐다.

한 가지 의도적인 단순화: `send()`는 워커에 메시지를 posting만 하고 바로 resolve되는 fire-and-forget이다. 매 프레임(특히 100ms마다 나가는 하트비트)마다 워커의 응답을 기다리는 왕복을 만들지 않기 위한 선택이고, 대신 전송이 실패하면 `onSendError` 콜백으로 비동기 통지된다 — 실패가 어느 `send()` 호출 때문이었는지 정확히 짚어주지는 못한다.

이 두 파일은 `SharedWorker`, `self.onconnect` 같은 브라우저 전용 API에 의존해서 Node에서는 실행할 수 없다(Node에 `SharedWorker`가 없다). `node --check`로 문법만 확인했고, 실제 동작(탭 두 개를 열었을 때 펌웨어 로그에 `connection opened`가 한 번만 찍히는지, 탭을 하나씩 닫아도 나머지 탭은 멀쩡한지, 모든 탭이 닫히고 나서야 워치독이 독자적으로 정지시키는지)은 이 스택에서 처음으로 사람이 실제 브라우저로 직접 검증해야 하는 부분이다.

## packages/device-abstraction/src/manifest.js, drive-device.js

`loadManifest(url)`은 `fetch()`로 매니페스트 JSON을 받아오는 것뿐이다. 스키마 검증은 없고, fetch 자체가 실패하면 에러를 던진다. 브라우저 전용이라(HTTP fetch를 가정) Node에서 직접 쓸 일은 없다.

`createDriveDevice(transport, manifest)`가 이번 단계에서 아키텍처 문서의 원래 설계를 실제로 고친 지점이다. 처음 설계 문서에서는 `motors.left`/`motors.right`를 각각 독립적인 액션으로 그렸는데, 실제 와이어 프로토콜의 `SET_VELOCITY`는 좌우 바퀴 목표값을 **한 프레임에 같이** 담는다(`research.md` 참고). 그래서 "왼쪽 바퀴"와 "오른쪽 바퀴"를 독립된 액션으로 두면 하나를 호출할 때 다른 쪽 목표값을 모르는 문제가 생긴다. 구현 단계에서 이걸 발견하고 매니페스트와 코드를 둘 다 "drive"라는 단일 액션으로 고쳤다 — 디퍼렌셜 드라이브 로봇은 애초에 물리적으로 좌우 바퀴가 하나의 명령으로 같이 움직이는 게 맞으므로, 오히려 원래 아키텍처보다 실제 하드웨어를 더 정직하게 반영한다. `setVelocity(leftMps, rightMps)` 하나가 `SET_VELOCITY` 프레임 하나를 만들어 `transport.send()`한다. 속도 리드백(`velocity: "GET_ENCODER"`)은 펌웨어/시뮬레이터 양쪽 다 아직 안 만들어서 여기도 TODO다.

## packages/nodes/src/teleop-node.js

`TeleopNode` — Gamepad API를 폴링해서 버스에 주행 명령을 발행한다. Gamepad API에는 "축 값이 바뀌었다" 이벤트가 아예 없어서(연결/해제 이벤트만 있음), `navigator.getGamepads()`를 타이머로 주기적으로 읽는 것 말고는 방법이 없다. 기본 50ms 간격.

왼쪽 스틱의 Y축(위로 밀면 음수가 나오므로 부호를 뒤집는다)을 전진/후진으로, X축을 회전으로 써서 아케이드 믹싱(`left = forward + turn`, `right = forward - turn`)으로 좌우 바퀴 목표 속도를 만든다. 스틱을 살짝만 건드려도 값이 남는 걸 막기 위해 ±0.08 미만은 0으로 죽이는 데드존을 뒀다.

`navigator.getGamepads()`를 직접 목(mock)으로 바꿔서(Node 22의 전역 `navigator`는 읽기 전용이라 `globalThis.navigator.getGamepads = ...`로 메서드만 얹는 식) Node로 틱 빈도를 확인했더니 50ms 인터벌 기준 1초에 약 15번(기대치 20번과 비슷한 범위) 정상적으로 돌았다 — 로직 자체는 문제없다는 뜻이다.

이후 실제 크롬 테스트에서 같은 값이 50ms가 아니라 500ms 이상 간격으로만 로그에 찍히는 현상이 나와서 원인을 조사했지만, 결과적으로 그 테스트는 애초에 게임패드가 연결되지 않은 상태에서 진행된 것이었다 — `connectedGamepad()`가 `null`을 반환하니 `_tick()`이 매번 조기 리턴했을 뿐, 로그에 찍힌 값은 화면 수동 슬라이더에서 온 것이었다. 즉 버그가 아니라 전제가 틀린 조사였다. 그래도 조사 과정에서 만든 `onTick(pad)` 콜백(매 폴링마다, 게임패드가 없어도 호출)과 대시보드의 `teleop ticks/s`/`gamepad seen/s`/`raw axes` 카운터는 남겨뒀는데, 실제로 도움이 됐다 — 진짜 게임패드로 재시도했을 때는 `velocity set`이 정확히 ~50ms 간격으로, 스틱을 중립에 두면 데드존대로 `left=0 right=0`이 찍히는 걸로 폴링과 발행 로직이 설계대로 동작한다는 게 확인됐다(`plan.md` "실제 게임패드로 재시도 — 통과" 참고). Phase 3의 게임패드 통과 기준은 이걸로 충족됐다.

## packages/{rtc}/package.json, apps/{signaling-server}/package.json

이 두 곳만 아직 `package.json` 하나씩만 있는 빈 자리다. `description` 필드에 앞으로 뭘 담을지와 `plan.md`의 어느 단계(Phase 5)에서 채워질지를 적어뒀다.

## manifests/rover.manifest.json

레퍼런스 로버 하나를 기술하는 매니페스트. 처음 버전은 `motors.left`/`motors.right`를 독립 액션으로 뒀는데, 위 `drive-device.js` 설명에 적은 이유로 `drive.setVelocity`/`drive.velocity` 하나로 합쳤다. `velocity`(`GET_ENCODER` 리드백)는 아직 펌웨어/시뮬레이터에 구현이 없어서 매핑만 있고 실제로 쓰이진 않는다.

## scripts/prototype-client.mjs

`WebSocketTransport`와 `startHeartbeat`, `createDriveDevice`를 실제로 가져다 쓰는 Node.js 클라이언트. 처음엔 `net` 소켓을 직접 다뤘다가 `WebSocketTransport`로, 이번엔 하트비트 루프와 속도 명령까지 각각 `startHeartbeat`/`createDriveDevice`로 옮겨서 이 레포가 만든 추상화를 실제로 그대로 사용하도록(dogfooding) 계속 바꿔왔다.

동작 순서는 다음과 같다.

1. `WebSocketTransport`로 펌웨어(시뮬레이터)에 연결한다.
2. `startHeartbeat`로 100ms 간격 하트비트 전송을 시작한다.
3. 연결 150ms 시점에 `createDriveDevice(transport, manifest).setVelocity(0.5, 0.5)`를 한 번 호출한다.
4. 연결 350ms 시점에 `heartbeat.stop()`을 부르고, `transport._ws.close()`를 부르지 않은 채로 `process.exit(0)`을 호출해 프로세스를 그대로 죽인다.

`close()`를 일부러 부르지 않는 게 핵심이다. WebSocket의 정상 종료는 양쪽이 종료 프레임을 주고받는 핸드셰이크인데, 실제 탭 크래시는 그런 인사를 할 겨를이 없다. 그래서 프로세스를 그냥 죽여서 OS가 강제로 소켓을 정리하게 만드는 쪽이 "크래시"를 더 정직하게 흉내낸다. 이 스크립트는 ESTOP이 실제로 발동했는지 스스로 확인하지 않는데, 4번 단계에서 프로세스가 완전히 죽어버리기 때문에 애초에 확인할 방법이 없고, 그게 이 테스트의 요점이다 — 정지가 실제로 일어났는지는 펌웨어(시뮬레이터) 쪽 로그에서, 이 프로세스가 죽고 한참 뒤의 타임스탬프로 확인해야 한다. 자세한 실행 결과와 재현 절차는 `plan.md`의 "Phase 1 검증 기록" 절에 남겨뒀다.

## scripts/serve-dashboard.mjs

Node 내장 `http`/`fs`만 쓰는, 의존성 없는 정적 파일 서버. `apps/dashboard/index.html`이 `<script type="module">`로 `packages/transport/src/*.js`를 상대 경로로 불러오는데, HTML 파일을 `file://`로 그냥 열면 크로미움이 모듈 스크립트 로딩을 CORS로 막아버리기 때문에 `http://`로 서빙해야 한다. `web/` 디렉터리 전체를 루트로 서빙하고, 확장자에 따라 `Content-Type`을 최소한으로만 맞춰준다(`.js`/`.mjs`는 `text/javascript`로 지정 — 이게 틀리면 브라우저가 모듈로 인식하지 않는다). 기본 포트는 5173, `DASHBOARD_PORT` 환경변수로 바꿀 수 있다.

## apps/dashboard/index.html

Phase 1~2에서는 연결성 테스트용 최소 페이지였는데, 이번 Phase 3에서 실제 게임패드 teleop 대시보드로 바뀌었다. 연결 흐름과 하트비트 진단 로직(중복 Connect 가드, 전송 간격 경고, 전송 실패 로그)은 이전 버전에서 그대로 가져왔고, `startHeartbeat` 헬퍼로 옮겨서 코드는 오히려 줄었다.

Connect를 누르면 순서대로: `loadManifest()`로 `/manifests/rover.manifest.json`을 받아오고 → `WebSocketTransport`를 열고 → `createDriveDevice(transport, manifest)`로 주행 디바이스를 만들고 → `LocalBus`를 하나 만들어서 `rover-01/drive/cmd_vel` 토픽을 구독해 들어오는 `{left, right}`를 그대로 `drive.setVelocity()`에 넘기고 → `TeleopNode`를 그 버스에 붙여 시작하고 → `startHeartbeat`를 시작한다. 화면의 수동 슬라이더도 같은 토픽에 `publish`하는 것 말고는 하지 않는다 — 즉 게임패드든 수동 조작이든 마지막에 `drive.setVelocity()`를 호출하는 코드는 딱 한 군데(버스 구독 콜백)뿐이다. 페이지 상단에 이 구조를 그대로 설명해뒀다.

Phase 4에서는 `WebSocketTransport`를 직접 여는 대신 `HardwareBridgeClient`를 연다는 점만 바뀌었다. `createDriveDevice`는 두 경우 모두 같은 방식으로 호출되므로(둘 다 `HardwareTransport` 모양을 구현하기 때문에) 이 위의 배선 설명은 그대로 유효하다. 달라진 건 하트비트 관련 UI 갱신 방식이다 — 이전엔 이 탭이 직접 `startHeartbeat`를 불렀지만, 지금은 그 호출 자체가 워커 안으로 옮겨갔기 때문에 이 탭은 `transport.onHeartbeatSent`/`onHeartbeatGap`/`onHeartbeatError`로 워커가 보내주는 소식을 받아 화면 숫자만 갱신한다.

게임패드 연결 상태는 `window`의 `gamepadconnected`/`gamepaddisconnected` 이벤트로 표시한다 — 참고로 Gamepad API 스펙상 실제로 게임패드의 버튼이나 스틱을 한 번 조작해야 브라우저가 그 게임패드를 "연결됨"으로 인식하는 브라우저가 많다(연결만 해두고 가만히 있으면 안 뜰 수 있음).

이 페이지가 존재하는 이유는 단 하나, 지금까지의 모든 검증이 Node.js끼리의 통신이었고 실제 브라우저 탭은 이 흐름에 한 번도 들어온 적이 없었기 때문이다. `scripts/serve-dashboard.mjs`로 서빙한 뒤 `http://localhost:5173/apps/dashboard/index.html`로 열어야 한다.

### 실사용 테스트 히스토리

첫 실사용 테스트에서 탭이 포그라운드에 있고 사용자가 계속 버튼을 누르고 있었는데도 하트비트가 23초 가까이 끊겨 ESTOP이 발동하는 원인 불명의 현상이 있었다. 진단 로직(중복 Connect 가드, 전송 간격 경고, 전송 실패 로그, sent/ack 카운터 분리)을 추가한 뒤 같은 방식(탭을 직접 닫는 것)으로 재시도했을 때는 경고 없이 깨끗하게 재현됐다 — `connection closed` 후 311ms 만에 ESTOP. 처음 현상은 재현되지 않아 일회성 현상으로 기록해뒀다.

두 번째 실사용 테스트는 처음엔 게임패드 조작으로 오해했지만, 실제로는 게임패드가 연결되지 않은 상태에서 수동 슬라이더만으로 진행된 것이었다 — `TeleopNode`는 정확히 설계대로 아무것도 발행하지 않았을 뿐이다. 원인 조사 과정에서 만든 `teleop ticks/s`, `gamepad seen/s`, `raw axes` 실시간 카운터(1초마다 리셋)는 결과적으로 필요 없었던 조사의 부산물이지만, 실제 게임패드 테스트 때 그대로 쓸 수 있어서 남겨뒀다. `teleop-node.js`의 `onTick` 콜백이 매 폴링마다 불리는 걸 그대로 세는 것뿐이라, 이 숫자가 낮게 나오면 `_tick()` 자체가 안 도는 것이고, 숫자는 정상인데 `raw axes`가 안 바뀌면 크롬의 게임패드 상태 갱신이 문제라는 걸 구분할 수 있다.

전체 타임라인과 로그는 `plan.md`의 "Phase 2 진행"과 "실사용 테스트 — 게임패드 조작 확인" 절에 있다.
