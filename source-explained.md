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

## packages/transport/src/codecs.js

와이어 프로토콜 코덱 레지스트리 — 매니페스트 `transport.kind`로 고른다. 코덱 = `{ Decoder, encode }`: `Decoder`는 `new Decoder().push(bytes)`로 정규화 메시지(`{type:'ack'|'reply'|'line',...}`)를 내는 클래스, `encode(spec)`은 명령 하나를 와이어 바이트로. `getCodec(kind, manifest?)`가 조회하고 미등록 kind면 throw. 레지스트리 항목은 (a) 정적 `{ Decoder, encode }`(무상태, 재사용 — Roboteq) 또는 (b) `{ factory(manifest) → { Decoder, encode } }`(transport마다 새 인스턴스 필요 — TB3). 등록된 것: `roboteq-serial`(`RoboteqDecoder` + `encodeCommand`), `turtlebot3-opencr`(팩토리 → `makeTurtlebot3OpenCRCodec(opencrConfigFromManifest(manifest))`). transport 3종(`WebSocket`/`WebSerial`/`Rtc`)이 생성자에서 `{ codec = getCodec() }`를 받아 `this._decoder = new codec.Decoder()`를 만들고 `encode(spec)` 메서드를 노출하므로, `packages/nodes`·`device-abstraction`·`rtc-host-bridge`는 코덱을 직접 import하지 않는다. roadmap.md "세 타깃 동시 진행" 참고.

## packages/transport/src/dynamixel-protocol2.js

DYNAMIXEL Protocol 2.0 와이어 포맷 — TB3의 OpenCR 보드가 쓰는 것(OpenCR은 컨트롤테이블을 가진 DXL 장치 id 200으로 행세). 프로토콜 표준이라 TB3 비의존. 패킷: `FF FF FD 00 | ID | LEN(LE 2) | INSTR | PARAMS | CRC(LE 2)`, LEN은 INSTR부터 CRC까지. `crc16`(poly 0x8005, MSB-first, ROBOTIS SDK 테이블), `stuff`/`unstuff`(PARAMS 안 `FF FF FD` → `FF FF FD FD`), `toLE`/`fromLE`(2의 보수). `buildRead`/`buildWrite`/`buildPing`/`buildInstruction`. `Protocol2Decoder.push(bytes)` → `{ id, error, params }` 배열, STATUS 패킷만 통과, 헤더 재동기 + CRC 불일치 드롭, push 경계 걸쳐도 파싱. 스모크는 ROBOTIS e-manual 레퍼런스 벡터(PING id1, WRITE id1 addr116=512) CRC까지 대조.

## packages/transport/src/turtlebot3-opencr.js

`codecs.js`용 코덱 팩토리 — HardwareTransport 호출을 OpenCR 컨트롤테이블 R/W로 바꾸고, 응답을 `roboteq.js`와 **같은** 정규화 메시지로 되돌려 `packages/nodes`(특히 `OdometryNode`)를 무변경으로 만든다. Protocol 2.0 STATUS 패킷이 어느 주소에 대한 응답인지 안 알려주므로 `encode`와 `Decoder`가 작은 FIFO(`pending`)를 공유 — 그래서 `codecs.js`가 이 코덱만 transport마다 팩토리로 새로 만든다. `encode({op:'read', key})`는 `reads[key]`(필드 offset/width/signed) 조회 → READ 패킷 + `pending`에 push. `encode({op:'write', from:'<컨트롤테이블명>', value})` 또는 `{op:'write', address, fields:[{value,width,signed}]}` → WRITE 패킷. `Decoder.push`는 STATUS마다 `pending.shift()`: read면 필드 적용해 `{type:'reply', key, values}`, write/ping이면 `{type:'ack', ok: error===0}`. `opencrConfigFromManifest(manifest)`가 `transport.controlTable`의 `presentPositionLeft/Right`(연속 주소 가정)를 read 그룹 `C`로 조립. **주소는 전부 매니페스트에 있고 하드웨어에서 검증 필요 — `todo-tb3.md`.**

## packages/transport/src/lidar-lds.js

TB3 Burger 라이다(ROBOTIS LDS-01 = HLS-LFCD2) 시리얼 파서. 출력은 시뮬레이터 센서 스트림(`:8766`)이 이미 `MapNode`에 주는 것과 **같은** 스캔 객체 모양이라 실물 전환 시 위쪽 무변경. LDS-01: 1회전 = 90패킷 × 4샘플, 패킷 22바이트(`0xFA` + index `0xA0..0xF9` + 속도 uint16 + 4×4바이트 샘플 + 체크섬 uint16). `ldsChecksum`(20바이트에 대한 10개 uint16 워드, `chk32 = (chk32<<1) + word`, `((chk32 & 0x7FFF) + (chk32>>15)) & 0x7FFF`). `LdsDecoder.push(bytes)` → 완성된 회전마다 `{ type:'scan', angleMin..rangeMax, ranges[360] (m, ≥rangeMax = no return), rpm }`. invalid 플래그/거리 0 → rangeMax. index가 되감기면 회전 경계 → flush. LDS-02는 framing 달라서 미지원(`model` 파라미터). `buildLdsPacket`은 스모크용.

## packages/transport/src/websocket-transport.js

**시뮬레이터** 상대 테스트용 `HardwareTransport` 구현체. 전역 `WebSocket`(브라우저 + Node 22+)만 써서 브라우저 탭에서도 Node 스크립트에서도 수정 없이 돈다 — `apps/dashboard/index.html`(local 모드), `host.html`(simulator 모드), `scripts/*.mjs`가 같은 클래스를 쓴다.

생성자에서 `{ codec = getCodec() }`(기본 Roboteq)를 받아 `codec.Decoder` 하나를 들고 있다가, `onmessage`로 바이트가 들어오면 (1) 먼저 raw 바이트 그대로 `onRaw` 콜백들에 넘기고 (2) 디코더에 밀어넣어 나온 파싱 메시지를 `onMessage` 콜백들에 넘긴다. `onRaw`는 파싱하지 않고 바이트만 중계해야 하는 곳(`RtcHostBridge`)을 위한 것이다. `encode(spec)`은 코덱의 `encode`에 위임한다.

`connect()`는 `onopen`에서 resolve, `onclose`에서 disconnect 콜백. `send()`는 `ws.send(frame)`. `close()`는 인터페이스의 선택적 부분 — 정상 종료 핸드셰이크를 보내는 의도적 종료다. "크래시처럼 인사 없이 끊기"를 재현하려면 `close()`를 부르지 않고 프로세스/페이지를 죽여야 하고(`prototype-client.mjs`가 그렇게 한다), `RtcHostBridge`는 operator 세션을 깔끔히 끝낼 때 쓴다.

## packages/transport/src/web-serial-transport.js

**실제 Roboteq 컨트롤러** 상대 `HardwareTransport` 구현체 — `navigator.serial`로 로봇 자기 PC의 `/dev/ttyMOTOR`(115200 8N1)를 연다. `WebSocketTransport`와 완전히 같은 모양(`connect`/`send`/`onMessage`/`onRaw`/`onDisconnect`/`close`)이라 `RtcHostBridge`·`createDriveDevice`는 둘 중 뭘 받든 상관 안 한다. Chromium 전용 + secure context(localhost 포함) 필요. Node에서 못 돌려서 `node --check`만, 실검증은 로봇에서 사람이.

생성자는 `WebSocketTransport`와 같은 `{ codec = getCodec() }`를 받고 `encode(spec)`을 노출한다. `connect()`: 이미 승인된 포트가 정확히 하나면 그걸 재사용(재부팅 후 프롬프트 없음), 아니면 `navigator.serial.requestPort()`로 피커를 띄운다 — `filters`(Former FTDI 어댑터 `0x0403`/`0x6001`)로 피커를 좁힌다. 열고 나서 `writable.getWriter()`로 쓰고, `readable.getReader()`를 돌리는 `_readLoop()`가 들어온 청크를 `onRaw` → `codec.Decoder` → `onMessage` 순으로 흘린다. 포트 물리적 분리는 `disconnect` 이벤트로, 스트림 종료/에러는 read 루프의 `finally`에서 `onDisconnect`로 통지. `close()`는 reader cancel → writer close → port close 순.

매 부팅 프롬프트를 없애려면 엔터프라이즈 정책 `SerialAllowUsbDevicesForUrls`(대시보드 origin + FTDI VID/PID)를 깔아야 한다. 이 파일에도 그 주석이 있다.

## packages/transport/src/heartbeat.js

`startHeartbeat(transport, options)`. 100ms마다 `!B 3 1`(Roboteq keepalive bool — 레퍼런스 ROS 드라이버가 매 제어 사이클 보내고, 온보드 안전 스크립트도 이걸 본다)을 보낸다. Former의 Roboteq는 시리얼이 ~1초 조용하면 RWD 워치독이 모터를 세우므로, 그 안에서 뭐라도 계속 보내는 게 이 루프의 역할이다. 매 틱마다 직전 전송과의 간격(`performance.now()`)을 재서 `gapWarnMs`(기본 150ms)를 넘으면 `onGap` — 원인 불명의 23초 정지(plan.md "Phase 2 진행") 이후 남긴 진단. `onSend`/`onSendError`, `stop()`.

## packages/transport/src/index.js

`roboteq.js`, `codecs.js`, `websocket-transport.js`, `web-serial-transport.js`, `heartbeat.js`를 재수출한다. 파일 맨 위 주석에 `HardwareTransport` 인터페이스 모양(`connect`/`send`/`encode`/`onMessage`/`onDisconnect`, 선택적 `onRaw`/`close`)과 와이어 프로토콜이 매니페스트 `transport.kind`로 선택된다는 것(지금은 Roboteq 하나), 그리고 두 구현체(`WebSocketTransport` = 시뮬레이터, `WebSerialTransport` = 실기)를 적어뒀다. 그 위 코드(device-abstraction, nodes, dashboard)는 transport 구현이나 코덱을 갈아끼워도 안 건드리는 게 이 인터페이스를 둔 이유다.

## packages/bus/src/local-bus.js

`LocalBus` — `BroadcastChannel` 위에 토픽 이름으로 필터링하는 pub/sub을 얹은 것. 구현하면서 실제로 걸려 넘어진 버그가 하나 있어서 그대로 적어둔다.

`BroadcastChannel`은 스펙상 **자기 자신이 보낸 메시지를 자기 자신에게는 전달하지 않는다** — 같은 채널 이름을 쓰는 *다른* `BroadcastChannel` 인스턴스에만 전달된다(같은 페이지 안에서 만든 다른 인스턴스여도 상관없이). 처음 짠 버전은 `publish()`가 `postMessage()`만 호출했는데, `apps/dashboard/index.html`에서 `TeleopNode`와 구독 로직이 **같은 `LocalBus` 인스턴스**를 공유하도록 짜다 보니 발행한 명령이 자기 자신의 구독자에게 영원히 전달되지 않는 상황이 됐다. Node로 직접 재현해서 확인했다(`bc.postMessage()` 후 같은 인스턴스의 `onmessage`가 500ms 동안 안 옴 → 확정). 지금 버전은 `publish()`가 `postMessage()`와 함께 `_dispatch()`를 직접 호출해서 같은 인스턴스의 구독자에게도 즉시 전달하도록 고쳤다 — 그 결과 같은 인스턴스를 공유하든(대시보드처럼) 노드마다 별도 인스턴스를 만들든(원래 의도했던 구조) 둘 다 정상 동작한다. 두 경우 다 Node 스크립트로 실제 검증했다.

## packages/bus/src/hardware-bridge-worker.js, hardware-bridge-client.js

Phase 4(plan.md)에서 추가한, 하드웨어 연결을 탭 여러 개가 공유하는 부분. `hardware-bridge-worker.js`는 일반 모듈이 아니라 `new SharedWorker(url, { type: 'module' })`로 로드되는 워커 스크립트 자체다 — 이 스크립트 하나가 오리진 전체에서 단 하나만 실행되고, 탭이 몇 개든 전부 이 하나의 인스턴스에 `MessagePort`로 연결된다.

워커 안에는 진짜 `WebSocketTransport` 인스턴스가 딱 하나만 존재한다. 첫 번째 탭이 `{type:'connect'}`를 보내면 그때 실제로 `transport.connect()`를 호출하고 `startHeartbeat()`도 그때 한 번만 시작한다(`ensureConnected()`가 진행 중인 연결 시도를 프라미스로 캐싱해서 여러 탭이 동시에 연결을 요청해도 실제 연결 시도는 한 번만 일어나게 막는다). 이후 새로 열리는 탭은 이미 연결되어 있으면 그 상태를 바로 돌려받는다. 컨트롤러에서 오는 파싱된 메시지(`{type:'message', msg}`)와 연결 상태, keepalive 진단(간격 경고, 전송 실패)은 전부 `broadcast()`로 연결된 모든 포트에 똑같이 전달된다.

`hardware-bridge-client.js`의 `HardwareBridgeClient`는 탭 쪽에서 이 워커에 붙는 프록시다. 의도적으로 `WebSocketTransport`와 같은 모양(`connect()`, `send()`, `encode()`, `onMessage()`, `onDisconnect()`)을 그대로 구현했다 — `createDriveDevice(transport, manifest)`는 `WebSocketTransport`를 받든 `HardwareBridgeClient`를 받든 코드 한 줄도 안 바뀐다. 인코딩은 탭 쪽(`encode()` = 생성자 `{ codec = getCodec() }`)에서, 디코딩은 워커 쪽에서 한다(둘 다 기본 Roboteq). 요청/응답 상태를 공유하는 팩토리 코덱(TB3 OpenCR)은 이 워커 경계를 넘어 쪼갤 수 없지만, WebSerial 포트는 SharedWorker로 transfer가 안 되므로 그 조합은 애초에 안 생긴다.

한 가지 의도적인 단순화: `send()`는 워커에 메시지를 posting만 하고 바로 resolve되는 fire-and-forget이다. 매 명령(특히 100ms마다 나가는 keepalive)마다 워커의 응답을 기다리는 왕복을 만들지 않기 위한 선택이고, 대신 전송이 실패하면 `onSendError` 콜백으로 비동기 통지된다.

이 두 파일은 `SharedWorker`, `self.onconnect` 같은 브라우저 전용 API에 의존해서 Node에서는 실행할 수 없다(Node에 `SharedWorker`가 없다). `node --check`로 문법만 확인했고, 실제 동작(탭 두 개를 열었을 때 펌웨어 로그에 `connection opened`가 한 번만 찍히는지, 탭을 하나씩 닫아도 나머지 탭은 멀쩡한지, 모든 탭이 닫히고 나서야 워치독이 독자적으로 정지시키는지)은 이 스택에서 처음으로 사람이 실제 브라우저로 직접 검증해야 하는 부분이다.

## packages/device-abstraction/src/manifest.js, drive-device.js

`loadManifest(url)`은 `fetch()`로 매니페스트 JSON을 받아오는 것뿐이다. 스키마 검증은 없고, fetch 자체가 실패하면 에러를 던진다. 브라우저 전용이라(HTTP fetch를 가정) Node에서 직접 쓸 일은 없다.

`createDriveDevice(transport, manifest)`는 **디바이스 무관**하다. 명령 어휘를 코드에 박지 않고 전부 매니페스트에서 읽는다 — `drive.commands`(`enable`/`estop`/`setVelocity` 템플릿), `drive.channels`(`{left, right}` 채널 번호), `drive.scale`(정규화 → 와이어 단위 스케일). 그래서 같은 와이어 프로토콜을 쓰는 다른 디퍼렌셜 베이스는 이 파일을 안 건드리고 매니페스트 파일만 새로 쓰면 된다(plan.md Phase 3의 "로봇마다 바뀌는 건 매니페스트 하나" 목표).

`setVelocity(left, right)`는 정규화 [-1, 1](TeleopNode·슬라이더가 내는 값)을 받아 `toUnits`로 `±scale`(Former은 1000 = ±200 바퀴 RPM)로 클램프한 뒤, `drive.commands.setVelocity`를 채워 보낸다. `drive.commands.*` 항목은 **문자열 템플릿**(Roboteq ASCII 라인)이거나 **구조화 객체**(TB3 OpenCR write/read op)일 수 있고, `fillSpec`가 둘 다 처리한다 — 문자열 leaf의 `${a.b}` 참조를 해석하고, **정확히 `"${ref}"` 하나뿐인 leaf는 참조 값의 타입을 유지**(그래서 수치 와이어 필드가 `"350"`이 아니라 `350`으로 남음). Former: `"!G ${ch.left} ${v.left}_!G ${ch.right} ${v.right}"` → `!G 1 500_!G 2 -300`. TB3: `{op:'write', from:'goalVelocityLeft', fields:[{value:'${v.left}',width:4,signed:true},{value:'${v.right}',width:4,signed:true}]}` → 코덱이 Protocol 2.0 WRITE 패킷으로. `enable()`/`estop()`은 항목이 있으면 채워 보내고 없으면 no-op(연결/ESTOP/워치독 이후 모터가 비활성으로 시작하므로 연결 후 `enable()` 한 번 필요; Former은 `!MG`/`!EX`, TB3는 `torqueEnable` write 1/0).

매니페스트에 **일부러 안 넣은 것** 두 가지: (1) 바이트 인코딩(라인 종결자·프레이밍)은 와이어 프로토콜 소관이라 `transport.encode()`(= `manifest.transport.kind`로 고른 코덱)에서 온다 — 이 파일은 프로토콜을 import하지 않는다. (2) 정규화 [-1, 1] 규약은 스택 전역 계약이라 코드에 둔다. 실단위(m/s) API는 별도 후속.

**리드백(opt-in)**: `createDriveDevice(transport, manifest, { readbackHz, onState })`. `readbackHz > 0`이면 `drive.readback.*` 쿼리들을 그 주기로 폴하고 `transport.onMessage`로 답을 받아 `getState()` → `{ counts, velocity(m/s), battery(V), current(A), temperature, faultFlags, estopButton, updatedAt }`에 접는다. 리플라이 키는 쿼리에서 유도(`"?V 2"` → `V`), 파싱은 Roboteq 관례 가정(`?V`는 0.1V 단위, `?DI` 첫 입력 0 = 눌림). 바퀴 속도는 `?C` 카운트 델타 / `mPerCount`(= `2π·wheelRadius/countsPerRev`) / dt. `startReadback()`/`stopReadback()`로 제어(연결 끊을 때 정지). TB3 OpenCR 매니페스트는 실제 컨트롤테이블 확정 후 필드별 처리가 필요할 수 있음(`todo-tb3.md`).

**실단위 API**: `setVelocityMps(leftMps, rightMps)` — `maxWheelMps`(= `(maxWheelRpm·2π/60)·wheelRadius`)로 정규화 [-1,1]로 환산 후 `setVelocity`. `maxWheelMps`/`mpsToNormalized`도 export.

## packages/nodes/src/teleop-node.js

`TeleopNode` — Gamepad API를 폴링해서 버스에 주행 명령을 발행한다. Gamepad API에는 "축 값이 바뀌었다" 이벤트가 아예 없어서(연결/해제 이벤트만 있음), `navigator.getGamepads()`를 타이머로 주기적으로 읽는 것 말고는 방법이 없다. 기본 50ms 간격.

왼쪽 스틱의 Y축(위로 밀면 음수가 나오므로 부호를 뒤집는다)을 전진/후진으로, X축을 회전으로 써서 아케이드 믹싱(`left = forward + turn`, `right = forward - turn`)으로 좌우 바퀴 목표 속도를 만든다. 스틱을 살짝만 건드려도 값이 남는 걸 막기 위해 ±0.08 미만은 0으로 죽이는 데드존을 뒀다.

`navigator.getGamepads()`를 직접 목(mock)으로 바꿔서(Node 22의 전역 `navigator`는 읽기 전용이라 `globalThis.navigator.getGamepads = ...`로 메서드만 얹는 식) Node로 틱 빈도를 확인했더니 50ms 인터벌 기준 1초에 약 15번(기대치 20번과 비슷한 범위) 정상적으로 돌았다 — 로직 자체는 문제없다는 뜻이다.

이후 실제 크롬 테스트에서 같은 값이 50ms가 아니라 500ms 이상 간격으로만 로그에 찍히는 현상이 나와서 원인을 조사했지만, 결과적으로 그 테스트는 애초에 게임패드가 연결되지 않은 상태에서 진행된 것이었다 — `connectedGamepad()`가 `null`을 반환하니 `_tick()`이 매번 조기 리턴했을 뿐, 로그에 찍힌 값은 화면 수동 슬라이더에서 온 것이었다. 즉 버그가 아니라 전제가 틀린 조사였다. 그래도 조사 과정에서 만든 `onTick(pad)` 콜백(매 폴링마다, 게임패드가 없어도 호출)과 대시보드의 `teleop ticks/s`/`gamepad seen/s`/`raw axes` 카운터는 남겨뒀는데, 실제로 도움이 됐다 — 진짜 게임패드로 재시도했을 때는 `velocity set`이 정확히 ~50ms 간격으로, 스틱을 중립에 두면 데드존대로 `left=0 right=0`이 찍히는 걸로 폴링과 발행 로직이 설계대로 동작한다는 게 확인됐다(`plan.md` "실제 게임패드로 재시도 — 통과" 참고). Phase 3의 게임패드 통과 기준은 이걸로 충족됐다.

## packages/planner-wasm/src/index.js

`loadPlanner()` — 자매 프로젝트 `pathfinder`(별개 GitHub 저장소, `robot-project/pathfinder`)의 `grid` 패키지(Go, Grid A*/Hybrid A*)를 컴파일한 WASM 빌드를 로드해 `findPath(request)`를 노출한다. `vendor/pathfinder.wasm` + `vendor/wasm_exec.js`는 이 저장소가 유지보수하는 소스가 아니라 vendor된 빌드 산출물이다 — `robot-base`의 `roboteq.js`가 이 저장소에도 바이트 단위로 복제되어 있는 것과 같은 원칙("두 저장소가 파일시스템 경로를 공유하지 않는다")으로, `pathfinder` 쪽에서 `npm run build:wasm`을 돌린 뒤 이 패키지의 `scripts/refresh-vendor.mjs`로 갱신한다.

`fetch(wasmUrl)`이 기본 바이트 로더인데, Node의 내장 `fetch`는 `file:` URL을 지원하지 않아서(HTTP(S) 전용) 브라우저에서는 그대로 쓰고 Node 테스트(`scripts/planner-wasm-smoke.mjs`)는 `loadBytes` 옵션으로 `readFile` 기반 로더를 주입한다. Go 프로그램의 `main()`이 `select{}`로 절대 끝나지 않으므로 `go.run(instance)`를 awiat하지 않고(끝나길 기다리면 영원히 안 끝남), 대신 `main.go`가 `pathfinderFindPath`를 등록한 직후 호출하는 `globalThis.__pathfinderWasmReady()`로 준비 완료 시점을 정확히 신호받는다(고정 `setTimeout` 추측 대신).

## packages/nodes/src/planner-node.js

`PlannerNode` — roadmap.md Phase 7의 "PlannerNode: 격자 A* → path"를 위 `planner-wasm`으로 구현. `requestTopic`을 구독해 `{ requestId, ...findPathRequest }`를 받으면 WASM 호출 결과(또는 에러)를 `{ requestId, path, distance }`(또는 `{ requestId, error }`) 형태로 `pathTopic`에 발행한다. 격자를 스스로 만들지 않고 요청에 실려 오는 격자를 그대로 쓴다 — 그 격자를 LIDAR 스캔에서 만드는 건 `MapNode`(아래), `path`를 `cmd_vel`로 바꾸는 건 `PathFollowerNode`(커밋 `60bf0b0`)의 몫.

## packages/nodes/src/map-node.js

`MapNode` — roadmap.md Phase 7/8. `scanTopic`(LaserScan 형태 `{ angleMin, angleIncrement, rangeMin, rangeMax, ranges }`)과 `poseTopic`(`{x,y,theta}` — `OdometryNode`/`PoseFusionNode`가 주거나 sim ground truth)을 구독해, 마지막으로 본 pose에서 각 스캔을 격자에 접고 `mapTopic`에 발행한다. 출력 객체는 `@ros-chromium/planner-wasm`의 `findPath` 요청 / pathfinder `grid.NewGridFromOccupancy`에 그대로 넣는 형태: `{ originX, originY, cellSize, cols, rows, occupied, occupiedInflated, prob, updatedAt }`. `occupied`/`occupiedInflated`는 row-major `bool[]`(`row*cols+col`), 원점은 최소 코너, `col=floor((x-originX)/cellSize)` — 전부 `pathfinder/grid/grid.go`의 `CellAt`/`index`와 바이트 단위로 동일. `prob`는 `Uint8Array`(p·255) — grayscale 뷰용.

설계 결정:
- **Phase 8 — log-odds 이진 베이즈 필터**. 셀마다 `L` 값을 두고 빔마다 Amanatides–Woo 순회(`castRayCells`): 지나간 셀은 `L += free`(기본 −0.4), 끝 셀은 `L += occ`(+0.85; "no return" 빔은 끝 셀도 `free`). `L`을 `[min, max]`(기본 −2 ~ 3.5)로 클램프 → 신뢰도가 유계라서 오래된 벽/복도도 나중에 갱신 가능. `p = 1 − 1/(1+e^L)`, `L=0`이 `p=0.5`(미관측). `occupied`는 `p > threshold`(기본 0.5). Phase 7의 hit/miss 집계 대비 이점은 **관성** — 30번 본 벽이 노이즈 1발에 안 지워지고, 30번 비었던 복도가 반사 1발에 벽이 안 됨. `probFromLogOdds`/`occupancyFromLogOdds`/`probGridU8` export.
- **`occupiedInflated` = `occupied`를 로봇 body 반경만큼 원형 팽창**(`inflateOccupancy`). pathfinder A*는 로봇을 점으로 보므로 PlannerNode 요청엔 이 팽창본을 넣는다. **현재 pose 셀은 강제 free**(`clearDisc`) — 팽창 벽이 자기 셀 덮으면 플래너가 `"start inside obstacle"`로 거부하기 때문.
- **저장/로드**: `serialize()` → `{ format:'mapnode-logodds-v1', 격자 config, scale, data: base64(int8 양자화 L) }`. `load(saved)`는 격자 config가 일치해야 복원(안 그러면 셀이 안 맞음). `reset()`은 `L` 전부 0. `serializeMap`/`deserializeMap` export.
- TF 없이 pose·scan을 명시적 bus 입력으로 받아 `world = pose ⊕ (range, angle)` 직접.
- `publishHz`(기본 2) 타이머, 스캔이 하나라도 들어왔을 때만. `snapshot()`으로 동기 조회.

## scripts/map-node-smoke.mjs

`MapNode`를 브라우저·시뮬레이터 없이 검증(34개 체크). 격자 기하 ↔ `grid.go`, `castRayCells` 완주, `probFromLogOdds`/`occupancyFromLogOdds`, **log-odds 관성**(20번 free 후 반사 1발에도 free 유지 / 20번 hit 후 통과 1발에도 wall 유지 — hit/miss였으면 뒤집힘), `serializeMap`/`deserializeMap` 왕복(int8 양자화 오차 이내), 팽창·`clearDisc` 단위 검증. 그다음 합성 360빔 방 스캔을 넣어 네 벽 occupied·실내 free·`prob` 그리드 발행을 확인하고, **발행 격자를 실제 `planner-wasm`에 넣어** 경로 왕복(팽창 벽 안쪽 목표는 거부), 마지막으로 `serialize → reset(벽 사라짐) → load(벽 복원)` + 격자 불일치 로드 거부.

## scripts/planner-wasm-smoke.mjs

`planner-wasm` + `PlannerNode`를 브라우저 없이 검증하는 스모크 테스트. pathfinder의 Go 테스트가 이미 다루는 시나리오(열린 공간, 벽 우회, Hybrid A*, 완전히 막힌 목적지)를 WASM 경유로 재확인하고, `grid.NewGridFromOccupancy`(원시 점유 비트맵) 경로가 동등한 폴리곤 기반 경로와 같은 결과를 내는지, 그리고 `PlannerNode`를 통한 `LocalBus` 요청/응답 왕복까지 확인한다(6개 체크). `node scripts/planner-wasm-smoke.mjs`. 2026-08-29에 이 스모크 테스트와 별개로, 실제 Chromium(Node/V8이 아니라)에서 `findPath`를 직접 호출해 동일한 결과(거리 9.806, 경로점 41개)가 나오는 것도 수동으로 확인했다.

## apps/signaling-server/src/index.js

WebRTC 시그널링만 하는 최소 WebSocket 서버. robot id 하나가 "방" 하나이고, 방마다 host 하나 + operator 여럿이 붙는다. 클라이언트가 보내는 메시지는 세 가지뿐이다 — `hello`(role + robot id, host면 manifest도 옵션), `signal`(상대에게 그대로 넘길 불투명한 `data` 블롭), `list`(알려진 로봇 목록, Phase 6 씨앗). 서버는 `signal`의 `data` 안을 절대 들여다보지 않는다. `hello`에 대한 응답 `ready`에는 이미 방에 있던 피어 목록을 실어줘서, 새로 들어온 쪽이 WebRTC offer를 지금 보낼지 상대가 올 때까지 기다릴지 판단하게 한다. 같은 robot에 두 번째 host가 붙으면 거부한다. 포트는 `SIGNALING_PORT`(기본 9770). 로봇 명령·텔레메트리는 여기를 절대 지나가지 않는다 — 핸드셰이크가 끝나면 데이터채널로 P2P로 흐른다.

## packages/rtc/src/signaling-client.js

`apps/signaling-server`의 피어 쪽 절반. WebSocket 하나를 감싸서 서버의 JSON 메시지를 콜백(`onPeerJoined`/`onPeerLeft`/`onSignal`/`onClose`)으로 바꾼다. 전역 `WebSocket`만 써서 브라우저와 Node(22+) 양쪽에서 그대로 돈다 — 브라우저 대시보드와 `scripts/signaling-smoke.mjs`가 같은 클래스를 쓴다. `connect()`는 `ready`가 올 때 `{ peerId, peers }`로 resolve한다. WebRTC가 뭔지는 전혀 모르고, SDP/ICE를 나르는 `data`는 불투명하게 전달만 한다.

## packages/rtc/src/rtc-transport.js

원격 세션의 operator 쪽. `WebSocketTransport`와 똑같은 `connect`/`send`/`onMessage`/`onDisconnect` 모양을 구현해서, 대시보드가 원격 로봇을 로컬 로봇과 같은 코드로 몬다 — transport 생성자만 바뀐다(`HardwareBridgeClient`가 Phase 4에서 한 것과 같은 수법, 한 홉 더 밖). operator가 능동적인 쪽이라 `RTCPeerConnection`과 데이터채널을 만들고 offer를 보낸다(host는 answer만; 역할이 고정이라 perfect-negotiation 안 함). host가 이미 방에 있으면 바로 offer하고, 없으면 `peer-joined`를 기다린다. 생성자는 `WebSocketTransport`처럼 `{ codec = getCodec() }`를 받고 `encode(spec)`을 노출한다. 들어오는 데이터채널 메시지는 `codec.Decoder`를 거쳐 파싱된 메시지로 나온다(`WebSocketTransport`와 같은 디코딩 경로 재사용).

이 transport는 **자체 keepalive가 없다**. operator 대시보드가 `startHeartbeat(rtcTransport)`를 직접 돈다 — keepalive(`!B 3 1`)가 operator 자신의 링크를 타야, operator가 얼거나 끊겼을 때 Roboteq RWD 워치독이 ~1초 뒤 모터를 0으로 만든다. host가 대신 보내면 이 보장이 깨진다. ICE 서버는 지금 공용 STUN 하나뿐이고, 크로스-NAT용 TURN과 LAN WebSocket 릴레이 폴백은 아직 미결(`plan.md` "아직 정하지 않은 것").

## packages/rtc/src/rtc-host-bridge.js

원격 세션의 host 쪽. 로봇 자기 PC(또는 시뮬레이터에 닿는 아무 머신)에서 돈다. 방에 나타나는 operator마다 WebRTC offer에 answer하고, 그 operator의 데이터채널과 컨트롤러로 향하는 transport 사이를 **양방향 모두 raw 바이트 그대로** 지나가는 파이프가 된다 — `transport.onRaw`를 쓰지 파싱된 `onMessage`를 안 쓴다. **keepalive를 절대 안 보내고**, ESTOP도 트래픽 파싱도 안 한다.

생성자가 `firmwareUrl` 문자열 대신 `makeTransport` 팩토리(`() => HardwareTransport`)와 `initCommands`(문자열 배열)를 받도록 바뀌었다. 그래서 같은 브리지가 `WebSocketTransport`(시뮬레이터)든 `WebSerialTransport`(실기 Roboteq)든 그대로 앞단다. **딱 하나의 예외**: 컨트롤러에 연결한 직후 `initCommands`(매니페스트의 `drive.commands.init` — 실기 Roboteq면 `^ECHOF 1`/`!R 2`/`!AC`/`!DC`, 시뮬레이터면 빈 배열)를 100ms 간격으로 보낸다. 이건 컨트롤러 bring-up이지 주행이 아니라서 예외로 뒀다.

operator 세션 하나당 transport를 새로 연다. 컨트롤러는 새 연결(또는 `!MG`) 때만 정지 상태에서 재무장하므로 "operator 연결됨" ↔ "transport 열림"을 1:1로 묶어야 연결→주행→해제→재연결이 맞는다. operator가 떠나면 그 세션의 transport를 `close()`. **주의**: 실제 시리얼 포트는 배타적이라 operator 2명 이상이면 브리지가 transport 하나를 공유해야 한다(SharedWorker가 탭에 해주듯) — Phase 5는 operator 1명 전제라 범위 밖.

`RTCPeerConnection` 의존이라 Node에서는 못 돌린다 — `node --check`만 했고 실제 동작은 사람이 브라우저로 검증(`plan.md` Phase 5 "사람이 확인해줘야 하는 것").

## packages/rtc/src/index.js

`signaling-client.js`, `rtc-transport.js`, `rtc-host-bridge.js`를 재수출하고, 파일 맨 위 주석에 세 조각의 역할과 "무엇이 Node에서 돌고 무엇이 브라우저 전용인지"를 적어뒀다.

## apps/dashboard/host.html

host 브리지 콘솔. 조작 UI가 없다 — **Controller 모드 라디오(simulator / hardware)**, 시그널링/sim URL/robot id 입력, Start bridge 버튼, 연결된 operator 목록(펌웨어 링크 상태 + 양방향 바이트), 이벤트 로그가 전부다. simulator 모드면 `makeTransport = () => new WebSocketTransport(fwUrl)`(init 없음), hardware 모드면 `() => new WebSerialTransport({ baudRate, filters })` + 매니페스트의 `drive.commands.init`을 `initCommands`로 넘긴다. `SignalingClient({role:'host'})` + `RtcHostBridge(sig, makeTransport, { initCommands, onEvent })`를 만들어 `start()`할 뿐이고, 실제 중계는 전부 `RtcHostBridge` 안. 페이지 상단에 "몰지 않고 keepalive도 안 보낸다(단 bring-up 명령은 보냄)"를 명시.

## manifests/former.manifest.json

Former 2.0을 기술하는 매니페스트 — 이제 실제로 소비된다. `drive.commands`: `init`(실기 Roboteq bring-up 시퀀스 `["^ECHOF 1", "!R 2", "!AC 1 6000_!AC 2 6000", "!DC 1 6000_!DC 2 6000"]` — 값 6000은 레퍼런스 ROS 드라이버의 `robot_acceleration`/`robot_deceleration`), `enable`/`estop`/`setVelocity` 템플릿 문자열. `drive.channels`(`{left:1, right:2}`), `drive.scale`(1000)이 `createDriveDevice`의 주행 동작을 규정하고, `drive.commands.init`은 `host.html` hardware 모드가 읽어 `RtcHostBridge`의 `initCommands`로 넘긴다. `transport.baud`(115200)는 `WebSerialTransport` 보드레이트. `drive.geometry`와 `drive.readback`은 아직 향후 용도. 이전 이름은 `rover.manifest.json`. 두 Node 스크립트도 이 파일을 그대로 읽는다(dogfooding).

## scripts/prototype-client.mjs

`WebSocketTransport` + `startHeartbeat` + `createDriveDevice`를 실제로 가져다 쓰는(dogfooding) Node.js 클라이언트. 브라우저 탭 하나가 하는 걸 그대로 한다.

1. `WebSocketTransport`로 시뮬레이터에 연결.
2. `?FID` 한 번 보내 컨트롤러 응답 로그, `drive.enable()`(`!MG`).
3. `startHeartbeat`로 100ms 간격 `!B 3 1` keepalive 시작.
4. 연결 150ms 시점에 `drive.setVelocity(0.5, 0.5)` → `!G 1 500_!G 2 500`.
5. 연결 350ms 시점에 `heartbeat.stop()` 후 `transport.close()`를 **부르지 않고** `process.exit(0)` — 크래시 흉내.

`close()`를 일부러 안 부르는 게 핵심이다. 정상 종료는 핸드셰이크가 있지만 크래시는 그럴 겨를이 없다. 이 스크립트는 정지를 스스로 확인하지 않는다(프로세스가 죽어버리니까) — 시뮬레이터 로그에서 이 프로세스가 죽고 `SIM_RWD_MS`(기본 1초) 뒤 찍히는 `motors zeroed — RWD: ...`로 확인한다. 빠르게 보려면 시뮬레이터를 `SIM_RWD_MS=300`으로 띄운다.

## scripts/roboteq-smoke.mjs

Roboteq 라인 프로토콜 전 구간을 브라우저 없이 검증하는 스모크 테스트 — `WebSocketTransport` + `roboteq.js` 코덱 + 시뮬레이터의 Roboteq 에뮬레이터 + `createDriveDevice`, 그리고 load-bearing한 RWD 워치독. 자체적으로 시뮬레이터를 짧은 `SIM_RWD_MS`로 자식 프로세스로 띄우고 8개 체크(`?FID` 응답, 매니페스트 `drive.commands.init` 시퀀스 전부 `+` ack, `!MG` 전엔 `!G` 무시, `!MG` 후 엔코더 증가, `+` ack, `!EX` 후 `FF=16`/`DI=0` 래치, 침묵 시 RWD 정지)를 돌린 뒤 PASS/FAIL, 실패 시 non-zero 종료. `node scripts/roboteq-smoke.mjs`.

마지막 체크("침묵 시 RWD 정지")는 단순히 로그에 그 줄이 있는지가 아니라, 침묵을 시작한 시점부터 그 줄이 실제로 찍히기까지의 시간을 20ms 간격으로 폴링해 측정하고 `RWD_MS`~`RWD_MS+150ms` 범위 안인지 어서션하는 **타이밍 회귀 테스트**다(`plan.md` "npm test 배선 + RWD 타이밍 검사를 회귀 테스트로 강화" 참고) — 워치독이 원래보다 훨씬 느려져도 "언젠가는 멈췄으니" 통과해버리는 구멍을 막기 위함.

## scripts/signaling-smoke.mjs

`apps/signaling-server`와 `SignalingClient`를 브라우저 없이 검증하는 스모크 테스트. `RTCPeerConnection`은 Node에 없으므로 여기서는 안 건드리고, "랑데부"만 본다 — hello→ready, peer-joined/peer-left, 한 피어의 `signal` 블롭이 다른 피어에게 그대로(순서 보존 포함) 나오는지. SDP/ICE 페이로드는 가짜 문자열이다(서버가 안 들여다보므로). 자체적으로 던져버릴 포트에 시그널링 서버를 자식 프로세스로 띄우고, 8개 체크를 돌린 뒤 PASS/FAIL을 찍고 실패 시 non-zero로 종료한다. `node scripts/signaling-smoke.mjs`.

두 스모크 테스트 모두 루트에서 `npm test`로 함께 실행할 수 있다(`package.json`의 `test` 스크립트).

## scripts/serve-dashboard.mjs

Node 내장 `http`/`fs`만 쓰는, 의존성 없는 정적 파일 서버. `apps/dashboard/*.html`이 `<script type="module">`로 `packages/*/src/*.js`를 상대 경로로 불러오는데, HTML을 `file://`로 열면 크로미움이 모듈 임포트를 CORS로 막으므로 `http://`로 서빙해야 한다. 레포 루트(번들에선 `stack/`)를 통째로 서빙하고, 확장자별 `Content-Type`을 최소한으로 맞춘다(`.js`/`.mjs` → `text/javascript`). 모든 인터페이스에 바인딩하므로 원격 operator 노트북이 `http://<robot-ip>:5173`으로 접근할 수 있다. 기본 포트 5173, `DASHBOARD_PORT`.

## deploy/ — 오프라인 설치 번들

회사망에서 로봇이 외부 접근이 안 돼서, 노트북에서 만들어 옮기는 설치 번들. 상세는 `deploy/README.md`와 `deploy/bundle/README.md`.

- `deploy/make-offline-bundle.sh` (노트북, 인터넷 필요) — 스택 소스 복사(`node_modules`/`.git`/`deploy`/`dist` 제외) + `node_modules/ws` 벤더링(스택의 유일한 외부 런타임 의존성, transitive 0) + Node linux tarball 다운로드 + `debian:<suite>` Docker 컨테이너에서 Chromium `.deb` 의존성 클로저 받기. `--suite`/`--arch`로 로봇 Debian에 맞추고, Docker 없으면 `--skip-chromium`. 결과물 `web/dist/former-webstack-offline-<날짜>.tar.gz`.
- `deploy/bundle/install.sh` (로봇, 오프라인, sudo 재실행) — `/opt/former-webstack`에 stack + node 설치, 번들 `.deb`를 dpkg, 호출 유저를 `dialout`에 추가, `/dev/ttyMOTOR`가 없으면 udev 규칙 설치, Chromium managed policy(`serial-policy.json` → `/etc/chromium/policies/managed/`)로 WebSerial 포트 피커를 1회 이후 억제, 데스크톱 kiosk autostart(`.desktop`) + 서버용 systemd 유닛(설치만, enable은 안 함). arch 불일치면 중단.
- `deploy/bundle/kiosk-launch.sh` — `signaling-server` + `serve-dashboard.mjs`를 띄우고(0.0.0.0), 정적 서버가 올라올 때까지 기다린 뒤 `chromium --kiosk`로 `host.html`을 연다. `chromium`/`chromium-browser` 둘 다 대응.
- `deploy/bundle/former-webstack.service` — 두 Node 서버만 도는 헤드리스용 유닛(`__USER__`/`__BASE__`는 install.sh가 치환). kiosk Chromium은 그래픽 세션이 필요해서 여기 말고 데스크톱 autostart로 뺐다.
- `deploy/bundle/99-former-serial.rules` — ROAS `former_bringup`의 udev 규칙 사본(FTDI `0403:6001`을 devpath로 구분해 `/dev/ttyMOTOR`). `former_bringup`이 깔려 있으면 그쪽을 쓴다.

스테이징 로직(제외 패턴, `ws` 위치)과 스테이지 복사본에서 두 서버가 기동하는지는 Windows에서 확인. Node/Chromium 실제 다운로드와 로봇 설치는 로봇 Debian 버전 확정 후.

## apps/dashboard/index.html

Phase 1~2에서는 연결성 테스트용 최소 페이지였는데, 이번 Phase 3에서 실제 게임패드 teleop 대시보드로 바뀌었다. 연결 흐름과 하트비트 진단 로직(중복 Connect 가드, 전송 간격 경고, 전송 실패 로그)은 이전 버전에서 그대로 가져왔고, `startHeartbeat` 헬퍼로 옮겨서 코드는 오히려 줄었다.

Connect를 누르면 순서대로: `loadManifest()`로 `/manifests/former.manifest.json`을 받아오고 → `getCodec(manifest.transport.kind, manifest)`로 코덱을 골라 transport에 넘기고 → `createDriveDevice(transport, manifest, { readbackHz: 5, onState })`로 주행 디바이스를 만들고(리드백 폴 켜짐 — `onState`가 batt/temp/fault/estop/wheel-v 화면 갱신) → 연결 후 `drive.enable()`(`!MG`) → `LocalBus`로 `former-01/drive/cmd_vel`를 구독해 `{left, right}`를 `drive.setVelocity()`에 넘기고 → `TeleopNode` 시작. 수동 슬라이더도 같은 토픽에 `publish`만 한다.

`@ros-chromium/planner-wasm`이 nodes 배럴(→ `planner-node.js`) 경유로 bare specifier 들어오므로 `<head>`에 `<script type="importmap">` 필요(`nav.html`과 동일). `transport.onDisconnect(() => teardown())` 배선은 `await transport.connect()` **성공 후에만** — SharedWorker가 connect 진행 중에 이전 세션의 stale "not connected"를 흘리면 teardown이 transport/drive/bus를 스스로 null로 만들어버리기 때문(연결 중 실패는 connect()의 reject로 처리). `teardown()`에 `drive?.stopReadback()` 추가.

Phase 4에서는 `WebSocketTransport`를 직접 여는 대신 `HardwareBridgeClient`를 연다는 점만 바뀌었다. `createDriveDevice`는 두 경우 모두 같은 방식으로 호출되므로(둘 다 `HardwareTransport` 모양을 구현하기 때문에) 이 위의 배선 설명은 그대로 유효하다. 달라진 건 하트비트 관련 UI 갱신 방식이다 — 이전엔 이 탭이 직접 `startHeartbeat`를 불렀지만, 지금은 그 호출 자체가 워커 안으로 옮겨갔기 때문에 이 탭은 `transport.onHeartbeatSent`/`onHeartbeatGap`/`onHeartbeatError`로 워커가 보내주는 소식을 받아 화면 숫자만 갱신한다.

게임패드 연결 상태는 `window`의 `gamepadconnected`/`gamepaddisconnected` 이벤트로 표시한다 — 참고로 Gamepad API 스펙상 실제로 게임패드의 버튼이나 스틱을 한 번 조작해야 브라우저가 그 게임패드를 "연결됨"으로 인식하는 브라우저가 많다(연결만 해두고 가만히 있으면 안 뜰 수 있음).

Phase 5에서 모드 선택(`local` / `operator`)이 추가됐다. `local`은 지금까지대로 `HardwareBridgeClient`(SharedWorker)를 쓰고, `operator`는 `SignalingClient` + `RtcTransport`를 써서 원격 host 브리지에 WebRTC로 붙는다. transport 위쪽 배선(`createDriveDevice`, `drive.enable()`, `LocalBus`, `TeleopNode`, 버스 구독)은 두 모드가 완전히 동일하고, 갈리는 건 keepalive뿐이다 — `local`은 워커가 keepalive를 소유하므로 이 탭은 `onHeartbeatSent`/`onHeartbeatGap`/`onHeartbeatError`를 듣기만 하고, `operator`는 `RtcTransport`에 keepalive가 없으므로 이 탭이 (Phase 4 이전처럼) `startHeartbeat`를 직접 돈다. 이게 load-bearing한 선택인 이유는 `plan.md` Phase 5 "하트비트는 operator가 소유한다" 절에 적어뒀다. 컨트롤러가 각 명령에 보내는 `+` ack는 `transport.onMessage`에서 세어 "acked" 카운터로 쓴다(keepalive가 100ms마다 하나씩 `+`를 받으므로 왕복 liveness 지표). `teardown()`은 두 모드 공통 정리(keepalive·teleop·bus 정지, transport null)를 한군데로 모은 것이다.

이 페이지가 존재하는 이유는 단 하나, 지금까지의 모든 검증이 Node.js끼리의 통신이었고 실제 브라우저 탭은 이 흐름에 한 번도 들어온 적이 없었기 때문이다. `scripts/serve-dashboard.mjs`로 서빙한 뒤 `http://localhost:5173/apps/dashboard/index.html`로 열어야 한다.

## apps/dashboard/nav.html

roadmap.md Phase 7·8의 통과 페이지 — 시뮬레이터 상대로 **브라우저 안에서** nav 스택 전체를 돌린다. `apps/sim-driver`(Node)도, pathfinder 서버도 필요 없다. 페이지가 직접:

- `WebSocketTransport('ws://127.0.0.1:8765')` (Roboteq) + raw `WebSocket('ws://127.0.0.1:8766')` (센서 스트림)에 붙는다.
- **[Connect]** — 매니페스트(`tb3-sim`) 로드 → transport 연결 → init 시퀀스 → `createDriveDevice` → `startHeartbeat` → `drive.enable()`(`!MG`) → `OdometryNode` + `PoseFusionNode` 생성 + bus 구독 배선.
- **[Start mapping]** — `MapNode`(격자 `gridConfigFromBounds([-1,-1]..[16,12], 0.05m)`, 인플레이션 0.18m) + `PlannerNode`(WASM 로드) + `PathFollowerNode` 생성, 그리고 센서 WS 오픈. 센서 프레임마다: `scan`을 `sim/scan`에 발행, 1.5초마다 ground truth + 가우시안 노이즈를 `sim/correction`에 발행(VPS 대역). 첫 correction 전엔 `scan`을 안 흘려서 MapNode가 (0,0) 콜드스타트 pose에서 래스터화하는 걸 막는다.
- **캔버스 클릭** — 클릭 좌표를 월드 좌표로 변환 → `mapNode.snapshot()`의 `occupiedInflated`(팽창본)를 `occupied`로, 현재 fused pose를 start로, 클릭점을 goal로 해서 `sim/plan-request` 발행. `PlannerNode` 응답(`sim/plan-result`)이 오면 경로를 `sim/path`에 발행 → `PathFollowerNode`가 pure pursuit로 `sim/drive/cmd_vel` 발행 → 버스 구독이 `drive.setVelocity()` 호출 → 시뮬레이터 주행.
- **[Save map]/[Load map]** — `mapNode.serialize()` ↔ `localStorage["mapnode:nav"]`. Phase 8의 저장/로드.
- rAF 루프가 캔버스에 그린다: 점유격자를 **확률 grayscale**로(`prob` 필드: p→0 흰색, p≈0.5 미관측 투명, p→1 검정 — `MapNode` 발행 때 오프스크린 `ImageData`로 래스터), 팽창 셀은 반투명 파랑 halo, fused pose(초록), odom 고스트(점선), 레이저 스캔점(빨강), 계획 경로(파랑), goal 마커.

`@ros-chromium/planner-wasm`이 `planner-node.js`에서 bare specifier로 import되므로 — 브라우저엔 패키지 리졸버가 없다 — 페이지 `<head>`의 `<script type="importmap">`로 서빙 경로에 매핑한다. `serve-dashboard.mjs` MIME 맵에 `.wasm`도 추가했다.

**Phase 7 통과 확인(2026-08-30, 실제 Chrome)**: open 월드에서 시작(1,1) → 클릭 → PlannerNode 108점/6.19m 경로 → PathFollowerNode 추종 → 로봇이 목표로 자율 주행, 맵은 주행 중 계속 채워짐. 순수 pure pursuit는 목표가 뒤/급격한 옆에 있으면 forward-only라 기어가는 데드존이 있어서 `pursuitStep`에 "헤딩에서 크게(>~57°) 벗어나면 제자리 회전" 분기를 추가했다(스모크에 케이스 추가).

### 실사용 테스트 히스토리

첫 실사용 테스트에서 탭이 포그라운드에 있고 사용자가 계속 버튼을 누르고 있었는데도 하트비트가 23초 가까이 끊겨 ESTOP이 발동하는 원인 불명의 현상이 있었다. 진단 로직(중복 Connect 가드, 전송 간격 경고, 전송 실패 로그, sent/ack 카운터 분리)을 추가한 뒤 같은 방식(탭을 직접 닫는 것)으로 재시도했을 때는 경고 없이 깨끗하게 재현됐다 — `connection closed` 후 311ms 만에 ESTOP. 처음 현상은 재현되지 않아 일회성 현상으로 기록해뒀다.

두 번째 실사용 테스트는 처음엔 게임패드 조작으로 오해했지만, 실제로는 게임패드가 연결되지 않은 상태에서 수동 슬라이더만으로 진행된 것이었다 — `TeleopNode`는 정확히 설계대로 아무것도 발행하지 않았을 뿐이다. 원인 조사 과정에서 만든 `teleop ticks/s`, `gamepad seen/s`, `raw axes` 실시간 카운터(1초마다 리셋)는 결과적으로 필요 없었던 조사의 부산물이지만, 실제 게임패드 테스트 때 그대로 쓸 수 있어서 남겨뒀다. `teleop-node.js`의 `onTick` 콜백이 매 폴링마다 불리는 걸 그대로 세는 것뿐이라, 이 숫자가 낮게 나오면 `_tick()` 자체가 안 도는 것이고, 숫자는 정상인데 `raw axes`가 안 바뀌면 크롬의 게임패드 상태 갱신이 문제라는 걸 구분할 수 있다.

전체 타임라인과 로그는 `plan.md`의 "Phase 2 진행"과 "실사용 테스트 — 게임패드 조작 확인" 절에 있다.
