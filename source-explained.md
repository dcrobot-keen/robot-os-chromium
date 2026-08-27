# 소스 코드 설명

이 레포에 실제로 존재하는 코드 파일들을 하나씩 설명한다. 아키텍처 구상이 아니라 "지금 여기 있는 코드가 정확히 무엇을 하는가"를 남기는 문서다. 대부분의 패키지는 아직 `package.json`만 있는 빈 껍데기이고, 실제로 동작하는 코드는 `packages/transport`, `scripts/`, `apps/dashboard/index.html`뿐이다.

## package.json / pnpm-workspace.yaml

루트에 있는 pnpm 워크스페이스 정의. `packages/*`와 `apps/*` 아래를 전부 하나의 워크스페이스로 묶는다. 아직 `pnpm install`을 실행한 적은 없다 — 지금까지는 어떤 패키지도 서로를 `import`로 참조하지 않고 `prototype-client.mjs`가 `packages/transport/src`를 상대 경로로 직접 불러오는 방식이라, 워크스페이스 링크가 없어도 동작한다.

## packages/transport/src/commands.js

펌웨어 레포의 `firmware/sim/src/commands.js`와 바이트 하나 다르지 않게 동일한 파일이다. `HEARTBEAT`(0x01), `SET_VELOCITY`(0x02), `ESTOP`(0x03) 세 개만 실제로 쓰이고, `GET_ENCODER`/`GET_IMU`/`GET_BATTERY`는 번호만 예약되어 있다. 두 레포가 서로의 파일을 참조하지 않기 때문에 일부러 복제해뒀고, 프로토콜이 바뀌면 두 파일을 사람이 직접 맞춰야 한다.

## packages/transport/src/frame.js

이것도 `firmware/sim/src/frame.js`와 동일한 코드다. `SOF|LEN|CMD|PAYLOAD|CRC16|EOF` 프레임을 만드는 `encodeFrame()`과, 바이트가 조각나서 들어와도 완성된 프레임 단위로 복원해주는 `FrameDecoder`가 들어있다. 상세한 동작(재동기화 로직, CRC 알고리즘)은 펌웨어 레포의 `source-explained.md`에 이미 적어뒀고, 여기서는 완전히 같은 내용이라 반복하지 않는다.

## packages/transport/src/websocket-transport.js

`HardwareTransport` 인터페이스의 첫 실제 구현체. 브라우저와 Node.js(22 이상) 양쪽에 다 있는 전역 `WebSocket` 클래스 하나만 써서 만들었기 때문에, 이 파일은 수정 없이 실제 브라우저 탭 안에서도, Node 스크립트 안에서도 똑같이 동작한다 — `apps/dashboard/index.html`과 `scripts/prototype-client.mjs`가 정확히 같은 이 클래스를 가져다 쓴다.

내부적으로 `FrameDecoder` 인스턴스를 하나 들고 있다가, `onmessage`로 바이너리 메시지(`ArrayBuffer`)가 들어올 때마다 `Uint8Array`로 바꿔서 디코더에 밀어넣고, 완성된 `{cmd, payload}` 프레임이 나오면 등록된 콜백들에 전달한다. WebSocket은 이미 메시지 단위로 배달해주기 때문에 TCP 때처럼 바이트가 쪼개져 오는 걱정은 없지만, 프레임 포맷 자체(CRC 검증 등)는 어차피 나중에 WebSerial(진짜 바이트 스트림)에서도 그대로 써야 하므로 굳이 다른 디코딩 경로를 따로 만들지 않고 `FrameDecoder`를 그대로 재사용했다.

`connect()`는 `WebSocket`을 열고 `onopen`에서 resolve하는 프라미스를 반환하고, `onclose`가 오면 등록된 disconnect 콜백들을 부른다. `send()`는 그냥 `ws.send(frame)`이다 — 표준 `close()` 메서드는 항상 정상 종료 핸드셰이크를 보내기 때문에, "크래시처럼 인사 없이 끊기"를 재현하려면 `close()`를 부르지 않고 프로세스/페이지 자체를 죽여야 한다(아래 `prototype-client.mjs` 설명 참고).

## packages/transport/src/index.js

`frame.js`, `commands.js`, `websocket-transport.js`를 재수출(`export *`)한다. 파일 맨 위 주석에 `HardwareTransport` 인터페이스의 모양을 적어뒀고, 지금은 `WebSocketTransport`가 그 모양을 구현한 첫 번째이자 유일한 구현체다. `WebSerialTransport`, `WebUSBTransport` 등은 여전히 TODO로 남아있다 — 실제 보드가 정해지면 `WebSocketTransport`와 똑같은 모양으로 하나 더 추가하면 되고, 그 위의 코드(device-abstraction, nodes, dashboard)는 건드릴 필요가 없는 게 이 인터페이스를 둔 이유다.

## packages/{device-abstraction,bus,nodes,rtc}/package.json, apps/{dashboard,signaling-server}/package.json

이 다섯 곳은 전부 `package.json` 하나씩만 있는 빈 자리다. 각 파일의 `description` 필드에 이 패키지가 앞으로 뭘 담을지와 `plan.md`의 어느 단계에서 채워질지를 적어뒀다. 코드는 없다.

## manifests/rover.manifest.json

레퍼런스 로버 하나를 기술하는 매니페스트 예시. `motors.left`/`motors.right`가 각각 `SET_VELOCITY`의 어느 필드에 속도를 쓰고 `GET_ENCODER`의 어느 필드에서 현재 속도를 읽어올지를 매핑하고, `imu.orientation`이 `GET_IMU`에 연결된다. 다만 `GET_ENCODER`와 `GET_IMU`는 펌웨어/시뮬레이터 양쪽 다 아직 구현되어 있지 않으므로, 지금 이 매니페스트는 "나중에 device-abstraction 패키지가 이런 모양의 파일을 읽게 될 것이다"를 보여주는 예시일 뿐 실제로 어디서 로드해서 쓰이고 있지는 않다.

## scripts/prototype-client.mjs

`WebSocketTransport`를 실제로 가져다 쓰는 Node.js 클라이언트. 예전 버전은 `net` 소켓을 직접 다뤘지만, 지금은 이 레포가 만든 실제 추상화를 그대로 사용하도록(dogfooding) 바꿨다.

동작 순서는 다음과 같다.

1. `WebSocketTransport`로 펌웨어(시뮬레이터)에 연결한다.
2. 100ms 간격으로 `HEARTBEAT` 프레임을 계속 보낸다(`seq` 번호를 매번 증가시키면서).
3. 연결 150ms 시점에 `SET_VELOCITY(0.5, 0.5)` 프레임을 한 번 보낸다.
4. 연결 350ms 시점에 하트비트 전송을 멈추고, `transport._ws.close()`를 부르지 않은 채로 `process.exit(0)`을 호출해 프로세스를 그대로 죽인다.

`close()`를 일부러 부르지 않는 게 핵심이다. WebSocket의 정상 종료는 양쪽이 종료 프레임을 주고받는 핸드셰이크인데, 실제 탭 크래시는 그런 인사를 할 겨를이 없다. 그래서 프로세스를 그냥 죽여서 OS가 강제로 소켓을 정리하게 만드는 쪽이 "크래시"를 더 정직하게 흉내낸다. 이 스크립트는 ESTOP이 실제로 발동했는지 스스로 확인하지 않는데, 4번 단계에서 프로세스가 완전히 죽어버리기 때문에 애초에 확인할 방법이 없고, 그게 이 테스트의 요점이다 — 정지가 실제로 일어났는지는 펌웨어(시뮬레이터) 쪽 로그에서, 이 프로세스가 죽고 한참 뒤의 타임스탬프로 확인해야 한다. 자세한 실행 결과와 재현 절차는 `plan.md`의 "Phase 1 검증 기록" 절에 남겨뒀다.

## scripts/serve-dashboard.mjs

Node 내장 `http`/`fs`만 쓰는, 의존성 없는 정적 파일 서버. `apps/dashboard/index.html`이 `<script type="module">`로 `packages/transport/src/*.js`를 상대 경로로 불러오는데, HTML 파일을 `file://`로 그냥 열면 크로미움이 모듈 스크립트 로딩을 CORS로 막아버리기 때문에 `http://`로 서빙해야 한다. `web/` 디렉터리 전체를 루트로 서빙하고, 확장자에 따라 `Content-Type`을 최소한으로만 맞춰준다(`.js`/`.mjs`는 `text/javascript`로 지정 — 이게 틀리면 브라우저가 모듈로 인식하지 않는다). 기본 포트는 5173, `DASHBOARD_PORT` 환경변수로 바꿀 수 있다.

## apps/dashboard/index.html

이번 단계에서 처음으로 만들어진, 진짜 브라우저에서 여는 페이지. Phase 3의 정식 dashboard가 아니라 "연결성 테스트"용 최소 페이지라는 걸 페이지 안에도 명시해뒀다. Connect 버튼을 누르면 `WebSocketTransport`로 펌웨어 시뮬레이터에 연결하고, 연결되는 즉시 100ms 간격 하트비트 전송을 시작하며, 받은 하트비트 에코의 `seq` 값을 화면에 실시간으로 표시한다. 슬라이더 두 개로 좌우 속도를 골라 `SET_VELOCITY`를 수동으로 보낼 수 있고, 로그 영역에 모든 이벤트가 타임스탬프와 함께 쌓인다.

이 페이지가 존재하는 이유는 단 하나, 지금까지의 모든 검증이 Node.js끼리의 통신이었고 실제 브라우저 탭은 이 흐름에 한 번도 들어온 적이 없었기 때문이다. 이 페이지를 열어 연결한 뒤 탭을 직접 닫아보면(또는 브라우저 전체를 종료해보면), 펌웨어 시뮬레이터 로그에 약 300ms 뒤 ESTOP이 찍히는 걸로 안전 모델이 실제 브라우저 환경에서도 성립하는지 처음으로 확인할 수 있다. `scripts/serve-dashboard.mjs`로 서빙한 뒤 `http://localhost:5173/apps/dashboard/index.html`로 열어야 한다.
