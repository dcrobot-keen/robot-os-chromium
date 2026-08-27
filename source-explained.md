# 소스 코드 설명

이 레포에 실제로 존재하는 코드 파일들을 하나씩 설명한다. 아키텍처 구상이 아니라 "지금 여기 있는 코드가 정확히 무엇을 하는가"를 남기는 문서다. 대부분의 패키지는 아직 `package.json`만 있는 빈 껍데기이고, 실제로 동작하는 코드는 `packages/transport`와 `scripts/prototype-client.mjs`뿐이다.

## package.json / pnpm-workspace.yaml

루트에 있는 pnpm 워크스페이스 정의. `packages/*`와 `apps/*` 아래를 전부 하나의 워크스페이스로 묶는다. 아직 `pnpm install`을 실행한 적은 없다 — 지금까지는 어떤 패키지도 서로를 `import`로 참조하지 않고 `prototype-client.mjs`가 `packages/transport/src`를 상대 경로로 직접 불러오는 방식이라, 워크스페이스 링크가 없어도 동작한다.

## packages/transport/src/commands.js

펌웨어 레포의 `firmware/sim/src/commands.js`와 바이트 하나 다르지 않게 동일한 파일이다. `HEARTBEAT`(0x01), `SET_VELOCITY`(0x02), `ESTOP`(0x03) 세 개만 실제로 쓰이고, `GET_ENCODER`/`GET_IMU`/`GET_BATTERY`는 번호만 예약되어 있다. 두 레포가 서로의 파일을 참조하지 않기 때문에 일부러 복제해뒀고, 프로토콜이 바뀌면 두 파일을 사람이 직접 맞춰야 한다.

## packages/transport/src/frame.js

이것도 `firmware/sim/src/frame.js`와 동일한 코드다. `SOF|LEN|CMD|PAYLOAD|CRC16|EOF` 프레임을 만드는 `encodeFrame()`과, 바이트가 조각나서 들어와도 완성된 프레임 단위로 복원해주는 `FrameDecoder`가 들어있다. 상세한 동작(재동기화 로직, CRC 알고리즘)은 펌웨어 레포의 `source-explained.md`에 이미 적어뒀고, 여기서는 완전히 같은 내용이라 반복하지 않는다.

## packages/transport/src/index.js

지금은 `frame.js`와 `commands.js`를 그대로 재수출(`export *`)하는 것 말고는 하는 일이 없다. 파일 맨 위 주석에 `HardwareTransport` 인터페이스의 모양(`connect`, `send`, `onFrame`, `onDisconnect`)을 문서화해뒀지만, 실제 코드로는 아직 존재하지 않는다 — `WebSerialTransport`, `WebSocketTransport` 등 구체 구현체는 전부 TODO다. 지금의 프로토타입(`scripts/prototype-client.mjs`)은 이 인터페이스를 거치지 않고 `net` 소켓을 직접 다루는데, 이건 인터페이스를 아직 안 만들어서가 아니라 프로토콜 자체의 정확성을 먼저 확인하고 그 다음에 인터페이스를 씌우기 위한 의도적인 순서다.

## packages/{device-abstraction,bus,nodes,rtc}/package.json, apps/{dashboard,signaling-server}/package.json

이 다섯 곳은 전부 `package.json` 하나씩만 있는 빈 자리다. 각 파일의 `description` 필드에 이 패키지가 앞으로 뭘 담을지와 `plan.md`의 어느 단계에서 채워질지를 적어뒀다. 코드는 없다.

## manifests/rover.manifest.json

레퍼런스 로버 하나를 기술하는 매니페스트 예시. `motors.left`/`motors.right`가 각각 `SET_VELOCITY`의 어느 필드에 속도를 쓰고 `GET_ENCODER`의 어느 필드에서 현재 속도를 읽어올지를 매핑하고, `imu.orientation`이 `GET_IMU`에 연결된다. 다만 `GET_ENCODER`와 `GET_IMU`는 펌웨어/시뮬레이터 양쪽 다 아직 구현되어 있지 않으므로, 지금 이 매니페스트는 "나중에 device-abstraction 패키지가 이런 모양의 파일을 읽게 될 것이다"를 보여주는 예시일 뿐 실제로 어디서 로드해서 쓰이고 있지는 않다.

## scripts/prototype-client.mjs

지금 이 레포에서 유일하게 "실행하면 뭔가 실제로 증명되는" 코드다. 브라우저 탭이 나중에 할 일을 TCP 클라이언트로 흉내낸다.

동작 순서는 다음과 같다.

1. `net.connect`로 펌웨어(시뮬레이터)에 연결한다.
2. 100ms 간격으로 `HEARTBEAT` 프레임을 계속 보낸다(`seq` 번호를 매번 증가시키면서).
3. 연결 150ms 시점에 `SET_VELOCITY(0.5, 0.5)` 프레임을 한 번 보낸다.
4. 연결 350ms 시점에 하트비트 전송을 멈추고, `socket.destroy()`로 인사 없이 연결을 끊은 뒤, 로그를 남기고 `process.exit(0)`으로 프로세스 자체를 종료한다.

받은 프레임 중 `HEARTBEAT` 에코는 `seq` 번호와 함께 로그로 찍지만, 그 외에는 아무것도 검증하지 않는다. 이 스크립트가 일부러 하지 않는 일이 하나 있는데, ESTOP이 실제로 발동했는지 스스로 확인하지 않는다는 것이다. 4번 단계에서 프로세스가 완전히 죽어버리기 때문에 애초에 확인할 방법이 없고, 그게 이 테스트의 요점이다 — 정지가 실제로 일어났는지는 이 스크립트가 아니라 펌웨어(시뮬레이터) 쪽 로그에서, 이 프로세스가 죽고 한참 뒤의 타임스탬프로 확인해야 한다. 자세한 실행 결과와 재현 절차는 `plan.md`의 "Phase 1 검증 기록" 절에 남겨뒀다.
