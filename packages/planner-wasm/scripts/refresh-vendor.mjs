// Refreshes vendor/pathfinder.wasm + vendor/wasm_exec.js from a sibling
// `pathfinder` checkout's build output. Not part of `npm test` -- this is a
// manual, occasional step for when pathfinder/pathfinder/grid changes and the
// vendored copy here needs to catch up (this package does not rebuild from
// pathfinder's Go source itself, on purpose -- see src/index.js's header
// comment on why the two repos don't share a filesystem path).
//
// Assumes `pathfinder` is cloned as a sibling of `ros-chromium` under the
// same parent directory (the same layout robot-base/plan.md's reproduction
// steps assume for firmware/web, and roboteq-smoke.mjs already relies on for
// ../../robot-base). Run `npm run build:wasm` in that pathfinder checkout
// first so its dist-wasm/ exists.
//
//   node scripts/refresh-vendor.mjs
import { copyFileSync, existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const SRC_DIR = resolve(__dirname, '../../../../../pathfinder/dist-wasm');
const DEST_DIR = resolve(__dirname, '../vendor');

for (const file of ['pathfinder.wasm', 'wasm_exec.js']) {
  const src = resolve(SRC_DIR, file);
  if (!existsSync(src)) {
    console.error(
      `${src} 를 찾을 수 없습니다. 형제 디렉터리에 pathfinder를 clone하고 그 안에서 ` +
        `\`npm run build:wasm\`을 먼저 실행하세요.`
    );
    process.exit(1);
  }
  copyFileSync(src, resolve(DEST_DIR, file));
  console.log(`복사: ${src} -> ${resolve(DEST_DIR, file)}`);
}

console.log('완료. git diff로 실제 바뀐 게 있는지 확인하고 커밋하세요.');
