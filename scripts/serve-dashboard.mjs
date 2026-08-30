// Minimal static file server, Node built-ins only (no deps). Serves the
// whole web/ directory so apps/dashboard/index.html can import
// packages/transport/src/*.js by relative path — opening the HTML file
// directly via file:// blocks module imports in Chromium, so it has to be
// served over http:// instead.
//
// Run: node scripts/serve-dashboard.mjs
// Then open: http://localhost:5173/apps/dashboard/index.html

import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const PORT = Number(process.env.DASHBOARD_PORT || 5173);

const MIME = {
  '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript',
  '.json': 'application/json', '.wasm': 'application/wasm',
};

const server = http.createServer((req, res) => {
  const reqPath = decodeURIComponent(req.url.split('?')[0]);
  const filePath = path.join(ROOT, reqPath === '/' ? '/apps/dashboard/index.html' : reqPath);

  if (!filePath.startsWith(ROOT)) {
    res.writeHead(403);
    res.end('forbidden');
    return;
  }

  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404);
      res.end('not found');
      return;
    }
    res.writeHead(200, { 'Content-Type': MIME[path.extname(filePath)] || 'application/octet-stream' });
    res.end(data);
  });
});

server.listen(PORT, () => {
  console.log(`[serve-dashboard] http://localhost:${PORT}/apps/dashboard/index.html`);
});
