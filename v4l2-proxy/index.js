// mjpeg-proxy.js
//
// Single-file TCP -> WebSocket MJPEG proxy + embedded HTML client.
// Usage:
//   npm install ws
//   node mjpeg-proxy.js
//
// Then open http://localhost:8080/

const http = require('http');
const net = require('net');
const WebSocket = require('ws');

const TCP_HOST = process.env.TCP_HOST || '192.168.168.175';
const TCP_PORT = parseInt(process.env.TCP_PORT || '1337', 10);
const HTTP_PORT = parseInt(process.env.HTTP_PORT || '8080', 10);
const WSS_PATH = '/ws';

// minimal HTML client embedded here
const HTML_PAGE = `<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MJPEG over WebSocket</title>
<style>
  html,body{height:100%;margin:0;background:#000;display:flex;align-items:center;justify-content:center}
  img{max-width:100%;max-height:100%;display:block}
</style>
</head>
<body>
  <img id="player" alt="MJPEG stream">
<script>
(function(){
  const img = document.getElementById('player');
  const loc = window.location;
  const wsProto = (loc.protocol === 'https:') ? 'wss:' : 'ws:';
  const wsUrl = wsProto + '//' + loc.host + '${WSS_PATH}';
  let prevUrl = null;
  let ws;

  function connect() {
    ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => console.log('WS connected');
    ws.onmessage = (ev) => {
      const blob = new Blob([ev.data], {type: 'image/jpeg'});
      const url = URL.createObjectURL(blob);
      img.src = url;
      if (prevUrl) URL.revokeObjectURL(prevUrl);
      prevUrl = url;
    };
    ws.onclose = () => {
      console.log('WS closed, reconnecting in 1s');
      setTimeout(connect, 1000);
    };
    ws.onerror = (e) => {
      console.error('WS error', e);
      ws.close();
    };
  }

  connect();
})();
</script>
</body>
</html>`;

// --- HTTP server that serves the HTML and upgrades to WebSocket ---
const server = http.createServer((req, res) => {
  if (req.url === '/' || req.url === '/index.html') {
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
    res.end(HTML_PAGE);
    return;
  }
  res.writeHead(404);
  res.end('Not found');
});

const wss = new WebSocket.Server({ server, path: WSS_PATH });

wss.on('connection', (ws, req) => {
  console.log('Browser connected. Clients:', wss.clients.size);
  ws.on('close', () => console.log('Client disconnected. Clients:', wss.clients.size));
});

// helper to broadcast a JPEG buffer to all connected websocket clients
function broadcastJPEG(buf) {
  if (wss.clients.size === 0) return;
  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(buf);
    }
  });
}

// --- TCP connection and JPEG frame extraction ---
let tcpSocket = null;
let acc = Buffer.alloc(0);

function connectTcp() {
  console.log(`Connecting to TCP ${TCP_HOST}:${TCP_PORT}...`);
  tcpSocket = net.connect(TCP_PORT, TCP_HOST, () => {
    console.log('Connected to TCP MJPEG source.');
    acc = Buffer.alloc(0);
  });

  tcpSocket.on('data', (chunk) => {
    // accumulate and search for JPEG start (0xFFD8) and end (0xFFD9)
    acc = Buffer.concat([acc, chunk]);
    let start = acc.indexOf(Buffer.from([0xff, 0xd8]));
    let end = start >= 0 ? acc.indexOf(Buffer.from([0xff, 0xd9]), start + 2) : -1;

    while (start >= 0 && end >= 0) {
      const jpeg = acc.slice(start, end + 2);
      broadcastJPEG(jpeg);
      acc = acc.slice(end + 2);
      start = acc.indexOf(Buffer.from([0xff, 0xd8]));
      end = start >= 0 ? acc.indexOf(Buffer.from([0xff, 0xd9]), start + 2) : -1;
    }

    // keep acc as any trailing partial data
    // to avoid unbounded growth, if acc grows very large without markers,
    // trim it conservatively (e.g., keep last 1MB). Adjust as needed.
    const MAX_ACC = 1024 * 1024;
    if (acc.length > MAX_ACC) {
      acc = acc.slice(-65536); // keep last 64KB as a fallback
    }
  });

  tcpSocket.on('close', () => {
    console.log('TCP connection closed, retrying in 1s...');
    setTimeout(connectTcp, 1000);
  });

  tcpSocket.on('error', (err) => {
    console.error('TCP socket error:', err.message);
    tcpSocket.destroy();
    setTimeout(connectTcp, 1000);
  });
}

server.listen(HTTP_PORT, () => {
  console.log(`HTTP server listening on http://localhost:${HTTP_PORT}/`);
  connectTcp();
});
