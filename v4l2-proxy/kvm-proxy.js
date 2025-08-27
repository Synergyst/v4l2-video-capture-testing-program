// kvm-proxy.js
//
// MJPEG TCP -> WebSocket proxy + HTML client with keyboard/mouse capture.
// Also forwards browser input events to a destination PC via a TCP "control" connection (JSON lines).
//
// Usage:
//   npm install ws
//   node kvm-proxy.js
//
// Open http://localhost:8080/
//
// ENV:
//   TCP_HOST, TCP_PORT              (MJPEG source)
//   CONTROL_TCP_HOST, CONTROL_TCP_PORT (destination PC control agent)
//   HTTP_PORT (default 8080)
//   AUTH_TOKEN (optional)
//
// JSON-L protocol for control channel (one JSON object per line):
//   Mouse move: {"type":"mouse","action":"move","x":0.123,"y":0.456}
//   Mouse down/up: {"type":"mouse","action":"down","button":0} | {"type":"mouse","action":"up","button":0}
//   Wheel: {"type":"mouse","action":"wheel","deltaX":-10,"deltaY":120}
//   Key down/up: {"type":"keyboard","action":"down","code":"KeyA","key":"a","ctrl":false,"shift":false,"alt":false,"meta":false}
//   Optionally: {"type":"hello","token":"..."} as first message from client when AUTH_TOKEN is set.
//
const http = require('http');
const net = require('net');
const WebSocket = require('ws');

const TCP_HOST = process.env.TCP_HOST || '192.168.168.175';
const TCP_PORT = parseInt(process.env.TCP_PORT || '1337', 10);
const CONTROL_TCP_HOST = process.env.CONTROL_TCP_HOST || '192.168.168.46';
const CONTROL_TCP_PORT = parseInt(process.env.CONTROL_TCP_PORT || '1444', 10);
const HTTP_PORT = parseInt(process.env.HTTP_PORT || '8080', 10);
const WSS_PATH = '/ws';
const AUTH_TOKEN = process.env.AUTH_TOKEN || null;

// --- Embedded HTML client ---
const HTML_PAGE = `<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Remote Viewer + Input</title>
<style>
  :root{color-scheme:dark}
  html,body{height:100%;margin:0;background:#000;overflow:hidden}
  #wrap{position:fixed;inset:0;display:flex;align-items:center;justify-content:center;background:#000}
  #player{max-width:100%;max-height:100%;display:block;user-select:none;cursor:crosshair}
  #hud{position:fixed;left:8px;top:8px;right:auto;display:flex;gap:.5rem;align-items:center;font:14px system-ui;color:#fff;opacity:.95;flex-wrap:wrap}
  #hud button,#hud input[type=checkbox]+label{background:#111;border:1px solid #333;color:#eee;padding:.35rem .6rem;border-radius:6px;cursor:pointer}
  #hud .pill{padding:.35rem .6rem;border-radius:999px;border:1px solid #333;background:#111}
  #hud .status{font-weight:600}
  #hud .dim{opacity:.7}
  #help{position:fixed;right:8px;top:8px;max-width:44ch;background:#111;border:1px solid #333;border-radius:8px;padding:.6rem .8rem;color:#ddd;font:13px system-ui}
  #help h3{margin:.2rem 0 .4rem 0;font-size:14px}
  #help ul{margin:.2rem 0;padding-left:1.1rem}
  #help li{margin:.2rem 0}
  #toasts{position:fixed;left:8px;bottom:8px;display:flex;flex-direction:column;gap:6px;z-index:10}
  .toast{background:#111;border:1px solid #333;color:#eee;padding:.5rem .7rem;border-radius:6px;max-width:60ch;white-space:pre-wrap;word-break:break-word}
  .toast.ok{border-color:#2b5}
  .toast.err{border-color:#d55}
</style>
</head>
<body>
  <div id="wrap">
    <img id="player" alt="MJPEG stream">
  </div>
  <div id="hud">
    <span class="pill">WS: <span id="wsState" class="status">connecting...</span></span>
    <span class="pill">Control: <span id="ctlState" class="status">n/a</span></span>
    <button id="btnCapture">Start Capture</button>
    <button id="btnFull">Fullscreen</button>
    <input id="chkLock" type="checkbox" style="display:none">
    <label for="chkLock" title="Pointer Lock">Lock Cursor</label>
    <span class="pill dim" id="resInfo"></span>
    <!-- New controls -->
    <button id="btnReboot" title="Reboot remote machine">Reboot</button>
    <button id="btnPoweroff" title="Power off remote machine">Poweroff</button>
    <button id="btnSendText" title="Send plain text as keystrokes">Send Text</button>
    <button id="btnShell" title="Execute shell command on agent">Shell</button>
  </div>
  <div id="help">
    <h3>Controls</h3>
    <ul>
      <li>Click "Start Capture" to send mouse/keyboard</li>
      <li>Optional: enable "Lock Cursor" for raw mouse</li>
      <li>Esc releases pointer lock</li>
      <li>F toggles fullscreen • C toggles capture</li>
      <li>Use Reboot/Poweroff/Shell/Send Text via the buttons</li>
    </ul>
  </div>
  <div id="toasts"></div>
<script>
(function(){
  const img = document.getElementById('player');
  const wsState = document.getElementById('wsState');
  const ctlState = document.getElementById('ctlState');
  const resInfo = document.getElementById('resInfo');
  const btnCapture = document.getElementById('btnCapture');
  const chkLock = document.getElementById('chkLock');
  const btnFull = document.getElementById('btnFull');

  const btnReboot = document.getElementById('btnReboot');
  const btnPoweroff = document.getElementById('btnPoweroff');
  const btnSendText = document.getElementById('btnSendText');
  const btnShell = document.getElementById('btnShell');

  const toasts = document.getElementById('toasts');

  function showToast(msg, ok=true) {
    const div = document.createElement('div');
    div.className = 'toast ' + (ok ? 'ok' : 'err');
    div.textContent = msg;
    toasts.appendChild(div);
    setTimeout(() => { div.remove(); }, 6000);
  }

  const loc = window.location;
  const wsProto = (loc.protocol === 'https:') ? 'wss:' : 'ws:';
  const wsUrl = wsProto + '//' + loc.host + '${WSS_PATH}';
  const authToken = new URLSearchParams(location.search).get('token') || null;

  let ws;
  let prevUrl = null;
  let capturing = false;
  let pointerLocked = false;
  let remoteSize = null; // optional info from server/agent

  function setCaptureState(on) {
    capturing = !!on;
    btnCapture.textContent = capturing ? 'Stop Capture' : 'Start Capture';
    if (!capturing && document.pointerLockElement) document.exitPointerLock();
  }

  function setPointerLock(on) {
    if (on && !document.pointerLockElement) {
      img.requestPointerLock?.();
    } else if (!on && document.pointerLockElement) {
      document.exitPointerLock?.();
    }
  }

  // Map client coords to normalized 0..1 based on displayed image rect
  function getNormFromEvent(ev) {
    const rect = img.getBoundingClientRect();
    const x = (ev.clientX - rect.left) / rect.width;
    const y = (ev.clientY - rect.top) / rect.height;
    return { x: Math.min(1, Math.max(0, x)), y: Math.min(1, Math.max(0, y)) };
  }

  function send(msg) {
    if (!ws || ws.readyState !== 1) return;
    try { ws.send(JSON.stringify(msg)); } catch(e){}
  }

  function connectWs() {
    ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => {
      wsState.textContent = 'open';
      if (authToken) send({type:'hello', token: authToken});
    };
    ws.onclose = () => {
      wsState.textContent = 'closed';
      ctlState.textContent = 'n/a';
      setTimeout(connectWs, 1000);
    };
    ws.onerror = (e) => {
      wsState.textContent = 'error';
      ws.close();
    };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') {
        try {
          const msg = JSON.parse(ev.data);
          if (msg.type === 'info') {
            if (msg.remoteSize) {
              remoteSize = msg.remoteSize;
              resInfo.textContent = 'Remote: ' + remoteSize.w + '×' + remoteSize.h;
            }
            if (typeof msg.controlConnected === 'boolean') {
              ctlState.textContent = msg.controlConnected ? 'ready' : 'retrying...';
              if (ctlState.textContent === 'ready') {
                send({type:'relallkeys'});
              }
            }
            if (msg.note) showToast(String(msg.note), true);
            if (msg.error) showToast('Error: ' + String(msg.error), false);
          } else if (msg.type === 'shellResult') {
            const text = 'Shell [' + (msg.id ?? '?') + '] exit=' + (msg.code ?? '?') + '\\n' +
                         (msg.stdout ? ('STDOUT:\\n' + msg.stdout + '\\n') : '') +
                         (msg.stderr ? ('STDERR:\\n' + msg.stderr) : '');
            showToast(text, msg.code === 0);
          }
        } catch {}
        return;
      }
      // binary JPEG
      const blob = new Blob([ev.data], {type:'image/jpeg'});
      const url = URL.createObjectURL(blob);
      img.src = url;
      if (prevUrl) URL.revokeObjectURL(prevUrl);
      prevUrl = url;
      if (img.naturalWidth && img.naturalHeight) {
        resInfo.textContent = 'Remote: ' + img.naturalWidth + '×' + img.naturalHeight + (remoteSize ? ' • Agent: ' + remoteSize.w + '×' + remoteSize.h : '');
      }
    };
  }

  // Input handlers
  function onMouseMove(ev) {
    if (!capturing) return;
    if (document.pointerLockElement === img) {
      send({type:'mouse', action:'moveRelative', dx: ev.movementX, dy: ev.movementY});
    } else {
      const {x,y} = getNormFromEvent(ev);
      send({type:'mouse', action:'move', x, y});
    }
    ev.preventDefault();
  }
  function onMouseDown(ev) {
    if (!capturing) return;
    const btn = ev.button;
    send({type:'mouse', action:'down', button: btn});
    ev.preventDefault();
  }
  function onMouseUp(ev) {
    if (!capturing) return;
    const btn = ev.button;
    send({type:'mouse', action:'up', button: btn});
    ev.preventDefault();
  }
  function onWheel(ev) {
    if (!capturing) return;
    const LINE_HEIGHT = 16;
    let dx = ev.deltaX;
    let dy = ev.deltaY;
    if (ev.deltaMode === 1) { dx *= LINE_HEIGHT; dy *= LINE_HEIGHT; }
    else if (ev.deltaMode === 2) { dx *= 800; dy *= 800; }
    send({type:'mouse', action:'wheel', deltaX: dx, deltaY: dy});
    ev.preventDefault();
  }
  function onKey(ev, action) {
    if (!capturing) return;
    if (action === 'down' && ev.repeat) { ev.preventDefault(); return; }
    send({type:'keyboard', action, code: ev.code, key: ev.key, ctrl: ev.ctrlKey, shift: ev.shiftKey, alt: ev.altKey, meta: ev.metaKey});
    ev.preventDefault();
  }

  // UI bindings
  btnCapture.addEventListener('click', () => setCaptureState(!capturing));
  btnFull.addEventListener('click', () => {
    if (!document.fullscreenElement) document.documentElement.requestFullscreen?.();
    else document.exitFullscreen?.();
  });
  chkLock.addEventListener('change', () => setPointerLock(chkLock.checked));
  document.addEventListener('pointerlockchange', () => {
    pointerLocked = document.pointerLockElement === img;
    chkLock.checked = pointerLocked;
  });

  // New button handlers
  btnReboot.addEventListener('click', () => {
    if (confirm('Send reboot to control agent?')) {
      send({type:'system', action:'reboot'});
    }
  });
  btnPoweroff.addEventListener('click', () => {
    if (confirm('Send poweroff to control agent?')) {
      send({type:'system', action:'poweroff'});
    }
  });
  btnSendText.addEventListener('click', () => {
    const text = prompt('Enter text to send as keystrokes:', '');
    if (text != null && text !== '') {
      send({type:'text', text});
    }
  });
  btnShell.addEventListener('click', () => {
    const cmd = prompt('Shell command to execute on the control agent:', '');
    if (cmd != null && cmd.trim() !== '') {
      const id = Date.now().toString(36);
      send({type:'shell', cmd, id});
    }
  });

  // Global shortcuts
  window.addEventListener('keydown', (ev) => {
    if (ev.code === 'KeyF') { btnFull.click(); ev.preventDefault(); return; }
    if (ev.code === 'KeyC') { btnCapture.click(); ev.preventDefault(); return; }
    if (capturing) onKey(ev, 'down');
  }, {capture:true});
  window.addEventListener('keyup', (ev) => { if (capturing) onKey(ev, 'up'); }, {capture:true});

  // Mouse on image
  img.addEventListener('mousemove', onMouseMove, {passive:false});
  img.addEventListener('mousedown', onMouseDown, {passive:false});
  img.addEventListener('mouseup', onMouseUp, {passive:false});
  img.addEventListener('wheel', onWheel, {passive:false});
  img.addEventListener('contextmenu', (e) => { if (capturing) e.preventDefault(); });

  // Start
  connectWs();
})();
</script>
</body>
</html>`;

// --- HTTP + WebSocket server ---
const httpServer = http.createServer((req, res) => {
  if (req.url === '/' || req.url === '/index.html') {
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
    res.end(HTML_PAGE);
    return;
  }
  res.writeHead(404);
  res.end('Not found');
});

const wss = new WebSocket.Server({ server: httpServer, path: WSS_PATH });

// Track one "controller" client (first that sends hello or first connection)
let controller = null;

// Broadcast JPEG to all connected websocket clients
function broadcastJPEG(buf) {
  if (wss.clients.size === 0) return;
  for (const client of wss.clients) {
    if (client.readyState === WebSocket.OPEN) {
      client.send(buf);
    }
  }
}

// Optionally send info to clients (string JSON)
function broadcastInfo(obj) {
  const msg = JSON.stringify({type:'info', ...obj});
  for (const client of wss.clients) {
    if (client.readyState === WebSocket.OPEN) client.send(msg);
  }
}

wss.on('connection', (ws) => {
  console.log('Browser connected. Clients:', wss.clients.size);

  // push initial status to this client so HUD shows the right Control state
  try {
    ws.send(JSON.stringify({
      type: 'info',
      controlConnected,
      ...(lastRemoteSize ? { remoteSize: lastRemoteSize } : {})
    }));
  } catch {}

  ws.on('close', () => {
    console.log('Browser disconnected. Clients:', wss.clients.size);
    if (ws === controller) controller = null;
  });

  ws.on('message', (data, isBinary) => {
    if (isBinary) return; // we don't expect binary from client
    let msg;
    try { msg = JSON.parse(data.toString('utf8')); } catch { return; }

    // Simple auth and controller selection
    if (msg.type === 'hello') {
      if (AUTH_TOKEN && msg.token !== AUTH_TOKEN) {
        try { ws.send(JSON.stringify({type:'info', error:'auth_failed'})); } catch {}
        ws.close(1008, 'Auth failed');
        return;
      }
      if (!controller) controller = ws;
      return;
    }

    // Only accept input from controller (or first sender)
    if (!controller) controller = ws;
    if (ws !== controller) return;

    // Forward to control TCP
    sendToControl(JSON.stringify(msg) + '\n');
  });
});

// --- TCP: MJPEG source connection and JPEG frame extraction ---
let videoSock = null;
let acc = Buffer.alloc(0);

function connectVideo() {
  console.log(`Connecting to MJPEG TCP ${TCP_HOST}:${TCP_PORT}...`);
  videoSock = net.connect(TCP_PORT, TCP_HOST, () => {
    console.log('Connected to MJPEG source.');
    acc = Buffer.alloc(0);
  });

  videoSock.on('data', (chunk) => {
    acc = Buffer.concat([acc, chunk]);
    // scan for JPEG SOI/EOI markers
    let start = acc.indexOf(Buffer.from([0xff, 0xd8]));
    let end = start >= 0 ? acc.indexOf(Buffer.from([0xff, 0xd9]), start + 2) : -1;
    while (start >= 0 && end >= 0) {
      const jpeg = acc.slice(start, end + 2);
      broadcastJPEG(jpeg);
      acc = acc.slice(end + 2);
      start = acc.indexOf(Buffer.from([0xff, 0xd8]));
      end = start >= 0 ? acc.indexOf(Buffer.from([0xff, 0xd9]), start + 2) : -1;
    }
    // prevent unbounded growth on malformed data
    const MAX_ACC = 1024 * 1024;
    if (acc.length > MAX_ACC) {
      acc = acc.slice(-65536);
    }
  });

  videoSock.on('close', () => {
    console.log('MJPEG TCP closed, retrying in 1s...');
    setTimeout(connectVideo, 1000);
  });

  videoSock.on('error', (err) => {
    console.error('MJPEG TCP error:', err.message);
    videoSock.destroy();
    setTimeout(connectVideo, 1000);
  });
}

// --- TCP: Control connection to destination PC ---
let controlSock = null;
let controlQueue = [];
let controlConnected = false;
let controlReconnectTimer = null;
let lastRemoteSize = null;

function flushControlQueue() {
  if (!controlSock || !controlConnected) return;
  while (controlQueue.length) {
    const line = controlQueue.shift();
    try { controlSock.write(line); } catch (e) { controlQueue.unshift(line); break; }
  }
}

function sendToControl(line) {
  if (controlConnected && controlSock) {
    try { controlSock.write(line); }
    catch { controlQueue.push(line); }
  } else {
    controlQueue.push(line);
  }
}

function scheduleControlReconnect() {
  if (controlReconnectTimer) return;
  broadcastInfo({ controlConnected: false });
  controlReconnectTimer = setTimeout(() => {
    controlReconnectTimer = null;
    connectControl();
  }, 1000);
}

function connectControl() {
  console.log(`Connecting to CONTROL TCP ${CONTROL_TCP_HOST}:${CONTROL_TCP_PORT}...`);
  controlSock = net.connect(CONTROL_TCP_PORT, CONTROL_TCP_HOST, () => {
    controlConnected = true;
    console.log('Connected to CONTROL agent.');
    broadcastInfo({ controlConnected: true, ...(lastRemoteSize ? { remoteSize: lastRemoteSize } : {}) });
    flushControlQueue();
    // Optionally ask for remoteSize
    try { controlSock.write(JSON.stringify({type:'query', what:'remoteSize'}) + '\n'); } catch {}
  });

  controlSock.on('data', (chunk) => {
    // Optional: forward simple info from agent to clients (e.g., remote size)
    // Expect JSON lines
    const lines = chunk.toString('utf8').split(/\\r?\\n/).filter(Boolean);
    for (const line of lines) {
      try {
        const obj = JSON.parse(line);
        if (obj.remoteSize) {
          lastRemoteSize = obj.remoteSize; // NEW: remember
          broadcastInfo({ remoteSize: lastRemoteSize });
        } else if (obj.type === 'info' || obj.type === 'hello') {
          if (obj.remoteSize) {
            lastRemoteSize = obj.remoteSize; // NEW: remember
          }
          broadcastInfo(obj);
        }
      } catch {}
    }
  });

  controlSock.on('close', () => {
    controlConnected = false;
    console.log('CONTROL TCP closed, retrying...');
    scheduleControlReconnect();
  });

  controlSock.on('error', (err) => {
    controlConnected = false;
    console.error('CONTROL TCP error:', err.message);
    try { controlSock.destroy(); } catch {}
    scheduleControlReconnect();
  });
}

// --- Start servers ---
httpServer.listen(HTTP_PORT, () => {
  console.log(`HTTP server listening on http://192.168.168.175:${HTTP_PORT}/`);
  //console.log(`HTTP server listening on http://prv.synergyst.club:32080/`);
  connectVideo();
  connectControl();
});
