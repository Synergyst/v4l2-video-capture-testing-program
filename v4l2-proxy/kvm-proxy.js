// kvm-proxy.js
//
// MJPEG TCP -> WebSocket proxy + HTML client with keyboard/mouse capture.
// Also forwards browser input events to a destination PC via a TCP "control" connection (JSON lines).
//
// Usage:
//   npm install ws
//   node kvm-proxy.js
//
// ENV:
//   TCP_HOST, TCP_PORT
//   CONTROL_TCP_HOST, CONTROL_TCP_PORT
//   HTTP_PORT (default 8080)
//   AUTH_TOKEN (optional)
//   ALLOWED_CONTROL_SUBNETS (optional)
//   TRUSTED_PROXIES (optional)
//   FALLBACK_IMAGE_URL / FALLBACK_MJPEG_URL (optional defaults)

const http = require('http');
const net = require('net');
const WebSocket = require('ws');

const TCP_HOST = process.env.TCP_HOST || '192.168.168.175';
const TCP_PORT = parseInt(process.env.TCP_PORT || '1337', 10);
const CONTROL_TCP_HOST = process.env.CONTROL_TCP_HOST || '192.168.168.46';
const CONTROL_TCP_PORT = parseInt(process.env.CONTROL_TCP_PORT || '1444', 10);
const HTTP_PORT = parseInt(process.env.HTTP_PORT || '8080', 10);

// Comma-delimited whitelist (CIDR or single IP or exact IPv6).
const ALLOWED_CONTROL_SUBNETS = (process.env.ALLOWED_CONTROL_SUBNETS || '192.168.168.0/24,127.0.0.1,75.132.12.230,68.70.17.159').trim();

const WSS_PATH = '/ws';
const AUTH_TOKEN = process.env.AUTH_TOKEN || null;

// --- Exponential backoff configuration for reconnections ---
const VIDEO_RETRY_BASE = 1000;   // 1s
const VIDEO_RETRY_MAX = 3000;    // 3s
const CONTROL_RETRY_BASE = 1000; // 1s
const CONTROL_RETRY_MAX = 3000;  // 3s

let videoRetry = { attempts: 0, timer: null };
let controlRetry = { attempts: 0, timer: null };
let lastRemoteSize = null;
let videoConnected = false;

const DEFAULT_FALLBACK_IMAGE = process.env.FALLBACK_IMAGE_URL || 'https://totallynotbombcodes.synergyst.club/nosignal.png';
const DEFAULT_FALLBACK_MJPEG = process.env.FALLBACK_MJPEG_URL || '';

// --- Embedded HTML client ---
const HTML_PAGE = `<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MJPEG over WebSocket + Input</title>
<style>
  :root{color-scheme:dark}
  html,body{height:100%;margin:0;background:#000;overflow:hidden}
  #wrap{position:fixed;inset:0;display:flex;align-items:center;justify-content:center;background:#000}
  #player{max-width:100%;max-height:100%;display:block;user-select:none;cursor:crosshair}
  #hud{position:fixed;left:8px;top:8px;right:auto;display:flex;gap:.5rem;align-items:center;font:14px system-ui;color:#fff;opacity:.95;flex-wrap:wrap;z-index:20}
  #hud button,#hud input[type=checkbox]+label{background:#111;border:1px solid #333;color:#eee;padding:.35rem .6rem;border-radius:6px;cursor:pointer}
  #hud .pill{padding:.35rem .6rem;border-radius:999px;border:1px solid #333;background:#111}
  #hud .status{font-weight:600}
  #hud .dim{opacity:.7}
  #help{position:fixed;right:8px;top:8px;max-width:44ch;background:#111;opacity:.68;border:1px solid #333;border-radius:8px;padding:.6rem .8rem;color:#ddd;font:13px system-ui;z-index:19}
  #help h3{margin:.2rem 0 .4rem 0;font-size:14px;opacity:1.0}
  #help ul{margin:.2rem 0;padding-left:1.1rem;opacity:1.0}
  #help li{margin:.2rem 0;opacity:1.0}
  #toasts{position:fixed;left:8px;bottom:8px;display:flex;flex-direction:column;gap:6px;z-index:30}
  .toast{background:#111;border:1px solid #333;color:#eee;padding:.5rem .7rem;border-radius:6px;max-width:60ch;white-space:pre-wrap;word-break:break-word;opacity:1;transition:opacity 0.6s ease-out}
  .toast.ok{border-color:#2b5}
  .toast.err{border-color:#d55}
  .small{font-size:12px;padding:.2rem .4rem}
  input[type="text"]{background:#111;border:1px solid #333;color:#eee;padding:.2rem .4rem;border-radius:4px}
  select{background:#111;border:1px solid #333;color:#eee;padding:.2rem .4rem;border-radius:4px}
  #btnHelp{background:#111;border:1px solid #333;color:#0af;padding:.35rem .6rem;border-radius:6px;cursor:pointer}
  #vfURL.hidden{display:none}
</style>
</head>
<body>
  <div id="wrap">
    <img id="player" alt="MJPEG stream" src="${DEFAULT_FALLBACK_MJPEG || DEFAULT_FALLBACK_IMAGE || ''}">
  </div>

  <div id="hud">
    <span class="pill">WS: <span id="wsState" class="status">connecting...</span></span>
    <span class="pill">Video: <span id="vidState" class="status">stream lost</span></span>
    <span class="pill">Control: <span id="ctlState" class="status">agent lost</span></span>
    <button id="btnCapture">Start Capture</button>
    <button id="btnFull">Fullscreen</button>
    <input id="chkLock" type="checkbox" style="display:none">
    <label for="chkLock" title="Pointer Lock">Lock Cursor</label>

    <!-- Video fallback controls -->
    <!--<label class="small" style="margin-left:6px;color:#ddd">No signal:</label>-->
    <select id="vfMode" class="small" title="Video fallback mode">
      <option value="keep">Last frame</option>
      <option value="image">Image URL</option>
      <option value="mjpeg">Alt stream</option>
    </select>
    <input id="vfURL" type="text" placeholder="Fallback URL (image or MJPEG)" class="small vfURL" style="min-width:20px" size="2">
    <button id="vfApply" class="small" style="display:none">Apply</button>

    <!-- Remember preferences -->
    <label style="color:#ddd" class="small"><input id="chkAutoCapture" type="checkbox"> Auto Capture</label>
    <label style="color:#ddd" class="small"><input id="chkRememberLock" type="checkbox"> Remember Lock</label>
    <span class="pill dim" id="resInfo"></span>

    <!-- Actions -->
    <button id="btnReboot" title="Reboot remote machine">Reboot</button>
    <button id="btnPoweroff" title="Power off remote machine">Poweroff</button>
    <button id="btnSendText" title="Send plain text as keystrokes">Send Text</button>
    <button id="btnShell" title="Execute shell command on agent">Shell</button>
    <button id="btnRelAllKeys" title="Releases all keys which the USB HID may have held down">Release keys</button>

    <!-- Help toggle (appears when help pane is closed) -->
    <button id="btnHelp" class="small" style="display:none">Help</button>
  </div>

  <div id="help">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <h3 style="margin:0">Controls</h3>
      <button id="helpClose" style="background:#111;color:#f66;border:1px solid #333;border-radius:6px;padding:.2rem .4rem">Close</button>
    </div>
    <ul>
      <li>Click "Start Capture" to send mouse/keyboard</li>
      <li>Optional: enable "Lock Cursor" for raw mouse (browser may require a gesture)</li>
      <li>Esc releases pointer lock</li>
      <li>Use Reboot/Poweroff/Shell/Send Text via the buttons</li>
      <li>When video goes away, pick a fallback (Last frame / Image / Alt MJPEG)</li>
    </ul>
  </div>

  <div id="toasts"></div>

<script>
(function(){
  // DOM elements
  const img = document.getElementById('player');
  const wsState = document.getElementById('wsState');
  const vidState = document.getElementById('vidState');
  const ctlState = document.getElementById('ctlState');
  const resInfo = document.getElementById('resInfo');
  const btnCapture = document.getElementById('btnCapture');
  const chkLock = document.getElementById('chkLock');
  const chkAutoCapture = document.getElementById('chkAutoCapture');
  const chkRememberLock = document.getElementById('chkRememberLock');
  const btnFull = document.getElementById('btnFull');
  const btnReboot = document.getElementById('btnReboot');
  const btnPoweroff = document.getElementById('btnPoweroff');
  const btnSendText = document.getElementById('btnSendText');
  const btnShell = document.getElementById('btnShell');
  const btnRelAllKeys = document.getElementById('btnRelAllKeys');
  const vfModeSel = document.getElementById('vfMode');
  const vfURLInput = document.getElementById('vfURL');
  const vfApply = document.getElementById('vfApply');
  const toasts = document.getElementById('toasts');
  const helpPane = document.getElementById('help');
  const helpClose = document.getElementById('helpClose');
  const btnHelp = document.getElementById('btnHelp');

  // showToast with fade-out only (no fade-in). duration default 6000ms
  function showToast(msg, ok=true, duration=6000) {
    const div = document.createElement('div');
    div.className = 'toast ' + (ok ? 'ok' : 'err');
    div.textContent = msg;
    div.style.opacity = '1';
    toasts.appendChild(div);
    // schedule fade-out before removal
    const fadeMs = 600; // fade length
    const visibleMs = Math.max(0, duration - fadeMs);
    setTimeout(() => {
      // start fade-out
      div.style.opacity = '0';
      setTimeout(() => {
        try { div.remove(); } catch (e) {}
      }, fadeMs + 20);
    }, visibleMs);
  }

  // Default placeholder
  const DEFAULT_PLACEHOLDER_DATAURL = 'data:image/svg+xml;charset=utf-8,' + encodeURIComponent(
    '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360" viewBox="0 0 640 360">' +
      '<rect width="100%" height="100%" fill="#000"/>' +
      '<text x="50%" y="50%" fill="#ddd" font-family="system-ui,Arial" font-size="20" text-anchor="middle" dominant-baseline="middle">No video signal</text>' +
    '</svg>'
  );

  const loc = window.location;
  const wsProto = (loc.protocol === 'https:') ? 'wss:' : 'ws:';
  const wsUrl = wsProto + '//' + loc.host + '${WSS_PATH}';
  const authToken = new URLSearchParams(location.search).get('token') || null;

  // state
  let ws = null;
  let prevUrl = null;
  let lastFrameUrl = null;
  let capturing = false;
  let pointerLocked = false;
  let remoteSize = null;
  let lastClientPos = null; // {x: px, y: px} within img element when unlocked
  let captureWantedOnFocus = false;
  let pendingWantedCapture = false;

  // preferences
  const PREF_VF_MODE = 'vf.mode';
  const PREF_VF_URL = 'vf.url';
  const PREF_AUTO_CAPTURE = 'vf.autoCapture';
  const PREF_REMEMBER_LOCK = 'vf.rememberLock';
  // remember Help panel visibility
  const PREF_HELP_OPEN = 'ui.helpOpen';
  const savedHelpOpen = localStorage.getItem(PREF_HELP_OPEN);
  if (savedHelpOpen === '0') {
    helpPane.style.display = 'none';
    btnHelp.style.display = 'inline-block';
  }
  let fallbackMode = localStorage.getItem(PREF_VF_MODE) || ( '${DEFAULT_FALLBACK_MJPEG ? 'mjpeg' : (DEFAULT_FALLBACK_IMAGE ? 'image' : 'keep')}' );
  let fallbackURL = localStorage.getItem(PREF_VF_URL) || '${DEFAULT_FALLBACK_MJPEG || DEFAULT_FALLBACK_IMAGE || ''}';
  let autoCapturePref = localStorage.getItem(PREF_AUTO_CAPTURE) === '1';
  let rememberLockPref = localStorage.getItem(PREF_REMEMBER_LOCK) === '1';

  // init UI from prefs
  vfModeSel.value = fallbackMode;
  vfURLInput.value = fallbackURL;
  chkAutoCapture.checked = autoCapturePref;
  chkRememberLock.checked = rememberLockPref;
  // helpers to show/hide Apply, URL input
  function setApplyVisible(v) { vfApply.style.display = v ? 'inline-block' : 'none'; }
  function updateUrlVisibility() {
    if (vfModeSel.value === 'keep') {
      vfURLInput.classList.add('hidden');
    } else {
      vfURLInput.classList.remove('hidden');
    }
  }
  // compute whether UI values differ from stored prefs
  function checkPrefsChanged() {
    const modeDifferent = vfModeSel.value !== fallbackMode;
    const urlDifferent = (vfURLInput.value.trim() !== (fallbackURL || '').trim());
    setApplyVisible(modeDifferent || urlDifferent);
  }
  // wire initial visibility
  updateUrlVisibility();
  checkPrefsChanged();

  function setCaptureState(on) {
    capturing = !!on;
    btnCapture.textContent = capturing ? 'Stop Capture' : 'Start Capture';
    if (!capturing && document.pointerLockElement) document.exitPointerLock();
  }

  function setPointerLock(on) {
    if (on && !document.pointerLockElement) img.requestPointerLock?.();
    else if (!on && document.pointerLockElement) document.exitPointerLock?.();
  }

  /*function getNormFromEvent(ev) {
    const rect = img.getBoundingClientRect();
    const x = (ev.clientX - rect.left) / rect.width;
    const y = (ev.clientY - rect.top) / rect.height;
    return { x: Math.min(1, Math.max(0, x)), y: Math.min(1, Math.max(0, y)) };
  }*/
  function getNormFromEvent(ev) {
    const rect = img.getBoundingClientRect();
    // If the event target is the img, offsetX/offsetY are already relative to the image.
    if (ev.target === img && typeof ev.offsetX === 'number' && typeof ev.offsetY === 'number') {
      const nx = ev.offsetX / (img.clientWidth || rect.width || 1);
      const ny = ev.offsetY / (img.clientHeight || rect.height || 1);
      return { x: Math.min(1, Math.max(0, nx)), y: Math.min(1, Math.max(0, ny)) };
    }
    // Fallback to clientX/clientY relative to image bounding rect
    const cx = ev.clientX, cy = ev.clientY;
    const nx = (cx - rect.left) / (rect.width || 1);
    const ny = (cy - rect.top) / (rect.height || 1);
    return { x: Math.min(1, Math.max(0, nx)), y: Math.min(1, Math.max(0, ny)) };
  }

  function send(msg) {
    if (!ws || ws.readyState !== 1) return;
    try { ws.send(JSON.stringify(msg)); } catch (e) {}
  }

  function sendRelAllKeys() {
    send({type:'relallkeys'});
    send({type:'keyboard', action:'up', code:'AltLeft', key:'Alt', ctrl:false, shift:false, alt:false, meta:false});
    send({type:'keyboard', action:'up', code:'AltRight', key:'Alt', ctrl:false, shift:false, alt:false, meta:false});
    setCaptureState(false);
    showToast('Released all keys and stopped capture', true);
  }

  function applyFallback() {
    if (videoConnected) return;
    if (fallbackMode === 'keep') {
      if (lastFrameUrl) img.src = lastFrameUrl;
      return;
    }
    const url = (fallbackURL && fallbackURL.trim()) ? fallbackURL.trim() : DEFAULT_PLACEHOLDER_DATAURL;
    if (prevUrl && prevUrl.startsWith('blob:')) {
      try { URL.revokeObjectURL(prevUrl); } catch (e) {}
      prevUrl = null;
    }
    img.src = url;
    prevUrl = url;
  }

  function setFallback(mode, url) {
    fallbackMode = mode;
    fallbackURL = url || '';
    localStorage.setItem(PREF_VF_MODE, fallbackMode);
    localStorage.setItem(PREF_VF_URL, fallbackURL);
    applyFallback();
    checkPrefsChanged();
    showToast('Fallback set: ' + fallbackMode + (fallbackURL ? ' -> ' + fallbackURL : ''), true);
  }

  function saveAutoCapturePref(v) {
    autoCapturePref = !!v;
    localStorage.setItem(PREF_AUTO_CAPTURE, autoCapturePref ? '1' : '0');
  }
  function saveRememberLockPref(v) {
    rememberLockPref = !!v;
    localStorage.setItem(PREF_REMEMBER_LOCK, rememberLockPref ? '1' : '0');
  }

  // WS connect
  function connectWs() {
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) return;
    ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer';
    ws.onopen = () => {
      wsState.textContent = 'online';
      if (authToken) send({type:'hello', token: authToken});
      if (pendingWantedCapture) {
        pendingWantedCapture = false;
        if (autoCapturePref || captureWantedOnFocus) {
          try {
            setCaptureState(true);
            if (rememberLockPref) setPointerLock(true);
            showToast('Auto-restored capture after focus', true);
          } catch (e) { console.warn('Auto-restore failed', e && e.message); }
        }
      }
    };
    ws.onclose = () => {
      wsState.textContent = 'offline';
      ctlState.textContent = 'agent lost';
      videoConnected = false;
      vidState.textContent = 'stream lost';
      applyFallback();
      // reconnect WS
      setTimeout(connectWs, 1000);
    };
    ws.onerror = (e) => {
      wsState.textContent = 'error';
      try { ws.close(); } catch (_) {}
    };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') {
        let msg = null;
        try { msg = JSON.parse(ev.data); } catch (e) { return; }
        if (msg.type === 'info') {
          if (msg.remoteSize) {
            remoteSize = msg.remoteSize;
            resInfo.textContent = 'Remote: ' + remoteSize.w + '×' + remoteSize.h;
          }
          if (typeof msg.videoConnected === 'boolean') {
            videoConnected = !!msg.videoConnected;
            vidState.textContent = videoConnected ? 'online' : 'retrying...';
            if (!videoConnected) applyFallback();
          }
          if (typeof msg.allowedControl === 'boolean') {
            allowedControl = !!msg.allowedControl;
          }
          if (typeof msg.controlConnected === 'boolean') {
            controlIsConnected = !!msg.controlConnected;
          }
          if (controlIsConnected !== null) {
            ctlState.textContent = controlIsConnected ? (allowedControl === false ? 'online+banned' : 'online') : 'retrying...';
          }
          if (msg.startCapture && (autoCapturePref || !AUTH_TOKEN)) {
            try {
              setCaptureState(true);
              if (rememberLockPref) setPointerLock(true);
            } catch (e) {}
          }
          if (msg.note) showToast(String(msg.note), true);
          if (msg.error) showToast('Error: ' + String(msg.error), false);
        } else if (msg.type === 'shellResult') {
          const text = 'Shell [' + (msg.id ?? '?') + '] exit=' + (msg.code ?? '?') + '\\n' +
            (msg.stdout ? ('STDOUT:\\n' + msg.stdout) : '') +
            (msg.stderr ? ('\\nSTDERR:\\n' + msg.stderr) : '');
          showToast(text, (msg.code || 0) === 0);
        }
        return;
      }
      // binary JPEG
      try {
        const blob = new Blob([ev.data], {type:'image/jpeg'});
        const url = URL.createObjectURL(blob);
        img.src = url;
        if (prevUrl && prevUrl !== url) {
          try { URL.revokeObjectURL(prevUrl); } catch(_) {}
        }
        prevUrl = url;
        lastFrameUrl = url;
        videoConnected = true;
        vidState.textContent = 'online';
      } catch (e) {
        console.error('Failed to handle frame', e);
      }
    };
  }

  // Event handlers & UI wiring
  btnCapture.addEventListener('click', () => { setCaptureState(!capturing); });

  img.addEventListener('mouseleave', () => { lastClientPos = null; });

  helpClose.addEventListener('click', () => {
    helpPane.style.display = 'none';
    btnHelp.style.display = 'inline-block';
    localStorage.setItem(PREF_HELP_OPEN, '0');
  });

  btnHelp.addEventListener('click', () => {
    helpPane.style.display = 'block';
    btnHelp.style.display = 'none';
    localStorage.setItem(PREF_HELP_OPEN, '1');
  });

  // Show fallback whenever the image errors (e.g., before first frame or broken src)
  img.addEventListener('error', () => {
    if (!videoConnected) applyFallback();
  });

  btnFull.addEventListener('click', () => {
    if (!document.fullscreenElement) document.documentElement.requestFullscreen?.();
    else document.exitFullscreen?.();
  });

  chkLock.addEventListener('change', () => {
    setPointerLock(chkLock.checked);
    if (rememberLockPref) saveRememberLockPref(true);
  });

  chkAutoCapture.addEventListener('change', () => saveAutoCapturePref(chkAutoCapture.checked));
  chkRememberLock.addEventListener('change', () => saveRememberLockPref(chkRememberLock.checked));

  document.addEventListener('pointerlockchange', () => {
    pointerLocked = document.pointerLockElement === img;
    chkLock.checked = pointerLocked;
    // Reset the unlocked tracking baseline when lock state changes
    lastClientPos = null;
  });

  btnReboot.addEventListener('click', () => { if (confirm('Send reboot to control agent?')) send({type:'system', action:'reboot'}); });
  btnPoweroff.addEventListener('click', () => { if (confirm('Send poweroff to control agent?')) send({type:'system', action:'poweroff'}); });
  btnSendText.addEventListener('click', () => {
    const text = prompt('Enter text to send as keystrokes:', '');
    if (text != null && text !== '') send({type:'text', text});
  });
  btnShell.addEventListener('click', () => {
    const cmd = prompt('Shell command to execute on the control agent:', '');
    if (cmd != null && cmd.trim() !== '') {
      const id = Date.now().toString(36);
      send({type:'shell', cmd, id});
    }
  });
  btnRelAllKeys.addEventListener('click', () => { if (confirm('Release all potentially held down keys?')) sendRelAllKeys(); });

  // fallback Apply visibility logic
  vfModeSel.addEventListener('change', () => {
    updateUrlVisibility();
    checkPrefsChanged();
  });
  vfURLInput.addEventListener('input', () => checkPrefsChanged());
  vfApply.addEventListener('click', () => {
    setFallback(vfModeSel.value, vfURLInput.value.trim());
    // after apply, hide Apply button
    setApplyVisible(false);
  });

  // Help panel show/hide logic
  helpClose.addEventListener('click', () => {
    helpPane.style.display = 'none';
    btnHelp.style.display = 'inline-block';
  });
  btnHelp.addEventListener('click', () => {
    helpPane.style.display = 'block';
    btnHelp.style.display = 'none';
  });

  // keyboard/focus safety + focus restore logic
  window.addEventListener('blur', () => {
    if (capturing) { captureWantedOnFocus = true; sendRelAllKeys(); }
    else captureWantedOnFocus = false;
  });

  document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
      if (capturing) { captureWantedOnFocus = true; sendRelAllKeys(); }
      else captureWantedOnFocus = false;
    } else {
      if (autoCapturePref || captureWantedOnFocus) {
        if (ws && ws.readyState === WebSocket.OPEN) {
          setCaptureState(true);
          if (rememberLockPref) setPointerLock(true);
          showToast('Capture restored on visibility', true);
          captureWantedOnFocus = false;
        } else {
          pendingWantedCapture = true;
          if (!ws || ws.readyState === WebSocket.CLOSED) connectWs();
        }
      }
    }
  });

  window.addEventListener('focus', () => {
    if (autoCapturePref || captureWantedOnFocus) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        setCaptureState(true);
        if (rememberLockPref) setPointerLock(true);
        showToast('Capture restored on focus', true);
        captureWantedOnFocus = false;
      } else {
        pendingWantedCapture = true;
        if (!ws || ws.readyState === WebSocket.CLOSED) connectWs();
      }
    }
  });

  window.addEventListener('pagehide', () => { if (capturing) sendRelAllKeys(); });
  window.addEventListener('beforeunload', () => {
    if (capturing) {
      try { send({ type: 'relallkeys' }); } catch (e) {}
      setCaptureState(false);
    }
  });

  document.addEventListener('pointerlockchange', () => { if (!document.pointerLockElement && capturing) sendRelAllKeys(); });

  window.addEventListener('keydown', (ev) => { if (capturing) onKey(ev, 'down'); }, {capture:true});
  window.addEventListener('keyup', (ev) => { if (capturing) onKey(ev, 'up'); }, {capture:true});

  // mouse handlers
  img.addEventListener('mousemove', onMouseMove, {passive:false});
  img.addEventListener('mousedown', onMouseDown, {passive:false});
  img.addEventListener('mouseup', onMouseUp, {passive:false});
  img.addEventListener('wheel', onWheel, {passive:false});
  img.addEventListener('contextmenu', (e) => { if (capturing) e.preventDefault(); });

  // start
  connectWs();
  updateUrlVisibility();
  checkPrefsChanged();
  if (!videoConnected) applyFallback();

  // helper: show/hide Apply + url input utilities already declared above
  function setApplyVisible(v) { vfApply.style.display = v ? 'inline-block' : 'none'; }
  function updateUrlVisibility() {
    if (vfModeSel.value === 'keep') vfURLInput.classList.add('hidden');
    else vfURLInput.classList.remove('hidden');
  }
  function checkPrefsChanged() {
    const modeDifferent = vfModeSel.value !== fallbackMode;
    const urlDifferent = (vfURLInput.value.trim() !== (fallbackURL || '').trim());
    setApplyVisible(modeDifferent || urlDifferent);
  }

  // forwarding functions for input handlers defined earlier in this file (they reference send)
  /*function onMouseMove(ev) {
    if (!capturing) return;
    if (document.pointerLockElement === img) {
      // movementX/Y are in CSS pixels of the displayed image area.
      // Scale them to remote framebuffer pixels using the last-known remoteSize and the displayed image rect.
      let dx = ev.movementX || 0;
      let dy = ev.movementY || 0;
      if (remoteSize && remoteSize.w && remoteSize.h) {
        const rect = img.getBoundingClientRect();
        // Avoid division by zero; fall back to 1:1 if rect width/height are 0
        const scaleX = rect.width > 0 ? (remoteSize.w / rect.width) : 1;
        const scaleY = rect.height > 0 ? (remoteSize.h / rect.height) : 1;
        dx = Math.trunc(dx * scaleX);
        dy = Math.trunc(dy * scaleY);
      }
      send({type:'mouse', action:'moveRelative', dx: dx, dy: dy});
    } else {
      const {x,y} = getNormFromEvent(ev);
      send({type:'mouse', action:'move', x, y});
    }
    ev.preventDefault();
  }*/
  /*function onMouseMove(ev) {
    if (!capturing) return;

    if (document.pointerLockElement === img) {
      // movementX/Y are in CSS pixels; scale them to remote framebuffer pixels
      let dx = ev.movementX || 0;
      let dy = ev.movementY || 0;
      const rect = img.getBoundingClientRect();

      // Prefer authoritative remoteSize if available; otherwise fall back to image natural size
      let scaleX = 1, scaleY = 1;
      if (remoteSize && remoteSize.w && remoteSize.h) {
        if (rect.width > 0) scaleX = remoteSize.w / rect.width;
        if (rect.height > 0) scaleY = remoteSize.h / rect.height;
      } else if (img.naturalWidth && img.naturalHeight && rect.width && rect.height) {
        scaleX = img.naturalWidth / rect.width;
        scaleY = img.naturalHeight / rect.height;
      }

      dx = Math.trunc(dx * scaleX);
      dy = Math.trunc(dy * scaleY);

      send({type:'mouse', action:'moveRelative', dx: dx, dy: dy});
    } else {
      // Unlocked mode: send absolute normalized coords relative to the displayed image
      const {x, y} = getNormFromEvent(ev);
      send({type:'mouse', action:'move', x, y});
    }
    ev.preventDefault();
  }*/
  /*function onMouseMove(ev) {
  if (!capturing) return;

  const rect = img.getBoundingClientRect();

  if (document.pointerLockElement === img) {
    // pointer lock: use movementX/Y scaled from displayed image -> remote pixels
    let dx = ev.movementX || 0;
    let dy = ev.movementY || 0;
    if (remoteSize && remoteSize.w && remoteSize.h && rect.width > 0 && rect.height > 0) {
      const scaleX = remoteSize.w / rect.width;
      const scaleY = remoteSize.h / rect.height;
      dx = Math.trunc(dx * scaleX);
      dy = Math.trunc(dy * scaleY);
    }
    if (dx !== 0 || dy !== 0) send({ type:'mouse', action:'moveRelative', dx, dy });
  } else {
    // unlocked: compute client pixel position inside the image and send deltas (relative)
    const clientX = ev.clientX - rect.left;
    const clientY = ev.clientY - rect.top;
    // clamp to image rect
    const cx = Math.max(0, Math.min(rect.width, clientX));
    const cy = Math.max(0, Math.min(rect.height, clientY));

    if (!lastClientPos) {
      // First sample after entering / baseline: store and don't send a spurious large delta
      lastClientPos = { x: cx, y: cy };
      return;
    }

    const dxPix = cx - lastClientPos.x;
    const dyPix = cy - lastClientPos.y;
    lastClientPos.x = cx;
    lastClientPos.y = cy;

    if (dxPix === 0 && dyPix === 0) {
      ev.preventDefault();
      return;
    }

    let dx = Math.trunc(dxPix);
    let dy = Math.trunc(dyPix);
    if (remoteSize && remoteSize.w && remoteSize.h && rect.width > 0 && rect.height > 0) {
      const scaleX = remoteSize.w / rect.width;
      const scaleY = remoteSize.h / rect.height;
      dx = Math.trunc(dxPix * scaleX);
      dy = Math.trunc(dyPix * scaleY);
    }

    if (dx !== 0 || dy !== 0) send({ type:'mouse', action:'moveRelative', dx, dy });
  }
  ev.preventDefault();
}*/
function onMouseMove(ev) {
  if (!capturing) return;
  const rect = img.getBoundingClientRect();

  if (document.pointerLockElement === img) {
    // pointer-locked: send relative deltas scaled to remote framebuffer pixels
    let dx = ev.movementX || 0;
    let dy = ev.movementY || 0;
    if (remoteSize && remoteSize.w && remoteSize.h && rect.width > 0 && rect.height > 0) {
      const scaleX = remoteSize.w / rect.width;
      const scaleY = remoteSize.h / rect.height;
      dx = Math.trunc(dx * scaleX);
      dy = Math.trunc(dy * scaleY);
    }
    if (dx !== 0 || dy !== 0) send({ type: 'mouse', action: 'moveRelative', dx, dy });
  } else {
    // unlocked: compute normalized coordinates inside the MJPEG image element and send absolute move
    const clientX = ev.clientX - rect.left;
    const clientY = ev.clientY - rect.top;
    const cx = rect.width > 0 ? Math.max(0, Math.min(rect.width, clientX)) : 0;
    const cy = rect.height > 0 ? Math.max(0, Math.min(rect.height, clientY)) : 0;
    const nx = rect.width > 0 ? (cx / rect.width) : 0;
    const ny = rect.height > 0 ? (cy / rect.height) : 0;
    send({ type: 'mouse', action: 'move', x: nx, y: ny });

    if (ws && ws.readyState === WebSocket.OPEN) {
      try {
        ws.send(JSON.stringify({
          type: 'clientLog',
          mousePage: { x: ev.clientX, y: ev.clientY },
          mouseImg:  { x: Math.round(cx), y: Math.round(cy) }
        }));
      } catch (e) { /* ignore logging failures */ }
    }
  }
  ev.preventDefault();
}
  function onMouseDown(ev) {
    if (!capturing) return;
    send({type:'mouse', action:'down', button: ev.button});
    ev.preventDefault();
  }
  function onMouseUp(ev) {
    if (!capturing) return;
    send({type:'mouse', action:'up', button: ev.button});
    ev.preventDefault();
  }
  function onWheel(ev) {
    if (!capturing) return;
    const LINE_HEIGHT = 16;
    let dx = ev.deltaX, dy = ev.deltaY;
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

})(); // end IIFE
</script>
</body>
</html>`;

// ---------- server code ----------

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

// controller, broadcast helpers
let controller = null;
function broadcastJPEG(buf) {
  if (wss.clients.size === 0) return;
  for (const client of wss.clients) {
    if (client.readyState === WebSocket.OPEN) client.send(buf);
  }
}
function broadcastInfo(obj) {
  const msg = JSON.stringify({ type: 'info', ...obj });
  for (const client of wss.clients) {
    if (client.readyState === WebSocket.OPEN) client.send(msg);
  }
}

// ----- IP helpers (trusted proxies, CIDR whitelist) -----
const TRUSTED_PROXIES = (process.env.TRUSTED_PROXIES || '127.0.0.1,::1,192.168.168.170,192.168.168.178').split(',').map(s => s.trim()).filter(Boolean);

function normalizeRemoteIp(ip) {
  if (!ip) return '';
  if (ip.includes(',')) ip = ip.split(',')[0].trim();
  if (ip.startsWith('::ffff:')) return ip.substring(7);
  const pct = ip.indexOf('%');
  if (pct !== -1) ip = ip.substring(0, pct);
  return ip;
}
function parseXFF(xffHeader) {
  if (!xffHeader) return '';
  const parts = String(xffHeader).split(',').map(p => p.trim()).filter(Boolean);
  return parts.length ? parts[0] : '';
}
function getClientIp(req) {
  let peer = req.socket && req.socket.remoteAddress ? String(req.socket.remoteAddress) : '';
  peer = normalizeRemoteIp(peer);
  if (peer && TRUSTED_PROXIES.includes(peer)) {
    const xff = req.headers['x-forwarded-for'] || req.headers['x-real-ip'];
    const client = normalizeRemoteIp(parseXFF(xff));
    if (client) return client;
    return peer;
  }
  return peer;
}

// IPv4/CIDR helpers
function ipv4ToInt(ip) {
  const m = ip.split('.');
  if (m.length !== 4) return null;
  const a = Number(m[0]), b = Number(m[1]), c = Number(m[2]), d = Number(m[3]);
  if ([a,b,c,d].some(x => Number.isNaN(x) || x < 0 || x > 255)) return null;
  return ((a << 24) >>> 0) | ((b << 16) >>> 0) | ((c << 8) >>> 0) | (d >>> 0);
}
function makeMask(prefix) {
  if (prefix <= 0) return 0 >>> 0;
  if (prefix >= 32) return 0xffffffff >>> 0;
  return ((0xffffffff << (32 - prefix)) >>> 0);
}
function parseAllowedControlSubnets(str) {
  const out = [];
  if (!str) return out;
  for (const raw of str.split(',')) {
    const token = raw.trim();
    if (!token) continue;
    if (token.includes('/')) {
      const parts = token.split('/');
      if (parts.length !== 2) { console.warn('Invalid allowed token (bad CIDR):', token); continue; }
      const base = parts[0].trim();
      const prefix = Number(parts[1].trim());
      const baseInt = ipv4ToInt(base);
      if (baseInt === null || Number.isNaN(prefix) || prefix < 0 || prefix > 32) {
        console.warn('Invalid allowed CIDR token:', token);
        continue;
      }
      const mask = makeMask(prefix);
      const netBase = (baseInt & mask) >>> 0;
      out.push({ type: 'cidr', base: netBase >>> 0, mask: mask >>> 0, raw: token });
      continue;
    }
    const ipInt = ipv4ToInt(token);
    if (ipInt !== null) {
      out.push({ type: 'cidr', base: ipInt >>> 0, mask: 0xffffffff >>> 0, raw: token });
      continue;
    }
    out.push({ type: 'exact', ip: token, raw: token });
  }
  return out;
}

const ALLOWED_CONTROL_RULES = parseAllowedControlSubnets(ALLOWED_CONTROL_SUBNETS);
console.log('Allowed control subnets:', ALLOWED_CONTROL_RULES.map(r => r.raw).join(', '));

function isControlIpAllowed(rawIp) {
  const ip = normalizeRemoteIp(String(rawIp || ''));
  if (!ip) return false;
  if (ip === '127.0.0.1' || ip === '::1') return ALLOWED_CONTROL_RULES.length === 0 ? true :
    ALLOWED_CONTROL_RULES.some(r => (r.type === 'exact' && (r.ip === '127.0.0.1' || r.ip === '::1')) || (r.type === 'cidr' && r.base === 0xffffffff && r.mask === 0xffffffff));
  const ipv4 = ipv4ToInt(ip);
  if (ipv4 !== null) {
    const ipInt = ipv4 >>> 0;
    for (const r of ALLOWED_CONTROL_RULES) {
      if (r.type === 'cidr') {
        if (((ipInt & r.mask) >>> 0) === (r.base >>> 0)) return true;
      } else if (r.type === 'exact') {
        if (r.ip === ip) return true;
      }
    }
    return false;
  }
  for (const r of ALLOWED_CONTROL_RULES) {
    if (r.type === 'exact' && r.ip === ip) return true;
  }
  return false;
}

// WebSocket connection handler
wss.on('connection', (ws, req) => {
  const rawIp = normalizeRemoteIp(getClientIp(req));
  const allowed = isControlIpAllowed(rawIp);
  ws._clientIP = rawIp;
  ws._allowedControl = allowed;
  console.log('Browser connected. Clients:', wss.clients.size, 'IP:', rawIp, 'controlAllowed:', allowed);

  // initial info
  try {
    ws.send(JSON.stringify({
      type: 'info',
      controlConnected,
      videoConnected,
      allowedControl: allowed,
      ...(lastRemoteSize ? { remoteSize: lastRemoteSize } : {}),
      ...(AUTH_TOKEN ? {} : { startCapture: true })
    }));
  } catch (e) {}

  if (!AUTH_TOKEN && !controller && allowed) controller = ws;

  ws.on('close', () => {
    if (ws === controller) controller = null;
    console.log('Browser disconnected. Clients:', wss.clients.size, 'IP:', rawIp);
  });

  ws.on('message', (data, isBinary) => {
    if (isBinary) return;
    let msg;
    try { msg = JSON.parse(data.toString('utf8')); } catch { return; }

    if (msg.type === 'clientLog') {
      const ip = ws._clientIP || '';
      const p = msg.mousePage || {};
      const im = msg.mouseImg || {};
      console.log(`[ClientLog ${ip}] page=${p.x ?? 0},${p.y ?? 0} img=${im.x ?? 0},${im.y ?? 0}`);
      return;
    }

    if (!ws._allowedControl) return;

    if (msg.type === 'hello') {
      if (AUTH_TOKEN && msg.token !== AUTH_TOKEN) {
        try { ws.send(JSON.stringify({type:'info', error:'auth_failed'})); } catch {}
        ws.close(1008, 'Auth failed');
        return;
      }
      if (!controller) controller = ws;
      return;
    }

    if (!controller) controller = ws;
    if (ws !== controller) return;

    sendToControl(JSON.stringify(msg) + '\n');
  });
});

// --- TCP: Video source ---
let videoSock = null;
let acc = Buffer.alloc(0);

function scheduleVideoRetry() {
  if (videoRetry.timer) { clearTimeout(videoRetry.timer); videoRetry.timer = null; }
  const base = VIDEO_RETRY_BASE;
  const max = VIDEO_RETRY_MAX;
  const delay = Math.min(max, base * Math.pow(2, videoRetry.attempts)) + Math.floor(Math.random() * base);
  videoRetry.attempts++;
  videoRetry.timer = setTimeout(connectVideo, delay);
  console.log(`[Video] reconnect in ~${delay}ms (attempt ${videoRetry.attempts})`);
}
function resetVideoRetry() {
  videoRetry.attempts = 0;
  if (videoRetry.timer) { clearTimeout(videoRetry.timer); videoRetry.timer = null; }
}

function connectVideo() {
  console.log(`Connecting to MJPEG TCP ${TCP_HOST}:${TCP_PORT}...`);
  videoSock = net.connect(TCP_PORT, TCP_HOST, () => {
    console.log('Connected to MJPEG source.');
    acc = Buffer.alloc(0);
    resetVideoRetry();
    videoConnected = true;
    broadcastInfo({ videoConnected: true });

    // Try to bring control connection up immediately when video returns
    if (!controlConnected) {
      console.log('Video restored: attempting immediate control connect...');
      resetControlRetry();
      try { connectControl(); } catch (e) { console.warn('Immediate control connect failed:', e && e.message); }
    }

    // Ask allowed clients to start capture
    for (const client of wss.clients) {
      if (client && client.readyState === WebSocket.OPEN && client._allowedControl) {
        try { client.send(JSON.stringify({ type: 'info', startCapture: true })); } catch (e) {}
      }
    }
  });

  videoSock.on('data', (chunk) => {
    acc = Buffer.concat([acc, chunk]);
    let start = acc.indexOf(Buffer.from([0xff, 0xd8]));
    let end = start >= 0 ? acc.indexOf(Buffer.from([0xff, 0xd9]), start + 2) : -1;
    while (start >= 0 && end >= 0) {
      const jpeg = acc.slice(start, end + 2);
      broadcastJPEG(jpeg);
      acc = acc.slice(end + 2);
      start = acc.indexOf(Buffer.from([0xff, 0xd8]));
      end = start >= 0 ? acc.indexOf(Buffer.from([0xff, 0xd9]), sart + 2) : -1;
    }
    const MAX_ACC = 1024 * 1024;
    if (acc.length > MAX_ACC) acc = acc.slice(-65536);
  });

  videoSock.on('close', () => {
    console.log('Video TCP closed, retrying with backoff...');
    try { videoSock.destroy(); } catch(_) {}
    videoSock = null;
    videoConnected = false;
    broadcastInfo({ videoConnected: false });
    scheduleVideoRetry();
  });

  videoSock.on('error', (err) => {
    console.error('Video TCP error:', err.message);
    try { videoSock.destroy(); } catch(_) {}
    videoSock = null;
    videoConnected = false;
    broadcastInfo({ videoConnected: false });
    scheduleVideoRetry();
  });

  try { videoSock.setKeepAlive(true, VIDEO_RETRY_MAX); } catch (_) {}
}

// --- TCP: Control connection to destination PC ---
let controlSock = null;
let controlQueue = [];
let controlConnected = false;

function scheduleControlRetry() {
  if (controlRetry.timer) return;
  broadcastInfo({ controlConnected: false });
  const base = CONTROL_RETRY_BASE;
  const max = CONTROL_RETRY_MAX;
  const delay = Math.min(max, base * Math.pow(2, controlRetry.attempts)) + Math.floor(Math.random() * base);
  controlRetry.attempts++;
  controlRetry.timer = setTimeout(() => { controlRetry.timer = null; connectControl(); }, delay);
  console.log(`[#Control] reconnect in ~${delay}ms (attempt ${controlRetry.attempts})`);
}
function resetControlRetry() {
  controlRetry.attempts = 0;
  if (controlRetry.timer) { clearTimeout(controlRetry.timer); controlRetry.timer = null; }
}

function connectControl() {
  console.log(`Connecting to CONTROL TCP ${CONTROL_TCP_HOST}:${CONTROL_TCP_PORT}...`);
  controlSock = net.connect(CONTROL_TCP_PORT, CONTROL_TCP_HOST, () => {
    controlConnected = true;
    console.log('Connected to CONTROL agent.');
    broadcastInfo({ controlConnected: true, ...(lastRemoteSize ? { remoteSize: lastRemoteSize } : {}) });
    flushControlQueue();
    resetControlRetry();
    try { controlSock.write(JSON.stringify({type:'query', what:'remoteSize'}) + '\n'); } catch (e) {}
  });

  controlSock.on('data', (chunk) => {
    const lines = chunk.toString('utf8').split(/\r?\n/).filter(Boolean);
    for (const line of lines) {
      try {
        const obj = JSON.parse(line);
        if (obj.remoteSize) { lastRemoteSize = obj.remoteSize; broadcastInfo({ remoteSize: lastRemoteSize }); }
        else if (obj.type === 'info' || obj.type === 'hello') {
          if (obj.remoteSize) lastRemoteSize = obj.remoteSize;
          broadcastInfo(obj);
        }
      } catch (e) {}
    }
  });

  controlSock.on('close', () => {
    controlConnected = false;
    console.log('CONTROL TCP closed, retrying with backoff...');
    try { controlSock.destroy(); } catch(_) {}
    controlSock = null;
    scheduleControlRetry();
  });

  controlSock.on('error', (err) => {
    controlConnected = false;
    console.error('CONTROL TCP error:', err.message);
    try { controlSock.destroy(); } catch(_) {}
    controlSock = null;
    scheduleControlRetry();
  });

  try { controlSock.setKeepAlive(true, CONTROL_RETRY_MAX); } catch (_) {}
}

// queue helpers
function flushControlQueue() {
  if (!controlSock || !controlConnected) return;
  while (controlQueue.length) {
    const line = controlQueue.shift();
    try { controlSock.write(line); } catch (e) { controlQueue.unshift(line); break; }
  }
}
function sendToControl(line) {
  if (controlConnected && controlSock) {
    try { controlSock.write(line); } catch (e) { controlQueue.push(line); }
  } else controlQueue.push(line);
}

// Start servers
httpServer.listen(HTTP_PORT, () => {
  console.log(`HTTP server listening on http://localhost:${HTTP_PORT}/`);
  connectVideo();
  connectControl();
});
