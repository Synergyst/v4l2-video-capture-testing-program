// control-sink.js
// Usage:
//   npm install serialport @serialport/parser-readline robotjs
//   CONTROL_TCP_PORT=1444 SERIAL_PORT=/dev/ttyACM0 node control-sink.js

const { exec } = require('child_process');
const ENABLE_SYSTEM_CMDS = process.env.ENABLE_SYSTEM_CMDS === '1'; // default off
const net = require('net');
const sp = require('serialport');
const fs = require('fs');

let fbInfo = { w: 1920, h: 1080, bpp: 32 };
let lastMousePos = { x: Math.floor(fbInfo.w / 2), y: Math.floor(fbInfo.h / 2) };

function readFbInfo() {
  try {
    const vs = fs.readFileSync('/sys/class/graphics/fb0/virtual_size', 'utf8').trim();
    const bppStr = fs.readFileSync('/sys/class/graphics/fb0/bits_per_pixel', 'utf8').trim();
    const parts = vs.split(',').map(s => s.trim());
    const w = parseInt(parts[0], 10) || fbInfo.w;
    const h = parseInt(parts[1], 10) || fbInfo.h;
    const bpp = parseInt(bppStr, 10) || fbInfo.bpp;
    fbInfo = { w, h, bpp };
    lastMousePos = { x: Math.floor(fbInfo.w / 2), y: Math.floor(fbInfo.h / 2) };
  } catch (e) {
    // keep defaults if reading fails (e.g., not present)
  }
}
readFbInfo();

let ReadlineParserCtor;
try {
  // v10+: { ReadlineParser }
  ({ ReadlineParser: ReadlineParserCtor } = require('@serialport/parser-readline'));
} catch {
  // fallback (older @serialport/parser-readline exported the class directly)
  ReadlineParserCtor = require('@serialport/parser-readline');
}

// robotjs is used to convert normalized absolute mouse to relative deltas
const robot = require('robotjs');

const CONTROL_PORT = parseInt(process.env.CONTROL_TCP_PORT || '1444', 10);
const SERIAL_PORT_PATH = process.env.SERIAL_PORT || (process.platform === 'win32' ? 'COM8' : '/dev/ttyACM0');
const SERIAL_BAUD = parseInt(process.env.SERIAL_BAUD || '115200', 10);

// Build a version-agnostic "createPort" for serialport
let SerialPortCtor = null;
if (sp && typeof sp.SerialPort === 'function') {
  // serialport v10+
  SerialPortCtor = sp.SerialPort;
} else if (typeof sp === 'function') {
  // serialport v8/9
  SerialPortCtor = sp;
} else {
  throw new Error('Unsupported "serialport" module shape. Please `npm i serialport@latest`.');
}
function createPort(path, baud, autoOpen) {
  if (SerialPortCtor === sp) {
    // old API: new SerialPort(path, { baudRate, autoOpen })
    return new SerialPortCtor(path, { baudRate: baud, autoOpen });
  }
  // new API: new SerialPort({ path, baudRate, autoOpen })
  return new SerialPortCtor({ path, baudRate: baud, autoOpen });
}

let serial = null;
let serialParser = null;
let serialReady = false;
let pendingLines = [];

function openSerial() {
  console.log('Opening serial', SERIAL_PORT_PATH, '@', SERIAL_BAUD);
  serial = createPort(SERIAL_PORT_PATH, SERIAL_BAUD, false);
  // parser
  serialParser = serial.pipe(new ReadlineParserCtor({ delimiter: '\n', encoding: 'utf8' }));

  serial.on('open', () => {
    console.log('Serial open');
    serialReady = true;
    while (pendingLines.length) {
      const l = pendingLines.shift();
      try { serial.write(l + '\n'); } catch (e) { pendingLines.unshift(l); break; }
    }
  });

  serial.on('error', (err) => {
    console.error('Serial error:', err.message);
    serialReady = false;
  });

  serial.on('close', () => {
    console.log('Serial closed, retrying in 1s...');
    serialReady = false;
    setTimeout(openSerial, 1000);
  });

  serialParser.on('data', (line) => {
    console.log('[Teensy]', String(line || '').trim());
  });

  // open
  if (typeof serial.open === 'function') {
    serial.open((err) => {
      if (err) {
        console.error('Failed to open serial:', err.message);
        setTimeout(openSerial, 1000);
      }
    });
  } else {
    // v10 autoOpen may open immediately when autoOpen !== false; but we set autoOpen:false.
    // If open() is missing for some reason, try write to force error:
    try { serial.write(''); } catch {}
  }
}

function sendToTeensy(line) {
  if (!serialReady) {
    pendingLines.push(line);
    return;
  }
  try {
    process.stdout.write(line);
    serial.write(line);
  } catch (e) {
    console.error('Serial write failed, queuing:', e.message);
    pendingLines.push(line);
  }
}

// Convert normalized absolute mouse move (0..1) to relative deltas using robotjs
function handleAbsoluteMove(msg) {
  try {
    //const screen = robot.getScreenSize();
    const targetX = Math.round(Math.max(0, Math.min(1, msg.x)) * (fbInfo.w - 1));
    const targetY = Math.round(Math.max(0, Math.min(1, msg.y)) * (fbInfo.h - 1));
    const cur = robot.getMousePos();
    const dx = targetX - cur.x;
    const dy = targetY - cur.y;
    if (dx === 0 && dy === 0) return;
    sendToTeensy(`MREL ${dx} ${dy}\n`);
  } catch (e) {
    console.error('Absolute move conversion failed:', e.message);
  }
}
/*function handleAbsoluteMove(msg) {
  try {
    const nx = Math.max(0, Math.min(1, Number(msg.x) || 0));
    const ny = Math.max(0, Math.min(1, Number(msg.y) || 0));
    const targetX = Math.round(nx * (fbInfo.w - 1));
    const targetY = Math.round(ny * (fbInfo.h - 1));
    const dx = targetX - lastMousePos.x;
    const dy = targetY - lastMousePos.y;
    if (dx === 0 && dy === 0) return;
    // update tracked position (clamp to framebuffer)
    lastMousePos.x = Math.max(0, Math.min(fbInfo.w - 1, lastMousePos.x + dx));
    lastMousePos.y = Math.max(0, Math.min(fbInfo.h - 1, lastMousePos.y + dy));
    process.stdout.write('targetX:' + targetX + ', targetY:' + targetY + ', lastMousePos.x:' + lastMousePos.x + ', lastMousePos.y:' + lastMousePos.y + ', msg.x:' + msg.x + ', msg.y:' + msg.y + '\n');
    sendToTeensy(`MREL ${dx} ${dy}\n`);
  } catch (e) {
    console.error('Absolute move conversion failed:', e && e.message);
  }
}*/
/*function handleAbsoluteMove(msg) {
  try {
    const nx = Math.max(0, Math.min(1, Number(msg.x) || 0));
    const ny = Math.max(0, Math.min(1, Number(msg.y) || 0));
    const tx = Math.round(nx * (fbInfo.w - 1));
    const ty = Math.round(ny * (fbInfo.h - 1));
    // Send MABS to Teensy (Teensy will compute delta internally).
    sendToTeensy(`MABS ${tx} ${ty}\n`);
    // Update sink-tracked cursor position (used for future deltas and SETPOS).
    lastMousePos.x = tx;
    lastMousePos.y = ty;
  } catch (e) {
    console.error('Absolute move conversion failed:', e && e.message);
  }
}*/

function quoteForTeensy(s) {
  // Wrap in quotes; replace " -> ' for simplicity
  return `"${String(s).replace(/"/g, "'")}"`;
}

// TCP server: accept controller connections (the proxy)
const server = net.createServer((sock) => {
  console.log('Controller connected:', sock.remoteAddress + ':' + sock.remotePort);

  // Send info: local screen size
  try {
    //const screen = robot.getScreenSize();
    //sock.write(JSON.stringify({ type: 'info', remoteSize: { w: screen.width, h: screen.height } }) + '\n');
    //sock.write(JSON.stringify({ type: 'info', remoteSize: { w: 1920, h: 1080 } }) + '\n');
    sendToTeensy(`MSCRSZ ${fbInfo.w} ${fbInfo.h}\n`);
    sock.write(JSON.stringify({ type: 'info', remoteSize: { w: fbInfo.w, h: fbInfo.h } }) + '\n');
    // Tell Teensy our tracked starting cursor position so MABS/MREL behave predictably.
    if (Number.isFinite(lastMousePos.x) && Number.isFinite(lastMousePos.y)) {
      sendToTeensy(`SETPOS ${lastMousePos.x} ${lastMousePos.y}\n`);
    }
  } catch (e) {
    console.warn('Could not get screen size:', e.message);
  }


  let buf = '';
  sock.on('data', (chunk) => {
    buf += chunk.toString('utf8');
    let idx;
    while ((idx = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, idx).trim();
      buf = buf.slice(idx + 1);
      if (!line) continue;
      let msg;
      try { msg = JSON.parse(line); } catch { console.warn('Bad JSON:', line); continue; }

      if (msg.type === 'mouse') {
        const action = msg.action;
        if (action === 'move') {
          handleAbsoluteMove(msg);
        } else if (action === 'moveRelative') {
          /*const dx = Math.trunc(Number.isFinite(msg.dx) ? msg.dx : (msg.x || 0));
          const dy = Math.trunc(Number.isFinite(msg.dy) ? msg.dy : (msg.y || 0));
          sendToTeensy(`MREL ${dx} ${dy}\n`);
          // Update tracked cursor position
          lastMousePos.x = Math.max(0, Math.min(fbInfo.w - 1, lastMousePos.x + dx));
          lastMousePos.y = Math.max(0, Math.min(fbInfo.h - 1, lastMousePos.y + dy));*/
          /*const dx = Math.trunc(Number.isFinite(msg.dx) ? msg.dx : (msg.x || 0));
          const dy = Math.trunc(Number.isFinite(msg.dy) ? msg.dy : (msg.y || 0));
          sendToTeensy(`MREL ${dx} ${dy}\n`);*/
          const dx = Math.trunc(Number.isFinite(msg.dx) ? msg.dx : (msg.x || 0));
          const dy = Math.trunc(Number.isFinite(msg.dy) ? msg.dy : (msg.y || 0));
          sendToTeensy(`MREL ${dx} ${dy}\n`);
          // Update tracked cursor position in framebuffer coords
          lastMousePos.x = Math.max(0, Math.min(fbInfo.w - 1, lastMousePos.x + dx));
          lastMousePos.y = Math.max(0, Math.min(fbInfo.h - 1, lastMousePos.y + dy));
        } else if (action === 'down') {
          const btn = (typeof msg.button === 'number') ? msg.button : 0;
          sendToTeensy(`MDOWN ${btn}\n`);
        } else if (action === 'up') {
          const btn = (typeof msg.button === 'number') ? msg.button : 0;
          sendToTeensy(`MUP ${btn}\n`);
        } else if (action === 'wheel') {
          const dx = Math.trunc(msg.deltaX || 0);
          const dy = Math.trunc(msg.deltaY || 0);
          sendToTeensy(`MWHEEL ${dx} ${dy}\n`);
        }
      } else if (msg.type === 'keyboard') {
        const action = msg.action;
        const code = msg.code || '';
        const key = msg.key || '';
        const ctrl = msg.ctrl ? 1 : 0;
        const shift = msg.shift ? 1 : 0;
        const alt = msg.alt ? 1 : 0;
        const meta = msg.meta ? 1 : 0;
        if (action === 'down') {
          sendToTeensy(`KDOWN ${escapeArg(code)} ${escapeArg(key)} ${ctrl} ${shift} ${alt} ${meta}\n`);
        } else if (action === 'up') {
          sendToTeensy(`KUP ${escapeArg(code)} ${escapeArg(key)} ${ctrl} ${shift} ${alt} ${meta}\n`);
        }
      }  else if (msg.type === 'system' && (msg.action === 'reboot' || msg.action === 'poweroff')) {
        if (!ENABLE_SYSTEM_CMDS) {
          sock.write(JSON.stringify({type:'info', error:'system commands disabled'}) + '\n');
        } else {
          const act = msg.action;
          let cmd;
          if (process.platform === 'win32') {
            cmd = act === 'reboot' ? 'shutdown /r /t 0' : 'shutdown /s /t 0';
          } else if (process.platform === 'darwin') {
            cmd = act === 'reboot' ? 'sudo shutdown -r now' : 'sudo shutdown -h now';
          } else {
            cmd = act === 'reboot' ? 'reboot || sudo reboot' : 'systemctl reboot || sudo systemctl reboot';
          }
          exec(cmd, (err, stdout, stderr) => {
            sock.write(JSON.stringify({
              type:'shellResult',
              id: act,
              code: err ? (err.code ?? 1) : 0,
              stdout: stdout?.toString() || '',
              stderr: stderr?.toString() || (err?.message || '')
            }) + '\n');
          });
        }
      } else if (msg.type === 'text' && typeof msg.text === 'string') {
        sendToTeensy(`KTEXT ${escapeArg(msg.text)}\n`);
      } else if (msg.type === 'relallkeys') {
        sendToTeensy(`RELALLKEYS\n`);
      } else if (msg.type === 'shell' && typeof msg.cmd === 'string') {
        const id = msg.id || Date.now().toString(36);
        exec(msg.cmd, { timeout: 30000 }, (err, stdout, stderr) => {
          sock.write(JSON.stringify({
            type:'shellResult',
            id,
            code: err ? (err.code ?? 1) : 0,
            stdout: stdout?.toString() || '',
            stderr: stderr?.toString() || (err?.message || '')
          }) + '\n');
        });
      } else if (msg.type === 'query' && msg.what === 'remoteSize') {
        process.stdout.write('Sent remoteSize\n');
        //const screen = robot.getScreenSize();
        //sock.write(JSON.stringify({ type: 'info', remoteSize: { w: screen.width, h: screen.height } }) + '\n');
        //sock.write(JSON.stringify({ type: 'info', remoteSize: { w: 1920, h: 1080 } }) + '\n');
        sendToTeensy(`MSCRSZ ${fbInfo.w} ${fbInfo.h}\n`);
        sock.write(JSON.stringify({ type: 'info', remoteSize: { w: fbInfo.w, h: fbInfo.h } }) + '\n');
        // Tell Teensy our tracked starting cursor position so MABS/MREL behave predictably.
        if (Number.isFinite(lastMousePos.x) && Number.isFinite(lastMousePos.y)) {
          sendToTeensy(`SETPOS ${lastMousePos.x} ${lastMousePos.y}\n`);
        }
      } else {
        sendToTeensy('RAW ' + escapeArg(JSON.stringify(msg)) + '\n');
      }
    }
  });

  sock.on('close', () => console.log('Controller disconnected'));
  sock.on('error', (err) => console.log('Controller socket error:', err.message));
});

function escapeArg(s) {
  if (typeof s !== 'string') s = String(s);
  if (s.includes(' ') || s === '') return `"${s.replace(/"/g, "'")}"`;
  return s;
}

server.listen(CONTROL_PORT, () => {
  console.log('Control sink listening on', CONTROL_PORT);
  openSerial();
});
