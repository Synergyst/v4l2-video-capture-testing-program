// control-sink.js
// Usage:
//   npm install serialport @serialport/parser-readline robotjs
//   CONTROL_TCP_PORT=1444 SERIAL_PORT=/dev/ttyACM0 node control-sink.js

const net = require('net');
const sp = require('serialport');
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
const SERIAL_PORT_PATH = process.env.SERIAL_PORT || (process.platform === 'win32' ? 'COM3' : '/dev/ttyACM0');
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
    process.stdout.write(line + '\n');
    serial.write(line + '\n');
  } catch (e) {
    console.error('Serial write failed, queuing:', e.message);
    pendingLines.push(line);
  }
}

// Convert normalized absolute mouse move (0..1) to relative deltas using robotjs
function handleAbsoluteMove(msg) {
  try {
    const screen = robot.getScreenSize();
    const targetX = Math.round(Math.max(0, Math.min(1, msg.x)) * (screen.width - 1));
    const targetY = Math.round(Math.max(0, Math.min(1, msg.y)) * (screen.height - 1));
    const cur = robot.getMousePos();
    const dx = targetX - cur.x;
    const dy = targetY - cur.y;
    if (dx === 0 && dy === 0) return;
    sendToTeensy(`MREL ${dx} ${dy}`);
  } catch (e) {
    console.error('Absolute move conversion failed:', e.message);
  }
}

// TCP server: accept controller connections (the proxy)
const server = net.createServer((sock) => {
  console.log('Controller connected:', sock.remoteAddress + ':' + sock.remotePort);

  // Send info: local screen size
  try {
    const screen = robot.getScreenSize();
    sock.write(JSON.stringify({ type: 'info', remoteSize: { w: screen.width, h: screen.height } }) + '\n');
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
          const dx = Math.trunc(Number.isFinite(msg.dx) ? msg.dx : (msg.x || 0));
          const dy = Math.trunc(Number.isFinite(msg.dy) ? msg.dy : (msg.y || 0));
          sendToTeensy(`MREL ${dx} ${dy}`);
        } else if (action === 'down') {
          const btn = (typeof msg.button === 'number') ? msg.button : 0;
          sendToTeensy(`MDOWN ${btn}`);
        } else if (action === 'up') {
          const btn = (typeof msg.button === 'number') ? msg.button : 0;
          sendToTeensy(`MUP ${btn}`);
        } else if (action === 'wheel') {
          const dx = Math.trunc(msg.deltaX || 0);
          const dy = Math.trunc(msg.deltaY || 0);
          sendToTeensy(`MWHEEL ${dx} ${dy}`);
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
          sendToTeensy(`KDOWN ${escapeArg(code)} ${escapeArg(key)} ${ctrl} ${shift} ${alt} ${meta}`);
        } else if (action === 'up') {
          sendToTeensy(`KUP ${escapeArg(code)} ${escapeArg(key)} ${ctrl} ${shift} ${alt} ${meta}`);
        }
      } else {
        sendToTeensy('RAW ' + escapeArg(JSON.stringify(msg)));
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
