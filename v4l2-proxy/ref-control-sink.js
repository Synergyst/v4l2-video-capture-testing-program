// control-sink.js
const net = require('net');
const PORT = parseInt(process.env.CONTROL_TCP_PORT || '1444', 10);
const server = net.createServer((sock) => {
  console.log('Controller connected:', sock.remoteAddress);
  sock.write(JSON.stringify({type:'info', remoteSize:{w:1920,h:1080}}) + '\n');
  let buf = '';
  sock.on('data', (chunk) => {
    buf += chunk.toString('utf8');
    let idx;
    while ((idx = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, idx); buf = buf.slice(idx+1);
      if (!line.trim()) continue;
      try {
        const msg = JSON.parse(line);
        console.log('INPUT:', msg);
        // Here you'd inject with OS APIs (e.g., pynput, win32 SendInput, etc.)
      } catch (e) {
        console.log('Bad JSON:', line);
      }
    }
  });
  sock.on('close', () => console.log('Controller disconnected'));
});
server.listen(PORT, () => console.log('Control sink listening on', PORT));
