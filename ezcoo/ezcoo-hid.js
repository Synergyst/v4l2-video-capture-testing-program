#!/usr/bin/env node
'use strict';

const HID = require('node-hid');
const readline = require('readline');

const VENDOR_ID = 0x28e9;
const PRODUCT_ID = 0x028a;
const REPORT_SIZE = 64;

// Automatically append CRLF if missing
function toReport(cmd) {
  let payload = cmd;
  if (!payload.endsWith('\r\n')) payload += '\r\n';
  // HID reports: [reportId, ...payloadBytes]
  const arr = [0, ...Buffer.from(payload, 'ascii')];
  while (arr.length < REPORT_SIZE) arr.push(0);
  return arr;
}

function findDevice() {
  const devices = HID.devices();
  const devInfo = devices.find(d => d.vendorId === VENDOR_ID && d.productId === PRODUCT_ID);
  if (!devInfo) throw new Error('GD32 custom HID not found. Ensure it’s plugged in and permissions allowed.');
  return devInfo;
}

function openDevice() {
  const info = findDevice();
  return new HID.HID(info.path);
}

// Escape for safe logging: non-printables shown as dots
function asAsciiSafe(data) {
  // data is a Buffer
  const str = data.toString('ascii');
  //return str.replace(/[^\x20-\x7E\s]/g, '.');
  return str.replace(/[^\x20-\x7E\s]/g, '');
}

// Simple sanitation for logging user input
function logDisplay(s) {
  return s.replace(/\r/g, '\\r').replace(/\n/g, '\\n');
}

async function main() {
  // Non-option args become the command; default is "EZH"
  const raw = process.argv.slice(2);
  const interactive = raw.includes('-i') || raw.includes('--interactive');
  const useFeature = raw.includes('-f') || raw.includes('--feature');

  // Build the command to send (without trailing CRLF; toReport will append)
  const nonOpts = raw.filter(a => !a.startsWith('-'));
  const cmdArg = nonOpts.length ? nonOpts.join(' ') : 'EZH';

  const device = openDevice();

  device.on('error', (err) => {
    console.error('Device error:', err);
  });

  if (interactive) {
    console.log('Connected to GD32 HID. Type commands to send. Ctrl+C to exit.');
    device.on('data', (data) => {
      // Print incoming data as ASCII (sanitized)
      const safe = asAsciiSafe(data);
      process.stdout.write(`${safe}`);
    });

    const rl = readline.createInterface({ input: process.stdin, output: process.stdout });
    rl.setPrompt('');
    rl.prompt();

    rl.on('line', (line) => {
      // User input line; append CRLF via toReport()
      const payload = line; // we send line; toReport adds \r\n
      const report = toReport(payload);
      if (useFeature) device.sendFeatureReport(report);
      else device.write(report);
      console.log(`SENT: ${logDisplay(line) + '\\r\\n'}`);
      rl.prompt();
    });

    process.on('SIGINT', () => {
      rl.close();
      device.close();
      process.exit(0);
    });

  } else {
    // Non-interactive: send the provided command (default "EZH")
    const report = toReport(cmdArg);
    if (useFeature) device.sendFeatureReport(report);
    else device.write(report);
    console.log(`SENT: ${logDisplay(cmdArg)}`);
    device.close();
  }
}

main().catch((err) => {
  console.error('Error:', err.message);
  process.exit(1);
});
