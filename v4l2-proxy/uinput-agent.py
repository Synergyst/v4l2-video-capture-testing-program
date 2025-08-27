#!/usr/bin/env python3
"""
uinput-agent.py

Create two uinput devices (keyboard and mouse) that look like USB HID devices.
Read JSON lines from stdin and inject keyboard/mouse events accordingly.

Prereqs:
  sudo apt-get install python3-evdev    # or pip3 install evdev
  Must have access to /dev/uinput (run as root or use udev rule below)

Example JSON messages (one per line):
  {"type":"keyboard","action":"down","code":"KeyA"}
  {"type":"keyboard","action":"up","code":"KeyA"}
  {"type":"mouse","action":"moveRelative","dx":5,"dy":-3}
  {"type":"mouse","action":"down","button":0}
  {"type":"mouse","action":"up","button":0}
  {"type":"mouse","action":"wheel","deltaY":120}
"""
import sys
import json
from evdev import UInput, ecodes, AbsInfo

# Use a large ABS range if you ever send absolute moves.
ABS_MAX = 32767

# Map Browser KeyboardEvent.code -> evdev KEY_* name
# Expand this map as you need more keys.
CODE_TO_KEYNAME = {}

# Letters A..Z
for c in range(ord('A'), ord('Z') + 1):
    CODE_TO_KEYNAME[f'Key{chr(c)}'] = f'KEY_{chr(c)}'

# Digits 0..9
for d in range(10):
    CODE_TO_KEYNAME[f'Digit{d}'] = f'KEY_{d}'

# Common named keys
CUSTOM = {
    'Enter': 'KEY_ENTER',
    'NumpadEnter': 'KEY_ENTER',
    'Space': 'KEY_SPACE',
    'Tab': 'KEY_TAB',
    'Backspace': 'KEY_BACKSPACE',
    'Escape': 'KEY_ESC',
    'ArrowLeft': 'KEY_LEFT',
    'ArrowRight': 'KEY_RIGHT',
    'ArrowUp': 'KEY_UP',
    'ArrowDown': 'KEY_DOWN',
    'PageUp': 'KEY_PAGEUP',
    'PageDown': 'KEY_PAGEDOWN',
    'End': 'KEY_END',
    'Home': 'KEY_HOME',
    'Insert': 'KEY_INSERT',
    'Delete': 'KEY_DELETE',
    'CapsLock': 'KEY_CAPSLOCK',
    'Pause': 'KEY_PAUSE',
    'PrintScreen': 'KEY_SYSRQ',
    'ScrollLock': 'KEY_SCROLLLOCK',
    'Minus': 'KEY_MINUS',
    'Equal': 'KEY_EQUAL',
    'BracketLeft': 'KEY_LEFTBRACE',
    'BracketRight': 'KEY_RIGHTBRACE',
    'Backslash': 'KEY_BACKSLASH',
    'Semicolon': 'KEY_SEMICOLON',
    'Quote': 'KEY_APOSTROPHE',
    'Comma': 'KEY_COMMA',
    'Period': 'KEY_DOT',
    'Slash': 'KEY_SLASH',
}
CODE_TO_KEYNAME.update(CUSTOM)

# Modifiers
MODS = {
    'ShiftLeft': 'KEY_LEFTSHIFT',
    'ShiftRight': 'KEY_RIGHTSHIFT',
    'ControlLeft': 'KEY_LEFTCTRL',
    'ControlRight': 'KEY_RIGHTCTRL',
    'AltLeft': 'KEY_LEFTALT',
    'AltRight': 'KEY_RIGHTALT',
    'MetaLeft': 'KEY_LEFTMETA',
    'MetaRight': 'KEY_RIGHTMETA',
}
CODE_TO_KEYNAME.update(MODS)

def code_to_keycode(code):
    """Return evdev numeric keycode for a given KeyboardEvent.code or None."""
    if not code:
        return None
    keyname = CODE_TO_KEYNAME.get(code)
    if keyname:
        return ecodes.ecodes.get(keyname)
    # fallback: single char
    if len(code) == 1:
        cand = f'KEY_{code.upper()}'
        return ecodes.ecodes.get(cand)
    # last resort try 'KEY_' + upper
    return ecodes.ecodes.get('KEY_' + code.upper())

# Build capability sets for keyboard and mouse
kbd_keys = set()
for kn in set(CODE_TO_KEYNAME.values()):
    code = ecodes.ecodes.get(kn)
    if code:
        kbd_keys.add(code)

# Ensure common keys present
for kn in ['KEY_SPACE','KEY_ENTER','KEY_BACKSPACE','KEY_TAB','KEY_ESC']:
    c = ecodes.ecodes.get(kn)
    if c:
        kbd_keys.add(c)

mouse_keys = [ecodes.BTN_LEFT, ecodes.BTN_RIGHT, ecodes.BTN_MIDDLE]

# Keyboard device capabilities: EV_KEY only (keyboard keys)
kbd_cap = {
    ecodes.EV_KEY: list(kbd_keys),
}

# Mouse device capabilities: EV_KEY (buttons) + EV_REL (movement/wheel)
mouse_cap = {
    ecodes.EV_KEY: mouse_keys,
    ecodes.EV_REL: [ecodes.REL_X, ecodes.REL_Y, ecodes.REL_WHEEL, ecodes.REL_HWHEEL],
}

# Create two devices that look like USB devices (bustype BUS_USB).
# Pick vendor/product values (arbitrary for virtual device).
try:
    kb = UInput(events=kbd_cap, name="Remote-Keyboard", vendor=0x1234, product=0x5678, version=1, bustype=ecodes.BUS_USB)
    ms = UInput(events=mouse_cap, name="Remote-Mouse", vendor=0x1234, product=0x8765, version=1, bustype=ecodes.BUS_USB)
except Exception as e:
    print("Failed to create uinput devices:", e, file=sys.stderr)
    sys.exit(2)

print("Created devices:")
print("  Keyboard:", kb.devnode if hasattr(kb, 'devnode') else "unknown")
print("  Mouse   :", ms.devnode if hasattr(ms, 'devnode') else "unknown", file=sys.stderr)

# Track relative/absolute values if needed (not strictly required)
last_abs_x = 0
last_abs_y = 0

def handle_keyboard(msg):
    code = msg.get('code') or msg.get('key')
    if not code:
        return
    keycode = code_to_keycode(code)
    if keycode is None:
        print("Unknown key code:", code, file=sys.stderr)
        return
    action = msg.get('action')
    if action == 'down':
        kb.write(ecodes.EV_KEY, keycode, 1)
        kb.syn()
    elif action == 'up':
        kb.write(ecodes.EV_KEY, keycode, 0)
        kb.syn()

def handle_mouse(msg):
    action = msg.get('action')
    if action == 'moveRelative':
        dx = int(msg.get('dx', 0))
        dy = int(msg.get('dy', 0))
        if dx:
            ms.write(ecodes.EV_REL, ecodes.REL_X, dx)
        if dy:
            ms.write(ecodes.EV_REL, ecodes.REL_Y, dy)
        ms.syn()
    elif action == 'move':
        global last_abs_x, last_abs_y
        # We do relative moves for console compatibility. If you want ABS,
        # create ABS capabilities in mouse_cap and write EV_ABS events.
        x = float(msg.get('x', 0.5))
        y = float(msg.get('y', 0.5))
        # Convert normalized to ABS and emit REL delta as well for good measure
        abs_x = int(max(0.0, min(1.0, x)) * ABS_MAX)
        abs_y = int(max(0.0, min(1.0, y)) * ABS_MAX)
        dx = abs_x - last_abs_x
        dy = abs_y - last_abs_y
        if dx:
            ms.write(ecodes.EV_REL, ecodes.REL_X, dx)
        if dy:
            ms.write(ecodes.EV_REL, ecodes.REL_Y, dy)
        # Update last (module-level)
        last_abs_x = abs_x
        last_abs_y = abs_y
        ms.syn()
    elif action in ('down', 'up'):
        btn = int(msg.get('button', 0))
        btn_map = {0: ecodes.BTN_LEFT, 1: ecodes.BTN_MIDDLE, 2: ecodes.BTN_RIGHT}
        btn_code = btn_map.get(btn, ecodes.BTN_LEFT)
        val = 1 if action == 'down' else 0
        ms.write(ecodes.EV_KEY, btn_code, val)
        ms.syn()
    elif action == 'wheel':
        dy = msg.get('deltaY', 0)
        dx = msg.get('deltaX', 0)
        # Convert to wheel steps; adjust if sensitivity is off
        def steps(v):
            try:
                v = float(v)
            except:
                return 0
            if abs(v) < 1:
                return int(v)
            return int(v / 120) or (1 if v > 0 else -1)
        sy = steps(dy)
        sx = steps(dx)
        if sy:
            ms.write(ecodes.EV_REL, ecodes.REL_WHEEL, sy)
        if sx:
            ms.write(ecodes.EV_REL, ecodes.REL_HWHEEL, sx)
        ms.syn()

def main():
    buf = ""
    while True:
        chunk = sys.stdin.buffer.read(4096)
        if not chunk:
            break  # EOF -> exit
        try:
            s = chunk.decode('utf8')
        except Exception:
            continue
        buf += s
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except Exception as e:
                print("Bad JSON:", line, file=sys.stderr)
                continue
            t = msg.get('type')
            if t == 'keyboard':
                handle_keyboard(msg)
            elif t == 'mouse':
                handle_mouse(msg)
            else:
                # ignore other messages
                pass

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            kb.close()
            ms.close()
        except Exception:
            pass
