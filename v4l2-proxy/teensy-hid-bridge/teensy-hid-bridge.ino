#include <Arduino.h>
#include <Mouse.h>
#include <Keyboard.h>
#include <limits.h>

// ---------------- Config ----------------
int32_t lastMouseX = INT32_MIN;
int32_t lastMouseY = INT32_MIN;
#define DEFAULT_XRES 1920
#define DEFAULT_YRES 1080

// ---------------- Serial helpers ----------------
void setup() {
  Serial.begin(2000000);
  // Give USB a moment to come up
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* wait briefly */
  }

  Keyboard.begin();
  Mouse.begin();
  delay(2000);
}

String readLine() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = line;
      line = "";
      return out;
    }
    line += c;
  }
  return String();
}

bool isWhitespace(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

// ---------------- Mouse helpers (unchanged) ----------------
void chunkedMouseMove(int dx, int dy) {
  const int MAX_STEP = 127;
  while (dx != 0 || dy != 0) {
    int sx = (dx > 0) ? min(dx, MAX_STEP) : max(dx, -MAX_STEP);
    int sy = (dy > 0) ? min(dy, MAX_STEP) : max(dy, -MAX_STEP);
    Mouse.move(sx, sy, 0);
    dx -= sx;
    dy -= sy;
    //delay(3);
  }
}

void chunkedWheel(int dx, int dy) {
  const int MAX_STEP = 127;
  // Vertical wheel
  while (dy != 0) {
    int step = (dy > 0) ? min(dy, MAX_STEP) : max(dy, -MAX_STEP);
    Mouse.move(0, 0, step);
    dy -= step;
    //delay(3);
  }
  // Horizontal wheel ignored by default; add mapping if you need it.
}

// ---------------- Keyboard micro manager state ----------------
//
// We maintain our own modifier bitmask (8 bits matching USB order) and a list of up to 6 concurrent keys.
// For Arduino (RP2040), we do not have raw HID frame setters, so we diff against previous state and call
// Keyboard.press()/Keyboard.release() as needed.

// Our modifier bit layout matches USB HID usage (so it’s easy to reason about):
// bit0 LCTRL, bit1 LSHIFT, bit2 LALT, bit3 LGUI, bit4 RCTRL, bit5 RSHIFT, bit6 RALT, bit7 RGUI
static uint8_t modMask = 0;
static uint8_t keys[6] = { 0, 0, 0, 0, 0, 0 };

// Track what we actually have down on the host so we can diff and send precise press/release
static uint8_t prevModMask = 0;
static uint8_t prevKeys[6] = { 0, 0, 0, 0, 0, 0 };

String unquote(String s) {
  if (s.startsWith("\"") && s.endsWith("\"") && s.length() >= 2) {
    return s.substring(1, s.length() - 1);
  }
  return s;
}

static inline bool arrContains(const uint8_t arr[6], uint8_t k) {
  for (int i = 0; i < 6; i++)
    if (arr[i] == k) return true;
  return false;
}

bool addKey(uint8_t k) {
  if (k == 0) return false;
  // already pressed?
  for (int i = 0; i < 6; i++)
    if (keys[i] == k) return false;
  // find empty slot
  for (int i = 0; i < 6; i++) {
    if (keys[i] == 0) {
      keys[i] = k;
      return true;
    }
  }
  // no room: drop (or replace oldest if preferred)
  return false;
}

bool removeKey(uint8_t k) {
  if (k == 0) return false;
  bool removed = false;
  for (int i = 0; i < 6; i++) {
    if (keys[i] == k) {
      keys[i] = 0;
      removed = true;
    }
  }
  // compact
  if (removed) {
    uint8_t tmp[6] = { 0, 0, 0, 0, 0, 0 };
    int j = 0;
    for (int i = 0; i < 6; i++)
      if (keys[i]) tmp[j++] = keys[i];
    for (int i = 0; i < 6; i++) keys[i] = tmp[i];
  }
  return removed;
}

// ---------------- Mapping helpers ----------------

// Modifier bits (internal)
#define MODBIT_LCTRL 0x01
#define MODBIT_LSHIFT 0x02
#define MODBIT_LALT 0x04
#define MODBIT_LGUI 0x08
#define MODBIT_RCTRL 0x10
#define MODBIT_RSHIFT 0x20
#define MODBIT_RALT 0x40
#define MODBIT_RGUI 0x80

uint8_t codeToModifierMask(const String &code) {
  String c = unquote(code);
  String lc = c;
  lc.toLowerCase();
  if (lc == "shiftleft") return MODBIT_LSHIFT;
  if (lc == "shiftright") return MODBIT_RSHIFT;
  if (lc == "controlleft") return MODBIT_LCTRL;
  if (lc == "controlright") return MODBIT_RCTRL;
  if (lc == "altleft") return MODBIT_LALT;
  if (lc == "altright") return MODBIT_RALT;
  if (lc == "metaleft" || lc == "osleft" || lc == "superleft") return MODBIT_LGUI;
  if (lc == "metaright" || lc == "osright" || lc == "superright") return MODBIT_RGUI;
  return 0;
}

// Map KeyboardEvent.code -> Arduino Keyboard keycode:
// - Printable keys: return their unshifted ASCII ('a', '1', '-', etc.)
// - Special keys: return Arduino KEY_* constants.
// If not known, return 0 and we may try char fallback.
uint8_t codeToKeyCode(const String &code) {
  String c = unquote(code);

  // Letters KeyA..KeyZ -> 'a'..'z' (unshifted)
  if (c.length() == 4 && c.startsWith("Key")) {
    char L = c.charAt(3);
    if (L >= 'A' && L <= 'Z') {
      return (uint8_t)('a' + (L - 'A'));
    }
  }

  // Top-row digits Digit1..Digit0 (note: 0 comes after 9) -> '1'..'0'
  if (c.startsWith("Digit") && c.length() == 6) {
    char d = c.charAt(5);
    if (d >= '1' && d <= '9') return (uint8_t)d;
    if (d == '0') return (uint8_t)'0';
  }

  // Main punctuation (US, unshifted)
  if (c == "Minus") return (uint8_t)'-';
  if (c == "Equal") return (uint8_t)'=';
  if (c == "BracketLeft") return (uint8_t)'[';
  if (c == "BracketRight") return (uint8_t)']';
  if (c == "Backslash") return (uint8_t)'\\';
  if (c == "IntlBackslash") return (uint8_t)'\\';  // best-effort
  if (c == "Semicolon") return (uint8_t)';';
  if (c == "Quote") return (uint8_t)'\'';
  if (c == "Comma") return (uint8_t)',';
  if (c == "Period") return (uint8_t)'.';
  if (c == "Slash") return (uint8_t)'/';
  if (c == "Backquote") return (uint8_t)'`';
  if (c == "Space") return (uint8_t)' ';

  // Non-printable / navigation / editing
  if (c == "Enter") return KEY_RETURN;
  if (c == "NumpadEnter") return KEY_RETURN;  // keypad enter -> return
  if (c == "Backspace") return KEY_BACKSPACE;
  if (c == "Tab") return KEY_TAB;
  if (c == "Escape") return KEY_ESC;
  if (c == "Delete") return KEY_DELETE;
  if (c == "Insert") return KEY_INSERT;
  if (c == "Home") return KEY_HOME;
  if (c == "End") return KEY_END;
  if (c == "PageUp") return KEY_PAGE_UP;
  if (c == "PageDown") return KEY_PAGE_DOWN;
  if (c == "ArrowLeft") return KEY_LEFT_ARROW;
  if (c == "ArrowRight") return KEY_RIGHT_ARROW;
  if (c == "ArrowUp") return KEY_UP_ARROW;
  if (c == "ArrowDown") return KEY_DOWN_ARROW;
  if (c == "CapsLock") return KEY_CAPS_LOCK;
// These may not be present on all cores; if they aren't, they’ll be 0 and ignored.
// If your core defines them differently, adjust here:
#ifdef KEY_PRINT_SCREEN
  if (c == "PrintScreen") return KEY_PRINT_SCREEN;
#endif
#ifdef KEY_PAUSE
  if (c == "Pause") return KEY_PAUSE;
#endif

  // Function keys
  if (c.length() >= 2 && c.charAt(0) == 'F') {
    int fn = c.substring(1).toInt();
    if (fn >= 1 && fn <= 12) {
      switch (fn) {
        case 1: return KEY_F1;
        case 2: return KEY_F2;
        case 3: return KEY_F3;
        case 4: return KEY_F4;
        case 5: return KEY_F5;
        case 6: return KEY_F6;
        case 7: return KEY_F7;
        case 8: return KEY_F8;
        case 9: return KEY_F9;
        case 10: return KEY_F10;
        case 11: return KEY_F11;
        case 12: return KEY_F12;
      }
    }
  }

  // Numpad digits/operators -> map to printable equivalents
  if (c == "Numpad0") return (uint8_t)'0';
  if (c == "Numpad1") return (uint8_t)'1';
  if (c == "Numpad2") return (uint8_t)'2';
  if (c == "Numpad3") return (uint8_t)'3';
  if (c == "Numpad4") return (uint8_t)'4';
  if (c == "Numpad5") return (uint8_t)'5';
  if (c == "Numpad6") return (uint8_t)'6';
  if (c == "Numpad7") return (uint8_t)'7';
  if (c == "Numpad8") return (uint8_t)'8';
  if (c == "Numpad9") return (uint8_t)'9';
  if (c == "NumpadDecimal") return (uint8_t)'.';
  if (c == "NumpadAdd") return (uint8_t)'+';
  if (c == "NumpadSubtract") return (uint8_t)'-';
  if (c == "NumpadMultiply") return (uint8_t)'*';
  if (c == "NumpadDivide") return (uint8_t)'/';

  return 0;  // unknown
}

// Fallback for a single-character token -> base ASCII + optional SHIFT bit (unshifted code + needShift)
bool charToKeyCode(char ch, uint8_t &code, uint8_t &needShift) {
  code = 0;
  needShift = 0;

  // Letters
  if (ch >= 'a' && ch <= 'z') {
    code = (uint8_t)ch;
    return true;
  }
  if (ch >= 'A' && ch <= 'Z') {
    code = (uint8_t)(ch - 'A' + 'a');  // unshifted letter
    needShift = MODBIT_LSHIFT;         // use shift
    return true;
  }

  // Digits
  if (ch >= '1' && ch <= '9') {
    code = (uint8_t)ch;
    return true;
  }
  if (ch == '0') {
    code = (uint8_t)'0';
    return true;
  }

  // Space
  if (ch == ' ') {
    code = (uint8_t)' ';
    return true;
  }

  // Common US punctuation (unshifted)
  const char unshifted[] = "-=[]\\;'`,./";
  for (unsigned i = 0; i < sizeof(unshifted) - 1; i++) {
    if (ch == unshifted[i]) {
      code = (uint8_t)ch;
      return true;
    }
  }

  // Shifted variants -> map to base + SHIFT
  if (ch == '_') {
    code = (uint8_t)'-';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '+') {
    code = (uint8_t)'=';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '{') {
    code = (uint8_t)'[';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '}') {
    code = (uint8_t)']';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '|') {
    code = (uint8_t)'\\';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == ':') {
    code = (uint8_t)';';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '"') {
    code = (uint8_t)'\'';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '<') {
    code = (uint8_t)',';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '>') {
    code = (uint8_t)'.';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '?') {
    code = (uint8_t)'/';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '~') {
    code = (uint8_t)'`';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '!') {
    code = (uint8_t)'1';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '@') {
    code = (uint8_t)'2';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '#') {
    code = (uint8_t)'3';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '$') {
    code = (uint8_t)'4';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '%') {
    code = (uint8_t)'5';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '^') {
    code = (uint8_t)'6';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '&') {
    code = (uint8_t)'7';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '*') {
    code = (uint8_t)'8';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '(') {
    code = (uint8_t)'9';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == ')') {
    code = (uint8_t)'0';
    needShift = MODBIT_LSHIFT;
    return true;
  }

  return false;
}

// Apply a "frame": press/release differences between prev and current states
void sendFrame() {
  // 1) Modifiers diff
  auto modPressRelease = [](bool press, uint8_t bit, uint8_t keycode) {
    if (press) Keyboard.press(keycode);
    else Keyboard.release(keycode);
  };

  uint8_t changed = modMask ^ prevModMask;
  if (changed) {
    // LCTRL
    if (changed & MODBIT_LCTRL) modPressRelease(modMask & MODBIT_LCTRL, MODBIT_LCTRL, KEY_LEFT_CTRL);
    if (changed & MODBIT_LSHIFT) modPressRelease(modMask & MODBIT_LSHIFT, MODBIT_LSHIFT, KEY_LEFT_SHIFT);
    if (changed & MODBIT_LALT) modPressRelease(modMask & MODBIT_LALT, MODBIT_LALT, KEY_LEFT_ALT);
    if (changed & MODBIT_LGUI) modPressRelease(modMask & MODBIT_LGUI, MODBIT_LGUI, KEY_LEFT_GUI);
    if (changed & MODBIT_RCTRL) modPressRelease(modMask & MODBIT_RCTRL, MODBIT_RCTRL, KEY_RIGHT_CTRL);
    if (changed & MODBIT_RSHIFT) modPressRelease(modMask & MODBIT_RSHIFT, MODBIT_RSHIFT, KEY_RIGHT_SHIFT);
    if (changed & MODBIT_RALT) modPressRelease(modMask & MODBIT_RALT, MODBIT_RALT, KEY_RIGHT_ALT);
    if (changed & MODBIT_RGUI) modPressRelease(modMask & MODBIT_RGUI, MODBIT_RGUI, KEY_RIGHT_GUI);
    prevModMask = modMask;
  }

  // 2) Normal keys diff: release keys that disappeared
  for (int i = 0; i < 6; i++) {
    uint8_t k = prevKeys[i];
    if (k != 0 && !arrContains(keys, k)) {
      Keyboard.release(k);
    }
  }
  // 3) Press keys that are new
  for (int i = 0; i < 6; i++) {
    uint8_t k = keys[i];
    if (k != 0 && !arrContains(prevKeys, k)) {
      Keyboard.press(k);
    }
  }
  // Update prevKeys snapshot
  for (int i = 0; i < 6; i++) prevKeys[i] = keys[i];
}

// Apply KDOWN/KUP using micro manager state.
void handleKeyEvent(bool isDown, const String &codeTok, const String &keyTok) {
  // 1) If it's a modifier code, toggle modMask and send frame.
  uint8_t mm = codeToModifierMask(codeTok);
  if (mm) {
    if (isDown) modMask |= mm;
    else modMask &= ~mm;
    sendFrame();
    return;
  }

  // 2) Map KeyboardEvent.code -> ASCII or KEY_* keycode
  uint8_t kc = codeToKeyCode(codeTok);

  // 3) Fallback: if not mapped and keyTok is a single character, map that to ASCII + optional SHIFT
  if (kc == 0) {
    String k = unquote(keyTok);
    if (k.length() == 1) {
      uint8_t needShift = 0;
      if (charToKeyCode((char)k.charAt(0), kc, needShift) && kc != 0) {
        if (isDown) {
          // temporarily add shift if needed for this single key press
          bool addedShift = false;
          if (needShift && !(modMask & needShift)) {
            modMask |= needShift;
            addedShift = true;
          }
          if (addKey(kc)) sendFrame();
          // If you want per-stroke shift strictly (release on KUP), uncomment in KUP path below.
        } else {
          if (removeKey(kc)) {
            // Optional: if this key needed a temporary shift, drop it now (if you tracked it)
            // modMask &= ~needShift;  // uncomment if you want per-stroke shift
            sendFrame();
          }
        }
        return;
      }
    }
  }

  // 4) If still unknown, ignore.
  if (kc == 0) return;

  // 5) Normal key handling via 6-key state
  if (isDown) {
    if (addKey(kc)) sendFrame();
  } else {
    if (removeKey(kc)) sendFrame();
  }
}

// ---------------- Main loop ----------------
void loop() {
  String line = readLine();
  if (line.length() == 0) return;

  // Tokenize while preserving quoted tokens
  const int MAX_TOK = 16;
  String tok[MAX_TOK];
  int n = 0;
  bool inQ = false;
  String cur = "";
  for (unsigned int i = 0; i < line.length(); i++) {
    char ch = line.charAt(i);
    if (ch == '"') {
      inQ = !inQ;
      continue;
    }
    if (!inQ && isWhitespace(ch)) {
      if (cur.length() > 0 && n < MAX_TOK) {
        tok[n++] = cur;
        cur = "";
      }
    } else {
      cur += ch;
    }
  }
  if (cur.length() > 0 && n < MAX_TOK) tok[n++] = cur;
  if (n == 0) return;

  String cmd = tok[0];
  cmd.toUpperCase();

  if (cmd == "MREL" && n >= 3) {
    int dx = tok[1].toInt();
    int dy = tok[2].toInt();
    chunkedMouseMove(dx, dy);
    if (lastMouseX != INT32_MIN && lastMouseY != INT32_MIN) {
      lastMouseX += dx;
      lastMouseY += dy;
    }
  } else if (cmd == "MABS" && n >= 3) {
    int32_t tx = tok[1].toInt();
    int32_t ty = tok[2].toInt();
    if (lastMouseX == INT32_MIN || lastMouseY == INT32_MIN) {
      lastMouseX = tx;
      lastMouseY = ty;
      Serial.print("MABS initial sync, tracking set but no movement\n");
    } else {
      int dx = (int)(tx - lastMouseX);
      int dy = (int)(ty - lastMouseY);
      if (dx != 0 || dy != 0) {
        chunkedMouseMove(dx, dy);
        lastMouseX = tx;
        lastMouseY = ty;
      }
    }
  } else if (cmd == "SETPOS" && n >= 3) {
    lastMouseX = tok[1].toInt();
    lastMouseY = tok[2].toInt();
    Serial.print("SETPOS tracked: ");
    Serial.print(lastMouseX);
    Serial.print(", ");
    Serial.println(lastMouseY);
  } else if (cmd == "MDOWN" && n >= 2) {
    int b = tok[1].toInt();  // 0 L, 1 M, 2 R
    if (b == 0) Mouse.press(MOUSE_LEFT);
    else if (b == 1) Mouse.press(MOUSE_MIDDLE);
    else if (b == 2) Mouse.press(MOUSE_RIGHT);
  } else if (cmd == "MUP" && n >= 2) {
    int b = tok[1].toInt();
    if (b == 0) Mouse.release(MOUSE_LEFT);
    else if (b == 1) Mouse.release(MOUSE_MIDDLE);
    else if (b == 2) Mouse.release(MOUSE_RIGHT);
  } else if (cmd == "MWHEEL" && n >= 3) {
    int dx = tok[1].toInt();
    int dy = tok[2].toInt();
    chunkedWheel(dx, dy);
  } else if ((cmd == "KDOWN" || cmd == "KUP") && n >= 7) {
    bool isDown = (cmd == "KDOWN");
    String codeTok = tok[1];
    String keyTok = tok[2];
    // We ignore ctrl/shift/alt/meta flags in the remaining tokens and rely on explicit modifier key events.
    handleKeyEvent(isDown, codeTok, keyTok);
  } else if (cmd == "KTEXT" && n >= 2) {
    // Rejoin tokens with spaces (they came from a quoted string already split)
    String s = "";
    for (int i = 1; i < n; i++) {
      if (i > 1) s += ' ';
      s += tok[i];
    }
    // Type it with a tiny pacing
    for (unsigned int i = 0; i < s.length(); i++) {
      Keyboard.print(s.charAt(i));
      delay(2);
    }
  } else if (cmd == "RELALLKEYS") {
    Serial.println("Releasing all keys..");
    // Clear our state and sync to host
    modMask = 0;
    for (int i = 0; i < 6; i++) keys[i] = 0;
    sendFrame();            // releases modifiers and any tracked keys
    Keyboard.releaseAll();  // safety net in case anything was missed
    for (int i = 0; i < 6; i++) prevKeys[i] = 0;
    prevModMask = 0;
    Serial.println("Released all keys!");
  } else if (cmd == "RAW" && n >= 2) {
    Serial.print("RAW: ");
    for (int i = 1; i < n; i++) {
      Serial.print(tok[i]);
      if (i + 1 < n) Serial.print(' ');
    }
    Serial.println();
  } else {
    Serial.print("Unknown: ");
    Serial.println(line);
  }
}