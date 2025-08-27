// teensy-hid-bridge.ino
// Teensy 3.1: Serial -> USB HID Keyboard/Mouse
// FIX: KDOWN/KUP now use Teensy's micro manager API (set_modifier/set_key*/send_now)
// so special keys like Backspace, Tab, Enter are sent as HID key codes, not ASCII.
//
// Supported commands (one per line):
//   MREL dx dy
//   MDOWN n
//   MUP n
//   MWHEEL dx dy
//   KDOWN code key ctrl shift alt meta
//   KUP   code key ctrl shift alt meta
//   RAW text
//
// Reference: Teensyduino USB Keyboard library (press/release and micro manager) [1]

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ;
  Keyboard.begin();
  Mouse.begin();
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
    delay(3);
  }
}

void chunkedWheel(int dx, int dy) {
  const int MAX_STEP = 127;
  // Vertical wheel (Teensy uses third arg of Mouse.move)
  while (dy != 0) {
    int step = (dy > 0) ? min(dy, MAX_STEP) : max(dy, -MAX_STEP);
    Mouse.move(0, 0, step);
    dy -= step;
    delay(3);
  }
  // Horizontal wheel ignored by default; add mapping if you need it.
}

// ---------------- Keyboard micro manager state ----------------
//
// We maintain:
// - modMask: MODIFIERKEY_* bitmask
// - keys[6]: up to 6 currently pressed normal keys (KEY_* / KEYPAD_*)
// Then push with Keyboard.set_modifier + set_key1..6 + send_now() [1].

uint8_t modMask = 0;
uint8_t keys[6] = { 0, 0, 0, 0, 0, 0 };

String unquote(String s) {
  if (s.startsWith("\"") && s.endsWith("\"") && s.length() >= 2) {
    return s.substring(1, s.length() - 1);
  }
  return s;
}

void sendFrame() {
  Keyboard.set_modifier(modMask);  // MODIFIERKEY_* bitmask [1]
  Keyboard.set_key1(keys[0]);
  Keyboard.set_key2(keys[1]);
  Keyboard.set_key3(keys[2]);
  Keyboard.set_key4(keys[3]);
  Keyboard.set_key5(keys[4]);
  Keyboard.set_key6(keys[5]);
  Keyboard.send_now();  // send HID state frame [1]
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
  // no room: drop (or replace oldest if you prefer)
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

// ---------------- Mapping: code -> MODIFIERKEY_* or KEY_* ----------------

uint8_t codeToModifierMask(const String &code) {
  String c = unquote(code);
  String lc = c;
  lc.toLowerCase();
  if (lc == "shiftleft") return MODIFIERKEY_SHIFT;
  if (lc == "shiftright") return MODIFIERKEY_RIGHT_SHIFT;
  if (lc == "controlleft") return MODIFIERKEY_CTRL;
  if (lc == "controlright") return MODIFIERKEY_RIGHT_CTRL;
  if (lc == "altleft") return MODIFIERKEY_ALT;
  if (lc == "altright") return MODIFIERKEY_RIGHT_ALT;
  if (lc == "metaleft" || lc == "osleft" || lc == "superleft") return MODIFIERKEY_GUI;
  if (lc == "metaright" || lc == "osright" || lc == "superright") return MODIFIERKEY_RIGHT_GUI;
  return 0;
}

// Map KeyboardEvent.code to Teensy KEY_* / KEYPAD_* usage codes.
// Only usage codes here; NO ASCII.
uint8_t codeToKeyCode(const String &code) {
  String c = unquote(code);

  // Letters KeyA..KeyZ
  if (c.length() == 4 && c.startsWith("Key")) {
    char L = c.charAt(3);
    if (L >= 'A' && L <= 'Z') {
      // Teensy has KEY_A..KEY_Z [1]
      return KEY_A + (L - 'A');
    }
  }
  // Top-row digits Digit1..Digit0 (note: 0 comes after 9)
  if (c.startsWith("Digit") && c.length() == 6) {
    char d = c.charAt(5);
    if (d >= '1' && d <= '9') return KEY_1 + (d - '1');
    if (d == '0') return KEY_0;
  }

  // Main punctuation (US) [1]
  if (c == "Minus") return KEY_MINUS;
  if (c == "Equal") return KEY_EQUAL;
  if (c == "BracketLeft") return KEY_LEFT_BRACE;
  if (c == "BracketRight") return KEY_RIGHT_BRACE;
  if (c == "Backslash") return KEY_BACKSLASH;
  if (c == "IntlBackslash") return KEY_BACKSLASH;  // best-effort
  if (c == "Semicolon") return KEY_SEMICOLON;
  if (c == "Quote") return KEY_QUOTE;
  if (c == "Comma") return KEY_COMMA;
  if (c == "Period") return KEY_PERIOD;
  if (c == "Slash") return KEY_SLASH;
  if (c == "Backquote") return KEY_TILDE;  // backtick/tilde key
  if (c == "Space") return KEY_SPACE;

  // Non-printable / navigation / editing [1]
  if (c == "Enter") return KEY_ENTER;
  if (c == "NumpadEnter") return KEYPAD_ENTER;
  if (c == "Backspace") return KEY_BACKSPACE;
  if (c == "Tab") return KEY_TAB;
  if (c == "Escape") return KEY_ESC;
  if (c == "Delete") return KEY_DELETE;
  if (c == "Insert") return KEY_INSERT;
  if (c == "Home") return KEY_HOME;
  if (c == "End") return KEY_END;
  if (c == "PageUp") return KEY_PAGE_UP;
  if (c == "PageDown") return KEY_PAGE_DOWN;
  if (c == "ArrowLeft") return KEY_LEFT;
  if (c == "ArrowRight") return KEY_RIGHT;
  if (c == "ArrowUp") return KEY_UP;
  if (c == "ArrowDown") return KEY_DOWN;
  if (c == "CapsLock") return KEY_CAPS_LOCK;
  if (c == "PrintScreen") return KEY_PRINTSCREEN;
  if (c == "Pause") return KEY_PAUSE;

  // Function keys [1]
  if (c.length() >= 2 && c.charAt(0) == 'F') {
    int fn = c.substring(1).toInt();
    if (fn >= 1 && fn <= 12) return KEY_F1 + (fn - 1);
  }

  // Numpad digits/operators [1]
  if (c == "Numpad0") return KEYPAD_0;
  if (c == "Numpad1") return KEYPAD_1;
  if (c == "Numpad2") return KEYPAD_2;
  if (c == "Numpad3") return KEYPAD_3;
  if (c == "Numpad4") return KEYPAD_4;
  if (c == "Numpad5") return KEYPAD_5;
  if (c == "Numpad6") return KEYPAD_6;
  if (c == "Numpad7") return KEYPAD_7;
  if (c == "Numpad8") return KEYPAD_8;
  if (c == "Numpad9") return KEYPAD_9;
  if (c == "NumpadDecimal") return KEYPAD_PERIOD;
  if (c == "NumpadAdd") return KEYPAD_PLUS;
  if (c == "NumpadSubtract") return KEYPAD_MINUS;
  if (c == "NumpadMultiply") return KEYPAD_ASTERIX;
  if (c == "NumpadDivide") return KEYPAD_SLASH;

  return 0;  // unknown
}

// Optional fallback for single-character key tokens, mapped to KEY_* codes.
// Avoids ASCII path entirely.
bool charToKeyCode(char ch, uint8_t &code, uint8_t &needShift) {
  code = 0;
  needShift = 0;
  // Letters
  if (ch >= 'a' && ch <= 'z') {
    code = KEY_A + (ch - 'a');
    return true;
  }
  if (ch >= 'A' && ch <= 'Z') {
    code = KEY_A + (ch - 'A');
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  // Digits
  if (ch >= '1' && ch <= '9') {
    code = KEY_1 + (ch - '1');
    return true;
  }
  if (ch == '0') {
    code = KEY_0;
    return true;
  }
  // Space
  if (ch == ' ') {
    code = KEY_SPACE;
    return true;
  }
  // Common US punctuation (unshifted)
  if (ch == '-') {
    code = KEY_MINUS;
    return true;
  }
  if (ch == '=') {
    code = KEY_EQUAL;
    return true;
  }
  if (ch == '[') {
    code = KEY_LEFT_BRACE;
    return true;
  }
  if (ch == ']') {
    code = KEY_RIGHT_BRACE;
    return true;
  }
  if (ch == '\\') {
    code = KEY_BACKSLASH;
    return true;
  }
  if (ch == ';') {
    code = KEY_SEMICOLON;
    return true;
  }
  if (ch == '\'') {
    code = KEY_QUOTE;
    return true;
  }
  if (ch == ',') {
    code = KEY_COMMA;
    return true;
  }
  if (ch == '.') {
    code = KEY_PERIOD;
    return true;
  }
  if (ch == '/') {
    code = KEY_SLASH;
    return true;
  }
  if (ch == '`') {
    code = KEY_TILDE;
    return true;
  }
  // Shifted variants (examples)
  if (ch == '_') {
    code = KEY_MINUS;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '+') {
    code = KEY_EQUAL;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '{') {
    code = KEY_LEFT_BRACE;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '}') {
    code = KEY_RIGHT_BRACE;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '|') {
    code = KEY_BACKSLASH;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == ':') {
    code = KEY_SEMICOLON;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '"') {
    code = KEY_QUOTE;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '<') {
    code = KEY_COMMA;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '>') {
    code = KEY_PERIOD;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '?') {
    code = KEY_SLASH;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '~') {
    code = KEY_TILDE;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '!') {
    code = KEY_1;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '@') {
    code = KEY_2;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '#') {
    code = KEY_3;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '$') {
    code = KEY_4;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '%') {
    code = KEY_5;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '^') {
    code = KEY_6;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '&') {
    code = KEY_7;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '*') {
    code = KEY_8;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == '(') {
    code = KEY_9;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  if (ch == ')') {
    code = KEY_0;
    needShift = MODIFIERKEY_SHIFT;
    return true;
  }
  return false;
}

// Apply KDOWN/KUP using micro manager state.
void handleKeyEvent(bool isDown, const String &codeTok, const String &keyTok) {
  // 1) If it's a modifier code, toggle modMask and send frame [1].
  uint8_t mm = codeToModifierMask(codeTok);
  if (mm) {
    if (isDown) modMask |= mm;
    else modMask &= ~mm;
    sendFrame();
    return;
  }

  // 2) Try to map KeyboardEvent.code to a KEY_* / KEYPAD_* usage code.
  uint8_t kc = codeToKeyCode(codeTok);

  // 3) Fallback: if not mapped and keyTok is a single character, map that to KEY_*.
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
          // Note: we do NOT auto-remove shift here; release happens when KUP arrives.
          // If you want per-stroke shift, uncomment below in KUP path.
        } else {
          if (removeKey(kc)) {
            // Optional: if this key needed a temporary shift, drop it now.
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
    // Note: We ignore ctrl/shift/alt/meta flags here and rely on explicit modifier key events.
    handleKeyEvent(isDown, codeTok, keyTok);
  } else if (cmd == "KTEXT" && n >= 2) {
    // Rejoin tokens with spaces (they came from a quoted string already split)
    String s = "";
    for (int i = 1; i < n; i++) {
      if (i > 1) s += ' ';
      s += tok[i];
    }
    // s is unquoted text (quotes stripped by tokenizer). Type it.
    // Small chunks to avoid overwhelming host
    for (unsigned int i = 0; i < s.length(); i++) {
      Keyboard.print(s.charAt(i));
      delay(2);
    }
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