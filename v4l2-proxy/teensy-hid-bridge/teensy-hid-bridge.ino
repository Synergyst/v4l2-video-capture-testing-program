#include <Arduino.h>
#include <WiFi.h>  // Pico W / Pico 2 W (Earle Philhower RP2040 core)
#include <ArduinoJson.h>
#include <Keyboard.h>
#include <Mouse.h>
#include <limits.h>
#include <math.h>
#include "pico/multicore.h"
#include "pico/critical_section.h"
#include <WebSocketsServer.h>  // Links2004 WebSocket server (needed for runtime toggle)

// ====================== WiFi Config ======================
#define WIFI_SSID "WRTWRTWRTGRT"
#define WIFI_PASS "JeromeCrackie!"  // set your password

// ====================== Remote Screen Info ======================
#define DEFAULT_XRES 1920
#define DEFAULT_YRES 1080
struct FbInfo {
  int w = DEFAULT_XRES;
  int h = DEFAULT_YRES;
} fbInfo;
struct MousePos {
  int x = fbInfo.w / 2;
  int y = fbInfo.h / 2;
} lastMousePos;

// ====================== Debug/Event Logging Flags ======================
volatile bool gLogKeys = false;
volatile bool gLogMouse = false;

// ====================== Mouse Shared State (core0 -> core1) ======================
volatile int32_t g_acc_dx = 0;
volatile int32_t g_acc_dy = 0;
volatile int32_t g_acc_wheelY = 0;
volatile uint8_t g_desiredButtons = 0;  // MOUSE_LEFT|MOUSE_MIDDLE|MOUSE_RIGHT
critical_section_t gMouseCS;

// ====================== Transport runtime toggle ======================
enum TransportMode : uint8_t { TRANS_TCP = 0,
                               TRANS_WS = 1 };
static TransportMode gTransport = TRANS_TCP;
static const uint16_t CONTROL_PORT = 1444;

// TCP server and state
static const int MAX_CLIENTS = 4;
WiFiServer* tcpServer = nullptr;
WiFiClient clients[MAX_CLIENTS];
static String lineBufs[MAX_CLIENTS];

// WS server and state
WebSocketsServer* wsServer = nullptr;
static const int WS_MAX_CLIENTS = 8;
String wsFragments[WS_MAX_CLIENTS];  // accumulators for continuation frames

// Forward declarations for WS handlers
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);

// ====================== Keyboard Micro-manager ======================
#define MODBIT_LCTRL 0x01
#define MODBIT_LSHIFT 0x02
#define MODBIT_LALT 0x04
#define MODBIT_LGUI 0x08
#define MODBIT_RCTRL 0x10
#define MODBIT_RSHIFT 0x20
#define MODBIT_RALT 0x40
#define MODBIT_RGUI 0x80

static uint8_t modMask = 0;
static uint8_t keys[6] = { 0, 0, 0, 0, 0, 0 };
static uint8_t prevModMask = 0;
static uint8_t prevKeys[6] = { 0, 0, 0, 0, 0, 0 };

// ====================== WiFi / Serial utils ======================
static String serialBuf;

static inline void mouseEnqueueRel(int dx, int dy) {
  if (gLogMouse) {
    Serial.print("[mouse] rel dx=");
    Serial.print(dx);
    Serial.print(" dy=");
    Serial.println(dy);
  }
  critical_section_enter_blocking(&gMouseCS);
  g_acc_dx += dx;
  g_acc_dy += dy;
  critical_section_exit(&gMouseCS);
}
static inline void mouseEnqueueWheel(int dy) {
  if (!dy) return;
  if (gLogMouse) {
    Serial.print("[mouse] wheel dy=");
    Serial.println(dy);
  }
  critical_section_enter_blocking(&gMouseCS);
  g_acc_wheelY += dy;
  critical_section_exit(&gMouseCS);
}
static inline void mouseSetButton(uint8_t btnBit, bool down) {
  if (gLogMouse) {
    const char* name = (btnBit == MOUSE_LEFT) ? "L" : (btnBit == MOUSE_MIDDLE) ? "M"
                                                    : (btnBit == MOUSE_RIGHT)  ? "R"
                                                                               : "?";
    Serial.print("[mouse] button ");
    Serial.print(name);
    Serial.print(' ');
    Serial.println(down ? "down" : "up");
  }
  critical_section_enter_blocking(&gMouseCS);
  if (down) g_desiredButtons |= btnBit;
  else g_desiredButtons &= ~btnBit;
  critical_section_exit(&gMouseCS);
}

static inline bool arrContains(const uint8_t arr[6], uint8_t k) {
  for (int i = 0; i < 6; i++)
    if (arr[i] == k) return true;
  return false;
}
bool addKey(uint8_t k) {
  if (k == 0) return false;
  for (int i = 0; i < 6; i++)
    if (keys[i] == k) return false;
  for (int i = 0; i < 6; i++)
    if (keys[i] == 0) {
      keys[i] = k;
      return true;
    }
  return false;
}
bool removeKey(uint8_t k) {
  if (k == 0) return false;
  bool removed = false;
  for (int i = 0; i < 6; i++)
    if (keys[i] == k) {
      keys[i] = 0;
      removed = true;
    }
  if (removed) {
    uint8_t tmp[6] = { 0, 0, 0, 0, 0, 0 };
    int j = 0;
    for (int i = 0; i < 6; i++)
      if (keys[i]) tmp[j++] = keys[i];
    for (int i = 0; i < 6; i++) keys[i] = tmp[i];
  }
  return removed;
}

uint8_t codeToModifierMask(const String& code) {
  String lc = code;
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
uint8_t codeToKeyCode(const String& code) {
  String c = code;
  if (c.length() == 4 && c.startsWith("Key")) {
    char L = c.charAt(3);
    if (L >= 'A' && L <= 'Z') return (uint8_t)('a' + (L - 'A'));
  }
  if (c.startsWith("Digit") && c.length() == 6) {
    char d = c.charAt(5);
    if (d >= '1' && d <= '9') return (uint8_t)d;
    if (d == '0') return (uint8_t)'0';
  }
  if (c == "Minus") return (uint8_t)'-';
  if (c == "Equal") return (uint8_t)'=';
  if (c == "BracketLeft") return (uint8_t)'[';
  if (c == "BracketRight") return (uint8_t)']';
  if (c == "Backslash") return (uint8_t)'\\';
  if (c == "IntlBackslash") return (uint8_t)'\\';
  if (c == "Semicolon") return (uint8_t)';';
  if (c == "Quote") return (uint8_t)'\'';
  if (c == "Comma") return (uint8_t)',';
  if (c == "Period") return (uint8_t)'.';
  if (c == "Slash") return (uint8_t)'/';
  if (c == "Backquote") return (uint8_t)'`';
  if (c == "Space") return (uint8_t)' ';
  if (c == "Enter") return KEY_RETURN;
  if (c == "NumpadEnter") return KEY_RETURN;
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
  if (c.length() >= 2 && c.charAt(0) == 'F') {
    int fn = c.substring(1).toInt();
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
  return 0;
}
bool charToKeyCode(char ch, uint8_t& code, uint8_t& needShift) {
  code = 0;
  needShift = 0;
  if (ch >= 'a' && ch <= 'z') {
    code = (uint8_t)ch;
    return true;
  }
  if (ch >= 'A' && ch <= 'Z') {
    code = (uint8_t)(ch - 'A' + 'a');
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch >= '1' && ch <= '9') {
    code = (uint8_t)ch;
    return true;
  }
  if (ch == '0') {
    code = (uint8_t)'0';
    return true;
  }
  if (ch == ' ') {
    code = (uint8_t)' ';
    return true;
  }
  const char unshifted[] = "-=[]\\;'`,./";
  for (unsigned i = 0; i < sizeof(unshifted) - 1; i++)
    if (ch == unshifted[i]) {
      code = (uint8_t)ch;
      return true;
    }
  if (ch == '_') {
    code = '-';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '+') {
    code = '=';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '{') {
    code = '[';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '}') {
    code = ']';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '|') {
    code = '\\';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == ':') {
    code = ';';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '"') {
    code = '\'';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '<') {
    code = ',';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '>') {
    code = '.';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '?') {
    code = '/';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '~') {
    code = '`';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '!') {
    code = '1';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '@') {
    code = '2';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '#') {
    code = '3';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '$') {
    code = '4';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '%') {
    code = '5';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '^') {
    code = '6';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '&') {
    code = '7';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '*') {
    code = '8';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == '(') {
    code = '9';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  if (ch == ')') {
    code = '0';
    needShift = MODBIT_LSHIFT;
    return true;
  }
  return false;
}
void sendFrame() {
  auto modPressRelease = [](bool press, uint8_t keycode) {
    if (press) Keyboard.press(keycode);
    else Keyboard.release(keycode);
  };
  uint8_t changed = modMask ^ prevModMask;
  if (changed) {
    if (changed & MODBIT_LCTRL) modPressRelease(modMask & MODBIT_LCTRL, KEY_LEFT_CTRL);
    if (changed & MODBIT_LSHIFT) modPressRelease(modMask & MODBIT_LSHIFT, KEY_LEFT_SHIFT);
    if (changed & MODBIT_LALT) modPressRelease(modMask & MODBIT_LALT, KEY_LEFT_ALT);
    if (changed & MODBIT_LGUI) modPressRelease(modMask & MODBIT_LGUI, KEY_LEFT_GUI);
    if (changed & MODBIT_RCTRL) modPressRelease(modMask & MODBIT_RCTRL, KEY_RIGHT_CTRL);
    if (changed & MODBIT_RSHIFT) modPressRelease(modMask & MODBIT_RSHIFT, KEY_RIGHT_SHIFT);
    if (changed & MODBIT_RALT) modPressRelease(modMask & MODBIT_RALT, KEY_RIGHT_ALT);
    if (changed & MODBIT_RGUI) modPressRelease(modMask & MODBIT_RGUI, KEY_RIGHT_GUI);
    prevModMask = modMask;
  }
  for (int i = 0; i < 6; i++) {
    uint8_t k = prevKeys[i];
    if (k != 0 && !arrContains(keys, k)) Keyboard.release(k);
  }
  for (int i = 0; i < 6; i++) {
    uint8_t k = keys[i];
    if (k != 0 && !arrContains(prevKeys, k)) Keyboard.press(k);
  }
  for (int i = 0; i < 6; i++) prevKeys[i] = keys[i];
}
// Handle one keyboard event using our micro-manager
void handleKeyEvent(bool isDown, const String& codeTok, const String& keyTok) {
  uint8_t mm = codeToModifierMask(codeTok);
  if (mm) {
    if (gLogKeys) {
      Serial.print("[key] ");
      Serial.print(isDown ? "down" : "up");
      Serial.print(" MOD(");
      Serial.print(codeTok);
      Serial.print(") mask=0x");
      Serial.println(mm, HEX);
    }
    if (isDown) modMask |= mm;
    else modMask &= ~mm;
    sendFrame();
    return;
  }
  uint8_t kc = codeToKeyCode(codeTok);
  if (kc == 0) {
    if (keyTok.length() == 1) {
      uint8_t needShift = 0;
      if (charToKeyCode((char)keyTok.charAt(0), kc, needShift) && kc != 0) {
        if (gLogKeys) {
          Serial.print("[key] ");
          Serial.print(isDown ? "down" : "up");
          Serial.print(" char='");
          Serial.print(keyTok);
          Serial.print("' ascii=");
          Serial.print((int)kc);
          Serial.print(" needShift=0x");
          Serial.println(needShift, HEX);
        }
        if (isDown) {
          if (needShift && !(modMask & needShift)) modMask |= needShift;
          if (addKey(kc)) sendFrame();
        } else {
          if (removeKey(kc)) sendFrame();
        }
        return;
      }
    }
  }
  if (kc == 0) {
    if (gLogKeys) {
      Serial.print("[key] ignored ");
      Serial.print(isDown ? "down " : "up ");
      Serial.print("code='");
      Serial.print(codeTok);
      Serial.print("' key='");
      Serial.print(keyTok);
      Serial.println("'");
    }
    return;
  }
  if (gLogKeys) {
    Serial.print("[key] ");
    Serial.print(isDown ? "down" : "up");
    Serial.print(" code='");
    Serial.print(codeTok);
    Serial.print("' key='");
    Serial.print(keyTok);
    Serial.print("' kc=");
    Serial.println((int)kc);
  }
  if (isDown) {
    if (addKey(kc)) sendFrame();
  } else {
    if (removeKey(kc)) sendFrame();
  }
}

// ====================== Mouse Worker (core1) ======================
void core1MouseTask() {
  const int MAX_STEP = 127;
  uint8_t prevButtons = 0;
  for (;;) {
    int32_t dx = 0, dy = 0, wy = 0;
    uint8_t desiredButtons = 0;
    critical_section_enter_blocking(&gMouseCS);
    dx = g_acc_dx;
    g_acc_dx = 0;
    dy = g_acc_dy;
    g_acc_dy = 0;
    wy = g_acc_wheelY;
    g_acc_wheelY = 0;
    desiredButtons = g_desiredButtons;
    critical_section_exit(&gMouseCS);
    // Buttons
    uint8_t changed = desiredButtons ^ prevButtons;
    if (changed) {
      if (changed & MOUSE_LEFT) { (desiredButtons & MOUSE_LEFT) ? Mouse.press(MOUSE_LEFT) : Mouse.release(MOUSE_LEFT); }
      if (changed & MOUSE_MIDDLE) { (desiredButtons & MOUSE_MIDDLE) ? Mouse.press(MOUSE_MIDDLE) : Mouse.release(MOUSE_MIDDLE); }
      if (changed & MOUSE_RIGHT) { (desiredButtons & MOUSE_RIGHT) ? Mouse.press(MOUSE_RIGHT) : Mouse.release(MOUSE_RIGHT); }
      prevButtons = desiredButtons;
    }
    // Relative move with compensation
    if (dx != 0 || dy != 0) {
      while (dx != 0 || dy != 0) {
        int sx = (dx > 0) ? min<int32_t>(dx, MAX_STEP) : max<int32_t>(dx, -MAX_STEP);
        int sy = (dy > 0) ? min<int32_t>(dy, MAX_STEP) : max<int32_t>(dy, -MAX_STEP);
        Mouse.move(sx, sy, 0);
        dx -= sx;
        dy -= sy;
        critical_section_enter_blocking(&gMouseCS);
        if (g_acc_dx || g_acc_dy || g_acc_wheelY || g_desiredButtons != desiredButtons) {
          dx += g_acc_dx;
          g_acc_dx = 0;
          dy += g_acc_dy;
          g_acc_dy = 0;
          wy += g_acc_wheelY;
          g_acc_wheelY = 0;
          desiredButtons = g_desiredButtons;
        }
        critical_section_exit(&gMouseCS);
        tight_loop_contents();
      }
    }
    // Wheel (vertical) with compensation
    if (wy != 0) {
      while (wy != 0) {
        int step = (wy > 0) ? min<int32_t>(wy, MAX_STEP) : max<int32_t>(wy, -MAX_STEP);
        Mouse.move(0, 0, step);
        wy -= step;
        critical_section_enter_blocking(&gMouseCS);
        if (g_acc_wheelY || g_acc_dx || g_acc_dy || g_desiredButtons != desiredButtons) {
          wy += g_acc_wheelY;
          g_acc_wheelY = 0;
          dx += g_acc_dx;
          g_acc_dx = 0;
          dy += g_acc_dy;
          g_acc_dy = 0;
          desiredButtons = g_desiredButtons;
        }
        critical_section_exit(&gMouseCS);
        tight_loop_contents();
      }
    }
    if (!(changed || dx || dy || wy)) delayMicroseconds(200);
  }
}

// ====================== Helpers ======================
static inline double clamp01(double v) {
  return v < 0 ? 0 : (v > 1 ? 1 : v);
}
void handleAbsoluteMove(double nx, double ny) {
  nx = clamp01(nx);
  ny = clamp01(ny);
  int targetX = (int)llround(nx * (fbInfo.w - 1));
  int targetY = (int)llround(ny * (fbInfo.h - 1));
  int dx = targetX - lastMousePos.x;
  int dy = targetY - lastMousePos.y;
  if (dx == 0 && dy == 0) return;
  lastMousePos.x = max(0, min(fbInfo.w - 1, lastMousePos.x + dx));
  lastMousePos.y = max(0, min(fbInfo.h - 1, lastMousePos.y + dy));
  if (gLogMouse) {
    Serial.print("[mouse] abs nx=");
    Serial.print(nx, 4);
    Serial.print(" ny=");
    Serial.print(ny, 4);
    Serial.print(" -> rel dx=");
    Serial.print(dx);
    Serial.print(" dy=");
    Serial.print(dy);
    Serial.print(" new=(");
    Serial.print(lastMousePos.x);
    Serial.print(",");
    Serial.print(lastMousePos.y);
    Serial.println(")");
  }
  mouseEnqueueRel(dx, dy);
}

// Send info on connect (both transports)
void sendInfoRemoteSizeTCP(WiFiClient& c) {
  StaticJsonDocument<128> doc;
  doc["type"] = "info";
  JsonObject rs = doc.createNestedObject("remoteSize");
  rs["w"] = fbInfo.w;
  rs["h"] = fbInfo.h;
  String s;
  serializeJson(doc, s);
  s += "\n";
  c.print(s);
  StaticJsonDocument<128> pos;
  pos["type"] = "info";
  JsonObject mp = pos.createNestedObject("mousePos");
  mp["x"] = lastMousePos.x;
  mp["y"] = lastMousePos.y;
  s = "";
  serializeJson(pos, s);
  s += "\n";
  c.print(s);
}
void sendInfoRemoteSizeWS(uint8_t num) {
  if (!wsServer) return;
  StaticJsonDocument<128> doc;
  doc["type"] = "info";
  JsonObject rs = doc.createNestedObject("remoteSize");
  rs["w"] = fbInfo.w;
  rs["h"] = fbInfo.h;
  String s;
  serializeJson(doc, s);
  wsServer->sendTXT(num, s);
  StaticJsonDocument<128> pos;
  pos["type"] = "info";
  JsonObject mp = pos.createNestedObject("mousePos");
  mp["x"] = lastMousePos.x;
  mp["y"] = lastMousePos.y;
  s = "";
  serializeJson(pos, s);
  wsServer->sendTXT(num, s);
}

// Protocol handling (same JSON for both)
void handleControlJsonTCP(int idx, const JsonDocument& j) {
  const char* type = j["type"] | "";
  if (strcmp(type, "mouse") == 0) {
    const char* action = j["action"] | "";
    if (strcmp(action, "move") == 0) {
      double nx = j["x"] | 0.0, ny = j["y"] | 0.0;
      handleAbsoluteMove(nx, ny);
    } else if (strcmp(action, "moveRelative") == 0) {
      int dx = 0, dy = 0;
      if (j.containsKey("dx")) dx = (int)trunc(j["dx"].as<double>());
      else if (j.containsKey("x")) dx = (int)trunc(j["x"].as<double>());
      if (j.containsKey("dy")) dy = (int)trunc(j["dy"].as<double>());
      else if (j.containsKey("y")) dy = (int)trunc(j["y"].as<double>());
      mouseEnqueueRel(dx, dy);
      lastMousePos.x = max(0, min(fbInfo.w - 1, lastMousePos.x + dx));
      lastMousePos.y = max(0, min(fbInfo.h - 1, lastMousePos.y + dy));
    } else if (strcmp(action, "down") == 0) {
      int btn = j["button"] | 0;
      if (btn == 0) mouseSetButton(MOUSE_LEFT, true);
      else if (btn == 1) mouseSetButton(MOUSE_MIDDLE, true);
      else if (btn == 2) mouseSetButton(MOUSE_RIGHT, true);
    } else if (strcmp(action, "up") == 0) {
      int btn = j["button"] | 0;
      if (btn == 0) mouseSetButton(MOUSE_LEFT, false);
      else if (btn == 1) mouseSetButton(MOUSE_MIDDLE, false);
      else if (btn == 2) mouseSetButton(MOUSE_RIGHT, false);
    } else if (strcmp(action, "wheel") == 0) {
      int dy = j["deltaY"].isNull() ? 0 : (int)trunc(j["deltaY"].as<double>());
      mouseEnqueueWheel(dy);
    }
  } else if (strcmp(type, "keyboard") == 0) {
    const char* action = j["action"] | "";
    String code = j["code"].isNull() ? String("") : String(j["code"].as<const char*>());
    String key = j["key"].isNull() ? String("") : String(j["key"].as<const char*>());
    if (strcmp(action, "down") == 0) handleKeyEvent(true, code, key);
    else if (strcmp(action, "up") == 0) handleKeyEvent(false, code, key);
  } else if (strcmp(type, "text") == 0) {
    if (j["text"].is<String>()) {
      String s = j["text"].as<String>();
      if (gLogKeys) {
        Serial.print("[text] len=");
        Serial.println(s.length());
      }
      for (unsigned int i = 0; i < s.length(); i++) {
        Keyboard.print(s.charAt(i));
        delay(2);
      }
    }
  } else if (strcmp(type, "relallkeys") == 0) {
    if (gLogKeys) Serial.println("[key] release all");
    modMask = 0;
    for (int i = 0; i < 6; i++) keys[i] = 0;
    sendFrame();
    Keyboard.releaseAll();
    for (int i = 0; i < 6; i++) prevKeys[i] = 0;
    prevModMask = 0;
  } else if (strcmp(type, "query") == 0) {
    const char* what = j["what"] | "";
    if (strcmp(what, "remoteSize") == 0) {
      if (clients[idx]) sendInfoRemoteSizeTCP(clients[idx]);
    }
  }
}
void handleControlJsonWS(uint8_t num, const JsonDocument& j) {
  const char* type = j["type"] | "";
  if (strcmp(type, "mouse") == 0) {
    const char* action = j["action"] | "";
    if (strcmp(action, "move") == 0) {
      double nx = j["x"] | 0.0, ny = j["y"] | 0.0;
      handleAbsoluteMove(nx, ny);
    } else if (strcmp(action, "moveRelative") == 0) {
      int dx = 0, dy = 0;
      if (j.containsKey("dx")) dx = (int)trunc(j["dx"].as<double>());
      else if (j.containsKey("x")) dx = (int)trunc(j["x"].as<double>());
      if (j.containsKey("dy")) dy = (int)trunc(j["dy"].as<double>());
      else if (j.containsKey("y")) dy = (int)trunc(j["y"].as<double>());
      mouseEnqueueRel(dx, dy);
      lastMousePos.x = max(0, min(fbInfo.w - 1, lastMousePos.x + dx));
      lastMousePos.y = max(0, min(fbInfo.h - 1, lastMousePos.y + dy));
    } else if (strcmp(action, "down") == 0) {
      int btn = j["button"] | 0;
      if (btn == 0) mouseSetButton(MOUSE_LEFT, true);
      else if (btn == 1) mouseSetButton(MOUSE_MIDDLE, true);
      else if (btn == 2) mouseSetButton(MOUSE_RIGHT, true);
    } else if (strcmp(action, "up") == 0) {
      int btn = j["button"] | 0;
      if (btn == 0) mouseSetButton(MOUSE_LEFT, false);
      else if (btn == 1) mouseSetButton(MOUSE_MIDDLE, false);
      else if (btn == 2) mouseSetButton(MOUSE_RIGHT, false);
    } else if (strcmp(action, "wheel") == 0) {
      int dy = j["deltaY"].isNull() ? 0 : (int)trunc(j["deltaY"].as<double>());
      mouseEnqueueWheel(dy);
    }
  } else if (strcmp(type, "keyboard") == 0) {
    const char* action = j["action"] | "";
    String code = j["code"].isNull() ? String("") : String(j["code"].as<const char*>());
    String key = j["key"].isNull() ? String("") : String(j["key"].as<const char*>());
    if (strcmp(action, "down") == 0) handleKeyEvent(true, code, key);
    else if (strcmp(action, "up") == 0) handleKeyEvent(false, code, key);
  } else if (strcmp(type, "text") == 0) {
    if (j["text"].is<String>()) {
      String s = j["text"].as<String>();
      if (gLogKeys) {
        Serial.print("[text] len=");
        Serial.println(s.length());
      }
      for (unsigned int i = 0; i < s.length(); i++) {
        Keyboard.print(s.charAt(i));
        delay(2);
      }
    }
  } else if (strcmp(type, "relallkeys") == 0) {
    if (gLogKeys) Serial.println("[key] release all");
    modMask = 0;
    for (int i = 0; i < 6; i++) keys[i] = 0;
    sendFrame();
    Keyboard.releaseAll();
    for (int i = 0; i < 6; i++) prevKeys[i] = 0;
    prevModMask = 0;
  } else if (strcmp(type, "query") == 0) {
    const char* what = j["what"] | "";
    if (strcmp(what, "remoteSize") == 0) sendInfoRemoteSizeWS(num);
  }
}

// TCP accept/read loop
void tcpServerLoop() {
  if (!tcpServer) return;
  // Accept new connections
  if (tcpServer->hasClient()) {
    WiFiClient incoming = tcpServer->available();
    if (incoming) {
      int slot = -1;
      for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients[i] || !clients[i].connected()) {
          if (clients[i]) clients[i].stop();
          slot = i;
          break;
        }
      }
      if (slot >= 0) {
        IPAddress rip = incoming.remoteIP();
        uint16_t rport = incoming.remotePort();
        Serial.print("TCP client connected in slot ");
        Serial.print(slot);
        Serial.print(" from ");
        Serial.print(rip);
        Serial.print(":");
        Serial.println(rport);
        clients[slot] = incoming;
        clients[slot].setNoDelay(true);
        sendInfoRemoteSizeTCP(clients[slot]);
      } else {
        Serial.println("No free TCP client slots; dropping connection");
        incoming.stop();
      }
    }
  }
  // Read lines
  for (int i = 0; i < MAX_CLIENTS; i++) {
    auto& c = clients[i];
    if (!c || !c.connected()) continue;
    while (c.available()) {
      char ch = c.read();
      if (ch == '\r') continue;
      if (ch == '\n') {
        String line = lineBufs[i];
        lineBufs[i] = "";
        if (line.length() == 0) continue;
        StaticJsonDocument<1024> j;
        auto err = deserializeJson(j, line);
        if (err == DeserializationError::Ok) {
          handleControlJsonTCP(i, j);
        } else {
          Serial.print("JSON parse error from TCP client ");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(err.c_str());
        }
      } else {
        lineBufs[i] += ch;
        if (lineBufs[i].length() > 4096) lineBufs[i].remove(0, lineBufs[i].length() - 1024);
      }
    }
    if (!c.connected()) {
      IPAddress rip = c.remoteIP();
      uint16_t rport = c.remotePort();
      Serial.print("TCP client disconnected slot ");
      Serial.print(i);
      Serial.print(" (");
      Serial.print(rip);
      Serial.print(":");
      Serial.print(rport);
      Serial.println(")");
      c.stop();
    }
  }
}

// WS event
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (!wsServer) return;
  switch (type) {
    case WStype_CONNECTED:
      {
        IPAddress rip = wsServer->remoteIP(num);
        Serial.print("WS client #");
        Serial.print(num);
        Serial.print(" connected from ");
        Serial.println(rip);  // port not exposed by library
        sendInfoRemoteSizeWS(num);
        break;
      }
    case WStype_TEXT:
      {
        StaticJsonDocument<1024> j;
        if (deserializeJson(j, payload, length) == DeserializationError::Ok) {
          handleControlJsonWS(num, j);
        } else {
          Serial.println("WS JSON parse error");
        }
        break;
      }
    case WStype_FRAGMENT_TEXT_START:
      if (num < WS_MAX_CLIENTS) wsFragments[num] = "";
      break;
    case WStype_FRAGMENT:
      if (num < WS_MAX_CLIENTS) wsFragments[num] += String((const char*)payload).substring(0, length);
      break;
    case WStype_FRAGMENT_FIN:
      {
        if (num < WS_MAX_CLIENTS) {
          StaticJsonDocument<2048> j;
          if (deserializeJson(j, wsFragments[num]) == DeserializationError::Ok) {
            handleControlJsonWS(num, j);
          } else {
            Serial.println("WS fragment JSON parse error");
          }
          wsFragments[num] = "";
        }
        break;
      }
    case WStype_DISCONNECTED:
      {
        IPAddress rip = wsServer->remoteIP(num);
        Serial.print("WS client #");
        Serial.print(num);
        Serial.print(" disconnected (");
        Serial.print(rip);
        Serial.println(")");
        // Optionally release inputs
        modMask = 0;
        for (int i = 0; i < 6; i++) keys[i] = 0;
        sendFrame();
        Keyboard.releaseAll();
        mouseSetButton(MOUSE_LEFT, false);
        mouseSetButton(MOUSE_MIDDLE, false);
        mouseSetButton(MOUSE_RIGHT, false);
        break;
      }
    default: break;
  }
}

// Start/stop helpers
void stopTCP() {
  if (!tcpServer) return;
  Serial.println("Stopping TCP server...");
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i]) clients[i].stop();
    lineBufs[i] = "";
  }
  // Delete to release the bound port
  delete tcpServer;
  tcpServer = nullptr;
}
void startTCP() {
  if (tcpServer) return;
  tcpServer = new WiFiServer(CONTROL_PORT);
  tcpServer->begin();
  Serial.println("TCP server listening on :1444");
}

void stopWS() {
  if (!wsServer) return;
  Serial.println("Stopping WebSocket server...");
  // WebSocketsServer doesn't expose per-server close of all clients cleanly other than delete
  delete wsServer;
  wsServer = nullptr;
}
void startWS() {
  if (wsServer) return;
  wsServer = new WebSocketsServer(CONTROL_PORT);
  wsServer->begin();
  wsServer->onEvent(onWsEvent);
  Serial.println("WebSocket server listening on :1444");
}

// Switch transport (closes current, opens new)
int switchTransport(TransportMode m) {
  if (gTransport == m) {
    Serial.print("Transport already ");
    Serial.println(m == TRANS_TCP ? "TCP" : "WebSocket");
    return 1;
  }
  Serial.print("Switching transport to ");
  Serial.println(m == TRANS_TCP ? "TCP..." : "WebSocket...");

  // Stop current
  if (gTransport == TRANS_TCP) {
    stopTCP();
  } else {
    stopWS();
  }
  delay(50);
  // Start new
  if (m == TRANS_TCP) {
    startTCP();
  } else {
    startWS();
  }
  gTransport = m;
  return 0;
}

void printIP() {
  IPAddress ip = WiFi.localIP();
  Serial.print("IP: ");
  Serial.println(ip);
}
void printUptime() {
  uint32_t ms = millis();
  uint32_t s = ms / 1000;
  uint32_t d = s / 86400;
  s %= 86400;
  uint32_t h = s / 3600;
  s %= 3600;
  uint32_t m = s / 60;
  s %= 60;
  Serial.print("Uptime: ");
  if (d) {
    Serial.print(d);
    Serial.print("d ");
  }
  if (d || h) {
    Serial.print(h);
    Serial.print("h ");
  }
  if (d || h || m) {
    Serial.print(m);
    Serial.print("m ");
  }
  Serial.print(s);
  Serial.println("s");
}

static void printLogStatus() {
  Serial.print("Logging: keys=");
  Serial.print(gLogKeys ? "on" : "off");
  Serial.print(" mouse=");
  Serial.println(gLogMouse ? "on" : "off");
}

static void setLog(const String& which, const String& val) {
  bool on = val.equalsIgnoreCase("on");
  bool off = val.equalsIgnoreCase("off");
  if (!(on || off)) {
    Serial.println("Usage: log {keys|mouse|all} {on|off}");
    return;
  }
  if (which.equalsIgnoreCase("keys") || which.equalsIgnoreCase("all")) gLogKeys = on;
  if (which.equalsIgnoreCase("mouse") || which.equalsIgnoreCase("all")) gLogMouse = on;
  printLogStatus();
}

void serialCommandLoop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      String cmdline = serialBuf;
      serialBuf = "";
      cmdline.trim();
      if (cmdline.length() == 0) continue;

      // tokenize
      String tok[4];
      int nt = 0;
      String cur = "";
      for (unsigned i = 0; i < cmdline.length(); i++) {
        char c = cmdline[i];
        if (c == ' ' || c == '\t') {
          if (cur.length()) {
            if (nt < 4) tok[nt++] = cur;
            cur = "";
          }
        } else cur += c;
      }
      if (cur.length() && nt < 4) tok[nt++] = cur;
      if (nt == 0) continue;

      if (tok[0].equalsIgnoreCase("ip")) {
        printIP();
      } else if (tok[0].equalsIgnoreCase("uptime")) {
        printUptime();
      } else if (tok[0].equalsIgnoreCase("reset")) {
        Serial.println("Rebooting to UF2 bootloader...");
        Serial.flush();
        delay(250);
        rp2040.rebootToBootloader();
      } else if (tok[0].equalsIgnoreCase("reboot")) {
        Serial.println("Rebooting MCU...");
        Serial.flush();
        delay(250);
        rp2040.reboot();
      } else if (tok[0].equalsIgnoreCase("log")) {
        if (nt == 1 || (nt == 2 && tok[1].equalsIgnoreCase("status"))) printLogStatus();
        else if (nt == 3) setLog(tok[1], tok[2]);
        else Serial.println("Usage: log {status | keys on/off | mouse on/off | all on/off}");
      } else if (tok[0].equalsIgnoreCase("net")) {
        // net status | net tcp | net ws | net restart
        if (nt == 1 || (nt == 2 && tok[1].equalsIgnoreCase("status"))) {
          Serial.print("Transport: ");
          Serial.println(gTransport == TRANS_TCP ? "TCP" : "WebSocket");
        } else if (nt >= 2 && tok[1].equalsIgnoreCase("tcp")) {
          switchTransport(TRANS_TCP);
        } else if (nt >= 2 && tok[1].equalsIgnoreCase("ws")) {
          switchTransport(TRANS_WS);
        } else if (nt >= 2 && tok[1].equalsIgnoreCase("restart")) {
          Serial.println("Restarting current transport...");
          if (gTransport == TRANS_TCP) {
            stopTCP();
            delay(30);
            startTCP();
          } else {
            stopWS();
            delay(30);
            startWS();
          }
        } else {
          Serial.println("Usage: net {status | tcp | ws | restart}");
        }
      } else if (tok[0].equalsIgnoreCase("help")) {
        Serial.println("Commands:");
        Serial.println("  ip                -  Current IP of MCU");
        Serial.println("  uptime            -  Current uptime");
        Serial.println("  reset             -  Reboot the MCU into the UF2 bootloader");
        Serial.println("  reboot            -  Reboot the MCU");
        Serial.println("  log status        -  Current logging state for mouse+keyboard");
        Serial.println("  log keys on|off   -  Set logging state for keyboard");
        Serial.println("  log mouse on|off  -  Set logging state for mouse");
        Serial.println("  log all on|off    -  Set logging state for mouse+keyboard");
        Serial.println("  net status        -  Current network transport");
        Serial.println("  net tcp           -  Switch to TCP network transport");
        Serial.println("  net ws            -  Switch to WebSocket network transport");
        Serial.println("  net restart       -  Restart network transport");
      } else {
        Serial.print("Unknown command: ");
        Serial.println(cmdline);
        Serial.println("Type 'help' for commands.");
      }
    } else {
      serialBuf += ch;
      if (serialBuf.length() > 512) serialBuf.remove(0, serialBuf.length() - 256);
    }
  }
}

// ====================== WiFi ======================
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to SSID: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    printIP();
  } else {
    Serial.println("WiFi connect timeout (continuing without IP)");
  }
}

// ====================== Setup / Loop ======================
void setup() {
  Serial.begin(2000000);
  delay(7500);
  Serial.println("\n[control-agent] booting...");

  Keyboard.begin();
  Mouse.begin();

  critical_section_init(&gMouseCS);
  multicore_launch_core1(core1MouseTask);

  wifiConnect();

  // Start default transport (TCP). Change with "net ws".
  startTCP();
  gTransport = TRANS_TCP;

  Serial.println("Ready. Type 'help' for serial console commands.");
}

void loop() {
  serialCommandLoop();

  if (gTransport == TRANS_TCP) {
    tcpServerLoop();
  } else {
    if (wsServer) wsServer->loop();
  }

  delay(1);
}