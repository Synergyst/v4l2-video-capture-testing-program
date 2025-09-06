#include <uwebsockets/App.h>
#include <nlohmann/json.hpp>

#include <sys/stat.h>
#include <limits.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

using json = nlohmann::json;

// Listen socket so we can close on shutdown
static us_listen_socket_t *gListenSocket = nullptr;

// --------------- Linux helpers ---------------
static void closesock_fd(int fd) {
    if (fd >= 0) ::close(fd);
}

static int set_keepalive(int fd, int idle_ms) {
    int optval = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval));
#ifdef TCP_KEEPIDLE
    int idle = std::max(1, idle_ms / 1000);
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
#endif
    return 0;
}

// Resolve the directory of the running executable (Linux: /proc/self/exe)
static std::string getExecutableDir() {
    char buf[PATH_MAX];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    buf[n] = '\0';
    // Strip filename to keep the directory
    char *slash = strrchr(buf, '/');
    if (!slash) return ".";
    *slash = '\0';
    return std::string(buf);
}

// Read an entire small file into a std::string (binary-safe)
static bool readFileToString(const std::string &path, std::string &out) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return false;

    struct stat st;
    if (::fstat(fd, &st) != 0) {
        ::close(fd);
        return false;
    }

    out.clear();
    if (st.st_size > 0) out.reserve(static_cast<size_t>(st.st_size));

    char buf[8192];
    for (;;) {
        ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n > 0) {
            out.append(buf, buf + n);
        } else if (n == 0) {
            break; // EOF
        } else {
            if (errno == EINTR) continue;
            ::close(fd);
            return false;
        }
    }

    ::close(fd);
    return true;
}

// Non-blocking connect with timeout
static int connect_tcp(const std::string &host, int port, int timeout_ms = 1500) {
    struct addrinfo hints; memset(&hints, 0, sizeof(hints));
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_family = AF_UNSPEC;
    hints.ai_protocol = IPPROTO_TCP;

    char portbuf[16]; snprintf(portbuf, sizeof(portbuf), "%d", port);
    struct addrinfo *res = nullptr;
    if (getaddrinfo(host.c_str(), portbuf, &hints, &res) != 0) return -1;

    int fd = -1;
    for (struct addrinfo *p = res; p; p = p->ai_next) {
        fd = (int)socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (fd < 0) continue;

        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        int rc = connect(fd, p->ai_addr, (socklen_t)p->ai_addrlen);
        if (rc == 0) { freeaddrinfo(res); return fd; }
        if (errno != EINPROGRESS) { closesock_fd(fd); fd = -1; continue; }

        // wait for connect or timeout
        fd_set wfds; FD_ZERO(&wfds); FD_SET(fd, &wfds);
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        int sel = select(fd + 1, nullptr, &wfds, nullptr, &tv);
        if (sel > 0 && FD_ISSET(fd, &wfds)) {
            int soerr = 0; socklen_t slen = sizeof(soerr);
            getsockopt(fd, SOL_SOCKET, SO_ERROR, (char*)&soerr, &slen);
            if (soerr == 0) { freeaddrinfo(res); return fd; }
        }
        closesock_fd(fd);
        fd = -1;
    }
    freeaddrinfo(res);
    return -1;
}

// --------------- Config from ENV ---------------
static std::string envStr(const char *name, const char *defv) {
    const char *p = std::getenv(name);
    return p ? std::string(p) : std::string(defv);
}
static int envInt(const char *name, int defv) {
    const char *p = std::getenv(name);
    if (!p) return defv;
    try { return std::stoi(p); } catch (...) { return defv; }
}

static const std::string TCP_HOST = envStr("TCP_HOST", "192.168.168.163");
static const int TCP_PORT = envInt("TCP_PORT", 1337);
static const std::string CONTROL_TCP_HOST = envStr("CONTROL_TCP_HOST", "192.168.168.42");
//static const std::string CONTROL_TCP_HOST = envStr("CONTROL_TCP_HOST", "192.168.168.44");
//static const std::string CONTROL_TCP_HOST = envStr("CONTROL_TCP_HOST", "192.168.168.46");
static const int CONTROL_TCP_PORT = envInt("CONTROL_TCP_PORT", 1444);
static const int HTTP_PORT = envInt("HTTP_PORT", 34878);
static const std::string AUTH_TOKEN = envStr("AUTH_TOKEN", "");
static const std::string WSS_PATH = "/ws";
static const std::string ALLOWED_CONTROL_SUBNETS_RAW = envStr("ALLOWED_CONTROL_SUBNETS", "192.168.168.0/24,127.0.0.1,75.132.2.132");
static const std::string TRUSTED_PROXIES_RAW = envStr("TRUSTED_PROXIES", "127.0.0.1,::1,192.168.168.170,192.168.168.178");
static const std::string FALLBACK_IMAGE_URL = envStr("FALLBACK_IMAGE_URL", "https://totallynotbombcodes.synergyst.club/nosignal.png");
static const std::string FALLBACK_MJPEG_URL = envStr("FALLBACK_MJPEG_URL", "");

// Backoff
static const int VIDEO_RETRY_BASE = 1000;
static const int VIDEO_RETRY_MAX = 3000;
static const int CONTROL_RETRY_BASE = 1000;
static const int CONTROL_RETRY_MAX = 3000;

// --------------- Allowed control (CIDR) helpers ---------------
static inline std::string trim(const std::string &s) {
    size_t i = 0, j = s.size();
    while (i < j && std::isspace((unsigned char)s[i])) ++i;
    while (j > i && std::isspace((unsigned char)s[j - 1])) --j;
    return s.substr(i, j - i);
}
static inline std::vector<std::string> splitCSV(const std::string &s) {
    std::vector<std::string> out;
    size_t start = 0;
    for (;;) {
        size_t pos = s.find(',', start);
        if (pos == std::string::npos) { out.push_back(trim(s.substr(start))); break; }
        out.push_back(trim(s.substr(start, pos - start)));
        start = pos + 1;
    }
    std::vector<std::string> out2;
    for (auto &x : out) if (!x.empty()) out2.push_back(x);
    return out2;
}

static inline std::string toLower(std::string s) {
    for (auto &c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}

// Detect and convert IPv4-mapped IPv6 in full-hex form:
// 0000:0000:0000:0000:0000:ffff:HHHH:HHHH -> a.b.c.d
static inline bool tryMapV4MappedIPv6Hex(const std::string &ip, std::string &outV4) {
    // quick check: must have 7 ':' separators => 8 groups
    int colons = 0; for (char c : ip) if (c == ':') ++colons;
    if (colons != 7) return false;
    std::vector<std::string> parts;
    size_t start = 0;
    for (size_t i = 0; i <= ip.size(); ++i) {
        if (i == ip.size() || ip[i] == ':') {
            parts.push_back(ip.substr(start, i - start));
            start = i + 1;
        }
    }
    if (parts.size() != 8) return false;
    // normalize case
    for (auto &p : parts) p = toLower(trim(p));
    if (!(parts[0] == "0000" && parts[1] == "0000" && parts[2] == "0000" &&
          parts[3] == "0000" && parts[4] == "0000" && parts[5] == "ffff")) {
        return false;
    }
    auto parse16 = [](const std::string &h) -> std::optional<unsigned> {
        if (h.empty() || h.size() > 4) return std::nullopt;
        unsigned v = 0;
        for (char c : h) {
            if (!std::isxdigit((unsigned char)c)) return std::nullopt;
        }
        try {
            v = std::stoul(h, nullptr, 16);
        } catch (...) { return std::nullopt; }
        if (v > 0xFFFF) return std::nullopt;
        return v;
    };
    auto g6 = parse16(parts[6]); // high 16 bits
    auto g7 = parse16(parts[7]); // low 16 bits
    if (!g6 || !g7) return false;
    unsigned v6 = *g6, v7 = *g7;
    unsigned a = (v6 >> 8) & 0xFF;
    unsigned b = (v6) & 0xFF;
    unsigned c = (v7 >> 8) & 0xFF;
    unsigned d = (v7) & 0xFF;
    char buf[32];
    snprintf(buf, sizeof(buf), "%u.%u.%u.%u", a, b, c, d);
    outV4 = buf;
    return true;
}

static inline std::string normalizeRemoteIp(std::string ip) {
    if (ip.empty()) return ip;
    ip = trim(ip);
    // XFF style "a, b, c"
    if (ip.find(',') != std::string::npos) {
        ip = trim(ip.substr(0, ip.find(',')));
    }
    // strip scope id
    size_t pct = ip.find('%');
    if (pct != std::string::npos) ip = ip.substr(0, pct);

    // Already an IPv4 dotted?
    if (ip.find('.') != std::string::npos && ip.find(':') == std::string::npos) {
        return ip;
    }

    // IPv4-mapped IPv6 textual (::ffff:a.b.c.d)
    std::string low = toLower(ip);
    const std::string mapped = "::ffff:";
    if (low.rfind(mapped, 0) == 0) {
        std::string tail = ip.substr(mapped.size());
        return tail; // could be dotted already
    }

    // IPv4-mapped IPv6 in full hex groups: 0000:0000:0000:0000:0000:ffff:XXXX:YYYY
    std::string v4;
    if (tryMapV4MappedIPv6Hex(low, v4)) {
        return v4;
    }

    // Otherwise return as-is (IPv6 literal)
    return ip;
}

static inline std::string parseXFF(const std::string &hdr) {
    if (hdr.empty()) return "";
    size_t start = 0;
    while (start < hdr.size() && std::isspace((unsigned char)hdr[start])) ++start;
    size_t pos = hdr.find(',', start);
    std::string part = (pos == std::string::npos) ? hdr.substr(start) : hdr.substr(start, pos - start);
    return trim(part);
}

static inline std::optional<uint32_t> ipv4ToInt(const std::string &ip) {
    unsigned a, b, c, d; char tail;
    int n = sscanf(ip.c_str(), "%u.%u.%u.%u%c", &a, &b, &c, &d, &tail);
    if (n < 4) return std::nullopt;
    if (a > 255 || b > 255 || c > 255 || d > 255) return std::nullopt;
    uint32_t val = ((uint32_t)a << 24) | ((uint32_t)b << 16) | ((uint32_t)c << 8) | (uint32_t)d;
    return val;
}
static inline uint32_t makeMask(int prefix) {
    if (prefix <= 0) return 0U;
    if (prefix >= 32) return 0xffffffffU;
    return 0xffffffffU << (32 - prefix);
}

struct AllowedRule {
    enum Type { CIDR, EXACT } type{EXACT};
    uint32_t base = 0, mask = 0;
    std::string exact;
    std::string raw;
};
static std::vector<AllowedRule> parseAllowedControlSubnets(const std::string &raw) {
    std::vector<AllowedRule> out;
    for (const auto &tokRaw : splitCSV(raw)) {
        auto tok = trim(tokRaw);
        if (tok == "*") {
            AllowedRule r; r.type = AllowedRule::CIDR; r.base = 0; r.mask = 0; r.raw = "*";
            out.push_back(r);
            continue;
        }
        if (tok.find('/') != std::string::npos) {
            auto slash = tok.find('/');
            auto base = trim(tok.substr(0, slash));
            auto prefStr = trim(tok.substr(slash + 1));
            try {
                int prefix = std::stoi(prefStr);
                auto baseInt = ipv4ToInt(base);
                if (!baseInt || prefix < 0 || prefix > 32) {
                    std::cerr << "Invalid allowed CIDR token: " << tok << std::endl;
                    continue;
                }
                AllowedRule r;
                r.type = AllowedRule::CIDR;
                r.mask = makeMask(prefix);
                r.base = (*baseInt) & r.mask;
                r.raw = tok;
                out.push_back(r);
            } catch (...) {
                std::cerr << "Invalid allowed CIDR token: " << tok << std::endl;
            }
        } else {
            if (auto ip4 = ipv4ToInt(tok)) {
                AllowedRule r; r.type = AllowedRule::CIDR; r.base = *ip4; r.mask = 0xffffffffU; r.raw = tok;
                out.push_back(r);
            } else {
                AllowedRule r; r.type = AllowedRule::EXACT; r.exact = tok; r.raw = tok;
                out.push_back(r);
            }
        }
    }
    return out;
}

static const std::vector<AllowedRule> ALLOWED_CONTROL_RULES = parseAllowedControlSubnets(ALLOWED_CONTROL_SUBNETS_RAW);

// Normalize trusted proxies for robust matching
static std::vector<std::string> normalizeTrusted(const std::vector<std::string> &raw) {
    std::vector<std::string> out;
    out.reserve(raw.size());
    for (auto &t : raw) {
        auto n = normalizeRemoteIp(t);
        out.push_back(n);
    }
    return out;
}
static const std::vector<std::string> TRUSTED_PROXIES = normalizeTrusted(splitCSV(TRUSTED_PROXIES_RAW));

static bool isControlIpAllowed(const std::string &rawIp) {
    auto ip = normalizeRemoteIp(rawIp);
    if (ip.empty()) return false;

    if (ip == "127.0.0.1" || ip == "::1") {
        if (ALLOWED_CONTROL_RULES.empty()) return true;
        for (auto &r : ALLOWED_CONTROL_RULES) {
            if ((r.type == AllowedRule::EXACT && (r.exact == "127.0.0.1" || r.exact == "::1")) ||
                (r.type == AllowedRule::CIDR && r.mask == 0xffffffffU && r.base == 0xffffffffU))
                return true;
        }
        return false;
    }

    if (auto ip4 = ipv4ToInt(ip)) {
        uint32_t v = *ip4;
        for (auto &r : ALLOWED_CONTROL_RULES) {
            if (r.type == AllowedRule::CIDR) {
                if ((v & r.mask) == r.base) return true;
            } else {
                if (r.exact == ip) return true;
            }
        }
        return false;
    }
    for (auto &r : ALLOWED_CONTROL_RULES) {
        if (r.type == AllowedRule::EXACT && r.exact == ip) return true;
    }
    return false;
}

// --------------- HTML template (embedded) ---------------
static const char *HTML_TEMPLATE = R"HTML(<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KVM Proxy</title>
<style>
  :root{color-scheme:dark}
  html,body{height:100%;margin:0;background:#000;overflow:hidden}
  #wrap{position:fixed;inset:0;display:flex;align-items:center;justify-content:center;background:#000}
  #player{max-width:100%;max-height:100%;display:block;user-select:none;}
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
<link rel="icon" type="image/png" href="/favicon.png?1">
</head>
<body>
  <div id="wrap">
    <img id="player" draggable="false" alt="MJPEG stream" src="${DEFAULT_FALLBACK_MJPEG_OR_IMAGE}">
  </div>
  <div id="hud">
    <span class="pill">WS: <span id="wsState" class="status">connecting...</span></span>
    <span class="pill">Video: <span id="vidState" class="status">stream lost</span></span>
    <span class="pill">Control: <span id="ctlState" class="status">agent lost</span></span>
    <button id="btnCapture">Start Capture</button>
    <button id="btnFull">Fullscreen</button>
    <input id="chkLock" type="checkbox" style="display:none">
    <label for="chkLock" title="Pointer Lock">Lock Cursor</label>
    <select id="vfMode" class="small" title="Video fallback mode">
      <option value="keep">Last frame</option>
      <option value="image">Image URL</option>
      <option value="mjpeg">Alt stream</option>
    </select>
    <input id="vfURL" type="text" placeholder="Fallback URL (image or MJPEG)" class="small vfURL" style="min-width:20px" size="2">
    <button id="vfApply" class="small" style="display:none">Apply</button>
    <label style="color:#ddd" class="small"><input id="chkAutoCapture" type="checkbox"> Auto Capture</label>
    <label style="color:#ddd" class="small"><input id="chkRememberLock" type="checkbox"> Remember Lock</label>
    <span class="pill dim" id="resInfo"></span>
    <button id="btnReboot" title="Reboot remote machine">Reboot</button>
    <button id="btnPoweroff" title="Power off remote machine">Poweroff</button>
    <button id="btnSendText" title="Send plain text as keystrokes">Send Text</button>
    <button id="btnShell" title="Execute shell command on agent">Shell</button>
    <button id="btnRelAllKeys" title="Releases all keys which the USB HID may have held down">Release keys</button>
    <button id="btnKillAgent" title="Kills and agent">Kill agent</button>
    <button id="btnHelp" class="small" style="display:none">Help</button>
  </div>
  <div id="help">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <h3 style="margin:0">Controls</h3>
      <button id="helpClose" style="background:#111;color:#f66;border:1px solid #333;border-radius:6px;padding:.2rem .4rem">Close</button>
    </div>
    <ul>
      <li>Click "Start Capture" to send mouse/keyboard</li>
      <li>Optional: enable "Lock Cursor" for raw mouse</li>
      <li>Esc releases pointer lock</li>
      <li>Use Reboot/Poweroff/Shell/Send Text via the buttons</li>
      <li>When video goes away, pick a fallback (Last frame / Image / Alt MJPEG)</li>
    </ul>
  </div>
  <div id="toasts"></div>
<script>
(function(){
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
  const btnKillAgent = document.getElementById('btnKillAgent');
  const vfModeSel = document.getElementById('vfMode');
  const vfURLInput = document.getElementById('vfURL');
  const vfApply = document.getElementById('vfApply');
  const toasts = document.getElementById('toasts');
  const helpPane = document.getElementById('help');
  const helpClose = document.getElementById('helpClose');
  const btnHelp = document.getElementById('btnHelp');

  function showToast(msg, ok=true, duration=6000) {
    const div = document.createElement('div');
    div.className = 'toast ' + (ok ? 'ok' : 'err');
    div.textContent = msg;
    div.style.opacity = '1';
    toasts.appendChild(div);
    const fadeMs = 600;
    const visibleMs = Math.max(0, duration - fadeMs);
    setTimeout(() => {
      div.style.opacity = '0';
      setTimeout(() => { try { div.remove(); } catch (e) {} }, fadeMs + 20);
    }, visibleMs);
  }

  const DEFAULT_PLACEHOLDER_DATAURL = 'data:image/svg+xml;charset=utf-8,' + encodeURIComponent(
    '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360" viewBox="0 0 640 360">' +
      '<rect width="100%" height="100%" fill="#f0f0f0"/>' +
      '<text x="50%" y="50%" fill="#ddd" font-family="system-ui,Arial" font-size="20" text-anchor="middle" dominant-baseline="middle">No video signal</text>' +
    '</svg>'
  );
  const loc = window.location;
  const wsProto = (loc.protocol === 'https:') ? 'wss:' : 'ws:';
  const wsUrl = wsProto + '//' + loc.host + '${WSS_PATH}';
  const authToken = new URLSearchParams(location.search).get('token') || null;

  let ws = null;
  let prevUrl = null;
  let lastFrameUrl = null;
  let capturing = false;
  let pointerLocked = false;
  let remoteSize = null;
  let lastClientPos = null;
  let captureWantedOnFocus = false;
  let pendingWantedCapture = false;

  let allowedControl = null;
  let controlIsConnected = null;

  const PREF_VF_MODE = 'vf.mode';
  const PREF_VF_URL = 'vf.url';
  const PREF_AUTO_CAPTURE = 'vf.autoCapture';
  const PREF_REMEMBER_LOCK = 'vf.rememberLock';
  const PREF_HELP_OPEN = 'ui.helpOpen';
  const savedHelpOpen = localStorage.getItem(PREF_HELP_OPEN);
  if (savedHelpOpen === '0') {
    helpPane.style.display = 'none';
    btnHelp.style.display = 'inline-block';
  }

  let videoConnected = false;
  let fallbackMode = localStorage.getItem(PREF_VF_MODE) || ('${DEFAULT_FALLBACK_MODE}');
  let fallbackURL = localStorage.getItem(PREF_VF_URL) || '${DEFAULT_FALLBACK_URL}';
  let autoCapturePref = localStorage.getItem(PREF_AUTO_CAPTURE) === '1';
  let rememberLockPref = localStorage.getItem(PREF_REMEMBER_LOCK) === '1';

  vfModeSel.value = fallbackMode;
  vfURLInput.value = fallbackURL;
  chkAutoCapture.checked = autoCapturePref;
  chkRememberLock.checked = rememberLockPref;

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

  function setCaptureState(on) {
    capturing = !!on;
    btnCapture.textContent = capturing ? 'Stop Capture' : 'Start Capture';
    if (!capturing && document.pointerLockElement) document.exitPointerLock();
  }
  function setPointerLock(on) {
    if (on && !document.pointerLockElement) img.requestPointerLock?.();
    else if (!on && document.pointerLockElement) document.exitPointerLock?.();
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
          } catch (e) {}
        }
      }
    };
    ws.onclose = () => {
      wsState.textContent = 'offline';
      ctlState.textContent = 'agent lost';
      videoConnected = false;
      vidState.textContent = 'stream lost';
      applyFallback();
      setTimeout(connectWs, 1000);
    };
    ws.onerror = () => {
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
          if (msg.startCapture) {
            try {
              setCaptureState(true);
              if (rememberLockPref) setPointerLock(true);
            } catch (e) {}
          }
          if (msg.note) showToast(String(msg.note), true);
          if (msg.error) showToast('Error: ' + String(msg.error), false);
        } else if (msg.type === 'shellResult') {
          const text = 'Shell [' + (msg.id ?? '?') + '] exit=' + (msg.code ?? '?') + '\n' +
            (msg.stdout ? ('STDOUT:\n' + msg.stdout) : '') +
            (msg.stderr ? ('\nSTDERR:\n' + msg.stderr) : '');
          showToast(text, (msg.code || 0) === 0);
        }
        return;
      }
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
      } catch (e) {}
    };
  }

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
  img.addEventListener('error', () => { if (!videoConnected) applyFallback(); });
  btnFull.addEventListener('click', () => {
    if (!document.fullscreenElement) document.documentElement.requestFullscreen?.();
    else document.exitFullscreen?.();
  });
  chkLock.addEventListener('change', () => {
    setPointerLock(chkLock.checked);
    if (rememberLockPref) saveRememberLockPref(true);
    setCaptureState(chkLock.checked);
  });
  chkAutoCapture.addEventListener('change', () => saveAutoCapturePref(chkAutoCapture.checked));
  chkRememberLock.addEventListener('change', () => saveRememberLockPref(chkRememberLock.checked));
  document.addEventListener('pointerlockchange', () => {
    pointerLocked = document.pointerLockElement === img;
    chkLock.checked = pointerLocked;
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
  btnKillAgent.addEventListener('click', () => { if (confirm('Kill the agent?')) send({type:'killagent', action:'control'}); });
  vfModeSel.addEventListener('change', () => { updateUrlVisibility(); checkPrefsChanged(); });
  vfURLInput.addEventListener('input', () => checkPrefsChanged());
  vfApply.addEventListener('click', () => {
    setFallback(vfModeSel.value, vfURLInput.value.trim());
    setApplyVisible(false);
  });

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

  img.addEventListener('mousemove', onMouseMove, {passive:false});
  img.addEventListener('mousedown', onMouseDown, {passive:false});
  img.addEventListener('mouseup', onMouseUp, {passive:false});
  img.addEventListener('wheel', onWheel, {passive:false});
  img.addEventListener('contextmenu', (e) => { if (capturing) e.preventDefault(); });

  connectWs();
  updateUrlVisibility();
  checkPrefsChanged();
  if (!videoConnected) applyFallback();

  function onMouseMove(ev) {
    if (!capturing) return;
    const rect = img.getBoundingClientRect();
    if (document.pointerLockElement === img) {
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
      const clientX = ev.clientX - rect.left;
      const clientY = ev.clientY - rect.top;
      const cx = rect.width > 0 ? Math.max(0, Math.min(rect.width, clientX)) : 0;
      const cy = rect.height > 0 ? Math.max(0, Math.min(rect.height, clientY)) : 0;
      const nx = rect.width > 0 ? (cx / rect.width) : 0;
      const ny = rect.height > 0 ? (cy / rect.height) : 0;
      send({ type: 'mouse', action: 'move', x: nx, y: ny });
      if (ws && ws.readyState === WebSocket.OPEN) {
        try {
          ws.send(JSON.stringify({ type: 'clientLog', mousePage: { x: ev.clientX, y: ev.clientY }, mouseImg:  { x: Math.round(cx), y: Math.round(cy) } }));
        } catch (e) {}
      }
    }
    ev.preventDefault();
  }
  function onMouseDown(ev) { if (!capturing) return; send({type:'mouse', action:'down', button: ev.button}); ev.preventDefault(); }
  function onMouseUp(ev) { if (!capturing) return; send({type:'mouse', action:'up', button: ev.button}); ev.preventDefault(); }
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
</html>)HTML";

// Utility to replace all occurrences
static std::string replaceAll(std::string s, const std::string &from, const std::string &to) {
    if (from.empty()) return s;
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
        s.replace(pos, from.size(), to);
        pos += to.size();
    }
    return s;
}
static std::string buildHtmlPage() {
    const std::string defaultFallbackURL = !FALLBACK_MJPEG_URL.empty() ? FALLBACK_MJPEG_URL :
                                           (!FALLBACK_IMAGE_URL.empty() ? FALLBACK_IMAGE_URL : "");
    const std::string defaultFallbackMode = !FALLBACK_MJPEG_URL.empty() ? "mjpeg" :
                                            (!FALLBACK_IMAGE_URL.empty() ? "image" : "keep");
    std::string page = HTML_TEMPLATE;
    page = replaceAll(page, "${WSS_PATH}", WSS_PATH);
    page = replaceAll(page, "${DEFAULT_FALLBACK_MJPEG_OR_IMAGE}", defaultFallbackURL);
    page = replaceAll(page, "${DEFAULT_FALLBACK_MODE}", defaultFallbackMode);
    page = replaceAll(page, "${DEFAULT_FALLBACK_URL}", defaultFallbackURL);
    return page;
}

// --------------- Global WS state ---------------
struct ClientData {
    std::string clientIp;
    bool allowedControl{false};
    bool authed{false};
};

static std::mutex clientsMtx;
static std::set<uWS::WebSocket<false, true, ClientData>*> clients;
static uWS::WebSocket<false, true, ClientData>* controller = nullptr;

static std::atomic<bool> videoConnected{false};
static std::atomic<bool> controlConnected{false};
static std::optional<std::pair<int,int>> lastRemoteSize; // w,h

static uWS::Loop *mainLoop = nullptr;

// --------------- Broadcasting (run on mainLoop) ---------------
static void broadcastJPEG(const std::string &frame) {
    std::lock_guard<std::mutex> lk(clientsMtx);
    for (auto *ws : clients) {
        ws->send(std::string_view(frame.data(), frame.size()), uWS::OpCode::BINARY);
    }
}
static void broadcastInfo(const json &obj) {
    json out = json::object();
    out["type"] = "info";
    for (auto it = obj.begin(); it != obj.end(); ++it) out[it.key()] = it.value();
    std::string s = out.dump();
    std::lock_guard<std::mutex> lk(clientsMtx);
    for (auto *ws : clients) {
        ws->send(s, uWS::OpCode::TEXT);
    }
}

// --------------- Control outgoing queue ---------------
static std::mutex ctrlQmtx;
static std::deque<std::string> ctrlQueue;
static std::condition_variable ctrlCv;

// --------------- Common shutdown/backoff ---------------
static std::atomic<bool> shuttingDown{false};

static int computeBackoffMs(int base, int max, int attempt) {
    int expo = base;
    for (int i = 0; i < attempt; ++i) {
        if (expo >= max) { expo = max; break; }
        if (expo > max / 2) { expo = max; break; }
        expo <<= 1;
    }
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<int> jitter(0, std::max(0, base - 1));
    int delay = std::min(max, expo) + jitter(rng);
    return delay;
}

// --------------- Video thread ---------------
static void videoThreadFunc() {
    int attempts = 0;
    while (!shuttingDown.load()) {
        std::cout << "[video] Connecting to " << TCP_HOST << ":" << TCP_PORT << "...\n";
        int fd = connect_tcp(TCP_HOST, TCP_PORT, 1500);
        if (fd < 0) {
            int delay = computeBackoffMs(VIDEO_RETRY_BASE, VIDEO_RETRY_MAX, attempts++);
            std::cout << "[video] Connect failed. Retry in ~" << delay << "ms (attempt " << attempts << ")\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            continue;
        }
        attempts = 0;
        set_keepalive(fd, VIDEO_RETRY_MAX);

        mainLoop->defer([](){
            videoConnected.store(true);
            broadcastInfo(json{{"videoConnected", true}});
            // Ask allowed clients to capture on video return
            json msg = { {"type","info"}, {"startCapture", true} };
            std::string s = msg.dump();
            std::lock_guard<std::mutex> lk(clientsMtx);
            for (auto *ws : clients) {
                if (ws->getUserData()->allowedControl) ws->send(s, uWS::OpCode::TEXT);
            }
            std::cout << "[video] Reconnected to " << TCP_HOST << ":" << TCP_PORT << "\n";
        });

        std::vector<uint8_t> acc;
        acc.reserve(1 << 20);
        bool ok = true;

        auto findMarker = [](const std::vector<uint8_t>& buf, size_t start, uint8_t a, uint8_t b) -> ssize_t {
            for (size_t i = start; i + 1 < buf.size(); ++i) {
                if (buf[i] == a && buf[i+1] == b) return (ssize_t)i;
            }
            return -1;
        };

        while (!shuttingDown.load()) {
            uint8_t buf[8192];
            int n = (int)::recv(fd, buf, sizeof(buf), 0);
            if (n == 0) { ok = false; break; }
            if (n < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                ok = false; break;
            }
            acc.insert(acc.end(), buf, buf + n);

            // parse JPEG frames by 0xFF 0xD8 ... 0xFF 0xD9
            for (;;) {
                ssize_t start = findMarker(acc, 0, 0xFF, 0xD8);
                if (start < 0) {
                    if (acc.size() > (1<<20)) {
                        std::vector<uint8_t> tail(acc.end() - 65536, acc.end());
                        acc.swap(tail);
                    }
                    break;
                }
                ssize_t end = findMarker(acc, (size_t)start + 2, 0xFF, 0xD9);
                if (end < 0) {
                    if (start > 0) {
                        std::vector<uint8_t> tail(acc.begin() + start, acc.end());
                        acc.swap(tail);
                    }
                    break;
                }
                size_t sOff = (size_t)start;
                size_t eOff = (size_t)end + 2;
                std::string frame((const char*)acc.data() + sOff, (const char*)acc.data() + eOff);
                mainLoop->defer([fr = std::move(frame)]() mutable { broadcastJPEG(fr); });
                std::vector<uint8_t> tail(acc.begin() + eOff, acc.end());
                acc.swap(tail);
            }
        }

        closesock_fd(fd);

        mainLoop->defer([](){
            videoConnected.store(false);
            broadcastInfo(json{{"videoConnected", false}});
        });

        if (!ok && !shuttingDown.load()) {
            int delay = computeBackoffMs(VIDEO_RETRY_BASE, VIDEO_RETRY_MAX, attempts++);
            std::cout << "[video] Disconnected. Retry in ~" << delay << "ms (attempt " << attempts << ")\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }
    }
}

// --------------- Control thread ---------------
static void controlThreadFunc() {
    int attempts = 0;
    for (;;) {
        if (shuttingDown.load()) return;
        std::cout << "[control] Connecting to " << CONTROL_TCP_HOST << ":" << CONTROL_TCP_PORT << "...\n";
        int fd = connect_tcp(CONTROL_TCP_HOST, CONTROL_TCP_PORT, 1500);
        if (fd < 0) {
            int delay = computeBackoffMs(CONTROL_RETRY_BASE, CONTROL_RETRY_MAX, attempts++);
            std::cout << "[control] Connect failed. Retry in ~" << delay << "ms (attempt " << attempts << ")\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            continue;
        }
        attempts = 0;
        set_keepalive(fd, CONTROL_RETRY_MAX);

        mainLoop->defer([](){
            controlConnected.store(true);
            json inf = { {"controlConnected", true} };
            if (lastRemoteSize) {
                inf["remoteSize"] = { {"w", lastRemoteSize->first}, {"h", lastRemoteSize->second} };
            }
            broadcastInfo(inf);
            std::cout << "[control] Reconnected to " << CONTROL_TCP_HOST << ":" << CONTROL_TCP_PORT << "\n";
        });

        // query remote size
        {
            const std::string q = json({{"type","query"},{"what","remoteSize"}}).dump() + "\n";
            ::send(fd, q.c_str(), q.size(), 0);
        }

        std::string readBuf;
        readBuf.reserve(8192);

        while (!shuttingDown.load()) {
            // write queue
            {
                std::unique_lock<std::mutex> lk(ctrlQmtx);
                if (ctrlQueue.empty()) {
                    ctrlCv.wait_for(lk, std::chrono::milliseconds(100));
                }
                while (!ctrlQueue.empty()) {
                    std::string line = std::move(ctrlQueue.front());
                    ctrlQueue.pop_front();
                    lk.unlock();
                    ::send(fd, line.c_str(), line.size(), 0);
                    lk.lock();
                }
            }

            // read with select
            fd_set rfds; FD_ZERO(&rfds); FD_SET(fd, &rfds);
            struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 100 * 1000;
            int rv = select(fd + 1, &rfds, nullptr, nullptr, &tv);
            if (rv < 0) break;
            if (rv == 0) continue;
            if (!FD_ISSET(fd, &rfds)) continue;

            char buf[4096];
            int n = (int)::recv(fd, buf, sizeof(buf), 0);
            if (n <= 0) break;

            readBuf.append(buf, buf + n);
            // process lines
            size_t pos = 0;
            for (;;) {
                size_t eol = readBuf.find('\n', pos);
                if (eol == std::string::npos) {
                    if (pos > 0) readBuf.erase(0, pos);
                    break;
                }
                std::string line = readBuf.substr(pos, eol - pos);
                pos = eol + 1;
                if (line.empty()) continue;
                try {
                    auto obj = json::parse(line);
                    if (obj.contains("remoteSize") && obj["remoteSize"].is_object()) {
                        auto &rs = obj["remoteSize"];
                        if (rs.contains("w") && rs.contains("h")) {
                            int w = rs["w"].get<int>();
                            int h = rs["h"].get<int>();
                            lastRemoteSize = std::make_pair(w, h);
                            mainLoop->defer([w, h]() {
                                broadcastInfo(json{{"remoteSize", {{"w", w}, {"h", h}}}});
                            });
                        }
                    } else if (obj.contains("type") && (obj["type"] == "info" || obj["type"] == "hello")) {
                        if (obj.contains("remoteSize") && obj["remoteSize"].is_object()) {
                            auto &rs = obj["remoteSize"];
                            if (rs.contains("w") && rs.contains("h")) {
                                int w = rs["w"].get<int>();
                                int h = rs["h"].get<int>();
                                lastRemoteSize = std::make_pair(w, h);
                            }
                        }
                        mainLoop->defer([o = std::move(obj)]() {
                            broadcastInfo(o);
                        });
                    }
                } catch (...) {
                    // ignore malformed
                }
            }
        }

        closesock_fd(fd);

        mainLoop->defer([](){
            controlConnected.store(false);
            broadcastInfo(json{{"controlConnected", false}});
        });

        if (shuttingDown.load()) return;
        int delay = computeBackoffMs(CONTROL_RETRY_BASE, CONTROL_RETRY_MAX, attempts++);
        std::cout << "[control] Disconnected. Retry in ~" << delay << "ms (attempt " << attempts << ")\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
    }
}

// --------------- Shutdown helpers ---------------
static void initiateShutdown() {
    shuttingDown.store(true);
    ctrlCv.notify_all();
    if (mainLoop) {
        mainLoop->defer([]() {
            if (gListenSocket) {
                us_listen_socket_close(0, gListenSocket);
                gListenSocket = nullptr;
            }
            std::lock_guard<std::mutex> lk(clientsMtx);
            for (auto *ws : clients) {
                ws->close();
            }
        });
    }
}

// --------------- Main ---------------
int main() {
    std::cout << "Allowed control subnets: ";
    for (size_t i = 0; i < ALLOWED_CONTROL_RULES.size(); ++i) {
        if (i) std::cout << ", ";
        std::cout << ALLOWED_CONTROL_RULES[i].raw;
    }
    std::cout << std::endl;

    std::signal(SIGINT, [](int){ shuttingDown.store(true); });

    auto app = uWS::App();

    // Serve HTML
    const std::string htmlPage = buildHtmlPage();
    app.get("/", [htmlPage](auto *res, auto */*req*/) {
        res->writeHeader("Content-Type", "text/html; charset=utf-8")->end(htmlPage);
    });
    app.get("/index.html", [htmlPage](auto *res, auto */*req*/) {
        res->writeHeader("Content-Type", "text/html; charset=utf-8")->end(htmlPage);
    });
    app.any("/*", [](auto *res, auto * /*req*/) {
        res->writeStatus("404 Not Found")->end("Not found");
    });
    app.get("/favicon.png", [](auto *res, auto */*req*/) {
        std::string ico;
        // Prefer the directory where the binary resides; fallback to CWD
        const std::string exeDir = getExecutableDir();
        const std::string pathA = exeDir + "/favicon.png";
        const std::string pathB = "favicon.png";
        if (!readFileToString(pathA, ico) && !readFileToString(pathB, ico)) {
            res->writeStatus("404 Not Found")->end("favicon.png not found");
            return;
        }
        res->writeHeader("Content-Type", "image/png")->writeHeader("Cache-Control", "public, max-age=86400")->end(std::move(ico));
    });

    // WebSocket behavior
    app.ws<ClientData>(WSS_PATH, {
        .upgrade = [](auto *res, auto *req, auto *context) {
            // Peer IP (reverse proxy handling)
            std::string peerTxt;
            {
                auto sv = res->getRemoteAddressAsText();
                peerTxt.assign(sv.data(), sv.size());
            }
            std::string peerNorm = normalizeRemoteIp(peerTxt);

            // If peer is trusted proxy, use x-forwarded-for / x-real-ip (first hop)
            auto xff = req->getHeader("x-forwarded-for");
            auto xri = req->getHeader("x-real-ip");

            bool trusted = false;
            for (auto &p : TRUSTED_PROXIES) {
                if (peerNorm == p) { trusted = true; break; }
            }

            std::string clientIp = peerNorm;
            if (trusted) {
                std::string h = xff.length() ? std::string(xff) : std::string(xri);
                if (!h.empty()) {
                    auto parsed = parseXFF(h);
                    if (!parsed.empty()) clientIp = normalizeRemoteIp(parsed);
                }
            }

            bool allowed = isControlIpAllowed(clientIp);

            ClientData init;
            init.clientIp = clientIp;
            init.allowedControl = allowed;
            init.authed = AUTH_TOKEN.empty();

            std::cout << "WS upgrade: peer=" << peerTxt
                      << " norm=" << peerNorm
                      << " clientIp=" << clientIp
                      << " trusted=" << (trusted ? "1":"0")
                      << " allowed=" << (allowed ? "1":"0")
                      << std::endl;

            res->template upgrade<ClientData>(
                std::move(init),
                req->getHeader("sec-websocket-key"),
                req->getHeader("sec-websocket-protocol"),
                req->getHeader("sec-websocket-extensions"),
                context
            );
        },
        .open = [](auto *ws) {
            {
                std::lock_guard<std::mutex> lk(clientsMtx);
                clients.insert(ws);
            }
            auto *ud = ws->getUserData();
            std::cout << "Browser connected. Clients: " << clients.size()
                      << " IP: " << ud->clientIp
                      << " controlAllowed: " << (ud->allowedControl ? "true":"false")
                      << std::endl;

            json info = {
                {"controlConnected", controlConnected.load()},
                {"videoConnected", videoConnected.load()},
                {"allowedControl", ud->allowedControl}
            };
            if (lastRemoteSize) {
                info["remoteSize"] = { {"w", lastRemoteSize->first}, {"h", lastRemoteSize->second} };
            }
            if (AUTH_TOKEN.empty()) {
                info["startCapture"] = true;
            }
            json out = { {"type","info"} };
            for (auto it = info.begin(); it != info.end(); ++it) out[it.key()] = it.value();
            ws->send(out.dump(), uWS::OpCode::TEXT);

            if (AUTH_TOKEN.empty() && controller == nullptr && ud->allowedControl) {
                controller = ws;
            }
        },
        .message = [](auto *ws, std::string_view msg, uWS::OpCode opCode) {
            if (opCode == uWS::OpCode::BINARY) return;
            json obj;
            try { obj = json::parse(msg); } catch (...) { return; }

            auto *ud = ws->getUserData();

            if (obj.contains("type") && obj["type"] == "clientLog") {
                return;
            }

            if (obj.contains("type") && obj["type"] == "hello") {
                std::string token = obj.value("token", "");
                if (!AUTH_TOKEN.empty() && token != AUTH_TOKEN) {
                    json r = { {"type","info"}, {"error","auth_failed"} };
                    ws->send(r.dump(), uWS::OpCode::TEXT);
                    ws->close();
                    return;
                }
                ud->authed = true;
                if (!controller && ud->allowedControl) controller = ws;
                return;
            }

            if (!ud->allowedControl || (!AUTH_TOKEN.empty() && !ud->authed)) return;

            if (!controller) controller = ws;
            if (ws != controller) return;

            std::string line(msg);
            line.push_back('\n');
            {
                std::lock_guard<std::mutex> lk(ctrlQmtx);
                ctrlQueue.push_back(std::move(line));
            }
            ctrlCv.notify_one();
        },
        .close = [](auto *ws, int /*code*/, std::string_view /*msg*/) {
            {
                std::lock_guard<std::mutex> lk(clientsMtx);
                clients.erase(ws);
            }
            if (ws == controller) controller = nullptr;
            auto *ud = ws->getUserData();
            std::cout << "Browser disconnected. Clients: " << clients.size()
                      << " IP: " << ud->clientIp << std::endl;
        }
    });

    app.listen("127.0.0.1", HTTP_PORT, [](auto *token) {
        if (token) {
            gListenSocket = token;
            std::cout << "HTTP server listening on http://localhost:" << HTTP_PORT << "/\n";
        } else {
            std::cerr << "Failed to listen on port " << HTTP_PORT << "\n";
            std::exit(1);
        }
    });

    mainLoop = uWS::Loop::get();

    // Threads
    std::thread videoThread(videoThreadFunc);
    std::thread controlThread(controlThreadFunc);

    // Shutdown watcher
    std::thread shutdownWatcher([]{
        while (!shuttingDown.load()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        initiateShutdown();
    });

    // Run app (blocks until listen socket closed)
    app.run();

    shuttingDown.store(true);
    ctrlCv.notify_all();

    if (shutdownWatcher.joinable()) shutdownWatcher.join();
    if (videoThread.joinable()) videoThread.join();
    if (controlThread.joinable()) controlThread.join();
    return 0;
}
