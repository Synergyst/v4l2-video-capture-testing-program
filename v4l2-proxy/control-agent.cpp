// control-agent.cpp
// Linux-only control agent: plain TCP JSON line server + MCU serial bridge.
// Compatible with your controlThreadFunc client (connect_tcp, newline-delimited JSON).
// Env:
//   CONTROL_TCP_PORT (default 1444)
//   SERIAL_PORT (default /dev/ttyACM0)
//   SERIAL_BAUD (default 2000000)
//   ENABLE_SYSTEM_CMDS (1 enables reboot/poweroff exec)

#include <nlohmann/json.hpp>

#include <atomic>
#include <cerrno>
#include <csignal>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <vector>
#include <cmath>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#ifdef BOTHER
#include <asm/termbits.h>
#endif

using json = nlohmann::json;

// -------------------------------
// Env helpers
// -------------------------------
static std::string getenvOr(const char* key, const std::string& def) {
    const char* v = std::getenv(key);
    return v ? std::string(v) : def;
}
static int getenvOrInt(const char* key, int def) {
    const char* v = std::getenv(key);
    if (!v || !*v) return def;
    char* end = nullptr;
    long val = std::strtol(v, &end, 10);
    if (!end || *end) return def;
    return (int)val;
}
static bool getenvFlag(const char* key) {
    const char* v = std::getenv(key);
    return v && std::string(v) == "1";
}

// -------------------------------
// File read
// -------------------------------
static std::string readAllTrim(const std::string& path) {
    int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return {};
    std::string out;
    char buf[1024];
    for (;;) {
        ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n > 0) out.append(buf, buf + n);
        else break;
    }
    ::close(fd);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
    return out;
}

// -------------------------------
// Framebuffer info + mouse state
// -------------------------------
struct FbInfo { int w = 1920; int h = 1080; int bpp = 32; } fbInfo;
struct MousePos { int x = fbInfo.w / 2; int y = fbInfo.h / 2; } lastMousePos;
static std::mutex mouseMtx;

static void readFbInfo() {
    try {
        std::string vs = readAllTrim("/sys/class/graphics/fb0/virtual_size");
        std::string bppStr = readAllTrim("/sys/class/graphics/fb0/bits_per_pixel");
        if (!vs.empty()) {
            size_t comma = vs.find(',');
            if (comma != std::string::npos) {
                int w = std::stoi(vs.substr(0, comma));
                int h = std::stoi(vs.substr(comma + 1));
                if (w > 0 && h > 0) { fbInfo.w = w; fbInfo.h = h; }
            }
        }
        if (!bppStr.empty()) {
            int bpp = std::stoi(bppStr);
            if (bpp > 0) fbInfo.bpp = bpp;
        }
        std::lock_guard<std::mutex> lk(mouseMtx);
        lastMousePos.x = fbInfo.w / 2;
        lastMousePos.y = fbInfo.h / 2;
    } catch (...) {
        // keep defaults
    }
}

// -------------------------------
// Serial manager
// -------------------------------
class SerialManager {
public:
    SerialManager(std::string path, int baud) : path_(std::move(path)), baud_(baud) {}
    ~SerialManager() { stop(); }

    void start() {
        running_.store(true);
        readerThread_ = std::thread([this]{ readerLoop(); });
        writerThread_ = std::thread([this]{ writerLoop(); });
    }
    void stop() {
        running_.store(false);
        {
            std::lock_guard<std::mutex> lk(outMtx_);
            outCv_.notify_all();
        }
        if (readerThread_.joinable()) readerThread_.join();
        if (writerThread_.joinable()) writerThread_.join();
        closeFd();
    }

    void sendLine(std::string s) {
        if (s.empty() || s.back() != '\n') s.push_back('\n');
        std::lock_guard<std::mutex> lk(outMtx_);
        outQ_.push_back(std::move(s));
        outCv_.notify_one();
    }

    bool ready() const { return ready_.load(); }

private:
    std::string path_;
    int baud_;
    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::atomic<bool> ready_{false};
    std::thread readerThread_, writerThread_;

    std::mutex outMtx_;
    std::condition_variable outCv_;
    std::deque<std::string> outQ_;

    void closeFd() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        ready_.store(false);
    }

    static bool set_baud(int fd, int baud) {
#ifdef B2000000
        // Try known constants first
        speed_t spd = 0;
        switch (baud) {
            case 9600: spd = B9600; break;
            case 19200: spd = B19200; break;
            case 38400: spd = B38400; break;
            case 57600: spd = B57600; break;
            case 115200: spd = B115200; break;
#ifdef B230400
            case 230400: spd = B230400; break;
#endif
#ifdef B460800
            case 460800: spd = B460800; break;
#endif
#ifdef B921600
            case 921600: spd = B921600; break;
#endif
#ifdef B1000000
            case 1000000: spd = B1000000; break;
#endif
#ifdef B1500000
            case 1500000: spd = B1500000; break;
#endif
#ifdef B2000000
            case 2000000: spd = B2000000; break;
#endif
            default: spd = 0; break;
        }
        if (spd != 0) {
            termios tty{};
            if (tcgetattr(fd, &tty) != 0) return false;
            cfmakeraw(&tty);
            cfsetispeed(&tty, spd);
            cfsetospeed(&tty, spd);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cc[VMIN] = 1;
            tty.c_cc[VTIME] = 0;
            return tcsetattr(fd, TCSANOW, &tty) == 0;
        }
#endif
#ifdef BOTHER
        // Arbitrary baud via termios2
        struct termios2 tio{};
        if (ioctl(fd, TCGETS2, &tio) != 0) return false;
        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER | CLOCAL | CREAD;
        tio.c_ispeed = baud;
        tio.c_ospeed = baud;
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_iflag = 0;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cc[VMIN] = 1;
        tio.c_cc[VTIME] = 0;
        return ioctl(fd, TCSETS2, &tio) == 0;
#else
        (void)baud;
        return false;
#endif
    }

    bool openSerial() {
        int fd = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) return false;
        if (!set_baud(fd, baud_)) {
            std::cerr << "Failed to set baud " << baud_ << ", fallback 115200\n";
            set_baud(fd, 115200);
        }
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        fd_ = fd;
        ready_.store(true);
        std::cout << "Serial open: " << path_ << " @ " << baud_ << "\n";
        return true;
    }

    void readerLoop() {
        std::string lineBuf;
        while (running_.load()) {
            if (fd_ < 0) {
                std::cout << "Opening serial " << path_ << " @ " << baud_ << "\n";
                if (!openSerial()) {
                    ready_.store(false);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    continue;
                }
            }
            struct pollfd pfd{fd_, POLLIN, 0};
            int pr = ::poll(&pfd, 1, 500);
            if (pr > 0 && (pfd.revents & POLLIN)) {
                char buf[512];
                ssize_t n = ::read(fd_, buf, sizeof(buf));
                if (n > 0) {
                    for (ssize_t i = 0; i < n; ++i) {
                        char c = buf[i];
                        if (c == '\n') {
                            if (!lineBuf.empty() && lineBuf.back() == '\r') lineBuf.pop_back();
                            std::cout << "[MCU] " << lineBuf << "\n";
                            lineBuf.clear();
                        } else lineBuf.push_back(c);
                    }
                } else if (n == 0) {
                    std::cerr << "Serial EOF, reopening...\n";
                    closeFd();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        std::cerr << "Serial read error: " << std::strerror(errno) << ", reopening...\n";
                        closeFd();
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                }
            } else if (pr < 0) {
                std::cerr << "Serial poll error: " << std::strerror(errno) << ", reopening...\n";
                closeFd();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    }

    void writerLoop() {
        while (running_.load()) {
            std::string msg;
            {
                std::unique_lock<std::mutex> lk(outMtx_);
                outCv_.wait_for(lk, std::chrono::milliseconds(100), [&]{ return !outQ_.empty() || !running_.load(); });
                if (!running_.load()) break;
                if (outQ_.empty() || fd_ < 0 || !ready_.load()) continue;
                msg = std::move(outQ_.front());
                outQ_.pop_front();
            }
            const char* p = msg.data();
            size_t left = msg.size();
            while (left > 0) {
                ssize_t n = ::write(fd_, p, left);
                if (n > 0) {
                    p += n; left -= (size_t)n;
                } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                } else {
                    std::cerr << "Serial write failed, requeue & reopen: " << std::strerror(errno) << "\n";
                    {
                        std::lock_guard<std::mutex> lk(outMtx_);
                        outQ_.push_front(std::string(p, p + left));
                    }
                    closeFd();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    break;
                }
            }
        }
    }
};

// -------------------------------
// Command execution with timeout
// -------------------------------
struct CmdResult { int code = 0; std::string out; std::string err; };

static CmdResult runCommandWithTimeout(const std::string& cmd, int timeoutMs) {
    CmdResult r;
    int outP[2], errP[2];
    if (pipe(outP) != 0 || pipe(errP) != 0) { r.code = 1; r.err = "pipe failed"; return r; }
    pid_t pid = fork();
    if (pid < 0) { r.code = 1; r.err = "fork failed"; close(outP[0]); close(outP[1]); close(errP[0]); close(errP[1]); return r; }
    if (pid == 0) {
        setpgid(0, 0);
        dup2(outP[1], STDOUT_FILENO);
        dup2(errP[1], STDERR_FILENO);
        close(outP[0]); close(outP[1]); close(errP[0]); close(errP[1]);
        execl("/bin/sh", "sh", "-c", cmd.c_str(), (char*)nullptr);
        _exit(127);
    }
    close(outP[1]); close(errP[1]);
    fcntl(outP[0], F_SETFL, fcntl(outP[0], F_GETFL, 0) | O_NONBLOCK);
    fcntl(errP[0], F_SETFL, fcntl(errP[0], F_GETFL, 0) | O_NONBLOCK);
    auto start = std::chrono::steady_clock::now();
    int status = 0;
    for (;;) {
        char buf[1024];
        ssize_t n;
        while ((n = ::read(outP[0], buf, sizeof(buf))) > 0) r.out.append(buf, n);
        while ((n = ::read(errP[0], buf, sizeof(buf))) > 0) r.err.append(buf, n);
        pid_t w = waitpid(pid, &status, WNOHANG);
        if (w == pid) break;
        int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeoutMs) {
            kill(-pid, SIGTERM);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            kill(-pid, SIGKILL);
            waitpid(pid, &status, 0);
            r.code = 124; // timeout
            close(outP[0]); close(errP[0]);
            return r;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // drain
    char buf2[1024]; ssize_t n2;
    while ((n2 = ::read(outP[0], buf2, sizeof(buf2))) > 0) r.out.append(buf2, n2);
    while ((n2 = ::read(errP[0], buf2, sizeof(buf2))) > 0) r.err.append(buf2, n2);
    close(outP[0]); close(errP[0]);
    if (WIFEXITED(status)) r.code = WEXITSTATUS(status);
    else if (WIFSIGNALED(status)) r.code = 128 + WTERMSIG(status);
    else r.code = 1;
    return r;
}

// -------------------------------
// Helpers for MCU command args
// -------------------------------
static std::string escapeArg(const std::string& in) {
    if (in.empty() || in.find(' ') != std::string::npos) {
        std::string s = in;
        for (auto& c : s) if (c == '"') c = '\'';
        return std::string("\"") + s + "\"";
    }
    return in;
}
static std::string quoteForMCU(const std::string& in) {
    std::string s = in;
    for (auto& c : s) if (c == '"') c = '\'';
    return std::string("\"") + s + "\"";
}

// -------------------------------
// Mouse absolute -> relative
// -------------------------------
static inline double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

static void handleAbsoluteMove(double nx, double ny, SerialManager& serial) {
    nx = clamp01(nx); ny = clamp01(ny);
    int targetX = (int)std::llround(nx * (fbInfo.w - 1));
    int targetY = (int)std::llround(ny * (fbInfo.h - 1));
    int dx, dy;
    {
        std::lock_guard<std::mutex> lk(mouseMtx);
        dx = targetX - lastMousePos.x;
        dy = targetY - lastMousePos.y;
        if (dx == 0 && dy == 0) return;
        lastMousePos.x = std::max(0, std::min(fbInfo.w - 1, lastMousePos.x + dx));
        lastMousePos.y = std::max(0, std::min(fbInfo.h - 1, lastMousePos.y + dy));
    }
    serial.sendLine("MREL " + std::to_string(dx) + " " + std::to_string(dy));
}

// -------------------------------
// TCP server
// -------------------------------
static std::atomic<bool> shuttingDown{false};
static int g_listen_fd = -1;
static std::mutex g_clients_mtx;
static std::set<int> g_clients; // fds for logging/cleanup

static void setKeepAlive(int fd) {
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
    int nodelay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
}

static void clientHandler(int fd, SerialManager* serial) {
    {
        std::lock_guard<std::mutex> lk(g_clients_mtx);
        g_clients.insert(fd);
    }
    sockaddr_storage ss{}; socklen_t slen = sizeof(ss);
    getpeername(fd, (sockaddr*)&ss, &slen);
    char addrbuf[128] = {0};
    if (ss.ss_family == AF_INET) {
        auto* a = (sockaddr_in*)&ss;
        inet_ntop(AF_INET, &a->sin_addr, addrbuf, sizeof(addrbuf));
        std::cout << "Controller connected: " << addrbuf << ":" << ntohs(a->sin_port) << "\n";
    } else if (ss.ss_family == AF_INET6) {
        auto* a = (sockaddr_in6*)&ss;
        inet_ntop(AF_INET6, &a->sin6_addr, addrbuf, sizeof(addrbuf));
        std::cout << "Controller connected: [" << addrbuf << "]:" << ntohs(a->sin6_port) << "\n";
    } else {
        std::cout << "Controller connected\n";
    }

    // On connect: send MSCRSZ to MCU and info to client; send SETPOS for predictability.
    serial->sendLine("MSCRSZ " + std::to_string(fbInfo.w) + " " + std::to_string(fbInfo.h));
    {
        json info = { {"type","info"}, {"remoteSize", {{"w", fbInfo.w}, {"h", fbInfo.h}}} };
        std::string s = info.dump();
        s.push_back('\n');
        ::send(fd, s.c_str(), s.size(), 0);
    }
    {
        std::lock_guard<std::mutex> lk(mouseMtx);
        serial->sendLine("SETPOS " + std::to_string(lastMousePos.x) + " " + std::to_string(lastMousePos.y));
    }

    std::string buf;
    buf.reserve(8192);
    char tmp[4096];

    for (;;) {
        // Read
        int n = (int)::recv(fd, tmp, sizeof(tmp), 0);
        if (n == 0) break;
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
            break;
        }
        buf.append(tmp, tmp + n);

        // Process lines
        size_t start = 0;
        for (;;) {
            size_t eol = buf.find('\n', start);
            if (eol == std::string::npos) {
                if (start > 0) buf.erase(0, start);
                break;
            }
            std::string line = buf.substr(start, eol - start);
            start = eol + 1;
            if (line.empty()) continue;

            json j;
            try { j = json::parse(line); } catch (...) {
                std::cerr << "Bad JSON: " << line << "\n";
                continue;
            }

            std::string type = j.value("type", "");
            if (type == "mouse") {
                std::string action = j.value("action", "");
                if (action == "move") {
                    double nx = j.contains("x") && j["x"].is_number() ? j["x"].get<double>() : 0.0;
                    double ny = j.contains("y") && j["y"].is_number() ? j["y"].get<double>() : 0.0;
                    handleAbsoluteMove(nx, ny, *serial);
                } else if (action == "moveRelative") {
                    int dx = 0, dy = 0;
                    if (j.contains("dx") && j["dx"].is_number()) dx = (int)std::trunc(j["dx"].get<double>());
                    else if (j.contains("x") && j["x"].is_number()) dx = (int)std::trunc(j["x"].get<double>());
                    if (j.contains("dy") && j["dy"].is_number()) dy = (int)std::trunc(j["dy"].get<double>());
                    else if (j.contains("y") && j["y"].is_number()) dy = (int)std::trunc(j["y"].get<double>());
                    serial->sendLine("MREL " + std::to_string(dx) + " " + std::to_string(dy));
                    {
                        std::lock_guard<std::mutex> lk(mouseMtx);
                        lastMousePos.x = std::max(0, std::min(fbInfo.w - 1, lastMousePos.x + dx));
                        lastMousePos.y = std::max(0, std::min(fbInfo.h - 1, lastMousePos.y + dy));
                    }
                } else if (action == "down") {
                    int btn = j.contains("button") && j["button"].is_number() ? j["button"].get<int>() : 0;
                    serial->sendLine("MDOWN " + std::to_string(btn));
                } else if (action == "up") {
                    int btn = j.contains("button") && j["button"].is_number() ? j["button"].get<int>() : 0;
                    serial->sendLine("MUP " + std::to_string(btn));
                } else if (action == "wheel") {
                    int dx = j.contains("deltaX") && j["deltaX"].is_number() ? (int)std::trunc(j["deltaX"].get<double>()) : 0;
                    int dy = j.contains("deltaY") && j["deltaY"].is_number() ? (int)std::trunc(j["deltaY"].get<double>()) : 0;
                    serial->sendLine("MWHEEL " + std::to_string(dx) + " " + std::to_string(dy));
                }
            } else if (type == "keyboard") {
                std::string action = j.value("action", "");
                std::string code = j.value("code", "");
                std::string key = j.value("key", "");
                int ctrl = j.value("ctrl", false) ? 1 : 0;
                int shift = j.value("shift", false) ? 1 : 0;
                int alt = j.value("alt", false) ? 1 : 0;
                int meta = j.value("meta", false) ? 1 : 0;
                if (action == "down") {
                    std::string cmd = "KDOWN " + escapeArg(code) + " " + escapeArg(key) + " " +
                                      std::to_string(ctrl) + " " + std::to_string(shift) + " " +
                                      std::to_string(alt) + " " + std::to_string(meta);
                    serial->sendLine(cmd);
                } else if (action == "up") {
                    std::string cmd = "KUP " + escapeArg(code) + " " + escapeArg(key) + " " +
                                      std::to_string(ctrl) + " " + std::to_string(shift) + " " +
                                      std::to_string(alt) + " " + std::to_string(meta);
                    serial->sendLine(cmd);
                }
            } else if (type == "text") {
                if (j.contains("text") && j["text"].is_string()) {
                    std::string s = j["text"].get<std::string>();
                    serial->sendLine("KTEXT " + quoteForMCU(s));
                }
            } else if (type == "relallkeys") {
                serial->sendLine("RELALLKEYS");
            } else if (type == "system") {
                std::string action = j.value("action", "");
                if (action == "reboot" || action == "poweroff") {
                    if (!getenvFlag("ENABLE_SYSTEM_CMDS")) {
                        json resp = { {"type","info"}, {"error","system commands disabled"} };
                        std::string s = resp.dump(); s.push_back('\n');
                        ::send(fd, s.c_str(), s.size(), 0);
                    } else {
                        std::string cmd = (action == "reboot") ? "reboot || sudo reboot"
                                                               : "systemctl reboot || sudo systemctl reboot";
                        std::thread([fd, cmd](){
                            CmdResult cr = runCommandWithTimeout(cmd, 30000);
                            json resp = { {"type","shellResult"}, {"id", "reboot"},
                                          {"code", cr.code}, {"stdout", cr.out}, {"stderr", cr.err} };
                            std::string s = resp.dump(); s.push_back('\n');
                            ::send(fd, s.c_str(), s.size(), 0);
                        }).detach();
                    }
                }
            } else if (type == "shell") {
                if (j.contains("cmd") && j["cmd"].is_string()) {
                    std::string cmd = j["cmd"].get<std::string>();
                    std::string id = j.contains("id") && j["id"].is_string() ? j["id"].get<std::string>() : "cmd";
                    std::thread([fd, id, cmd](){
                        CmdResult cr = runCommandWithTimeout(cmd, 30000);
                        json resp = { {"type","shellResult"}, {"id", id},
                                      {"code", cr.code}, {"stdout", cr.out}, {"stderr", cr.err} };
                        std::string s = resp.dump(); s.push_back('\n');
                        ::send(fd, s.c_str(), s.size(), 0);
                    }).detach();
                }
            } else if (type == "query" && j.value("what", "") == "remoteSize") {
                std::cout << "Sent remoteSize\n";
                serial->sendLine("MSCRSZ " + std::to_string(fbInfo.w) + " " + std::to_string(fbInfo.h));
                {
                    json info = { {"type","info"}, {"remoteSize", {{"w", fbInfo.w}, {"h", fbInfo.h}}} };
                    std::string s = info.dump(); s.push_back('\n');
                    ::send(fd, s.c_str(), s.size(), 0);
                }
                {
                    std::lock_guard<std::mutex> lk(mouseMtx);
                    serial->sendLine("SETPOS " + std::to_string(lastMousePos.x) + " " + std::to_string(lastMousePos.y));
                }
            } else {
                // Unknown -> RAW to MCU
                serial->sendLine("RAW " + quoteForMCU(j.dump()));
            }
        }
    }

    std::cout << "Controller disconnected\n";
    ::close(fd);
    {
        std::lock_guard<std::mutex> lk(g_clients_mtx);
        g_clients.erase(fd);
    }
}

static int runServer(uint16_t port, SerialManager* serial) {
    const bool dualStack = getenvFlag("CONTROL_DUAL_STACK");

    if (!dualStack) {
        // IPv4-only
        int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            std::cerr << "socket(AF_INET) failed: " << std::strerror(errno) << "\n";
            return -1;
        }
        int reuse = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{}; addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);
        if (bind(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
            std::cerr << "bind(IPv4) failed: " << std::strerror(errno) << "\n";
            ::close(fd); return -1;
        }
        if (listen(fd, 16) != 0) {
            std::cerr << "listen() failed: " << std::strerror(errno) << "\n";
            ::close(fd); return -1;
        }
        g_listen_fd = fd;
        std::cout << "Control sink listening on " << port << " (IPv4-only)\n";

        while (!shuttingDown.load()) {
            sockaddr_storage ss{}; socklen_t slen = sizeof(ss);
            int cfd = ::accept(fd, (sockaddr*)&ss, &slen);
            if (cfd < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) { std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue; }
                if (shuttingDown.load()) break;
                std::cerr << "accept() error: " << std::strerror(errno) << "\n";
                continue;
            }
            setKeepAlive(cfd);
            std::thread(clientHandler, cfd, serial).detach();
        }
        if (g_listen_fd >= 0) { ::close(g_listen_fd); g_listen_fd = -1; }
        return 0;
    } else {
        // Dual-stack (IPv6 with IPv4-mapped)
        int fd = ::socket(AF_INET6, SOCK_STREAM, 0);
        if (fd < 0) {
            std::cerr << "socket(AF_INET6) failed: " << std::strerror(errno) << "\n";
            return -1;
        }
        int v6only = 0; setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only));
        int reuse = 1; setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in6 addr{}; addr.sin6_family = AF_INET6;
        addr.sin6_addr = in6addr_any;
        addr.sin6_port = htons(port);
        if (bind(fd, (sockaddr*)&addr, sizeof(addr)) != 0) {
            std::cerr << "bind(IPv6 dual-stack) failed: " << std::strerror(errno) << "\n";
            ::close(fd); return -1;
        }
        if (listen(fd, 16) != 0) {
            std::cerr << "listen() failed: " << std::strerror(errno) << "\n";
            ::close(fd); return -1;
        }
        g_listen_fd = fd;
        std::cout << "Control sink listening on " << port << " (dual-stack)\n";

        while (!shuttingDown.load()) {
            sockaddr_storage ss{}; socklen_t slen = sizeof(ss);
            int cfd = ::accept(fd, (sockaddr*)&ss, &slen);
            if (cfd < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) { std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue; }
                if (shuttingDown.load()) break;
                std::cerr << "accept() error: " << std::strerror(errno) << "\n";
                continue;
            }
            setKeepAlive(cfd);
            std::thread(clientHandler, cfd, serial).detach();
        }
        if (g_listen_fd >= 0) { ::close(g_listen_fd); g_listen_fd = -1; }
        return 0;
    }
}

// -------------------------------
// main
// -------------------------------
int main() {
    const int CONTROL_PORT = getenvOrInt("CONTROL_TCP_PORT", 1444);
    const std::string SERIAL_PORT = getenvOr("SERIAL_PORT", "/dev/ttyACM0");
    const int SERIAL_BAUD = getenvOrInt("SERIAL_BAUD", 2000000);

    std::signal(SIGINT, [](int){ shuttingDown.store(true); if (g_listen_fd >= 0) ::shutdown(g_listen_fd, SHUT_RD); });

    readFbInfo();

    SerialManager serial(SERIAL_PORT, SERIAL_BAUD);
    serial.start();

    // Start server (blocking accept loop)
    runServer((uint16_t)CONTROL_PORT, &serial);

    shuttingDown.store(true);
    serial.stop();
    // Close any client fds left
    {
        std::lock_guard<std::mutex> lk(g_clients_mtx);
        for (int fd : g_clients) ::close(fd);
        g_clients.clear();
    }
    return 0;
}
