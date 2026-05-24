// controller.cpp -- read the per-camera px/mm scale from the motion controller (TCP).
// Windows-only (Winsock); not in the off-target tests. Fetched once in a detached
// thread (never blocks the render path) and cached behind a mutex. The endpoint is
// discovered from our process's own live connection, falling back to the default.

#include "controller.h"

#include "capture.h"

#define NOMINMAX
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601  // Win7+: inet_pton / ws2tcpip
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

namespace vis {
namespace {

constexpr char kCtrlIp[] = "192.168.0.8";  // default controller IP (fallback)
constexpr unsigned short kCtrlPort = 701;  // controller port
constexpr std::uint32_t kKeyUpX = 20, kKeyUpY = 21, kKeyDownX = 34, kKeyDownY = 35;

std::mutex g_mtx;
CamScale g_up, g_down;
std::once_flag g_once;
bool g_wsaUp = false;

// Fallback endpoint when discovery finds nothing.
sockaddr_in default_endpoint() {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(kCtrlPort);
    inet_pton(AF_INET, kCtrlIp, &a.sin_addr);
    return a;
}

// Find this process's own established connection on the controller port and reuse its
// remote address, so we follow whatever endpoint is actually in use. false = none.
bool discover_endpoint(sockaddr_in& out) {
    const DWORD pid = GetCurrentProcessId();
    DWORD size = 0;
    if (GetExtendedTcpTable(nullptr, &size, FALSE, AF_INET, TCP_TABLE_OWNER_PID_ALL, 0) !=
        ERROR_INSUFFICIENT_BUFFER)
        return false;
    std::vector<char> buf(size);
    if (GetExtendedTcpTable(buf.data(), &size, FALSE, AF_INET, TCP_TABLE_OWNER_PID_ALL, 0) != NO_ERROR)
        return false;
    const auto* tbl = reinterpret_cast<const MIB_TCPTABLE_OWNER_PID*>(buf.data());
    for (DWORD i = 0; i < tbl->dwNumEntries; ++i) {
        const MIB_TCPROW_OWNER_PID& r = tbl->table[i];
        if (r.dwOwningPid != pid || r.dwState != MIB_TCP_STATE_ESTAB) continue;
        if (ntohs(static_cast<u_short>(r.dwRemotePort)) != kCtrlPort) continue;
        out = sockaddr_in{};
        out.sin_family = AF_INET;
        out.sin_addr.s_addr = r.dwRemoteAddr;  // already network byte order
        out.sin_port = static_cast<u_short>(r.dwRemotePort);
        return true;
    }
    return false;
}

// Connect with a bounded timeout (non-blocking connect + select), then switch the
// socket back to blocking with short send/recv timeouts. INVALID_SOCKET on failure.
SOCKET connect_timeout(const sockaddr_in& addr, int ms) {
    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) return INVALID_SOCKET;
    u_long nb = 1;
    ioctlsocket(s, static_cast<long>(FIONBIO), &nb);
    connect(s, reinterpret_cast<const sockaddr*>(&addr), sizeof addr);
    fd_set wf;
    FD_ZERO(&wf);
    FD_SET(s, &wf);
    timeval tv{ms / 1000, (ms % 1000) * 1000};
    if (select(0, nullptr, &wf, nullptr, &tv) <= 0) {
        closesocket(s);
        return INVALID_SOCKET;
    }
    nb = 0;
    ioctlsocket(s, static_cast<long>(FIONBIO), &nb);
    DWORD to = 500;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&to), sizeof to);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&to), sizeof to);
    BOOL nodelay = TRUE;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nodelay), sizeof nodelay);
    return s;
}

// Discard any bytes already waiting before a request.
void drain(SOCKET s) {
    u_long avail = 0;
    char buf[256];
    for (int i = 0; i < 8; ++i) {
        if (ioctlsocket(s, FIONREAD, &avail) != 0 || avail == 0) return;
        if (recv(s, buf, static_cast<int>(sizeof buf), 0) <= 0) return;
    }
}

// A short greeting arrives a few ms after connect; block one recv to wait for and
// discard it before sending requests, then sweep any remainder.
void consume_banner(SOCKET s) {
    char buf[256];
    (void)recv(s, buf, static_cast<int>(sizeof buf), 0);  // blocks up to SO_RCVTIMEO
    drain(s);
}

// Read one param: [1,0,0,0, key(LE32), 0,0,0,0] -> value at response bytes[8..11].
bool read_param(SOCKET s, std::uint32_t key, std::int32_t& out) {
    drain(s);
    unsigned char req[12] = {1, 0, 0, 0,
                             static_cast<unsigned char>(key),
                             static_cast<unsigned char>(key >> 8),
                             static_cast<unsigned char>(key >> 16),
                             static_cast<unsigned char>(key >> 24),
                             0, 0, 0, 0};
    if (send(s, reinterpret_cast<const char*>(req), 12, 0) != 12) return false;
    unsigned char resp[64];
    int got = 0;
    while (got < 12) {
        int n = recv(s, reinterpret_cast<char*>(resp) + got, static_cast<int>(sizeof resp) - got, 0);
        if (n <= 0) return false;
        got += n;
    }
    if (resp[0] != 1) return false;  // command echo
    out = static_cast<std::int32_t>(static_cast<std::uint32_t>(resp[8]) |
                                    (static_cast<std::uint32_t>(resp[9]) << 8) |
                                    (static_cast<std::uint32_t>(resp[10]) << 16) |
                                    (static_cast<std::uint32_t>(resp[11]) << 24));
    return true;
}

void fetch() {
    if (!g_wsaUp) {
        WSADATA w{};
        if (WSAStartup(MAKEWORD(2, 2), &w) != 0) return;
        g_wsaUp = true;
    }
    for (int attempt = 0; attempt < 10; ++attempt) {
        sockaddr_in ep{};
        const bool discovered = discover_endpoint(ep);  // host's live target...
        if (!discovered) ep = default_endpoint();       // ...else the factory default
        SOCKET s = connect_timeout(ep, 500);
        const bool connected = (s != INVALID_SOCKET);
        std::int32_t ux = 0, uy = 0, dx = 0, dy = 0;
        bool ok = false;
        if (connected) {
            consume_banner(s);
            ok = read_param(s, kKeyUpX, ux) && read_param(s, kKeyUpY, uy) &&
                 read_param(s, kKeyDownX, dx) && read_param(s, kKeyDownY, dy);
            closesocket(s);
        }
        if (cap::armed()) {  // diagnostic: confirm discovery/endpoint/values when armed
            char ips[INET_ADDRSTRLEN] = {};
            inet_ntop(AF_INET, &ep.sin_addr, ips, sizeof ips);
            char line[160];
            std::snprintf(line, sizeof line,
                          "controller: discovered=%d ep=%s:%d connect=%d read=%d down=(%d,%d) up=(%d,%d)",
                          discovered ? 1 : 0, ips, ntohs(ep.sin_port), connected ? 1 : 0, ok ? 1 : 0,
                          dx, dy, ux, uy);
            cap::log_line(line);
        }
        if (ok) {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_up = {ux / 10000.0, uy / 10000.0, ux > 0 && uy > 0};
            g_down = {dx / 10000.0, dy / 10000.0, dx > 0 && dy > 0};
            return;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void ensure() {
    std::call_once(g_once, [] { std::thread(fetch).detach(); });
}

}  // namespace

CamScale down_cam_scale() {
    ensure();
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_down;
}

CamScale up_cam_scale() {
    ensure();
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_up;
}

}  // namespace vis
