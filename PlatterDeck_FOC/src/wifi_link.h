// PlatterDeck Wi-Fi protocol v2. See crates/proto/src/wifi.rs.
// Portable: no Arduino dependencies, also compiled by the host protocol tests.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace platter {
constexpr size_t PACKET_LEN = 40;
constexpr uint16_t PORT = 19704;
// Command silence after which the bridge is disabled and the platter coasts.
// Long enough to ride out a Windows Wi-Fi background scan; a dead PC still
// never leaves the platter driven for more than half a second. The PC uses
// the same value for telemetry loss.
constexpr uint32_t TIMEOUT_US = 500000;
constexpr uint8_t COMMAND = 1, TELEMETRY = 2;
enum Mode : uint8_t { FREE = 0, SPIN = 1, BRAKE = 2, ESTOP = 3 };
enum Status : uint8_t { ENABLED = 1, SENSOR_OK = 2, FAULT = 4, LINK_TIMEOUT = 8 };
inline uint32_t read32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline void write32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i));
}
inline uint8_t crc8(const uint8_t* p, size_t n) {
  uint8_t crc = 0;
  while (n--) { crc ^= *p++; for (int i = 0; i < 8; ++i) crc = (crc & 128) ? (crc << 1) ^ 7 : crc << 1; }
  return crc;
}
inline uint32_t crc32(const uint8_t* p, size_t n) {
  uint32_t crc = ~0u;
  while (n--) { crc ^= *p++; for (int i = 0; i < 8; ++i) crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0); }
  return ~crc;
}
inline bool newer(uint32_t a, uint32_t b) { return a != b && uint32_t(a - b) < 0x80000000u; }
struct Packet {
  uint8_t kind = COMMAND;
  uint32_t session = 0, seq = 0, timeUs = 0, boot = 0;
  uint8_t payload[12] = {};
  bool decode(const uint8_t* p, size_t n) {
    if (n != PACKET_LEN || memcmp(p, "PDW2", 4) || p[5] || p[6] || p[7] ||
        (p[4] != COMMAND && p[4] != TELEMETRY) || crc32(p, 36) != read32(p + 36) ||
        p[24] != 0xaa || crc8(p + 24, 11) != p[35]) return false;
    kind = p[4]; session = read32(p + 8); seq = read32(p + 12);
    timeUs = read32(p + 16); boot = read32(p + 20); memcpy(payload, p + 24, 12);
    return session != 0 && (kind != COMMAND || (payload[2] <= ESTOP && !payload[9] && !payload[10]));
  }
  void encode(uint8_t* p) const {
    memset(p, 0, PACKET_LEN); memcpy(p, "PDW2", 4); p[4] = kind;
    write32(p + 8, session); write32(p + 12, seq); write32(p + 16, timeUs); write32(p + 20, boot);
    memcpy(p + 24, payload, 12); write32(p + 36, crc32(p, 36));
  }
};
// Network task owns the gate. The motor task independently enforces the deadline.
struct CommandGate {
  uint32_t session = 0, lastSeq = 0, lastRx = 0;
  bool active = false;
  bool live(uint32_t now) const { return active && uint32_t(now - lastRx) <= TIMEOUT_US; }
  bool accept(const Packet& p, uint32_t now, uint32_t boot, bool samePeer) {
    if (p.kind != COMMAND || !p.session || p.payload[2] > ESTOP) return false;
    const bool free = p.payload[2] == FREE;
    const bool fresh = p.boot == boot && uint32_t(now - p.timeUs) <= TIMEOUT_US;
    if (live(now)) {
      if (!samePeer || p.session != session || !newer(p.seq, lastSeq)) return false;
      if (!fresh && !(free && p.boot == 0)) return false;
    } else {
      if (!free || (p.boot != 0 && !fresh)) return false;
      if (active && p.session == session && !newer(p.seq, lastSeq)) return false;
    }
    active = true; session = p.session; lastSeq = p.seq; lastRx = now;
    return true;
  }
};
struct Command { Packet packet; uint32_t receivedUs = 0; };
struct Sample { int32_t angle = 0, velocity = 0; uint8_t status = 0; uint32_t timeUs = 0, seq = 0; };
// Independent 12 ms angle regression; leave SimpleFOC's velocity state alone.
struct VelocityEstimator {
  int32_t angle[8] = {}; uint32_t time[8] = {};
  unsigned head = 0, count = 0;
  float update(int32_t a, uint32_t t) {
    angle[head] = a; time[head] = t; head = (head + 1) % 8;
    if (count < 8) ++count;
    double sx = 0, sy = 0, sxx = 0, sxy = 0; unsigned n = 0;
    uint32_t span = 0;
    for (unsigned i = 0; i < count; ++i) {
      const uint32_t age = t - time[i]; if (age > 12000) continue;
      const double x = -double(age) * 1e-6;
      const double y = double(int32_t(uint32_t(angle[i]) - uint32_t(a))) * (6.283185307179586 / 65536.0);
      sx += x; sy += y; sxx += x*x; sxy += x*y; ++n;
      if (age > span) span = age;
    }
    const double denom = n*sxx - sx*sx;
    return span >= 4000 && denom > 1e-12 ? float((n*sxy - sx*sy) / denom) : 0;
  }
};
}
