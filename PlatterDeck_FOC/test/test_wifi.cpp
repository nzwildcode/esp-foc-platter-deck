#include "../src/wifi_link.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
using namespace platter;
int main() {
  const uint8_t golden[40] = {
    0x50,0x44,0x57,0x32,0x01,0,0,0,0x78,0x56,0x34,0x12,0x2a,0,0,0,
    0xd2,0x04,0,0,0x63,0,0,0,0xaa,0,0,0,0,0,0,0,0,0,0,0x82,0x03,0xba,0x4b,0x96
  };
  Packet p;
  assert(p.decode(golden, 40));
  assert(p.session == 0x12345678 && p.seq == 42 && p.timeUs == 1234 && p.boot == 99);
  uint8_t encoded[40]; p.encode(encoded); assert(!memcmp(encoded, golden, 40));
  for (size_t i = 0; i < 40; ++i) {
    memcpy(encoded, golden, 40); encoded[i] ^= 1;
    Packet bad; assert(!bad.decode(encoded, 40)); assert(!bad.decode(golden, i));
  }
  assert(crc32((const uint8_t*)"123456789", 9) == 0xcbf43926);
  CommandGate g;
  p.session = 1; p.seq = 1; p.boot = 0; p.timeUs = 0;
  p.payload[2] = SPIN; assert(!g.accept(p, 1000, 99, true)); // boot disabled
  p.payload[2] = FREE; assert(g.accept(p, 1000, 99, false)); // first handshake
  p.payload[2] = SPIN; p.seq = 2;
  assert(!g.accept(p, 2000, 99, true)); // no fresh board echo
  p.boot = 99; p.timeUs = 1500;
  assert(!g.accept(p, 2000, 99, false)); // second peer
  p.session = 2; assert(!g.accept(p, 2000, 99, true)); // second session
  p.session = 1; assert(g.accept(p, 2000, 99, true));
  assert(!g.accept(p, 2100, 99, true)); // duplicate
  p.seq = 1; assert(!g.accept(p, 2100, 99, true)); // out of order
  p.seq = 3; p.boot = 100; assert(!g.accept(p, 2100, 99, true)); // wrong board boot
  p.boot = 99; p.timeUs = 0xffffffff; assert(!g.accept(p, TIMEOUT_US + 1000, 99, true)); // stale echo, 1 us past the limit
  p.timeUs = TIMEOUT_US + 2001; assert(!g.live(TIMEOUT_US + 2001)); // last accepted at 2000
  assert(!g.accept(p, TIMEOUT_US + 2001, 99, true)); // timeout forbids resuming SPIN
  p.payload[2] = FREE; assert(g.accept(p, TIMEOUT_US + 2001, 99, true));
  p.seq++; p.payload[2] = SPIN; assert(g.accept(p, TIMEOUT_US + 3000, 99, true));
  CommandGate wrap;
  p.payload[2] = FREE; p.boot = 0; p.seq = 0xffffffff;
  assert(wrap.accept(p, 0xfffffff0, 99, false));
  p.seq = 0; p.boot = 99; p.timeUs = 0xfffffff0; p.payload[2] = SPIN;
  assert(wrap.accept(p, 10, 99, true)); // sequence and timer wrap
  VelocityEstimator velocity;
  float measured = 0;
  for (uint32_t i = 0; i < 500; ++i) {
    // 33 1/3 RPM, encoder quantisation and alternating one-count noise.
    uint32_t t = i * 2000 + (i % 3) * 200;
    int32_t a = int32_t(lround(double(t) * (65536.0 / 1.8e6) / 4.0)) * 4 + (i % 2 ? 4 : -4);
    measured = velocity.update(a, t);
    if (i > 10) assert(fabs(measured - 3.490659) < 0.12);
  }
  for (uint32_t i = 0; i < 12; ++i) measured = velocity.update(37000, 1000000 + i * 2000);
  assert(fabs(measured) < 0.0001);
  for (uint32_t i = 0; i < 12; ++i) measured = velocity.update(37000 - i * 74, 1024000 + i * 2000);
  assert(measured < -3.4 && measured > -3.6);
  puts("Wi-Fi codec, command gate and telemetry velocity: all checks passed");
}
