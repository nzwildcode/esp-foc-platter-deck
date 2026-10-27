#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Update.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <SimpleFOC.h>
#include "config.h"
#include "platter_sensor.h"
#include <WiFiUdp.h>
#include "wifi_link.h"
#include "web_assets.h"

// FreeRTOS ownership model:
// - Core 1 motor task exclusively owns SimpleFOC and the SPI sensor.
// - Core 0 network task owns Wi-Fi, OTA and WebSocket publishing.
// - Web callbacks send POD commands through a FreeRTOS queue.
// - Telemetry crosses cores through a mutex-protected snapshot.

enum UiMode : uint8_t {
  MODE_VEL_OPEN = 0,
  MODE_ANG_OPEN = 1,
  MODE_VELOCITY = 2,
  MODE_ANGLE = 3,
  MODE_TORQUE = 4,
};

struct StoredConfig {
  int polePairs = DEFAULT_POLE_PAIRS;
  float supply = DEFAULT_SUPPLY_VOLTAGE;
  float voltageLimit = DEFAULT_VOLTAGE_LIMIT;
  float velocityLimit = DEFAULT_VELOCITY_LIMIT;
  float currentLimit = DEFAULT_CURRENT_LIMIT;
  float velP = DEFAULT_VEL_P;
  float velI = DEFAULT_VEL_I;
  float velD = DEFAULT_VEL_D;
  float velTf = DEFAULT_VEL_TF;
  float angP = DEFAULT_ANG_P;
  bool calibrated = false;
  float zeroElectricAngle = 0.0f;
  int8_t sensorDirection = 1;
  float csGainA = 0.0f;
  float csGainB = 0.0f;
};

StoredConfig cfg;
Preferences preferences;
UiMode selectedMode = MODE_VEL_OPEN;

// Checked, pipelined AS5048A reads and a least-squares velocity estimate;
// see platter_sensor.h for why SimpleFOC's MagneticSensorSPI is not used.
PlatterSensor sensor(SPI, ENC_CS, ENC_SPI_HZ);
BLDCMotor motor(DEFAULT_POLE_PAIRS);
BLDCDriver3PWM driver(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE);

#if CURRENT_SENSE_ENABLED
InlineCurrentSense currentSense(CS_SHUNT_OHM, CS_GAIN,
                                CS_PHASE_A, CS_PHASE_B, _NC);
#endif

bool sensorOK = false;
bool driverOK = false;
bool currentSenseOK = false;
bool focReady = false;
uint16_t sensorMagnitude = 0;
uint32_t sensorErrors = 0;

AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws("/ws");
enum CmdKind : uint8_t {
  CK_TARGET,
  CK_MODE,
  CK_ENABLE,
  CK_ESTOP,
  CK_PID,
  CK_LIMITS,
  CK_SUPPLY,
  CK_POLE_PAIRS,
  CK_CALIBRATE,
  CK_SAVE,
  CK_RESTART,
  CK_PROBE,
  CK_SET_HARMONIC,
  CK_CLEAR_HARMONICS,
};

struct Cmd {
  CmdKind kind;
  float f[5];
};

QueueHandle_t cmdQueue = nullptr;
SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t networkWake = nullptr;
SemaphoreHandle_t networkReady = nullptr;
volatile bool stateRequested = false;
QueueHandle_t deckCommands = nullptr;
QueueHandle_t deckSamples = nullptr;
SemaphoreHandle_t updateStopped = nullptr;
WiFiUDP deckUdp;
platter::CommandGate deckGate;
IPAddress deckPeer;
uint16_t deckPeerPort = 0;
uint32_t bootId = 0;
// These state variables belong exclusively to the motor task.
uint32_t deckLastRx = 0;
bool deckSeen = false, deckTimedOut = false, deckEstop = false;
platter::Mode deckMode = platter::FREE;
platter::VelocityEstimator deckVelocity;
float deckVelocityRad = 0;
// Bench identification: up to three injected q-voltage harmonics of shaft
// angle (amp * sin(k * angle + phase)) added on top of the compensation.
struct ProbeHarmonic { float k = 0, amp = 0, phase = 0; };
ProbeHarmonic probe[3];
bool probeActive = false;
// Identified compensation: q-voltage harmonics of shaft angle applied in
// closed-loop velocity mode, found by tools/harmonic_compensate.py and kept
// in NVS as (k, amp, phase). They are summed into a 1024-bin table of volts
// per shaft angle, rebuilt a few bins per control loop after any change, so
// the per-loop cost is one interpolated lookup however many there are.
constexpr int COMP_SLOTS = 48;
constexpr unsigned COMP_BINS = 1024;
ProbeHarmonic comp[COMP_SLOTS];
bool compDirty = false;
float compTable[COMP_BINS];
float compPeakV = 0.0f;              // max |compTable| after the last rebuild
unsigned compBuildBin = COMP_BINS;   // < COMP_BINS while a rebuild is running
uint32_t ffClipCount = 0;                        // loops where PI output + feed-forward hit the limit

void compRebuild() { compBuildBin = 0; }

// Four bins per call (about 130 us with all slots busy); the table stays in
// use while it is rebuilt, which mixes old and new volts for the ~50 ms a
// rebuild takes. Called from the control loop whether or not the motor runs.
void compBuildStep() {
  if (compBuildBin >= COMP_BINS) return;
  constexpr unsigned PER_CALL = 4;
  const float dtheta = 6.28318530718f / COMP_BINS;
  for (unsigned n = 0; n < PER_CALL && compBuildBin < COMP_BINS; ++n, ++compBuildBin) {
    const float theta = compBuildBin * dtheta;
    float v = 0.0f;
    for (const auto& h : comp)
      if (h.k > 0 && h.amp > 0) v += h.amp * sinf(h.k * theta + h.phase);
    compTable[compBuildBin] = v;
  }
  if (compBuildBin >= COMP_BINS) {
    float peak = 0.0f;
    for (float v : compTable) peak = fmaxf(peak, fabsf(v));
    compPeakV = peak;
  }
}

// Table volts at a shaft angle (rad, any range), linearly interpolated.
float compLookup(float angle) {
  float f = angle * (float(COMP_BINS) / 6.28318530718f);
  f -= floorf(f / COMP_BINS) * COMP_BINS;
  unsigned b0 = unsigned(f);
  if (b0 >= COMP_BINS) b0 = COMP_BINS - 1;
  const float frac = f - b0;
  const unsigned b1 = (b0 + 1) % COMP_BINS;
  return compTable[b0] + frac * (compTable[b1] - compTable[b0]);
}

struct RuntimeSnapshot {
  StoredConfig config;
  UiMode mode = MODE_VEL_OPEN;
  bool driver = false;
  bool sensor = false;
  bool current = false;
  bool calibrated = false;
  bool enabled = false;
  float angle = 0.0f;
  float velocity = 0.0f;
  float target = 0.0f;
  float uq = 0.0f;
  float currentAmps = 0.0f;
  uint16_t magnitude = 0;
  uint32_t sensorErrorCount = 0;
  uint32_t rate = 0;
  bool deckConnected = false;
};

RuntimeSnapshot runtimeSnapshot;

uint32_t loopCounter = 0;
uint32_t loopRate = 0;
unsigned long lastTelemetry = 0;
unsigned long lastStateSent = 0;
unsigned long lastSecond = 0;
unsigned long lastSensorProbe = 0;

volatile bool httpUpdateEStopRequested = false;
volatile bool httpUpdateInProgress = false;
volatile bool httpUpdateReady = false;
volatile bool httpUpdateRestartRequested = false;
volatile unsigned long httpUpdateRestartAt = 0;
bool httpUpdateWriteFailed = false;

float clampFinite(float value, float low, float high) {
  if (isnan(value) || isinf(value)) return low;
  return value < low ? low : (value > high ? high : value);
}

float finiteOrZero(float value) {
  return (isnan(value) || isinf(value)) ? 0.0f : value;
}

bool isClosedMode(UiMode mode) {
  return mode == MODE_VELOCITY || mode == MODE_ANGLE || mode == MODE_TORQUE;
}

bool motorIsEnabled() {
#if MOTOR_STAGE_ENABLED
  return driverOK && motor.enabled;
#else
  return false;
#endif
}

float targetLimit() {
  switch (selectedMode) {
    case MODE_VEL_OPEN:
    case MODE_VELOCITY:
      return cfg.velocityLimit;
    case MODE_ANG_OPEN:
    case MODE_ANGLE:
      return MAX_ANGLE_TARGET;
    case MODE_TORQUE:
      return currentSenseOK ? cfg.currentLimit : cfg.voltageLimit;
  }
  return 0.0f;
}

void sendMessage(bool ok, const String& text) {
  JsonDocument doc;
  doc["type"] = "msg";
  doc["ok"] = ok;
  doc["text"] = text;
  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}

// AS5048A register access for connection/magnet diagnostics. The angle
// itself is read by PlatterSensor with checked, pipelined frames.
bool readSensorRegister(uint16_t address, uint16_t& value) {
  return sensor.readRegister(address, value);
}

bool probeSensor() {
  uint16_t magnitude = 0;
  bool valid = readSensorRegister(ENC_MAG_REG, magnitude);
  if (!valid) {
    // Reading ERRFL clears a latched AS5048A communication error. Retry once
    // so one disturbed frame does not leave the dashboard permanently offline.
    uint16_t ignored = 0;
    readSensorRegister(ENC_ERROR_REG, ignored);
    valid = readSensorRegister(ENC_MAG_REG, magnitude);
  }
  if (!valid || magnitude == 0 || magnitude == 0x3FFF) {
    sensorErrors++;
    return false;
  }
  sensorMagnitude = magnitude;
  return true;
}

void saveConfig() {
  if (!preferences.begin(NVS_NAMESPACE, false)) {
    sendMessage(false, "Could not open NVS configuration");
    return;
  }
  preferences.putBool("valid", true);
  preferences.putInt("pp", cfg.polePairs);
  preferences.putFloat("supply", cfg.supply);
  preferences.putFloat("limV", cfg.voltageLimit);
  preferences.putFloat("limVel", cfg.velocityLimit);
  preferences.putFloat("limC", cfg.currentLimit);
  preferences.putFloat("velP", cfg.velP);
  preferences.putFloat("velI", cfg.velI);
  preferences.putFloat("velD", cfg.velD);
  preferences.putFloat("velTf", cfg.velTf);
  preferences.putFloat("angP", cfg.angP);
  preferences.putBool("cal", cfg.calibrated);
  preferences.putFloat("zea", cfg.zeroElectricAngle);
  preferences.putChar("dir", cfg.sensorDirection);
  preferences.putFloat("csA", cfg.csGainA);
  preferences.putFloat("csB", cfg.csGainB);
  preferences.putBytes("harm", comp, sizeof comp);
  compDirty = false;
  preferences.end();
  sendMessage(true, "Configuration saved to NVS");
}

// Persist changed harmonics once the motor is released; never while the
// FOC loop is driving, since NVS writes stall the flash cache.
void saveHarmonicsIfDirty() {
  if (!compDirty || !preferences.begin(NVS_NAMESPACE, false)) return;
  preferences.putBytes("harm", comp, sizeof comp);
  preferences.end();
  compDirty = false;
}

void loadConfig() {
  if (!preferences.begin(NVS_NAMESPACE, true)) {
    return;
  }
  if (!preferences.getBool("valid", false)) {
    preferences.end();
    return;
  }
  cfg.polePairs = constrain(preferences.getInt("pp", DEFAULT_POLE_PAIRS),
                            MIN_POLE_PAIRS, MAX_POLE_PAIRS);
  cfg.supply = clampFinite(preferences.getFloat("supply", DEFAULT_SUPPLY_VOLTAGE),
                           MIN_SUPPLY_VOLTAGE, MAX_SUPPLY_VOLTAGE);
  cfg.voltageLimit = clampFinite(preferences.getFloat("limV", DEFAULT_VOLTAGE_LIMIT),
                                 0.0f, min(cfg.supply, MAX_VOLTAGE_LIMIT));
  cfg.velocityLimit = clampFinite(preferences.getFloat("limVel", DEFAULT_VELOCITY_LIMIT),
                                  0.0f, MAX_VELOCITY_LIMIT);
  cfg.currentLimit = clampFinite(preferences.getFloat("limC", DEFAULT_CURRENT_LIMIT),
                                 0.0f, MAX_CURRENT_LIMIT);
  cfg.velP = clampFinite(preferences.getFloat("velP", DEFAULT_VEL_P), 0.0f, MAX_PID_P);
  cfg.velI = clampFinite(preferences.getFloat("velI", DEFAULT_VEL_I), 0.0f, MAX_PID_I);
  cfg.velD = clampFinite(preferences.getFloat("velD", DEFAULT_VEL_D), 0.0f, MAX_PID_D);
  cfg.velTf = clampFinite(preferences.getFloat("velTf", DEFAULT_VEL_TF), 0.0f, MAX_PID_TF);
  cfg.angP = clampFinite(preferences.getFloat("angP", DEFAULT_ANG_P), 0.0f, MAX_ANG_P);
  cfg.calibrated = preferences.getBool("cal", false);
  cfg.zeroElectricAngle = preferences.getFloat("zea", 0.0f);
  cfg.sensorDirection = preferences.getChar("dir", 1) < 0 ? -1 : 1;
  cfg.csGainA = preferences.getFloat("csA", 0.0f);
  cfg.csGainB = preferences.getFloat("csB", 0.0f);
  if (preferences.getBytesLength("harm") == sizeof comp) {
    preferences.getBytes("harm", comp, sizeof comp);
    for (auto& h : comp) {
      h.k = clampFinite(h.k, 0.0f, 512.0f);
      h.amp = clampFinite(h.amp, 0.0f, 0.5f);
      h.phase = clampFinite(h.phase, -7.0f, 7.0f);
    }
  }
  compRebuild();
  preferences.end();
}

void applyTuning() {
#if MOTOR_STAGE_ENABLED
  motor.PID_velocity.P = cfg.velP;
  motor.PID_velocity.I = cfg.velI;
  motor.PID_velocity.D = cfg.velD;
  motor.LPF_velocity.Tf = cfg.velTf;
  motor.P_angle.P = cfg.angP;
  motor.voltage_limit = cfg.voltageLimit;
  motor.PID_velocity.limit = cfg.voltageLimit;
  motor.velocity_limit = cfg.velocityLimit;
  motor.current_limit = cfg.currentLimit;
  motor.voltage_sensor_align = min(ALIGN_VOLTAGE_MAX, cfg.voltageLimit);
#endif
}

void applyMode(UiMode mode) {
  selectedMode = mode;
#if MOTOR_STAGE_ENABLED
  switch (mode) {
    case MODE_VEL_OPEN: motor.controller = MotionControlType::velocity_openloop; break;
    case MODE_ANG_OPEN: motor.controller = MotionControlType::angle_openloop; break;
    case MODE_VELOCITY: motor.controller = MotionControlType::velocity; break;
    case MODE_ANGLE: motor.controller = MotionControlType::angle; break;
    case MODE_TORQUE: motor.controller = MotionControlType::torque; break;
  }
  motor.PID_velocity.reset();
  motor.P_angle.reset();
  if (mode == MODE_ANG_OPEN || mode == MODE_ANGLE)
    motor.target = sensorOK ? sensor.getAngle() : 0.0f;
  else
    motor.target = 0.0f;
#endif
}

void updateRuntimeSnapshot() {
  RuntimeSnapshot next;
  next.config = cfg;
  next.mode = selectedMode;
  next.driver = driverOK;
  next.sensor = sensorOK;
  next.current = currentSenseOK;
  next.calibrated = focReady;
  next.enabled = motorIsEnabled();
  next.angle = sensorOK ? finiteOrZero(sensor.getAngle()) : 0.0f;
  next.velocity = sensorOK ? deckVelocityRad : 0.0f;
  next.target = finiteOrZero(motor.target);
  next.uq = driverOK ? finiteOrZero(motor.voltage.q) : 0.0f;
#if CURRENT_SENSE_ENABLED
  next.currentAmps = currentSenseOK
      ? finiteOrZero(currentSense.getDCCurrent(motor.electrical_angle)) : 0.0f;
#endif
  next.magnitude = sensorMagnitude;
  next.sensorErrorCount = sensorErrors + sensor.frameErrors() + sensor.rejectedJumps();
  next.rate = loopRate;
  next.deckConnected = deckSeen && !deckTimedOut;

  if (stateMutex && xSemaphoreTake(stateMutex, 0) == pdTRUE) {
    runtimeSnapshot = next;
    xSemaphoreGive(stateMutex);
  }
}

RuntimeSnapshot copyRuntimeSnapshot() {
  RuntimeSnapshot copy;
  if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    copy = runtimeSnapshot;
    xSemaphoreGive(stateMutex);
  }
  return copy;
}

String buildStateJson() {
  const RuntimeSnapshot state = copyRuntimeSnapshot();
  JsonDocument doc;
  doc["type"] = "state";
  doc["deckConnected"] = state.deckConnected;
  doc["motorCompiled"] = (bool)MOTOR_STAGE_ENABLED;
  doc["currentCompiled"] = (bool)CURRENT_SENSE_ENABLED;
  doc["driver"] = state.driver;
  doc["sensor"] = state.sensor;
  doc["current"] = state.current;
  doc["calibrated"] = state.calibrated;
  doc["enabled"] = state.enabled;
  doc["mode"] = (int)state.mode;
  doc["target"] = state.target;
  doc["polePairs"] = state.config.polePairs;
  doc["supply"] = state.config.supply;
  doc["voltageLimit"] = state.config.voltageLimit;
  doc["velocityLimit"] = state.config.velocityLimit;
  doc["currentLimit"] = state.config.currentLimit;
  doc["velP"] = state.config.velP;
  doc["velI"] = state.config.velI;
  doc["velD"] = state.config.velD;
  doc["velTf"] = state.config.velTf;
  doc["angP"] = state.config.angP;
  doc["magnitude"] = state.magnitude;
  doc["sensorErrors"] = state.sensorErrorCount;
  doc["rate"] = state.rate;
  doc["probeActive"] = probeActive;
  doc["compPeakV"] = compPeakV;
  doc["ffClips"] = ffClipCount;
  JsonArray harmonics = doc["harmonics"].to<JsonArray>();
  for (const auto& h : comp) {
    if (h.k <= 0 || h.amp <= 0) continue;
    JsonArray item = harmonics.add<JsonArray>();
    item.add(h.k); item.add(h.amp); item.add(h.phase);
  }
  doc["heap"] = (uint32_t)ESP.getFreeHeap();
  doc["ip"] = WiFi.softAPIP().toString();
  String output;
  serializeJson(doc, output);
  return output;
}

void sendState() {
  if (ws.count() == 0) { lastStateSent = millis(); return; }
  ws.textAll(buildStateJson());
  lastStateSent = millis();
}

void sendTelemetry() {
  if (ws.count() == 0) return;
  const RuntimeSnapshot state = copyRuntimeSnapshot();
  JsonDocument doc;
  doc["type"] = "telemetry";
  doc["time"] = (uint32_t)millis();
  doc["angle"] = state.angle;
  doc["velocity"] = state.velocity;
  doc["target"] = state.target;
  doc["uq"] = state.uq;
  doc["current"] = state.currentAmps;
  doc["enabled"] = state.enabled;
  doc["rate"] = state.rate;
  doc["magnitude"] = state.magnitude;
  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}

void doEstop() {
  deckEstop = true;
  deckMode = platter::ESTOP;
#if MOTOR_STAGE_ENABLED
  motor.target = 0.0f;
  if (driverOK) motor.disable();
  digitalWrite(MOTOR_ENABLE, LOW);
#endif
  sendMessage(false, "E-STOP: target cleared and motor output disabled");
}

void disableDeck(platter::Mode mode) {
  motor.target = 0;
  if (driverOK) motor.disable();
  digitalWrite(MOTOR_ENABLE, LOW);
  deckMode = mode;
  saveHarmonicsIfDirty();
}

// Runs every control loop right after motor.move(): add the identified
// cogging compensation, plus any bench probe, to the q-axis voltage.
void serviceCompensation() {
  if (!motor.enabled || motor.controller != MotionControlType::velocity) return;
  // Index by the raw encoder angle in the motor's direction, not the
  // low-pass filtered shaft_angle: any lag here misphases the table.
  const float direction = motor.sensor_direction == Direction::CCW ? -1.0f : 1.0f;
  const float angle = direction * sensor.getMechanicalAngle();
  // Probes add on top so a residual can be identified with the
  // compensation in place.
  float ff = compLookup(angle);
  if (probeActive) {
    for (const auto& p : probe)
      if (p.k > 0 && p.amp > 0) ff += p.amp * sinf(p.k * angle + p.phase);
  }
  if (ff != 0.0f) {
    const float limit = motor.voltage_limit;
    float uq = motor.voltage.q + ff;
    if (uq > limit) { uq = limit; ++ffClipCount; }
    if (uq < -limit) { uq = -limit; ++ffClipCount; }
    motor.voltage.q = uq;
  }
}

void serviceDeckMotor() {
  const uint32_t now = micros();
  if (deckSeen && !deckTimedOut && uint32_t(now - deckLastRx) > platter::TIMEOUT_US) {
    deckTimedOut = true;
    disableDeck(deckEstop ? platter::ESTOP : platter::FREE);
    stateRequested = true;
  }
  platter::Command c;
  // Bounded FIFO preserves FREE before SPIN and never overwrites an E-STOP.
  for (int i = 0; i < 8 && xQueueReceive(deckCommands, &c, 0) == pdTRUE; ++i) {
    if (uint32_t(now - c.receivedUs) > platter::TIMEOUT_US) continue;
    const auto mode = static_cast<platter::Mode>(c.packet.payload[2]);
    if ((!deckSeen || deckTimedOut) && mode != platter::FREE) continue;
    deckSeen = true; deckLastRx = c.receivedUs; deckTimedOut = false;
    stateRequested = true;
    if (mode == platter::FREE) {
      deckEstop = false;
      disableDeck(mode);
    } else if (mode == platter::ESTOP) {
      deckEstop = true;
      disableDeck(mode);
    } else if (!deckEstop && sensorOK && focReady && driverOK) {
      if (deckMode != mode || !motor.enabled) applyMode(MODE_VELOCITY);
      // PC torque is additionally capped by the saved dashboard limit.
      motor.voltage_limit = min(cfg.voltageLimit, c.packet.payload[3] * 0.02f);
      motor.PID_velocity.limit = motor.voltage_limit;
      motor.target = mode == platter::SPIN
          ? clampFinite(int32_t(platter::read32(c.packet.payload + 4)) * 0.001f,
                        -cfg.velocityLimit, cfg.velocityLimit) : 0;
      if (!motor.enabled) motor.enable();
      deckMode = mode;
    } else {
      disableDeck(deckEstop ? platter::ESTOP : platter::FREE);
    }
  }
}

void publishDeckSample() {
  static uint32_t last = 0, sequence = 0;
  const uint32_t now = micros();
  if (uint32_t(now - last) < 2000) return; // 500 Hz; never queue historical samples.
  last += 2000;
  if (uint32_t(now - last) >= 2000) last = now;
  platter::Sample s;
  // Stamp the sample with the encoder read time, not the publish time.
  s.timeUs = sensorOK ? sensor.lastReadUs() : now; s.seq = sequence++;
  if (sensorOK) {
    // Preserve precision after many turns without converting a large float angle.
    const int64_t units = int64_t(sensor.getFullRotations()) * 65536 +
        int64_t(lroundf(sensor.getMechanicalAngle() * (65536.0f / _2PI)));
    // Match SimpleFOC's calibrated positive direction: SPIN +33 plays forward
    // even when alignment found a reversed sensor/phase orientation.
    const int direction = cfg.sensorDirection < 0 ? -1 : 1;
    s.angle = int32_t(uint32_t(units * direction));
    deckVelocityRad = deckVelocity.update(s.angle, now);
    s.velocity = int32_t(lroundf(deckVelocityRad * 1000));
  }
  s.status = (motorIsEnabled() ? platter::ENABLED : 0) |
      (sensorOK ? platter::SENSOR_OK : 0) |
      ((!sensorOK || !driverOK || !focReady) ? platter::FAULT : 0) |
      (deckTimedOut ? platter::LINK_TIMEOUT : 0) | (uint8_t(deckMode) << 4);
  if (xQueueSend(deckSamples, &s, 0) != pdTRUE) {
    platter::Sample discarded;
    xQueueReceive(deckSamples, &discarded, 0);
    xQueueSend(deckSamples, &s, 0);
  }
  xSemaphoreGive(networkWake);
}

void serviceDeckNetwork() {
  // All socket operations stay on core 0.
  for (int i = 0; i < 8; ++i) {
    const int size = deckUdp.parsePacket();
    if (!size) break;
    const IPAddress ip = deckUdp.remoteIP();
    const uint16_t port = deckUdp.remotePort();
    uint8_t bytes[platter::PACKET_LEN];
    if (size != sizeof(bytes)) { deckUdp.flush(); continue; }
    const int n = deckUdp.read(bytes, sizeof(bytes));
    platter::Command c;
    if (!c.packet.decode(bytes, n) || httpUpdateInProgress) continue;
    c.receivedUs = micros();
    if (!deckGate.accept(c.packet, c.receivedUs, bootId, ip == deckPeer && port == deckPeerPort)) continue;
    deckPeer = ip; deckPeerPort = port;
    // A full queue cannot delay E-STOP behind a command flood: latch via the
    // dedicated stop request, consumed before any queued motor command.
    if (xQueueSend(deckCommands, &c, 0) != pdTRUE && c.packet.payload[2] == platter::ESTOP)
      httpUpdateEStopRequested = true;
  }
  // Four timestamped samples per datagram avoids 500 tiny Wi-Fi sends/sec.
  // Preserve intermediate angles for the PC's interpolation buffer.
  static platter::Sample pending[4];
  static unsigned count = 0;
  if (!deckGate.live(micros())) {
    count = 0;
    platter::Sample discarded;
    while (xQueueReceive(deckSamples, &discarded, 0) == pdTRUE) {}
    return;
  }
  while (count < 4 && xQueueReceive(deckSamples, &pending[count], 0) == pdTRUE) ++count;
  if (!count || (count < 4 && uint32_t(micros() - pending[0].timeUs) < 8000)) return;
  uint8_t bytes[platter::PACKET_LEN * 4]; unsigned length = 0;
  for (unsigned i = 0; i < count; ++i) {
    const auto& s = pending[i];
    if (uint32_t(micros() - s.timeUs) > platter::TIMEOUT_US) continue;
    platter::Packet p;
    p.kind = platter::TELEMETRY; p.session = deckGate.session;
    p.seq = s.seq; p.timeUs = s.timeUs; p.boot = bootId;
    p.payload[0] = 0xaa; p.payload[1] = uint8_t(s.seq);
    platter::write32(p.payload + 2, uint32_t(s.angle));
    platter::write32(p.payload + 6, uint32_t(s.velocity));
    p.payload[10] = s.status; p.payload[11] = platter::crc8(p.payload, 11);
    p.encode(bytes + length); length += platter::PACKET_LEN;
  }
  count = 0;
  if (length && deckUdp.beginPacket(deckPeer, deckPeerPort)) {
    deckUdp.write(bytes, length); deckUdp.endPacket();
  }
}

void runCalibration() {
#if !MOTOR_STAGE_ENABLED
  sendMessage(false, "Motor stage is locked in this build; use esp32dev-motor after verifying pins");
  return;
#else
  if (!driverOK || !sensorOK) {
    sendMessage(false, "Calibration requires a working driver and AS5048A");
    return;
  }
  sendMessage(true, "Calibrating: the motor will move");
  motor.target = 0.0f;
  motor.zero_electric_angle = NOT_SET;
  motor.sensor_direction = Direction::UNKNOWN;
#if CURRENT_SENSE_ENABLED
  if (currentSenseOK) currentSense.skip_align = false;
#endif
  applyTuning();
  motor.enable();
  const int result = motor.initFOC();
  motor.target = 0.0f;
  motor.disable();
  if (result == 1) {
    focReady = true;
    cfg.calibrated = true;
    cfg.zeroElectricAngle = motor.zero_electric_angle;
    cfg.sensorDirection = motor.sensor_direction == Direction::CCW ? -1 : 1;
#if CURRENT_SENSE_ENABLED
    if (currentSenseOK) {
      cfg.csGainA = currentSense.gain_a;
      cfg.csGainB = currentSense.gain_b;
    }
#endif
    saveConfig();
    sendMessage(true, "Calibration complete; configuration saved");
  } else {
    focReady = false;
    cfg.calibrated = false;
    sendMessage(false, "Calibration failed; check phases, SPI and pole pairs");
  }
#endif
}

void applyCommand(const Cmd& cmd) {
  if (httpUpdateInProgress || httpUpdateRestartRequested) return;
  if (deckSeen && !deckTimedOut && cmd.kind != CK_ESTOP &&
      cmd.kind != CK_PROBE && cmd.kind != CK_SET_HARMONIC && cmd.kind != CK_CLEAR_HARMONICS) {
    sendMessage(false, "Disconnect the Windows deck before changing dashboard controls");
    return;
  }
  switch (cmd.kind) {
    case CK_TARGET:
#if MOTOR_STAGE_ENABLED
      motor.target = clampFinite(cmd.f[0], -targetLimit(), targetLimit());
#endif
      break;

    case CK_MODE: {
      const UiMode mode = (UiMode)constrain((int)cmd.f[0], 0, 4);
      if (isClosedMode(mode) && (!focReady || !sensorOK)) {
        sendMessage(false, "A live encoder and calibration are required for closed-loop mode");
        break;
      }
      if (mode == MODE_TORQUE && !currentSenseOK) {
        // Voltage torque is still valid without a current sensor.
#if MOTOR_STAGE_ENABLED
        motor.torque_controller = TorqueControlType::voltage;
#endif
      }
      applyMode(mode);
      break;
    }

    case CK_ENABLE:
#if MOTOR_STAGE_ENABLED
      if (!driverOK) {
        sendMessage(false, "Motor driver did not initialise");
      } else if (cmd.f[0] > 0.5f) {
        deckEstop = false; // explicit local enable after stopping the deck
        applyTuning();
        if (isClosedMode(selectedMode) && (!focReady || !sensorOK)) {
          sendMessage(false, "A live encoder and calibration are required for closed-loop control");
        } else {
          motor.PID_velocity.reset();
          motor.P_angle.reset();
          if (selectedMode == MODE_ANG_OPEN || selectedMode == MODE_ANGLE)
            motor.target = sensorOK ? sensor.getAngle() : 0.0f;
          motor.enable();
          sendMessage(true, "Motor output enabled");
        }
      } else {
        motor.target = 0.0f;
        motor.disable();
        sendMessage(true, "Motor output disabled");
      }
#else
      sendMessage(false, "Motor stage is locked in the safe build");
#endif
      break;

    case CK_ESTOP:
      doEstop();
      break;

    case CK_PID:
      if (!isnan(cmd.f[0])) cfg.velP = clampFinite(cmd.f[0], 0, MAX_PID_P);
      if (!isnan(cmd.f[1])) cfg.velI = clampFinite(cmd.f[1], 0, MAX_PID_I);
      if (!isnan(cmd.f[2])) cfg.velD = clampFinite(cmd.f[2], 0, MAX_PID_D);
      if (!isnan(cmd.f[3])) cfg.velTf = clampFinite(cmd.f[3], 0, MAX_PID_TF);
      if (!isnan(cmd.f[4])) cfg.angP = clampFinite(cmd.f[4], 0, MAX_ANG_P);
      applyTuning();
      break;

    case CK_LIMITS:
      if (!isnan(cmd.f[0]))
        cfg.voltageLimit = clampFinite(cmd.f[0], 0, min(cfg.supply, MAX_VOLTAGE_LIMIT));
      if (!isnan(cmd.f[1]))
        cfg.velocityLimit = clampFinite(cmd.f[1], 0, MAX_VELOCITY_LIMIT);
      if (!isnan(cmd.f[2]))
        cfg.currentLimit = clampFinite(cmd.f[2], 0, MAX_CURRENT_LIMIT);
      applyTuning();
      break;

    case CK_SUPPLY:
      cfg.supply = clampFinite(cmd.f[0], MIN_SUPPLY_VOLTAGE, MAX_SUPPLY_VOLTAGE);
      cfg.voltageLimit = min(cfg.voltageLimit, min(cfg.supply, MAX_VOLTAGE_LIMIT));
#if MOTOR_STAGE_ENABLED
      driver.voltage_power_supply = cfg.supply;
      driver.voltage_limit = cfg.supply;
#endif
      applyTuning();
      break;

    case CK_POLE_PAIRS:
      if (cfg.polePairs == constrain((int)cmd.f[0], MIN_POLE_PAIRS, MAX_POLE_PAIRS)) break;
      disableDeck(platter::FREE);
      cfg.polePairs = constrain((int)cmd.f[0], MIN_POLE_PAIRS, MAX_POLE_PAIRS);
      motor.pole_pairs = cfg.polePairs;
      if (focReady) {
        focReady = false;
        cfg.calibrated = false;
        sendMessage(true, "Pole pairs changed; recalibration required");
      }
      break;

    case CK_CALIBRATE:
      runCalibration();
      break;

    case CK_SAVE:
      saveConfig();
      break;

    case CK_RESTART:
      doEstop();
      sendMessage(true, "Restarting controller");
      delay(250);
      ESP.restart();
      break;

    case CK_PROBE: {
      // f[0] slot 0..2, f[1] harmonic k (0 clears the slot), f[2] amplitude V, f[3] phase rad
      const int slot = constrain((int)cmd.f[0], 0, 2);
      probe[slot].k = clampFinite(cmd.f[1], 0.0f, 512.0f);
      probe[slot].amp = clampFinite(cmd.f[2], 0.0f, 0.5f);
      probe[slot].phase = clampFinite(cmd.f[3], -7.0f, 7.0f);
      probeActive = false;
      for (const auto& p : probe) if (p.k > 0 && p.amp > 0) probeActive = true;
      sendMessage(true, probeActive ? "Probe harmonics active" : "Probe cleared");
      break;
    }

    case CK_SET_HARMONIC: {
      // f[0] slot, f[1] k (0 clears), f[2] amplitude V, f[3] phase rad
      const int slot = constrain((int)cmd.f[0], 0, COMP_SLOTS - 1);
      comp[slot].k = clampFinite(cmd.f[1], 0.0f, 512.0f);
      comp[slot].amp = clampFinite(cmd.f[2], 0.0f, 0.5f);
      comp[slot].phase = clampFinite(cmd.f[3], -7.0f, 7.0f);
      compDirty = true;
      compRebuild();
      sendMessage(true, "Compensation harmonic set");
      break;
    }

    case CK_CLEAR_HARMONICS:
      for (auto& h : comp) h = ProbeHarmonic();
      compDirty = true;
      compRebuild();
      ffClipCount = 0;
      sendMessage(true, "Compensation harmonics cleared");
      break;
  }
  stateRequested = true;
  if (networkWake) xSemaphoreGive(networkWake);
}

void drainCommands() {
  Cmd cmd;
  while (cmdQueue && xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE)
    applyCommand(cmd);
}

void queueCommand(const Cmd& cmd) {
  if (cmdQueue) xQueueSend(cmdQueue, &cmd, 0);
}

float optionalFloat(JsonVariantConst value) {
  return value.isNull() ? NAN : value.as<float>();
}

void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
  if (!(info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT)) return;

  JsonDocument doc;
  if (deserializeJson(doc, data, len)) return;
  const String action = doc["action"] | "";
  Cmd cmd = {};

  if (action == "setTarget") {
    cmd.kind = CK_TARGET;
    cmd.f[0] = doc["value"] | 0.0f;
  } else if (action == "setMode") {
    cmd.kind = CK_MODE;
    cmd.f[0] = (float)(int)(doc["mode"] | 0);
  } else if (action == "setEnable") {
    cmd.kind = CK_ENABLE;
    cmd.f[0] = (doc["on"] | false) ? 1.0f : 0.0f;
  } else if (action == "estop") {
    cmd.kind = CK_ESTOP;
  } else if (action == "setPID") {
    cmd.kind = CK_PID;
    cmd.f[0] = optionalFloat(doc["velP"]);
    cmd.f[1] = optionalFloat(doc["velI"]);
    cmd.f[2] = optionalFloat(doc["velD"]);
    cmd.f[3] = optionalFloat(doc["velTf"]);
    cmd.f[4] = optionalFloat(doc["angP"]);
  } else if (action == "setLimits") {
    cmd.kind = CK_LIMITS;
    cmd.f[0] = optionalFloat(doc["voltage"]);
    cmd.f[1] = optionalFloat(doc["velocity"]);
    cmd.f[2] = optionalFloat(doc["current"]);
  } else if (action == "setSupply") {
    cmd.kind = CK_SUPPLY;
    cmd.f[0] = doc["volts"] | DEFAULT_SUPPLY_VOLTAGE;
  } else if (action == "setPolePairs") {
    cmd.kind = CK_POLE_PAIRS;
    cmd.f[0] = (float)(int)(doc["value"] | DEFAULT_POLE_PAIRS);
  } else if (action == "calibrate") {
    cmd.kind = CK_CALIBRATE;
  } else if (action == "saveConfig") {
    cmd.kind = CK_SAVE;
  } else if (action == "restart") {
    cmd.kind = CK_RESTART;
  } else if (action == "probe" || action == "setHarmonic") {
    cmd.kind = action == "probe" ? CK_PROBE : CK_SET_HARMONIC;
    cmd.f[0] = (float)(int)(doc["slot"] | 0);
    cmd.f[1] = doc["k"] | 0.0f;
    cmd.f[2] = doc["amp"] | 0.0f;
    cmd.f[3] = doc["phase"] | 0.0f;
  } else if (action == "clearHarmonics") {
    cmd.kind = CK_CLEAR_HARMONICS;
  } else if (action == "getState") {
    stateRequested = true;
    if (networkWake) xSemaphoreGive(networkWake);
    return;
  } else {
    return;
  }
  queueCommand(cmd);
}

void onWebSocketEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    stateRequested = true;
    if (networkWake) xSemaphoreGive(networkWake);
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

void handleFirmwareUpload(AsyncWebServerRequest*, const String& filename,
                          size_t index, uint8_t* data, size_t len, bool final) {
  if (index == 0) {
    httpUpdateWriteFailed = false;
    httpUpdateReady = false;
    httpUpdateRestartRequested = false;
    String lower = filename;
    lower.toLowerCase();
    if (!lower.endsWith(".bin")) {
      httpUpdateWriteFailed = true;
      return;
    }
    xSemaphoreTake(updateStopped, 0);
    httpUpdateInProgress = true;
    httpUpdateEStopRequested = true;
    // Do not erase/write flash until the motor task has disabled the bridge.
    if (xSemaphoreTake(updateStopped, pdMS_TO_TICKS(500)) != pdTRUE) {
      httpUpdateWriteFailed = true;
      httpUpdateInProgress = false;
      return;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      httpUpdateInProgress = false;
      httpUpdateWriteFailed = true;
      Update.printError(Serial);
      return;
    }
  }

  if (!httpUpdateInProgress) return;
  if (!httpUpdateWriteFailed && len && Update.write(data, len) != len) {
    httpUpdateWriteFailed = true;
    Update.printError(Serial);
  }

  if (final) {
    const bool completed = !httpUpdateWriteFailed && Update.end(true);
    if (!completed) {
      httpUpdateWriteFailed = true;
      Update.abort();
      httpUpdateInProgress = false;
    } else {
      httpUpdateReady = true;
    }
  }
}

void setupFirmwareEndpoint() {
  server.on("/update", HTTP_POST,
    [](AsyncWebServerRequest* request) {
      if (httpUpdateWriteFailed || !httpUpdateReady) {
        httpUpdateWriteFailed = false;
        request->send(400, "application/json",
                      "{\"ok\":false,\"error\":\"Firmware update failed\"}");
        return;
      }
      httpUpdateReady = false;
      request->send(200, "application/json",
                    "{\"ok\":true,\"message\":\"Firmware accepted; rebooting\"}");
      httpUpdateRestartRequested = true;
      httpUpdateRestartAt = millis() + 1500;
    },
    handleFirmwareUpload);
}

void setupSensor() {
  pinMode(ENC_CS, OUTPUT);
  digitalWrite(ENC_CS, HIGH);
  SPI.begin(ENC_SCLK, ENC_MISO, ENC_MOSI, ENC_CS);
  sensorOK = probeSensor();
  sensor.init();
  if (!sensorOK) Serial.println("ERROR: AS5048A not detected");
}

void setupMotor() {
#if MOTOR_STAGE_ENABLED
  driver.pwm_frequency = FOC_PWM_FREQ;
  driver.voltage_power_supply = cfg.supply;
  driver.voltage_limit = cfg.supply;
  driverOK = driver.init();
  if (!driverOK) {
    Serial.println("ERROR: motor PWM initialisation failed");
    return;
  }

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.pole_pairs = cfg.polePairs;
  motor.torque_controller = TorqueControlType::voltage;
  applyMode(MODE_VEL_OPEN);
  applyTuning();
  motor.target = 0.0f;
  motor.init();

#if CURRENT_SENSE_ENABLED
  currentSense.linkDriver(&driver);
  currentSenseOK = currentSense.init() == 1;
  if (currentSenseOK) motor.linkCurrentSense(&currentSense);
#endif

  if (sensorOK && cfg.calibrated) {
    motor.zero_electric_angle = cfg.zeroElectricAngle;
    motor.sensor_direction = cfg.sensorDirection < 0 ? Direction::CCW : Direction::CW;
#if CURRENT_SENSE_ENABLED
    if (currentSenseOK) {
      currentSense.skip_align = true;
      if (cfg.csGainA != 0.0f) {
        currentSense.gain_a = cfg.csGainA;
        currentSense.gain_b = cfg.csGainB;
      }
    }
#endif
    focReady = motor.initFOC() == 1;
    if (!focReady) cfg.calibrated = false;
  }

  motor.target = 0.0f;
  motor.disable();
#endif
}

bool setupAccessPoint() {
  // Keep AP startup identical to Espressif's supported minimal sequence.
  WiFi.mode(WIFI_AP);
  const bool started = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD,
                                   WIFI_CHANNEL, false, WIFI_MAX_CLIENTS);
  const IPAddress ip = WiFi.softAPIP();
  const bool ready = started && ip != IPAddress(0, 0, 0, 0);
  if (ready)
    Serial.printf("AP ready: %s  http://%s\n", WIFI_SSID, ip.toString().c_str());
  else
    Serial.println("ERROR: Wi-Fi AP failed");
  return ready;
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    auto* response = request->beginResponse(200, "text/html", web_index, sizeof(web_index));
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Cache-Control", "no-store"); request->send(response);
  });
  server.on("/app.js", HTTP_GET, [](AsyncWebServerRequest* request) {
    auto* response = request->beginResponse(200, "application/javascript", web_app, sizeof(web_app));
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Cache-Control", "no-store"); request->send(response);
  });
  server.on("/styles.css", HTTP_GET, [](AsyncWebServerRequest* request) {
    auto* response = request->beginResponse(200, "text/css", web_styles, sizeof(web_styles));
    response->addHeader("Content-Encoding", "gzip");
    response->addHeader("Cache-Control", "no-store"); request->send(response);
  });
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "application/json", "{\"ok\":true,\"firmware\":\"PlatterDeck-WiFi\",\"protocol\":2,\"udpPort\":19704,\"telemetryHz\":500}");
  });
  setupFirmwareEndpoint();
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();
}

void motorControlTask(void*) {
  unsigned long snapshotAt = 0;
  uint32_t yieldCounter = 0;

  for (;;) {
    if (httpUpdateEStopRequested) {
      httpUpdateEStopRequested = false;
      doEstop();
      xQueueReset(deckCommands);
      xQueueReset(cmdQueue);
      xSemaphoreGive(updateStopped);
    }

    if (httpUpdateInProgress) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    serviceDeckMotor();
    drainCommands();
    compBuildStep();
    if (sensorOK && !sensor.healthy()) {
      // Sixteen consecutive bad SPI frames: never drive FOC from that.
      sensorOK = false;
      deckEstop = true;
      disableDeck(platter::ESTOP);
    }

#if MOTOR_STAGE_ENABLED
    if (driverOK && motor.enabled) {
      motor.loopFOC();
      motor.move();
      serviceCompensation();
    } else if (sensorOK) {
      sensor.update();
    }
#else
    if (sensorOK) sensor.update();
#endif

    loopCounter++;
    const unsigned long now = millis();

    if (now - lastSensorProbe >= SENSOR_PROBE_MS) {
      lastSensorProbe = now;
      sensorOK = probeSensor();
      if (!sensorOK) { deckEstop = true; disableDeck(platter::ESTOP); }
    }
    if (now - lastSecond >= 1000) {
      lastSecond = now;
      loopRate = loopCounter;
      loopCounter = 0;
    }
    if (now - snapshotAt >= 10) {
      snapshotAt = now;
      updateRuntimeSnapshot();
    }
    publishDeckSample();

    // Let the core-1 idle task run without reducing the active control rate
    // to the 1 kHz RTOS tick frequency.
    if (!motorIsEnabled() || ++yieldCounter >= 64) {
      yieldCounter = 0;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void networkTask(void*) {
  bool apReady = setupAccessPoint();
  bootId = esp_random(); if (!bootId) bootId = 1;
  bool udpReady = deckUdp.begin(platter::PORT);
  setupWebServer();
  xSemaphoreGive(networkReady);

  unsigned long lastApRetry = millis();
  unsigned long lastCleanup = millis();
  for (;;) {
    const unsigned long now = millis();
    if (udpReady) serviceDeckNetwork();

    if (httpUpdateRestartRequested &&
        (long)(now - httpUpdateRestartAt) >= 0) {
      ESP.restart();
    }

    if (!apReady || WiFi.getMode() != WIFI_AP ||
        WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) {
      if (now - lastApRetry >= 2000) {
        lastApRetry = now;
        apReady = setupAccessPoint();
        deckUdp.stop(); udpReady = deckUdp.begin(platter::PORT);
      }
    }

    if (now - lastTelemetry >= TELEMETRY_MS) {
      lastTelemetry = now;
      sendTelemetry();
    }
    if (stateRequested && now - lastStateSent >= STATE_MIN_MS) {
      stateRequested = false;
      sendState();
    }
    if (now - lastCleanup >= 1000) {
      lastCleanup = now;
      ws.cleanupClients();
    }

    if (!udpReady && now - lastApRetry >= 2000) {
      lastApRetry = now; udpReady = deckUdp.begin(platter::PORT);
    }
    xSemaphoreTake(networkWake, pdMS_TO_TICKS(1));
  }
}

void setup() {
#if MOTOR_STAGE_ENABLED
  // Keep the IR2104 bridge shut down before filesystem/network startup.
  digitalWrite(MOTOR_ENABLE, LOW);
  pinMode(MOTOR_ENABLE, OUTPUT);
  const int pwmPins[] = {MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C};
  for (int pin : pwmPins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
#endif

  Serial.begin(115200);

  cmdQueue = xQueueCreate(CMD_QUEUE_LEN, sizeof(Cmd));
  stateMutex = xSemaphoreCreateMutex();
  networkWake = xSemaphoreCreateBinary();
  networkReady = xSemaphoreCreateBinary();
  deckCommands = xQueueCreate(8, sizeof(platter::Command));
  deckSamples = xQueueCreate(16, sizeof(platter::Sample));
  updateStopped = xSemaphoreCreateBinary();
  if (!cmdQueue || !stateMutex || !networkWake || !networkReady || !deckCommands || !deckSamples || !updateStopped) {
    Serial.println("ERROR: RTOS object allocation failed");
    return;
  }

  loadConfig();
  setupSensor();

  // Start radio ownership on core 0 and wait until the AP and server exist
  // before any PWM peripheral is configured on core 1.
  if (xTaskCreatePinnedToCore(networkTask, "network", NETWORK_TASK_STACK,
                              nullptr, NETWORK_TASK_PRIORITY, nullptr, 0) != pdPASS) {
    Serial.println("ERROR: network task creation failed");
    return;
  }
  if (xSemaphoreTake(networkReady, pdMS_TO_TICKS(5000)) != pdTRUE)
    Serial.println("ERROR: network task startup timeout");

  setupMotor();
  updateRuntimeSnapshot();
  if (xTaskCreatePinnedToCore(motorControlTask, "motor", MOTOR_TASK_STACK,
                              nullptr, MOTOR_TASK_PRIORITY, nullptr, 1) != pdPASS)
    Serial.println("ERROR: motor task creation failed");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

