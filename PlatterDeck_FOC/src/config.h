#ifndef GM2804_CONFIG_H
#define GM2804_CONFIG_H

// Board: FOC_ESP32 / 303FOCESP01 / LC Technology LCT-279.
// ESP32-WROOM-32E, 3 x IR2104, 6 x NMOS, 2 x INA240A2.

#ifndef MOTOR_STAGE_ENABLED
#define MOTOR_STAGE_ENABLED 0
#endif

#ifndef CURRENT_SENSE_ENABLED
#define CURRENT_SENSE_ENABLED 0
#endif

#if CURRENT_SENSE_ENABLED && !MOTOR_STAGE_ENABLED
#error "Current sensing requires MOTOR_STAGE_ENABLED"
#endif

// ---------------- Network and web service ----------------
#define WIFI_SSID       "PlatterDeck"
#define WIFI_PASSWORD   "12345678"
#define WIFI_CHANNEL    1
#define WIFI_MAX_CLIENTS 4
#define HTTP_PORT       80

// ---------------- AS5048A encoder: FOC_ESP32_V1.1 P2 header ----------------
// MOSI/MISO/SCLK use the classic ESP32 VSPI pins, but this PCB routes P2 CS
// to GPIO13. GPIO5 is phase-C PWM and must not be used as encoder CS.
#define ENC_SCLK        18
#define ENC_MISO        19
#define ENC_MOSI        23
#define ENC_CS          13
#define ENC_SPI_HZ      1000000
#define ENC_ANGLE_REG   0x3FFF
#define ENC_MAG_REG     0x3FFE
#define ENC_DIAG_REG    0x3FFD
#define ENC_ERROR_REG   0x0001
#define ENC_DIAG_SPI_HZ 100000

// ---------------- Motor stage: FOC_ESP32_V1.1 map ----------------
// Confirmed by a working-board report and example for this exact board family.
#define MOTOR_PWM_A     16
#define MOTOR_PWM_B     17
#define MOTOR_PWM_C      5
#define MOTOR_ENABLE     4
#define FOC_PWM_FREQ    20000

// GPIO4 controls the common IR2104 shutdown/enable net.

// ---------------- Current sensing: FOC_ESP32_V1.1 map ----------------
// Two INA240A2 amplifiers, 10 mOhm shunts and gain 50 V/V.
#define CS_SHUNT_OHM    0.01f
#define CS_GAIN         50.0f
#define CS_PHASE_A      39
#define CS_PHASE_B      36

// ---------------- Conservative GM2804 defaults ----------------
#define DEFAULT_SUPPLY_VOLTAGE    12.0f
#define DEFAULT_POLE_PAIRS         7
#define DEFAULT_VOLTAGE_LIMIT      0.5f
#define DEFAULT_VELOCITY_LIMIT    20.0f
#define DEFAULT_CURRENT_LIMIT      0.5f
// Velocity loop: the quietest stable point of a gain sweep with the
// least-squares estimator. Higher gains raise the 12-60 Hz band around the
// loop's ~27 Hz peak; P 0.2 / I 2 rings at 31 Hz. Angle-locked ripple is
// handled by the identified cogging harmonics in NVS, not by gain.
#define DEFAULT_VEL_P              0.08f
#define DEFAULT_VEL_I              0.4f
#define DEFAULT_VEL_D              0.0f
#define DEFAULT_VEL_TF             0.005f
#define DEFAULT_ANG_P             10.0f
#define ALIGN_VOLTAGE_MAX          0.75f

// Input clamps for all browser commands.
#define MIN_SUPPLY_VOLTAGE        12.0f
#define MAX_SUPPLY_VOLTAGE        28.0f
#define MAX_VOLTAGE_LIMIT          6.0f
#define MAX_VELOCITY_LIMIT       100.0f
#define MAX_CURRENT_LIMIT          3.0f
#define MAX_ANGLE_TARGET         314.16f
#define MAX_PID_P                 50.0f
#define MAX_PID_I               1000.0f
#define MAX_PID_D                  5.0f
#define MAX_PID_TF                 1.0f
#define MAX_ANG_P                100.0f
#define MIN_POLE_PAIRS             1
#define MAX_POLE_PAIRS            30

// ---------------- Runtime ----------------
#define TELEMETRY_MS              50
#define STATE_MIN_MS             200
#define SENSOR_PROBE_MS         1000
#define CMD_QUEUE_LEN             24
#define MOTOR_TASK_STACK        8192
#define NETWORK_TASK_STACK      8192
#define MOTOR_TASK_PRIORITY        3
#define NETWORK_TASK_PRIORITY      2
#define NVS_NAMESPACE          "gm2804foc"

#endif
