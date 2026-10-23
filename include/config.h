#pragma once
#include <stdint.h>

// ============================================================================
//  Receiver
// ============================================================================
#define RC_PROTOCOL_SBUS 1   // 100000 baud, 8E2, inverted  -> COM port in RS232 mode
#define RC_PROTOCOL_IBUS 2   // 115200 baud, 8N1, normal    -> COM port in TTL mode

#ifndef RC_PROTOCOL
#define RC_PROTOCOL RC_PROTOCOL_IBUS
#endif

// Which ClearCore COM connector the receiver is wired to (ConnectorCOM0 / ConnectorCOM1)
#define RC_PORT ConnectorCOM1

// No valid frame for this long -> failsafe (motors stop, disarm)
constexpr uint32_t RC_TIMEOUT_MS = 100;

// Channel mapping, 0-based (Mode 2 AETR: ch1 = index 0 ...)
constexpr uint8_t CH_STEERING = 0;   // right stick left/right
constexpr uint8_t CH_THROTTLE = 2;   // left stick up/down (use a self-centering stick for forward/reverse!)
constexpr uint8_t CH_ARM      = 4;   // a 2-position switch

// Arm switch: set USE_ARM_SWITCH to 0 to arm automatically once the link is up
#ifndef USE_ARM_SWITCH
#define USE_ARM_SWITCH 1
#endif
constexpr uint16_t ARM_THRESHOLD_US = 1700;

constexpr uint16_t STICK_DEADBAND_US = 20;   // around 1500 us center
constexpr float    STEERING_GAIN     = 0.7f; // how hard steering differentiates the wheels

// Mixing: 1 = differential/tank (throttle + steering), 0 = direct (CH_LEFT/CH_RIGHT)
#ifndef MIX_DIFFERENTIAL
#define MIX_DIFFERENTIAL 1
#endif
constexpr uint8_t CH_LEFT  = 2;  // only used when MIX_DIFFERENTIAL == 0
constexpr uint8_t CH_RIGHT = 1;

// ============================================================================
//  Motors
// ============================================================================
#define MOTOR_TYPE_STEP_DIR 1  // ClearPath-SD or any step/dir drive on M-0 (left) / M-1 (right)
#define MOTOR_TYPE_PWM_DIR  2  // external H-bridge driver: PWM on IO-0/IO-1, DIR on IO-2/IO-3

#ifndef MOTOR_TYPE
#define MOTOR_TYPE MOTOR_TYPE_STEP_DIR
#endif

// Mirror one side so "forward" turns both wheels forward
constexpr bool INVERT_LEFT  = false;
constexpr bool INVERT_RIGHT = true;

// --- Step/dir settings (must match the steps/rev configured in ClearPath MSP) ---
constexpr int32_t  MAX_VELOCITY_SPS = 20000;   // steps/s at full stick (e.g. 800 steps/rev -> 1500 rpm)
constexpr uint32_t MAX_ACCEL_SPS2   = 60000;   // steps/s^2, also used for braking

// --- PWM/dir settings ---
constexpr float PWM_RAMP_PER_S = 3.0f;         // full scale change per second (0 -> 100% in ~0.33 s)
