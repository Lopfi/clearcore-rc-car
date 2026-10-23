#pragma once
#include <stdint.h>

// ============================================================================
//  Receiver
// ============================================================================
#define RC_PROTOCOL_SBUS 1   // 100000 baud, 8E2, inverted  -> COM port in RS232 mode
#define RC_PROTOCOL_IBUS 2   // 115200 baud, 8N1, normal    -> COM port in TTL mode
#define RC_PROTOCOL_PPM  3   // PPM sum signal on an interrupt-capable input (DI-6 .. A-12)

#ifndef RC_PROTOCOL
#define RC_PROTOCOL RC_PROTOCOL_PPM
#endif

// Which ClearCore COM connector the receiver is wired to (ConnectorCOM0 / ConnectorCOM1)
#define RC_PORT ConnectorCOM0
// Input used for PPM (must be DI-6, DI-7, DI-8, A-9 .. A-12)
#define RC_PPM_INPUT ConnectorDI6

// No valid frame for this long -> failsafe (motors stop, disarm)
constexpr uint32_t RC_TIMEOUT_MS = 100;

// Channel mapping, 0-based (Mode 2 AETR: ch1 = index 0 ...)
constexpr uint8_t CH_STEERING = 0;   // right stick left/right
constexpr uint8_t CH_THROTTLE = 1;   // left stick up/down (use a self-centering stick for forward/reverse!)
constexpr uint8_t CH_ARM      = 2;   // a 2-position switch

// Arm switch: set USE_ARM_SWITCH to 0 to arm automatically once the link is up
#ifndef USE_ARM_SWITCH
#define USE_ARM_SWITCH 1
#endif
constexpr uint16_t ARM_THRESHOLD_US = 1700;

// IO connector driven high while armed; its LED next to the connector lights up.
// The pin outputs supply voltage, so it can also drive an external 24 V lamp.
// Don't pick IO-0..IO-3 with MOTOR_TYPE_PWM_DIR.
#define ARMED_LED_OUTPUT ConnectorIO5

constexpr uint16_t STICK_DEADBAND_US = 30;   // around 1500 us center
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

// --- Step/dir fault recovery ---
// On a motor fault (HLFB deasserted while enabled = ClearPath shutdown) the
// enable line is cycled to clear it, then driving resumes. While one motor
// recovers, the other one is braked to 0 so the car doesn't spin off.
constexpr uint32_t FAULT_DISABLE_MS        = 100;   // enable held low to clear the shutdown
constexpr uint32_t FAULT_ENABLE_TIMEOUT_MS = 2000;  // HLFB must assert within this after re-enable
constexpr uint8_t  FAULT_MAX_RETRIES       = 5;     // then give up until disarmed + re-armed
constexpr uint32_t FAULT_RETRY_RESET_MS    = 3000;  // fault-free this long -> retry counter resets

// --- PWM/dir settings ---
constexpr float PWM_RAMP_PER_S = 3.0f;         // full scale change per second (0 -> 100% in ~0.33 s)
