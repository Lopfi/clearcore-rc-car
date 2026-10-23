#include "drive.h"

#include <Arduino.h>
#include <math.h>
#include "ClearCore.h"
#include "config.h"

#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
static MotorDriver *const kMotor[2] = {&ConnectorM0, &ConnectorM1};
#elif MOTOR_TYPE == MOTOR_TYPE_PWM_DIR
// NOTE: ClearCore IO outputs are *sourcing outputs at supply voltage (~24 V)*.
// Don't wire them straight into a 3.3/5 V logic input - see README.
static DigitalInOut *const kPwm[2] = {&ConnectorIO0, &ConnectorIO1};
static DigitalInOut *const kDir[2] = {&ConnectorIO2, &ConnectorIO3};
#else
#error "Unknown MOTOR_TYPE"
#endif

void Drive::begin() {
#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
    MotorMgr.MotorModeSet(MotorManager::MOTOR_M0M1, Connector::CPM_MODE_STEP_AND_DIR);
    for (uint8_t i = 0; i < 2; i++) {
        kMotor[i]->HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
        kMotor[i]->HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
        kMotor[i]->VelMax(MAX_VELOCITY_SPS);
        kMotor[i]->AccelMax(MAX_ACCEL_SPS2);
        kMotor[i]->EnableRequest(false);
    }
#else
    for (uint8_t i = 0; i < 2; i++) {
        kPwm[i]->Mode(Connector::OUTPUT_PWM);
        kPwm[i]->PwmDuty(0);
        kDir[i]->Mode(Connector::OUTPUT_DIGITAL);
        kDir[i]->State(false);
    }
#endif
    m_lastUpdateUs = micros();
}

void Drive::update(bool enable, float left, float right) {
    const uint32_t now = micros();
    float dt = (now - m_lastUpdateUs) * 1e-6f;
    m_lastUpdateUs = now;
    if (dt > 0.1f) dt = 0.1f;

    float cmd[2] = {INVERT_LEFT ? -left : left, INVERT_RIGHT ? -right : right};
    if (!enable) cmd[0] = cmd[1] = 0.0f;

    if (enable && !m_enabled) {
#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
        for (uint8_t i = 0; i < 2; i++) {
            kMotor[i]->ClearAlerts();
            kMotor[i]->EnableRequest(true);
            m_lastVel[i] = 0;
        }
#endif
        m_enabled = true;
    }

    if (m_enabled) {
        apply(0, cmd[0], dt);
        apply(1, cmd[1], dt);
    }

    // Disable only after both motors have braked to a stop
    if (!enable && m_enabled) {
#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
        const bool stopped = kMotor[0]->VelocityRefCommanded() == 0 &&
                             kMotor[1]->VelocityRefCommanded() == 0;
        if (stopped || faulted()) {
            kMotor[0]->EnableRequest(false);
            kMotor[1]->EnableRequest(false);
            m_enabled = false;
        }
#else
        if (m_out[0] == 0.0f && m_out[1] == 0.0f) m_enabled = false;
#endif
    }
}

void Drive::apply(uint8_t i, float cmd, float dt) {
#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
    (void)dt;  // ramping is done by the ClearCore step generator (AccelMax)
    const int32_t vel = (int32_t)lroundf(cmd * (float)MAX_VELOCITY_SPS);
    if (vel != m_lastVel[i]) {
        if (kMotor[i]->MoveVelocity(vel)) {
            m_lastVel[i] = vel;
        }
    }
#else
    // Simple slew-rate limit so direction changes don't slam the gearbox
    const float maxStep = PWM_RAMP_PER_S * dt;
    float d = cmd - m_out[i];
    if (d > maxStep) d = maxStep;
    if (d < -maxStep) d = -maxStep;
    m_out[i] += d;
    if (fabsf(m_out[i]) < 0.002f && cmd == 0.0f) m_out[i] = 0.0f;

    kDir[i]->State(m_out[i] < 0.0f);
    kPwm[i]->PwmDuty((uint8_t)lroundf(fabsf(m_out[i]) * 255.0f));
#endif
}

bool Drive::faulted() const {
#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
    for (uint8_t i = 0; i < 2; i++) {
        if (kMotor[i]->StatusReg().bit.AlertsPresent ||
            kMotor[i]->StatusReg().bit.MotorInFault) {
            return true;
        }
    }
#endif
    return false;
}
