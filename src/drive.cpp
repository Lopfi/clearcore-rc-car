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
        resetRecovery(millis());
#endif
        m_enabled = true;
    }

    // ClearCore itself forces the motor outputs off on Vsupply over-voltage
    // (and re-enables them once it drops). Hold still until then.
    if (supplyFault()) cmd[0] = cmd[1] = 0.0f;

    if (m_enabled) {
#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
        const uint32_t nowMs = millis();
        if (supplyFault()) {
            // Not the motor's fault: don't burn retries, pause recovery timers
            for (uint8_t i = 0; i < 2; i++) {
                if (m_rec[i] == Recovery::Disabling || m_rec[i] == Recovery::Enabling) {
                    m_recSinceMs[i] = nowMs;
                }
            }
        } else {
            superviseFaults(0, nowMs);
            superviseFaults(1, nowMs);
        }
        // One side down -> brake the other one too, otherwise the car spins
        if (recovering() || gaveUp()) cmd[0] = cmd[1] = 0.0f;
        for (uint8_t i = 0; i < 2; i++) {
            if (m_rec[i] == Recovery::None) apply(i, cmd[i], dt);
        }
#else
        apply(0, cmd[0], dt);
        apply(1, cmd[1], dt);
#endif
    }

    // Disable only after both motors have braked to a stop
    if (!enable && m_enabled) {
#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
        const bool stopped = kMotor[0]->VelocityRefCommanded() == 0 &&
                             kMotor[1]->VelocityRefCommanded() == 0;
        if (stopped || faulted()) {
            kMotor[0]->EnableRequest(false);
            kMotor[1]->EnableRequest(false);
            resetRecovery(millis());
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
        if (m_rec[i] != Recovery::None ||
            kMotor[i]->StatusReg().bit.AlertsPresent ||
            kMotor[i]->StatusReg().bit.MotorInFault) {
            return true;
        }
    }
#endif
    return false;
}

bool Drive::supplyFault() {
    const StatusManager::StatusRegister s = StatusMgr.StatusRT();
    return s.bit.VSupplyOverVoltage || s.bit.VSupplyUnderVoltage || s.bit.HBridgeReset;
}

bool Drive::recovering() const {
    return m_rec[0] == Recovery::Disabling || m_rec[0] == Recovery::Enabling ||
           m_rec[1] == Recovery::Disabling || m_rec[1] == Recovery::Enabling;
}

bool Drive::gaveUp() const {
    return m_rec[0] == Recovery::GaveUp || m_rec[1] == Recovery::GaveUp;
}

void Drive::resetRecovery(uint32_t nowMs) {
    for (uint8_t i = 0; i < 2; i++) {
        m_rec[i] = Recovery::None;
        m_attempts[i] = 0;
        m_okSinceMs[i] = nowMs;
    }
}

#if MOTOR_TYPE == MOTOR_TYPE_STEP_DIR
// A ClearPath shutdown shows up as HLFB deasserted while enabled. ClearCore
// then latches the MotorFaulted alert and rejects every further move until the
// alerts are cleared - so without this the motor stays dead until re-armed.
// Clearing the shutdown itself requires the enable line to go low for a moment.
void Drive::superviseFaults(uint8_t i, uint32_t nowMs) {
    MotorDriver &m = *kMotor[i];

    switch (m_rec[i]) {
    case Recovery::None:
        if (m.StatusReg().bit.MotorInFault || m.AlertReg().bit.MotorFaulted) {
            Serial.print("M");
            Serial.print(i);
            Serial.print(" fault, alerts=0x");
            Serial.println(m.AlertReg().reg, HEX);
            retryOrGiveUp(i, nowMs);
        } else if (m.StatusReg().bit.AlertsPresent) {
            // Other alerts (e.g. a rejected move) also block all further moves
            m.ClearAlerts();
            m_lastVel[i] = INT32_MIN;  // force apply() to resend the velocity
        } else if (m_attempts[i] && nowMs - m_okSinceMs[i] >= FAULT_RETRY_RESET_MS) {
            m_attempts[i] = 0;
        }
        break;

    case Recovery::Disabling:
        if (nowMs - m_recSinceMs[i] >= FAULT_DISABLE_MS) {
            m.ClearAlerts();
            m.EnableRequest(true);
            m_rec[i] = Recovery::Enabling;
            m_recSinceMs[i] = nowMs;
        }
        break;

    case Recovery::Enabling:
        if (m.StatusReg().bit.Enabled && m.HlfbState() != MotorDriver::HLFB_DEASSERTED) {
            m.ClearAlerts();  // MotorFaulted was latched again while HLFB came up
            m_rec[i] = Recovery::None;
            m_okSinceMs[i] = nowMs;
            m_lastVel[i] = INT32_MIN;
            Serial.print("M");
            Serial.print(i);
            Serial.println(" recovered");
        } else if (nowMs - m_recSinceMs[i] >= FAULT_ENABLE_TIMEOUT_MS) {
            Serial.print("M");
            Serial.print(i);
            Serial.println(" no HLFB after re-enable");
            retryOrGiveUp(i, nowMs);
        }
        break;

    case Recovery::GaveUp:
        break;
    }
}

void Drive::retryOrGiveUp(uint8_t i, uint32_t nowMs) {
    kMotor[i]->EnableRequest(false);
    if (m_attempts[i] >= FAULT_MAX_RETRIES) {
        m_rec[i] = Recovery::GaveUp;
        Serial.print("M");
        Serial.print(i);
        Serial.println(" giving up - disarm and re-arm to retry");
        return;
    }
    m_attempts[i]++;
    m_rec[i] = Recovery::Disabling;
    m_recSinceMs[i] = nowMs;
    Serial.print("M");
    Serial.print(i);
    Serial.print(" cycling enable, attempt ");
    Serial.println(m_attempts[i]);
}
#else
void Drive::superviseFaults(uint8_t, uint32_t) {}
void Drive::retryOrGiveUp(uint8_t, uint32_t) {}
#endif
