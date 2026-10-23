#pragma once
#include <stdint.h>

// Two drive motors, commanded with -1.0 .. +1.0 each.
// Backend (step/dir on M-0/M-1, or PWM+DIR on IO pins) is chosen in config.h.
class Drive {
public:
    void begin();

    // Call every loop. When enable == false the motors brake to zero
    // (ramped) and are then disabled.
    void update(bool enable, float left, float right);

    bool enabled() const { return m_enabled; }
    bool faulted() const;
    bool recovering() const;  // a motor is being cleared by cycling its enable
    bool gaveUp() const;      // too many failed recoveries; disarm + re-arm to retry
    static bool supplyFault(); // ClearCore Vsupply out of range -> motor outputs forced off

private:
    enum class Recovery : uint8_t { None, Disabling, Enabling, GaveUp };

    void apply(uint8_t i, float cmd, float dt);
    void superviseFaults(uint8_t i, uint32_t nowMs);
    void retryOrGiveUp(uint8_t i, uint32_t nowMs);
    void resetRecovery(uint32_t nowMs);

    bool     m_enabled = false;
    Recovery m_rec[2] = {Recovery::None, Recovery::None};
    uint32_t m_recSinceMs[2] = {0, 0};   // start of the current recovery step
    uint32_t m_okSinceMs[2] = {0, 0};    // last successful recovery
    uint8_t  m_attempts[2] = {0, 0};
    int32_t  m_lastVel[2] = {0, 0};      // step/dir
    float    m_out[2] = {0.0f, 0.0f};    // pwm/dir (ramped output)
    uint32_t m_lastUpdateUs = 0;
};
