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

private:
    void apply(uint8_t i, float cmd, float dt);

    bool     m_enabled = false;
    int32_t  m_lastVel[2] = {0, 0};      // step/dir
    float    m_out[2] = {0.0f, 0.0f};    // pwm/dir (ramped output)
    uint32_t m_lastUpdateUs = 0;
};
