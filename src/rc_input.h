#pragma once
#include <stdint.h>

// Reads SBUS or iBus (selected in config.h) from a ClearCore COM port.
// Channel values are normalized to "microseconds" (1000..2000, center 1500)
// for both protocols, so the rest of the code doesn't care which one is used.
class RcInput {
public:
    static constexpr uint8_t MAX_CHANNELS = 16;

    void begin();
    void poll();                      // call as often as possible from loop()

    bool     signalOk() const;        // recent frame and receiver not in failsafe
    uint16_t us(uint8_t ch) const;    // channel value in us
    float    axis(uint8_t ch, uint16_t deadbandUs) const;  // -1.0 .. +1.0

    uint32_t frames() const { return m_frames; }
    uint32_t errors() const { return m_errors; }
    bool     receiverFailsafe() const { return m_failsafe; }

private:
    void feed(uint8_t b, uint32_t nowUs);
    void decodeFrame();

    uint8_t  m_buf[32] = {};
    uint8_t  m_idx = 0;
    uint32_t m_lastByteUs = 0;
    uint32_t m_lastFrameMs = 0;
    bool     m_haveFrame = false;
    bool     m_failsafe = true;
    uint16_t m_us[MAX_CHANNELS] = {};
    uint32_t m_frames = 0;
    uint32_t m_errors = 0;
};
