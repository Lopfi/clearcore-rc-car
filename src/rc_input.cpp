#include "rc_input.h"

#include <Arduino.h>
#include "ClearCore.h"
#include "config.h"

// Both protocols send a burst, then stay idle for several ms. An idle gap
// longer than this inside a frame means we lost sync -> restart.
static constexpr uint32_t FRAME_GAP_US = 2000;

#if RC_PROTOCOL == RC_PROTOCOL_SBUS
// SBUS: 0x0F | 22 bytes = 16 x 11-bit channels | flags | footer   (25 bytes)
static constexpr uint8_t FRAME_LEN = 25;
static constexpr uint8_t SBUS_HEADER = 0x0F;
static constexpr uint8_t SBUS_FLAG_FRAME_LOST = 0x04;
static constexpr uint8_t SBUS_FLAG_FAILSAFE   = 0x08;

// SBUS raw 172..992..1811  ->  ~988..1500..2012 us (same scaling the receivers use)
static inline uint16_t sbusToUs(uint16_t raw) { return (uint16_t)((raw * 5u) / 8u + 880u); }

#elif RC_PROTOCOL == RC_PROTOCOL_IBUS
// iBus: 0x20 0x40 | 14 x uint16 LE | checksum uint16 LE   (32 bytes)
static constexpr uint8_t FRAME_LEN = 32;
static constexpr uint8_t IBUS_CHANNELS = 14;
#else
#error "Unknown RC_PROTOCOL"
#endif

void RcInput::begin() {
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) m_us[i] = 1500;

#if RC_PROTOCOL == RC_PROTOCOL_SBUS
    // ClearCore's "RS232" mode inverts the UART signal on 0/5 V levels -
    // exactly what SBUS needs, no external inverter required.
    RC_PORT.Mode(Connector::RS232);
    RC_PORT.Speed(100000);
    RC_PORT.Parity(SerialBase::PARITY_E);
    RC_PORT.StopBits(2);
#else
    RC_PORT.Mode(Connector::TTL);
    RC_PORT.Speed(115200);
    RC_PORT.Parity(SerialBase::PARITY_N);
    RC_PORT.StopBits(1);
#endif
    RC_PORT.CharSize(8);
    RC_PORT.PortOpen();
}

void RcInput::poll() {
    const uint32_t now = micros();
    int16_t c;
    while ((c = RC_PORT.CharGet()) >= 0) {
        feed((uint8_t)c, now);
    }
}

void RcInput::feed(uint8_t b, uint32_t nowUs) {
    if (m_idx > 0 && (nowUs - m_lastByteUs) > FRAME_GAP_US) {
        m_idx = 0;  // stale partial frame
    }
    m_lastByteUs = nowUs;

#if RC_PROTOCOL == RC_PROTOCOL_SBUS
    if (m_idx == 0 && b != SBUS_HEADER) return;
#else
    if (m_idx == 0 && b != 0x20) return;
    if (m_idx == 1 && b != 0x40) {
        m_idx = 0;
        if (b != 0x20) return;  // this byte may itself be a new header
    }
#endif

    m_buf[m_idx++] = b;
    if (m_idx >= FRAME_LEN) {
        m_idx = 0;
        decodeFrame();
    }
}

void RcInput::decodeFrame() {
#if RC_PROTOCOL == RC_PROTOCOL_SBUS
    const uint8_t footer = m_buf[24];
    // 0x00 = SBUS, 0x04/0x14/0x24/0x34 = SBUS2
    if (footer != 0x00 && (footer & 0x0F) != 0x04) {
        m_errors++;
        return;
    }

    // Unpack 16 x 11 bit, LSB first
    uint32_t bits = 0;
    uint8_t nbits = 0;
    uint8_t ch = 0;
    for (uint8_t i = 1; i <= 22 && ch < 16; i++) {
        bits |= (uint32_t)m_buf[i] << nbits;
        nbits += 8;
        while (nbits >= 11 && ch < 16) {
            m_us[ch++] = sbusToUs(bits & 0x7FF);
            bits >>= 11;
            nbits -= 11;
        }
    }

    const uint8_t flags = m_buf[23];
    m_failsafe = (flags & SBUS_FLAG_FAILSAFE) != 0;
    if (flags & SBUS_FLAG_FRAME_LOST) m_errors++;  // single lost frame, informational

#else  // IBUS
    uint16_t sum = 0xFFFF;
    for (uint8_t i = 0; i < 30; i++) sum -= m_buf[i];
    const uint16_t rxSum = (uint16_t)m_buf[30] | ((uint16_t)m_buf[31] << 8);
    if (sum != rxSum) {
        m_errors++;
        return;
    }
    for (uint8_t ch = 0; ch < IBUS_CHANNELS; ch++) {
        const uint8_t lo = m_buf[2 + 2 * ch];
        const uint8_t hi = m_buf[3 + 2 * ch];
        m_us[ch] = ((uint16_t)lo | ((uint16_t)hi << 8)) & 0x0FFF;
    }
    // iBus has no failsafe flag: configure failsafe values on the transmitter,
    // or rely on the timeout if the receiver stops sending.
    m_failsafe = false;
#endif

    m_frames++;
    m_haveFrame = true;
    m_lastFrameMs = millis();
}

bool RcInput::signalOk() const {
    return m_haveFrame && !m_failsafe && (millis() - m_lastFrameMs) < RC_TIMEOUT_MS;
}

uint16_t RcInput::us(uint8_t ch) const {
    return ch < MAX_CHANNELS ? m_us[ch] : 1500;
}

float RcInput::axis(uint8_t ch, uint16_t deadbandUs) const {
    int32_t d = (int32_t)us(ch) - 1500;
    const int32_t db = deadbandUs;
    if (d <= db && d >= -db) return 0.0f;
    d += (d > 0) ? -db : db;
    float v = (float)d / (float)(500 - db);
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return v;
}
