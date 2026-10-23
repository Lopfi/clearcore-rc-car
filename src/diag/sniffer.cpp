// COM port sniffer: shows raw bytes coming from the receiver.
// Build/upload with:  pio run -e sniffer -t upload && pio device monitor
//
// Keys in the serial monitor:
//   i = iBus settings  (TTL,   115200 8N1)   <- default
//   s = SBUS settings  (RS232 = inverted, 100000 8E2)
//   0 / 1 = listen on COM-0 / COM-1
#include <Arduino.h>
#include "ClearCore.h"

static SerialDriver *port = &ConnectorCOM0;
static bool sbus = false;

static uint32_t bytesTotal = 0;
static uint32_t headers = 0;     // iBus 0x20 0x40 or SBUS 0x0F seen
static uint8_t dump[32];
static uint8_t dumpLen = 0;
static uint8_t prev = 0;

static void openPort() {
    port->PortClose();
    if (sbus) {
        port->Mode(Connector::RS232);
        port->Speed(100000);
        port->Parity(SerialBase::PARITY_E);
        port->StopBits(2);
    } else {
        port->Mode(Connector::TTL);
        port->Speed(115200);
        port->Parity(SerialBase::PARITY_N);
        port->StopBits(1);
    }
    port->CharSize(8);
    port->PortOpen();
    bytesTotal = headers = 0;
    dumpLen = 0;
    Serial.print("\n--- listening on COM-");
    Serial.print(port == &ConnectorCOM0 ? 0 : 1);
    Serial.println(sbus ? " as SBUS (RS232/inverted, 100000 8E2)"
                        : " as iBus (TTL, 115200 8N1)");
}

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) {}
    openPort();
}

void loop() {
    // keyboard commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'i') { sbus = false; openPort(); }
        if (c == 's') { sbus = true;  openPort(); }
        if (c == '0') { port = &ConnectorCOM0; openPort(); }
        if (c == '1') { port = &ConnectorCOM1; openPort(); }
    }

    int16_t c;
    while ((c = port->CharGet()) >= 0) {
        uint8_t b = (uint8_t)c;
        bytesTotal++;
        if (!sbus && prev == 0x20 && b == 0x40) headers++;
        if (sbus && b == 0x0F) headers++;
        if (dumpLen < sizeof(dump)) dump[dumpLen++] = b;
        prev = b;
    }

    static uint32_t last = 0;
    if (millis() - last >= 500) {
        last = millis();
        Serial.print("bytes=");
        Serial.print(bytesTotal);
        Serial.print(" headers=");
        Serial.print(headers);
        Serial.print("  first bytes:");
        for (uint8_t i = 0; i < dumpLen; i++) {
            Serial.print(' ');
            if (dump[i] < 0x10) Serial.print('0');
            Serial.print(dump[i], HEX);
        }
        Serial.println();
        dumpLen = 0;
    }
}
