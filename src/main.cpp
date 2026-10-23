#include <Arduino.h>
#include <math.h>
#include "ClearCore.h"

#include "config.h"
#include "drive.h"
#include "rc_input.h"

static RcInput rc;
static Drive drive;
static bool armed = false;

static void printStatus(bool link, float left, float right) {
    Serial.print(link ? "LINK " : "NOLINK ");
    Serial.print(armed ? "ARMED " : "safe  ");
    if (drive.faulted()) Serial.print("MOTOR-FAULT ");
    Serial.print("frames=");
    Serial.print(rc.frames());
    Serial.print(" err=");
    Serial.print(rc.errors());
    Serial.print(" ch:");
    for (uint8_t i = 0; i < 8; i++) {
        Serial.print(' ');
        Serial.print(rc.us(i));
    }
    Serial.print("  L=");
    Serial.print((int)lroundf(left * 100));
    Serial.print("% R=");
    Serial.print((int)lroundf(right * 100));
    Serial.println('%');
}

void setup() {
    Serial.begin(115200);  // USB debug output
    rc.begin();
    drive.begin();
}

void loop() {
    rc.poll();

    const bool link = rc.signalOk();

    float left = 0.0f;
    float right = 0.0f;

#if MIX_DIFFERENTIAL
    const float throttle = link ? rc.axis(CH_THROTTLE, STICK_DEADBAND_US) : 0.0f;
    const float steering = link ? rc.axis(CH_STEERING, STICK_DEADBAND_US) : 0.0f;
    left  = throttle + STEERING_GAIN * steering;
    right = throttle - STEERING_GAIN * steering;
    // keep the ratio between both sides when one saturates
    const float m = fmaxf(fabsf(left), fabsf(right));
    if (m > 1.0f) {
        left /= m;
        right /= m;
    }
    const bool sticksNeutral = (throttle == 0.0f);
#else
    left  = link ? rc.axis(CH_LEFT, STICK_DEADBAND_US) : 0.0f;
    right = link ? rc.axis(CH_RIGHT, STICK_DEADBAND_US) : 0.0f;
    const bool sticksNeutral = (left == 0.0f && right == 0.0f);
#endif

    // Arming: switch on + link + sticks neutral. Losing the link or
    // flipping the switch off disarms immediately.
#if USE_ARM_SWITCH
    const bool armRequest = link && rc.us(CH_ARM) > ARM_THRESHOLD_US;
#else
    const bool armRequest = link;
#endif
    if (!armRequest) {
        armed = false;
    } else if (!armed && sticksNeutral) {
        armed = true;
    }

    if (!armed) {
        left = right = 0.0f;
    }

    drive.update(armed, left, right);
    ConnectorLed.State(armed);

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 250) {
        lastPrint = millis();
        printStatus(link, left, right);
    }
}
