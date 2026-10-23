# ClearCore RC car (PlatformIO)

Reads an SBUS or iBus receiver on a ClearCore COM port and drives two motors
with differential (tank) mixing, an arm switch and failsafe.

## 1. Toolchain setup

1. Windows only: install the ClearCore USB driver
   (`ClearCore_Port_Driver_Install.zip` from teknic.com, step 1 of Teknic's
   manual install guide). Linux/macOS use the standard CDC driver.
2. Install VS Code + the PlatformIO extension.
3. Open this folder. The first build downloads the community
   `platform-atmelsam` fork (pinned in `platformio.ini`), the ARM toolchain and
   the ClearCore Arduino wrapper incl. prebuilt `libClearCore`/`LwIP`.
4. Build, then upload over USB (`pio run -t upload`). If the port isn't found
   automatically, add `upload_port = COM11` (or `/dev/ttyACM0`) to the env.
5. `pio device monitor` shows link state, channel values and motor commands.

You can use the full ClearCore API (`#include "ClearCore.h"`, `ConnectorM0`,
`ConnectorCOM0`, `MotorMgr` …) alongside the Arduino API, exactly as in the
Arduino IDE.

## 2. Using the latest official Teknic library (optional)

The fork bundles wrapper 1.1.x; Teknic's current release is newer. To use it:

1. Download the release zip listed in Teknic's
   `package_clearcore_index.json` (e.g. `ClearCore-1.7.4.zip`) and unzip it,
   e.g. to `C:/dev/clearcore/ClearCore-1.7.4`. That folder should contain
   `cores/`, `variants/`, `Teknic/` and `libraries/`.
2. Copy `tools/package.json` into that folder (adjust `version` if needed).
3. In `platformio.ini`, uncomment `[env:clearcore_official]`, fix the path, and
   build with `pio run -e clearcore_official`.

This combination is not tested - if the build script trips over a changed path
in the newer release, fall back to the default env.

## 3. Receiver wiring

| | SBUS | iBus |
|---|---|---|
| Format | 100000 baud, 8E2, inverted | 115200 baud, 8N1 |
| ClearCore COM mode | `RS232` (ClearCore inverts the signal, no external inverter needed) | `TTL` |
| `config.h` | `RC_PROTOCOL_SBUS` | `RC_PROTOCOL_IBUS` |

Connect receiver signal → COM-0 RX and GND → GND (pinout of the RJ45 COM
connector: see the ClearCore user manual). Power the receiver from a 5 V
source it accepts.

Most receivers output 3.3 V logic. Check in the serial monitor that `frames`
counts up and `err` stays near zero; if not, the ClearCore input may not see
3.3 V reliably and a small level shifter/buffer is needed.

For **iBus** there is no failsafe flag in the protocol: set failsafe on the
transmitter (throttle neutral, arm switch off) or make the receiver stop
output on signal loss. SBUS failsafe is detected automatically.

## 4. Motors

**`MOTOR_TYPE_STEP_DIR` (default)** - ClearPath-SD (or any step/dir drive) on
M-0 (left) and M-1 (right), velocity mode via `MoveVelocity()`.
In ClearPath MSP: step & direction mode, and set `MAX_VELOCITY_SPS` /
`MAX_ACCEL_SPS2` to match your steps/rev and the speed you want.
HLFB is set to "ASG with measured torque" style (bipolar PWM, 482 Hz) - adjust
in `drive.cpp` if you configured HLFB differently.

**`MOTOR_TYPE_PWM_DIR`** - external brushed-motor driver: PWM on IO-0/IO-1,
direction on IO-2/IO-3. ClearCore IO outputs switch the *supply voltage*
(typically 24 V), so use a driver with 24 V-tolerant inputs, or add a divider /
optocoupler / level shifter. Check the ClearCore manual for the IO PWM frequency
and make sure your driver accepts it.

## 5. Behaviour

- Arms only when: link OK, arm switch (CH5) high, throttle centered.
- Link lost / receiver failsafe / switch off → motors ramp to 0, then disable.
- Motor fault (step/dir): the faulted motor's enable is cycled to clear the
  ClearPath shutdown, the other motor brakes to 0, and driving resumes once
  HLFB is back. After `FAULT_MAX_RETRIES` failed attempts it gives up until
  you disarm and re-arm. Timings are in `config.h`.
- Onboard user LED = armed.
- Throttle stick must be self-centering for forward/reverse (or use a
  spring-loaded gimbal / set `CH_THROTTLE` to another channel).

Test with the wheels off the ground first.
