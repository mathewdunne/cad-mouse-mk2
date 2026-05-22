# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository layout

This repo holds the full design of the CAD Mouse MK2, a DIY 6DoF space-mouse. Most directories are hardware artifacts; the only code is the firmware.

- `firmware/` — Arduino/PlatformIO firmware for the Seeed XIAO RP2040 (the focus of this file).
- `enclosure/` — Fusion sources, STEP, and STL files for the 3D-printed enclosure and spring.
- `pcbs/` — KiCad sources (`src/`), fabrication outputs (`fab/`), and PCB BOM.
- `bom/` — Top-level BOM CSV for the whole build.
- `images/` — Marketing/thumbnail images for the README.
- `platformio.ini` — PlatformIO project config; lives at repo root but points `src_dir`/`include_dir`/`lib_dir`/`test_dir` into `firmware/`.

## Build / flash / develop (firmware)

PlatformIO is the build system. The project root contains `platformio.ini` and is the directory PlatformIO must be invoked from. There is one environment: `seeed_xiao_rp2040` (Earle Philhower RP2040 core, TinyUSB).

- Build: `pio run`
- Build + upload to the board: `pio run -t upload`
- Open serial monitor (telemetry stream): `pio device monitor -b 115200`
- Clean: `pio run -t clean`

No tests exist; `firmware/test/` only contains the default PlatformIO placeholder README.

USB identity is configured in `platformio.ini` via `board_build.arduino.earlephilhower.usb_vid/pid/manufacturer/product`. The current VID/PID (`0x256F`/`0xC635`) and manufacturer string (`3Dconnexion`) make the device impersonate a SpaceMouse so commercial 3Dconnexion drivers will pick it up. Changing these breaks driver recognition; see the video timestamp linked from [firmware/README.md](firmware/README.md).

## Firmware architecture

`firmware/src/main.cpp` is tiny by design — it constructs six singleton controllers, calls `begin()` on each in `setup()`, and in `loop()` only does two things: pump TinyUSB (`hidController.task()`) and tick the state machine.

All actual behavior lives in two layers:

### Layer 1: Controllers (`firmware/src/controllers/`)

Each controller owns one hardware subsystem or pipeline stage. They are declared as `extern` globals in [Controllers.h](firmware/include/Controllers.h) and defined in [main.cpp](firmware/src/main.cpp). States call them by name.

- `SensorController` — Owns the three Infineon TLx493D magnetic sensors over I²C. Because all three share the same default I²C address, `begin()` brings them up one at a time via load-switch pins (`PIN_MAG{1,2,3}_LS`), reassigning each to a unique address (`A2`, `A1`, `A0`) before powering the next. Also runs the calibration pass that averages `ZERO_SAMPLES` readings into `baseline_[9]`. `readRaw(out[9])` packs the 9 axes in order `[mag1xyz, mag2xyz, mag3xyz]` — every downstream consumer assumes that layout.
- `MotionController` — Pure-math stage: takes the 9 raw values + the baseline + `dt`, produces a 6-axis output `[Tx,Ty,Tz,Rx,Ry,Rz]`. See "Motion math" below.
- `InputController` — Debounces the two physical buttons via AceButton, exposes `buttonBits()`, and detects a 3-second both-buttons hold to request recalibration (`takeCalibrationRequest()`). It also exposes `takeActivity()` (consumed-on-read) which the idle/sleep transitions use to track wake-from-sleep and reset the inactivity timer.
- `HIDController` — TinyUSB HID with a hand-rolled report descriptor that matches the SpaceMouse format: report ID 1 (six int16 axes, range ±350) and report ID 3 (button bits). Suppresses redundant reports — only sends when axes or buttons actually change.
- `LEDController` — NeoPixel ring (8 LEDs) with a load-switch (`PIN_LED_LS`) so the ring can be powered off entirely in sleep. Three modes: `Solid`, `Spinner` (used during calibration), `Off`.
- `TelemetryController` — When `Config::ENABLE_TELEMETRY` is true, prints axes/buttons/HID-sent flag to `Serial` every 5 ticks in the [Teleplot](https://github.com/nesnes/teleplot) `>name:value` format.

### Layer 2: State machine (`firmware/src/states/`)

[StateMachine.h](firmware/include/StateMachine.h) holds static instances of the three states as members, and a single `stateMachine` global. Each `State` implements `enter()`/`update()`/`exit()`. `main.cpp` transitions into `CalibratingState` after `setup()`.

Transitions:
- `Calibrating → Idle` once `sensorController.calibrationDone()` flips true (after `Config::ZERO_SAMPLES` averaged reads).
- `Idle → Calibrating` when `inputController.takeCalibrationRequest()` returns true (both buttons held 3 s).
- `Idle → Sleep` after `Config::IDLE_SLEEP_TIMEOUT_MS` with no motion or button activity. Motion activity is detected by `MotionController::hasMotionActivity()` (any axis non-zero after dead-zone), button activity via `InputController::takeActivity()`.
- `Sleep → Idle` on any button activity.

`IdleState::update()` is where the per-loop pipeline runs: `sensorController.readRaw → motionController.compute → hidController.sendReports → telemetryController.publish`.

### Motion math (`MotionController::compute`)

The three sensors sit on a triangle: `mag1` bottom, `mag2` top-left, `mag3` top-right. After baseline subtraction:

```
Tx = (mag1x + mag2x + mag3x) / 3       // averaged translations
Ty = (mag1y + mag2y + mag3y) / 3
Tz = (mag1z + mag2z + mag3z) / 3
Rx = sqrt(3) * (mag2z + mag3z - 2*mag1z) / 3   // pitch: top pair vs bottom
Ry = mag3z - mag2z                              // roll: right vs left
Rz = sum_i (posXi*magYi - posYi*magXi)         // yaw: triangle-position-weighted swirl
```

Each axis is then multiplied by a per-axis sign and gain (`Config::SIGN_AXIS`, `Config::GAIN_T`/`GAIN_R`), passed through a dead-zone (`DEAD_T`/`DEAD_R`), a one-pole low-pass (time constant `SMOOTH_TAU_S`), clamped to `±AXIS_LIMIT` (350, must match the HID descriptor's LOGICAL_MIN/MAX), and finally hard-zeroed within the dead zone.

Known limitations documented in [firmware/README.md](firmware/README.md): the axes are computed independently (bleed between them) and the sensors are assumed linear (they aren't). The whole motion-processing block is the intended place to experiment.

### Tuning

[Config.h](firmware/include/Config.h) is the single source of truth for pin assignments, gains, dead zones, smoothing, LED colors, calibration sample count, and the sleep timeout. The HID report descriptor's logical range (`-350`/`+350`) in [HIDController.cpp](firmware/src/controllers/HIDController.cpp) is hard-coded — if `Config::AXIS_LIMIT` changes, that descriptor must change too.
