# Motion Calibration

This folder contains a guided one-time calibration tool for the CAD Mouse MK2
motion decoder. It learns a `6x9` matrix that maps the nine raw magnetometer
channels into six SpaceMouse HID axes.

## Install

From the repository root:

```sh
python -m pip install -r firmware/calibration/requirements.txt
```

## Firmware

Build and upload the normal firmware first:

```sh
pio run -t upload
```

The firmware exposes a small serial command mode at `115200` baud:

```text
CAL?
CAL RAW 1
CAL RAW 0
```

When raw mode is enabled, processed telemetry is suppressed and the firmware
streams lines like:

```text
CALRAW,<millis>,mag1x,mag1y,mag1z,mag2x,mag2y,mag2z,mag3x,mag3y,mag3z
```

## Run Calibration

Find the serial port for the XIAO RP2040, then run:

```sh
python firmware/calibration/calibrate_motion.py --port COM12
```

The script will prompt for:

```text
rest
+Tx, -Tx
+Ty, -Ty
+Tz, -Tz
+Rx, -Rx
+Ry, -Ry
+Rz, -Rz
```

For each prompt, move and hold the knob in the requested direction with a
normal comfortable amount of force, then press Enter. The script discards the
settle period and records the hold window.

Default capture settings:

```text
--repeats 3
--rest-seconds 5
--hold-seconds 2.5
--settle-seconds 0.75
--target-output 300
--ridge 0.03
```

## Outputs

The tool writes raw logs and diagnostics under:

```text
firmware/calibration/out/
```

If validation succeeds, it generates:

```text
firmware/include/CalibrationData.h
```

That header is specific to your physical build and is ignored by git. Rebuild
and upload the firmware after generating it:

```sh
pio run -t upload
```

If the report says rank is below 6, the captured motions are not sufficiently
independent for a calibrated decoder. Repeat calibration with cleaner, more
separated motions.
