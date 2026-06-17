You can customize several variables to tune gains, smoothing, and deadzones for all six axes.
Most of these settings are defined in [`Config.h`](include/Config.h) and are the main place to adjust the overall feel of the device.

```cpp
// Gains and sign fixes
const float GAIN_T[3] = {28.0, 28.0, 24.0};
const float GAIN_R[3] = {18.0, 18.0, 20.0};
const int SIGN_AXIS[6] = {+1, -1, +1, -1, -1, -1};

// Dead zones
const float DEAD_T = 16.0;
const float DEAD_R = 20.0;

// Smoothing (per-axis one-pole time constant, [Tx,Ty,Tz,Rx,Ry,Rz])
const float SMOOTH_TAU_S[6] = {0.132, 0.128, 0.073, 0.149, 0.02, 0.034};
```

The `SMOOTH_TAU_S` values are device-specific feel parameters: longer means more
smoothing (quieter but laggier). The calibration workflow prints a data-driven
`SMOOTH_TAU_S[6]` line in its report — sized so every axis settles to a uniform
residual-jitter floor — that you can copy in as a starting point and tune by ear.

⚠️ Refer to the video at [6:23](https://youtu.be/62xlzGs8LXA?si=ld2shDCaTxOLIGB8&t=383) for a demo of driver support. Related settings can be found commented in[`platformio.ini`](../platformio.ini).

⚠️ As mentioned in the video, the motion processing still needs work and may eventually be replaced entirely. This is beyond me for now, so contributions and improvements are welcome.

There is bleed between the axis due to the way each of them is calculated independently. This implementation also assumes the readings from the sensors are linear, which is not true.

If you want to experiment with different motion processing approaches, you can modify the [`MotionController`](src/controllers/MotionController.cpp). The current implementation is as follows:

The firmware now also supports an optional calibrated 9D-to-6D decoder. Run the guided Python workflow in [`calibration/`](calibration/) to generate a local `include/CalibrationData.h`; if that file is absent, the firmware falls back to the hand-derived mapping below.

**Sensor layout:**
- `mag1` = bottom
- `mag2` = top left
- `mag3` = top right

```
Tx = (mag1x + mag2x + mag3x) / 3
Ty = (mag1y + mag2y + mag3y) / 3
Tz = (mag1z + mag2z + mag3z) / 3

Rx = sqrt(3) * (mag2z + mag3z - 2 * mag1z) / 3
Ry = mag3z - mag2z
Rz = sum_i (posXi * magYi - posYi * magXi)
```
- `Tx`, `Ty`, `Tz`: average of all three sensors
- `Ry`: left-right z difference across the top edge
- `Rx`: top pair versus bottom, scaled for the triangle geometry
- `Rz`: twist estimate from the x/y sensor positions

**Output conditioning (both decoders):** each of the six axes is then shaped
through a soft dead zone, smoothed by the per-axis low-pass, clamped to
`±AXIS_LIMIT`, and snapped to zero below a sub-count settle threshold. The soft
dead zone maps inputs inside the dead band to zero and rescales the surviving
range back to full scale, so motion ramps in smoothly from the threshold instead
of jumping ("popping") to the dead-zone width. Dead-zone widths come from
`CalibrationData::DEADZONE` when a calibration header is present, otherwise from
`DEAD_T`/`DEAD_R`.
