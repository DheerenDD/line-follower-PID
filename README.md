# Elliptical Path Following (PID)

Arduino sketch for the elliptical-path challenge: a two-wheeled robot follows a
black elliptical line on a white surface using two IR reflectance sensors and a
PID-trimmed edge-following controller.

## Approach

The robot edge-follows the line: one sensor rides the line edge while the other
stays on white. Two ideas are combined:

- **Constant curve bias (`TURN_BIAS`)** — holds the base elliptical arc so the
  robot keeps curving along the path.
- **PID correction** — trims deviation from the line edge on top of that arc.

The PID acts only on the edge-tracking error (how far the active sensor is onto
the line), not on the whole path, so the ellipse is preserved while small
deviations are corrected smoothly.

A four-state controller handles every sensor combination:

| Sensors | State | Action |
|---|---|---|
| Left black, right white | TRACK | PID trim, line toward the left |
| Right black, left white | R-TRACK | PID trim, mirrored |
| Both black | BOTH | On the line; ease forward with base bias |
| Both white | RECOVER | Line lost; reset PID and hard-pivot toward last-seen side |

## Hardware

- Arduino Uno / Nano
- Parallax Board of Education (BOE) Shield
- Two Parallax continuous-rotation servos (left on pin 13, right on pin 12)
- Two FlyingFish MH-series IR reflectance sensors on analog outputs (A0, A1)
- Battery pack powering the servos through the shield

## Key Parameters

- `LEFT_STOP`, `RIGHT_STOP` — calibrated neutral pulse for each servo.
- `ON_LINE_THRESH` — analog reading above which a sensor is over black.
- `BASE_SPEED` — forward speed.
- `TURN_BIAS` — constant bias that sets the base elliptical curve.
- `KP`, `KI`, `KD` — PID gains for the edge-tracking correction.
- `TURN_CAP` — limit on the PID steering output.
- `I_CAP` — anti-windup clamp on the integral term.
- `PIVOT_HARD` — pivot strength used to recover when the line is lost.

## Tuning the PID

Tune on the actual track, in this order:

1. Set `KI` and `KD` to 0. Raise `KP` until the robot tracks the line but
   begins to oscillate.
2. Add `KD` to damp the oscillation.
3. Add a small `KI` only if a persistent offset remains; otherwise leave it 0.

Retune if the surface, lighting, or speed changes.

## Design Notes

- **Derivative guard:** the D term uses a measured timestep `dt` with a guard
  (`if (dt <= 0) dt = 0.001`) so it cannot spike when the loop runs very fast.
- **Anti-windup:** the integral is clamped to `I_CAP` and reset whenever the
  line is lost, so stale error does not accumulate.
- **Mirrored servos:** the two servos face opposite ways, so the left command
  subtracts speed and the right adds it; equal speeds then drive straight.
- **Stable ADC reads:** each sensor is read with a dummy read plus a short
  settle delay before the value is taken.
- **Throttled Serial:** debug output is limited to one line every
  `PRINT_INTERVAL_MS` so printing does not slow the control loop.

## Serial Output

At 9600 baud, the sketch prints the sensor readings, current state, last-seen
direction, and the two wheel speeds, for tuning and diagnosis.
