# Two-Sensor Line Following Robot

An Arduino-based robot that follows a line around an elliptical track using two
reflectance sensors and two continuous-rotation servos. Steering is handled by a
state machine that tracks one edge of the line and scales its corrections to how
far off-course the robot is.

## Hardware

- Arduino (Uno or compatible)
- 2 × reflectance sensor modules (analog output), read on pins `A0` (left) and `A1` (right)
- 2 × continuous-rotation servos on pins `13` (left) and `12` (right)
- Chassis with two driven wheels
- Power supply for the servos

The two sensors are mounted at the front of the robot, side by side, facing down
at the track. A dark line runs on a light background.

## Wiring

| Component      | Arduino pin |
|----------------|-------------|
| Left sensor    | A0          |
| Right sensor   | A1          |
| Left servo     | 13          |
| Right servo    | 12          |

Each sensor's signal wire must go to the **analog output (AO)** pin on the module,
not the digital output (DO). The DO pin only reports a hard on/off value and the
onboard trimpot sets its threshold; the AO pin gives the continuous reading this
code relies on.

## How it works

Each sensor reads high (~980) over the black line and low (~110) over the white
background. A threshold of 550 classifies each sensor as on-line or off-line.

The robot follows the **left edge** of the line: the target state is the left
sensor on the line and the right sensor on white. Every loop it reads both
sensors and picks one of four states:

- **Left black, right white** — on target. Drive forward, trimming proportionally
  to stay on the edge.
- **Both white** — line has slipped off to the right. Steer right, gently.
- **Both black** — drifted right onto the line. Steer left, gently.
- **Right black, left white** — line has crossed fully over. Hard pivot left to
  recover.

Corrections in the first three states scale with how far off the line is, so a
small drift produces a small turn and a large one a bigger turn. Only the full-loss
recovery state uses a fixed hard pivot.

## Calibration and tuning

All tuning values are `#define` constants at the top of the sketch.

| Constant         | Purpose                                                        |
|------------------|----------------------------------------------------------------|
| `ON_LINE_THRESH` | Reading above this = on the black line (default 550)           |
| `BASE_SPEED`     | Forward speed when tracking                                    |
| `TURN_BIAS`      | Constant curve to match the ellipse; flip sign to curve the other way |
| `TURN_DIV`       | Larger = gentler proportional turn                             |
| `TURN_CAP`       | Ceiling on the soft-state turn amount                          |
| `PIVOT_HARD`     | Turn strength for the full-loss recovery pivot                 |
| `LEFT_STOP` / `RIGHT_STOP` | Servo stop-pulse values (µs) where each servo sits still |

Tuning guide:

- Turning too hard when a sensor fires → raise `TURN_DIV`, lower `TURN_CAP`.
- Wandering and reacting late → lower `TURN_DIV`.
- Losing the tight ends of the ellipse → raise `PIVOT_HARD`.
- Curving the wrong way around the loop → flip the sign of `TURN_BIAS`.

The `LEFT_STOP` and `RIGHT_STOP` values are specific to your servos. Find them by
sending pulse widths until each servo sits completely still, then set the constants
to those numbers.

## Running it

1. Set `LEFT_STOP` and `RIGHT_STOP` to your calibrated stop values.
2. Confirm sensor signal wires are on the AO pins.
3. Upload the sketch.
4. Bench-test with the wheels off the ground and watch the Serial Monitor: slide
   the line under each sensor and confirm the printed state and wheel speeds change
   as expected.
5. Place the robot with the left sensor on the line and the right on white, aimed
   along the line, and let it go.

The Serial output prints both sensor readings, the current state, and the two wheel
speeds each loop, which is the main tool for diagnosing misbehaviour.

## Notes

- The Serial prints slow the loop slightly; comment them out for best tracking once
  the robot is running well.
- If the robot handles one side of the loop well but struggles on the other, the
  followed edge (left) may be the harder one for that direction — mirroring the
  logic to follow the right edge can help.
