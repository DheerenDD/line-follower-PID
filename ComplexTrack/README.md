# Complex Track Line Follower (Centre-Tracking)

Part 2 of the line-following robot project. This version follows an arbitrary
closed track with alternating left- and right-hand curves, tight hairpins and
varying curvature — unlike Part 1, which followed a simple ellipse.

The robot straddles the line: the line runs in the gap between the two sensors,
and the controller works to keep it there.

## What changed from the elliptical version

| | Elliptical (Part 1) | Complex track (Part 2) |
|---|---|---|
| Tracking style | Rides one edge of the line | Straddles the line, keeps it centred |
| Target state | Left sensor on line, right on white | Both sensors on white, line in the gap |
| Turn bias | Constant `TURN_BIAS` for the sustained curve | None — curvature reverses |
| Recovery | Fixed-direction hard pivot | Direction-aware, uses `lastDir` |
| Turn authority | Could exceed base speed (pivoting) | Capped below base speed (smooth arcs) |

The elliptical version assumed the robot turns one direction for the whole lap,
which let it use a constant feed-forward bias. On a track whose curvature
reverses, that bias fights the controller for half the lap, so it was removed.

## Hardware

- Arduino (Uno or compatible)
- 2 × reflectance sensor modules (analog output), on `A0` (left) and `A1` (right)
- 2 × continuous-rotation servos, on pins `13` (left) and `12` (right)
- Two-wheel differential drive chassis

| Component | Arduino pin |
|-----------|-------------|
| Left sensor | A0 |
| Right sensor | A1 |
| Left servo | 13 |
| Right servo | 12 |

Each sensor's signal wire must connect to the module's **analog output (AO)** pin,
not the digital output (DO). The DO pin reports only a thresholded on/off value
set by the onboard trimpot and cannot provide the continuous reading the
proportional control needs.

## Sensor mounting

The two sensors sit side by side at the front of the robot, spaced so the track
line fits **between** them. When the robot is correctly on course, both sensors
read white and the line passes through the gap untouched.

## How it works

Each sensor reads high (~930) over the line and low (~85) over the background.
A threshold of 550 classifies each as on-line or off-line. Four states result:

| Sensors | State | Action |
|---------|-------|--------|
| Both white | `CENTER` | On target — drive straight, no correction |
| Left black only | `L-hit` | Drifted right — steer left, proportionally |
| Right black only | `R-hit` | Drifted left — steer right, proportionally |
| Both black | `CORNER` | On the line or at a sharp corner — turn toward `lastDir` |

The robot applies **no correction at all** while centred. It reacts only when a
sensor actually touches the line, and the size of that reaction scales with how
far onto the line the sensor has gone — a slight touch gives a slight turn.

`lastDir` records which side the line was last seen drifting toward. The `CORNER`
state uses it to decide which way to turn when both sensors are black and the
difference alone gives no direction.

## Tuning parameters

| Constant | Purpose |
|----------|---------|
| `ON_LINE_THRESH` | Reading above this = sensor is over the line (default 550) |
| `BASE_SPEED` | Forward speed when centred |
| `TURN_DIV` | Divisor for proportional turn — **larger = gentler** |
| `TURN_CAP` | Ceiling on the tracking turn — must stay well below `BASE_SPEED` |
| `CORNER_TURN` | Turn strength for the `CORNER` state (sharp corners) |
| `LEFT_STOP` / `RIGHT_STOP` | Servo stop-pulse values (µs) where each servo sits still |

### The key tuning rule

`TURN_CAP` must be meaningfully **less than** `BASE_SPEED`. If a turn magnitude
exceeds the base speed, one wheel goes negative and the robot pivots in place
instead of arcing smoothly along the line. Keeping the cap below the base speed
guarantees both wheels stay driving forward, producing a smooth curve.

### Symptom guide

| Symptom | Fix |
|---------|-----|
| Twitchy / oversteers on gentle curves | Raise `TURN_DIV` |
| Drifts out of the gap before reacting | Lower `TURN_DIV` |
| Pivots instead of arcing | Lower `TURN_CAP`, or raise `BASE_SPEED` |
| Can't get around hairpins | Raise `CORNER_TURN` (leave `TURN_CAP` alone) |
| Steers the wrong way when a sensor is hit | Swap the `A0` / `A1` pin defines |

Tune gentle tracking (`TURN_DIV`, `TURN_CAP`) and sharp cornering (`CORNER_TURN`)
independently — that is the point of separating the `CORNER` state from the
normal tracking states.

## Setup and running

1. Set `LEFT_STOP` and `RIGHT_STOP` to your servos' calibrated stop values. Find
   them by sending pulse widths until each servo sits completely still.
2. Confirm both sensor signal wires are on the AO pins.
3. Upload the sketch and open the Serial Monitor at 9600 baud.
4. Bench-test with the wheels off the ground:
   - Line in the gap → `CENTER`, both wheels equal.
   - Slide so the line touches the left sensor → `L-hit`, left wheel slows.
   - Slide the other way → `R-hit`, right wheel slows.
   - If these are reversed, swap the `A0` / `A1` defines.
5. Place the robot with the line in the gap, aimed along the track, and run it.

Test in sections rather than on the whole loop. Start on the gentle sweeps to
confirm smooth tracking, then hand-place the robot just before each tight corner
to isolate the hard cases.

## Notes

- The Serial prints slow the control loop slightly. Comment them out once tuning
  is complete for best tracking performance.
- If a corner's radius is tighter than roughly the robot's wheelbase, it cannot
  be followed as a curve at any tuning — the robot must pivot through it. This is
  a geometric limit, not a control problem.
