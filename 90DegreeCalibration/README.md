# 90-Degree Turn Test

A minimal Arduino sketch for calibrating and verifying the robot's 90-degree
pivot, one direction at a time. Used to tune the timed open-loop turn durations
that the grid-navigation code relies on.

## Purpose

The robot turns by pivoting in place: one wheel drives forward while the other
drives backward, rotating the robot about its center. Because the turn is
open-loop (timed, not sensor-guided), each direction needs its own calibrated
duration. This sketch performs a single pivot so the turn angle can be checked
and the timing adjusted until it lands squarely on 90 degrees.

The two servos are not perfectly matched, so the right and left turns use
separate durations (`TURN_TIME_MS`).

## Hardware

- Arduino Uno / Nano
- Parallax Board of Education (BOE) Shield
- Two Parallax continuous-rotation servos
- Battery pack powering the servos through the shield

## Wiring

- Left servo on `LEFT_PIN` (12)
- Right servo on `RIGHT_PIN` (13)

Each servo uses its own calibrated neutral pulse (`LEFT_STOP`, `RIGHT_STOP`).
Power the servos from the battery pack through the shield, not from USB alone.

## Sequence

The sketch runs once on power-up or reset:

1. Rest at neutral for 2 seconds.
2. Pivot 90 degrees for `TURN_TIME_MS`.
3. Return to rest.

There is no loop; press reset to run it again.

## How to Use

**Right turn (default):**
- `TURN_TIME_MS` = 1925
- Left wheel forward, right wheel backward.

**Left turn:**
- Change `TURN_TIME_MS` to 1950.
- Flip both wheel signs so the robot pivots the other way:
  left wheel backward, right wheel forward.

**Calibrating:**
- Run the pivot and check where the robot ends up.
- If it under-rotates (stops short of 90 degrees), increase that direction's
  `TURN_TIME_MS`.
- If it over-rotates (past 90 degrees), decrease it.
- Adjust and re-run until the turn lands square. Record the final value for
  that direction.

**If it turns the wrong way:** flip the sign on one wheel's command.

## Notes

- Calibrate each direction separately and keep the two durations independent;
  the right and left turns will generally differ.
- Keep the pivot speed low (`PIVOT_SPEED` 25) for a controlled, repeatable turn.
- Run the test on the actual surface the robot will drive on — traction affects
  the turn timing.
