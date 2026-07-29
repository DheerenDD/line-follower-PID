# Single Servo Test

A minimal Arduino sketch for checking the behaviour of one Parallax
continuous-rotation servo at a time. Used during calibration to find each
servo's neutral (stop) pulse and to observe its direction and speed for a
given pulse width.

## Purpose

Continuous-rotation servos are commanded by a pulse width in microseconds
rather than an angle:

- `1507` µs — neutral (servo stopped)
- greater than neutral — rotates in one direction
- less than neutral — rotates in the other direction

Each servo has its own true neutral point, which is often not exactly 1500 µs.
This sketch lets you send a single fixed pulse to one servo so you can confirm
where it stops and how it responds either side of that point.

## Hardware

- Arduino Uno / Nano
- Parallax Board of Education (BOE) Shield
- One Parallax continuous-rotation servo
- Battery pack powering the servo through the shield

## Wiring

Connect the servo under test to the pin defined by `TEST_PIN` (default 13).
Power the servo from the battery pack through the shield, not from USB alone —
USB cannot supply a stable rail for a moving servo.

## How to Use

1. Set `TEST_PIN` to the pin of the servo you want to test.
2. Set the value inside `writeMicroseconds(...)` to the pulse you want to try.
3. Upload the sketch and watch the servo.

Suggested sequence for finding a servo's neutral:

- Start at `1507`. If the wheel creeps, nudge the value up or down a few
  microseconds and re-upload until it sits completely still. That pulse is the
  servo's true stop value.
- Try values above and below neutral (e.g. 1450, 1400, 1550, 1600) to confirm
  the direction and speed of rotation each way.

To test the other servo, change `TEST_PIN` and repeat.

## Notes

- The command is placed in `setup()` and held, so the servo runs once and
  stays at the commanded pulse; there is no loop.
- Lift the wheels off the ground while testing so the servo can turn freely.
- Record each servo's neutral separately — the two are not identical, so the
  main robot code uses a separate stop value for each (e.g. `LEFT_STOP` and
  `RIGHT_STOP`).
