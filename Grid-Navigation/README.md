# Grid Navigation Robot

Part 3 of the line-following robot project. The robot navigates a 7x7
intersection grid (coordinates 0-6 on each axis), driving between a set of
operator-defined waypoints in order, then performs a finishing spin.

Unlike Parts 1 and 2, which are purely reactive line followers, this stage is
**deliberative**: the robot tracks its own position and heading on the grid and
executes a planned route. The line-following behaviour becomes a subroutine within
a larger navigation scheme.

## Hardware

- Arduino Nano on a Parallax Board of Education
- 2 x FlyingFish MH-series IR reflectance sensors (analog output), on `A0` (left)
  and `A1` (right)
- 2 x continuous-rotation servos, on pins `13` (left) and `12` (right)

| Component | Arduino pin |
|-----------|-------------|
| Left sensor | A0 |
| Right sensor | A1 |
| Left servo | 13 |
| Right servo | 12 |

Sensor signal wires must connect to the analog output (AO) pin of each module.
The two sensors are spaced roughly 2 cm apart; the grid lines are roughly 1 cm
wide, so the travel line fits in the gap between the sensors.

## How it works

The robot maintains three pieces of state: its current intersection `(x, y)`, its
heading (North/East/South/West), and its progress through the waypoint list. Its
behaviour is built from two primitives.

**Driving one cell.** The robot drives forward along the grid line until an
intersection is detected, then advances a short fixed nudge so its turning axis
sits on the intersection, and stops. Because the line runs between the sensors and
alignment is handled by placement, no active steering correction is applied while
driving; the robot drives straight through crossings so it cannot deviate at an
intersection.

**Turning.** Turns are timed open-loop pivots from rest. The robot stops, waits for
its forward momentum to die, then pivots at a fixed speed for a calibrated time.
Right and left turns use separately calibrated durations, because
continuous-rotation servos rarely pivot at exactly the same rate in both
directions.

**Intersection detection.** An intersection is detected on the rising edge of
*either* sensor going black. This covers both interior crossings, where a
perpendicular line passes under both sensors at once, and boundary crossings on the
outer edge of the grid, where only the inner sensor passes over the line. A blanking
window prevents a single thick crossing from being counted more than once.

**Path planning.** Movement between waypoints is Manhattan-style: the robot travels
the full X distance first, then the full Y distance, turning to face each direction
in turn.

## Configuration

Set these before each run, at the top of the sketch:

```cpp
#define START_X       0
#define START_Y       0
#define START_HEADING 0     // 0=North, 1=East, 2=South, 3=West

int waypoints[][2] = {
  {3, 2},
  {5, 5},
  {1, 4}
};
```

`numWaypoints` is computed automatically, so adding or removing a waypoint is a
single-line edit.

## Tuning parameters

| Constant | Purpose |
|----------|---------|
| `ON_LINE_THRESH` | Reading above this = sensor over a black line |
| `BASE_SPEED` | Forward driving speed |
| `PIVOT_SPEED` | Wheel speed during a pivot turn |
| `TURN_TIME_RIGHT_MS` | Calibrated duration of a 90-degree right turn |
| `TURN_TIME_LEFT_MS` | Calibrated duration of a 90-degree left turn |
| `TURN_SETTLE_MS` | Stop time before a pivot, to kill forward momentum |
| `NUDGE_TIME_MS` | Forward drive after detecting an intersection, to place the turning axis on it |
| `CROSS_BLANK_MS` | Minimum time between counted crossings (debounce) |
| `SPIN_TIME_MS` | Duration of the finishing spin |

### Calibration notes

The three values that most affect navigation:

- **Turn times.** Calibrate each direction separately by pivoting the robot on the
  spot and adjusting until it turns exactly 90 degrees. Turn angle depends on both
  `PIVOT_SPEED` and the turn time, so fix the pivot speed first, then tune the time.
  Always calibrate from rest, since a turn started while still moving over-rotates.
- **Nudge.** After the sensors detect an intersection, the robot's turning axis is
  still short of it. `NUDGE_TIME_MS` drives forward to place the axis on the
  intersection so that a pivot leaves the new line centred between the sensors. If
  the line ends up behind the sensors after a turn, increase it; if ahead, decrease
  it. Note that this distance scales with `BASE_SPEED` -- lowering the speed
  requires increasing the nudge time to travel the same distance.
- **Crossing debounce.** If intersections are double-counted, raise `CROSS_BLANK_MS`;
  if closely-spaced intersections are missed, lower it.

## Serial output

The Serial Monitor (9600 baud) reports three things:

1. The start position and heading, and the given waypoint list.
2. The full expanded route -- every intersection the robot will pass through,
   derived from the same X-then-Y planning the movement code uses, so the
   prediction always matches the actual path.
3. A live update at every intersection crossed and every turn taken, so the run can
   be followed step by step.

Because the grid has no distinguishing marks, the robot cannot detect a miscount.
During testing, compare the live coordinates against the robot's true position -- a
divergence is the signal that a crossing was mis-detected and a timing parameter
needs adjustment.

## Setup and running

1. Set the waypoints, start position and heading.
2. Calibrate the turn times, nudge and crossing debounce as above.
3. Confirm sensor wires are on the AO pins.
4. Place the robot on the start intersection with the travel line centred between
   the sensors, aimed along the start heading.
5. Upload and open the Serial Monitor.

Test incrementally: confirm a single cell drive stops centred on the next
intersection, confirm each turn is 90 degrees, then run a single waypoint, then the
full list.

## Notes

- The route runs once inside `setup()` and then halts, so it cannot restart from an
  incorrect position. Reset the board to run again.
- The robot relies entirely on dead reckoning between intersections; there is no
  external localisation. Reliable intersection counting and consistent turns are
  therefore essential, and are where most of the tuning effort is spent.
