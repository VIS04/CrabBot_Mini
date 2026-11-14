# CrabBot Mini

MakeCode extension for the **CrabBot Mini** 8-DOF quadruped robot (standalone, with built-in PCA9685 servo driver).

## Modes & Actions

### 1. Operating Modes (`CrabBotMini.setMode(...)`)
- **Stand** — Neutral 30° X-stance
- **Run** — Active command mode for movement & action blocks
- **Self Balance** — Closed-loop accelerometer pitch & roll leveling
- **Stop** — Interrupts any active motion and recovers safely to neutral stance

### 2. Run-Mode Actions (Timed)
When in **Run** mode, use these blocks (timed movements return to neutral stance when complete):
- **`CrabBotMini.walk(direction, durationMs)`** — Walk forward or backward for the specified duration
- **`CrabBotMini.turn(direction, strengthPct, durationMs)`** — Differential stride turn left or right for the specified duration
- **`CrabBotMini.spin(direction, speed, durationMs)`** — Spin in place left or right for the specified duration
- **`CrabBotMini.wave()`** — Tripod-balanced front-right leg wave
- **`CrabBotMini.dance()`** — 4-leg sine/cubic dance + body groove + celebration pose

## Hardware Pinout (Servo Headers S1–S8)

| Leg | Base (Horizontal) | Lift (Vertical) |
| :--- | :--- | :--- |
| Front Right (`FR`) | `S1` | `S2` |
| Rear Right (`RR`) | `S3` | `S4` |
| Front Left (`FL`) | `S5` | `S6` |
| Rear Left (`RL`) | `S7` | `S8` |

## License

MIT

## Supported targets

* for PXT/microbit
(The metadata above is needed for package search.)

