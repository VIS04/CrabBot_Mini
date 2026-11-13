# CrabBot Mini

MakeCode extension for the **CrabBot Mini** 8-DOF quadruped robot (standalone, with built-in PCA9685 servo driver).

## Features & Selectable Modes

Use `CrabBotMini.setMode(...)` (or `CrabBotMini.runModeFor(...)`) to switch cleanly between modes without servo or I2C collisions:

- **Stand** — Neutral 30° X-stance
- **Walk Forward / Walk Backward** — Smooth 4-phase crawl gait
- **Turn Left / Turn Right** — Differential stride turning
- **Spin Left / Spin Right** — Turn in place
- **Wave** — Tripod-balanced front-right leg wave
- **Dance** — 4-leg sine/cubic dance + body groove + celebration pose
- **Self Balance** — Closed-loop accelerometer pitch & roll leveling
- **Stop** — Interrupts any active motion and recovers safely to neutral stance

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

