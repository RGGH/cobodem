# cobodem

A fake myCobot 280 TCP server, written in Rust — a drop-in stand-in for the real robot's `Server_280.py` during client development.

## Why

Developing a client for the myCobot 280 normally means working against `Server_280.py` running on the robot's Raspberry Pi over the network — or against the physical arm itself. That's a problem when you're:

- writing or debugging framing/parsing logic and don't want a serial port or a robot on the desk to do it
- iterating on reconnect and timeout handling, which is hard to trigger reliably against real hardware
- working somewhere without access to the robot at all (a different machine, CI, a plane)
- worried about sending a malformed command to real motors while your client is still buggy

`cobmock` removes the robot from the loop. It speaks the exact same wire protocol as `Server_280.py`, so a client written against it should work unmodified against the real Pi — just point it at `127.0.0.1:9000` instead of the robot's IP.

## Features

- **Protocol-accurate framing** — implements the real `FE FE LEN CMD DATA FA` frame shape used by `pymycobot`, including the same big-endian i16 encoding for angles (`degrees * 100`) and coordinates (`value * 10`).
- **Stateful, not just an echo server** — tracks joint angles, speed, power, and pause state per server instance, so `SEND_ANGLE(S)` followed by `GET_ANGLES` reflects the change you just made, and a get/compare loop sees consistent, moving data instead of static stub values.
- **Derived (fake) coordinates** — `GET_COORDS` isn't real forward kinematics, but it's computed from the current joint angles so the numbers move plausibly and consistently as you drive the joints, rather than staying frozen.
- **Multi-client, thread-per-connection** — each connection is handled on its own thread against shared, mutex-guarded robot state, so you can exercise concurrent-client behavior if your setup needs it.
- **Tolerant frame scanning** — resyncs on the `FE FE` header even if stray bytes show up first, mirroring how `pymycobot`'s own receiver behaves.
- **Graceful handling of unknown commands** — anything not explicitly implemented gets logged server-side and answered with a generic ack, instead of dropping the connection, so an unfinished client doesn't have to crash the mock while you're still adding features.
- **Configurable port** — pass a port as the first CLI arg (`cargo run -- 9001`); defaults to `9000`.

## Supported commands

| Command | Code | Notes |
|---|---|---|
| `POWER_ON` / `POWER_OFF` | `0x10` / `0x11` | sets power state, acks |
| `IS_POWER_ON` | `0x12` | returns current power state |
| `RELEASE_ALL_SERVOS` | `0x13` | acks |
| `GET_ANGLES` | `0x20` | returns all 6 joint angles |
| `SEND_ANGLE` | `0x21` | moves one joint (`joint_id, angle, speed`) |
| `SEND_ANGLES` | `0x22` | moves all 6 joints + speed at once |
| `GET_COORDS` | `0x23` | returns derived (non-kinematic) coordinates |
| `SEND_COORD` / `SEND_COORDS` | `0x24` / `0x25` | accepted and acked; not reflected in state |
| `PAUSE` / `RESUME` | `0x26` / `0x28` | sets pause state, acks |
| `IS_PAUSED` | `0x27` | returns current pause state |
| `STOP` | `0x29` | acks |
| `IS_IN_POSITION` | `0x2A` | always reports in-position |
| `IS_MOVING` | `0x2B` | always reports idle |
| `GET_SPEED` / `SET_SPEED` | `0x40` / `0x41` | gets/sets movement speed |
| *anything else* | — | logged as unhandled, answered with a generic ack |

Command set and encoding are reverse-derived from `pymycobot/common.py`'s `ProtocolCode` and `DataProcessor`. Extend the `cmd` module and the `match` in `handle_client` as your client starts exercising more of the real protocol.

## Getting started

```bash
cargo run
# mycobot_mock listening on 0.0.0.0:9000
```

Point your client at `127.0.0.1:9000` instead of the robot's IP, and drive it exactly as you would the real arm.

## Testing

A Python test client is included (`main.py`) that exercises the protocol end-to-end: power/status, isolated single-joint moves, multi-joint moves, derived coordinates, speed control, pause/resume, motion flags, the unknown-command fallback path, and a rapid-fire stress loop to catch framing regressions. Run it with the server up:

```bash
uv run main.py
# ...
# All checks passed.
```

## Limitations

- **Not physically accurate.** `GET_COORDS` values are derived from joint angles with a simple formula, not real forward kinematics — good enough to look like moving, self-consistent data, not for validating motion planning.
- **No joint limits.** Unlike the real firmware, the mock accepts and reflects any angle you send, including ones outside the arm's actual mechanical range. Don't treat "the mock accepted it" as "the real robot will accept it."
- **State persists across connections** within one running server process — a fresh client connection does not reset angles/speed/power back to defaults. Restart the server for a clean baseline.
