# BottangoMPDriver

A community MicroPython driver for [Bottango](https://www.bottango.com/), the real-time servo animation software. It lets you connect a Raspberry Pi Pico, Pico 2, or ESP32 board directly to Bottango over USB serial — no Arduino required.

---

## Supported Boards

| Board | Chip | Status |
|---|---|---|
| Raspberry Pi Pico | RP2040 | ✅ Tested |
| Raspberry Pi Pico 2 | RP2350 | ✅ Tested |
| ESP32 (classic / S2 / S3 / C3) | Xtensa / RISC-V | ✅ Supported |

Any MicroPython-capable board with a USB serial port and enough GPIO should work with minor pin configuration changes in `main.py`.

---

## Features

### Protocol compatibility
Implements the Bottango serial protocol (v 0.8.0b1): handshake (`hRQ` / `btngoHSK`), module negotiation (`hMOD`), time sync (`tSYN`), effector registration, curve delivery (`sC`, `sSY`), stepper sync commands (`sycM`), and clean shutdown (`STOP`). Responses are byte-accurate and hash-validated by Bottango.

### Unique device ID
Reports a stable 16-character hex UID during the `hMOD` handshake so Bottango can recognise and restore the controller layout across sessions. Uses `machine.unique_id()` (flash chip ID on RP2040, chip ID on RP2350, MAC base on ESP32) with a filesystem-persisted fallback for boards whose hardware ID reads as all-zeros.

### Pin servos
Standard PWM servo output on any GPIO pin. Configurable pulse range (µs), maximum speed (µs/s), and start position. Speed-limiting is applied per-frame to produce smooth motion.

### I2C servos via PCA9685
Up to 48 channels across three PCA9685 boards (16 channels each) on a single I2C bus. Boards are auto-detected and shared transparently between effectors using a reference-counted driver pool. I2C pins and frequency are configurable; sensible defaults are provided for each supported platform.

### Step/Dir stepper motors
Two-pin (STEP + DIR) interface for common stepper drivers such as A4988, DRV8825, and TMC22xx.

- **Hardware pulse generation** — on RP2040/RP2350 a dedicated PIO state machine generates the step pulse train; on ESP32 the RMT peripheral is used. The CPU is never busy-waiting.
- **Dual-core execution** — on RP2040/RP2350 the stepper update loop runs on core 1 independently of the servo update loop on core 0, so heavy I2C traffic never causes stepper timing drift.
- **Synchronized homing** — supports Bottango's `sycM` homing protocol (`home`, `rst`). The driver reports `allowSync=1` when steppers are enabled, activating the homing UI in Bottango.
- **Trajectory accuracy** — Bezier curve evaluation and PIO burst scheduling are timed to 67 ms VelocityEffector segments, matching Bottango's animation resolution exactly.

### Bezier curve playback
Full floating-point Bezier curve evaluation matching Bottango's own curve format — same control-point layout, same 0–8192 signal space, same binary-search solver. Supports up to 8 queued curves per effector and end-snap behaviour identical to the Arduino reference driver. The `sSY` batch-curve command is fully handled, so the first animation segment plays correctly for all effectors simultaneously.

---

## Quick Start

1. Flash MicroPython firmware on your board.
2. Copy the entire project folder to the board (e.g. with `mpremote`, Thonny, or rshell).
3. Open `main.py` and set the `ENABLE_*` flags for the modules you need, then adjust pins and I2C settings as required.
4. Reset the board — it will print `BOOT` on the USB serial port.
5. Open Bottango, add a new Serial Controller, and select the board's COM port.

---

## Configuration (`main.py`)

| Parameter | Default | Description |
|---|---|---|
| `BAUD_RATE` | `115200` | Serial baud rate (must match Bottango) |
| `TIMEOUT_THRESH_MS` | `0` | Inactivity timeout before auto-deregistering (ms). `0` disables the timeout entirely (recommended). |
| `MAX_NUM_EFFECTORS` | `16` | Maximum simultaneous effectors |
| `ENABLE_PIN_SERVOS` | `False` | Enable GPIO PWM servo output |
| `ENABLE_STEP_DIR_STEPPERS` | `False` | Enable Step/Dir stepper motor support (PIO on RP2040/RP2350, RMT on ESP32) |
| `ENABLE_I2C_SERVOS` | `False` | Enable PCA9685 I2C servo expander |
| `REPORT_UID` | `True` | Report unique board ID during handshake |
| `I2C_ID` | `None` (platform default) | I2C bus index |
| `I2C_SDA_PIN` / `I2C_SCL_PIN` | `None` (platform default) | Override I2C pins |
| `I2C_FREQ` | `400000` | I2C clock frequency (Hz) |
| `ENABLE_STATUS_LIGHTS` | `True` | NeoPixel status LED |
| `STATUS_PIN` | `2` | GPIO pin for the NeoPixel LED |

### Platform default I2C pins

| Board | SDA | SCL | Bus |
|---|---|---|---|
| Pico / Pico 2 | GP4 | GP5 | 0 |
| ESP32 | GPIO21 | GPIO22 | 0 |

---

## Architecture notes

### Dual-core stepper timing (RP2040 / RP2350)

Servo I2C writes can take 300–500 µs each. With several servos active the main loop stretches well beyond Bottango's 67 ms stepper segment window, causing cumulative lag and a final jump to the end keyframe. The driver solves this by splitting the update loop:

- **Core 0** — serial communication, Bezier curve registration, servo PWM/I2C writes, status lights.
- **Core 1** — stepper `update_on_loop()` calls in a tight loop, fully independent of serial and I2C activity.

Per-effector `_thread` locks protect the shared curve buffer. The pool sets `_core1_active = False` and sleeps 10 ms before any structural modification (register / deregister / clear), ensuring core 1 has exited its current iteration.

### PIO pulse generation (RP2040 / RP2350)

Step bursts are encoded as a single 32-bit word (`half_period_seed << 16 | step_count_seed`) pushed to the PIO TX FIFO atomically. The state machine unpacks both fields with `out(x, 16)` / `out(y, 16)` and generates the pulse train without CPU involvement. A FIFO guard (`tx_fifo() > 0`) prevents re-entry before the previous burst finishes.

### `sSY` sync-curve command

Bottango batches the first curve of every effector into a single `sSY` command so all effectors start their animation at the same timestamp. The driver parses the semicolon-separated entries and delegates each to the same `handle_set_curve` path used for individual `sC` commands.

---

## License

This project is released under the **MIT License**. See [`LICENSE`](LICENSE) for the full text.

Bottango is a product of [Bottango LLC](https://www.bottango.com/). This driver is an independent community project and is not affiliated with or endorsed by Bottango LLC.
