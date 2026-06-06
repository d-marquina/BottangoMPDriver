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
Implements the Bottango serial protocol (v 0.8.0b1): handshake (`hRQ` / `btngoHSK`), module negotiation (`hMOD`), time sync (`tSYN`), effector registration, curve delivery (`sC`), and clean shutdown (`STOP`). Responses are byte-accurate and hash-validated by Bottango.

### Unique device ID
Reports a stable 16-character hex UID during the `hMOD` handshake so Bottango can recognise and restore the controller layout across sessions. Uses `machine.unique_id()` (flash chip ID on RP2040, chip ID on RP2350, MAC base on ESP32) with a filesystem-persisted fallback for boards whose hardware ID reads as all-zeros.

### Pin servos
Standard PWM servo output on any GPIO pin. Configurable pulse range (µs), maximum speed (µs/s), and start position. Speed-limiting is applied per-frame to produce smooth motion.

### I2C servos via PCA9685
Up to 48 channels across three PCA9685 boards (16 channels each) on a single I2C bus. Boards are auto-detected and shared transparently between effectors using a reference-counted driver pool. I2C pins and frequency are configurable; sensible defaults are provided for each supported platform.

### Bezier curve playback
Full floating-point Bezier curve evaluation matching Bottango's own curve format — same control-point layout, same 0–8192 signal space, same binary-search solver. Supports up to 8 queued curves per effector and end-snap behaviour identical to the Arduino reference driver.

---

## Roadmap

The following effector types are planned for future releases:

- **Step/Dir stepper motors** — two-pin (STEP + DIR) interface for common stepper drivers such as A4988, DRV8825, and TMC22xx.
- **4-wire stepper motors** — direct coil drive for 28BYJ-48 and similar unipolar/bipolar steppers wired to four GPIO pins.

---

## Quick Start

1. Flash MicroPython firmware on your board.
2. Copy the entire project folder to the board (e.g. with `mpremote`, Thonny, or rshell).
3. Open `main.py` and adjust the `Config` class if needed (pins, I2C address, enabled modules).
4. Reset the board — it will print `BOOT` on the USB serial port.
5. Open Bottango, add a new Serial Controller, and select the board's COM port.

---

## Configuration (`main.py`)

| Parameter | Default | Description |
|---|---|---|
| `BAUD_RATE` | `115200` | Serial baud rate (must match Bottango) |
| `TIMEOUT_THRESH_MS` | `0` | Inactivity timeout before auto-deregistering (ms). `0` disables the timeout entirely (recommended). The Arduino reference driver has no equivalent mechanism. |
| `MAX_NUM_EFFECTORS` | `16` | Maximum simultaneous effectors |
| `ENABLE_PIN_SERVOS` | `True` | Enable GPIO PWM servo output |
| `ENABLE_I2C_SERVOS` | `True` | Enable PCA9685 I2C servo expander |
| `REPORT_UID` | `True` | Report unique board ID during handshake |
| `I2C_SDA_PIN` / `I2C_SCL_PIN` | `None` (platform default) | Override I2C pins |
| `I2C_FREQ` | `400000` | I2C clock frequency (Hz) |
| `ENABLE_STATUS_LIGHTS` | `True` | NeoPixel status LED |

---

## License

This project is released under the **MIT License**. See [`LICENSE`](LICENSE) for the full text.

Bottango is a product of [Bottango LLC](https://www.bottango.com/). This driver is an independent community project and is not affiliated with or endorsed by Bottango LLC.
