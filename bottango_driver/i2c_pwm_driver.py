"""
Generic I2C PWM driver for 16-channel servo expansion boards.

Uses the standard 12-bit PWM expander protocol (used by PCA9685, PCA9635,
and compatible boards).  The public API is model-agnostic:

    driver = I2CPWMDriver(i2c, address=0x40)
    driver.write_microseconds(channel=0, us=1500)   # servo centre
    driver.set_all_off()                             # all channels off

Typical servo PWM frequency: 50 Hz (20 ms period).
"""

import time

# ── Register map ──────────────────────────────────────────────────────────────
_MODE1     = 0x00
_MODE2     = 0x01
_PRESCALE  = 0xFE
_LED0_ON_L = 0x06   # channel 0 base; each channel occupies 4 bytes

_RESTART = 0x80
_SLEEP   = 0x10
_ALLCALL = 0x01
_OUTDRV  = 0x04   # totem-pole outputs (needed for servo signal lines)

# Internal oscillator frequency used to compute the prescaler
_OSC_CLOCK = 25_000_000   # 25 MHz


class I2CPWMDriver:
    """
    I2C PWM expander driver with a servo-friendly microsecond API.

    Parameters
    ----------
    i2c     : machine.I2C instance
    address : 7-bit I2C address of the board (default 0x40)
    freq    : initial PWM frequency in Hz (default 50)
    """

    def __init__(self, i2c, address=0x40, freq=50):
        self._i2c     = i2c
        self._address = address

        # Soft-reset via MODE1
        self._write(_MODE1, _ALLCALL)
        time.sleep_ms(5)

        # Wake up (clear SLEEP bit)
        mode1 = self._read(_MODE1) & ~_SLEEP
        self._write(_MODE1, mode1)
        time.sleep_ms(5)

        # Totem-pole outputs
        self._write(_MODE2, _OUTDRV)

        self.set_freq(freq)

    # ── Public API ────────────────────────────────────────────────────────────

    def set_freq(self, freq):
        """Set PWM frequency for all channels (Hz).  Typically called once."""
        prescale = round(_OSC_CLOCK / (4096 * freq)) - 1
        prescale = max(3, min(255, prescale))

        mode1 = self._read(_MODE1)
        self._write(_MODE1, (mode1 & ~_RESTART) | _SLEEP)
        self._write(_PRESCALE, prescale)
        self._write(_MODE1, mode1)
        time.sleep_ms(5)
        self._write(_MODE1, mode1 | _RESTART)
        time.sleep_ms(5)

        self._freq      = freq
        self._period_us = 1_000_000 // freq   # e.g. 20 000 µs at 50 Hz

    def write_microseconds(self, channel, us):
        """
        Set the pulse width of *channel* (0–15) in microseconds.

        Typical servo range: 1000–2000 µs, centre 1500 µs.
        """
        ticks = int(us * 4096 // self._period_us)
        ticks = max(0, min(4095, ticks))
        self._set_pwm(channel, 0, ticks)

    def set_all_off(self):
        """Drive all 16 channels to 0 % duty cycle."""
        for ch in range(16):
            self._set_pwm(ch, 0, 0)

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _set_pwm(self, channel, on, off):
        # Write one byte per register — avoids dependency on AI (Auto-Increment)
        # bit in MODE1, which is 0 by default on the PCA9685.
        base = _LED0_ON_L + 4 * channel
        self._write(base,     on  & 0xFF)
        self._write(base + 1, (on  >> 8) & 0x0F)
        self._write(base + 2, off & 0xFF)
        self._write(base + 3, (off >> 8) & 0x0F)

    def _write(self, reg, value):
        self._i2c.writeto_mem(self._address, reg, bytes([value & 0xFF]))

    def _read(self, reg):
        return self._i2c.readfrom_mem(self._address, reg, 1)[0]
