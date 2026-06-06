"""
ESP32 RMT-based step-pulse generator.

One RMT channel per stepper motor.
Python writes a burst descriptor each velocity segment (~67 ms); the RMT
peripheral generates the pulses autonomously without main-loop CPU overhead.

RMT hardware limits (esp-idf / MicroPython):
  - 8 channels (channel 0-7)
  - Up to 64 pulse symbols per channel in a single burst
  - Clock: APB 80 MHz, divided by clock_div=80 → 1 MHz (1 µs resolution)

For Bottango velocity segments (67 ms):
  - At 1 000 steps/s: ~67 steps/segment — slightly above the 64-symbol limit.
  - At 200 steps/s:   ~13 steps/segment — well within limits.

Bursts >64 steps are automatically split into two consecutive writes.
The ~10 µs Python gap between writes is inaudible and physically invisible.

Usage:
    rmt = StepPulseRMT(step_pin_num)
    rmt.push_burst(count, half_period_us)
    rmt.stop()
"""

from machine import Pin

_RMT_MAX_SYMBOLS = 64    # hardware limit per burst
_RMT_CLOCK_DIV   = 80   # 80 MHz APB / 80 = 1 MHz → 1 µs / tick
_MIN_HALF_US     = 5     # minimum half-pulse width (µs)

_USED_CHANNELS: set = set()


def _alloc_channel() -> int:
    for ch in range(8):
        if ch not in _USED_CHANNELS:
            _USED_CHANNELS.add(ch)
            return ch
    raise RuntimeError("No free RMT channel for stepper")


def _free_channel(ch: int):
    _USED_CHANNELS.discard(ch)


class StepPulseRMT:
    """
    Wraps one ESP32 RMT channel that drives a single STEP pin.

    Usage (Python side):
        rmt = StepPulseRMT(step_pin_num)
        rmt.push_burst(count, half_period_us)   # non-blocking
        rmt.stop()                              # on destroy
    """

    def __init__(self, step_pin_num: int):
        import esp32
        self._step_pin_num = step_pin_num
        self._ch           = _alloc_channel()
        self._rmt          = esp32.RMT(
            self._ch,
            pin=Pin(step_pin_num, Pin.OUT, value=0),
            clock_div=_RMT_CLOCK_DIV,
        )

    # ------------------------------------------------------------------

    def push_burst(self, step_count: int, half_period_us: int):
        """
        Generate *step_count* step pulses at *half_period_us* µs per half.
        Non-blocking: returns immediately; RMT executes in background.
        Bursts > 64 steps are split into two sequential writes.

        step_count    : number of steps (must be ≥ 1)
        half_period_us: µs for each half of the STEP pulse (min 5)
        """
        if step_count < 1:
            return

        half_us = max(_MIN_HALF_US, half_period_us)

        # RMT write_pulses takes a flat tuple of durations, alternating HIGH/LOW.
        # Each step = two entries (high_ticks, low_ticks).
        chunk_size = _RMT_MAX_SYMBOLS
        remaining  = step_count

        while remaining > 0:
            count = min(remaining, chunk_size)
            # Build pulse sequence: (high, low) × count
            pulses = (half_us, half_us) * count
            self._rmt.write_pulses(pulses, start=1)
            # wait for this chunk to finish before writing the next
            if remaining > chunk_size:
                self._rmt.wait_done(timeout=int(half_us * 2 * count // 1000) + 10)
            remaining -= count

    def stop(self):
        """Permanent stop; releases the RMT channel."""
        try:
            self._rmt.deinit()
        except Exception:
            pass
        Pin(self._step_pin_num, Pin.OUT, value=0)
        _free_channel(self._ch)
        self._rmt = None
