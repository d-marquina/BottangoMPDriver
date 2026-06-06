"""
RP2040 / RP2350 PIO-based step-pulse generator.

One PIO State Machine (SM) per stepper motor.
Python writes one burst descriptor (2 × 32-bit words) to the TX FIFO each
velocity segment (~67 ms); the SM generates the pulses autonomously without
consuming any main-loop CPU time.

TX FIFO protocol (two consecutive 32-bit words per burst):
  Word 0 : step_count - 1           (y register seed; 0 → 1 step)
  Word 1 : half_period_cycles - 2   (x register seed; 3 → 5 µs half)

At PIO freq=1_000_000 (1 MHz), each PIO cycle = 1 µs.

Timing for one STEP pulse half (HIGH or LOW):
  1 cycle  : mov(x, isr)   ← reload period template, assert/deassert STEP
  x cycles : jmp(x_dec)    ← busy-wait loop (x iterates from seed to 0)
  1 cycle  : fall-through   ← exit loop at x=0
  Total = x_seed + 2 cycles = half_period_cycles µs

DIR pin is managed by Python (set before pushing each burst).
Minimum settle 2 µs is guaranteed by the time Python finishes writing to FIFO.
"""

import rp2
from machine import Pin, StateMachine
import time


# ── PIO program ────────────────────────────────────────────────────────────────
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW)
def _step_prog():
    """
    Generates N step pulses at a configurable half-period.
    STEP pin is driven via side-set.
    After all steps are emitted the SM stalls at pull(block) until
    Python writes the next burst.
    """
    wrap_target()
    # ── pull step count ───────────────────────────────────────────────────────
    pull(block)          .side(0)    # stall for Word 0; STEP=LOW
    mov(y, osr)                      # y = step_count - 1

    # ── pull half period ──────────────────────────────────────────────────────
    pull(block)                      # stall for Word 1
    mov(x, osr)                      # x = half_period_cycles - 2 (template)
    mov(isr, x)                      # save template → ISR for reload each half

    # ── step loop ─────────────────────────────────────────────────────────────
    label("step_loop")

    # HIGH half
    mov(x, isr)          .side(1)   # reload x, STEP=HIGH
    label("high_wait")
    jmp(x_dec, "high_wait")          # busy-wait

    # LOW half
    mov(x, isr)          .side(0)   # reload x, STEP=LOW
    label("low_wait")
    jmp(x_dec, "low_wait")           # busy-wait

    # next step
    jmp(y_dec, "step_loop")          # y-- and repeat (fall-through when y was 0)
    wrap()                           # back to pull(block) → stall until next burst


# ── Python wrapper ─────────────────────────────────────────────────────────────

# Minimum half-period cycles that satisfies the 5 µs datasheet requirement.
# At 1 MHz PIO clock: half_period = seed + 2 cycles → seed = 3 → 5 µs
_MIN_HALF_PERIOD_SEED = 3   # → 5 µs high, 5 µs low

# Statically allocate SM indices (one per stepper registered on this board).
# RP2040 has 8 SMs (PIO0: 0-3, PIO1: 4-7); RP2350 has 12 (PIO2: 8-11).
#
# IMPORTANT: MicroPython's built-in neopixel/WS2812 driver uses SM 0 (hardcoded).
# We therefore start allocation from SM 1 to avoid silently sharing the SM —
# a conflict causes sm.put() to block indefinitely, freezing the main loop.
_USED_SM_IDS: set = set()
_SM_FIRST = 1   # skip SM 0 (reserved for NeoPixel / WS2812)


def _alloc_sm_id() -> int:
    for sid in range(_SM_FIRST, 12):
        if sid not in _USED_SM_IDS:
            _USED_SM_IDS.add(sid)
            return sid
    raise RuntimeError("No free PIO state machine for stepper")


def _free_sm_id(sid: int):
    _USED_SM_IDS.discard(sid)


class StepPulsePIO:
    """
    Wraps one PIO SM that drives a single STEP pin.

    Usage (Python side):
        pio = StepPulsePIO(step_pin_num)
        pio.push_burst(count, half_period_us)   # non-blocking
        pio.stop()                              # on destroy
    """

    def __init__(self, step_pin_num: int):
        self._step_pin_num = step_pin_num
        self._sm_id = _alloc_sm_id()
        self._sm = StateMachine(
            self._sm_id,
            _step_prog,
            freq=1_000_000,              # 1 MHz → 1 µs per cycle
            sideset_base=Pin(step_pin_num, Pin.OUT, value=0),
        )
        self._sm.active(1)

    # ------------------------------------------------------------------

    def push_burst(self, step_count: int, half_period_us: int):
        """
        Queue N step pulses at the given half-period (µs each side).
        Truly non-blocking: checks FIFO space before writing.
        Returns False (and skips the burst) if the FIFO is unexpectedly full.

        step_count    : number of steps (must be ≥ 1)
        half_period_us: µs for each half of the STEP pulse (min 5)
        """
        if step_count < 1:
            return True
        # TX FIFO depth = 4 words.  Each burst uses 2 words.
        # If more than 2 words are already queued, the previous burst is still
        # running or the SM stalled.  Skip this burst rather than blocking.
        if self._sm.tx_fifo() > 2:
            return False
        seed_y = step_count - 1
        seed_x = max(_MIN_HALF_PERIOD_SEED, half_period_us - 2)
        self._sm.put(seed_y)
        self._sm.put(seed_x)
        return True

    def is_idle(self) -> bool:
        """True when the SM has consumed both FIFO words and is stalling at pull."""
        return self._sm.tx_fifo() == 0

    def abort(self):
        """Stop immediately (direction change or destroy)."""
        self._sm.active(0)
        # STEP pin may be left HIGH if we abort mid-pulse; drive it LOW.
        Pin(self._step_pin_num, Pin.OUT, value=0)
        self._sm.restart()
        self._sm.active(1)

    def stop(self):
        """Permanent stop; releases the SM slot."""
        self._sm.active(0)
        Pin(self._step_pin_num, Pin.OUT, value=0)
        _free_sm_id(self._sm_id)
        self._sm = None
