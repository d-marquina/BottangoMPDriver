"""
RP2040 / RP2350 PIO-based step-pulse generator.

One PIO State Machine (SM) per stepper motor.
Python writes ONE 32-bit word to the TX FIFO each velocity segment (~67 ms);
the SM unpacks both values and generates the pulses autonomously without
consuming any main-loop CPU time.

TX FIFO protocol (ONE 32-bit word per burst, atomic — no race condition):
  bits [31:16] : half_period_seed  = half_period_us - 2   (≥ 3 → 5 µs)
  bits [15: 0] : step_count_seed   = step_count - 1       (0 → 1 step)

  The SM uses `out x, 16` / `out y, 16` to unpack both values from a single
  OSR load, so there is no window between word 0 and word 1 where a stray
  value could corrupt the step count or period.

At PIO freq=1_000_000 (1 MHz), each PIO cycle = 1 µs.

Timing for one STEP pulse half (HIGH or LOW):
  1 cycle  : mov(x, isr)   ← reload period template, assert/deassert STEP
  x cycles : jmp(x_dec)    ← busy-wait loop (x iterates from seed to 0)
  1 cycle  : fall-through   ← exit loop at x=0
  Total = x_seed + 2 cycles = half_period_us

DIR pin is managed by Python (set before pushing each burst).
Minimum settle 2 µs is guaranteed by the time Python finishes writing to FIFO.
"""

import rp2
from machine import Pin, StateMachine


# ── PIO program ────────────────────────────────────────────────────────────────
@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT)
def _step_prog():
    """
    Generates N step pulses at a configurable half-period.
    STEP pin is driven via side-set.

    Protocol: one 32-bit word per burst.
      bits[31:16] = half_period_seed  (x register template)
      bits[15: 0] = step_count_seed   (y register; 0 = 1 step)

    After all steps are emitted the SM stalls at pull(block) until
    Python writes the next burst.
    """
    wrap_target()
    # ── pull both values from one word ────────────────────────────────────────
    pull(block)          .side(0)    # stall for next burst word; STEP=LOW
    out(x, 16)                       # x = half_period_seed  (bits 31..16)
    mov(isr, x)                      # save period template in ISR for reload
    out(y, 16)                       # y = step_count_seed   (bits 15.. 0)

    # ── step loop ─────────────────────────────────────────────────────────────
    label("step_loop")

    # HIGH half
    mov(x, isr)          .side(1)   # reload x from ISR, STEP=HIGH
    label("high_wait")
    jmp(x_dec, "high_wait")          # busy-wait x+1 cycles

    # LOW half
    mov(x, isr)          .side(0)   # reload x from ISR, STEP=LOW
    label("low_wait")
    jmp(x_dec, "low_wait")           # busy-wait x+1 cycles

    # next step (y_dec returns True while y > 0, falls through when y reaches 0)
    jmp(y_dec, "step_loop")
    wrap()                           # back to pull(block) → stall until next burst


# ── Python wrapper ─────────────────────────────────────────────────────────────

# Minimum half-period seed that satisfies the 5 µs datasheet requirement.
# half_period = seed + 2 cycles (mov + fall-through) → seed = 3 → 5 µs
_MIN_HALF_PERIOD_SEED = 3   # → 5 µs high, 5 µs low

# Maximum values that fit in 16 bits
_MAX_HALF_PERIOD_SEED = 0xFFFF
_MAX_STEP_COUNT_SEED  = 0xFFFF   # → 65 536 steps per burst

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

        Atomic single-word protocol — no race condition between step count
        and period values.  Non-blocking: skips the burst if the FIFO still
        holds a pending burst from the previous segment.

        step_count    : number of steps (must be ≥ 1)
        half_period_us: µs for each half of the STEP pulse (min 5)

        Returns True if the burst was queued, False if skipped.
        """
        if step_count < 1:
            return True

        # TX FIFO depth = 4 words; we use 1 word per burst.
        # If any word is already queued, the previous burst is still running.
        # Skip this burst (segment overlapped) rather than blocking.
        if self._sm.tx_fifo() > 0:
            return False

        half_seed  = min(_MAX_HALF_PERIOD_SEED, max(_MIN_HALF_PERIOD_SEED, half_period_us - 2))
        count_seed = min(_MAX_STEP_COUNT_SEED,  step_count - 1)

        # Pack into one 32-bit word: high 16 = half_seed, low 16 = count_seed
        word = (half_seed << 16) | count_seed
        self._sm.put(word)
        return True

    def is_idle(self) -> bool:
        """True when the SM has consumed the FIFO word and is stalling at pull."""
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
