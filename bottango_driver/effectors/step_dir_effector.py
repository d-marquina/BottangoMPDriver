"""
Step/Dir stepper motor effector.

Mirrors Arduino StepDirStepperEffector + VelocityEffector.

Protocol command:
  rSTDir,stepPin,dirPin,clockwiseIsLow,maxCCWSteps,maxCWSteps,maxStepsPerSec,startingOffset

Identifier: str(stepPin)

Signal units: integer step count in [maxCCWSteps, maxCWSteps].
  current_signal tracks the absolute step position relative to the homed origin.

Homing:
  Curves do not execute until the effector is homed. The host sends:
    sycM,<id>,home       → mark current position as home
    sycM,<id>,rst        → clear home flag
    sycM,<id>,<N>        → move N steps from current position (manual sync)
    sycM,<id>,aCW/aCC   → auto-home (not supported; silently ignored)

Velocity segments:
  Curves are chunked into VELOCITY_SEGMENT_MS (67 ms) slices.  At each slice
  boundary the effector evaluates the curve at t+67 ms, computes how many
  steps are needed, and starts a hardware pulse burst.
  The burst runs entirely in PIO/RMT hardware — no per-step Python code.

Pulse generation back-ends (auto-selected by platform):
  RP2040 / RP2350 → PIO (one State Machine per motor, 1 µs resolution)
  ESP32           → software fallback (RMT support can be added later)
  Other           → software fallback (bit-bang in main loop)

Pulse timing constants (mirror StepDirStepperEffector.h):
  MIN_PULSE_WIDTH_US = 5   µs per half-pulse (HIGH or LOW)
  DIR_PULSE_HOLD_US  = 2   µs DIR settle before first STEP
"""

import time
from machine import Pin
from bottango_driver.effectors.abstract_effector import (
    AbstractEffector, MAX_NUM_CURVES
)

# ── Constants ───────────────────────────────────────────────────────────────────

# Velocity segment length (ms).  Mirrors Arduino VELOCITY_SEGMENT_MS.
VELOCITY_SEGMENT_MS = 67

# Sync moves at half of max speed.  Mirrors Arduino STEPPER_SYNC_SPEED.
STEPPER_SYNC_SPEED = 2

# µs per half of a STEP pulse (applies to both PIO and software path).
MIN_PULSE_WIDTH_US = 5
DIR_SETTLE_US      = 2

# Sentinel for _in_progress_idx: snapping to the end of the last curve.
_MOVING_TO_END = -1


# ── Platform detection ──────────────────────────────────────────────────────────

def _detect_backend():
    """Return 'pio', 'rmt', or 'sw' based on the detected platform."""
    try:
        from bottango_driver.board_defs import PLATFORM_NAME
        if PLATFORM_NAME in ("RP2040", "RP2350"):
            return "pio"
        if PLATFORM_NAME == "ESP32":
            return "rmt"
    except Exception:
        pass
    return "sw"


_BACKEND = _detect_backend()


# ── StepDirEffector ─────────────────────────────────────────────────────────────

class StepDirEffector(AbstractEffector):
    """
    Step/Dir stepper motor effector.

    Pulse generation is offloaded to PIO (RP2040/RP2350) or software fallback.
    The Python main loop only runs at most once every VELOCITY_SEGMENT_MS to
    evaluate the next curve segment — no per-step timing overhead.
    """

    def __init__(self, step_pin_num, dir_pin_num, clockwise_is_low,
                 max_ccw_steps, max_cw_steps, max_steps_per_sec, starting_offset):
        identifier = str(step_pin_num)
        super().__init__(identifier, max_ccw_steps, max_cw_steps,
                         starting_offset, max_steps_per_sec)

        self.current_signal = starting_offset
        self.target_signal  = starting_offset

        self._dir_pin         = Pin(dir_pin_num, Pin.OUT)
        self.clockwise_is_low = clockwise_is_low
        self._curr_dir_cw     = True
        self._dir_pin.value(0 if clockwise_is_low else 1)

        # Homing state
        self.homed = False

        # Manual sync (steps remaining, + = CW, - = CCW)
        self._sync = 0

        # Velocity segment tracking
        self._segment_start_ms  = 0     # ticks_ms() when current burst was queued
        self._in_progress_idx   = 0     # curve slot being executed (or _MOVING_TO_END)

        # Initialise pulse backend
        self._backend = _BACKEND
        if _BACKEND == "pio":
            self._init_pio(step_pin_num)
        elif _BACKEND == "rmt":
            self._init_rmt(step_pin_num)
        else:
            self._init_sw(step_pin_num)

    # ------------------------------------------------------------------
    # Backend initialisation
    # ------------------------------------------------------------------

    def _init_pio(self, step_pin_num):
        try:
            from bottango_driver.effectors.step_pulse_pio import StepPulsePIO
            self._pio = StepPulsePIO(step_pin_num)
        except Exception as e:
            from bottango_driver.outgoing import Outgoing
            Outgoing.send_log("PIO init failed ({}), using SW".format(e))
            self._backend = "sw"
            self._init_sw(step_pin_num)

    def _init_rmt(self, step_pin_num):
        try:
            from bottango_driver.effectors.step_pulse_rmt import StepPulseRMT
            self._rmt = StepPulseRMT(step_pin_num)
        except Exception as e:
            from bottango_driver.outgoing import Outgoing
            Outgoing.send_log("RMT init failed ({}), using SW".format(e))
            self._backend = "sw"
            self._init_sw(step_pin_num)

    def _init_sw(self, step_pin_num):
        """Software fallback: bit-bang STEP pin in main loop."""
        self._step_pin            = Pin(step_pin_num, Pin.OUT, value=0)
        self._pio                 = None
        # SW-fallback state machine (mirrors old drive_on_loop)
        self._sw_drive            = 0    # -1, 0, +1
        self._sw_hold_start_us    = 0
        self._sw_step_high        = False
        self._sw_dir_switch       = False
        self._sw_period_us        = 0
        self._sw_last_step_us     = 0
        self._sw_steps_remaining  = 0

    # ------------------------------------------------------------------
    # Public sync / homing API  (called by sycM handler)
    # ------------------------------------------------------------------

    def set_home(self):
        """Mark current position as home. Curves can now execute."""
        self.homed = True

    def reset_home(self):
        """Clear home flag. Curves pause until re-homed."""
        self.homed = False

    def set_sync(self, steps):
        """
        Move *steps* steps at half max speed (manual homing).
        Positive = CW, negative = CCW.
        """
        if steps == 0:
            return
        self._sync = steps
        # Reset segment timer so the sync runs on the next update_on_loop call.
        self._segment_start_ms = 0

    # ------------------------------------------------------------------
    # Main loop — called every loop iteration
    # ------------------------------------------------------------------

    def update_on_loop(self, current_time_ms):
        """
        Evaluate velocity segments and issue step bursts.
        Called every main-loop iteration but does significant work only at
        67 ms segment boundaries.
        """
        # ── Software fallback: must drive pulses every iteration ────────
        if self._backend == "sw":
            self._sw_drive_on_loop()
            if self._sw_steps_remaining > 0 or self._sw_drive != 0:
                return   # pulse in progress; skip evaluation

        # ── Segment timer: wait until current burst window expires ───────
        if self._segment_start_ms != 0:
            elapsed = time.ticks_diff(current_time_ms, self._segment_start_ms)
            if elapsed < VELOCITY_SEGMENT_MS:
                return

        now_us = time.ticks_us()

        # ── Manual sync ─────────────────────────────────────────────────
        if self._sync != 0:
            # Maximum steps per segment (half max speed)
            if self._min_us_per_signal > 0:
                max_steps = (VELOCITY_SEGMENT_MS * 1000) // (self._min_us_per_signal * STEPPER_SYNC_SPEED * 2)
            else:
                max_steps = abs(self._sync)  # no speed limit → do all at once
            max_steps = max(1, max_steps)

            chunk = min(abs(self._sync), max_steps)
            direction = 1 if self._sync > 0 else -1
            steps_this_seg = chunk * direction

            half_us = (self._min_us_per_signal * STEPPER_SYNC_SPEED) if self._min_us_per_signal > 0 else MIN_PULSE_WIDTH_US
            self._issue_burst(steps_this_seg, half_us, current_time_ms)

            self._sync -= steps_this_seg
            if self._sync == 0:
                self.target_signal = self.current_signal  # anchor to avoid reversion
            return

        # ── Require homing ──────────────────────────────────────────────
        if not self.homed:
            return

        # ── Evaluate next velocity segment ──────────────────────────────
        next_click_ms = current_time_ms + VELOCITY_SEGMENT_MS
        last_curve    = None

        for i in range(MAX_NUM_CURVES):
            curve = self._curves[i]
            if curve is None:
                continue

            curve_end = curve.start_time + curve.duration

            if curve.start_time <= next_click_ms <= curve_end:
                movement   = curve.evaluate(next_click_ms)      # 0–8192
                new_target = self._lerp_signal(movement)         # steps
                delta      = new_target - self.current_signal

                if delta != 0:
                    self.target_signal    = new_target
                    self._in_progress_idx = i

                    # Spread steps evenly over the segment
                    time_us   = VELOCITY_SEGMENT_MS * 1000
                    half_us   = max(MIN_PULSE_WIDTH_US, time_us // (abs(delta) * 2))
                    self._issue_burst(delta, half_us, current_time_ms)
                else:
                    # No movement this segment; mark segment start to avoid
                    # busy-re-evaluation until the next 67 ms window.
                    self._segment_start_ms = current_time_ms
                return

            curve_end_t = curve.start_time + curve.duration
            if last_curve is None or curve_end_t > (last_curve.start_time + last_curve.duration):
                last_curve = curve

        # ── No active curve → snap to end of last finished ─────────────
        if last_curve is not None:
            lc_end = last_curve.start_time + last_curve.duration
            if lc_end < current_time_ms:
                end_target = self._lerp_signal(last_curve.end_val)
                delta      = end_target - self.current_signal
                if self.target_signal != end_target and delta != 0:
                    self.target_signal    = end_target
                    self._in_progress_idx = _MOVING_TO_END
                    half_us               = self._min_us_per_signal if self._min_us_per_signal > 0 else MIN_PULSE_WIDTH_US
                    self._issue_burst(delta, half_us, current_time_ms)

    # ------------------------------------------------------------------
    # Burst dispatch
    # ------------------------------------------------------------------

    def _issue_burst(self, steps: int, half_period_us: int, current_time_ms: int):
        """
        Move *steps* steps (sign = direction) at *half_period_us* µs per half.
        Updates current_signal and marks the segment start time.
        """
        if steps == 0:
            return

        want_cw = steps > 0
        if want_cw != self._curr_dir_cw:
            self._set_direction(want_cw)

        abs_steps = abs(steps)
        half_us   = max(MIN_PULSE_WIDTH_US, half_period_us)

        if self._backend == "pio":
            self._pio.push_burst(abs_steps, half_us)
        elif self._backend == "rmt":
            self._rmt.push_burst(abs_steps, half_us)
        else:
            self._sw_start_burst(abs_steps, half_us)

        # current_signal tracks committed position (PIO executes asynchronously;
        # we trust the burst will complete within the segment window).
        self.current_signal   += steps
        self._segment_start_ms = current_time_ms

    def _set_direction(self, cw: bool):
        """Assert DIR pin and wait the minimum settle time."""
        self._dir_pin.value(0 if (cw == self.clockwise_is_low) else 1)
        self._curr_dir_cw = cw
        # DIR must settle before the first STEP pulse.
        # On PIO path: the few µs Python takes to write to FIFO provide the settle.
        # On SW path: we add an explicit busy-wait.
        if self._backend == "sw":
            time.sleep_us(DIR_SETTLE_US)
        elif self._backend == "pio":
            # Abort any in-flight burst (direction change at segment boundary;
            # remaining steps from the previous segment are discarded).
            # The PIO SM is restarted; STEP pin is driven LOW.
            # The few µs of Python execution before the next push_burst()
            # provides ample DIR settle time (requirement: 2 µs).
            self._pio.abort()
        elif self._backend == "rmt":
            # RMT: let the current burst finish naturally (it's tiny at segment
            # boundary); the DIR settle is the Python execution time.
            pass

    # ------------------------------------------------------------------
    # Software fallback pulse state machine
    # (used only when PIO / RMT are not available)
    # ------------------------------------------------------------------

    def _sw_start_burst(self, count: int, half_period_us: int):
        """Queue a software-driven burst."""
        self._sw_steps_remaining = count
        self._sw_period_us       = half_period_us
        self._sw_last_step_us    = time.ticks_us()
        self._sw_drive           = 1 if self._curr_dir_cw else -1
        self._sw_step_high       = False
        self._sw_hold_start_us   = 0

    def _sw_drive_on_loop(self):
        """
        Per-iteration software pulse state machine (fallback only).
        Issues one STEP pulse at a time; timing resolution ≈ main loop period.
        """
        if self._sw_steps_remaining == 0:
            self._sw_drive = 0
            return

        now_us = time.ticks_us()

        # Active hold (HIGH or LOW phase)
        if self._sw_hold_start_us != 0:
            if time.ticks_diff(now_us, self._sw_hold_start_us) < MIN_PULSE_WIDTH_US:
                return  # still holding
            if self._sw_step_high:
                # End HIGH, start LOW
                self._step_pin.value(0)
                self._sw_step_high     = False
                self._sw_hold_start_us = now_us
            else:
                # End LOW → step complete
                self._sw_hold_start_us  = 0
                self._sw_steps_remaining -= 1
                if self._sw_steps_remaining == 0:
                    self._sw_drive = 0
            return

        # Check inter-step period
        if time.ticks_diff(now_us, self._sw_last_step_us) < self._sw_period_us * 2:
            return  # wait for next step window

        # Emit rising edge
        self._step_pin.value(1)
        self._sw_step_high     = True
        self._sw_hold_start_us = now_us
        self._sw_last_step_us  = now_us

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy(self):
        """Stop motion and release hardware on deregistration."""
        if self._backend == "pio" and self._pio is not None:
            self._pio.stop()
            self._pio = None
        elif self._backend == "rmt" and getattr(self, '_rmt', None) is not None:
            self._rmt.stop()
            self._rmt = None
        else:
            self._sw_drive           = 0
            self._sw_steps_remaining = 0
            try:
                self._step_pin.value(0)
            except Exception:
                pass
        try:
            self._dir_pin.value(0)
        except Exception:
            pass
        super().destroy()
