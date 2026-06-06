import time

# Set True to log snap-to-end events (for debugging trajectory issues).
_DBG_SNAP = False

# Bottango normalises all curve Y values to this range (0 = min, 8192 = max)
BOTTANGO_MAX_SIGNAL = 8192

# Number of curves to buffer per effector (mirrors Arduino MAX_NUM_CURVES)
MAX_NUM_CURVES = 8


class AbstractEffector:
    """
    Base class for all Bottango effectors.

    Mirrors Arduino LoopDrivenEffector:
      - Circular curve buffer (MAX_NUM_CURVES slots)
      - Speed limiting based on maxPWMSec
      - Holds the end value of the last finished curve
    """

    def __init__(self, identifier, min_signal, max_signal, start_val, max_speed):
        self.identifier   = identifier
        self.min_signal   = min_signal
        self.max_signal   = max_signal
        self.max_speed    = max_speed  # µs per second

        # Signal state in hardware units (µs for servos).
        # Subclasses override these in __init__ after calling super().
        self.current_signal = start_val
        self.target_signal  = start_val

        # Circular curve buffer
        self._curves    = [None] * MAX_NUM_CURVES
        self._curve_idx = 0       # next write slot

        # Speed limiting
        self._last_update_us    = time.ticks_us()
        self._min_us_per_signal = (1_000_000 // max_speed) if max_speed > 0 else 0

    # ------------------------------------------------------------------
    # Curve management
    # ------------------------------------------------------------------

    def add_curve(self, curve):
        """Add a curve to the circular buffer (mirrors AbstractEffector::addCurve)."""
        self._curves[self._curve_idx] = curve
        self._curve_idx = (self._curve_idx + 1) % MAX_NUM_CURVES

    def set_curve(self, curve):
        """Alias for add_curve (backwards compat)."""
        self.add_curve(curve)

    def clear_curves(self):
        """Discard all buffered curves (mirrors clearCurves)."""
        for i in range(MAX_NUM_CURVES):
            self._curves[i] = None

    def clear_curve(self):
        """Alias for clear_curves (backwards compat)."""
        self.clear_curves()

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def update_on_loop(self, current_time_ms):
        """
        Evaluate curves → update target_signal → drive hardware.

        Mirrors Arduino:
          LoopDrivenEffector::updateOnLoop()   ← curve evaluation
          PinServoEffector::driveOnLoop()      ← hardware write
        Both are called every loop cycle regardless of curve state.
        """
        last_curve    = None
        last_end_time = -1
        found_active  = False

        for curve in self._curves:
            if curve is None:
                continue

            start = curve.start_time
            end   = curve.start_time + curve.duration

            if start <= current_time_ms <= end:
                # Curve is active — evaluate and update target
                movement   = curve.evaluate(current_time_ms)  # 0–8192
                new_target = self._lerp_signal(movement)       # hardware units (µs)

                now_us  = time.ticks_us()
                limited = self._speed_limit(new_target, now_us)
                if self.target_signal != limited:
                    self._last_update_us = now_us
                    self.target_signal   = limited

                found_active = True
                break  # only one curve active at a time; do NOT return here

            else:
                # Curve finished or not started yet; track the last to end
                curve_end = curve.start_time + curve.duration
                if curve_end > last_end_time:
                    last_end_time = curve_end
                    last_curve    = curve

        # No active curve → snap to the end position of the most recently finished one
        if not found_active and last_curve is not None and last_end_time < current_time_ms:
            end_target = self._lerp_signal(last_curve.end_val)
            if _DBG_SNAP and self.target_signal != end_target:
                from bottango_driver.outgoing import Outgoing
                Outgoing.send_log("SNAP id={} cur={} lc_end={} tgt->{}".format(
                    self.identifier, current_time_ms, last_end_time, end_target))
            if self.target_signal != end_target:
                now_us  = time.ticks_us()
                limited = self._speed_limit(end_target, now_us)
                if self.target_signal != limited:
                    self.target_signal   = limited
                    self._last_update_us = now_us

        # ALWAYS drive hardware every loop (mirrors Arduino's separate driveOnLoop call)
        self.drive_on_loop()

    def drive_on_loop(self):
        """Override in subclasses to write target_signal to hardware."""
        pass

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _lerp_signal(self, movement_0_8192):
        """
        Map Bottango normalised value (0–8192) → hardware units [min_signal, max_signal].
        Mirrors AbstractEffector::lerpSignal().
        """
        ratio  = movement_0_8192 / float(BOTTANGO_MAX_SIGNAL)
        mapped = int(round(self.min_signal + (self.max_signal - self.min_signal) * ratio))
        lo, hi = (self.min_signal, self.max_signal) if self.max_signal >= self.min_signal \
                 else (self.max_signal, self.min_signal)
        return max(lo, min(hi, mapped))

    def _speed_limit(self, new_target, now_us):
        """
        Rate-limit signal change per µs.
        Mirrors LoopDrivenEffector::speedLimitSingal().
        """
        if self._min_us_per_signal == 0:
            return new_target

        elapsed_us = time.ticks_diff(now_us, self._last_update_us)
        max_delta  = elapsed_us // self._min_us_per_signal

        delta = new_target - self.current_signal
        if abs(delta) > max_delta:
            step   = max_delta if delta > 0 else -max_delta
            result = self.current_signal + step
        else:
            result = new_target

        lo, hi = (self.min_signal, self.max_signal) if self.max_signal >= self.min_signal \
                 else (self.max_signal, self.min_signal)
        return max(lo, min(hi, result))

    def destroy(self):
        """Called when the effector is deregistered."""
        self.clear_curves()
