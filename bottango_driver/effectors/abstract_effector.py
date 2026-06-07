import time

# Bottango normalises all curve Y values to this range (0 = min, 8192 = max)
BOTTANGO_MAX_SIGNAL = 8192

# Number of curves to buffer per effector (mirrors Arduino MAX_NUM_CURVES)
MAX_NUM_CURVES = 8

# Try to import _thread for dual-core curve-buffer locking.
# On RP2040/RP2350 each core runs its own Python VM in parallel, so a
# lock is required whenever core0 (protocol handler) writes to _curves
# while core1 (stepper loop) reads from it.
try:
    import _thread as _thr
    _THREADING = True
except ImportError:
    _thr       = None
    _THREADING = False


class AbstractEffector:
    """
    Base class for all Bottango effectors.

    Mirrors Arduino LoopDrivenEffector:
      - Circular curve buffer (MAX_NUM_CURVES slots)
      - Speed limiting based on maxPWMSec
      - Holds the end value of the last finished curve

    Thread safety
    -------------
    add_curve() / clear_curves() are called from core0 (protocol handler).
    update_on_loop() for StepDirEffectors is called from core1.
    A per-effector _curve_lock serialises access to the _curves array.
    Servo effectors keep the lock allocated but it is never contended
    (both add_curve and update_on_loop run on core0 for servos).
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

        # Circular curve buffer + per-effector lock
        self._curves     = [None] * MAX_NUM_CURVES
        self._curve_idx  = 0       # next write slot
        self._curve_lock = _thr.allocate_lock() if _THREADING else None

        # Speed limiting
        self._last_update_us    = time.ticks_us()
        self._min_us_per_signal = (1_000_000 // max_speed) if max_speed > 0 else 0

    # ------------------------------------------------------------------
    # Curve management
    # ------------------------------------------------------------------

    def add_curve(self, curve):
        """Add a curve to the circular buffer (mirrors AbstractEffector::addCurve)."""
        if self._curve_lock: self._curve_lock.acquire()
        self._curves[self._curve_idx] = curve
        self._curve_idx = (self._curve_idx + 1) % MAX_NUM_CURVES
        if self._curve_lock: self._curve_lock.release()

    def clear_curves(self):
        """Discard all buffered curves (mirrors clearCurves)."""
        if self._curve_lock: self._curve_lock.acquire()
        for i in range(MAX_NUM_CURVES):
            self._curves[i] = None
        if self._curve_lock: self._curve_lock.release()

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
        # Take a lock-protected snapshot of the curve references.
        # Holding the lock only for this brief memcopy keeps the critical
        # section short and avoids stalling add_curve() on core0 while the
        # Bezier solver runs on core1.
        if self._curve_lock: self._curve_lock.acquire()
        curves = self._curves[:]   # 8-element list copy (very fast)
        if self._curve_lock: self._curve_lock.release()

        last_curve    = None
        last_end_time = -1
        found_active  = False

        for curve in curves:
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
        mapped = int(round(self.min_signal + (self.max_signal - self.min_signal)
                           * movement_0_8192 / float(BOTTANGO_MAX_SIGNAL)))
        return max(min(self.min_signal, self.max_signal),
                   min(max(self.min_signal, self.max_signal), mapped))

    def _speed_limit(self, new_target, now_us):
        """
        Rate-limit signal change per µs.
        Mirrors LoopDrivenEffector::speedLimitSingal().
        """
        if self._min_us_per_signal == 0:
            return new_target

        elapsed_us = time.ticks_diff(now_us, self._last_update_us)
        max_delta  = elapsed_us // self._min_us_per_signal

        delta  = new_target - self.current_signal
        result = (self.current_signal + (max_delta if delta > 0 else -max_delta)
                  if abs(delta) > max_delta else new_target)

        return max(min(self.min_signal, self.max_signal),
                   min(max(self.min_signal, self.max_signal), result))

    def destroy(self):
        """Called when the effector is deregistered."""
        self.clear_curves()
