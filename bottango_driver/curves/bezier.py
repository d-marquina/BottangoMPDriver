"""
Cubic Bezier curve — mirrors Arduino FloatBezierCurve.

Coordinate system (matches Bottango wire format):
  X axis: milliseconds, 0 = curve start, duration = curve end
  Y axis: raw Bottango signal, 0–8192

Control points received from Bottango:
  cp1x: ms offset from curve START  (positive, toward end)
  cp1y: signal offset from startY   (can be positive or negative)
  cp2x: ms offset from curve END    (negative, toward start)
  cp2y: signal offset from endY     (can be positive or negative)

Internally we normalise X to [0,1] and keep Y in [0,8192] (raw).
evaluate() returns raw signal value in [0, 8192].
"""


class BezierCurve:
    def __init__(self, start_time_ms, duration_ms, start_val, end_val,
                 cp1x, cp1y, cp2x, cp2y):

        self.start_time = start_time_ms
        self.duration   = duration_ms
        self.start_val  = start_val    # raw 0–8192
        self.end_val    = end_val      # raw 0–8192

        # Normalise X control points to [0, 1]
        dur = float(duration_ms) if duration_ms > 0 else 1.0
        self._cp1x = cp1x / dur                    # P1.x in [0,1]
        self._cp2x = (duration_ms + cp2x) / dur    # P2.x in [0,1]  (cp2x ≤ 0)

        # Keep Y in raw units — same as Arduino
        # P0 = startY
        # P1 = startY + cp1y
        # P2 = endY   + cp2y
        # P3 = endY
        self._p1y = start_val + cp1y
        self._p2y = end_val   + cp2y

        self._last_u = 0.5  # cached u for faster convergence next call

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def evaluate(self, current_time_ms):
        """
        Return raw Bottango signal (0–8192) for the given absolute time.
        Mirrors FloatBezierCurve::getValue(currentTimeMs).
        """
        if self.duration <= 0:
            return self.end_val

        elapsed = current_time_ms - self.start_time

        if elapsed <= 0:
            return self.start_val
        if elapsed >= self.duration:
            return self.end_val

        u = self._solve_u(elapsed)
        return int(round(self._eval_y(u)))

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _solve_u(self, x_ms):
        """
        Binary search for u such that B_x(u) ≈ x_ms.
        Mirrors FloatBezierCurve::Evaluate() / the binary-search loop.
        x_ms is in ms (relative to curve start, i.e. 0 … duration).
        """
        # Normalise x to [0, 1]
        x = x_ms / float(self.duration)

        u_lo = 0.0
        u_hi = 1.0
        u    = min(max(self._last_u, 0.0), 1.0)

        for _ in range(20):          # max 20 iterations (Arduino has no limit, but 20 is plenty)
            ex = self._eval_x(u)
            diff = ex - x
            if abs(diff) < 0.001:   # ~1 ms precision at 1000 ms duration
                break
            if diff > 0:
                u_hi = u
            else:
                u_lo = u
            u = (u_hi + u_lo) * 0.5

        self._last_u = u
        return u

    @staticmethod
    def _lerp(a, b, u):
        return a + (b - a) * u

    def _eval_x(self, u):
        """Evaluate normalised X (0–1) for parameter u."""
        # P0.x=0, P1.x=cp1x, P2.x=cp2x, P3.x=1
        p11 = self._lerp(0.0,       self._cp1x, u)
        p12 = self._lerp(self._cp1x, self._cp2x, u)
        p13 = self._lerp(self._cp2x, 1.0,        u)
        p21 = self._lerp(p11, p12, u)
        p22 = self._lerp(p12, p13, u)
        return self._lerp(p21, p22, u)

    def _eval_y(self, u):
        """Evaluate raw Y signal (0–8192) for parameter u."""
        # P0=startY, P1=startY+cp1y, P2=endY+cp2y, P3=endY
        p11 = self._lerp(float(self.start_val), float(self._p1y),    u)
        p12 = self._lerp(float(self._p1y),       float(self._p2y),    u)
        p13 = self._lerp(float(self._p2y),        float(self.end_val), u)
        p21 = self._lerp(p11, p12, u)
        p22 = self._lerp(p12, p13, u)
        return self._lerp(p21, p22, u)
