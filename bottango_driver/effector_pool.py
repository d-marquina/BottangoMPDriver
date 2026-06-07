import time

from bottango_driver.errors import EFFECTOR_POOL_FULL, EFFECTOR_NOT_FOUND
from bottango_driver.outgoing import Outgoing
from bottango_driver.callbacks import on_effector_registered, on_effector_deregistered


class EffectorPool:
    """
    Manages all registered effectors and drives their update loops.

    Dual-core operation (RP2040 / RP2350)
    --------------------------------------
    Effectors are split into two independent lists so that core0 and
    core1 can iterate them simultaneously without a shared lock:

      _steppers  → iterated by core1 (_stepper_loop in BottangoCore)
      _servos    → iterated by core0 (main loop in BottangoCore)

    The only mutual-exclusion requirement is when the pool is being
    structurally modified (register / deregister / clear_all).  A
    _paused flag signals core1 to skip its iteration during those
    operations; a brief sleep gives it time to exit any in-progress
    update before we call destroy().

    Curve-buffer safety (add_curve vs stepper update_on_loop) is handled
    by per-effector locks inside AbstractEffector.
    """

    def __init__(self, max_effectors):
        self.max_effectors = max_effectors

        # Master list (all effectors, used for lookups and registration).
        self.effectors = []

        # Split lists for independent per-core iteration.
        self._steppers = []   # StepDir effectors  → updated on core1
        self._servos   = []   # all other effectors → updated on core0

        # Set False before modifying the pool; core1 checks this flag at
        # the top of each iteration to avoid touching partially-destroyed
        # objects.  Must be set back to True after the modification.
        self._core1_active = True

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------

    def get_effector_by_id(self, identifier):
        for eff in self.effectors:
            if eff.identifier == identifier:
                return eff
        return None

    # ------------------------------------------------------------------
    # Registration / deregistration
    # ------------------------------------------------------------------

    def register_effector(self, effector, type_name):
        existing = self.get_effector_by_id(effector.identifier)
        if existing:
            self.deregister_effector(effector.identifier)

        if len(self.effectors) >= self.max_effectors:
            Outgoing.send_error(EFFECTOR_POOL_FULL, "Pool full")
            return False

        self.effectors.append(effector)
        if type_name == "STEP_DIR":
            self._steppers.append(effector)
        else:
            self._servos.append(effector)

        on_effector_registered(effector.identifier, type_name)
        return True

    def deregister_effector(self, identifier):
        effector = self.get_effector_by_id(identifier)
        if not effector:
            Outgoing.send_error(EFFECTOR_NOT_FOUND, "Not found: {}".format(identifier))
            return False

        # Pause core1 so it does not access the effector during destroy().
        self._pause_core1()
        effector.destroy()
        self.effectors.remove(effector)
        if effector in self._steppers:
            self._steppers.remove(effector)
        elif effector in self._servos:
            self._servos.remove(effector)
        self._resume_core1()

        on_effector_deregistered(identifier)
        return True

    def clear_all(self):
        """Deregister ALL effectors (xE / handshake reset). Mirrors deregisterAll()."""
        self._pause_core1()
        for eff in list(self.effectors):
            eff.destroy()
            on_effector_deregistered(eff.identifier)
        self.effectors = []
        self._steppers = []
        self._servos   = []
        self._resume_core1()

    # ------------------------------------------------------------------
    # Pause / resume helpers for pool modification
    # ------------------------------------------------------------------

    def _pause_core1(self):
        """Signal core1 to stop iterating; wait for it to exit."""
        self._core1_active = False
        # 10 ms is more than enough for core1 to finish one stepper
        # update_on_loop call (typically < 1 ms even with Bezier eval).
        time.sleep_ms(10)

    def _resume_core1(self):
        self._core1_active = True

    # ------------------------------------------------------------------
    # Curve management (mirrors EffectorPool::clearAllCurves)
    # ------------------------------------------------------------------

    def clear_all_curves(self):
        """Clear buffered curves on every effector WITHOUT deregistering them (xC)."""
        for eff in self.effectors:
            eff.clear_curves()

    def clear_effector_curves(self, identifier):
        """Clear curves on a single effector (xUC)."""
        eff = self.get_effector_by_id(identifier)
        if eff:
            eff.clear_curves()

    # ------------------------------------------------------------------
    # Per-core update methods
    # ------------------------------------------------------------------

    def update_steppers(self, current_time_ms):
        """
        Update StepDir effectors only.
        Called from core1 — MUST NOT call Outgoing (sys.stdout not safe
        across cores in MicroPython).
        """
        if not self._core1_active:
            return
        for eff in self._steppers:
            try:
                eff.update_on_loop(current_time_ms)
            except Exception:
                pass   # swallow silently; cannot print from core1

    def update_servos(self, current_time_ms):
        """
        Update all non-StepDir effectors (servos, etc.).
        Called from core0 (main loop).
        """
        for eff in self._servos:
            try:
                eff.update_on_loop(current_time_ms)
            except Exception as e:
                Outgoing.send_log("update err [{}]: {}".format(eff.identifier, e))

    def update_all(self, current_time_ms):
        """
        Update ALL effectors on the calling core.
        Used as fallback when _thread is not available (single-core mode).
        """
        for eff in self.effectors:
            try:
                eff.update_on_loop(current_time_ms)
            except Exception as e:
                Outgoing.send_log("update err [{}]: {}".format(eff.identifier, e))

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    def get_hash(self):
        """Simple hash of registered effector IDs for change detection."""
        if not self.effectors:
            return "0"
        h = 0
        for eff in self.effectors:
            for ch in eff.identifier:
                h += ord(ch)
        return str(h)
