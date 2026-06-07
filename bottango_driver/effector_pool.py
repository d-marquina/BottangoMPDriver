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
    structurally modified (register / deregister / clear_all).  Setting
    _core1_active = False signals core1 to skip its iteration; a brief
    sleep ensures it exits any in-progress update before destroy().

    Curve-buffer safety (add_curve vs stepper update_on_loop) is handled
    by per-effector locks inside AbstractEffector.
    """

    def __init__(self, max_effectors):
        self.max_effectors = max_effectors
        self.effectors     = []          # all effectors (lookups / registration)
        self._steppers     = []          # StepDir  → updated on core1
        self._servos       = []          # all other → updated on core0
        self._core1_active = True        # cleared before pool modifications

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------

    def get_effector_by_id(self, identifier):
        return next((e for e in self.effectors if e.identifier == identifier), None)

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
        (self._steppers if type_name == "STEP_DIR" else self._servos).append(effector)
        on_effector_registered(effector.identifier, type_name)
        return True

    def deregister_effector(self, identifier):
        effector = self.get_effector_by_id(identifier)
        if not effector:
            Outgoing.send_error(EFFECTOR_NOT_FOUND, "Not found: {}".format(identifier))
            return False

        self._core1_active = False
        time.sleep_ms(10)
        effector.destroy()
        self.effectors.remove(effector)
        if effector in self._steppers:
            self._steppers.remove(effector)
        elif effector in self._servos:
            self._servos.remove(effector)
        self._core1_active = True

        on_effector_deregistered(identifier)
        return True

    def clear_all(self):
        """Deregister ALL effectors (xE / handshake reset). Mirrors deregisterAll()."""
        self._core1_active = False
        time.sleep_ms(10)
        for eff in list(self.effectors):
            eff.destroy()
            on_effector_deregistered(eff.identifier)
        self.effectors = []
        self._steppers = []
        self._servos   = []
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

    def _run_updates(self, effectors, current_time_ms, log_errors=True):
        """Shared update loop — iterates effectors and swallows exceptions."""
        for eff in effectors:
            try:
                eff.update_on_loop(current_time_ms)
            except Exception as e:
                if log_errors:
                    Outgoing.send_log("update err [{}]: {}".format(eff.identifier, e))

    def update_steppers(self, current_time_ms):
        """Update StepDir effectors only (core1). No serial I/O — not safe across cores."""
        if self._core1_active:
            self._run_updates(self._steppers, current_time_ms, log_errors=False)

    def update_servos(self, current_time_ms):
        """Update all non-StepDir effectors (core0)."""
        self._run_updates(self._servos, current_time_ms)

    def update_all(self, current_time_ms):
        """Update ALL effectors — fallback for single-core mode."""
        self._run_updates(self.effectors, current_time_ms)

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------

    def get_hash(self):
        """Simple hash of registered effector IDs for change detection."""
        return str(sum(ord(c) for e in self.effectors for c in e.identifier)) \
               if self.effectors else "0"
