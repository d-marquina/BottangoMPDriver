from bottango_driver.errors import EFFECTOR_POOL_FULL, EFFECTOR_NOT_FOUND
from bottango_driver.outgoing import Outgoing
from bottango_driver.callbacks import on_effector_registered, on_effector_deregistered


class EffectorPool:
    def __init__(self, max_effectors):
        self.max_effectors = max_effectors
        self.effectors = []

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
        on_effector_registered(effector.identifier, type_name)
        return True

    def deregister_effector(self, identifier):
        effector = self.get_effector_by_id(identifier)
        if not effector:
            Outgoing.send_error(EFFECTOR_NOT_FOUND, "Not found: {}".format(identifier))
            return False

        effector.destroy()
        self.effectors.remove(effector)
        on_effector_deregistered(identifier)
        return True

    def clear_all(self):
        """Deregister ALL effectors (xE / handshake reset). Mirrors deregisterAll()."""
        for eff in list(self.effectors):
            eff.destroy()
            on_effector_deregistered(eff.identifier)
        self.effectors = []

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
    # Main loop
    # ------------------------------------------------------------------

    def update_all(self, current_time_ms):
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
