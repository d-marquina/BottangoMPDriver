from bottango_driver.outgoing import Outgoing
from bottango_driver.modules import generate_module_report
from bottango_driver.curves.bezier import BezierCurve
from bottango_driver.callbacks import on_bottango_connected

class ProtocolHandler:
    def __init__(self, core):
        self.core = core
        self.config = core.config
        self._pending_modules = []

        self.commands = {
            'hRQ':    self.handle_handshake_request,
            'hMOD':   self.handle_modules_request,
            'OK':     self.handle_continue_multimessage,
            'tSYN':   self.handle_time_sync_set,
            'STOP':   self.handle_stop_all,
            'xE':     self.handle_clear_all,
            'rSVPin': self.handle_register_pin_servo,
            'rSVI2C': self.handle_register_i2c_servo,
            'xUE':    self.handle_deregister,
            'xC':     self.handle_clear_curves,
            'xUC':    self.handle_clear_effector_curves,
            'sC':     self.handle_set_curve,
        }

    def process_command(self, line):
        parts = line.split(',')
        if not parts:
            return
        cmd_id = parts[0]
        params = parts[1:]
        handler = self.commands.get(cmd_id)
        send_ready = True
        if handler:
            try:
                result = handler(params)
                if result is False:
                    send_ready = False
            except Exception as e:
                from bottango_driver.errors import INVALID_PARAMS
                Outgoing.send_error(INVALID_PARAMS, "Error in {}: {}".format(cmd_id, str(e)))
        if send_ready:
            Outgoing.send_ready()
            self._emit_next_pending_module()

    # --- handshake ---

    def handle_handshake_request(self, params):
        random_code = params[0] if params else "0"
        self.core.effector_pool.clear_all()
        Outgoing.send_handshake_response(self.config.DRIVER_VERSION, random_code)
        self.core.set_registered(True)
        on_bottango_connected()
        return True

    # --- modules ---

    def handle_modules_request(self, params):
        self._pending_modules = generate_module_report(self.config)
        return True

    def handle_continue_multimessage(self, params):
        return True

    def _emit_next_pending_module(self):
        if self._pending_modules:
            module = self._pending_modules.pop(0)
            Outgoing.send_custom_message(module)

    # --- time sync ---

    def handle_time_sync_set(self, params):
        if len(params) >= 1:
            self.core.time_sync.sync_time(int(params[0]))
        return True

    # --- stop / clear ---

    def handle_stop_all(self, params):
        # STOP → clear curves only (does NOT deregister effectors)
        # mirrors Arduino BasicCommands::clearAllCurves
        self.core.effector_pool.clear_all_curves()
        return True

    def handle_clear_all(self, params):
        # xE → deregister ALL effectors and disconnect
        self.core.effector_pool.clear_all()
        self.core.set_registered(False)
        return True

    def handle_clear_curves(self, params):
        # xC → clear curves on all effectors, keep them registered
        self.core.effector_pool.clear_all_curves()
        return True

    def handle_clear_effector_curves(self, params):
        # xUC,<identifier> → clear curves on one effector
        if len(params) >= 1:
            self.core.effector_pool.clear_effector_curves(params[0])
        return True

    # --- effectors ---

    def handle_deregister(self, params):
        if len(params) >= 1:
            self.core.effector_pool.deregister_effector(params[0])
        return True

    def handle_register_pin_servo(self, params):
        # rSVPin,pinId,minPWM,maxPWM,maxPWMSec,startPWM[,hHASH]
        if len(params) < 5:
            return True
        pin_num   = int(params[0])
        min_pwm   = int(params[1])
        max_pwm   = int(params[2])
        max_speed = int(params[3])
        start_val = int(params[4])
        from bottango_driver.effectors.pin_servo import PinServoEffector
        effector = PinServoEffector(str(pin_num), pin_num, min_pwm, max_pwm,
                                    max_speed, start_val)
        self.core.effector_pool.register_effector(effector, "PIN_SERVO")
        return True

    def handle_register_i2c_servo(self, params):
        # rSVI2C,i2cAddress,channel,minPWM,maxPWM,maxPWMSec,startPWM[,hHASH]
        if not getattr(self.config, 'ENABLE_I2C_SERVOS', False):
            return True
        if len(params) < 6:
            return True
        i2c_address = int(params[0])
        channel     = int(params[1])
        min_pwm     = int(params[2])
        max_pwm     = int(params[3])
        max_speed   = int(params[4])
        start_val   = int(params[5])
        from bottango_driver.effectors.i2c_servo_effector import I2CServoEffector
        effector = I2CServoEffector(self.core.i2c_pool, i2c_address, channel,
                                    min_pwm, max_pwm, max_speed, start_val)
        self.core.effector_pool.register_effector(effector, "I2C_SERVO")
        return True

    # --- curves ---

    def handle_set_curve(self, params):
        # sC,identifier,startTime,duration,startY,cp1x,cp1y,endY,cp2x,cp2y[,hHASH]
        if len(params) < 9:
            return True
        identifier        = params[0]
        start_time_offset = int(params[1])
        duration          = int(params[2])
        start_val         = int(params[3])
        cp1x              = int(params[4])
        cp1y              = int(params[5])
        end_val           = int(params[6])
        cp2x              = int(params[7])
        cp2y              = int(params[8])
        last_sync = self.core.time_sync.get_last_synced_time_ms()
        if start_time_offset < 0 and abs(start_time_offset) > last_sync:
            start_time = 0
        else:
            start_time = last_sync + start_time_offset
        effector = self.core.effector_pool.get_effector_by_id(identifier)
        if not effector:
            return True
        curve = BezierCurve(start_time, duration, start_val, end_val,
                            cp1x, cp1y, cp2x, cp2y)
        effector.add_curve(curve)  # queues into circular buffer (up to 8)
        return True
