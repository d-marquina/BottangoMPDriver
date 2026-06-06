import sys
import time
from bottango_driver.time_sync import TimeSync
from bottango_driver.effector_pool import EffectorPool
from bottango_driver.protocol import ProtocolHandler
from bottango_driver.status_lights import StatusLights, Status
from bottango_driver.callbacks import on_bottango_deregistered

class BottangoCore:
    def __init__(self, config):
        self.config = config
        self.time_sync = TimeSync()
        self.effector_pool = EffectorPool(config.MAX_NUM_EFFECTORS)
        self.protocol = ProtocolHandler(self)

        self.status_lights = StatusLights(
            getattr(config, 'STATUS_PIN', 2),
            getattr(config, 'STATUS_NUM_LEDS', 1),
            getattr(config, 'ENABLE_STATUS_LIGHTS', False)
        )

        self.is_registered = False
        self.last_comm_time_ms = time.ticks_ms()

        self.buffer = ""
        self.timeout_ms = getattr(config, 'TIMEOUT_THRESH_MS', 2000)

        self.i2c_pool = None
        if getattr(config, 'ENABLE_I2C_SERVOS', False):
            self._init_i2c_pool(config)

        self._setup_serial()

    def _init_i2c_pool(self, config):
        from bottango_driver.i2c_pool import I2CPool
        from bottango_driver.board_defs import (
            DEFAULT_I2C_ID, DEFAULT_I2C_SDA_PIN, DEFAULT_I2C_SCL_PIN
        )

        # Use board defaults for any pin not explicitly set in Config
        i2c_id  = getattr(config, 'I2C_ID',      None)
        sda_pin = getattr(config, 'I2C_SDA_PIN', None)
        scl_pin = getattr(config, 'I2C_SCL_PIN', None)
        freq    = getattr(config, 'I2C_FREQ',    400000)

        if i2c_id  is None: i2c_id  = DEFAULT_I2C_ID
        if sda_pin is None: sda_pin = DEFAULT_I2C_SDA_PIN
        if scl_pin is None: scl_pin = DEFAULT_I2C_SCL_PIN

        self.i2c_pool = I2CPool(
            i2c_id=i2c_id, sda_pin=sda_pin, scl_pin=scl_pin, freq=freq
        )

    def _setup_serial(self):
        try:
            import uselect
            self.poller = uselect.poll()
            self.poller.register(sys.stdin, uselect.POLLIN)
        except ImportError:
            self.poller = None

    def set_registered(self, status):
        self.is_registered = status
        if status:
            self.status_lights.set_status(Status.CONNECTED)
        else:
            self.status_lights.set_status(Status.NOT_CONNECTED)

    def run(self):
        self.status_lights.set_status(Status.NOT_CONNECTED)

        while True:
            self._read_serial()

            current_time_ms = self.time_sync.get_current_time_ms()
            self.effector_pool.update_all(current_time_ms)

            if self.is_registered and self.timeout_ms > 0:
                now_ms = self.time_sync.get_local_ticks_ms()
                if time.ticks_diff(now_ms, self.last_comm_time_ms) > self.timeout_ms:
                    self.effector_pool.clear_all()
                    self.set_registered(False)
                    on_bottango_deregistered()

    def _read_serial(self):
        if not self.poller:
            return

        # Drain ALL available bytes into the buffer first (prevents UART FIFO
        # overflow during bursts of sC commands from animations).
        while self.poller.poll(0):
            char = sys.stdin.read(1)
            if not char:
                break
            self.buffer += char

        # Process at most ONE complete line per main-loop iteration so that
        # update_all() (PWM writes) is never starved by a burst of commands.
        if '\n' in self.buffer:
            line, self.buffer = self.buffer.split('\n', 1)
            line = line.strip('\r')
            if line:
                self.protocol.process_command(line)
                self.last_comm_time_ms = self.time_sync.get_local_ticks_ms()
