"""
I2C Servo Effector — drives a servo channel on a PCA9685 I2C PWM expander.

Mirrors Arduino I2CServoEffector + LoopDrivenEffector.

Identifier format (matches Bottango): str(i2c_address) + str(channel)
  e.g. address=64 (0x40), channel=0  →  identifier = "640"
       address=64,          channel=15 →  identifier = "6415"

Signal units (after AbstractEffector._lerp_signal):
  target_signal / current_signal are in µs, range [min_pwm_us, max_pwm_us].
"""

from bottango_driver.effectors.abstract_effector import AbstractEffector, BOTTANGO_MAX_SIGNAL


class I2CServoEffector(AbstractEffector):

    def __init__(self, i2c_pool, i2c_address, channel,
                 min_pwm_us, max_pwm_us, max_speed, start_val_us):

        identifier = str(i2c_address) + str(channel)

        # Convert start_val_us → normalised 0–8192 for AbstractEffector.__init__
        span = max_pwm_us - min_pwm_us
        if span > 0:
            norm_start = int((start_val_us - min_pwm_us) / span * BOTTANGO_MAX_SIGNAL)
        else:
            norm_start = 0
        norm_start = max(0, min(BOTTANGO_MAX_SIGNAL, norm_start))

        super().__init__(identifier, min_pwm_us, max_pwm_us, norm_start, max_speed)

        # Override signal state to µs (same pattern as PinServoEffector)
        self.target_signal  = start_val_us
        self.current_signal = start_val_us

        self.i2c_pool    = i2c_pool
        self.i2c_address = i2c_address
        self.channel     = channel
        self.driver      = None

        try:
            self.driver = i2c_pool.acquire(i2c_address)
            self._write_us(start_val_us)
        except Exception as e:
            from bottango_driver.outgoing import Outgoing
            Outgoing.send_log(
                "I2CServo init err addr={} ch={}: {}".format(i2c_address, channel, e)
            )

    # ── AbstractEffector interface ────────────────────────────────────────────

    def drive_on_loop(self):
        """Write to hardware only when target changed (mirrors PinServoEffector)."""
        if self.driver is None:
            return
        if self.current_signal != self.target_signal:
            self._write_us(self.target_signal)
            self.current_signal = self.target_signal

    def destroy(self):
        if self.driver is not None:
            try:
                mid = (self.min_signal + self.max_signal) // 2
                self._write_us(mid)
            except Exception:
                pass
            self.i2c_pool.release(self.i2c_address)
            self.driver = None
        super().destroy()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _write_us(self, us):
        """Write microseconds to the PCA9685 channel."""
        self.driver.write_microseconds(self.channel, us)
