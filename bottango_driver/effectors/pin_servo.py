from bottango_driver.effectors.abstract_effector import AbstractEffector, BOTTANGO_MAX_SIGNAL
from machine import Pin, PWM

_PERIOD_US = 20000  # 50 Hz → 20 ms period


class PinServoEffector(AbstractEffector):
    """
    PWM servo effector.  Mirrors Arduino PinServoEffector + LoopDrivenEffector.

    Signal units internally: raw Bottango 0–8192.
    target_signal is in µs (after _lerp_signal maps 0-8192 → [minPWM, maxPWM]).
    """

    def __init__(self, identifier, pin_num, min_pwm_us, max_pwm_us, max_speed, start_val_us):
        # Convert start position (in µs) to Bottango normalised range (0–8192)
        span = max_pwm_us - min_pwm_us
        if span > 0:
            norm_start = int((start_val_us - min_pwm_us) / span * BOTTANGO_MAX_SIGNAL)
        else:
            norm_start = 0
        norm_start = max(0, min(BOTTANGO_MAX_SIGNAL, norm_start))

        super().__init__(identifier, min_pwm_us, max_pwm_us, norm_start, max_speed)

        # target_signal starts at start_val_us (µs), not in 0-8192 units,
        # because _lerp_signal outputs µs for this effector type.
        self.target_signal  = start_val_us
        self.current_signal = start_val_us

        self.pin_num = int(pin_num)

        try:
            self.pwm = PWM(Pin(self.pin_num))
            self.pwm.freq(50)
            self._write_us(start_val_us)
        except Exception as e:
            from bottango_driver.outgoing import Outgoing
            Outgoing.send_log("PWM init error pin {}: {}".format(self.pin_num, e))
            self.pwm = None

    def drive_on_loop(self):
        """Write target_signal (µs) to hardware if it changed."""
        if not self.pwm:
            return
        if self.current_signal != self.target_signal:
            self._write_us(self.target_signal)
            self.current_signal = self.target_signal

    def _write_us(self, us):
        duty = int((us / _PERIOD_US) * 65535)
        self.pwm.duty_u16(duty)

    def destroy(self):
        if self.pwm:
            self.pwm.deinit()
        super().destroy()
