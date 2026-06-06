"""
I2C PWM driver pool.

Multiple servo effectors on the same physical board share one I2CPWMDriver
instance.  A driver is created on first use and torn down (all channels set
to 0) when the last servo that references it is deregistered.

Usage
-----
    from bottango_driver.i2c_pool import I2CPool

    pool = I2CPool(i2c_id=0, sda_pin=4, scl_pin=5, freq=400_000)

    driver = pool.acquire(address=0x40)      # ref-count +1
    driver.write_microseconds(channel=0, us=1500)

    pool.release(address=0x40)               # ref-count -1; destroys when 0
"""

from machine import I2C, Pin
from bottango_driver.i2c_pwm_driver import I2CPWMDriver

_MAX_DRIVERS = 3   # mirrors the Arduino driver limit


class I2CPool:
    def __init__(self, i2c_id=0, sda_pin=4, scl_pin=5, freq=400_000):
        self._i2c = I2C(i2c_id, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=freq)
        # { address(int): {"driver": I2CPWMDriver, "count": int} }
        self._drivers = {}

    def acquire(self, address):
        """Return the I2CPWMDriver for *address*, creating it if needed."""
        if address not in self._drivers:
            if len(self._drivers) >= _MAX_DRIVERS:
                raise RuntimeError(
                    "I2CPool full: max {} boards supported".format(_MAX_DRIVERS)
                )
            driver = I2CPWMDriver(self._i2c, address=address, freq=50)
            self._drivers[address] = {"driver": driver, "count": 0}

        self._drivers[address]["count"] += 1
        return self._drivers[address]["driver"]

    def release(self, address):
        """Decrement ref-count; destroy the driver when it reaches 0."""
        if address not in self._drivers:
            return
        entry = self._drivers[address]
        entry["count"] -= 1
        if entry["count"] <= 0:
            try:
                entry["driver"].set_all_off()
            except Exception:
                pass
            del self._drivers[address]

    def get(self, address):
        """Return the existing driver for *address*, or None."""
        entry = self._drivers.get(address)
        return entry["driver"] if entry else None
