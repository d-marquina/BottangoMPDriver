import machine
import time
from bottango_driver.outgoing import Outgoing

class Status:
    NOT_CONNECTED = 0
    CONNECTED = 1
    ERROR = 2

class StatusLights:
    def __init__(self, pin_num, num_leds, enabled=True):
        self.enabled = enabled
        self.status = Status.NOT_CONNECTED
        self.last_update = 0
        self.np = None
        self.num_leds = num_leds
        
        if not self.enabled:
            return
            
        try:
            import neopixel
            self.np = neopixel.NeoPixel(machine.Pin(pin_num), num_leds)
            self.set_status(Status.NOT_CONNECTED)
        except Exception as e:
            Outgoing.send_log(f"Error initializing NeoPixel: {str(e)}")
            self.enabled = False

    def set_status(self, status):
        if not self.enabled or not self.np:
            return
            
        self.status = status
        
        if status == Status.NOT_CONNECTED:
            # Blue
            self._fill((0, 0, 50))
        elif status == Status.CONNECTED:
            # Green
            self._fill((0, 50, 0))
        elif status == Status.ERROR:
            # Red
            self._fill((50, 0, 0))
            
    def _fill(self, color):
        for i in range(self.num_leds):
            self.np[i] = color
        self.np.write()
