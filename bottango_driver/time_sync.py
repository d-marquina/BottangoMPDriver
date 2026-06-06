import time

class TimeSync:
    def __init__(self):
        self.time_offset_ms = 0
        self.last_syncd_time = 0

    def sync_time(self, incoming_time_ms):
        """Sets the time based on tSYN command"""
        self.last_syncd_time = incoming_time_ms
        self.time_offset_ms = incoming_time_ms - time.ticks_ms()

    def get_current_time_ms(self):
        """Returns the current synchronized time in milliseconds"""
        return time.ticks_add(time.ticks_ms(), self.time_offset_ms)

    def get_last_synced_time_ms(self):
        """Returns the time of the last sync command"""
        return self.last_syncd_time

    def get_local_ticks_ms(self):
        """Returns the local hardware ticks"""
        return time.ticks_ms()
