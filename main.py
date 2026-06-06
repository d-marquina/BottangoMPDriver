# BottangoMPDriver - Main Configuration and Entry Point

# -- USER CONFIGURATION --

class Config:
    # Driver version
    DRIVER_VERSION = "0.8.0b1"

    # Communication
    BAUD_RATE = 115200
    COMMAND_BUFF_LEN = 512   # Serial read buffer size (bytes)
    TIMEOUT_THRESH_MS = 2000 # Time before deregistering due to inactivity

    # Capacity
    MAX_NUM_EFFECTORS = 16

    # Enabled modules — set unused ones to False to save memory
    ENABLE_PIN_SERVOS        = True
    ENABLE_PIN_STEPPERS      = False
    ENABLE_STEP_DIR_STEPPERS = False
    ENABLE_CUSTOM_MOTORS     = False
    ENABLE_I2C_SERVOS        = True
    ENABLE_COLOR             = False
    ENABLE_VELOCITY          = False
    ENABLE_LOOP_DRIVEN       = False

    # I2C configuration (only used when ENABLE_I2C_SERVOS = True)
    # Leave as None to use the platform default pins (auto-detected):
    #   Pico / Pico 2  →  SDA=GP4,    SCL=GP5    (I2C bus 0)
    #   ESP32          →  SDA=GPIO21, SCL=GPIO22  (I2C bus 0)
    # Set explicit integer values to override, e.g. I2C_SDA_PIN = 6
    I2C_ID      = None   # I2C bus ID  (None = platform default)
    I2C_SDA_PIN = None   # SDA GPIO pin (None = platform default)
    I2C_SCL_PIN = None   # SCL GPIO pin (None = platform default)
    I2C_FREQ    = 400000 # I2C clock frequency in Hz

    # Custom events
    ENABLE_CURVED_EVENTS  = False
    ENABLE_ONOFF_EVENTS   = False
    ENABLE_TRIGGER_EVENTS = False
    ENABLE_COLOR_EVENTS   = False

    # Unique device identifier
    # When True, the driver reports MOD,UID,<16-char-hex> during the hMOD
    # handshake so Bottango can uniquely identify this controller.
    # Uses machine.unique_id() — always stable (flash/chip ID, not random).
    #   Pico / Pico 2 → 8-byte flash ID  → 16 uppercase hex chars
    #   ESP32         → 6-byte MAC base   → zero-padded to 16 uppercase hex chars
    REPORT_UID = True

    # Extra features
    ENABLE_STATUS_LIGHTS = True
    STATUS_PIN           = 2  # Pin used for NeoPixel/WS2812 if enabled
    STATUS_NUM_LEDS      = 1  # Number of LEDs in the strip


# -- ENTRY POINT --

def start():
    # Import core only when start() is called
    # to avoid memory issues when loading modules
    from bottango_driver.bottango_core import BottangoCore
    from bottango_driver.board_defs import PLATFORM_NAME

    import sys
    sys.stdout.write("BOOT\n")  # '\n' only — avoid MicroPython CDC '\r\n' translation

    core = BottangoCore(Config)
    core.run()


if __name__ == "__main__":
    start()
