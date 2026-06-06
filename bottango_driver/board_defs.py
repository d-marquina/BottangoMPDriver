import sys
import os
import machine

# Identify the platform
PLATFORM = sys.platform

PLATFORM_NAME = "UNKNOWN"
HAS_FPU = True
MAX_PWM_FREQ = 1000
PWM_RESOLUTION = 65535  # 16-bit by default in MicroPython

try:
    _uname = os.uname()
    _machine_str = _uname.machine.lower()

    if "rp2040" in _machine_str or "pico" in _machine_str:
        if "rp2350" in _machine_str or "pico 2" in _machine_str:
            PLATFORM_NAME = "RP2350"
        else:
            PLATFORM_NAME = "RP2040"
    elif "esp32" in _machine_str:
        PLATFORM_NAME = "ESP32"

except AttributeError:
    pass

# ── Default I2C pins per platform ─────────────────────────────────────────────
# These are used when the user leaves I2C_SDA_PIN / I2C_SCL_PIN as None
# in main.py.  Override them there if your board wiring differs.
#
#  RP2040 / RP2350 (Pico, Pico 2)
#    I2C0: SDA=GP4, SCL=GP5  ← most breakout boards use these pads
#    I2C1: SDA=GP6, SCL=GP7  (alternative if GP4/5 are occupied)
#
#  ESP32 (classic / S2 / S3 / C3)
#    Default Arduino-style I2C: SDA=GPIO21, SCL=GPIO22
#    These are the pins used by virtually every ESP32 breakout and shield.
#
if PLATFORM_NAME in ("RP2040", "RP2350"):
    DEFAULT_I2C_ID      = 0
    DEFAULT_I2C_SDA_PIN = 4
    DEFAULT_I2C_SCL_PIN = 5
elif PLATFORM_NAME == "ESP32":
    DEFAULT_I2C_ID      = 0
    DEFAULT_I2C_SDA_PIN = 21
    DEFAULT_I2C_SCL_PIN = 22
else:
    # Safe fallback — matches Pico pinout
    DEFAULT_I2C_ID      = 0
    DEFAULT_I2C_SDA_PIN = 4
    DEFAULT_I2C_SCL_PIN = 5


_UID_FILE = "/bottango_uid.txt"


def get_unique_id():
    """
    Returns a stable 16-character UPPERCASE hex unique ID for this board,
    matching the Arduino driver format (UID_HEX_LEN = 16).

    Strategy — tried in order:

    1. machine.unique_id() if the value is non-zero.
         RP2350 (Pico 2) → 8 bytes from RP2350 OTP (silicon, always reliable)
         ESP32           → 6 bytes from MAC base  (silicon, always reliable),
                           zero-padded on the left to 16 chars.
         RP2040 (Pico)   → 8 bytes from the *external* flash chip (JEDEC 0x4B).
                           Reliable on genuine Pico boards (Winbond W25Q).
                           Some third-party RP2040 boards use cheaper flash that
                           returns all-zeros for this command — detected below.

    2. Persistent random ID stored in the filesystem (/bottango_uid.txt).
         Generated once with os.urandom(8), saved and re-used on every boot.
         Mirrors Arduino's NVS/EEPROM strategy: random but stable per device.
         Triggered when machine.unique_id() returns all zeros or is unavailable.

    3. Emergency fallback: a one-time random string kept in RAM.
         Used only if the filesystem is also unavailable (should never happen).
    """
    # ── 1. Try hardware ID ────────────────────────────────────────────────────
    try:
        uid_bytes = machine.unique_id()
        if any(b != 0 for b in uid_bytes):          # all-zero → unreliable flash
            hex_str = ''.join(['{:02X}'.format(b) for b in uid_bytes])
            return hex_str[:16].zfill(16)           # pad ESP32 (12→16), cap at 16
    except Exception:
        pass

    # ── 2. Persistent random ID (filesystem) ─────────────────────────────────
    return _get_or_create_persistent_uid()


def _get_or_create_persistent_uid():
    """
    Read the stored UID from flash filesystem, or generate + save a new one.
    Mirrors Arduino PersistentConfigUtil::getUID() / esp_fill_random() logic.
    """
    # Try to read an existing UID
    try:
        with open(_UID_FILE, 'r') as f:
            uid = f.read().strip().upper()
        if len(uid) == 16 and all(c in '0123456789ABCDEF' for c in uid):
            return uid
    except Exception:
        pass  # file doesn't exist yet — generate one

    # Generate 8 random bytes (os.urandom uses the hardware RNG on all targets)
    try:
        import uos
        raw = uos.urandom(8)
    except Exception:
        try:
            import os
            raw = os.urandom(8)
        except Exception:
            # Last resort: mix ticks + platform name for some entropy
            import time
            seed = time.ticks_us() ^ hash(PLATFORM_NAME)
            raw = bytes([(seed >> (i * 8)) & 0xFF for i in range(8)])

    uid = ''.join(['{:02X}'.format(b) for b in raw])

    # Persist it so it survives reboots
    try:
        with open(_UID_FILE, 'w') as f:
            f.write(uid)
    except Exception:
        pass  # filesystem write failed — uid is still valid for this session

    return uid


def get_free_ram():
    """Returns approximate free RAM in bytes."""
    import gc
    gc.collect()
    return gc.mem_free()
