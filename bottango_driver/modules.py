from bottango_driver.board_defs import get_unique_id


def generate_module_report(config):
    """
    Returns the ordered list of MOD lines to send during hMOD.
    One entry per page — the protocol sends them one at a time,
    waiting for an OK from Bottango between each.

    Order mirrors Arduino ModulesResponder (case 0 first):
      0  UID        (if REPORT_UID)
      1  CMD_SRC    always
      2  CMD_CFG    always
      3  9685       (if ENABLE_I2C_SERVOS)
      n  EoM        always last
    """
    reports = []

    # ── Module 0: Unique device ID ─────────────────────────────────────────────
    # Mirrors Arduino sendUIDResponse() / #define REPORT_UID
    # Format: MOD,UID,<16 uppercase hex chars>
    #
    # machine.unique_id() sizes per platform:
    #   RP2040 (Pico)  → 8 bytes → 16 hex chars  (flash chip 64-bit factory ID)
    #   RP2350 (Pico2) → 8 bytes → 16 hex chars  (RP2350 chip unique ID)
    #   ESP32          → 6 bytes → 12 hex chars   (MAC base, zero-padded to 16)
    if getattr(config, 'REPORT_UID', False):
        reports.append("MOD,UID," + get_unique_id())

    # ── Module 1: Command source ───────────────────────────────────────────────
    # 0 = Live Only Mode  (no SD card / exported animations)
    reports.append("MOD,CMD_SRC,0")

    # ── Module 2: Command configuration ───────────────────────────────────────
    # maxLen, maxCurves, allowSync, signalBits
    # allowSync=1 when step/dir steppers are enabled (enables homing UI in Bottango).
    # MicroPython on RP2040/ESP32 uses 32-bit ints → signalBits = 32
    allow_sync = 1 if getattr(config, 'ENABLE_STEP_DIR_STEPPERS', False) else 0
    reports.append("MOD,CMD_CFG,248,8,{},32".format(allow_sync))

    # ── Module 3: PCA9685 I2C servo expander ──────────────────────────────────
    # Bottango requires this entry to allow adding I2C servos in the UI.
    if getattr(config, 'ENABLE_I2C_SERVOS', False):
        reports.append("MOD,9685")

    # ── End of modules — must always be last ──────────────────────────────────
    reports.append("MOD,EoM")

    return reports
