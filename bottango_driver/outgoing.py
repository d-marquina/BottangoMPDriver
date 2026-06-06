import sys

class Outgoing:
    @staticmethod
    def _send(message):
        """
        Write a line to USB serial with exactly '\n' terminator — no '\r'.

        MicroPython's print() goes through the REPL layer which converts '\n'
        to '\r\n' on USB CDC (terminal emulation).  Bottango's parser is strict
        about line endings: a trailing '\r' makes the UID value 17 chars instead
        of 16 and fails validation.  Writing to sys.stdout directly bypasses the
        '\r\n' translation.
        """
        sys.stdout.write(message + '\n')

    @staticmethod
    def send_handshake_response(version, random_code):
        Outgoing._send("btngoHSK," + version + "," + random_code)

    @staticmethod
    def send_ready():
        Outgoing._send("OK")

    @staticmethod
    def send_error(code, message=""):
        Outgoing._send("ERR," + str(code) + "," + str(message))

    @staticmethod
    def send_log(message):
        Outgoing._send("LOG," + str(message))

    @staticmethod
    def send_custom_message(message):
        Outgoing._send(message)
