#!/usr/bin/env python3
"""
Plain RFC2217 server using pyserial's standard PortManager.

Required for ESP32-C3 native USB Serial/JTAG (ttyACM) devices,
where DTR/RTS must pass through directly for bootloader entry.

Espressif's esp_rfc2217_server uses EspPortManager which intercepts
DTR/RTS and runs its own reset sequence in a separate thread. This
works for UART bridge chips (ttyUSB / CP2102 / CH340) but breaks
ESP32-C3 native USB because the chip's USB controller handles
bootloader entry internally via DTR/RTS signals.

This server uses pyserial's standard serial.rfc2217.PortManager
which passes DTR/RTS directly to the serial device — exactly what
the C3 native USB needs.

The portal detects ttyACM devices and launches this server instead
of esp_rfc2217_server automatically.
"""
import argparse
import logging
import os
import socket
import termios
import threading
import time

import serial
import serial.rfc2217


def main():
    parser = argparse.ArgumentParser(
        description="Plain RFC2217 server (direct DTR/RTS passthrough)")
    parser.add_argument("SERIALPORT")
    parser.add_argument("-p", "--localport", type=int, default=2217)
    parser.add_argument("-v", "--verbose", dest="verbosity",
                        action="count", default=0)
    parser.add_argument("--tap", default=None, metavar="PATH",
                        help="FIFO path to copy all received serial bytes into")
    args = parser.parse_args()

    level = (logging.WARNING, logging.INFO, logging.DEBUG, logging.NOTSET)[
        min(args.verbosity, 3)]
    logging.basicConfig(format="%(levelname)s: %(message)s",
                        level=logging.INFO)
    logging.getLogger("rfc2217").setLevel(level)

    ser = serial.serial_for_url(args.SERIALPORT, do_not_open=True,
                                exclusive=False)
    ser.baudrate = 115200  # default for ESP32 serial console; RFC2217 clients can override
    ser.timeout = 0.1  # short timeout keeps the reader thread responsive
    ser.dtr = False
    ser.rts = False
    ser.open()
    # Linux CDC ACM driver asserts DTR+RTS on open.  On ESP32-C3 native USB,
    # the USB-Serial/JTAG controller interprets DTR/RTS as reset + boot-mode
    # signals.  DTR=1 → GPIO9 LOW (download mode), RTS=1 → chip in reset.
    #
    # Controlled boot sequence to ensure SPI boot (not download mode):
    #   1. Clear DTR first  → GPIO9 HIGH (SPI boot selected)
    #   2. Brief delay      → let the USB-JTAG controller see DTR=0
    #   3. Clear RTS        → release reset → chip boots in SPI mode
    if hasattr(ser, 'fd'):
        attrs = termios.tcgetattr(ser.fd)
        attrs[2] &= ~termios.HUPCL  # cflag: clear HUPCL
        termios.tcsetattr(ser.fd, termios.TCSANOW, attrs)
    ser.dtr = False          # GPIO9 HIGH — select SPI boot
    time.sleep(0.1)          # Let USB-JTAG controller latch DTR=0
    ser.rts = False          # Release reset — chip boots normally
    time.sleep(0.1)
    settings = ser.get_settings()

    # Open tap FIFO for writing (O_RDWR so it succeeds without a reader present;
    # O_NONBLOCK so writes never stall if the portal's ring buffer is full).
    tap_fd = None
    if args.tap:
        try:
            tap_fd = os.open(args.tap, os.O_RDWR | os.O_NONBLOCK)
        except OSError as e:
            logging.warning("Cannot open tap FIFO %s: %s", args.tap, e)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", args.localport))
    srv.listen(1)
    logging.info("Listening on port %d for %s", args.localport,
                 args.SERIALPORT)

    # Shared state: the currently connected RFC2217 client.
    # The reader thread runs continuously and forwards to the client when present.
    client_lock = threading.Lock()
    client_conn = [None]   # socket or None
    client_pm = [None]     # PortManager or None

    def reader():
        """Always-on serial reader: taps to FIFO and forwards to RFC2217 client."""
        while True:
            try:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                if tap_fd is not None:
                    try:
                        os.write(tap_fd, data)
                    except OSError:
                        pass
                with client_lock:
                    conn = client_conn[0]
                    pm = client_pm[0]
                    if conn is not None and pm is not None:
                        try:
                            conn.sendall(b"".join(pm.escape(data)))
                        except OSError:
                            pass
            except Exception:
                break

    threading.Thread(target=reader, daemon=True).start()

    while True:
        srv.settimeout(5)
        conn = None
        try:
            while conn is None:
                try:
                    conn, addr = srv.accept()
                except TimeoutError:
                    pass
        except KeyboardInterrupt:
            break

        logging.info("Client connected from %s", addr)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        class Sender:
            def write(self_, data):
                try:
                    conn.sendall(data)
                except (BrokenPipeError, OSError):
                    pass

        try:
            pm = serial.rfc2217.PortManager(
                ser, Sender(),
                logger=logging.getLogger("rfc2217") if args.verbosity > 0
                else None,
            )
        except (BrokenPipeError, OSError):
            logging.info("Client disconnected during negotiation")
            conn.close()
            continue

        with client_lock:
            client_conn[0] = conn
            client_pm[0] = pm

        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                ser.write(b"".join(pm.filter(data)))
        except Exception:
            pass

        with client_lock:
            client_conn[0] = None
            client_pm[0] = None

        conn.close()
        logging.info("Client disconnected")
        ser.dtr = False
        ser.rts = False
        ser.apply_settings(settings)


if __name__ == "__main__":
    main()
