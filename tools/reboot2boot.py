#!/usr/bin/env python3
"""Put the Tufty into its UF2 bootloader (1200-baud touch on its USB serial
port). The badge is found by its exact USB serial number only (never by port
name or product string, so another Tufty or board is never touched); --help
opens no port.

    python3 tools/reboot2boot.py [--serial df6750955f2a3536]

Prints "bootloader True" once the RPI-RP2 drive is mounted (macOS); if the
drive doesn't mount, tools/picoflash flashes over PICOBOOT instead.
"""
import argparse, os, sys, time

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--serial", default="df6750955f2a3536", help="the badge's USB serial number")
ap.add_argument("--volume", default="/Volumes/RPI-RP2", help="where the bootloader drive mounts")
args = ap.parse_args()

import serial
from serial.tools import list_ports

ports = [p.device for p in list_ports.comports() if p.serial_number == args.serial]
if not ports:
    sys.exit(f"no USB serial device with serial number {args.serial}")
serial.Serial(ports[0], 1200).close()
for _ in range(150):
    if os.path.isdir(args.volume):
        break
    time.sleep(0.1)
time.sleep(2)
print("bootloader", os.path.isdir(args.volume))
