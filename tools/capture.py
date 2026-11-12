#!/usr/bin/env python3
"""Flash a UF2 onto the badge (it must already be in its bootloader, see
reboot2boot.py), catch the boot log, send commands and print everything raw.
The badge is found by its exact USB serial number only; --help touches no
hardware.

    python3 tools/capture.py tufty.uf2 [cmd ...]     # cmd 'sleep:N' reads for N s
"""
import argparse, os, shutil, sys, time

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("uf2", help="firmware to copy to the bootloader drive")
ap.add_argument("cmds", nargs="*", help="console commands; sleep:N waits N seconds while reading")
ap.add_argument("--serial", default="df6750955f2a3536", help="the badge's USB serial number")
ap.add_argument("--volume", default="/Volumes/RPI-RP2", help="where the bootloader drive mounts")
args = ap.parse_args()
if not os.path.isfile(args.uf2) or not args.uf2.endswith(".uf2"):
    sys.exit(f"{args.uf2}: not a .uf2 file")

import serial
from serial.tools import list_ports


def tufty():
    """The badge's port, by exact USB serial number (never by port name)."""
    for p in list_ports.comports():
        if p.serial_number == args.serial:
            return p.device
    return None


if not os.path.isdir(args.volume):
    sys.exit("badge not in bootloader (run tools/reboot2boot.py first)")
shutil.copy(args.uf2, args.volume + "/")
port = None
for _ in range(500):
    port = tufty()
    if port:
        break
    time.sleep(0.02)
if not port:
    sys.exit("the badge did not come back on USB")
time.sleep(0.2)
p = serial.Serial(port, 115200, timeout=0.1)


def pump(secs):
    end = time.time() + secs
    while time.time() < end:
        b = p.read(4096)
        if b:
            sys.stdout.write(b.decode(errors="replace"))
            sys.stdout.flush()


pump(3)
for c in args.cmds:
    if c.startswith("sleep:"):
        pump(float(c[6:]))
        continue
    print(f"\n>>> {c}")
    p.write(c.encode() + b"\n")
    pump(0.6)
