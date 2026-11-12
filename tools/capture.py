"""Flash tufty.uf2 (badge must be in RPI-RP2 mode), catch the boot log, send
commands and print everything raw. Usage: capture.py [uf2] cmd ...;
cmd 'sleep:N' waits N seconds while reading."""
import os, shutil, sys, time
import serial
from serial.tools import list_ports


def tufty():
    """The badge's port, found by USB serial number (never by port name), or
    by VID:PID + product for firmware that doesn't report the serial."""
    ports = list_ports.comports()
    for p in ports:
        if p.serial_number == "df6750955f2a3536":
            return p.device
    for p in ports:
        if p.vid == 0x2E8A and p.product == "Tufty 2040":
            return p.device
    return None


uf2, cmds = sys.argv[1], sys.argv[2:]
assert os.path.isdir("/Volumes/RPI-RP2"), "badge not in bootloader"
shutil.copy(uf2, "/Volumes/RPI-RP2/")
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
for c in cmds:
    if c.startswith("sleep:"):
        pump(float(c[6:]))
        continue
    print(f"\n>>> {c}")
    p.write(c.encode() + b"\n")
    pump(0.6)
