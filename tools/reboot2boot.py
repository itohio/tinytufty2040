"""Put the Tufty into its UF2 bootloader (1200-baud touch). Finds it by USB
serial df6750955f2a3536, or by VID:PID 2e8a:1002 + product "Tufty 2040" for
firmware that doesn't report a serial."""
import os, sys, time
import serial
from serial.tools import list_ports
ports = [p.device for p in list_ports.comports() if p.serial_number == "df6750955f2a3536"] or \
        [p.device for p in list_ports.comports() if p.vid == 0x2E8A and p.pid == 0x1002 and p.product == "Tufty 2040"]
if not ports:
    sys.exit("no Tufty on USB")
serial.Serial(ports[0], 1200).close()
for _ in range(150):
    if os.path.isdir("/Volumes/RPI-RP2"):
        break
    time.sleep(0.1)
time.sleep(2)
print("bootloader", os.path.isdir("/Volumes/RPI-RP2"))
