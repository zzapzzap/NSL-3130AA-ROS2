#!/usr/bin/env python3
"""
Print NSL-3130AA USB serial number to stdout.
Scans /sys/bus/usb/devices/ for VID 1fc9 (NanoSystems), PID 0099 (vendor) or 0094 (original).
Exit 0 with serial on stdout, exit 1 if camera not found.
"""
import sys
from pathlib import Path


def detect_camera_serial() -> str:
    usb_sys = Path('/sys/bus/usb/devices')
    try:
        for dev in usb_sys.iterdir():
            vid_f = dev / 'idVendor'
            pid_f = dev / 'idProduct'
            ser_f = dev / 'serial'
            if not vid_f.exists() or not ser_f.exists():
                continue
            vid = vid_f.read_text().strip()
            pid = pid_f.read_text().strip() if pid_f.exists() else ''
            if vid == '1fc9' and pid in ('0099', '0094'):
                serial = ser_f.read_text().strip()
                if serial:
                    return serial
    except Exception:
        pass
    return ''


if __name__ == '__main__':
    serial = detect_camera_serial()
    if serial:
        print(serial)
    else:
        print('[detect_camera_id] NSL camera not found (VID 1fc9, PID 0099/0094)', file=sys.stderr)
        sys.exit(1)
