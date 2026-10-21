#!/usr/bin/env python3
"""
Print NSL-3130AA USB serial number to stdout.
Scans /sys/bus/usb/devices/ for VID 1fc9 (NanoSystems), PID 0099 (vendor) or 0094 (original).
Exit 0 with serial on stdout, exit 1 if camera not found.
"""
import sys
from pathlib import Path


def compose_camera_serial(chip_id, wafer_id):
    """The fleet's NSL device serial is N + four hex digits per SDK ID."""
    if any(type(value) is not int or not 0 <= value <= 0xffff for value in (chip_id, wafer_id)):
        raise ValueError('SDK identity fields must be unsigned 16-bit integers')
    if (chip_id, wafer_id) in ((0, 0), (0xffff, 0xffff)):
        raise ValueError('SDK identity fields are uninitialized')
    return f'N{chip_id:04X}{wafer_id:04X}'


def validate_ethernet_identity(record, sensor_host=None):
    if not isinstance(record, dict) or record.get('source') != 'ethernet_sdk_ids':
        raise ValueError('Not a live Ethernet SDK identity observation')
    serial = compose_camera_serial(record.get('chip_id'), record.get('wafer_id'))
    if record.get('serial') != serial:
        raise ValueError('Camera serial does not match its SDK chip/wafer IDs')
    if sensor_host and record.get('sensor_host') != sensor_host:
        raise ValueError('Camera identity was read from a different sensor IP')
    return serial


def detect_camera_serials(usb_sys=Path('/sys/bus/usb/devices')) -> list:
    usb_sys = Path(usb_sys)
    serials = set()
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
                    serials.add(serial)
    except Exception:
        pass
    return sorted(serials)


def detect_camera_serial() -> str:
    serials = detect_camera_serials()
    return serials[0] if len(serials) == 1 else ''


if __name__ == '__main__':
    serial = detect_camera_serial()
    if serial:
        print(serial)
    else:
        print('[detect_camera_id] NSL camera not found (VID 1fc9, PID 0099/0094)', file=sys.stderr)
        sys.exit(1)
