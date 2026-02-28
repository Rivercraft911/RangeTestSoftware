"""
Serial helpers and RF profile management for LoRa image transfer.
"""
import time

import serial
import serial.tools.list_ports
from rich.console import Console

console = Console(highlight=False)

# Apple logo rainbow (green → yellow → orange → red → purple → blue)
APPLE = ["#61BB46", "#FDB827", "#F5821F", "#E03A3E", "#963D97", "#009DDC"]

IMAGE_DATA_PER_PKT = 245

RF_PROFILES = {
    "1": ("High Speed",   "SF7  / BW 1625 kHz", 1),
    "2": ("Medium Speed", "SF7  / BW 812 kHz",  2),
    "3": ("Long Range",   "SF8  / BW 406 kHz",  3),
    "4": ("Max Range",    "SF10 / BW 203 kHz",  4),
}


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    usb_ports = [p for p in ports if "usb" in p.device.lower()
                 or "acm" in p.device.lower()
                 or "usbmodem" in p.device.lower()]
    if not usb_ports:
        usb_ports = list(ports)
    return usb_ports


def pick_serial_port():
    while True:
        ports = list_serial_ports()
        if not ports:
            console.print("  No serial ports found. Plug in a Pico and press Enter.")
            input()
            continue

        console.print("\n  Available ports:")
        for i, p in enumerate(ports):
            c = APPLE[i % len(APPLE)]
            console.print(f"  [bold {c}]{i + 1})[/]  {p.device}  —  {p.description}")

        choice = input(f"\n  Select port [1-{len(ports)}]: ").strip()
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                return ports[idx].device
        except ValueError:
            pass
        if choice.startswith("/dev/") or choice.startswith("COM"):
            return choice
        console.print("  Invalid selection, try again.")


def open_serial(port, baud=115200):
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.3)
            ser.reset_input_buffer()
            return ser
        except serial.SerialException as e:
            console.print(f"  Could not open {port}: {e}")
            retry = input("  Retry? (y/n): ").strip().lower()
            if retry != "y":
                return None


def read_line(ser, timeout=5):
    if timeout is None:
        while True:
            raw = ser.readline()
            if raw:
                return raw.decode("utf-8", errors="replace")
    else:
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = ser.readline()
            if raw:
                return raw.decode("utf-8", errors="replace")
    return None


def wait_for(ser, prefix, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith(prefix):
                return line
    return None


def set_rf_profile(ser, profile_num):
    ser.write(f"SET_PROFILE,profile={profile_num}\n".encode())
    resp = wait_for(ser, "PROFILE_", timeout=3)
    if resp and "PROFILE_SET" in resp:
        return True
    console.print(f"  Profile change failed: {resp}")
    return False


def pick_rf_profile(ser):
    console.print("\n  RF Profile:")
    for j, (k, (name, desc, _)) in enumerate(RF_PROFILES.items()):
        c = APPLE[j % len(APPLE)]
        tag = " (default)" if k == "1" else ""
        console.print(f"  [bold {c}]{k})[/]  {name:15s} — {desc}{tag}")

    choice = input("\n  Select profile [1-4]: ").strip()
    if choice not in RF_PROFILES:
        choice = "1"
    name, desc, num = RF_PROFILES[choice]
    console.print(f"  Setting: {name} ({desc})")
    set_rf_profile(ser, num)
    return num
