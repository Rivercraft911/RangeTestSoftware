"""
LoRa S-Band image transfer tool.
River Dowdy - Winter 2025

Dependencies: pip install pyserial Pillow
"""
import io
import sys
import time
from pathlib import Path

import serial
from PIL import Image, ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

IMAGE_DATA_PER_PKT = 245

BANNER = r"""
 _     _   _                    ______    ______     _ __                      ______
' )   /   //                      /         //      ' )  )                       /      _/_
 / / /_  // _. __ ______  _    --/__     --//_  _    /--' __.  ____  _,  _    --/_  _   /
(_(_/</_</_(__(_)/ / / <_</_  (_/(_)    (_// /_</_  /  \_(_/|_/ / <_(_)_</_  (_/</_/_)_<__
                                                                     /|
                                                                    |/
"""


def cmd_send(port, baud):
    image_path = input("Image path: ").strip().strip('"').strip("'")
    if not Path(image_path).exists():
        print(f"File not found: {image_path}")
        return

    data = Path(image_path).read_bytes()
    total_pkts = (len(data) + IMAGE_DATA_PER_PKT - 1) // IMAGE_DATA_PER_PKT
    print(f"Image: {image_path} ({len(data)} bytes, {total_pkts} packets)")

    ser = serial.Serial(port, baud, timeout=5)
    time.sleep(0.5)
    ser.reset_input_buffer()

    ser.write(f"IMG_SEND,total_bytes={len(data)},total_pkts={total_pkts}\n".encode())

    if not _wait_for(ser, "IMG_ACK,ready"):
        print("Error: no ready ACK from Pico")
        return

    print("Pico ready, sending...")

    for i in range(total_pkts):
        offset = i * IMAGE_DATA_PER_PKT
        chunk = data[offset:offset + IMAGE_DATA_PER_PKT]
        hex_str = chunk.hex().upper()

        ser.write(f"IMG_CHUNK,pkt={i},len={len(chunk)},hex={hex_str}\n".encode())

        resp = _read_line(ser, timeout=10)
        if resp is None:
            print(f"\nError: timeout waiting for ACK on pkt {i}")
            return
        if "IMG_NACK" in resp:
            print(f"\n  Warning: {resp.strip()}")

        if (i + 1) % 50 == 0 or i == total_pkts - 1:
            pct = 100 * (i + 1) / total_pkts
            print(f"  {i + 1}/{total_pkts} ({pct:.0f}%)")

    ser.write(b"IMG_DONE\n")
    resp = _read_line(ser, timeout=10)
    if resp:
        print(resp.strip())

    ser.close()
    print("Done.")


def cmd_receive(port, baud):
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Listening on {port}...")

    total_pkts = 0
    received = {}
    transfer_num = 0

    while True:
        line = _read_line(ser, timeout=None)
        if line is None:
            continue

        line = line.strip()
        if not line:
            continue

        parts = line.split(",")
        rec = parts[0]
        fields = {}
        for p in parts[1:]:
            if "=" in p:
                k, v = p.split("=", 1)
                fields[k] = v

        if rec == "IMG_START":
            total_pkts = int(fields.get("total_pkts", 0))
            received = {}
            print(f"\nTransfer started: {total_pkts} packets expected")

        elif rec == "IMG_DATA":
            pkt_num = int(fields.get("pkt", 0))
            data_len = int(fields.get("len", 0))
            hex_str = fields.get("hex", "")

            if len(hex_str) == data_len * 2:
                received[pkt_num] = bytes.fromhex(hex_str)
            else:
                print(f"  Warning: pkt {pkt_num} hex length mismatch")

            if len(received) % 50 == 0:
                rssi = fields.get("rssi", "?")
                snr = fields.get("snr", "?")
                pct = 100 * len(received) / max(1, total_pkts)
                print(f"  Progress: {len(received)}/{total_pkts} ({pct:.0f}%) "
                      f"rssi={rssi} snr={snr}")

        elif rec == "IMG_END":
            transfer_num += 1
            rx_count = len(received)
            missing = [i for i in range(total_pkts) if i not in received]

            print(f"\nTransfer complete: {rx_count}/{total_pkts} "
                  f"({100 * rx_count / max(1, total_pkts):.1f}%)")
            if missing:
                shown = missing[:20]
                print(f"  Missing ({len(missing)}): {shown}"
                      f"{'...' if len(missing) > 20 else ''}")

            jpeg = _reassemble(received, total_pkts)
            out_name = f"received_{transfer_num}.jpg"
            Path(out_name).write_bytes(jpeg)
            print(f"  Saved: {out_name} ({len(jpeg)} bytes)")

            img = Image.open(io.BytesIO(jpeg))
            img.show()

        elif rec == "ImageTransfer":
            print("RX firmware ready.")

        elif rec == "ERROR" or rec == "FATAL":
            print(f"  Pico: {line}")


def _reassemble(received, total_pkts):
    parts = []
    for i in range(total_pkts):
        if i in received:
            parts.append(received[i])
        else:
            parts.append(b'\x00' * IMAGE_DATA_PER_PKT)
    return b''.join(parts)


def _read_line(ser, timeout=5):
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


def _wait_for(ser, prefix, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith(prefix):
                return True
    return False


def main():
    print(BANNER)
    print("  1) Send image")
    print("  2) Receive image")
    print()

    choice = input("Select mode: ").strip()

    port = input("Serial port (e.g. /dev/tty.usbmodem101): ").strip()
    baud = 115200

    if choice == "1":
        cmd_send(port, baud)
    elif choice == "2":
        cmd_receive(port, baud)
    else:
        print("Invalid choice.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
