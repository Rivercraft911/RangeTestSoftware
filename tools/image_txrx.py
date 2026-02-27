"""
LoRa S-Band image transfer tool.
  send    - Stream a JPEG to the TX Pico over serial
  receive - Reassemble and display an image from the RX Pico

River Dowdy - Winter 2025
"""
import argparse
import serial
import sys
import time
from pathlib import Path

IMAGE_DATA_PER_PKT = 245


def cmd_send(args):
    data = Path(args.image).read_bytes()
    total_pkts = (len(data) + IMAGE_DATA_PER_PKT - 1) // IMAGE_DATA_PER_PKT

    print(f"Image: {args.image} ({len(data)} bytes, {total_pkts} packets)")

    ser = serial.Serial(args.port, args.baud, timeout=5)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Send start command
    ser.write(f"IMG_SEND,total_bytes={len(data)},total_pkts={total_pkts}\n".encode())

    # Wait for ACK
    if not _wait_for(ser, "IMG_ACK,ready"):
        print("Error: no ready ACK from Pico", file=sys.stderr)
        return

    print("Pico ready, sending...")

    # Stream chunks
    for i in range(total_pkts):
        offset = i * IMAGE_DATA_PER_PKT
        chunk = data[offset:offset + IMAGE_DATA_PER_PKT]
        hex_str = chunk.hex().upper()

        line = f"IMG_CHUNK,pkt={i},len={len(chunk)},hex={hex_str}\n"
        ser.write(line.encode())

        # Wait for ACK
        resp = _read_line(ser, timeout=10)
        if resp is None:
            print(f"\nError: timeout waiting for ACK on pkt {i}", file=sys.stderr)
            return
        if "IMG_NACK" in resp:
            print(f"\n  Warning: {resp.strip()}")

        if (i + 1) % 50 == 0 or i == total_pkts - 1:
            pct = 100 * (i + 1) / total_pkts
            print(f"  {i + 1}/{total_pkts} ({pct:.0f}%)")

    # Signal done
    ser.write(b"IMG_DONE\n")
    resp = _read_line(ser, timeout=10)
    if resp:
        print(resp.strip())

    ser.close()
    print("Done.")


def cmd_receive(args):
    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Listening on {args.port}...")

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

            # Reassemble
            jpeg = _reassemble(received, total_pkts)
            out_name = args.output or f"received_{transfer_num}.jpg"
            Path(out_name).write_bytes(jpeg)
            print(f"  Saved: {out_name} ({len(jpeg)} bytes)")

            # Display
            if not args.no_display:
                _display(jpeg)

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


def _display(jpeg_bytes):
    try:
        from PIL import Image, ImageFile
        import io
        ImageFile.LOAD_TRUNCATED_IMAGES = True
        img = Image.open(io.BytesIO(jpeg_bytes))
        img.show()
    except ImportError:
        print("  Install Pillow to auto-display: pip install Pillow")
    except Exception as e:
        print(f"  Display failed: {e}")


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
    ap = argparse.ArgumentParser(description="LoRa S-Band image transfer")
    sub = ap.add_subparsers(dest="command", required=True)

    tx = sub.add_parser("send", help="Send an image to the TX Pico")
    tx.add_argument("--port", required=True, help="Serial port")
    tx.add_argument("--image", required=True, help="JPEG file to send")
    tx.add_argument("--baud", type=int, default=115200)

    rx = sub.add_parser("receive", help="Receive and display image from RX Pico")
    rx.add_argument("--port", required=True, help="Serial port")
    rx.add_argument("--baud", type=int, default=115200)
    rx.add_argument("--output", default=None, help="Output filename")
    rx.add_argument("--no-display", action="store_true", help="Don't auto-display")

    args = ap.parse_args()
    if args.command == "send":
        cmd_send(args)
    elif args.command == "receive":
        cmd_receive(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
