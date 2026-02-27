"""
LoRa S-Band image transfer tool.
River Dowdy - Winter 2025

Dependencies: pip install pyserial Pillow matplotlib tqdm
"""
import io
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports
from PIL import Image, ImageFile
from tqdm import tqdm

ImageFile.LOAD_TRUNCATED_IMAGES = True

IMAGE_DATA_PER_PKT = 245
ARQ_MAX_ROUNDS = 3

BANNER = r"""                    .                            .--------'  .--------'             .-.                                  .--------'
..-.     .-.        /                            (_)   /     (_)   /    /           (_) )-.                              (_)   /            /
   )   (    .-.   / .-.  .-._..  .-. .-.   .-.       /.-._.      /    /-.   .-.       /   \  .-.  .  .-.    .-.    .-.       /  .-.  . ---/---
  /     \ ./.-'_ / (    (   )  )/   )   )./.-'_     /(   )      /    /   |./.-'_     /     )(  |   )/   )  (   ) ./.-'_     / ./.-'_/ \  /
 (   .   )(__.'_/_.-`---'`-'  '/   /   ( (__.'   .-/._`-'    .-/.__.'    |(__.'   .-/  `--'  `-'-''/   (    `-/-'(__.'   .-/._(__.'/ ._)/
  `-' `-'                               `-'     (_/  `-     (_/  `-              (_/     `-._)          `--._/          (_/  `-   /             """

RF_PROFILES = {
    "1": ("High Speed",   "SF7  / BW 1625 kHz", 1),
    "2": ("Medium Speed", "SF7  / BW 812 kHz",  2),
    "3": ("Long Range",   "SF8  / BW 406 kHz",  3),
    "4": ("Max Range",    "SF10 / BW 203 kHz",  4),
}

# -- Rosé Pine palette --
RP_BASE     = "#191724"
RP_SURFACE  = "#1f1d2e"
RP_OVERLAY  = "#26233a"
RP_TEXT     = "#e0def4"
RP_SUBTLE   = "#6e6a86"
RP_MUTED    = "#908caa"
RP_ROSE     = "#ebbcba"
RP_PINE     = "#31748f"
RP_GOLD     = "#f6c177"
RP_IRIS     = "#c4a7e7"
RP_LOVE     = "#eb6f92"
RP_FOAM     = "#9ccfd8"


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

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
            print("  No serial ports found. Plug in a Pico and press Enter.")
            input()
            continue

        print("\n  Available ports:")
        for i, p in enumerate(ports):
            print(f"    {i + 1}) {p.device}  — {p.description}")

        choice = input(f"\n  Select port [1-{len(ports)}]: ").strip()
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(ports):
                return ports[idx].device
        except ValueError:
            pass
        # Allow direct path entry
        if choice.startswith("/dev/") or choice.startswith("COM"):
            return choice
        print("  Invalid selection, try again.")


def open_serial(port, baud=115200):
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            time.sleep(0.3)
            ser.reset_input_buffer()
            return ser
        except serial.SerialException as e:
            print(f"  Could not open {port}: {e}")
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
    print(f"  Profile change failed: {resp}")
    return False


# ---------------------------------------------------------------------------
# Profile selection
# ---------------------------------------------------------------------------

def pick_rf_profile(ser):
    print("\n  RF Profile:")
    for k, (name, desc, _) in RF_PROFILES.items():
        tag = " (default)" if k == "1" else ""
        print(f"    {k}) {name:15s} — {desc}{tag}")

    choice = input("\n  Select profile [1-4]: ").strip()
    if choice not in RF_PROFILES:
        choice = "1"
    name, desc, num = RF_PROFILES[choice]
    print(f"  Setting: {name} ({desc})")
    set_rf_profile(ser, num)
    return num


# ---------------------------------------------------------------------------
# Send
# ---------------------------------------------------------------------------

def cmd_send(ser):
    profile = pick_rf_profile(ser)

    while True:
        image_path = input("\n  Image path: ").strip().strip('"').strip("'")
        if Path(image_path).exists():
            break
        print(f"  File not found: {image_path}")

    data = Path(image_path).read_bytes()
    total_pkts = (len(data) + IMAGE_DATA_PER_PKT - 1) // IMAGE_DATA_PER_PKT
    print(f"  Image: {Path(image_path).name} ({len(data):,} bytes, {total_pkts} packets)")

    ser.reset_input_buffer()
    ser.write(f"IMG_SEND,total_bytes={len(data)},total_pkts={total_pkts}\n".encode())

    resp = wait_for(ser, "IMG_ACK,ready", timeout=5)
    if not resp:
        print("  Error: no ready ACK from Pico")
        return

    start_time = time.time()
    failed_pkts = []

    bar = tqdm(total=total_pkts, unit="pkt", desc="  TX",
               bar_format="  {l_bar}{bar:30}{r_bar}",
               colour="#ebbcba", ncols=80)

    for i in range(total_pkts):
        offset = i * IMAGE_DATA_PER_PKT
        chunk = data[offset:offset + IMAGE_DATA_PER_PKT]
        hex_str = chunk.hex().upper()

        ser.write(f"IMG_CHUNK,pkt={i},len={len(chunk)},hex={hex_str}\n".encode())

        resp = read_line(ser, timeout=10)
        if resp is None:
            bar.write(f"  Timeout on pkt {i}")
            failed_pkts.append(i)
        elif "IMG_NACK" in resp:
            failed_pkts.append(i)

        elapsed = time.time() - start_time
        kbps = ((i + 1) * IMAGE_DATA_PER_PKT / 1024) / max(0.001, elapsed)
        bar.set_postfix_str(f"{kbps:.1f} KB/s")
        bar.update(1)

    bar.close()

    ser.write(b"IMG_DONE\n")
    resp = read_line(ser, timeout=10)
    if resp:
        print(f"  {resp.strip()}")

    elapsed = time.time() - start_time

    # ARQ: wait for retransmission requests
    for arq_round in range(ARQ_MAX_ROUNDS):
        resp = read_line(ser, timeout=2)
        if not resp:
            break
        line = resp.strip()

        if "ARQ_DONE" in line:
            break

        if "RETX_REQ" in line:
            # Parse missing packet list
            parts = line.split(",")
            fields = {}
            pkts_str = ""
            for p in parts:
                if "=" in p:
                    k, v = p.split("=", 1)
                    fields[k] = v
                    if k == "pkts":
                        pkts_str = v
            # pkts might span into subsequent comma-separated values
            # Re-parse: everything after "pkts=" is the list
            idx = line.find("pkts=")
            if idx >= 0:
                pkts_str = line[idx + 5:]

            missing = [int(x) for x in pkts_str.split(",") if x.strip().isdigit()]
            print(f"  ARQ round {arq_round + 1}: retransmitting {len(missing)} packets")

            retx_bar = tqdm(total=len(missing), unit="pkt", desc="  RETX",
                            bar_format="  {l_bar}{bar:30}{r_bar}",
                            colour="#f6c177", ncols=80)

            for pkt_num in missing:
                offset = pkt_num * IMAGE_DATA_PER_PKT
                chunk = data[offset:offset + IMAGE_DATA_PER_PKT]
                hex_str = chunk.hex().upper()
                ser.write(f"IMG_CHUNK,pkt={pkt_num},len={len(chunk)},hex={hex_str}\n".encode())
                resp = read_line(ser, timeout=10)
                retx_bar.update(1)

            retx_bar.close()
            ser.write(b"RETX_DONE\n")

            # Read the ARQ round complete message
            resp = read_line(ser, timeout=5)
            if resp:
                print(f"  {resp.strip()}")

    total_elapsed = time.time() - start_time
    avg_kbps = (len(data) / 1024) / max(0.001, total_elapsed)
    print(f"\n  Transfer complete: {total_elapsed:.1f}s, {avg_kbps:.1f} KB/s avg")
    if failed_pkts:
        print(f"  Initial failures: {len(failed_pkts)} packets")


# ---------------------------------------------------------------------------
# Receive (with live plots + partial image preview)
# ---------------------------------------------------------------------------

def cmd_receive(ser):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    profile = pick_rf_profile(ser)
    print(f"\n  Listening for incoming transfers...")

    transfer_num = 0

    while True:
        # Wait for a transfer to start
        total_pkts = 0
        received = {}
        rssi_data = []
        snr_data = []
        pkt_times = []
        start_time = None
        retx_count = 0

        # Set up live plot figure
        fig = None
        ax_rssi = None
        ax_snr = None
        ax_thru = None
        ax_stats = None
        img_fig = None
        img_ax = None
        bar = None

        while True:
            line = read_line(ser, timeout=None)
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
                rssi_data = []
                snr_data = []
                pkt_times = []
                retx_count = 0
                start_time = time.time()
                transfer_num += 1
                print(f"\n  Transfer #{transfer_num}: {total_pkts} packets expected")

                bar = tqdm(total=total_pkts, unit="pkt", desc="  RX",
                           bar_format="  {l_bar}{bar:30}{r_bar}",
                           colour="#9ccfd8", ncols=80)

                # Init live plots
                plt.ion()
                fig = plt.figure(figsize=(12, 8), facecolor=RP_BASE)
                fig.canvas.manager.set_window_title(f"Transfer #{transfer_num} — Live Stats")
                gs = GridSpec(2, 2, figure=fig, hspace=0.35, wspace=0.3)

                ax_rssi = fig.add_subplot(gs[0, 0])
                ax_snr = fig.add_subplot(gs[0, 1])
                ax_thru = fig.add_subplot(gs[1, 0])
                ax_stats = fig.add_subplot(gs[1, 1])

                for ax in [ax_rssi, ax_snr, ax_thru, ax_stats]:
                    ax.set_facecolor(RP_SURFACE)
                    ax.tick_params(colors=RP_MUTED, labelsize=8)
                    ax.spines["bottom"].set_color(RP_OVERLAY)
                    ax.spines["left"].set_color(RP_OVERLAY)
                    ax.spines["top"].set_visible(False)
                    ax.spines["right"].set_visible(False)
                    ax.xaxis.label.set_color(RP_TEXT)
                    ax.yaxis.label.set_color(RP_TEXT)
                    ax.title.set_color(RP_TEXT)
                    ax.grid(True, color=RP_OVERLAY, alpha=0.5, linewidth=0.5)

                ax_rssi.set_title("RSSI (dBm)", fontsize=10, fontweight="bold")
                ax_rssi.set_xlabel("Packet #", fontsize=8)
                ax_snr.set_title("SNR (dB)", fontsize=10, fontweight="bold")
                ax_snr.set_xlabel("Packet #", fontsize=8)
                ax_thru.set_title("Throughput (KB/s)", fontsize=10, fontweight="bold")
                ax_thru.set_xlabel("Packet #", fontsize=8)
                ax_stats.set_title("Packet Stats", fontsize=10, fontweight="bold")

                fig.canvas.draw()
                fig.canvas.flush_events()

                # Init partial image preview
                img_fig = plt.figure(figsize=(6, 6), facecolor=RP_BASE)
                img_fig.canvas.manager.set_window_title("Image Preview")
                img_ax = img_fig.add_subplot(111)
                img_ax.set_facecolor(RP_BASE)
                img_ax.axis("off")
                img_fig.canvas.draw()
                img_fig.canvas.flush_events()

            elif rec == "IMG_DATA":
                pkt_num = int(fields.get("pkt", 0))
                data_len = int(fields.get("len", 0))
                hex_str = fields.get("hex", "")
                rssi_raw = int(fields.get("rssi", 0))
                snr_raw = int(fields.get("snr", 0))

                rssi_dbm = rssi_raw / 100.0
                snr_db = snr_raw / 100.0

                if len(hex_str) == data_len * 2:
                    received[pkt_num] = bytes.fromhex(hex_str)
                else:
                    if bar:
                        bar.write(f"  Warning: pkt {pkt_num} hex length mismatch")

                rssi_data.append(rssi_dbm)
                snr_data.append(snr_db)
                pkt_times.append(time.time() - start_time if start_time else 0)

                if bar:
                    bar.set_postfix_str(f"RSSI={rssi_dbm:.0f}dBm SNR={snr_db:.1f}dB")
                    bar.update(1)

                # Update live plots every 5 packets
                n = len(rssi_data)
                if fig and n % 5 == 0:
                    _update_live_plots(ax_rssi, ax_snr, ax_thru, ax_stats,
                                       rssi_data, snr_data, pkt_times,
                                       len(received), total_pkts, retx_count)
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

                # Update image preview every 20 packets
                if img_fig and n % 20 == 0 and len(received) > 10:
                    _update_image_preview(img_ax, received, total_pkts)
                    img_fig.canvas.draw_idle()
                    img_fig.canvas.flush_events()

            elif rec == "IMG_END":
                if bar:
                    bar.close()
                    bar = None

                rx_count = len(received)
                missing = [i for i in range(total_pkts) if i not in received]
                elapsed = time.time() - start_time if start_time else 0

                print(f"\n  Received: {rx_count}/{total_pkts} "
                      f"({100 * rx_count / max(1, total_pkts):.1f}%) "
                      f"in {elapsed:.1f}s")

                # ARQ: request retransmission of missing packets
                if missing and len(missing) < total_pkts:
                    arq_round = 0
                    while missing and arq_round < ARQ_MAX_ROUNDS:
                        arq_round += 1
                        print(f"  ARQ round {arq_round}: requesting {len(missing)} packets")

                        # Send NACK via Pico -> LoRa
                        # Can fit ~120 per NACK packet
                        batch_size = 120
                        for batch_start in range(0, len(missing), batch_size):
                            batch = missing[batch_start:batch_start + batch_size]
                            nack_str = ",".join(str(x) for x in batch)
                            ser.write(f"IMG_NACK_SEND,missing={nack_str}\n".encode())
                            resp = read_line(ser, timeout=5)
                            if resp:
                                resp = resp.strip()
                                if "NACK_ERR" in resp:
                                    print(f"  NACK send failed: {resp}")

                        # Wait for retransmitted packets
                        retx_bar = tqdm(total=len(missing), unit="pkt", desc="  ARQ",
                                        bar_format="  {l_bar}{bar:30}{r_bar}",
                                        colour="#f6c177", ncols=80)
                        retx_deadline = time.time() + 30
                        retx_got = 0
                        while time.time() < retx_deadline:
                            rline = read_line(ser, timeout=2)
                            if rline is None:
                                continue
                            rline = rline.strip()
                            if not rline:
                                continue

                            rparts = rline.split(",")
                            rrec = rparts[0]
                            rfields = {}
                            for rp in rparts[1:]:
                                if "=" in rp:
                                    rk, rv = rp.split("=", 1)
                                    rfields[rk] = rv

                            if rrec == "IMG_DATA":
                                rpn = int(rfields.get("pkt", 0))
                                rdl = int(rfields.get("len", 0))
                                rhex = rfields.get("hex", "")
                                rrssi = int(rfields.get("rssi", 0))
                                rsnr = int(rfields.get("snr", 0))

                                if len(rhex) == rdl * 2:
                                    received[rpn] = bytes.fromhex(rhex)
                                    retx_count += 1
                                    retx_got += 1

                                rssi_data.append(rrssi / 100.0)
                                snr_data.append(rsnr / 100.0)
                                pkt_times.append(time.time() - start_time)
                                retx_bar.update(1)

                            elif rrec == "IMG_END":
                                break

                        retx_bar.close()

                        missing = [i for i in range(total_pkts) if i not in received]
                        if not missing:
                            print(f"  ARQ complete: all packets received!")
                            break
                        print(f"  Still missing: {len(missing)} packets")

                # Final stats
                rx_count = len(received)
                total_elapsed = time.time() - start_time if start_time else 0
                avg_kbps = (rx_count * IMAGE_DATA_PER_PKT / 1024) / max(0.001, total_elapsed)
                loss_pct = 100 * (total_pkts - rx_count) / max(1, total_pkts)

                print(f"\n  Final: {rx_count}/{total_pkts} packets "
                      f"({100 * rx_count / max(1, total_pkts):.1f}%)")
                print(f"  Time: {total_elapsed:.1f}s | Avg: {avg_kbps:.1f} KB/s | "
                      f"Loss: {loss_pct:.1f}% | Retransmitted: {retx_count}")

                if rssi_data:
                    print(f"  RSSI: min={min(rssi_data):.0f} max={max(rssi_data):.0f} "
                          f"avg={sum(rssi_data)/len(rssi_data):.0f} dBm")
                if snr_data:
                    print(f"  SNR:  min={min(snr_data):.1f} max={max(snr_data):.1f} "
                          f"avg={sum(snr_data)/len(snr_data):.1f} dB")

                # Save image
                jpeg = _reassemble(received, total_pkts)
                out_name = f"received_{transfer_num}.jpg"
                Path(out_name).write_bytes(jpeg)
                print(f"  Saved: {out_name} ({len(jpeg):,} bytes)")

                # Final plot update
                if fig:
                    _update_live_plots(ax_rssi, ax_snr, ax_thru, ax_stats,
                                       rssi_data, snr_data, pkt_times,
                                       rx_count, total_pkts, retx_count)
                    fig.canvas.draw()
                    fig.canvas.flush_events()

                    plot_name = f"received_{transfer_num}_stats.png"
                    fig.savefig(plot_name, facecolor=RP_BASE, dpi=150, bbox_inches="tight")
                    print(f"  Stats plot: {plot_name}")

                # Show final image
                if img_fig:
                    _update_image_preview(img_ax, received, total_pkts)
                    img_fig.canvas.draw()
                    img_fig.canvas.flush_events()

                try:
                    img = Image.open(io.BytesIO(jpeg))
                    img.show()
                except Exception:
                    pass

                print("\n  Waiting for next transfer... (Ctrl+C to quit)")

            elif rec == "ImageTransfer":
                print("  Pico ready.")
            elif rec == "PROFILE_SET":
                pass
            elif rec in ("ERROR", "FATAL"):
                print(f"  Pico: {line}")


def _update_live_plots(ax_rssi, ax_snr, ax_thru, ax_stats,
                       rssi_data, snr_data, pkt_times,
                       rx_count, total_pkts, retx_count):
    x = list(range(len(rssi_data)))

    ax_rssi.clear()
    ax_rssi.set_facecolor(RP_SURFACE)
    ax_rssi.grid(True, color=RP_OVERLAY, alpha=0.5, linewidth=0.5)
    ax_rssi.set_title("RSSI (dBm)", fontsize=10, fontweight="bold", color=RP_TEXT)
    ax_rssi.set_xlabel("Packet #", fontsize=8, color=RP_TEXT)
    ax_rssi.tick_params(colors=RP_MUTED, labelsize=8)
    ax_rssi.plot(x, rssi_data, color=RP_ROSE, linewidth=1, alpha=0.9)
    if rssi_data:
        ax_rssi.axhline(y=sum(rssi_data)/len(rssi_data), color=RP_ROSE,
                        linewidth=0.8, linestyle="--", alpha=0.5)

    ax_snr.clear()
    ax_snr.set_facecolor(RP_SURFACE)
    ax_snr.grid(True, color=RP_OVERLAY, alpha=0.5, linewidth=0.5)
    ax_snr.set_title("SNR (dB)", fontsize=10, fontweight="bold", color=RP_TEXT)
    ax_snr.set_xlabel("Packet #", fontsize=8, color=RP_TEXT)
    ax_snr.tick_params(colors=RP_MUTED, labelsize=8)
    ax_snr.plot(x, snr_data, color=RP_PINE, linewidth=1, alpha=0.9)
    if snr_data:
        ax_snr.axhline(y=sum(snr_data)/len(snr_data), color=RP_PINE,
                       linewidth=0.8, linestyle="--", alpha=0.5)

    # Rolling throughput
    ax_thru.clear()
    ax_thru.set_facecolor(RP_SURFACE)
    ax_thru.grid(True, color=RP_OVERLAY, alpha=0.5, linewidth=0.5)
    ax_thru.set_title("Throughput (KB/s)", fontsize=10, fontweight="bold", color=RP_TEXT)
    ax_thru.set_xlabel("Packet #", fontsize=8, color=RP_TEXT)
    ax_thru.tick_params(colors=RP_MUTED, labelsize=8)
    if len(pkt_times) > 1:
        window = 20
        thru = []
        for i in range(len(pkt_times)):
            start_idx = max(0, i - window)
            dt = pkt_times[i] - pkt_times[start_idx]
            if dt > 0:
                pkts_in_window = i - start_idx
                thru.append((pkts_in_window * IMAGE_DATA_PER_PKT / 1024) / dt)
            else:
                thru.append(0)
        ax_thru.plot(x, thru, color=RP_GOLD, linewidth=1, alpha=0.9)
        ax_thru.fill_between(x, thru, color=RP_GOLD, alpha=0.15)

    # Packet stats bar
    ax_stats.clear()
    ax_stats.set_facecolor(RP_SURFACE)
    ax_stats.set_title("Packet Stats", fontsize=10, fontweight="bold", color=RP_TEXT)
    ax_stats.tick_params(colors=RP_MUTED, labelsize=8)
    missing = total_pkts - rx_count
    categories = ["Received", "Missing", "Retransmitted"]
    values = [rx_count, max(0, missing), retx_count]
    colors = [RP_FOAM, RP_LOVE, RP_GOLD]
    bars = ax_stats.bar(categories, values, color=colors, alpha=0.85, edgecolor=RP_OVERLAY)
    for b, v in zip(bars, values):
        if v > 0:
            ax_stats.text(b.get_x() + b.get_width()/2, b.get_height() + 0.5,
                         str(v), ha="center", va="bottom", color=RP_TEXT, fontsize=9)
    ax_stats.set_ylim(0, max(total_pkts, 1) * 1.15)

    for ax in [ax_rssi, ax_snr, ax_thru, ax_stats]:
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.spines["bottom"].set_color(RP_OVERLAY)
        ax.spines["left"].set_color(RP_OVERLAY)


def _update_image_preview(img_ax, received, total_pkts):
    jpeg = _reassemble(received, total_pkts)
    try:
        img = Image.open(io.BytesIO(jpeg))
        img_ax.clear()
        img_ax.imshow(img)
        img_ax.axis("off")
        img_ax.set_title(f"Preview ({len(received)}/{total_pkts} pkts)",
                        fontsize=10, color=RP_TEXT)
    except Exception:
        pass


def _reassemble(received, total_pkts):
    parts = []
    for i in range(total_pkts):
        if i in received:
            parts.append(received[i])
        else:
            parts.append(b'\x00' * IMAGE_DATA_PER_PKT)
    return b''.join(parts)


# ---------------------------------------------------------------------------
# Main menu
# ---------------------------------------------------------------------------

def main():
    print(BANNER)

    port = pick_serial_port()
    ser = open_serial(port)
    if not ser:
        return

    print(f"  Connected: {port}")

    while True:
        print("\n  1) Send image")
        print("  2) Receive image")
        print("  3) Change serial port")
        print("  4) Quit")

        choice = input("\n  Select mode: ").strip()

        if choice == "1":
            try:
                cmd_send(ser)
            except Exception as e:
                print(f"  Error: {e}")
        elif choice == "2":
            try:
                cmd_receive(ser)
            except KeyboardInterrupt:
                print("\n  Stopped receiving.")
            except Exception as e:
                print(f"  Error: {e}")
        elif choice == "3":
            ser.close()
            port = pick_serial_port()
            ser = open_serial(port)
            if not ser:
                return
            print(f"  Connected: {port}")
        elif choice == "4":
            break
        else:
            print("  Invalid choice.")

    ser.close()
    print("  Goodbye.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Stopped.")
