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

# -- Kawaii Pastel palette --
KP_BG       = "#FFF5F9"   # Pale Pink background
KP_FACE     = "#FFFAFC"   # Rose White plot face
KP_TEXT     = "#5B4970"   # Muted Purple text
KP_GRID     = "#F0D6E8"   # Light Mauve grid
KP_SPINE    = "#E8C6DC"   # Soft Rose spines
KP_PINK     = "#F075AB"   # Rose Pink (RSSI)
KP_LAVENDER = "#9B8EC4"   # Soft Lavender (SNR)
KP_PEACH    = "#F2A685"   # Peach Glow (throughput)
KP_MINT     = "#7EC8B8"   # Mint Dream (received)
KP_LILAC    = "#D484BF"   # Soft Lilac (missing)
KP_BUTTER   = "#F0D58C"   # Butter Yellow (retransmitted)
KP_FILL     = "#ffdef2"   # Pastel Pink (area fills)


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
               colour="#F075AB", ncols=80)

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
            idx = line.find("pkts=")
            if idx >= 0:
                pkts_str = line[idx + 5:]
            else:
                pkts_str = ""

            missing = [int(x) for x in pkts_str.split(",") if x.strip().isdigit()]
            print(f"  ARQ round {arq_round + 1}: retransmitting {len(missing)} packets")

            retx_bar = tqdm(total=len(missing), unit="pkt", desc="  RETX",
                            bar_format="  {l_bar}{bar:30}{r_bar}",
                            colour="#F2A685", ncols=80)

            for pkt_num in missing:
                offset = pkt_num * IMAGE_DATA_PER_PKT
                chunk = data[offset:offset + IMAGE_DATA_PER_PKT]
                hex_str = chunk.hex().upper()
                ser.write(f"IMG_CHUNK,pkt={pkt_num},len={len(chunk)},hex={hex_str}\n".encode())
                resp = read_line(ser, timeout=10)
                retx_bar.update(1)

            retx_bar.close()
            ser.write(b"RETX_DONE\n")

            resp = read_line(ser, timeout=5)
            if resp:
                print(f"  {resp.strip()}")

    total_elapsed = time.time() - start_time
    avg_kbps = (len(data) / 1024) / max(0.001, total_elapsed)
    print(f"\n  Transfer complete: {total_elapsed:.1f}s, {avg_kbps:.1f} KB/s avg")
    if failed_pkts:
        print(f"  Initial failures: {len(failed_pkts)} packets")


# ---------------------------------------------------------------------------
# Matplotlib theme setup
# ---------------------------------------------------------------------------

def _apply_kawaii_theme():
    import matplotlib.pyplot as plt
    import matplotlib as mpl

    plt.rcParams.update({
        "axes.prop_cycle": mpl.cycler(color=[
            KP_PINK, KP_LAVENDER, KP_MINT, KP_PEACH, KP_LILAC, KP_BUTTER
        ]),
        "figure.facecolor": KP_BG,
        "axes.facecolor": KP_FACE,
        "text.color": KP_TEXT,
        "axes.labelcolor": KP_TEXT,
        "xtick.color": KP_TEXT,
        "ytick.color": KP_TEXT,
        "axes.grid": True,
        "grid.color": KP_GRID,
        "grid.alpha": 0.6,
        "grid.linewidth": 0.8,
        "grid.linestyle": "--",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.edgecolor": KP_SPINE,
        "axes.linewidth": 1.2,
        "font.family": "sans-serif",
        "font.sans-serif": [
            "Arial Rounded MT Bold", "Avenir Next", "Helvetica Neue",
            "Avenir", "Arial", "sans-serif"
        ],
        "font.size": 10,
        "axes.titlesize": 13,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "lines.linewidth": 2.5,
        "lines.markersize": 7,
        "legend.frameon": True,
        "legend.framealpha": 0.8,
        "legend.facecolor": KP_FACE,
        "legend.edgecolor": KP_GRID,
        "legend.fontsize": 9,
        "axes.titlepad": 12,
        "axes.labelpad": 8,
        "figure.dpi": 120,
        "savefig.dpi": 180,
        "savefig.facecolor": KP_BG,
    })


# ---------------------------------------------------------------------------
# Receive (with live plots + partial image preview)
# ---------------------------------------------------------------------------

def cmd_receive(ser):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    _apply_kawaii_theme()

    profile = pick_rf_profile(ser)
    print(f"\n  Listening for incoming transfers... (Ctrl+C to return to menu)")

    transfer_num = 0

    try:
        while True:
            total_pkts = 0
            received = {}
            rssi_data = []
            snr_data = []
            pkt_times = []
            start_time = None
            retx_count = 0

            fig = None
            ax_rssi = None
            ax_snr = None
            ax_thru = None
            ax_stats = None
            img_fig = None
            img_ax = None
            bar = None
            transfer_done = False

            while not transfer_done:
                line = read_line(ser, timeout=1)
                if line is None:
                    # Flush plot events while idle
                    if fig:
                        fig.canvas.flush_events()
                    if img_fig:
                        img_fig.canvas.flush_events()
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
                               colour="#7EC8B8", ncols=80)

                    # Init live plots
                    plt.ion()
                    fig = plt.figure(figsize=(11, 7), facecolor=KP_BG)
                    fig.canvas.manager.set_window_title(
                        f"Transfer #{transfer_num} — Live Stats")
                    gs = GridSpec(2, 2, figure=fig, hspace=0.4, wspace=0.35)

                    ax_rssi = fig.add_subplot(gs[0, 0])
                    ax_snr = fig.add_subplot(gs[0, 1])
                    ax_thru = fig.add_subplot(gs[1, 0])
                    ax_stats = fig.add_subplot(gs[1, 1])

                    fig.canvas.draw()
                    fig.canvas.flush_events()

                    # Init partial image preview
                    img_fig = plt.figure(figsize=(5, 5), facecolor=KP_BG)
                    img_fig.canvas.manager.set_window_title("Image Preview")
                    img_ax = img_fig.add_subplot(111)
                    img_ax.set_facecolor(KP_BG)
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
                    elif bar:
                        bar.write(f"  Warning: pkt {pkt_num} hex mismatch")

                    rssi_data.append(rssi_dbm)
                    snr_data.append(snr_db)
                    pkt_times.append(
                        time.time() - start_time if start_time else 0)

                    if bar:
                        bar.set_postfix_str(
                            f"RSSI={rssi_dbm:.0f}dBm SNR={snr_db:.1f}dB")
                        bar.update(1)

                    # Update live plots every 5 packets
                    n = len(rssi_data)
                    if fig and n % 5 == 0:
                        _update_live_plots(
                            ax_rssi, ax_snr, ax_thru, ax_stats,
                            rssi_data, snr_data, pkt_times,
                            len(received), total_pkts, retx_count)
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()

                    # Update image preview every 20 packets
                    if img_fig and n % 20 == 0 and len(received) > 10:
                        _update_image_preview(
                            img_ax, received, total_pkts)
                        img_fig.canvas.draw_idle()
                        img_fig.canvas.flush_events()

                elif rec == "IMG_END":
                    if bar:
                        bar.close()
                        bar = None

                    rx_count = len(received)
                    missing = [i for i in range(total_pkts)
                               if i not in received]
                    elapsed = (time.time() - start_time
                               if start_time else 0)

                    print(f"\n  Received: {rx_count}/{total_pkts} "
                          f"({100 * rx_count / max(1, total_pkts):.1f}%) "
                          f"in {elapsed:.1f}s")

                    # ARQ retransmission
                    if missing and len(missing) < total_pkts:
                        arq_round = 0
                        while missing and arq_round < ARQ_MAX_ROUNDS:
                            arq_round += 1
                            print(f"  ARQ round {arq_round}: "
                                  f"requesting {len(missing)} packets")

                            batch_size = 120
                            for bs in range(0, len(missing), batch_size):
                                batch = missing[bs:bs + batch_size]
                                nack_str = ",".join(str(x) for x in batch)
                                ser.write(
                                    f"IMG_NACK_SEND,missing={nack_str}\n"
                                    .encode())
                                resp = read_line(ser, timeout=5)
                                if resp and "NACK_ERR" in resp.strip():
                                    print(f"  NACK failed: {resp.strip()}")

                            retx_bar = tqdm(
                                total=len(missing), unit="pkt",
                                desc="  ARQ",
                                bar_format="  {l_bar}{bar:30}{r_bar}",
                                colour="#F2A685", ncols=80)
                            retx_deadline = time.time() + 30
                            while time.time() < retx_deadline:
                                rline = read_line(ser, timeout=2)
                                if rline is None:
                                    continue
                                rline = rline.strip()
                                if not rline:
                                    continue

                                rparts = rline.split(",")
                                rrec = rparts[0]
                                rf = {}
                                for rp in rparts[1:]:
                                    if "=" in rp:
                                        rk, rv = rp.split("=", 1)
                                        rf[rk] = rv

                                if rrec == "IMG_DATA":
                                    rpn = int(rf.get("pkt", 0))
                                    rdl = int(rf.get("len", 0))
                                    rhex = rf.get("hex", "")
                                    rrssi = int(rf.get("rssi", 0))
                                    rsnr = int(rf.get("snr", 0))

                                    if len(rhex) == rdl * 2:
                                        received[rpn] = bytes.fromhex(rhex)
                                        retx_count += 1

                                    rssi_data.append(rrssi / 100.0)
                                    snr_data.append(rsnr / 100.0)
                                    pkt_times.append(
                                        time.time() - start_time)
                                    retx_bar.update(1)

                                elif rrec == "IMG_END":
                                    break

                            retx_bar.close()
                            missing = [i for i in range(total_pkts)
                                       if i not in received]
                            if not missing:
                                print("  ARQ complete: all packets received!")
                                break
                            print(f"  Still missing: {len(missing)} packets")

                    # Final stats
                    rx_count = len(received)
                    total_elapsed = (time.time() - start_time
                                     if start_time else 0)
                    avg_kbps = ((rx_count * IMAGE_DATA_PER_PKT / 1024)
                                / max(0.001, total_elapsed))
                    loss_pct = (100 * (total_pkts - rx_count)
                                / max(1, total_pkts))

                    print(f"\n  === Transfer Complete ===")
                    print(f"  Packets: {rx_count}/{total_pkts} "
                          f"({100 * rx_count / max(1, total_pkts):.1f}%)")
                    print(f"  Time: {total_elapsed:.1f}s | "
                          f"Avg: {avg_kbps:.1f} KB/s | "
                          f"Loss: {loss_pct:.1f}% | "
                          f"Retransmitted: {retx_count}")

                    if rssi_data:
                        avg_rssi = sum(rssi_data) / len(rssi_data)
                        print(f"  RSSI: min={min(rssi_data):.0f} "
                              f"max={max(rssi_data):.0f} "
                              f"avg={avg_rssi:.0f} dBm")
                    if snr_data:
                        avg_snr = sum(snr_data) / len(snr_data)
                        print(f"  SNR:  min={min(snr_data):.1f} "
                              f"max={max(snr_data):.1f} "
                              f"avg={avg_snr:.1f} dB")

                    # Save image
                    jpeg = _reassemble(received, total_pkts)
                    out_name = f"received_{transfer_num}.jpg"
                    Path(out_name).write_bytes(jpeg)
                    print(f"  Saved: {out_name} ({len(jpeg):,} bytes)")

                    # Final plot update + save
                    if fig:
                        _update_live_plots(
                            ax_rssi, ax_snr, ax_thru, ax_stats,
                            rssi_data, snr_data, pkt_times,
                            rx_count, total_pkts, retx_count)
                        fig.canvas.draw()
                        fig.canvas.flush_events()

                        plot_name = f"received_{transfer_num}_stats.png"
                        fig.savefig(plot_name, facecolor=KP_BG,
                                    dpi=180, bbox_inches="tight")
                        print(f"  Stats: {plot_name}")

                    # Final image preview
                    if img_fig:
                        _update_image_preview(
                            img_ax, received, total_pkts)
                        img_fig.canvas.draw()
                        img_fig.canvas.flush_events()

                    try:
                        img = Image.open(io.BytesIO(jpeg))
                        img.show()
                    except Exception:
                        pass

                    print(f"\n  Ready for next transfer! "
                          f"(Ctrl+C to return to menu)")
                    transfer_done = True

                elif rec == "ImageTransfer":
                    print("  Pico ready.")
                elif rec == "PROFILE_SET":
                    pass
                elif rec in ("ERROR", "FATAL"):
                    print(f"  Pico: {line}")

    except KeyboardInterrupt:
        if bar:
            bar.close()
        print("\n  Stopped receiving.")


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _update_live_plots(ax_rssi, ax_snr, ax_thru, ax_stats,
                       rssi_data, snr_data, pkt_times,
                       rx_count, total_pkts, retx_count):
    x = list(range(len(rssi_data)))

    # -- RSSI --
    ax_rssi.clear()
    ax_rssi.set_title("RSSI (dBm)", fontsize=13, fontweight="bold")
    ax_rssi.set_xlabel("Packet #", fontsize=10)
    ax_rssi.plot(x, rssi_data, color=KP_PINK, linewidth=2.5, alpha=0.9)
    ax_rssi.fill_between(x, rssi_data, color=KP_PINK, alpha=0.12)
    if rssi_data:
        avg = sum(rssi_data) / len(rssi_data)
        ax_rssi.axhline(y=avg, color=KP_LILAC, linewidth=1.2,
                        linestyle="--", alpha=0.7)
        ax_rssi.annotate(f"avg {avg:.0f}", xy=(len(x) * 0.02, avg),
                        fontsize=8, color=KP_LILAC, va="bottom")

    # -- SNR --
    ax_snr.clear()
    ax_snr.set_title("SNR (dB)", fontsize=13, fontweight="bold")
    ax_snr.set_xlabel("Packet #", fontsize=10)
    ax_snr.plot(x, snr_data, color=KP_LAVENDER, linewidth=2.5, alpha=0.9)
    ax_snr.fill_between(x, snr_data, color=KP_LAVENDER, alpha=0.12)
    if snr_data:
        avg = sum(snr_data) / len(snr_data)
        ax_snr.axhline(y=avg, color=KP_LILAC, linewidth=1.2,
                       linestyle="--", alpha=0.7)
        ax_snr.annotate(f"avg {avg:.1f}", xy=(len(x) * 0.02, avg),
                       fontsize=8, color=KP_LILAC, va="bottom")

    # -- Throughput --
    ax_thru.clear()
    ax_thru.set_title("Throughput (KB/s)", fontsize=13, fontweight="bold")
    ax_thru.set_xlabel("Packet #", fontsize=10)
    if len(pkt_times) > 1:
        window = 20
        thru = []
        for i in range(len(pkt_times)):
            si = max(0, i - window)
            dt = pkt_times[i] - pkt_times[si]
            if dt > 0:
                pkts_w = i - si
                thru.append((pkts_w * IMAGE_DATA_PER_PKT / 1024) / dt)
            else:
                thru.append(0)
        ax_thru.plot(x, thru, color=KP_PEACH, linewidth=2.5, alpha=0.9)
        ax_thru.fill_between(x, thru, color=KP_PEACH, alpha=0.15)

    # -- Packet stats bar --
    ax_stats.clear()
    ax_stats.set_title("Packet Stats", fontsize=13, fontweight="bold")
    miss = max(0, total_pkts - rx_count)
    cats = ["Received", "Missing", "Retx"]
    vals = [rx_count, miss, retx_count]
    colors = [KP_MINT, KP_LILAC, KP_BUTTER]
    bars = ax_stats.bar(cats, vals, color=colors, alpha=0.85,
                        edgecolor="white", linewidth=1.5)
    for b, v in zip(bars, vals):
        if v > 0:
            ax_stats.text(b.get_x() + b.get_width() / 2,
                         b.get_height() + max(total_pkts * 0.01, 0.5),
                         str(v), ha="center", va="bottom",
                         color=KP_TEXT, fontsize=10, fontweight="bold")
    ax_stats.set_ylim(0, max(total_pkts, 1) * 1.2)


def _update_image_preview(img_ax, received, total_pkts):
    jpeg = _reassemble(received, total_pkts)
    try:
        img = Image.open(io.BytesIO(jpeg))
        img_ax.clear()
        img_ax.imshow(img)
        img_ax.axis("off")
        img_ax.set_title(f"Preview ({len(received)}/{total_pkts} pkts)",
                        fontsize=11, fontweight="bold", color=KP_TEXT)
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
            cmd_receive(ser)
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
