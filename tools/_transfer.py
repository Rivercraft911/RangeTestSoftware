"""
TX/RX transfer logic with ARQ for LoRa image transfer.
"""
import time
from pathlib import Path

from rich.console import Console
from rich.progress import Progress, BarColumn, TextColumn, TaskProgressColumn, TimeRemainingColumn
from rich.table import Table

from _serial_bridge import (
    IMAGE_DATA_PER_PKT, read_line, wait_for, pick_rf_profile, console,
)
from _dashboard import (
    KP_PINK, KP_PEACH, KP_MINT, KP_BG,
    apply_kawaii_theme, init_stats_figure, init_preview_figure,
    update_live_plots, update_image_preview, reassemble, show_image,
)

ARQ_MAX_ROUNDS = 3


def _make_progress(description, color):
    return Progress(
        TextColumn(f"  [bold {color}]{description}[/]"),
        BarColumn(bar_width=30, complete_style=color, finished_style=color),
        TaskProgressColumn(),
        TextColumn("{task.fields[status]}"),
        TimeRemainingColumn(),
        console=console,
    )


def _print_stats_table(rx_count, total_pkts, elapsed, kbps, loss, retx,
                       rssi_data, snr_data):
    table = Table(title="Transfer Complete", title_style=f"bold {KP_PINK}",
                  border_style=KP_MINT, show_header=False, padding=(0, 1))
    table.add_column("Metric", style="bold")
    table.add_column("Value")

    pct = 100 * rx_count / max(1, total_pkts)
    table.add_row("Packets", f"{rx_count}/{total_pkts} ({pct:.1f}%)")
    table.add_row("Time", f"{elapsed:.1f}s")
    table.add_row("Throughput", f"{kbps:.1f} KB/s")
    table.add_row("Loss", f"{loss:.1f}%")
    table.add_row("Retransmitted", str(retx))

    if rssi_data:
        avg_rssi = sum(rssi_data) / len(rssi_data)
        table.add_row("RSSI",
                      f"min={min(rssi_data):.0f}  max={max(rssi_data):.0f}  "
                      f"avg={avg_rssi:.0f} dBm")
    if snr_data:
        avg_snr = sum(snr_data) / len(snr_data)
        table.add_row("SNR",
                      f"min={min(snr_data):.1f}  max={max(snr_data):.1f}  "
                      f"avg={avg_snr:.1f} dB")

    console.print()
    console.print(table)


# ---------------------------------------------------------------------------
# Send
# ---------------------------------------------------------------------------

def cmd_send(ser):
    profile = pick_rf_profile(ser)

    while True:
        image_path = input("\n  Image path: ").strip().strip('"').strip("'")
        if Path(image_path).exists():
            break
        console.print(f"  File not found: {image_path}")

    data = Path(image_path).read_bytes()
    total_pkts = (len(data) + IMAGE_DATA_PER_PKT - 1) // IMAGE_DATA_PER_PKT
    console.print(f"  Image: {Path(image_path).name} ({len(data):,} bytes, {total_pkts} packets)")

    ser.reset_input_buffer()
    ser.write(f"IMG_SEND,total_bytes={len(data)},total_pkts={total_pkts}\n".encode())

    resp = wait_for(ser, "IMG_ACK,ready", timeout=5)
    if not resp:
        console.print("  Error: no ready ACK from Pico")
        return

    start_time = time.time()
    failed_pkts = []

    progress = _make_progress("TX", KP_PINK)
    progress.start()
    task_id = progress.add_task("tx", total=total_pkts, status="")

    for i in range(total_pkts):
        offset = i * IMAGE_DATA_PER_PKT
        chunk = data[offset:offset + IMAGE_DATA_PER_PKT]
        hex_str = chunk.hex().upper()

        ser.write(f"IMG_CHUNK,pkt={i},len={len(chunk)},hex={hex_str}\n".encode())

        resp = read_line(ser, timeout=10)
        if resp is None:
            progress.console.print(f"  Timeout on pkt {i}")
            failed_pkts.append(i)
        elif "IMG_NACK" in resp:
            failed_pkts.append(i)

        elapsed = time.time() - start_time
        bytes_sent = min((i + 1) * IMAGE_DATA_PER_PKT, len(data))
        kbps = (bytes_sent / 1024) / max(0.001, elapsed)
        progress.update(task_id, advance=1, status=f"{kbps:.1f} KB/s")

    progress.stop()

    ser.write(b"IMG_DONE\n")
    resp = read_line(ser, timeout=10)
    if resp:
        console.print(f"  {resp.strip()}")

    elapsed = time.time() - start_time

    # ARQ: wait for retransmission requests
    for arq_round in range(ARQ_MAX_ROUNDS):
        # Must wait long enough for: Pico TX LoRa RX timeout (8s) + processing
        resp = read_line(ser, timeout=12)
        if not resp:
            break
        line = resp.strip()

        if "ARQ_DONE" in line:
            break

        # Skip informational messages like ARQ_WAIT — keep reading
        if "ARQ_WAIT" in line:
            # The Pico is now listening for LoRa NACK; wait for the result
            resp = read_line(ser, timeout=12)
            if not resp:
                break
            line = resp.strip()
            if "ARQ_DONE" in line:
                break

        if "RETX_REQ" in line:
            idx = line.find("pkts=")
            pkts_str = line[idx + 5:] if idx >= 0 else ""

            missing = [int(x) for x in pkts_str.split(",") if x.strip().isdigit()]
            console.print(f"  ARQ round {arq_round + 1}: retransmitting {len(missing)} packets")

            retx_progress = _make_progress("RETX", KP_PEACH)
            retx_progress.start()
            retx_task = retx_progress.add_task("retx", total=len(missing), status="")

            for pkt_num in missing:
                offset = pkt_num * IMAGE_DATA_PER_PKT
                chunk = data[offset:offset + IMAGE_DATA_PER_PKT]
                hex_str = chunk.hex().upper()
                ser.write(f"IMG_CHUNK,pkt={pkt_num},len={len(chunk)},hex={hex_str}\n".encode())
                resp = read_line(ser, timeout=10)
                retx_progress.update(retx_task, advance=1)

            retx_progress.stop()
            ser.write(b"RETX_DONE\n")

            # Read ARQ_RETX_COMPLETE and any remaining messages
            resp = read_line(ser, timeout=5)
            if resp:
                console.print(f"  {resp.strip()}")

    total_elapsed = time.time() - start_time
    avg_kbps = (len(data) / 1024) / max(0.001, total_elapsed)
    console.print(f"\n  Transfer complete: {total_elapsed:.1f}s, {avg_kbps:.1f} KB/s avg")
    if failed_pkts:
        console.print(f"  Initial failures: {len(failed_pkts)} packets")


# ---------------------------------------------------------------------------
# Receive (with live plots + partial image preview)
# ---------------------------------------------------------------------------

def cmd_receive(ser):
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt

    apply_kawaii_theme()

    profile = pick_rf_profile(ser)
    console.print(f"\n  Listening for incoming transfers... (Ctrl+C to return to menu)")

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
            progress = None
            rx_task = None
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
                    console.print(f"\n  Transfer #{transfer_num}: {total_pkts} packets expected")

                    progress = _make_progress("RX", KP_MINT)
                    progress.start()
                    rx_task = progress.add_task("rx", total=total_pkts, status="")

                    # Init live plots
                    plt.ion()
                    fig, ax_rssi, ax_snr, ax_thru, ax_stats = \
                        init_stats_figure(transfer_num)
                    img_fig, img_ax = init_preview_figure()

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
                    elif progress:
                        progress.console.print(f"  Warning: pkt {pkt_num} hex mismatch")

                    rssi_data.append(rssi_dbm)
                    snr_data.append(snr_db)
                    pkt_times.append(
                        time.time() - start_time if start_time else 0)

                    if progress and rx_task is not None:
                        progress.update(
                            rx_task, advance=1,
                            status=f"RSSI={rssi_dbm:.0f}dBm SNR={snr_db:.1f}dB")

                    # Update live plots every 5 packets
                    n = len(rssi_data)
                    if fig and n % 5 == 0:
                        update_live_plots(
                            ax_rssi, ax_snr, ax_thru, ax_stats,
                            rssi_data, snr_data, pkt_times,
                            len(received), total_pkts, retx_count)
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()

                    # Update image preview every 20 packets
                    if img_fig and n % 20 == 0 and len(received) > 10:
                        update_image_preview(
                            img_ax, received, total_pkts)
                        img_fig.canvas.draw_idle()
                        img_fig.canvas.flush_events()

                elif rec == "IMG_END":
                    if progress:
                        progress.stop()
                        progress = None
                        rx_task = None

                    rx_count = len(received)
                    missing = [i for i in range(total_pkts)
                               if i not in received]
                    elapsed = (time.time() - start_time
                               if start_time else 0)

                    console.print(f"\n  Received: {rx_count}/{total_pkts} "
                          f"({100 * rx_count / max(1, total_pkts):.1f}%) "
                          f"in {elapsed:.1f}s")

                    # ARQ retransmission
                    if missing and len(missing) < total_pkts:
                        arq_round = 0
                        while missing and arq_round < ARQ_MAX_ROUNDS:
                            arq_round += 1
                            console.print(f"  ARQ round {arq_round}: "
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
                                    console.print(f"  NACK failed: {resp.strip()}")

                            arq_progress = _make_progress("ARQ", KP_PEACH)
                            arq_progress.start()
                            arq_task = arq_progress.add_task(
                                "arq", total=len(missing), status="")
                            retx_received = 0
                            retx_deadline = time.time() + 30
                            while time.time() < retx_deadline:
                                rline = read_line(ser, timeout=2)
                                if rline is None:
                                    if retx_received > 0:
                                        break
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
                                        retx_received += 1

                                    rssi_data.append(rrssi / 100.0)
                                    snr_data.append(rsnr / 100.0)
                                    pkt_times.append(
                                        time.time() - start_time)
                                    arq_progress.update(arq_task, advance=1)

                                elif rrec == "IMG_END":
                                    break

                            arq_progress.stop()
                            missing = [i for i in range(total_pkts)
                                       if i not in received]
                            if not missing:
                                console.print("  ARQ complete: all packets received!")
                                break
                            console.print(f"  Still missing: {len(missing)} packets")

                    # Final stats
                    rx_count = len(received)
                    total_elapsed = (time.time() - start_time
                                     if start_time else 0)
                    avg_kbps = ((rx_count * IMAGE_DATA_PER_PKT / 1024)
                                / max(0.001, total_elapsed))
                    loss_pct = (100 * (total_pkts - rx_count)
                                / max(1, total_pkts))

                    _print_stats_table(
                        rx_count, total_pkts, total_elapsed, avg_kbps,
                        loss_pct, retx_count, rssi_data, snr_data)

                    # Save image
                    jpeg = reassemble(received, total_pkts)
                    out_name = f"received_{transfer_num}.jpg"
                    Path(out_name).write_bytes(jpeg)
                    console.print(f"  Saved: {out_name} ({len(jpeg):,} bytes)")

                    # Final plot update + save
                    if fig:
                        update_live_plots(
                            ax_rssi, ax_snr, ax_thru, ax_stats,
                            rssi_data, snr_data, pkt_times,
                            rx_count, total_pkts, retx_count)
                        fig.canvas.draw()
                        fig.canvas.flush_events()

                        plot_name = f"received_{transfer_num}_stats.png"
                        fig.savefig(plot_name, facecolor=KP_BG,
                                    dpi=180, bbox_inches="tight")
                        console.print(f"  Stats: {plot_name}")

                    # Final image preview
                    if img_fig:
                        update_image_preview(
                            img_ax, received, total_pkts)
                        img_fig.canvas.draw()
                        img_fig.canvas.flush_events()

                    show_image(jpeg)

                    console.print(f"\n  Ready for next transfer! "
                          f"(Ctrl+C to return to menu)")
                    transfer_done = True

                elif rec == "ImageTransfer":
                    console.print("  Pico ready.")
                elif rec == "PROFILE_SET":
                    pass
                elif rec in ("ERROR", "FATAL"):
                    console.print(f"  Pico: {line}")

    except KeyboardInterrupt:
        if progress:
            progress.stop()
        console.print("\n  Stopped receiving.")
