#!/usr/bin/env python3
"""
Balloon Link Budget Ground Station

Terminal CLI for commands, matplotlib dashboard for live visualization.
Connects to a BalloonGround Pico over USB serial, parses telemetry,
sends commands, displays live RSSI/SNR/throughput graphs, and renders
images progressively during transfer.

Usage:
    python3 groundstation/balloon_ground.py
"""

import os
import sys
import subprocess
import threading
import time
import queue

# Add tools/ to path for _serial_bridge import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
sys.path.insert(0, os.path.dirname(__file__))

import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from rich.console import Console

from _protocol import (
    parse_line, build_cmd_line, build_nack_line,
    Beacon, CmdAck, BulkData, CmdSent, CmdFail,
    STATE_NAMES, CMD_NAMES, BAND_NAMES, describe_cmd_result,
)
from _state import GroundState
from _logger import BalloonLogger
from _dashboard import (
    apply_dark_theme, init_dashboard, update_dashboard,
    init_image_preview, update_image_preview,
    init_bulk_preview, update_bulk_preview, save_image,
)
from _serial_bridge import APPLE, pick_serial_port, RF_PROFILES

console = Console(highlight=False)

# ── Banner ──

BANNER = r"""
  ____        _ _                    _     _       _
 | __ )  __ _| | | ___   ___  _ __ | |   (_)_ __ | | __
 |  _ \ / _` | | |/ _ \ / _ \| '_ \| |   | | '_ \| |/ /
 | |_) | (_| | | | (_) | (_) | | | | |___| | | | |   <
 |____/ \__,_|_|_|\___/ \___/|_| |_|_____|_|_| |_|_|\_\
"""


def print_banner():
    for i, line in enumerate(BANNER.strip("\n").split("\n")):
        c = APPLE[i % len(APPLE)]
        console.print(f"[bold {c}]{line}[/]")
    console.print()
    console.print(f"  [bold {APPLE[5]}]Ground Station — Link Budget Test Platform[/]")
    console.print()
    console.print(f"  [{APPLE[0]}]green[/]  OK / beacon  "
                  f"[{APPLE[1]}]yellow[/]  warning  "
                  f"[{APPLE[2]}]orange[/]  throughput")
    console.print(f"  [{APPLE[3]}]red[/]    error       "
                  f"[{APPLE[4]}]purple[/]  S-Band   "
                  f"[{APPLE[5]}]blue[/]    command")
    console.print()


# ── Serial Reader Thread ──

class SerialReader(threading.Thread):
    def __init__(self, ser, state: GroundState, logger: BalloonLogger):
        super().__init__(daemon=True)
        self.ser = ser
        self.state = state
        self.logger = logger
        self.running = True

    def run(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                tag, record = parse_line(line)
                if tag is None:
                    continue

                if tag == "BEACON" and isinstance(record, Beacon):
                    self.state.record_beacon(record)
                    self.logger.log_beacon(record)

                elif tag == "CMD_ACK" and isinstance(record, CmdAck):
                    matched = self.state.record_ack(record)
                    self.logger.log_ack(record)
                    cmd_name = CMD_NAMES.get(record.cmd, f"?{record.cmd}")
                    result_str = describe_cmd_result(record.result)
                    color = APPLE[0] if record.result == 0 else (
                        APPLE[1] if record.result == 3 else APPLE[3]
                    )
                    seq_text = f"seq={record.seq}"
                    if matched and matched.seq != record.seq:
                        seq_text = f"host_seq={matched.seq} wire_seq={record.seq}"
                    console.print(
                        f"  [{color}]ACK[/] cmd={cmd_name} {seq_text} "
                        f"result={result_str} "
                        f"cmd_rssi={record.cmd_rssi_x100 / 100:.1f} "
                        f"cmd_snr={record.cmd_snr_x100 / 100:.1f}"
                    )

                elif tag == "BULK" and isinstance(record, BulkData):
                    self.state.record_bulk(record)
                    self.logger.log_bulk(record)
                    # Print milestone for image transfers (total > 0)
                    if record.total_pkts > 0:
                        snap = self.state.get_snapshot()
                        unique_count = snap["bulk_received_count"]
                        if record.pkt_num % 10 == 0 or \
                                record.pkt_num == record.total_pkts - 1:
                            pct = unique_count * 100 // record.total_pkts
                            console.print(
                                f"  [{APPLE[4]}]IMG[/] pkt "
                                f"{unique_count}/{record.total_pkts} "
                                f"({pct}%)"
                            )
                        if record.pkt_num == record.total_pkts - 1:
                            if unique_count == record.total_pkts:
                                console.print(
                                    f"  [{APPLE[1]}]Image payload complete[/] "
                                    f"balloon will accept NACKs briefly"
                                )
                            else:
                                missing_count = record.total_pkts - unique_count
                                console.print(
                                    f"  [{APPLE[3]}]Final image packet arrived[/] "
                                    f"but {missing_count} packet"
                                    f"{'' if missing_count == 1 else 's'} "
                                    f"still missing"
                                )

                elif tag == "CMD_SENT" and isinstance(record, CmdSent):
                    matched = self.state.record_cmd_sent(record)
                    cmd_name = CMD_NAMES.get(record.cmd, f"?{record.cmd}")
                    seq_text = f"seq={record.seq}"
                    if matched and matched.seq != record.seq:
                        seq_text = f"host_seq={matched.seq} wire_seq={record.seq}"
                    console.print(
                        f"  [{APPLE[2]}]RADIO_TX[/] cmd={cmd_name} {seq_text}"
                    )

                elif tag == "CMD_FAIL" and isinstance(record, CmdFail):
                    matched = self.state.record_cmd_fail(record)
                    cmd_name = CMD_NAMES.get(record.cmd, f"?{record.cmd}")
                    seq_text = f"seq={record.seq}"
                    if matched and matched.seq != record.seq:
                        seq_text = f"host_seq={matched.seq} wire_seq={record.seq}"
                    console.print(
                        f"  [{APPLE[3]}]CMD_FAIL[/] cmd={cmd_name} "
                        f"{seq_text} reason={record.reason}"
                    )

                elif tag == "NACK_SENT":
                    console.print(f"  [{APPLE[4]}]NACK sent[/]")

                elif tag == "NACK_FAIL":
                    reason = record.get("reason", "?") if isinstance(record, dict) else "?"
                    console.print(f"  [{APPLE[3]}]NACK failed: {reason}[/]")

                else:
                    console.print(f"  [dim]{line}[/]")

            except Exception as e:
                if self.running:
                    console.print(f"  [{APPLE[3]}]Serial error: {e}[/]")
                    time.sleep(0.5)

    def stop(self):
        self.running = False


# ── Input Thread ──

def _input_thread(q: queue.Queue):
    """Daemon thread: reads lines from stdin into a queue."""
    while True:
        try:
            line = sys.stdin.readline()
            if not line:
                q.put(None)
                break
            q.put(line.rstrip("\n"))
        except Exception:
            q.put(None)
            break


def _prompt(text="  > "):
    sys.stdout.write(text)
    sys.stdout.flush()


def _wait_input(input_q: queue.Queue) -> str:
    """Block until a line arrives from the input thread."""
    val = input_q.get()
    return val if val is not None else ""


# ── Command helpers ──

def _command_policy(cmd_name: str) -> tuple[int, float]:
    if cmd_name == "STOP":
        return 6, 0.35
    if cmd_name == "PING":
        return 2, 2.0
    if cmd_name == "SET_SBAND_PROFILE":
        return 2, 2.8
    if cmd_name in {"START_IMAGE", "START_BULK", "POWER_SWEEP"}:
        return 1, 2.8
    return 1, 2.0


def _wait_for_command_result(state: GroundState, seq: int, timeout_s: float):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        rec = state.get_command(seq)
        if rec and (rec.acked or rec.failed):
            return rec
        try:
            plt.pause(0.05)
        except Exception:
            time.sleep(0.05)
    return state.get_command(seq)


def send_command(ser, state: GroundState, logger: BalloonLogger,
                 cmd_name: str, param: int = 0):
    seq = state.submit_command(cmd_name, param)
    line = build_cmd_line(cmd_name, param, seq)
    logger.log_cmd(cmd_name, param, seq)
    retries, ack_timeout = _command_policy(cmd_name)

    for attempt in range(1, retries + 1):
        try:
            ser.write(line.encode())
            ser.flush()
        except serial.SerialException as e:
            state.mark_command_timeout(seq, reason="serial_write")
            console.print(f"  [{APPLE[3]}]Serial write failed[/] {e}")
            return state.get_command(seq)

        suffix = "" if attempt == 1 else f" attempt={attempt}/{retries}"
        console.print(
            f"  [{APPLE[5]}]>> {cmd_name}[/] param={param} seq={seq}{suffix}"
        )

        rec = _wait_for_command_result(state, seq, ack_timeout)
        if rec and (rec.acked or rec.failed):
            return rec

        if attempt < retries:
            console.print(
                f"  [{APPLE[1]}]No ACK yet[/] cmd={cmd_name} seq={seq}; retrying"
            )

    state.mark_command_timeout(seq)
    console.print(f"  [{APPLE[3]}]No ACK[/] cmd={cmd_name} seq={seq}")
    return state.get_command(seq)


def send_nack(ser, missing: list):
    line = build_nack_line(missing)
    ser.write(line.encode())
    ser.flush()
    console.print(f"  [{APPLE[4]}]>> NACK[/] missing={len(missing)} pkts")


# ── Menu ──

def print_menu():
    items = [
        ("1", "Dashboard",       "Open/refresh live telemetry dashboard"),
        ("2", "Ping",            "Send PING command"),
        ("3", "Set Profile",     "Change S-Band RF profile"),
        ("4", "Start Image",     "Begin preloaded image downlink"),
        ("5", "Start Bulk",      "Begin bulk stress test"),
        ("6", "Stop",            "Stop current operation"),
        ("7", "Power Sweep",     "Step through S-Band TX power levels"),
        ("8", "Send NACK",       "Request retransmission of missing image packets"),
        ("9", "Save Image",      "Save received image to disk"),
        ("B", "Brute Force",     "Timed max-throughput stress test"),
        ("L", "Link Report",     "Print link budget summary"),
        ("P", "Change Port",     "Switch serial port"),
        ("Q", "Quit",            "Exit ground station"),
    ]
    console.print()
    for i, (key, name, desc) in enumerate(items):
        c = APPLE[i % len(APPLE)]
        console.print(f"  [bold {c}]{key})[/]  {name:16s} — {desc}")
    console.print()


# ── Main ──

def main():
    print_banner()

    # Serial connection — open without flushing buffer so we catch "ready"
    port = pick_serial_port()
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.5)
    except serial.SerialException as e:
        console.print(f"  [{APPLE[3]}]Could not open {port}: {e}[/]")
        return

    console.print(f"  [{APPLE[0]}]Connected to {port}[/]")

    console.print(f"  [{APPLE[1]}]Probing ground firmware...[/]")

    # Init state, logger, reader
    state = GroundState()
    logger = BalloonLogger()
    console.print(f"  [{APPLE[5]}]Logging to: {logger.path}[/]")

    reader = SerialReader(ser, state, logger)
    reader.start()

    rec = send_command(ser, state, logger, "PING")
    if rec and rec.acked and rec.result == 0:
        console.print(f"  [{APPLE[0]}]Link probe succeeded[/] ground firmware is responding")
    else:
        console.print(f"  [{APPLE[3]}]Link probe failed[/] no PING ACK yet")

    # Start non-blocking input thread
    input_q = queue.Queue()
    inp_t = threading.Thread(target=_input_thread, args=(input_q,), daemon=True)
    inp_t.start()

    # Dashboard state
    dash_fig = None
    dash_axes = None
    img_fig = None
    img_ax = None
    bulk_fig = None
    bulk_ax = None
    sband_profile = 1
    auto_stop_time = None
    image_transfer_active = False
    last_dash_update = 0.0

    print_menu()
    _prompt()

    try:
        while True:
            now = time.time()

            if dash_fig is not None and not plt.fignum_exists(dash_fig.number):
                dash_fig = None
                dash_axes = None
            if img_fig is not None and not plt.fignum_exists(img_fig.number):
                img_fig = None
                img_ax = None
            if bulk_fig is not None and not plt.fignum_exists(bulk_fig.number):
                bulk_fig = None
                bulk_ax = None

            # Refresh dashboard / preview (~15 Hz when a figure is open)
            if (dash_fig is not None or img_fig is not None or bulk_fig is not None) and now - last_dash_update > 0.066:
                try:
                    snap = state.get_snapshot()
                    active_profile = (
                        snap["last_beacon"].sband_profile
                        if snap["last_beacon"] is not None else sband_profile
                    )
                    if dash_fig is not None:
                        update_dashboard(dash_fig, dash_axes, snap, active_profile)
                    if img_fig is not None and snap["bulk_total"] > 0:
                        update_image_preview(img_fig, img_ax,
                                             snap["bulk_data"],
                                             snap["bulk_total"])
                    if bulk_fig is not None and snap["bulk_text_preview"]:
                        band_name = "UHF" if snap["bulk_band"] == 1 else (
                            "S-Band" if snap["bulk_band"] == 2 else "Unknown"
                        )
                        update_bulk_preview(bulk_fig, bulk_ax,
                                            snap["bulk_text_preview"],
                                            snap["bulk_count"],
                                            band_name)
                    last_dash_update = now
                except Exception:
                    pass

            # Keep matplotlib event loop alive
            try:
                plt.pause(0.02)
            except Exception:
                time.sleep(0.02)

            # Check auto-stop timer (brute force test)
            if auto_stop_time and time.time() >= auto_stop_time:
                rec = send_command(ser, state, logger, "STOP")
                auto_stop_time = None
                image_transfer_active = False
                snap = state.get_snapshot()
                if rec and rec.acked and rec.result == 0:
                    console.print(
                        f"\n  [{APPLE[0]}]Brute force complete[/] "
                        f"{snap['bulk_count']} packets received"
                    )
                else:
                    console.print(
                        f"\n  [{APPLE[3]}]Auto-stop not confirmed[/] "
                        f"{snap['bulk_count']} packets received so far"
                    )
                print_menu()
                _prompt()

            if image_transfer_active:
                snap = state.get_snapshot()
                if (snap["bulk_total"] > 0 and
                        snap["bulk_received_count"] == snap["bulk_total"]):
                    image_transfer_active = False
                    console.print(
                        f"  [{APPLE[0]}]Image transfer complete[/] "
                        f"balloon should return to beacon shortly"
                    )

            # Non-blocking input check
            try:
                raw = input_q.get_nowait()
            except queue.Empty:
                continue

            if raw is None:
                break

            choice = raw.strip().upper()
            if not choice:
                print_menu()
                _prompt()
                continue

            if choice == "1":
                if dash_fig is None:
                    apply_dark_theme()
                    plt.ion()
                    dash_fig, dash_axes = init_dashboard()
                    plt.show(block=False)
                    console.print(f"  [{APPLE[0]}]Dashboard opened[/]")
                else:
                    snap = state.get_snapshot()
                    active_profile = (
                        snap["last_beacon"].sband_profile
                        if snap["last_beacon"] is not None else sband_profile
                    )
                    update_dashboard(dash_fig, dash_axes, snap, active_profile)
                    console.print(f"  [{APPLE[0]}]Dashboard refreshed[/]")

            elif choice == "2":
                send_command(ser, state, logger, "PING")

            elif choice == "3":
                console.print("\n  S-Band RF Profiles:")
                for j, (k, (name, desc, _)) in enumerate(RF_PROFILES.items()):
                    c = APPLE[j % len(APPLE)]
                    console.print(f"  [bold {c}]{k})[/]  {name:15s} — {desc}")
                _prompt("\n  Select profile [1-4]: ")
                p = _wait_input(input_q).strip()
                if p in RF_PROFILES:
                    requested_profile = int(p)
                    rec = send_command(ser, state, logger, "SET_SBAND_PROFILE",
                                       param=requested_profile)
                    if rec and rec.acked and rec.result == 0:
                        sband_profile = requested_profile
                    else:
                        console.print(
                            f"  [{APPLE[3]}]Profile change was not confirmed[/]"
                        )
                else:
                    console.print(f"  [{APPLE[3]}]Invalid profile[/]")

            elif choice == "4":
                state.clear_bulk()
                rec = send_command(ser, state, logger, "START_IMAGE")
                if rec and rec.acked and rec.result == 0:
                    image_transfer_active = True
                    if img_fig is None:
                        plt.ion()
                        img_fig, img_ax = init_image_preview()
                        plt.show(block=False)
                    console.print(f"  [{APPLE[4]}]Image transfer started[/] "
                                  f"watch preview window")
                else:
                    image_transfer_active = False
                    console.print(
                        f"  [{APPLE[3]}]Image transfer was not confirmed[/]"
                    )

            elif choice == "5":
                console.print("  Band for bulk test:")
                console.print(f"  [bold {APPLE[0]}]1)[/]  S-Band (default)")
                console.print(f"  [bold {APPLE[1]}]2)[/]  UHF")
                _prompt("  Select [1-2]: ")
                b = _wait_input(input_q).strip()
                # BALLOON_BAND_SBAND=2, BALLOON_BAND_UHF=1
                band = 1 if b == "2" else 2
                state.clear_bulk()
                image_transfer_active = False
                rec = send_command(ser, state, logger, "START_BULK", param=band)
                if rec and rec.acked and rec.result == 0:
                    if bulk_fig is None:
                        plt.ion()
                        bulk_fig, bulk_ax = init_bulk_preview()
                        plt.show(block=False)
                    console.print(f"  [{APPLE[5]}]Bulk test started on "
                                  f"{'UHF' if band == 1 else 'S-Band'}[/]")
                else:
                    console.print(
                        f"  [{APPLE[3]}]Bulk test was not confirmed[/]"
                    )

            elif choice == "6":
                send_command(ser, state, logger, "STOP")
                auto_stop_time = None
                image_transfer_active = False

            elif choice == "7":
                console.print("  Band for power sweep:")
                console.print(f"  [bold {APPLE[0]}]1)[/]  S-Band (default)")
                console.print(f"  [bold {APPLE[1]}]2)[/]  UHF")
                _prompt("  Select [1-2]: ")
                b = _wait_input(input_q).strip()
                band = 1 if b == "2" else 2
                _prompt("  Max power dBm [13]: ")
                p = _wait_input(input_q).strip()
                max_pwr = int(p) if p.isdigit() else 13
                if max_pwr > 20:
                    max_pwr = 20
                if max_pwr < 2:
                    max_pwr = 13
                # Encode: (band << 5) | max_power
                param = (band << 5) | max_pwr
                rec = send_command(ser, state, logger, "POWER_SWEEP", param=param)
                if rec and rec.acked and rec.result == 0:
                    band_name = "UHF" if band == 1 else "S-Band"
                    console.print(f"  [{APPLE[0]}]Power sweep on {band_name} "
                                  f"(2..{max_pwr} dBm)[/]")
                else:
                    console.print(
                        f"  [{APPLE[3]}]Power sweep was not confirmed[/]"
                    )

            elif choice == "8":
                snap = state.get_snapshot()
                total = snap["bulk_total"]
                received = snap["bulk_received_keys"]
                if total <= 0:
                    console.print(f"  [{APPLE[3]}]No active image transfer[/]")
                else:
                    missing = [i for i in range(total) if i not in received]
                    if not missing:
                        console.print(f"  [{APPLE[0]}]All {total} packets received![/]")
                    else:
                        console.print(f"  [{APPLE[1]}]Missing {len(missing)}/{total} "
                                      f"packets[/]")
                        send_nack(ser, missing[:120])

            elif choice == "9":
                snap = state.get_snapshot()
                data_dir = os.path.join(os.path.dirname(__file__), "data")
                os.makedirs(data_dir, exist_ok=True)
                path = save_image(snap["bulk_data"],
                                  snap["bulk_total"], data_dir)
                if path:
                    pct = len(snap["bulk_data"]) / max(snap["bulk_total"], 1) * 100
                    tag = "COMPLETE" if pct >= 100 else f"PARTIAL ({pct:.0f}%)"
                    console.print(f"  [{APPLE[0]}]Image saved: {path} [{tag}][/]")
                    if sys.platform == "darwin":
                        subprocess.Popen(["open", path])
                else:
                    console.print(f"  [{APPLE[3]}]No image data to save[/]")

            elif choice == "B":
                console.print("  Band for brute force test:")
                console.print(f"  [bold {APPLE[0]}]1)[/]  S-Band (default)")
                console.print(f"  [bold {APPLE[1]}]2)[/]  UHF")
                _prompt("  Select [1-2]: ")
                b = _wait_input(input_q).strip()
                band = 1 if b == "2" else 2

                _prompt("  Duration (seconds) [60]: ")
                d = _wait_input(input_q).strip()
                duration = int(d) if d.isdigit() else 60

                state.clear_bulk()
                image_transfer_active = False
                rec = send_command(ser, state, logger, "START_BULK", param=band)
                if rec and rec.acked and rec.result == 0:
                    if bulk_fig is None:
                        plt.ion()
                        bulk_fig, bulk_ax = init_bulk_preview()
                        plt.show(block=False)
                    auto_stop_time = time.time() + duration
                    console.print(
                        f"  [{APPLE[2]}]Brute force: "
                        f"{'UHF' if band == 1 else 'S-Band'} for "
                        f"{duration}s - auto-stop at end[/]"
                    )
                else:
                    console.print(
                        f"  [{APPLE[3]}]Brute force test was not confirmed[/]"
                    )

            elif choice == "L":
                snap = state.get_snapshot()
                logger.print_summary(snap)

            elif choice == "P":
                reader.stop()
                ser.close()
                # Port selection inline (avoids stdin conflict with input thread)
                ports = [p for p in serial.tools.list_ports.comports()
                         if "usb" in p.device.lower()
                         or "acm" in p.device.lower()
                         or "usbmodem" in p.device.lower()]
                if not ports:
                    ports = list(serial.tools.list_ports.comports())
                if not ports:
                    console.print(f"  [{APPLE[3]}]No serial ports found[/]")
                    break
                console.print("\n  Available ports:")
                for i, p in enumerate(ports):
                    c = APPLE[i % len(APPLE)]
                    console.print(f"  [bold {c}]{i+1})[/]  {p.device}  —  {p.description}")
                _prompt(f"\n  Select port [1-{len(ports)}]: ")
                p = _wait_input(input_q).strip()
                try:
                    idx = int(p) - 1
                    new_port = ports[idx].device if 0 <= idx < len(ports) else None
                except (ValueError, IndexError):
                    new_port = p if p.startswith("/dev/") else None
                if not new_port:
                    console.print(f"  [{APPLE[3]}]Invalid selection[/]")
                    break
                try:
                    ser = serial.Serial(new_port, 115200, timeout=0.1)
                    time.sleep(0.5)
                except serial.SerialException as e:
                    console.print(f"  [{APPLE[3]}]Could not open {new_port}: {e}[/]")
                    break
                console.print(f"  [{APPLE[0]}]Switched to {new_port}[/]")
                reader = SerialReader(ser, state, logger)
                reader.start()

            elif choice == "Q":
                break

            else:
                console.print(f"  [{APPLE[3]}]Unknown option: {choice}[/]")

            print_menu()
            _prompt()

    except KeyboardInterrupt:
        console.print(f"\n  [{APPLE[1]}]Shutting down...[/]")

    reader.stop()
    ser.close()
    logger.close()
    plt.close("all")
    console.print(f"  [{APPLE[0]}]Goodbye.[/]")


if __name__ == "__main__":
    main()
