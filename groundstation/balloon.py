#!/usr/bin/env python3
"""
Balloon Range Test 
River Dowdy — Winter 2025

Dependencies: pip install pyserial matplotlib rich
"""
import sys
import os
import time
import threading

# Add tools/ to path for shared utilities
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

from rich.console import Console

from _serial_bridge import pick_serial_port, open_serial, APPLE
from _protocol import (
    parse_line, build_cmd_line, build_nack_line,
    BeaconData, CmdAckData, BulkData,
    ImgStartData, ImgDataData, ImgEndData,
    PingTracker,
)
from _balloon_dashboard import (
    init_dashboard, DashboardState, update_dashboard,
)
from _link_budget import LinkBudgetLogger

console = Console(highlight=False)


# -- Banner --

BANNER = r"""
▖  ▖  ▜            ▗     ▗ ▌     ▄   ▜ ▜         ▄▖          ▄▖    ▗ 
▌▞▖▌█▌▐ ▛▘▛▌▛▛▌█▌  ▜▘▛▌  ▜▘▛▌█▌  ▙▘▀▌▐ ▐ ▛▌▛▌▛▌  ▙▘▀▌▛▌▛▌█▌  ▐ █▌▛▘▜▘
▛ ▝▌▙▖▐▖▙▖▙▌▌▌▌▙▖  ▐▖▙▌  ▐▖▌▌▙▖  ▙▘█▌▐▖▐▖▙▌▙▌▌▌  ▌▌█▌▌▌▙▌▙▖  ▐ ▙▖▄▌▐▖
                                                       ▄▌            
"""


def _print_banner():
    lines = BANNER.strip("\n").split("\n")
    for i, line in enumerate(lines):
        c = APPLE[i % len(APPLE)]
        console.print(f"[{c}]{line}[/]")
    console.print()


def _menu_item(num, label):
    c = APPLE[(num - 1) % len(APPLE)]
    console.print(f"  [bold {c}]{num})[/]  {label}")


# -- Serial reader thread --

class SerialReader:
    def __init__(self, ser):
        self.ser = ser
        self.running = False
        self._thread = None
        self._callbacks = []

    def add_callback(self, fn):
        self._callbacks.append(fn)

    def remove_callback(self, fn):
        try:
            self._callbacks.remove(fn)
        except ValueError:
            pass

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2)

    def _run(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                rec_type, data = parse_line(line)
                for cb in self._callbacks:
                    try:
                        cb(rec_type, data, line)
                    except Exception:
                        pass
            except Exception:
                if not self.running:
                    break
                time.sleep(0.1)


# -- Main CLI --

def cmd_dashboard(ser, reader, logger):
    """Run the live dashboard."""
    import matplotlib.pyplot as plt

    state = DashboardState()
    fig, ax_rssi, ax_snr, ax_rate, ax_margin, ax_log, ax_thru = \
        init_dashboard()

    beacon_count = [0]

    # Image tracking
    img_received = {}
    img_total = [0]
    img_first_pass_count = [0]
    img_retx_count = [0]
    img_active = [False]
    last_thru_time = [time.time()]
    last_thru_bytes = [0]

    def on_packet(rec_type, data, raw_line):
        if rec_type == "BEACON" and isinstance(data, BeaconData):
            state.add_beacon(data)
            logger.log_beacon(data)
            beacon_count[0] += 1

        elif rec_type == "CMD_ACK" and isinstance(data, CmdAckData):
            state.add_cmd_log(
                f"ACK cmd={data.cmd} seq={data.ack_seq} "
                f"status={data.status} "
                f"RSSI={data.cmd_rssi_x100 / 100.0:.1f}")

        elif rec_type == "BULK" and isinstance(data, BulkData):
            now = time.time()
            dt = now - last_thru_time[0]
            if dt > 0.5:
                kb = last_thru_bytes[0] / 1024.0
                state.add_throughput(kb / dt, now)
                last_thru_time[0] = now
                last_thru_bytes[0] = 0
            last_thru_bytes[0] += data.data_len

        elif rec_type == "IMG_START" and isinstance(data, ImgStartData):
            img_received.clear()
            img_total[0] = data.total_pkts
            img_first_pass_count[0] = 0
            img_retx_count[0] = 0
            img_active[0] = True
            state.add_cmd_log(
                f"IMG_START total={data.total_pkts}")

        elif rec_type == "IMG_DATA" and isinstance(data, ImgDataData):
            is_retx = data.pkt in img_received
            if is_retx:
                img_retx_count[0] += 1
            else:
                img_first_pass_count[0] += 1
            img_received[data.pkt] = bytes.fromhex(data.hex)

            now = time.time()
            dt = now - last_thru_time[0]
            if dt > 0.5:
                kb = last_thru_bytes[0] / 1024.0
                state.add_throughput(kb / dt, now)
                last_thru_time[0] = now
                last_thru_bytes[0] = 0
            last_thru_bytes[0] += data.data_len

        elif rec_type == "IMG_END" and isinstance(data, ImgEndData):
            total = img_total[0]
            first_prr = (img_first_pass_count[0] / total * 100
                         if total > 0 else 0)
            final_prr = (len(img_received) / total * 100
                         if total > 0 else 0)
            state.add_cmd_log(
                f"IMG_END  First-pass PRR: {first_prr:.0f}%  "
                f"After ARQ: {final_prr:.0f}%  "
                f"Retx: {img_retx_count[0]}")

            # Send NACKs if needed
            if total > 0:
                missing = [i for i in range(total)
                           if i not in img_received]
                if missing:
                    nack_line = build_nack_line(missing)
                    ser.write(nack_line.encode())
                    state.add_cmd_log(
                        f"NACK sent: {len(missing)} missing pkts")

    reader.add_callback(on_packet)

    console.print("  Dashboard running. Close the plot window to return.\n")

    plt.ion()
    plt.show(block=False)

    try:
        while plt.get_fignums():
            if beacon_count[0] >= 3:
                update_dashboard(fig, ax_rssi, ax_snr, ax_rate,
                                 ax_margin, ax_log, ax_thru, state)
                beacon_count[0] = 0
            plt.pause(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        reader.remove_callback(on_packet)
        plt.close("all")


def cmd_send_command(ser, ping_tracker):
    """Send a command to the balloon."""
    console.print("\n  Send command:")
    _menu_item(1, "Ping")
    _menu_item(2, "Set S-Band profile (1-4)")
    _menu_item(3, "Start image downlink (S-Band)")
    _menu_item(4, "Start bulk test — UHF (PRBS)")
    _menu_item(5, "Start bulk test — UHF (text)")
    _menu_item(6, "Start bulk test — S-Band (PRBS)")

    _menu_item(7, "Start bulk test — S-Band (text)")
    _menu_item(8, "Stop current activity")

    choice = input("\n  Select: ").strip()

    if choice == "1":
        seq = int(time.time()) & 0xFF
        line = build_cmd_line("PING", seq=seq)
        ser.write(line.encode())
        ping_tracker.record_send(seq)
        console.print(f"  Ping sent (seq={seq})")

    elif choice == "2":
        p = input("  Profile [1-4]: ").strip()
        if p in ("1", "2", "3", "4"):
            line = build_cmd_line("SET_SBAND_PROFILE", param1=int(p))
            ser.write(line.encode())
            console.print(f"  SET_SBAND_PROFILE param1={p}")
        else:
            console.print("  Invalid profile.")

    elif choice == "3":
        line = build_cmd_line("START_IMAGE", param1=0)
        ser.write(line.encode())
        console.print("  START_IMAGE sent")

    elif choice == "4":
        line = build_cmd_line("START_BULK", param1=1, param2=0)
        ser.write(line.encode())
        console.print("  START_BULK UHF PRBS sent")

    elif choice == "5":
        line = build_cmd_line("START_BULK", param1=1, param2=1)
        ser.write(line.encode())
        console.print("  START_BULK UHF text sent")

    elif choice == "6":
        line = build_cmd_line("START_BULK", param1=2, param2=0)
        ser.write(line.encode())
        console.print("  START_BULK S-Band PRBS sent")

    elif choice == "7":
        line = build_cmd_line("START_BULK", param1=2, param2=1)
        ser.write(line.encode())
        console.print("  START_BULK S-Band text sent")

    elif choice == "8":
        line = build_cmd_line("STOP")
        ser.write(line.encode())
        console.print("  STOP sent")

    else:
        console.print("  Invalid choice.")


def cmd_power_sweep(ser):
    """Send a power sweep command."""
    console.print("\n  Power Sweep (S-Band)")
    lo = input("  Min power dBm (e.g. -18): ").strip()
    hi = input("  Max power dBm (e.g. 13): ").strip()
    try:
        lo_val = int(lo)
        hi_val = int(hi)
    except ValueError:
        console.print("  Invalid input.")
        return
    line = build_cmd_line("POWER_SWEEP", param1=lo_val & 0xFF,
                          param2=hi_val & 0xFF)
    ser.write(line.encode())
    console.print(f"  POWER_SWEEP {lo_val} -> {hi_val} dBm sent")


def cmd_link_report(logger):
    """Print link budget summary."""
    logger.print_summary()


def main():
    _print_banner()

    port = pick_serial_port()
    ser = open_serial(port)
    if not ser:
        return

    console.print(f"  Connected: {port}")

    # Wait for READY from ground firmware
    console.print("  Waiting for ground station firmware...")
    deadline = time.time() + 5
    ready = False
    while time.time() < deadline:
        raw = ser.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").strip()
            if "READY" in line:
                ready = True
                break
    if ready:
        console.print(f"  [bold {APPLE[0]}]Ground station ready.[/]")
    else:
        console.print(f"  [bold {APPLE[3]}]Warning: no READY received "
                      f"(firmware may not be running).[/]")

    logger = LinkBudgetLogger()
    logger.open()

    ping_tracker = PingTracker()

    reader = SerialReader(ser)

    # Callback for printing ACKs in the main console
    def on_ack(rec_type, data, raw_line):
        if rec_type == "CMD_ACK" and isinstance(data, CmdAckData):
            rtt = ping_tracker.record_ack(data.ack_seq)
            rtt_str = f"  RTT: {rtt:.0f}ms" if rtt else ""
            console.print(
                f"  [bold {APPLE[4]}]ACK[/] cmd={data.cmd} "
                f"status={data.status} "
                f"RSSI={data.cmd_rssi_x100 / 100.0:.1f} dBm"
                f"{rtt_str}")

    reader.add_callback(on_ack)
    reader.start()

    try:
        while True:
            console.print()
            _menu_item(1, "Live dashboard")
            _menu_item(2, "Send command")
            _menu_item(3, "Power sweep")
            _menu_item(4, "Link budget report")
            _menu_item(5, "Change serial port")
            _menu_item(6, "Quit")

            choice = input("\n  Select mode: ").strip()

            if choice == "1":
                cmd_dashboard(ser, reader, logger)
            elif choice == "2":
                cmd_send_command(ser, ping_tracker)
            elif choice == "3":
                cmd_power_sweep(ser)
            elif choice == "4":
                cmd_link_report(logger)
            elif choice == "5":
                reader.stop()
                ser.close()
                port = pick_serial_port()
                ser = open_serial(port)
                if not ser:
                    return
                reader = SerialReader(ser)
                reader.add_callback(on_ack)
                reader.start()
                console.print(f"  Connected: {port}")
            elif choice == "6":
                break
            else:
                console.print("  Invalid choice.")

    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        ser.close()
        logger.close()
        console.print("\n  Goodbye.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n  Stopped.")
