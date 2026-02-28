"""
CSV logger for balloon link test sessions.

Logs every telemetry sample, command, and ACK for post-test analysis.
"""

import csv
import os
import time
from datetime import datetime
from pathlib import Path

from _protocol import Beacon, CmdAck, BulkData, BAND_NAMES, CMD_NAMES, STATE_NAMES


class BalloonLogger:
    def __init__(self, data_dir: str = None):
        if data_dir is None:
            data_dir = os.path.join(os.path.dirname(__file__), "data")
        Path(data_dir).mkdir(parents=True, exist_ok=True)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(data_dir, f"{ts}_balloon.csv")
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "timestamp", "record_type", "band",
            "rssi_dbm", "snr_db",
            "uptime_ms", "state", "sband_prof", "tx_pwr",
            "flags", "tx_cnt", "rx_cnt",
            "cmd", "seq", "result", "param",
            "pkt_num", "total_pkts", "data_len",
        ])
        self._start = time.time()

    def _ts(self) -> str:
        return f"{time.time() - self._start:.3f}"

    def log_beacon(self, bcn: Beacon):
        self._writer.writerow([
            self._ts(), "BEACON", "UHF",
            f"{bcn.rssi_dbm:.1f}", f"{bcn.snr_db:.1f}",
            bcn.uptime_ms, bcn.state, bcn.sband_profile, bcn.tx_power_dbm,
            bcn.flags, bcn.tx_count, bcn.rx_count,
            "", "", "", "",
            "", "", "",
        ])
        self._file.flush()

    def log_ack(self, ack: CmdAck):
        self._writer.writerow([
            self._ts(), "CMD_ACK", "UHF",
            f"{ack.rssi_dbm:.1f}", f"{ack.snr_x100 / 100:.1f}",
            "", "", "", "",
            "", "", "",
            ack.cmd, ack.seq, ack.result, "",
            "", "", "",
        ])
        self._file.flush()

    def log_bulk(self, bulk: BulkData):
        band = BAND_NAMES.get(bulk.band, f"?{bulk.band}")
        self._writer.writerow([
            self._ts(), "BULK", band,
            f"{bulk.rssi_dbm:.1f}", f"{bulk.snr_db:.1f}",
            "", "", "", "",
            "", "", "",
            "", "", "", "",
            bulk.pkt_num, bulk.total_pkts, bulk.data_len,
        ])
        self._file.flush()

    def log_cmd(self, cmd_name: str, param: int, seq: int):
        self._writer.writerow([
            self._ts(), "CMD_SENT", "UHF",
            "", "",
            "", "", "", "",
            "", "", "",
            cmd_name, seq, "", param,
            "", "", "",
        ])
        self._file.flush()

    def close(self):
        self._file.close()

    def print_summary(self, state_snapshot: dict):
        """Print a rich summary table from state snapshot."""
        from rich.console import Console
        from rich.table import Table

        console = Console(highlight=False)

        uhf_rssi = state_snapshot["uhf_rssi"]
        uhf_snr = state_snapshot["uhf_snr"]
        sband_rssi = state_snapshot["sband_rssi"]
        sband_snr = state_snapshot["sband_snr"]

        table = Table(title="Link Budget Summary", show_lines=True)
        table.add_column("Metric", style="bold")
        table.add_column("UHF", justify="right")
        table.add_column("S-Band", justify="right")

        def stats(data):
            if not data:
                return "—"
            mn, mx, avg = min(data), max(data), sum(data) / len(data)
            return f"{avg:.1f}  ({mn:.1f} .. {mx:.1f})"

        table.add_row("RSSI (dBm)", stats(uhf_rssi), stats(sband_rssi))
        table.add_row("SNR (dB)", stats(uhf_snr), stats(sband_snr))
        table.add_row("Samples", str(len(uhf_rssi)), str(len(sband_rssi)))
        table.add_row("Beacons", str(state_snapshot["beacon_count"]), "—")
        table.add_row("Bulk pkts", "—", str(state_snapshot["bulk_count"]))
        table.add_row("ACKs", str(state_snapshot["ack_count"]), "—")

        console.print(table)
        console.print(f"  Log file: {self.path}")
