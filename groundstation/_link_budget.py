"""
Link budget analysis and CSV logging for balloon range test.
"""
import csv
import datetime as dt
import os

from rich.console import Console
from rich.table import Table

console = Console(highlight=False)

# Receiver sensitivity reference (dBm)
SBAND_SENSITIVITY = {1: -117, 2: -120, 3: -123, 4: -129}
UHF_SENSITIVITY = -130

# Apple rainbow colors for rich output
APPLE = ["#61BB46", "#FDB827", "#F5821F", "#E03A3E", "#963D97", "#009DDC"]

CSV_FIELDS = [
    "ts_utc", "band", "profile", "tx_power_dbm",
    "rssi_dbm", "snr_db", "link_margin_db",
    "beacon_seq", "state",
]


_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


class LinkBudgetLogger:
    def __init__(self, csv_path=None):
        if csv_path is None:
            ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_path = os.path.join(_DATA_DIR, f"balloon_{ts}.csv")
        self.csv_path = csv_path
        self.rows = []
        self._file = None
        self._writer = None

    def open(self):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        is_new = not os.path.exists(self.csv_path) or \
                 os.path.getsize(self.csv_path) == 0
        self._file = open(self.csv_path, "a", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        if is_new:
            self._writer.writeheader()

    def close(self):
        if self._file:
            self._file.close()
            self._file = None

    def log_beacon(self, beacon):
        rssi_dbm = beacon.rssi_x100 / 100.0
        snr_db = beacon.snr_x100 / 100.0

        if beacon.band == "SBAND":
            floor = SBAND_SENSITIVITY.get(beacon.sband_profile, -120)
        else:
            floor = UHF_SENSITIVITY
        margin = rssi_dbm - floor

        row = {
            "ts_utc": dt.datetime.now(dt.timezone.utc).strftime(
                "%Y-%m-%dT%H:%M:%S.%fZ"),
            "band": beacon.band,
            "profile": beacon.sband_profile,
            "tx_power_dbm": beacon.tx_power_dbm,
            "rssi_dbm": f"{rssi_dbm:.2f}",
            "snr_db": f"{snr_db:.2f}",
            "link_margin_db": f"{margin:.2f}",
            "beacon_seq": beacon.seq,
            "state": beacon.state,
        }

        self.rows.append(row)
        if self._writer:
            self._writer.writerow(row)
            self._file.flush()

    def print_summary(self):
        if not self.rows:
            console.print("  No beacon data collected.")
            return

        table = Table(
            title="Link Budget Summary",
            show_header=True,
            header_style="bold",
            border_style="dim",
        )
        table.add_column("Band", style=f"bold {APPLE[0]}")
        table.add_column("Count", justify="right")
        table.add_column("RSSI min", justify="right")
        table.add_column("RSSI avg", justify="right")
        table.add_column("RSSI max", justify="right")
        table.add_column("SNR min", justify="right")
        table.add_column("SNR avg", justify="right")
        table.add_column("Margin min", justify="right", style=f"bold {APPLE[3]}")
        table.add_column("Margin avg", justify="right")

        bands = {}
        for r in self.rows:
            band = r["band"]
            if band not in bands:
                bands[band] = {"rssi": [], "snr": [], "margin": []}
            bands[band]["rssi"].append(float(r["rssi_dbm"]))
            bands[band]["snr"].append(float(r["snr_db"]))
            bands[band]["margin"].append(float(r["link_margin_db"]))

        for band in sorted(bands.keys()):
            d = bands[band]
            n = len(d["rssi"])
            rssi_min = min(d["rssi"])
            rssi_avg = sum(d["rssi"]) / n
            rssi_max = max(d["rssi"])
            snr_min = min(d["snr"])
            snr_avg = sum(d["snr"]) / n
            margin_min = min(d["margin"])
            margin_avg = sum(d["margin"]) / n

            table.add_row(
                band,
                str(n),
                f"{rssi_min:.1f}",
                f"{rssi_avg:.1f}",
                f"{rssi_max:.1f}",
                f"{snr_min:.1f}",
                f"{snr_avg:.1f}",
                f"{margin_min:.1f}",
                f"{margin_avg:.1f}",
            )

        console.print(table)
        console.print(f"\n  CSV saved: {self.csv_path}")
        console.print(f"  Total beacons: {len(self.rows)}")
