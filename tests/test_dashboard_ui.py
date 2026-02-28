import os
import sys
import time
import unittest

os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib

matplotlib.use("Agg")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GROUNDSTATION_DIR = os.path.join(ROOT, "groundstation")
if GROUNDSTATION_DIR not in sys.path:
    sys.path.insert(0, GROUNDSTATION_DIR)

from _dashboard import _derive_dashboard_metrics, init_dashboard, update_dashboard
from _protocol import Beacon
from _state import CommandRecord


def make_snapshot(**overrides):
    snap = {
        "uhf_rssi": [],
        "uhf_snr": [],
        "sband_rssi": [],
        "sband_snr": [],
        "throughput": [],
        "last_beacon": None,
        "last_beacon_time": 0.0,
        "beacon_count": 0,
        "bulk_count": 0,
        "ack_count": 0,
        "commands": [],
        "bulk_received_count": 0,
        "bulk_total": 0,
        "bulk_band": 0,
        "bulk_received_keys": set(),
        "bulk_data": {},
    }
    snap.update(overrides)
    return snap


class DashboardUiTests(unittest.TestCase):
    def test_metrics_no_transfer(self):
        metrics = _derive_dashboard_metrics(make_snapshot(), 1)
        self.assertIsNone(metrics["packet_loss_pct"])
        self.assertIsNone(metrics["transfer_progress_pct"])
        self.assertEqual(metrics["freshness"], "WAITING")
        self.assertEqual(metrics["throughput_trend"], "steady")

    def test_metrics_beacon_freshness_thresholds(self):
        beacon = Beacon(
            uptime_ms=111000,
            state=0,
            sband_profile=1,
            tx_power_dbm=13,
            flags=0b111,
            tx_count=109,
            rx_count=4,
            rssi_x100=-2900,
            snr_x100=980,
        )
        live = _derive_dashboard_metrics(
            make_snapshot(last_beacon=beacon, last_beacon_time=time.time() - 2),
            1,
        )
        idle = _derive_dashboard_metrics(
            make_snapshot(last_beacon=beacon, last_beacon_time=time.time() - 7),
            1,
        )
        stale = _derive_dashboard_metrics(
            make_snapshot(last_beacon=beacon, last_beacon_time=time.time() - 12),
            1,
        )

        self.assertEqual(live["freshness"], "LIVE")
        self.assertEqual(idle["freshness"], "IDLE")
        self.assertEqual(stale["freshness"], "STALE")

    def test_metrics_packet_loss_and_progress(self):
        metrics = _derive_dashboard_metrics(
            make_snapshot(
                bulk_total=117,
                bulk_received_count=81,
            ),
            1,
        )
        self.assertEqual(metrics["missing_count"], 36)
        self.assertAlmostEqual(metrics["packet_loss_pct"], 30.7692307, places=5)
        self.assertAlmostEqual(metrics["transfer_progress_pct"], 69.2307692, places=5)

    def test_metrics_throughput_trend_labels(self):
        rising = _derive_dashboard_metrics(
            make_snapshot(throughput=[5.0, 5.1, 5.0, 5.0, 5.7]),
            1,
        )
        falling = _derive_dashboard_metrics(
            make_snapshot(throughput=[5.8, 5.7, 5.6, 5.5, 4.8]),
            1,
        )
        steady = _derive_dashboard_metrics(
            make_snapshot(throughput=[5.0, 5.1, 5.0, 5.1, 5.05]),
            1,
        )

        self.assertEqual(rising["throughput_trend"], "rising")
        self.assertEqual(falling["throughput_trend"], "falling")
        self.assertEqual(steady["throughput_trend"], "steady")

    def test_metrics_last_command_rtt(self):
        rec = CommandRecord(
            "PING",
            0,
            1,
            time.time() - 0.25,
            acked=True,
            ack_time=time.time(),
        )
        metrics = _derive_dashboard_metrics(
            make_snapshot(commands=[rec]),
            1,
        )
        self.assertIsNotNone(metrics["last_command_rtt_ms"])
        self.assertGreater(metrics["last_command_rtt_ms"], 0.0)

    def test_dashboard_smoke_render(self):
        fig, axes = init_dashboard()
        empty = make_snapshot()
        update_dashboard(fig, axes, empty, 1)

        beacon = Beacon(
            uptime_ms=111000,
            state=0,
            sband_profile=1,
            tx_power_dbm=13,
            flags=0b111,
            tx_count=109,
            rx_count=4,
            rssi_x100=-2900,
            snr_x100=980,
        )
        rec = CommandRecord(
            "PING",
            0,
            1,
            time.time() - 0.2,
            acked=True,
            ack_time=time.time(),
        )
        active = make_snapshot(
            uhf_rssi=[-29.4, -29.2, -29.0],
            uhf_snr=[9.2, 9.5, 9.8],
            sband_rssi=[-10.4, -10.2, -10.0],
            throughput=[5.0, 5.1, 5.0, 5.2, 5.8],
            last_beacon=beacon,
            last_beacon_time=time.time() - 3,
            beacon_count=71,
            bulk_count=81,
            ack_count=5,
            commands=[rec],
            bulk_received_count=81,
            bulk_total=117,
        )
        update_dashboard(fig, axes, active, 1)
        self.assertEqual(axes["status"].get_title(), "Status")


if __name__ == "__main__":
    unittest.main()
