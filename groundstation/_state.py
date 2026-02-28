"""
Thread-safe ground station state for the balloon link test.

SerialReader updates state from its daemon thread.
Dashboard reads snapshots from the main thread.
"""

import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Optional

from _protocol import Beacon, CmdAck, BulkData, CmdSent, CmdFail, CMD_NAMES


@dataclass
class CommandRecord:
    cmd_name: str
    param: int
    seq: int
    sent_time: float
    acked: bool = False
    ack_time: float = 0.0
    result: int = -1
    failed: bool = False
    fail_reason: str = ""
    radio_sent: bool = False
    radio_sent_time: float = 0.0

    @property
    def rtt_ms(self) -> float:
        if self.acked and self.ack_time > self.sent_time:
            return (self.ack_time - self.sent_time) * 1000.0
        return -1.0


class GroundState:
    def __init__(self, maxlen: int = 500):
        self._lock = threading.Lock()
        self.uhf_rssi: deque = deque(maxlen=maxlen)
        self.uhf_snr: deque = deque(maxlen=maxlen)
        self.sband_rssi: deque = deque(maxlen=maxlen)
        self.sband_snr: deque = deque(maxlen=maxlen)
        self.throughput: deque = deque(maxlen=maxlen)
        self.last_beacon: Optional[Beacon] = None
        self.last_beacon_time: float = 0.0
        self.command_log: list = []
        self.bulk_received: dict = {}   # pkt_num -> bytes
        self.bulk_total: int = 0
        self.bulk_band: int = 0
        self.bulk_start_time: float = 0.0
        self.bulk_times: deque = deque(maxlen=maxlen)
        self.bulk_text_preview: str = ""
        self._seq_counter: int = 0
        self.beacon_count: int = 0
        self.bulk_count: int = 0
        self.ack_count: int = 0

    def next_seq(self) -> int:
        with self._lock:
            self._seq_counter = (self._seq_counter + 1) & 0xFF
            return self._seq_counter

    def _find_command_locked(self, seq: int) -> Optional[CommandRecord]:
        for rec in reversed(self.command_log):
            if rec.seq == seq:
                return rec
        return None

    def _find_pending_command_locked(self, cmd_name: str) -> Optional[CommandRecord]:
        for rec in reversed(self.command_log):
            if rec.cmd_name == cmd_name and not rec.acked and not rec.failed:
                return rec
        return None

    def record_beacon(self, bcn: Beacon):
        with self._lock:
            self.last_beacon = bcn
            self.last_beacon_time = time.time()
            self.beacon_count += 1
            self.uhf_rssi.append(bcn.rssi_dbm)
            self.uhf_snr.append(bcn.snr_db)

    def record_bulk(self, bulk: BulkData):
        with self._lock:
            now = time.time()
            self.bulk_count += 1

            if bulk.total_pkts > 0:
                if self.bulk_total != bulk.total_pkts:
                    self.bulk_received.clear()
                    self.bulk_total = bulk.total_pkts
                    self.bulk_band = bulk.band
                    self.bulk_start_time = now
                    self.bulk_times.clear()
                    self.bulk_text_preview = ""
            else:
                self.bulk_total = 0
                self.bulk_band = bulk.band
                if self.bulk_start_time == 0.0:
                    self.bulk_start_time = now
                chunk = bulk.data_bytes
                if chunk:
                    decoded = chunk.decode("utf-8", errors="replace")
                    self.bulk_text_preview = (self.bulk_text_preview + decoded)[-12000:]

            self.bulk_received[bulk.pkt_num] = bulk.data_bytes
            self.bulk_times.append(now)

            if bulk.band == 2:
                self.sband_rssi.append(bulk.rssi_dbm)
                self.sband_snr.append(bulk.snr_db)
            else:
                self.uhf_rssi.append(bulk.rssi_dbm)
                self.uhf_snr.append(bulk.snr_db)

            # Throughput: sliding window of last 20 packets
            window = min(20, len(self.bulk_times))
            if window >= 2:
                dt = self.bulk_times[-1] - self.bulk_times[-window]
                if dt > 0.01:
                    pkt_bytes = bulk.data_len if bulk.data_len > 0 else 243
                    self.throughput.append(pkt_bytes * window / dt / 1024.0)

    def record_ack(self, ack: CmdAck):
        with self._lock:
            self.ack_count += 1
            rec = self._find_command_locked(ack.seq)
            if rec is None:
                cmd_name = CMD_NAMES.get(ack.cmd)
                if cmd_name:
                    rec = self._find_pending_command_locked(cmd_name)
            if rec and not rec.acked:
                rec.acked = True
                rec.failed = False
                rec.fail_reason = ""
                rec.ack_time = time.time()
                rec.result = ack.result
            return replace(rec) if rec else None

    def submit_command(self, cmd_name: str, param: int = 0) -> int:
        seq = self.next_seq()
        rec = CommandRecord(
            cmd_name=cmd_name, param=param,
            seq=seq, sent_time=time.time()
        )
        with self._lock:
            self.command_log.append(rec)
        return seq

    def record_cmd_sent(self, sent: CmdSent):
        with self._lock:
            rec = self._find_command_locked(sent.seq)
            if rec is None:
                cmd_name = CMD_NAMES.get(sent.cmd)
                if cmd_name:
                    rec = self._find_pending_command_locked(cmd_name)
            if rec:
                rec.radio_sent = True
                rec.radio_sent_time = time.time()
            return replace(rec) if rec else None

    def record_cmd_fail(self, fail: CmdFail):
        with self._lock:
            rec = self._find_command_locked(fail.seq)
            if rec is None:
                cmd_name = CMD_NAMES.get(fail.cmd)
                if cmd_name:
                    rec = self._find_pending_command_locked(cmd_name)
            if rec is None:
                for cand in reversed(self.command_log):
                    if not cand.acked and not cand.failed:
                        rec = cand
                        break
            if rec and not rec.acked:
                rec.failed = True
                rec.fail_reason = fail.reason
            return replace(rec) if rec else None

    def mark_command_timeout(self, seq: int, reason: str = "no_ack"):
        with self._lock:
            rec = self._find_command_locked(seq)
            if rec and not rec.acked and not rec.failed:
                rec.failed = True
                rec.fail_reason = reason

    def get_command(self, seq: int) -> Optional[CommandRecord]:
        with self._lock:
            rec = self._find_command_locked(seq)
            return replace(rec) if rec else None

    def clear_bulk(self):
        with self._lock:
            self.bulk_received.clear()
            self.bulk_total = 0
            self.bulk_band = 0
            self.bulk_start_time = 0.0
            self.bulk_text_preview = ""
            self.bulk_times.clear()
            self.throughput.clear()

    def get_snapshot(self):
        """Return a dict snapshot for dashboard rendering."""
        with self._lock:
            return {
                "uhf_rssi": list(self.uhf_rssi),
                "uhf_snr": list(self.uhf_snr),
                "sband_rssi": list(self.sband_rssi),
                "sband_snr": list(self.sband_snr),
                "throughput": list(self.throughput),
                "last_beacon": self.last_beacon,
                "last_beacon_time": self.last_beacon_time,
                "beacon_count": self.beacon_count,
                "bulk_count": self.bulk_count,
                "ack_count": self.ack_count,
                "commands": list(self.command_log[-20:]),
                "bulk_received_count": len(self.bulk_received),
                "bulk_total": self.bulk_total,
                "bulk_band": self.bulk_band,
                "bulk_received_keys": set(self.bulk_received.keys()),
                "bulk_data": dict(self.bulk_received),
                "bulk_text_preview": self.bulk_text_preview,
            }
