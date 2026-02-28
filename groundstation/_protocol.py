"""
Packet parsing and command building for the balloon test ground station.
"""
import time
from collections import namedtuple


# --- Record types from firmware serial output ---

BeaconData = namedtuple("BeaconData", [
    "band", "seq", "uptime_ms", "tx_count", "rx_count",
    "last_cmd_rssi_x100", "last_cmd_snr_x100",
    "tx_power_dbm", "sband_profile", "state", "flags",
    "rssi_x100", "snr_x100",
])

CmdAckData = namedtuple("CmdAckData", [
    "cmd", "ack_seq", "status",
    "cmd_rssi_x100", "cmd_snr_x100",
    "rssi_x100", "snr_x100",
])

BulkData = namedtuple("BulkData", [
    "band", "seq", "seed", "data_len",
    "rssi_x100", "snr_x100", "hex",
])

ImgStartData = namedtuple("ImgStartData", ["total_pkts"])
ImgDataData = namedtuple("ImgDataData", [
    "pkt", "data_len", "rssi_x100", "snr_x100", "hex",
])
ImgEndData = namedtuple("ImgEndData", ["total_pkts"])


# --- Key-value line parsing (same pattern as range_listener.py) ---

def parse_kv_line(line: str):
    parts = line.strip().split(",")
    if not parts:
        return None, {}
    rec_type = parts[0].strip()
    data = {}
    for part in parts[1:]:
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        data[k.strip()] = v.strip()
    return rec_type, data


def to_int(data, key, default=0):
    try:
        v = data.get(key, default)
        return int(str(v), 0)  # base=0 auto-detects 0x hex prefix
    except (TypeError, ValueError):
        return default


# --- Parse a KV dict into a typed record ---

def parse_beacon(data):
    return BeaconData(
        band=data.get("band", "?"),
        seq=to_int(data, "seq"),
        uptime_ms=to_int(data, "uptime_ms"),
        tx_count=to_int(data, "tx_count"),
        rx_count=to_int(data, "rx_count"),
        last_cmd_rssi_x100=to_int(data, "last_cmd_rssi_x100"),
        last_cmd_snr_x100=to_int(data, "last_cmd_snr_x100"),
        tx_power_dbm=to_int(data, "tx_power_dbm"),
        sband_profile=to_int(data, "sband_profile"),
        state=to_int(data, "state"),
        flags=to_int(data, "flags"),
        rssi_x100=to_int(data, "rssi_x100"),
        snr_x100=to_int(data, "snr_x100"),
    )


def parse_cmd_ack(data):
    return CmdAckData(
        cmd=to_int(data, "cmd"),
        ack_seq=to_int(data, "ack_seq"),
        status=to_int(data, "status"),
        cmd_rssi_x100=to_int(data, "cmd_rssi_x100"),
        cmd_snr_x100=to_int(data, "cmd_snr_x100"),
        rssi_x100=to_int(data, "rssi_x100"),
        snr_x100=to_int(data, "snr_x100"),
    )


def parse_bulk(data):
    return BulkData(
        band=data.get("band", "?"),
        seq=to_int(data, "seq"),
        seed=to_int(data, "seed"),
        data_len=to_int(data, "len"),
        rssi_x100=to_int(data, "rssi_x100"),
        snr_x100=to_int(data, "snr_x100"),
        hex=data.get("hex", ""),
    )


def parse_img_start(data):
    return ImgStartData(total_pkts=to_int(data, "total_pkts"))


def parse_img_data(data):
    return ImgDataData(
        pkt=to_int(data, "pkt"),
        data_len=to_int(data, "len"),
        rssi_x100=to_int(data, "rssi_x100"),
        snr_x100=to_int(data, "snr_x100"),
        hex=data.get("hex", ""),
    )


def parse_img_end(data):
    return ImgEndData(total_pkts=to_int(data, "total_pkts"))


# --- Dispatch parser ---

PARSERS = {
    "BEACON":   parse_beacon,
    "CMD_ACK":  parse_cmd_ack,
    "BULK":     parse_bulk,
    "IMG_START": parse_img_start,
    "IMG_DATA": parse_img_data,
    "IMG_END":  parse_img_end,
}


def parse_line(line: str):
    rec_type, data = parse_kv_line(line)
    parser = PARSERS.get(rec_type)
    if parser:
        return rec_type, parser(data)
    return rec_type, data


# --- Command builders ---

CMD_NAMES = {
    "PING": 0,
    "SET_SBAND_PROFILE": 1,
    "START_IMAGE": 2,
    "START_BULK": 3,
    "STOP": 4,
    "POWER_SWEEP": 5,
}


def build_cmd_line(cmd_name, seq=0, param1=0, param2=0):
    return f"CMD,cmd={cmd_name},param1={param1},param2={param2},seq={seq}\n"


def build_nack_line(missing_pkts):
    hex_data = "".join(f"{p:04x}" for p in missing_pkts)
    return f"CMD,cmd=IMG_NACK,hex={hex_data}\n"


# --- Ping RTT tracker ---

class PingTracker:
    def __init__(self):
        self._pending = {}  # seq -> send_time

    def record_send(self, seq):
        self._pending[seq] = time.time()

    def record_ack(self, ack_seq):
        send_time = self._pending.pop(ack_seq, None)
        if send_time is not None:
            return (time.time() - send_time) * 1000.0  # ms
        return None

    def clear_stale(self, max_age_s=30):
        cutoff = time.time() - max_age_s
        self._pending = {k: v for k, v in self._pending.items()
                         if v > cutoff}
