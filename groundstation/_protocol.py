"""
Balloon link test serial protocol — parser and command builders.

Ground firmware prints KEY,field=value lines over USB serial.
This module parses them into typed dataclasses and builds command strings.
"""

from dataclasses import dataclass
from typing import Optional


# -- Parsed record types --

@dataclass
class Beacon:
    uptime_ms: int = 0
    state: int = 0
    sband_profile: int = 0
    tx_power_dbm: int = 0
    flags: int = 0
    tx_count: int = 0
    rx_count: int = 0
    cmd_rssi_x100: int = 0
    cmd_snr_x100: int = 0
    rssi_x100: int = 0       # ground RX RSSI
    snr_x100: int = 0        # ground RX SNR

    @property
    def rssi_dbm(self) -> float:
        return self.rssi_x100 / 100.0

    @property
    def snr_db(self) -> float:
        return self.snr_x100 / 100.0

    @property
    def uhf_ok(self) -> bool:
        return bool(self.flags & 2)   # BALLOON_FLAG_UHF_OK = bit 1

    @property
    def sband_ok(self) -> bool:
        return bool(self.flags & 1)   # BALLOON_FLAG_SBAND_OK = bit 0

    @property
    def image_loaded(self) -> bool:
        return bool(self.flags & 4)


@dataclass
class CmdAck:
    cmd: int = 0
    seq: int = 0
    result: int = 0
    cmd_rssi_x100: int = 0
    cmd_snr_x100: int = 0
    rssi_x100: int = 0
    snr_x100: int = 0

    @property
    def rssi_dbm(self) -> float:
        return self.rssi_x100 / 100.0


@dataclass
class BulkData:
    pkt_num: int = 0
    total_pkts: int = 0
    data_len: int = 0
    band: int = 0
    rssi_x100: int = 0
    snr_x100: int = 0
    hex_data: str = ""

    @property
    def rssi_dbm(self) -> float:
        return self.rssi_x100 / 100.0

    @property
    def snr_db(self) -> float:
        return self.snr_x100 / 100.0

    @property
    def data_bytes(self) -> bytes:
        return bytes.fromhex(self.hex_data) if self.hex_data else b""


@dataclass
class CmdSent:
    cmd: int = 0
    param: int = 0
    seq: int = 0


@dataclass
class CmdFail:
    cmd: int = 0
    seq: int = 0
    reason: str = ""


# -- KV line parser --

STATE_NAMES = {0: "BEACON", 1: "IMAGE", 2: "BULK", 3: "POWER_SWEEP"}
CMD_NAMES = {0: "PING", 1: "SET_SBAND_PROFILE", 2: "START_IMAGE",
             3: "START_BULK", 4: "STOP", 5: "POWER_SWEEP"}
BAND_NAMES = {1: "UHF", 2: "S-Band"}
CMD_RESULT_NAMES = {
    0: "OK",
    1: "INVALID_PARAM",
    2: "UNAVAILABLE",
    3: "BUSY",
    4: "RADIO_ERR",
}


def _parse_kv(fields_str: str) -> dict:
    """Parse 'key=value,key=value' into a dict."""
    result = {}
    for pair in fields_str.split(","):
        if "=" in pair:
            k, v = pair.split("=", 1)
            result[k.strip()] = v.strip()
    return result


def _int(d: dict, key: str, default: int = 0) -> int:
    try:
        return int(d.get(key, default))
    except (ValueError, TypeError):
        return default


def parse_line(line: str):
    """Parse a KEY,field=value serial line into a typed record.

    Returns (record_type: str, record) or (None, None) on failure.
    """
    line = line.strip()
    if not line or "," not in line:
        return None, None

    tag, _, rest = line.partition(",")
    kv = _parse_kv(rest)

    if tag == "BEACON":
        return "BEACON", Beacon(
            uptime_ms=_int(kv, "uptime"),
            state=_int(kv, "state"),
            sband_profile=_int(kv, "sband_prof"),
            tx_power_dbm=_int(kv, "tx_pwr"),
            flags=_int(kv, "flags"),
            tx_count=_int(kv, "tx_cnt"),
            rx_count=_int(kv, "rx_cnt"),
            cmd_rssi_x100=_int(kv, "cmd_rssi"),
            cmd_snr_x100=_int(kv, "cmd_snr"),
            rssi_x100=_int(kv, "rssi"),
            snr_x100=_int(kv, "snr"),
        )

    if tag == "CMD_ACK":
        return "CMD_ACK", CmdAck(
            cmd=_int(kv, "cmd"),
            seq=_int(kv, "seq"),
            result=_int(kv, "result"),
            cmd_rssi_x100=_int(kv, "cmd_rssi"),
            cmd_snr_x100=_int(kv, "cmd_snr"),
            rssi_x100=_int(kv, "rssi"),
            snr_x100=_int(kv, "snr"),
        )

    if tag == "BULK":
        return "BULK", BulkData(
            pkt_num=_int(kv, "pkt"),
            total_pkts=_int(kv, "total"),
            data_len=_int(kv, "len"),
            band=_int(kv, "band"),
            rssi_x100=_int(kv, "rssi"),
            snr_x100=_int(kv, "snr"),
            hex_data=kv.get("hex", ""),
        )

    if tag == "CMD_SENT":
        return "CMD_SENT", CmdSent(
            cmd=_int(kv, "cmd"),
            param=_int(kv, "param"),
            seq=_int(kv, "seq"),
        )

    if tag == "CMD_FAIL":
        return "CMD_FAIL", CmdFail(
            cmd=_int(kv, "cmd"),
            seq=_int(kv, "seq"),
            reason=kv.get("reason", "unknown"),
        )

    return tag, kv


# -- Command builders --

def build_cmd_line(cmd_name: str, param: int = 0, seq: int = 0) -> str:
    return f"CMD,cmd={cmd_name},param={param},seq={seq}\n"


def build_nack_line(missing_indices: list) -> str:
    """Build a NACK line with missing packet indices as little-endian uint16 hex."""
    hex_parts = []
    for idx in missing_indices:
        hex_parts.append(f"{idx & 0xFF:02X}{(idx >> 8) & 0xFF:02X}")
    return "NACK,hex=" + "".join(hex_parts) + "\n"


def describe_cmd_result(result: int) -> str:
    return CMD_RESULT_NAMES.get(result, f"ERR({result})")
