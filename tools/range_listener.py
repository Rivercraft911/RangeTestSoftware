#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import sys
from collections import defaultdict
import serial


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
        return int(data.get(key, default))
    except ValueError:
        return default


def to_float(v, default=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def print_summary(data):
    band = data.get("band", "?")
    cfg = data.get("cfg", "?")
    mode = data.get("mode", "?")
    direction = data.get("dir", "?")
    pwr = data.get("pwr_dbm", "?")
    window_id = data.get("window_id", "?")

    tx = to_int(data, "tx", 0)
    rx = to_int(data, "rx", 0)
    lost = max(0, tx - rx)
    crc = to_int(data, "crc", 0)
    prr = to_int(data, "prr_x100", 0) / 100.0
    drop = to_int(data, "drop_x100", 0) / 100.0
    goodput = to_int(data, "goodput_bps", 0) / 1000.0
    rssi = to_int(data, "rssi_mean_x100", 0) / 100.0
    snr = to_int(data, "snr_mean_x100", 0) / 100.0

    print(f"Window {window_id} | Mode: {mode} | Band: {band} | Config: {cfg} | Dir: {direction} | Power: {pwr} dBm")
    print(f"Sent: {tx} | Recv: {rx} | Lost: {lost} | CRC: {crc} | PRR: {prr:.2f}% | Drop: {drop:.2f}%")
    print(f"Goodput: {goodput:.2f} kbps | RSSI mean: {rssi:.2f} dBm | SNR mean: {snr:.2f} dB")


def compute_threshold(stats_by_key):
    # key: (band, cfg, dir) -> list[(pwr, prr)]
    out = {}
    for key, rows in stats_by_key.items():
        rows_sorted = sorted(rows, key=lambda x: x[0])
        thresh = None
        for i in range(len(rows_sorted) - 2):
            p0, prr0 = rows_sorted[i]
            _, prr1 = rows_sorted[i + 1]
            _, prr2 = rows_sorted[i + 2]
            if prr0 >= 99.0 and prr1 >= 99.0 and prr2 >= 99.0:
                thresh = p0
                break
        out[key] = thresh
    return out


def print_rollup(rows, summary_samples):
    if not rows:
        return

    band_totals = defaultdict(list)
    key_rows = defaultdict(list)
    for r in rows:
        band = r["band"]
        cfg = r["cfg"]
        direction = r["dir"]
        pwr = r["pwr_dbm"]
        prr = r["prr"]
        band_totals[band].append(prr)
        key_rows[(band, cfg, direction)].append((pwr, prr))

    print(f"ROLLUP | summaries={summary_samples}")
    for band, vals in sorted(band_totals.items()):
        avg = sum(vals) / len(vals)
        worst = min(vals)
        print(f"  {band}: avg PRR={avg:.2f}% | worst PRR={worst:.2f}%")

    thresh = compute_threshold(key_rows)
    for key, pwr in sorted(thresh.items()):
        band, cfg, direction = key
        pwr_str = "N/A"
        if pwr is not None:
            pwr_str = f"{pwr} dBm"
        print(f"  threshold({band} cfg={cfg} dir={direction}) => {pwr_str}")


def main():
    ap = argparse.ArgumentParser(description="Range test listener (observer serial parser)")
    ap.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbmodem2101")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("--csv", default="range_listener.csv", help="CSV output path")
    ap.add_argument("--rollup-every", type=int, default=20, help="Print rollup every N summaries")
    args = ap.parse_args()

    csv_fields = [
        "ts_utc",
        "test_id",
        "window_id",
        "band",
        "mode",
        "cfg",
        "dir",
        "pwr_dbm",
        "tx",
        "rx",
        "lost",
        "crc",
        "prr_percent",
        "drop_percent",
        "goodput_bps",
        "rssi_mean_dbm",
        "snr_mean_db",
    ]

    summary_rows = []
    summary_count = 0

    with serial.Serial(args.port, args.baud, timeout=0.25) as ser, open(args.csv, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=csv_fields)
        if f.tell() == 0:
            writer.writeheader()

        print(f"Listening on {args.port} @ {args.baud} | CSV: {args.csv}")

        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue

            rec_type, data = parse_kv_line(line)
            if rec_type == "HEARTBEAT":
                frm = data.get("from", "?")
                band = data.get("band", "?")
                mode = data.get("mode", "?")
                cfg = data.get("cfg", "?")
                win = data.get("window_id", "?")
                print(f"HB from {frm} | mode={mode} band={band} cfg={cfg} window={win}")
                continue

            if rec_type == "CONFIG":
                band = data.get("band", "?")
                mode = data.get("mode", "?")
                cfg = data.get("cfg", "?")
                direction = data.get("dir", "?")
                pwr = data.get("pwr_dbm", "?")
                print(f"CFG | mode={mode} band={band} cfg={cfg} dir={direction} pwr={pwr} dBm")
                continue

            if rec_type != "SUMMARY":
                # Ignore any other line types without failing the run.
                continue

            print_summary(data)

            tx = to_int(data, "tx", 0)
            rx = to_int(data, "rx", 0)
            lost = max(0, tx - rx)
            prr = to_int(data, "prr_x100", 0) / 100.0
            drop = to_int(data, "drop_x100", 0) / 100.0
            goodput = to_int(data, "goodput_bps", 0)
            rssi = to_int(data, "rssi_mean_x100", 0) / 100.0
            snr = to_int(data, "snr_mean_x100", 0) / 100.0

            row = {
                "ts_utc": dt.datetime.utcnow().isoformat(timespec="seconds") + "Z",
                "test_id": to_int(data, "test_id", 0),
                "window_id": to_int(data, "window_id", 0),
                "band": data.get("band", ""),
                "mode": data.get("mode", ""),
                "cfg": to_int(data, "cfg", 0),
                "dir": data.get("dir", ""),
                "pwr_dbm": to_int(data, "pwr_dbm", 0),
                "tx": tx,
                "rx": rx,
                "lost": lost,
                "crc": to_int(data, "crc", 0),
                "prr_percent": prr,
                "drop_percent": drop,
                "goodput_bps": goodput,
                "rssi_mean_dbm": rssi,
                "snr_mean_db": snr,
            }
            writer.writerow(row)
            f.flush()

            summary_rows.append(
                {
                    "band": row["band"],
                    "cfg": row["cfg"],
                    "dir": row["dir"],
                    "pwr_dbm": row["pwr_dbm"],
                    "prr": row["prr_percent"],
                }
            )
            summary_count += 1

            if args.rollup_every > 0 and (summary_count % args.rollup_every) == 0:
                print_rollup(summary_rows, summary_count)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nListener stopped.")
