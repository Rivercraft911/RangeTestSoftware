"""
Live dashboard for balloon link-budget testing.

Clean light scientific theme with Apple rainbow accent colors.
2x3 matplotlib grid: UHF RSSI, S-Band RSSI, Status, UHF SNR, Throughput, Packet Stats.
Separate image preview window for progressive rendering during transfer.
"""

import io
import time

import matplotlib
import matplotlib.pyplot as plt
from PIL import Image, ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

# -- Light scientific theme --
BG    = "#FFFFFF"
FACE  = "#F7F8FA"
TEXT  = "#1A1A2E"
GRID  = "#E2E4E8"
SPINE = "#C0C4CC"

# -- Apple rainbow accents (vivid on white) --
GREEN  = "#2E9E45"
YELLOW = "#D4A017"
ORANGE = "#E8590C"
RED    = "#DC3545"
PURPLE = "#7B2D8E"
BLUE   = "#1A73E8"

# Sensitivity floors (dBm)
UHF_FLOOR = -130
SBAND_FLOORS = {1: -117, 2: -120, 3: -126, 4: -129}

BULK_DATA_PER_PKT = 243  # BALLOON_BULK_DATA_MAX


def apply_dark_theme():
    """Apply the light scientific theme globally."""
    plt.rcParams.update({
        "figure.facecolor": BG,
        "axes.facecolor": FACE,
        "text.color": TEXT,
        "axes.labelcolor": TEXT,
        "xtick.color": TEXT,
        "ytick.color": TEXT,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.alpha": 0.7,
        "grid.linewidth": 0.5,
        "grid.linestyle": "-",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.edgecolor": SPINE,
        "axes.linewidth": 1.0,
        "font.family": "sans-serif",
        "font.sans-serif": [
            "Helvetica Neue", "Arial", "SF Pro Display", "sans-serif"
        ],
        "font.size": 10,
        "axes.titlesize": 13,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "lines.linewidth": 2.2,
        "lines.antialiased": True,
        "legend.frameon": True,
        "legend.framealpha": 0.9,
        "legend.facecolor": BG,
        "legend.edgecolor": SPINE,
        "legend.fontsize": 9,
        "axes.titlepad": 10,
        "axes.labelpad": 8,
        "figure.dpi": 150,
        "savefig.dpi": 200,
        "savefig.facecolor": BG,
        "axes.unicode_minus": False,
    })


def _margin_color(rssi: float, floor: float) -> str:
    margin = rssi - floor
    if margin > 10:
        return GREEN
    if margin > 3:
        return ORANGE
    return RED


def _stable_ylim(data, pad_frac=0.15, min_range=5.0):
    """Compute stable y-axis limits with padding."""
    if not data:
        return -100, 0
    lo, hi = min(data), max(data)
    span = max(hi - lo, min_range)
    pad = span * pad_frac
    return lo - pad, hi + pad


def _plot_rssi(ax, data, color, title, floor, label):
    ax.clear()
    ax.set_facecolor(FACE)
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.set_xlabel("Sample")
    ax.set_ylabel("dBm")

    if data:
        x = list(range(len(data)))
        ax.plot(x, data, color=color, linewidth=2.2, alpha=0.9)
        ax.fill_between(x, data, color=color, alpha=0.12)

        avg = sum(data) / len(data)
        ax.axhline(y=avg, color=color, linewidth=1.0, linestyle="--", alpha=0.5)
        ax.annotate(f"avg {avg:.1f}", xy=(0.02, 0.95),
                    xycoords="axes fraction", fontsize=9,
                    color=color, va="top", fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.2", fc=BG, ec="none", alpha=0.8))

        cur = data[-1]
        mc = _margin_color(cur, floor)
        margin = cur - floor
        ax.annotate(f"now {cur:.1f}  margin {margin:.0f} dB",
                    xy=(0.98, 0.95), xycoords="axes fraction",
                    fontsize=9, color=mc, va="top", ha="right", fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.2", fc=BG, ec="none", alpha=0.8))

        lo, hi = _stable_ylim(data, min_range=10.0)
        ax.set_ylim(min(lo, floor - 5), hi)
    else:
        ax.set_ylim(floor - 10, 0)

    ax.axhline(y=floor, color=RED, linewidth=1.0, linestyle=":", alpha=0.4)
    ax.annotate(f"floor {floor}", xy=(0.98, 0.02),
                xycoords="axes fraction", fontsize=8,
                color=RED, alpha=0.6, va="bottom", ha="right")


def _plot_snr(ax, data, color, title):
    ax.clear()
    ax.set_facecolor(FACE)
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.set_xlabel("Sample")
    ax.set_ylabel("dB")

    if data:
        x = list(range(len(data)))
        ax.plot(x, data, color=color, linewidth=2.2, alpha=0.9)
        ax.fill_between(x, data, color=color, alpha=0.12)

        avg = sum(data) / len(data)
        ax.axhline(y=avg, color=color, linewidth=1.0, linestyle="--", alpha=0.5)
        ax.annotate(f"avg {avg:.1f}", xy=(0.02, 0.95),
                    xycoords="axes fraction", fontsize=9,
                    color=color, va="top", fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.2", fc=BG, ec="none", alpha=0.8))
        ax.annotate(f"now {data[-1]:.1f}", xy=(0.98, 0.95),
                    xycoords="axes fraction", fontsize=9,
                    color=color, va="top", ha="right", fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.2", fc=BG, ec="none", alpha=0.8))

        lo, hi = _stable_ylim(data, min_range=3.0)
        ax.set_ylim(lo, hi)
    else:
        ax.set_ylim(-5, 15)


def init_dashboard():
    """Create the main 2x3 dashboard figure."""
    apply_dark_theme()

    fig, axes = plt.subplots(2, 3, figsize=(16, 9),
                             facecolor=BG, constrained_layout=True)
    fig.canvas.manager.set_window_title("Balloon Link Budget Dashboard")

    ax_uhf_rssi  = axes[0, 0]
    ax_sb_rssi   = axes[0, 1]
    ax_status    = axes[0, 2]
    ax_uhf_snr   = axes[1, 0]
    ax_throughput = axes[1, 1]
    ax_pkt_stats = axes[1, 2]

    ax_status.set_facecolor(FACE)
    ax_status.axis("off")

    fig.canvas.draw()
    fig.canvas.flush_events()

    return fig, {
        "uhf_rssi": ax_uhf_rssi,
        "sb_rssi": ax_sb_rssi,
        "status": ax_status,
        "uhf_snr": ax_uhf_snr,
        "throughput": ax_throughput,
        "pkt_stats": ax_pkt_stats,
    }


def update_dashboard(fig, axes, snap, sband_profile=1):
    """Refresh all dashboard plots from a state snapshot."""
    sband_floor = SBAND_FLOORS.get(sband_profile, -120)

    _plot_rssi(axes["uhf_rssi"], snap["uhf_rssi"], GREEN,
               "UHF RSSI", UHF_FLOOR, "UHF")

    _plot_rssi(axes["sb_rssi"], snap["sband_rssi"], PURPLE,
               "S-Band RSSI", sband_floor, "S-Band")

    _plot_snr(axes["uhf_snr"], snap["uhf_snr"], YELLOW, "UHF SNR")

    # Throughput
    ax_t = axes["throughput"]
    ax_t.clear()
    ax_t.set_facecolor(FACE)
    ax_t.set_title("Throughput (KB/s)", fontsize=13, fontweight="bold")
    ax_t.set_xlabel("Sample")
    tp = snap["throughput"]
    if tp:
        x = list(range(len(tp)))
        ax_t.plot(x, tp, color=ORANGE, linewidth=2.2, alpha=0.9)
        ax_t.fill_between(x, tp, color=ORANGE, alpha=0.12)
        avg = sum(tp) / len(tp)
        ax_t.annotate(f"avg {avg:.1f} KB/s", xy=(0.02, 0.95),
                      xycoords="axes fraction", fontsize=9,
                      color=ORANGE, va="top", fontweight="bold",
                      bbox=dict(boxstyle="round,pad=0.2", fc=BG, ec="none", alpha=0.8))
        ax_t.annotate(f"now {tp[-1]:.1f}", xy=(0.98, 0.95),
                      xycoords="axes fraction", fontsize=9,
                      color=ORANGE, va="top", ha="right", fontweight="bold",
                      bbox=dict(boxstyle="round,pad=0.2", fc=BG, ec="none", alpha=0.8))
        ax_t.set_ylim(0, max(tp) * 1.25 or 1)
    else:
        ax_t.set_ylim(0, 1)

    # Status panel
    ax_s = axes["status"]
    ax_s.clear()
    ax_s.set_facecolor(FACE)
    ax_s.axis("off")
    ax_s.set_title("Status", fontsize=13, fontweight="bold")

    bcn = snap["last_beacon"]
    if bcn:
        from _protocol import STATE_NAMES
        state_name = STATE_NAMES.get(bcn.state, f"?{bcn.state}")
        age = time.time() - snap["last_beacon_time"]
        age_color = GREEN if age < 5 else (ORANGE if age < 10 else RED)

        lines = [
            f"State:   {state_name}",
            f"Uptime:  {bcn.uptime_ms / 1000:.0f}s",
            f"Profile: {bcn.sband_profile}   TX: {bcn.tx_power_dbm} dBm",
            f"Flags:   UHF={'OK' if bcn.uhf_ok else 'NO'}  "
            f"SB={'OK' if bcn.sband_ok else 'NO'}  "
            f"IMG={'YES' if bcn.image_loaded else 'NO'}",
            f"TX: {bcn.tx_count}   RX: {bcn.rx_count}",
            f"Beacons: {snap['beacon_count']}   "
            f"Bulk: {snap['bulk_count']}   "
            f"ACKs: {snap['ack_count']}",
            f"Last beacon: {age:.1f}s ago",
        ]
        text = "\n".join(lines)
        ax_s.text(0.05, 0.95, text, transform=ax_s.transAxes,
                  fontsize=10, fontfamily="monospace", color=TEXT,
                  va="top", ha="left", linespacing=1.6)
    else:
        ax_s.text(0.5, 0.5, "Waiting for beacon...",
                  transform=ax_s.transAxes, fontsize=14,
                  color=BLUE, va="center", ha="center",
                  fontweight="bold")

    # Packet stats bar chart
    ax_p = axes["pkt_stats"]
    ax_p.clear()
    ax_p.set_facecolor(FACE)
    ax_p.set_title("Packet Stats", fontsize=13, fontweight="bold")

    total = snap["bulk_total"]
    rxd = snap["bulk_received_count"]
    missing = max(0, total - rxd) if total > 0 else 0

    cats = ["Received", "Missing"]
    vals = [rxd, missing]
    colors = [GREEN, RED]
    bars = ax_p.bar(cats, vals, color=colors, alpha=0.85,
                    edgecolor=SPINE, linewidth=1.0)
    for b, v in zip(bars, vals):
        if v > 0:
            ax_p.text(b.get_x() + b.get_width() / 2,
                      b.get_height() + max(total * 0.01, 0.5),
                      str(v), ha="center", va="bottom",
                      color=TEXT, fontsize=10, fontweight="bold")
    ax_p.set_ylim(0, max(total, rxd, 1) * 1.25)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()


def init_image_preview():
    """Create a separate image preview figure."""
    fig, ax = plt.subplots(1, 1, figsize=(6, 6),
                           facecolor=BG, constrained_layout=True)
    fig.canvas.manager.set_window_title("Image Preview")
    ax.set_facecolor(BG)
    ax.axis("off")
    fig.canvas.draw()
    fig.canvas.flush_events()
    return fig, ax


def update_image_preview(fig, ax, bulk_data: dict, total_pkts: int):
    """Reassemble and render image from received bulk packets."""
    if total_pkts <= 0:
        return

    parts = []
    for i in range(total_pkts):
        if i in bulk_data:
            parts.append(bulk_data[i])
        else:
            parts.append(b'\x00' * BULK_DATA_PER_PKT)
    jpeg = b''.join(parts)

    try:
        img = Image.open(io.BytesIO(jpeg))
        ax.clear()
        ax.imshow(img)
        ax.axis("off")
        pct = len(bulk_data) / total_pkts * 100 if total_pkts > 0 else 0
        ax.set_title(f"Image \u2014 {len(bulk_data)}/{total_pkts} pkts ({pct:.0f}%)",
                     fontsize=11, fontweight="bold")
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
    except Exception:
        pass


def save_image(bulk_data: dict, total_pkts: int, output_dir: str) -> str:
    """Reassemble and save the final image. Returns path or empty string."""
    import os
    from datetime import datetime

    if total_pkts <= 0 or not bulk_data:
        return ""

    parts = []
    for i in range(total_pkts):
        if i in bulk_data:
            parts.append(bulk_data[i])
        else:
            parts.append(b'\x00' * BULK_DATA_PER_PKT)
    jpeg = b''.join(parts)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    pct = len(bulk_data) / total_pkts * 100
    tag = "complete" if len(bulk_data) == total_pkts else f"partial_{pct:.0f}pct"
    path = os.path.join(output_dir, f"{ts}_balloon_{tag}.jpg")

    with open(path, "wb") as f:
        f.write(jpeg)

    return path
