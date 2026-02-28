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

LINE_WIDTH = 1.6
FILL_ALPHA = 0.07
REF_LINE_WIDTH = 0.8
REF_LINE_ALPHA = 0.45
FLOOR_LINE_ALPHA = 0.35
PILL_BBOX = dict(
    boxstyle="round,pad=0.18,rounding_size=0.25",
    fc=BG,
    ec="none",
    alpha=0.72,
)
STATUS_BADGE_BBOX = dict(
    boxstyle="round,pad=0.22,rounding_size=0.35",
    fc=BG,
    ec="none",
    alpha=0.88,
)
CARD_BG = "#FBFCFE"
CARD_EDGE = "#D8DEE8"
CARD_MUTED = "#6B7280"
CARD_SOFT = "#EEF3F8"


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
        "grid.alpha": 0.45,
        "grid.linewidth": 0.4,
        "grid.linestyle": "-",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.edgecolor": SPINE,
        "axes.linewidth": 0.8,
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
        "lines.linewidth": LINE_WIDTH,
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


def _safe_set_window_title(fig, title: str):
    manager = getattr(fig.canvas, "manager", None)
    if manager is not None and hasattr(manager, "set_window_title"):
        manager.set_window_title(title)


def _style_timeseries_axis(ax, title, ylabel):
    ax.clear()
    ax.set_facecolor(FACE)
    ax.set_title(title, fontsize=13)
    ax.set_xlabel("Sample")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.45, linewidth=0.4)
    for spine in ax.spines.values():
        spine.set_color(SPINE)
        spine.set_linewidth(0.8)


def _annotate_metric(ax, left_text, right_text, left_color, right_color):
    if left_text:
        ax.text(
            0.02, 0.95, left_text,
            transform=ax.transAxes,
            fontsize=8.5,
            color=left_color,
            va="top",
            ha="left",
            bbox=PILL_BBOX,
        )
    if right_text:
        ax.text(
            0.98, 0.95, right_text,
            transform=ax.transAxes,
            fontsize=8.5,
            color=right_color,
            va="top",
            ha="right",
            bbox=PILL_BBOX,
        )


def _draw_reference_line(
    ax,
    y,
    color,
    label,
    style="--",
    alpha=REF_LINE_ALPHA,
    text_pos=(0.98, 0.02),
    ha="right",
    va="bottom",
):
    ax.axhline(y=y, color=color, linewidth=REF_LINE_WIDTH, linestyle=style,
               alpha=alpha)
    ax.text(
        text_pos[0], text_pos[1], label,
        transform=ax.transAxes,
        fontsize=8,
        color=color,
        alpha=min(alpha + 0.2, 0.8),
        va=va,
        ha=ha,
    )


def _format_series_stat(data, unit="", precision=1):
    if not data:
        return "N/A"
    fmt = f"{{:.{precision}f}}"
    suffix = f" {unit}" if unit else ""
    return f"{fmt.format(data[-1])} / {fmt.format(sum(data) / len(data))}{suffix}"


def _throughput_trend(data):
    if len(data) < 5:
        return "steady"

    baseline = sum(data[-5:-1]) / 4.0
    current = data[-1]
    if baseline <= 0:
        return "steady"
    if current > baseline * 1.08:
        return "rising"
    if current < baseline * 0.92:
        return "falling"
    return "steady"


def _card_patch(ax, x, y, w, h, facecolor=CARD_BG, edgecolor=CARD_EDGE, linewidth=0.9):
    patch = matplotlib.patches.FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.012,rounding_size=0.02",
        transform=ax.transAxes,
        linewidth=linewidth,
        facecolor=facecolor,
        edgecolor=edgecolor,
    )
    ax.add_patch(patch)
    return patch


def _draw_kpi_card(ax, x, y, w, h, label, value, value_color=TEXT,
                   edgecolor=CARD_EDGE, fill=CARD_BG):
    _card_patch(ax, x, y, w, h, facecolor=fill, edgecolor=edgecolor)
    ax.text(
        x + 0.03, y + h - 0.035, label,
        transform=ax.transAxes,
        fontsize=7.6,
        color=CARD_MUTED,
        va="top",
        ha="left",
    )
    ax.text(
        x + 0.03, y + 0.035, value,
        transform=ax.transAxes,
        fontsize=12,
        color=value_color,
        va="bottom",
        ha="left",
        fontweight="bold",
    )


def _draw_info_card(ax, x, y, w, h, title, rows, dim_alpha=1.0):
    _card_patch(ax, x, y, w, h)
    ax.text(
        x + 0.03, y + h - 0.035, title,
        transform=ax.transAxes,
        fontsize=8,
        color=TEXT,
        va="top",
        ha="left",
        fontweight="bold",
    )
    if not rows:
        return

    top_y = y + h - 0.09
    bottom_y = y + 0.05
    if len(rows) == 1:
        ys = [0.5 * (top_y + bottom_y)]
    else:
        step = (top_y - bottom_y) / (len(rows) - 1)
        ys = [top_y - step * i for i in range(len(rows))]

    for row_y, row in zip(ys, rows):
        label = row.get("label", "")
        value = row.get("value", "")
        value_color = row.get("value_color", TEXT)
        ax.text(
            x + 0.03, row_y, label,
            transform=ax.transAxes,
            fontsize=7.6,
            color=CARD_MUTED,
            va="center",
            ha="left",
            alpha=dim_alpha,
        )
        ax.text(
            x + w * 0.53, row_y, value,
            transform=ax.transAxes,
            fontsize=8.6,
            color=value_color,
            va="center",
            ha="left",
            alpha=dim_alpha,
        )


def _draw_footer_card(ax, x, y, w, h, title, lines, dim_alpha=1.0):
    _card_patch(ax, x, y, w, h, facecolor=CARD_SOFT)
    ax.text(
        x + 0.03, y + h - 0.035, title,
        transform=ax.transAxes,
        fontsize=8,
        color=TEXT,
        va="top",
        ha="left",
        fontweight="bold",
    )
    if not lines:
        return

    top_y = y + h - 0.09
    bottom_y = y + 0.05
    if len(lines) == 1:
        ys = [0.5 * (top_y + bottom_y)]
    else:
        step = (top_y - bottom_y) / (len(lines) - 1)
        ys = [top_y - step * i for i in range(len(lines))]

    for row_y, line in zip(ys, lines):
        ax.text(
            x + 0.03, row_y, line,
            transform=ax.transAxes,
            fontsize=8,
            color=TEXT,
            va="center",
            ha="left",
            alpha=dim_alpha,
        )


def _derive_dashboard_metrics(snap, sband_profile):
    total = snap["bulk_total"]
    received = snap["bulk_received_count"]
    missing = max(0, total - received) if total > 0 else 0
    packet_loss_pct = (missing / total * 100.0) if total > 0 else None
    transfer_progress_pct = (received / total * 100.0) if total > 0 else None

    beacon = snap["last_beacon"]
    beacon_age_s = None
    freshness = "WAITING"
    freshness_color = BLUE
    link_health = "NO DATA"
    link_color = BLUE
    state_name = "WAITING"

    if beacon:
        from _protocol import STATE_NAMES

        state_name = STATE_NAMES.get(beacon.state, f"?{beacon.state}")
        beacon_age_s = max(0.0, time.time() - snap["last_beacon_time"])
        if beacon_age_s < 5:
            freshness = "LIVE"
            freshness_color = GREEN
        elif beacon_age_s < 10:
            freshness = "IDLE"
            freshness_color = ORANGE
        else:
            freshness = "STALE"
            freshness_color = RED

        if freshness == "STALE":
            link_health = "STALE"
            link_color = RED
        elif beacon.uhf_ok and beacon.sband_ok:
            link_health = "HEALTHY"
            link_color = GREEN
        elif beacon.uhf_ok or beacon.sband_ok:
            link_health = "DEGRADED"
            link_color = ORANGE
        else:
            link_health = "DOWN"
            link_color = RED

    throughput = snap["throughput"]
    throughput_now = throughput[-1] if throughput else None
    throughput_avg = (sum(throughput) / len(throughput)) if throughput else None
    last_command_rtt_ms = None
    for command in reversed(snap["commands"]):
        if getattr(command, "acked", False) and command.rtt_ms >= 0:
            last_command_rtt_ms = command.rtt_ms
            break

    return {
        "missing_count": missing,
        "packet_loss_pct": packet_loss_pct,
        "transfer_progress_pct": transfer_progress_pct,
        "beacon_age_s": beacon_age_s,
        "freshness": freshness,
        "freshness_color": freshness_color,
        "link_health": link_health,
        "link_color": link_color,
        "state_name": state_name,
        "throughput_now_kbps": throughput_now,
        "throughput_avg_kbps": throughput_avg,
        "throughput_trend": _throughput_trend(throughput),
        "last_command_rtt_ms": last_command_rtt_ms,
        "sband_floor": SBAND_FLOORS.get(sband_profile, -120),
        "uhf_rssi_summary": _format_series_stat(snap["uhf_rssi"], "dBm"),
        "uhf_snr_summary": _format_series_stat(snap["uhf_snr"], "dB"),
        "sband_rssi_summary": _format_series_stat(snap["sband_rssi"], "dBm"),
    }


def _plot_rssi(ax, data, color, title, floor):
    _style_timeseries_axis(ax, title, "dBm")

    if data:
        x = list(range(len(data)))
        ax.plot(x, data, color=color, linewidth=LINE_WIDTH, alpha=0.92)
        ax.fill_between(x, data, color=color, alpha=FILL_ALPHA)

        avg = sum(data) / len(data)
        cur = data[-1]
        mc = _margin_color(cur, floor)
        margin = cur - floor
        _draw_reference_line(ax, avg, color, f"avg {avg:.1f}", style="--")
        _annotate_metric(
            ax,
            f"avg {avg:.1f}",
            f"now {cur:.1f}  margin {margin:.0f} dB",
            color,
            mc,
        )

        lo, hi = _stable_ylim(data, min_range=10.0)
        ax.set_ylim(min(lo, floor - 5), hi)
    else:
        ax.set_ylim(floor - 10, 0)

    _draw_reference_line(
        ax, floor, RED, f"floor {floor}",
        style=":",
        alpha=FLOOR_LINE_ALPHA,
    )


def _plot_snr(ax, data, color, title):
    _style_timeseries_axis(ax, title, "dB")

    if data:
        x = list(range(len(data)))
        ax.plot(x, data, color=color, linewidth=LINE_WIDTH, alpha=0.92)
        ax.fill_between(x, data, color=color, alpha=FILL_ALPHA)

        avg = sum(data) / len(data)
        _draw_reference_line(ax, avg, color, f"avg {avg:.1f}", style="--")
        _annotate_metric(
            ax,
            f"avg {avg:.1f}",
            f"now {data[-1]:.1f}",
            color,
            color,
        )

        lo, hi = _stable_ylim(data, min_range=3.0)
        ax.set_ylim(lo, hi)
    else:
        ax.set_ylim(-5, 15)


def init_dashboard():
    """Create the main 2x3 dashboard figure."""
    apply_dark_theme()

    fig, axes = plt.subplots(2, 3, figsize=(16, 9),
                             facecolor=BG, constrained_layout=True)
    _safe_set_window_title(fig, "Balloon Link Budget Dashboard")

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
    metrics = _derive_dashboard_metrics(snap, sband_profile)

    _plot_rssi(axes["uhf_rssi"], snap["uhf_rssi"], GREEN,
               "UHF RSSI", UHF_FLOOR)

    _plot_rssi(axes["sb_rssi"], snap["sband_rssi"], PURPLE,
               "S-Band RSSI", metrics["sband_floor"])

    _plot_snr(axes["uhf_snr"], snap["uhf_snr"], YELLOW, "UHF SNR")

    # Throughput
    ax_t = axes["throughput"]
    _style_timeseries_axis(ax_t, "Throughput (KB/s)", "KB/s")
    tp = snap["throughput"]
    if tp:
        x = list(range(len(tp)))
        avg = metrics["throughput_avg_kbps"]
        now = metrics["throughput_now_kbps"]
        ax_t.plot(x, tp, color=ORANGE, linewidth=LINE_WIDTH, alpha=0.92)
        ax_t.fill_between(x, tp, color=ORANGE, alpha=FILL_ALPHA)
        _draw_reference_line(ax_t, avg, ORANGE, f"avg {avg:.1f}", style="--")
        _annotate_metric(
            ax_t,
            f"avg {avg:.1f} KB/s",
            f"now {now:.1f} • {metrics['throughput_trend']}",
            ORANGE,
            ORANGE,
        )
        ax_t.set_ylim(0, max(tp) * 1.25 or 1)
    else:
        ax_t.set_ylim(0, 1)
        _annotate_metric(ax_t, "avg N/A", "now N/A", ORANGE, ORANGE)

    # Status panel
    ax_s = axes["status"]
    ax_s.clear()
    ax_s.set_facecolor(FACE)
    ax_s.axis("off")
    ax_s.set_title("Status", fontsize=12, pad=6)

    bcn = snap["last_beacon"]
    stale_alpha = 0.68 if metrics["freshness"] == "STALE" else 1.0
    age_text = (
        f"{metrics['beacon_age_s']:.1f}s ago"
        if metrics["beacon_age_s"] is not None else "Waiting"
    )
    if bcn:
        progress = (
            f"{metrics['transfer_progress_pct']:.0f}%"
            if metrics["transfer_progress_pct"] is not None else "N/A"
        )
        loss = (
            f"{metrics['packet_loss_pct']:.1f}%"
            if metrics["packet_loss_pct"] is not None else "N/A"
        )
        rate_now = (
            f"{metrics['throughput_now_kbps']:.1f} KB/s"
            if metrics["throughput_now_kbps"] is not None else "N/A"
        )
        link_value = (
            f"{'OK' if bcn.uhf_ok else 'NO'} / {'OK' if bcn.sband_ok else 'NO'}"
        )
        status_fill = "#F8FBF8" if metrics["freshness"] != "STALE" else "#FFF8F8"
        status_edge = GREEN if metrics["link_health"] == "HEALTHY" else CARD_EDGE
        freshness_edge = metrics["freshness_color"]

        _draw_kpi_card(
            ax_s, 0.04, 0.76, 0.28, 0.12,
            "State", metrics["state_name"],
            value_color=TEXT,
            edgecolor=CARD_EDGE,
            fill=CARD_BG,
        )
        _draw_kpi_card(
            ax_s, 0.36, 0.76, 0.28, 0.12,
            "Link", metrics["link_health"],
            value_color=metrics["link_color"],
            edgecolor=status_edge,
            fill=status_fill,
        )
        _draw_kpi_card(
            ax_s, 0.68, 0.76, 0.28, 0.12,
            "Freshness", metrics["freshness"],
            value_color=metrics["freshness_color"],
            edgecolor=freshness_edge,
            fill="#FFF8F8" if metrics["freshness"] == "STALE" else CARD_BG,
        )

        radio_rows = [
            {"label": "Profile / TX", "value": f"P{bcn.sband_profile}  •  {bcn.tx_power_dbm} dBm"},
            {"label": "UHF / S-Band", "value": link_value,
             "value_color": metrics["link_color"] if bcn.uhf_ok and bcn.sband_ok else ORANGE},
            {"label": "Image", "value": "Loaded" if bcn.image_loaded else "Not loaded",
             "value_color": GREEN if bcn.image_loaded else ORANGE},
            {"label": "Last beacon", "value": age_text, "value_color": metrics["freshness_color"]},
        ]
        transfer_rows = [
            {"label": "Packets", "value": (
                f"{snap['bulk_received_count']} / {snap['bulk_total']}"
                if snap["bulk_total"] > 0 else "0 / -"
            )},
            {"label": "Progress", "value": progress},
            {"label": "Loss", "value": loss,
             "value_color": RED if metrics["packet_loss_pct"] else TEXT},
            {"label": "Rate now", "value": rate_now},
        ]
        _draw_info_card(ax_s, 0.04, 0.43, 0.44, 0.27, "Radio", radio_rows, stale_alpha)
        _draw_info_card(ax_s, 0.52, 0.43, 0.44, 0.27, "Transfer", transfer_rows, stale_alpha)

        footer_lines = [
            f"{snap['beacon_count']} beacons   {snap['bulk_count']} bulk   {snap['ack_count']} ACKs",
            (
                f"UHF {snap['uhf_rssi'][-1]:.1f} dBm  •  {snap['uhf_snr'][-1]:.1f} dB"
                if snap["uhf_rssi"] and snap["uhf_snr"] else "UHF link metrics unavailable"
            ),
        ]
        if snap["sband_rssi"]:
            footer_lines.append(f"S-Band {snap['sband_rssi'][-1]:.1f} dBm")
        if metrics["last_command_rtt_ms"] is not None:
            footer_lines.append(f"Last command RTT {metrics['last_command_rtt_ms']:.0f} ms")
        else:
            footer_lines.append(f"Device uptime {bcn.uptime_ms / 1000:.0f}s")
        _draw_footer_card(ax_s, 0.04, 0.12, 0.92, 0.24, "Recent", footer_lines, stale_alpha)
    else:
        _draw_kpi_card(ax_s, 0.04, 0.76, 0.28, 0.12, "State", "WAITING", BLUE)
        _draw_kpi_card(ax_s, 0.36, 0.76, 0.28, 0.12, "Link", "NO DATA", CARD_MUTED)
        _draw_kpi_card(ax_s, 0.68, 0.76, 0.28, 0.12, "Freshness", "WAITING", BLUE)
        _draw_info_card(
            ax_s, 0.04, 0.43, 0.44, 0.27, "Radio",
            [
                {"label": "Profile / TX", "value": "Waiting"},
                {"label": "UHF / S-Band", "value": "No data"},
                {"label": "Image", "value": "Unknown"},
                {"label": "Last beacon", "value": "Waiting", "value_color": BLUE},
            ],
        )
        _draw_info_card(
            ax_s, 0.52, 0.43, 0.44, 0.27, "Transfer",
            [
                {"label": "Packets", "value": "0 / -"},
                {"label": "Progress", "value": "N/A"},
                {"label": "Loss", "value": "N/A"},
                {"label": "Rate now", "value": "N/A"},
            ],
        )
        _draw_footer_card(
            ax_s, 0.04, 0.12, 0.92, 0.24, "Recent",
            [
                "Waiting for first beacon",
                "Packet loss is shown during active transfers",
                "Charts keep their last visible samples",
            ],
        )

    # Packet stats bar chart
    ax_p = axes["pkt_stats"]
    ax_p.clear()
    ax_p.set_facecolor(FACE)
    ax_p.set_title("Packet Stats", fontsize=13)
    ax_p.grid(True, axis="y", alpha=0.45, linewidth=0.4)
    ax_p.grid(False, axis="x")
    ax_p.set_ylabel("Packets")
    for spine in ax_p.spines.values():
        spine.set_color(SPINE)
        spine.set_linewidth(0.8)

    total = snap["bulk_total"]
    rxd = snap["bulk_received_count"]
    missing = metrics["missing_count"]

    cats = ["Received", "Missing"]
    vals = [rxd, missing]
    colors = [GREEN, RED]
    bars = ax_p.bar(cats, vals, color=colors, alpha=0.85,
                    edgecolor=SPINE, linewidth=0.8, width=0.55)
    for b, v in zip(bars, vals):
        if v > 0:
            ax_p.text(b.get_x() + b.get_width() / 2,
                      b.get_height() + max(total * 0.015, 0.6),
                      str(v), ha="center", va="bottom",
                      color=TEXT, fontsize=10, fontweight="bold")
    if total > 0 and metrics["packet_loss_pct"] is not None:
        summary = (
            f"Loss {metrics['packet_loss_pct']:.1f}% \u2022 "
            f"Progress {metrics['transfer_progress_pct']:.0f}%"
        )
    else:
        summary = "Waiting for transfer"
    ax_p.text(0.5, 0.97, summary, transform=ax_p.transAxes,
              fontsize=8.5, color=TEXT, ha="center", va="top", alpha=0.8)
    ax_p.set_ylim(0, max(total, rxd, 4) * 1.18)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()


def init_image_preview():
    """Create a separate image preview figure."""
    fig, ax = plt.subplots(1, 1, figsize=(6, 6),
                           facecolor=BG, constrained_layout=True)
    _safe_set_window_title(fig, "Image Preview")
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
