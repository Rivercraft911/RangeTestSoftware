"""Live dashboard for balloon link-budget testing."""

import io
import textwrap
import time

import matplotlib
import matplotlib.pyplot as plt
from PIL import Image, ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

SURFACE = "#0C1420"
INK = "#D7E1EC"
GRID = "#223242"
SPINE = "#31475C"
MUTED = "#8FA0B4"
TRACK = "#152232"
PANEL_STRIP = "#101B2A"
MONO_STACK = [
    "JetBrains Mono",
    "SF Mono",
    "Menlo",
    "Monaco",
    "Consolas",
    "Liberation Mono",
    "DejaVu Sans Mono",
    "monospace",
]

APPLE_GREEN = "#65B27E"
APPLE_YELLOW = "#D0B067"
APPLE_ORANGE = "#C98B4F"
APPLE_RED = "#D46A76"
APPLE_VIOLET = "#8B84D1"
APPLE_BLUE = "#78A6D2"
APPLE_TEAL = "#69AEBE"

UHF_FLOOR = -130
SBAND_FLOORS = {1: -117, 2: -120, 3: -126, 4: -129}

BULK_DATA_PER_PKT = 243  # BALLOON_BULK_DATA_MAX

LINE_WIDTH = 1.05
FILL_ALPHA = 0.018
REF_LINE_WIDTH = 0.5
REF_LINE_ALPHA = 0.32
FLOOR_LINE_ALPHA = 0.22
PILL_BBOX = dict(
    boxstyle="square,pad=0.12",
    fc=PANEL_STRIP,
    ec=SPINE,
    alpha=1.0,
)


def apply_dark_theme():
    """Apply a restrained mission-console theme globally."""
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "text.color": INK,
        "axes.labelcolor": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.alpha": 0.34,
        "grid.linewidth": 0.28,
        "grid.linestyle": "--",
        "axes.spines.top": True,
        "axes.spines.right": True,
        "axes.edgecolor": SPINE,
        "axes.linewidth": 0.8,
        "font.family": "monospace",
        "font.monospace": MONO_STACK,
        "font.size": 7.8,
        "axes.titlesize": 9.2,
        "axes.titleweight": "normal",
        "axes.labelsize": 7.1,
        "xtick.labelsize": 6.2,
        "ytick.labelsize": 6.2,
        "lines.linewidth": LINE_WIDTH,
        "lines.antialiased": True,
        "legend.frameon": True,
        "legend.framealpha": 0.95,
        "legend.facecolor": PANEL_STRIP,
        "legend.edgecolor": SPINE,
        "legend.fontsize": 6.2,
        "axes.titlepad": 4,
        "axes.labelpad": 3.5,
        "figure.dpi": 150,
        "savefig.dpi": 200,
        "savefig.facecolor": SURFACE,
        "axes.unicode_minus": False,
    })


def _margin_color(rssi: float, floor: float) -> str:
    margin = rssi - floor
    if margin > 10:
        return APPLE_GREEN
    if margin > 3:
        return APPLE_ORANGE
    return APPLE_RED


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


def _draw_empty_state(ax, title, message):
    ax.clear()
    ax.set_facecolor(SURFACE)
    ax.set_title(title, fontsize=9.2, pad=3)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    for spine in ax.spines.values():
        spine.set_color(SPINE)
        spine.set_linewidth(0.8)
    ax.text(
        0.5, 0.5, message,
        transform=ax.transAxes,
        fontsize=7.4,
        color=MUTED,
        ha="center",
        va="center",
    )


def _style_timeseries_axis(ax, title, ylabel):
    ax.clear()
    ax.set_axis_on()
    ax.set_facecolor(SURFACE)
    ax.set_title(title, fontsize=9.2, pad=3)
    ax.set_xlabel("Sample")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.34, linewidth=0.28, linestyle="--")
    ax.margins(x=0.025)
    ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(4))
    ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(4))
    ax.tick_params(length=2.5, width=0.6, colors=INK, pad=2)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color(SPINE)
        spine.set_linewidth(0.7)


def _annotate_metric(ax, left_text, right_text, left_color, right_color):
    if left_text:
        ax.text(
            0.02, 0.95, left_text,
            transform=ax.transAxes,
            fontsize=6.0,
            color=left_color,
            va="top",
            ha="left",
            bbox=PILL_BBOX,
            clip_on=True,
            fontfamily="monospace",
        )
    if right_text:
        ax.text(
            0.98, 0.95, right_text,
            transform=ax.transAxes,
            fontsize=6.0,
            color=right_color,
            va="top",
            ha="right",
            bbox=PILL_BBOX,
            clip_on=True,
            fontfamily="monospace",
        )


def _draw_reference_line(
    ax,
    y,
    color,
    style="--",
    alpha=REF_LINE_ALPHA,
    label=None,
    text_pos=(0.98, 0.02),
    ha="right",
    va="bottom",
):
    ax.axhline(y=y, color=color, linewidth=REF_LINE_WIDTH, linestyle=style, alpha=alpha)
    if label:
        ax.text(
            text_pos[0], text_pos[1], label,
            transform=ax.transAxes,
            fontsize=6.0,
            color=color,
            alpha=min(alpha + 0.2, 0.8),
            va=va,
            ha=ha,
        )


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


def _derive_dashboard_metrics(snap, sband_profile):
    total = snap["bulk_total"]
    received = snap["bulk_received_count"]
    missing = max(0, total - received) if total > 0 else 0
    packet_loss_pct = (missing / total * 100.0) if total > 0 else None
    transfer_progress_pct = (received / total * 100.0) if total > 0 else None

    beacon = snap["last_beacon"]
    beacon_age_s = None
    freshness = "WAITING"
    freshness_color = APPLE_BLUE
    link_health = "NO DATA"
    link_color = APPLE_BLUE
    state_name = "WAITING"

    if beacon:
        from _protocol import STATE_NAMES

        state_name = STATE_NAMES.get(beacon.state, f"?{beacon.state}")
        beacon_age_s = max(0.0, time.time() - snap["last_beacon_time"])
        if beacon_age_s < 5:
            freshness = "LIVE"
            freshness_color = APPLE_GREEN
        elif beacon_age_s < 10:
            freshness = "IDLE"
            freshness_color = APPLE_ORANGE
        else:
            freshness = "STALE"
            freshness_color = APPLE_RED

        if freshness == "STALE":
            link_health = "STALE"
            link_color = APPLE_RED
        elif beacon.uhf_ok and beacon.sband_ok:
            link_health = "HEALTHY"
            link_color = APPLE_GREEN
        elif beacon.uhf_ok or beacon.sband_ok:
            link_health = "DEGRADED"
            link_color = APPLE_ORANGE
        else:
            link_health = "DOWN"
            link_color = APPLE_RED

    throughput = snap["throughput"]
    throughput_now = throughput[-1] if throughput else None
    throughput_avg = (sum(throughput) / len(throughput)) if throughput else None

    uhf_margin_series = [value - UHF_FLOOR for value in snap["uhf_rssi"]]
    sband_floor = SBAND_FLOORS.get(sband_profile, -120)
    sband_margin_series = [value - sband_floor for value in snap["sband_rssi"]]

    last_command_rtt_ms = None
    for command in reversed(snap["commands"]):
        if getattr(command, "acked", False) and command.rtt_ms >= 0:
            last_command_rtt_ms = command.rtt_ms
            break

    command_ok = sum(1 for command in snap["commands"] if command.acked)
    command_fail = sum(
        1 for command in snap["commands"]
        if command.failed and not command.acked
    )
    command_pending = sum(
        1 for command in snap["commands"]
        if not command.acked and not command.failed
    )
    completed = command_ok + command_fail
    command_success_pct = (
        (command_ok / completed) * 100.0 if completed > 0 else None
    )

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
        "command_ok": command_ok,
        "command_fail": command_fail,
        "command_pending": command_pending,
        "command_success_pct": command_success_pct,
        "sband_floor": sband_floor,
        "uhf_margin_now_db": uhf_margin_series[-1] if uhf_margin_series else None,
        "sband_margin_now_db": sband_margin_series[-1] if sband_margin_series else None,
        "uhf_margin_series": uhf_margin_series,
        "sband_margin_series": sband_margin_series,
    }


def _plot_rssi(ax, data, color, title, floor):
    if not data:
        _draw_empty_state(ax, title, "Waiting for samples")
        return

    current = data[-1]
    margin = current - floor
    _style_timeseries_axis(ax, f"{title} - {margin:.0f} dB margin", "dBm")

    x = list(range(len(data)))
    avg = sum(data) / len(data)
    lo, hi = _stable_ylim(data, pad_frac=0.22, min_range=10.0)
    strongest = max(data)
    weakest = min(data)
    upper = min(-1.0, max(hi, strongest + 8.0))
    lower = min(lo, weakest - 6.0)
    ax.set_ylim(lower, upper)
    ax.plot(x, data, color=color, linewidth=LINE_WIDTH, alpha=0.95)
    ax.fill_between(
        x,
        data,
        [lower] * len(x),
        color=color,
        alpha=max(FILL_ALPHA, 0.09),
    )
    _draw_reference_line(ax, avg, color, style="--")
    _draw_reference_line(ax, floor, APPLE_RED, style=":", alpha=FLOOR_LINE_ALPHA)
    _annotate_metric(
        ax,
        f"avg {avg:.1f}",
        f"now {current:.1f}",
        color,
        _margin_color(current, floor),
    )


def _plot_snr(ax, data, color, title, empty_message="Waiting for samples"):
    if not data:
        _draw_empty_state(ax, title, empty_message)
        return

    _style_timeseries_axis(ax, title, "dB")
    x = list(range(len(data)))
    avg = sum(data) / len(data)
    ax.plot(x, data, color=color, linewidth=LINE_WIDTH, alpha=0.95)
    ax.fill_between(x, data, color=color, alpha=FILL_ALPHA)
    _draw_reference_line(ax, avg, color, style="--")
    _annotate_metric(
        ax,
        f"avg {avg:.1f}",
        f"now {data[-1]:.1f}",
        color,
        color,
    )

    lo, hi = _stable_ylim(data, pad_frac=0.20, min_range=3.0)
    ax.set_ylim(lo, hi)


def _plot_throughput(ax, data, metrics):
    if not data:
        _draw_empty_state(ax, "Throughput", "Waiting for samples")
        return

    _style_timeseries_axis(
        ax,
        f"Throughput - {metrics['throughput_trend']}",
        "KB/s",
    )
    x = list(range(len(data)))
    avg = metrics["throughput_avg_kbps"]
    now = metrics["throughput_now_kbps"]
    ax.plot(x, data, color=APPLE_ORANGE, linewidth=LINE_WIDTH, alpha=0.95)
    ax.fill_between(x, data, color=APPLE_ORANGE, alpha=FILL_ALPHA)
    _draw_reference_line(ax, avg, APPLE_ORANGE, style="--")
    _annotate_metric(
        ax,
        f"avg {avg:.1f} KB/s",
        f"now {now:.1f}",
        APPLE_ORANGE,
        APPLE_ORANGE,
    )
    ax.set_ylim(0, max(data) * 1.35 or 1)


def _plot_beacon_age(ax, metrics):
    ax.clear()
    ax.set_facecolor(SURFACE)
    ax.set_title("Beacon Age (s)", fontsize=9.2, pad=3)
    ax.grid(True, axis="y", alpha=0.34, linewidth=0.28, linestyle="--")
    ax.grid(False, axis="x")
    ax.set_ylabel("seconds")
    ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(4))
    ax.tick_params(length=2.5, width=0.6, colors=INK, pad=2)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color(SPINE)
        spine.set_linewidth(0.7)

    age = metrics["beacon_age_s"]
    ax.set_xticks([0])
    ax.set_xticklabels(["latest"])
    ax.set_xlim(-0.45, 0.45)
    if age is None:
        ax.set_ylim(0, 12)
        ax.bar([0], [0], color=TRACK, width=0.34, edgecolor=SPINE,
               linewidth=0.7, alpha=0.9)
        ax.text(
            0.5, 0.10, "no beacon",
            transform=ax.transAxes,
            ha="center", va="bottom",
            color=MUTED, fontsize=6.4,
            fontfamily="monospace",
            clip_on=True,
        )
        return

    age_color = (
        APPLE_GREEN if age < 5
        else (APPLE_ORANGE if age < 10 else APPLE_RED)
    )
    ax.axhline(5, color=APPLE_GREEN, linewidth=REF_LINE_WIDTH,
               linestyle="--", alpha=0.28)
    ax.axhline(10, color=APPLE_ORANGE, linewidth=REF_LINE_WIDTH,
               linestyle="--", alpha=0.28)
    ax.bar([0], [age], color=age_color, width=0.34,
           edgecolor=SPINE, linewidth=0.7, alpha=0.9, zorder=2)
    ax.set_ylim(0, max(12, age * 1.30))
    ax.text(
        0, age + max(0.28, age * 0.035), f"{age:.1f}s",
        ha="center", va="bottom",
        color=INK, fontsize=6.6,
        fontfamily="monospace",
        clip_on=True,
    )
    ax.text(
        0.98, 0.08, metrics["freshness"],
        transform=ax.transAxes,
        ha="right", va="bottom",
        color=age_color, fontsize=5.6,
        fontfamily="monospace",
        clip_on=True,
    )


def _plot_command_success(ax, metrics):
    ax.clear()
    ax.set_facecolor(SURFACE)
    ax.set_title("Command Success", fontsize=9.2, pad=3)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid(False)
    for spine in ax.spines.values():
        spine.set_color(SPINE)
        spine.set_linewidth(0.7)

    raw_labels = ["OK", "FAIL", "PEND"]
    raw_vals = [
        metrics["command_ok"],
        metrics["command_fail"],
        metrics["command_pending"],
    ]
    raw_colors = [APPLE_GREEN, APPLE_RED, APPLE_BLUE]
    total = sum(raw_vals)

    if total <= 0:
        ax.pie(
            [1],
            colors=[TRACK],
            startangle=90,
            counterclock=False,
            radius=0.76,
            wedgeprops=dict(width=0.28, edgecolor=SURFACE, linewidth=1.0),
        )
        ax.text(
            0.5, 0.56, "NONE",
            transform=ax.transAxes,
            ha="center", va="center",
            color=INK, fontsize=8.4, fontfamily="monospace",
        )
        ax.text(
            0.5, 0.43, "no cmds",
            transform=ax.transAxes,
            ha="center", va="center",
            color=MUTED, fontsize=5.6, fontfamily="monospace",
        )
    else:
        vals = []
        colors = []
        for value, color in zip(raw_vals, raw_colors):
            if value > 0:
                vals.append(value)
                colors.append(color)
        ax.pie(
            vals,
            colors=colors,
            startangle=90,
            counterclock=False,
            radius=0.76,
            wedgeprops=dict(width=0.28, edgecolor=SURFACE, linewidth=1.0),
        )
        success_pct = metrics["command_success_pct"]
        center_text = (
            f"{success_pct:.0f}%"
            if success_pct is not None
            else "N/A"
        )
        ax.text(
            0.5, 0.56, center_text,
            transform=ax.transAxes,
            ha="center", va="center",
            color=INK, fontsize=8.6, fontfamily="monospace",
        )
        ax.text(
            0.5, 0.43, "success",
            transform=ax.transAxes,
            ha="center", va="center",
            color=MUTED, fontsize=5.6, fontfamily="monospace",
        )

    stat_x = [0.18, 0.50, 0.82]
    for x, label, value, color in zip(stat_x, raw_labels, raw_vals, raw_colors):
        ax.text(
            x, 0.02, f"{label} {value}",
            transform=ax.transAxes,
            ha="center", va="bottom",
            color=color, fontsize=5.2,
            fontfamily="monospace",
            clip_on=True,
        )


def _plot_link_margin(ax, metrics):
    uhf = metrics["uhf_margin_series"]
    sband = metrics["sband_margin_series"]
    if not uhf and not sband:
        _draw_empty_state(ax, "Link Margin", "No margin data")
        return

    _style_timeseries_axis(ax, "Link Margin", "dB")
    combined = []
    if uhf:
        x = list(range(len(uhf)))
        ax.plot(x, uhf, color=APPLE_GREEN, linewidth=LINE_WIDTH, label="UHF")
        combined.extend(uhf)
    if sband:
        x = list(range(len(sband)))
        ax.plot(x, sband, color=APPLE_VIOLET, linewidth=LINE_WIDTH, label="S-Band")
        combined.extend(sband)
    if uhf and sband:
        ax.legend(loc="upper right")

    lo, hi = _stable_ylim(combined, pad_frac=0.20, min_range=8.0)
    ax.set_ylim(lo, hi)


def _plot_packet_coverage(ax, snap):
    ax.clear()
    ax.set_facecolor(SURFACE)
    ax.set_title("Packet Coverage", fontsize=9.2, pad=3)
    ax.grid(True, axis="y", alpha=0.34, linewidth=0.28, linestyle="--")
    ax.grid(False, axis="x")
    ax.set_ylabel("%")
    ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(4))
    ax.tick_params(length=2.5, width=0.6, colors=INK, pad=2)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color(SPINE)
        spine.set_linewidth(0.7)

    total = snap["bulk_total"]
    keys = snap["bulk_received_keys"]
    if total <= 0:
        ax.set_xticks([1])
        ax.set_xticklabels(["1"])
        ax.set_xlim(0.5, 1.5)
        ax.set_ylim(0, 100)
        ax.vlines(1, 0, 0.6, color=SPINE, linewidth=1.1, alpha=0.55)
        ax.scatter([1], [0.6], s=20, color=TRACK, edgecolors=SPINE,
                   linewidths=0.6, zorder=3)
        ax.text(
            0.5, 0.08, "no transfer",
            transform=ax.transAxes,
            fontsize=5.8,
            color=MUTED,
            ha="center",
            va="bottom",
            fontfamily="monospace",
            clip_on=True,
        )
        return

    bins = max(1, min(8, total))
    xs = []
    values = []
    colors = []
    for idx in range(bins):
        start = (total * idx) // bins
        end = (total * (idx + 1)) // bins
        span = max(1, end - start)
        hit_count = sum(1 for packet_idx in keys if start <= packet_idx < end)
        pct = (hit_count / span) * 100.0
        values.append(pct)
        xs.append(idx + 1)
        if pct >= 99.9:
            colors.append(APPLE_GREEN)
        elif pct > 0:
            colors.append(APPLE_ORANGE)
        else:
            colors.append(APPLE_RED)

    ax.axhline(100, color=MUTED, linewidth=0.45, linestyle=":", alpha=0.32)
    ax.set_xticks(xs)
    ax.set_xlim(0.5, bins + 0.5)
    ax.set_ylim(0, 104)
    ax.vlines(xs, 0, values, colors=colors, linewidth=1.55, alpha=0.9, zorder=2)
    ax.scatter(xs, values, s=28, c=colors, edgecolors=SPINE,
               linewidths=0.55, zorder=3)

    ax.text(
        0.98, 0.13, "chunk % received",
        transform=ax.transAxes,
        fontsize=5.4,
        color=MUTED,
        ha="right",
        va="bottom",
        fontfamily="monospace",
        clip_on=True,
    )
    ax.text(
        0.98, 0.05, f"{len(keys)} / {total} packets",
        transform=ax.transAxes,
        fontsize=5.4,
        color=MUTED,
        ha="right",
        va="bottom",
        fontfamily="monospace",
        clip_on=True,
    )


def init_dashboard():
    """Create the main 3x3 dashboard figure."""
    apply_dark_theme()

    fig = plt.figure(figsize=(15.2, 8.4), facecolor=SURFACE, layout="constrained")
    engine = fig.get_layout_engine()
    if engine is not None and hasattr(engine, "set"):
        engine.set(w_pad=1.2 / 72, h_pad=1.2 / 72, wspace=0.012, hspace=0.02)
    axes = fig.subplot_mosaic([
        ["uhf_rssi", "sb_rssi", "pkt_loss"],
        ["uhf_snr", "sb_snr", "progress"],
        ["throughput", "link_margin", "pkt_stats"],
    ])
    _safe_set_window_title(fig, "Balloon Link Budget Dashboard")

    for ax in axes.values():
        ax.set_facecolor(SURFACE)

    fig.canvas.draw()
    fig.canvas.flush_events()
    return fig, axes


def update_dashboard(fig, axes, snap, sband_profile=1):
    """Refresh all dashboard plots from a state snapshot."""
    metrics = _derive_dashboard_metrics(snap, sband_profile)

    _plot_rssi(axes["uhf_rssi"], snap["uhf_rssi"], APPLE_GREEN, "UHF RSSI", UHF_FLOOR)
    _plot_rssi(
        axes["sb_rssi"],
        snap["sband_rssi"],
        APPLE_VIOLET,
        "S-Band RSSI",
        metrics["sband_floor"],
    )
    _plot_beacon_age(axes["pkt_loss"], metrics)
    _plot_snr(axes["uhf_snr"], snap["uhf_snr"], APPLE_YELLOW, "UHF SNR")
    _plot_snr(
        axes["sb_snr"],
        snap["sband_snr"],
        APPLE_BLUE,
        "S-Band SNR",
        empty_message="No S-Band bulk yet",
    )
    _plot_command_success(axes["progress"], metrics)
    _plot_throughput(axes["throughput"], snap["throughput"], metrics)
    _plot_link_margin(axes["link_margin"], metrics)
    _plot_packet_coverage(axes["pkt_stats"], snap)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()


def init_image_preview():
    """Create a separate image preview figure."""
    fig, ax = plt.subplots(1, 1, figsize=(6, 6),
                           facecolor=SURFACE, constrained_layout=True)
    _safe_set_window_title(fig, "Image Preview")
    ax.set_facecolor(SURFACE)
    ax.axis("off")
    fig.canvas.draw()
    fig.canvas.flush_events()
    return fig, ax


def init_bulk_preview():
    """Create a separate text preview figure for bulk streaming."""
    fig, ax = plt.subplots(1, 1, figsize=(8, 6),
                           facecolor=SURFACE, constrained_layout=True)
    _safe_set_window_title(fig, "Bulk Text Preview")
    ax.set_facecolor(SURFACE)
    ax.axis("off")
    fig.canvas.draw()
    fig.canvas.flush_events()
    return fig, ax


def update_image_preview(fig, ax, bulk_data: dict, total_pkts: int):
    """Reassemble and render image from received bulk packets."""
    if total_pkts <= 0:
        return

    received_count = len(bulk_data)
    pct = received_count / total_pkts * 100 if total_pkts > 0 else 0
    missing_count = max(0, total_pkts - received_count)

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
        ax.set_facecolor(SURFACE)
        ax.imshow(img)
        ax.axis("off")
        ax.set_title(f"Image \u2014 {received_count}/{total_pkts} pkts ({pct:.0f}%)",
                     fontsize=10.5, color=INK)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
    except Exception:
        ax.clear()
        ax.set_facecolor(SURFACE)
        ax.axis("off")
        ax.set_title(f"Image \u2014 {received_count}/{total_pkts} pkts ({pct:.0f}%)",
                     fontsize=10.5, color=INK)
        if missing_count > 0:
            message = (
                "Image not decodable yet\n"
                f"Missing {missing_count} packet"
                f"{'' if missing_count == 1 else 's'}"
            )
        else:
            message = "Image decode pending"
        ax.text(
            0.5, 0.5, message,
            transform=ax.transAxes,
            fontsize=10,
            color=MUTED,
            ha="center",
            va="center",
        )
        fig.canvas.draw_idle()
        fig.canvas.flush_events()


def update_bulk_preview(fig, ax, preview_text: str, packet_count: int, band_name: str):
    """Render a rolling monospaced text view for continuous bulk streaming."""
    ax.clear()
    ax.set_facecolor(SURFACE)
    ax.axis("off")

    title = f"Bulk Text - {packet_count} pkts on {band_name}"
    ax.set_title(title, fontsize=11, color=INK)

    if preview_text:
        recent = preview_text[-2200:]
        wrapped = textwrap.fill(recent, width=68, break_long_words=False,
                                replace_whitespace=False)
        ax.text(0.02, 0.98, wrapped, transform=ax.transAxes,
                fontsize=9, fontfamily="monospace", color=INK,
                va="top", ha="left")
    else:
        ax.text(0.5, 0.5, "Waiting for bulk text...",
                transform=ax.transAxes, fontsize=13,
                color=APPLE_BLUE, va="center", ha="center",
                fontfamily="monospace")

    fig.canvas.draw_idle()
    fig.canvas.flush_events()


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
