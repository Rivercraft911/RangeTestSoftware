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
    ax.plot(x, data, color=color, linewidth=LINE_WIDTH, alpha=0.95)
    ax.fill_between(x, data, color=color, alpha=FILL_ALPHA)
    _draw_reference_line(ax, avg, color, style="--")
    _draw_reference_line(ax, floor, APPLE_RED, style=":", alpha=FLOOR_LINE_ALPHA)
    _annotate_metric(
        ax,
        f"avg {avg:.1f}",
        f"now {current:.1f}",
        color,
        _margin_color(current, floor),
    )

    lo, hi = _stable_ylim(data, pad_frac=0.22, min_range=10.0)
    ax.set_ylim(min(lo, floor - 5), hi)


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


def _plot_packet_loss_gauge(ax, snap, metrics):
    ax.clear()
    ax.set_facecolor(SURFACE)
    ax.set_title("", pad=0)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.text(
        0.5, 0.56, "RANGE TEST",
        transform=ax.transAxes,
        fontsize=12.5,
        color=INK,
        ha="center",
        va="center",
        fontfamily="monospace",
    )
    ax.text(
        0.5, 0.43, "DASHBOARD",
        transform=ax.transAxes,
        fontsize=12.5,
        color=APPLE_BLUE,
        ha="center",
        va="center",
        fontfamily="monospace",
    )
    ax.add_patch(
        matplotlib.patches.Rectangle(
            (0, 0), 1, 1,
            transform=ax.transAxes,
            fill=False,
            edgecolor=SPINE,
            linewidth=0.7,
        )
    )


def _plot_progress(ax, snap, metrics):
    ax.clear()
    ax.set_facecolor(SURFACE)
    ax.set_title("Transfer Progress", fontsize=9.2, pad=3)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    pct = metrics["transfer_progress_pct"]
    total = snap["bulk_total"]
    received = snap["bulk_received_count"]

    if pct is not None:
        headline = f"{pct:0.1f}%"
        footer = f"{received} / {total} PKTS"
        active_segments = int(round((pct / 100.0) * 28))
    else:
        headline = "N/A"
        footer = "STANDBY"
        active_segments = 0

    start_x = 0.045
    total_width = 0.91
    gap = 0.005
    segments = 28
    seg_w = (total_width - gap * (segments - 1)) / segments
    seg_y = 0.405
    seg_h = 0.17

    ax.add_patch(
        matplotlib.patches.Rectangle(
            (start_x - 0.012, seg_y - 0.025),
            total_width + 0.024,
            seg_h + 0.05,
            transform=ax.transAxes,
            fill=False,
            edgecolor=SPINE,
            linewidth=0.7,
        )
    )

    for index in range(segments):
        x = start_x + index * (seg_w + gap)
        color = APPLE_TEAL if index < active_segments else TRACK
        alpha = 0.95 if index < active_segments else 1.0
        ax.add_patch(
            matplotlib.patches.Rectangle(
                (x, seg_y),
                seg_w,
                seg_h,
                transform=ax.transAxes,
                linewidth=0,
                facecolor=color,
                alpha=alpha,
            )
        )

    ax.text(
        0.5, 0.66, headline,
        transform=ax.transAxes,
        fontsize=11.2,
        color=APPLE_TEAL,
        ha="center",
        va="center",
        fontfamily="monospace",
    )
    ax.text(
        0.5, 0.18, footer,
        transform=ax.transAxes,
        fontsize=6.2,
        color=MUTED,
        ha="center",
        va="center",
        fontfamily="monospace",
    )
    ax.add_patch(
        matplotlib.patches.Rectangle(
            (0, 0), 1, 1,
            transform=ax.transAxes,
            fill=False,
            edgecolor=SPINE,
            linewidth=0.7,
        )
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


def _plot_packet_stats(ax, snap, metrics):
    ax.clear()
    ax.set_facecolor(SURFACE)
    ax.set_title("Packet Stats", fontsize=9.2, pad=3)
    ax.grid(True, axis="y", alpha=0.34, linewidth=0.28, linestyle="--")
    ax.grid(False, axis="x")
    ax.set_ylabel("Packets")
    ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(5))
    ax.tick_params(length=2.5, width=0.6, colors=INK, pad=2)
    for spine in ax.spines.values():
        spine.set_color(SPINE)
        spine.set_linewidth(0.7)

    total = snap["bulk_total"]
    received = snap["bulk_received_count"]
    missing = metrics["missing_count"]

    bars = ax.bar(
        ["Received", "Missing"],
        [received, missing],
        color=[APPLE_GREEN, APPLE_RED],
        alpha=0.82,
        edgecolor=SPINE,
        linewidth=0.8,
        width=0.48,
    )
    for bar, value in zip(bars, [received, missing]):
        if value > 0:
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + max(total * 0.015, 0.45),
                str(value),
                ha="center",
                va="bottom",
                color=INK,
                fontsize=6.8,
                fontfamily="monospace",
            )

    subtitle = "counts" if total > 0 else "waiting"
    ax.text(
        0.5, 0.97, subtitle,
        transform=ax.transAxes,
        fontsize=5.8,
        color=MUTED,
        ha="center",
        va="top",
        fontfamily="monospace",
    )
    ax.set_ylim(0, max(total, received, 4) * 1.18)


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
    _plot_packet_loss_gauge(axes["pkt_loss"], snap, metrics)
    _plot_snr(axes["uhf_snr"], snap["uhf_snr"], APPLE_YELLOW, "UHF SNR")
    _plot_snr(
        axes["sb_snr"],
        snap["sband_snr"],
        APPLE_BLUE,
        "S-Band SNR",
        empty_message="No S-Band bulk yet",
    )
    _plot_progress(axes["progress"], snap, metrics)
    _plot_throughput(axes["throughput"], snap["throughput"], metrics)
    _plot_link_margin(axes["link_margin"], metrics)
    _plot_packet_stats(axes["pkt_stats"], snap, metrics)

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
