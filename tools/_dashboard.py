"""
Kawaii-themed matplotlib dashboard for LoRa image transfer stats.
"""
import io

from PIL import Image, ImageFile

from _serial_bridge import IMAGE_DATA_PER_PKT

ImageFile.LOAD_TRUNCATED_IMAGES = True

# -- Kawaii Pastel palette --
KP_BG       = "#FFF5F9"   # Pale Pink background
KP_FACE     = "#FFFAFC"   # Rose White plot face
KP_TEXT     = "#5B4970"   # Muted Purple text
KP_GRID     = "#F0D6E8"   # Light Mauve grid
KP_SPINE    = "#E8C6DC"   # Soft Rose spines
KP_PINK     = "#F075AB"   # Rose Pink (RSSI)
KP_LAVENDER = "#9B8EC4"   # Soft Lavender (SNR)
KP_PEACH    = "#F2A685"   # Peach Glow (throughput)
KP_MINT     = "#7EC8B8"   # Mint Dream (received)
KP_LILAC    = "#D484BF"   # Soft Lilac (missing)
KP_BUTTER   = "#F0D58C"   # Butter Yellow (retransmitted)
KP_FILL     = "#ffdef2"   # Pastel Pink (area fills)

RSSI_SENSITIVITY_FLOOR = -130


def apply_kawaii_theme():
    import matplotlib.pyplot as plt
    import matplotlib as mpl

    plt.rcParams.update({
        "axes.prop_cycle": mpl.cycler(color=[
            KP_PINK, KP_LAVENDER, KP_MINT, KP_PEACH, KP_LILAC, KP_BUTTER
        ]),
        "figure.facecolor": KP_BG,
        "axes.facecolor": KP_FACE,
        "text.color": KP_TEXT,
        "axes.labelcolor": KP_TEXT,
        "xtick.color": KP_TEXT,
        "ytick.color": KP_TEXT,
        "axes.grid": True,
        "grid.color": KP_GRID,
        "grid.alpha": 0.6,
        "grid.linewidth": 0.8,
        "grid.linestyle": "--",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.edgecolor": KP_SPINE,
        "axes.linewidth": 1.2,
        "font.family": "sans-serif",
        "font.sans-serif": [
            "Arial Rounded MT Bold", "Avenir Next", "Helvetica Neue",
            "Avenir", "Arial", "sans-serif"
        ],
        "font.size": 10,
        "axes.titlesize": 13,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "lines.linewidth": 2.5,
        "lines.markersize": 7,
        "legend.frameon": True,
        "legend.framealpha": 0.8,
        "legend.facecolor": KP_FACE,
        "legend.edgecolor": KP_GRID,
        "legend.fontsize": 9,
        "axes.titlepad": 12,
        "axes.labelpad": 8,
        "figure.dpi": 120,
        "savefig.dpi": 180,
        "savefig.facecolor": KP_BG,
        "axes.unicode_minus": False,
    })


def init_stats_figure(transfer_num):
    import matplotlib.pyplot as plt

    fig, ((ax_rssi, ax_snr), (ax_thru, ax_stats)) = plt.subplots(
        2, 2, figsize=(11, 7), facecolor=KP_BG, constrained_layout=True)
    fig.canvas.manager.set_window_title(
        f"Transfer #{transfer_num} — Live Stats")
    fig.canvas.draw()
    fig.canvas.flush_events()
    return fig, ax_rssi, ax_snr, ax_thru, ax_stats


def init_preview_figure():
    import matplotlib.pyplot as plt

    img_fig, img_ax = plt.subplots(
        1, 1, figsize=(5, 5), facecolor=KP_BG, constrained_layout=True)
    img_fig.canvas.manager.set_window_title("Image Preview")
    img_ax.set_facecolor(KP_BG)
    img_ax.axis("off")
    img_fig.canvas.draw()
    img_fig.canvas.flush_events()
    return img_fig, img_ax


def update_live_plots(ax_rssi, ax_snr, ax_thru, ax_stats,
                      rssi_data, snr_data, pkt_times,
                      rx_count, total_pkts, retx_count):
    x = list(range(len(rssi_data)))

    # -- RSSI --
    ax_rssi.clear()
    ax_rssi.set_title("RSSI (dBm)", fontsize=13, fontweight="bold")
    ax_rssi.set_xlabel("Packet #", fontsize=10)
    ax_rssi.plot(x, rssi_data, color=KP_PINK, linewidth=2.5, alpha=0.9)
    ax_rssi.fill_between(x, rssi_data, color=KP_PINK, alpha=0.12)
    ax_rssi.axhline(y=RSSI_SENSITIVITY_FLOOR, color=KP_LILAC,
                    linewidth=1.0, linestyle=":", alpha=0.5)
    if rssi_data:
        avg = sum(rssi_data) / len(rssi_data)
        ax_rssi.axhline(y=avg, color=KP_LILAC, linewidth=1.2,
                        linestyle="--", alpha=0.7)
        ax_rssi.annotate(f"avg {avg:.0f}", xy=(len(x) * 0.02, avg),
                        fontsize=8, color=KP_LILAC, va="bottom")

    # -- SNR --
    ax_snr.clear()
    ax_snr.set_title("SNR (dB)", fontsize=13, fontweight="bold")
    ax_snr.set_xlabel("Packet #", fontsize=10)
    ax_snr.plot(x, snr_data, color=KP_LAVENDER, linewidth=2.5, alpha=0.9)
    ax_snr.fill_between(x, snr_data, color=KP_LAVENDER, alpha=0.12)
    if snr_data:
        avg = sum(snr_data) / len(snr_data)
        ax_snr.axhline(y=avg, color=KP_LILAC, linewidth=1.2,
                       linestyle="--", alpha=0.7)
        ax_snr.annotate(f"avg {avg:.1f}", xy=(len(x) * 0.02, avg),
                       fontsize=8, color=KP_LILAC, va="bottom")

    # -- Throughput --
    ax_thru.clear()
    ax_thru.set_title("Throughput (KB/s)", fontsize=13, fontweight="bold")
    ax_thru.set_xlabel("Packet #", fontsize=10)
    if len(pkt_times) > 1:
        window = 20
        thru = []
        for i in range(len(pkt_times)):
            si = max(0, i - window)
            dt = pkt_times[i] - pkt_times[si]
            if dt > 0.1:
                pkts_w = i - si
                thru.append((pkts_w * IMAGE_DATA_PER_PKT / 1024) / dt)
            else:
                thru.append(0)
        if thru:
            p95 = sorted(thru)[int(len(thru) * 0.95)] if len(thru) > 5 else max(thru)
            ax_thru.set_ylim(0, max(p95 * 1.3, 1))
        ax_thru.plot(x, thru, color=KP_PEACH, linewidth=2.5, alpha=0.9)
        ax_thru.fill_between(x, thru, color=KP_PEACH, alpha=0.15)

    # -- Packet stats bar --
    ax_stats.clear()
    ax_stats.set_title("Packet Stats", fontsize=13, fontweight="bold")
    miss = max(0, total_pkts - rx_count)
    cats = ["Received", "Missing", "Retx"]
    vals = [rx_count, miss, retx_count]
    colors = [KP_MINT, KP_LILAC, KP_BUTTER]
    bars = ax_stats.bar(cats, vals, color=colors, alpha=0.85,
                        edgecolor="white", linewidth=1.5)
    for b, v in zip(bars, vals):
        if v > 0:
            ax_stats.text(b.get_x() + b.get_width() / 2,
                         b.get_height() + max(total_pkts * 0.01, 0.5),
                         str(v), ha="center", va="bottom",
                         color=KP_TEXT, fontsize=10, fontweight="bold")
    ax_stats.set_ylim(0, max(total_pkts, 1) * 1.2)


def update_image_preview(img_ax, received, total_pkts):
    jpeg = reassemble(received, total_pkts)
    try:
        img = Image.open(io.BytesIO(jpeg))
        img_ax.clear()
        img_ax.imshow(img)
        img_ax.axis("off")
        img_ax.set_title(f"Preview ({len(received)}/{total_pkts} pkts)",
                        fontsize=11, fontweight="bold", color=KP_TEXT)
    except Exception:
        pass


def reassemble(received, total_pkts):
    parts = []
    for i in range(total_pkts):
        if i in received:
            parts.append(received[i])
        else:
            parts.append(b'\x00' * IMAGE_DATA_PER_PKT)
    return b''.join(parts)


def show_image(jpeg_bytes):
    try:
        img = Image.open(io.BytesIO(jpeg_bytes))
        img.show()
    except Exception:
        pass
