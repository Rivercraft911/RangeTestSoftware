"""
Live matplotlib dashboard
"""

GH_BG     = "#0d1117"
GH_FACE   = "#161b22"
GH_TEXT   = "#c9d1d9"
GH_GRID   = "#21262d"
GH_SPINE  = "#30363d"

AP_GREEN  = "#61BB46"
AP_YELLOW = "#FDB827"
AP_ORANGE = "#F5821F"
AP_RED    = "#E03A3E"
AP_PURPLE = "#963D97"
AP_BLUE   = "#009DDC"

# -- Sensitivity floors (dBm) --
SBAND_SENSITIVITY = {1: -117, 2: -120, 3: -123, 4: -129}
UHF_SENSITIVITY = -130


def apply_dark_theme():
    import matplotlib.pyplot as plt
    import matplotlib as mpl

    plt.rcParams.update({
        "axes.prop_cycle": mpl.cycler(color=[
            AP_GREEN, AP_YELLOW, AP_ORANGE, AP_RED, AP_PURPLE, AP_BLUE
        ]),
        "figure.facecolor": GH_BG,
        "axes.facecolor": GH_FACE,
        "text.color": GH_TEXT,
        "axes.labelcolor": GH_TEXT,
        "xtick.color": GH_TEXT,
        "ytick.color": GH_TEXT,
        "axes.grid": True,
        "grid.color": GH_GRID,
        "grid.alpha": 0.6,
        "grid.linewidth": 0.8,
        "grid.linestyle": "--",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.edgecolor": GH_SPINE,
        "axes.linewidth": 1.2,
        "font.family": "sans-serif",
        "font.sans-serif": [
            "SF Pro Display", "Helvetica Neue",
            "Arial", "sans-serif"
        ],
        "font.size": 10,
        "axes.titlesize": 13,
        "axes.titleweight": "bold",
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "lines.linewidth": 2.0,
        "lines.markersize": 5,
        "legend.frameon": True,
        "legend.framealpha": 0.8,
        "legend.facecolor": GH_FACE,
        "legend.edgecolor": GH_GRID,
        "legend.fontsize": 9,
        "axes.titlepad": 12,
        "axes.labelpad": 8,
        "figure.dpi": 120,
        "savefig.dpi": 180,
        "savefig.facecolor": GH_BG,
        "axes.unicode_minus": False,
    })


def init_dashboard():
    import matplotlib.pyplot as plt

    apply_dark_theme()

    fig, axes = plt.subplots(2, 3, figsize=(16, 9),
                             facecolor=GH_BG, constrained_layout=True)
    fig.canvas.manager.set_window_title("Balloon Range Test — Live Dashboard")

    ax_rssi = axes[0, 0]
    ax_snr = axes[0, 1]
    ax_rate = axes[0, 2]
    ax_margin = axes[1, 0]
    ax_log = axes[1, 1]
    ax_thru = axes[1, 2]

    for ax in axes.flat:
        ax.set_facecolor(GH_FACE)

    ax_rssi.set_title("RSSI vs Time")
    ax_snr.set_title("SNR vs Time")
    ax_rate.set_title("Beacon Rate (per 10s)")
    ax_margin.set_title("Link Margin (dB)")
    ax_log.set_title("Command Log")
    ax_thru.set_title("Throughput (KB/s)")

    ax_log.axis("off")

    fig.canvas.draw()
    fig.canvas.flush_events()

    return fig, ax_rssi, ax_snr, ax_rate, ax_margin, ax_log, ax_thru


class DashboardState:
    def __init__(self):
        self.sband_rssi = []  # (time_s, rssi_dbm)
        self.uhf_rssi = []
        self.sband_snr = []   # (time_s, snr_db)
        self.uhf_snr = []
        self.sband_beacons = []  # timestamps
        self.uhf_beacons = []
        self.cmd_log = []     # list of strings
        self.thru_data = []   # (time_s, kb_s)
        self.sband_profile = 1
        self.start_time = None

    def elapsed(self, now=None):
        import time as _t
        if self.start_time is None:
            self.start_time = _t.time()
        if now is None:
            now = _t.time()
        return now - self.start_time

    def add_beacon(self, beacon, timestamp=None):
        import time as _t
        if timestamp is None:
            timestamp = _t.time()
        t = self.elapsed(timestamp)
        rssi_dbm = beacon.rssi_x100 / 100.0
        snr_db = beacon.snr_x100 / 100.0

        if beacon.band == "SBAND":
            self.sband_rssi.append((t, rssi_dbm))
            self.sband_snr.append((t, snr_db))
            self.sband_beacons.append(timestamp)
            self.sband_profile = beacon.sband_profile
        elif beacon.band == "UHF":
            self.uhf_rssi.append((t, rssi_dbm))
            self.uhf_snr.append((t, snr_db))
            self.uhf_beacons.append(timestamp)

    def add_cmd_log(self, text):
        self.cmd_log.append(text)
        if len(self.cmd_log) > 12:
            self.cmd_log = self.cmd_log[-12:]

    def add_throughput(self, kb_s, timestamp=None):
        import time as _t
        if timestamp is None:
            timestamp = _t.time()
        t = self.elapsed(timestamp)
        self.thru_data.append((t, kb_s))


def update_dashboard(fig, ax_rssi, ax_snr, ax_rate, ax_margin,
                     ax_log, ax_thru, state):
    import matplotlib.pyplot as plt
    import time as _t

    # -- RSSI vs Time --
    ax_rssi.clear()
    ax_rssi.set_title("RSSI vs Time (dBm)")
    ax_rssi.set_xlabel("Time (s)")

    if state.sband_rssi:
        ts, vs = zip(*state.sband_rssi)
        ax_rssi.plot(ts, vs, color=AP_GREEN, linewidth=2.0, alpha=0.9,
                     label="S-Band")
    if state.uhf_rssi:
        ts, vs = zip(*state.uhf_rssi)
        ax_rssi.plot(ts, vs, color=AP_YELLOW, linewidth=2.0, alpha=0.9,
                     label="UHF")

    # Sensitivity floors
    sband_floor = SBAND_SENSITIVITY.get(state.sband_profile, -120)
    ax_rssi.axhline(y=sband_floor, color=AP_GREEN, linewidth=0.8,
                    linestyle=":", alpha=0.4, label=f"S-Band floor ({sband_floor})")
    ax_rssi.axhline(y=UHF_SENSITIVITY, color=AP_YELLOW, linewidth=0.8,
                    linestyle=":", alpha=0.4, label=f"UHF floor ({UHF_SENSITIVITY})")
    ax_rssi.legend(loc="upper right", fontsize=8)

    # -- SNR vs Time --
    ax_snr.clear()
    ax_snr.set_title("SNR vs Time (dB)")
    ax_snr.set_xlabel("Time (s)")

    if state.sband_snr:
        ts, vs = zip(*state.sband_snr)
        ax_snr.plot(ts, vs, color=AP_GREEN, linewidth=2.0, alpha=0.9,
                    label="S-Band")
    if state.uhf_snr:
        ts, vs = zip(*state.uhf_snr)
        ax_snr.plot(ts, vs, color=AP_YELLOW, linewidth=2.0, alpha=0.9,
                    label="UHF")
    ax_snr.legend(loc="upper right", fontsize=8)

    # -- Beacon Rate --
    ax_rate.clear()
    ax_rate.set_title("Beacon Rate (per 10s)")

    now = _t.time()
    window = 10.0
    sband_recent = sum(1 for t in state.sband_beacons if now - t < window)
    uhf_recent = sum(1 for t in state.uhf_beacons if now - t < window)

    bars = ax_rate.bar(["S-Band", "UHF"], [sband_recent, uhf_recent],
                       color=[AP_GREEN, AP_YELLOW], alpha=0.85,
                       edgecolor="white", linewidth=1.2)
    for b, v in zip(bars, [sband_recent, uhf_recent]):
        if v > 0:
            ax_rate.text(b.get_x() + b.get_width() / 2,
                         b.get_height() + 0.2,
                         str(v), ha="center", va="bottom",
                         color=GH_TEXT, fontsize=10, fontweight="bold")
    ax_rate.set_ylim(0, max(sband_recent, uhf_recent, 1) * 1.3)

    # -- Link Margin --
    ax_margin.clear()
    ax_margin.set_title("Link Margin (dB)")

    sband_margin = None
    uhf_margin = None
    if state.sband_rssi:
        latest_rssi = state.sband_rssi[-1][1]
        sband_margin = latest_rssi - sband_floor
    if state.uhf_rssi:
        latest_rssi = state.uhf_rssi[-1][1]
        uhf_margin = latest_rssi - UHF_SENSITIVITY

    labels = []
    values = []
    colors = []
    if sband_margin is not None:
        labels.append("S-Band")
        values.append(sband_margin)
        colors.append(AP_GREEN if sband_margin > 10 else
                      AP_ORANGE if sband_margin > 3 else AP_RED)
    if uhf_margin is not None:
        labels.append("UHF")
        values.append(uhf_margin)
        colors.append(AP_YELLOW if uhf_margin > 10 else
                      AP_ORANGE if uhf_margin > 3 else AP_RED)

    if values:
        margin_bars = ax_margin.bar(labels, values, color=colors, alpha=0.85,
                                    edgecolor="white", linewidth=1.2)
        for b, v in zip(margin_bars, values):
            ax_margin.text(b.get_x() + b.get_width() / 2,
                           b.get_height() + 0.5,
                           f"{v:.1f} dB", ha="center", va="bottom",
                           color=GH_TEXT, fontsize=10, fontweight="bold")
        ax_margin.axhline(y=0, color=AP_RED, linewidth=1.0,
                          linestyle="-", alpha=0.5)

    # -- Command Log --
    ax_log.clear()
    ax_log.set_title("Command Log")
    ax_log.axis("off")
    if state.cmd_log:
        text = "\n".join(state.cmd_log[-10:])
        ax_log.text(0.02, 0.98, text, transform=ax_log.transAxes,
                    verticalalignment="top", fontsize=9,
                    color=GH_TEXT, family="monospace")

    # -- Throughput --
    ax_thru.clear()
    ax_thru.set_title("Throughput (KB/s)")
    ax_thru.set_xlabel("Time (s)")
    if state.thru_data:
        ts, vs = zip(*state.thru_data)
        ax_thru.plot(ts, vs, color=AP_ORANGE, linewidth=2.0, alpha=0.9)
        ax_thru.fill_between(ts, vs, color=AP_ORANGE, alpha=0.15)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()
