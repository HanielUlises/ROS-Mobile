"""Figure style for the multi-robot SLAM figures.

A single-column journal look: serif text with matching STIX math, hairline rules,
inward ticks on all four axes, no coloured furniture. Series colours are a
colour-blind-safe qualitative set (Okabe–Ito), and every distinction carried by
colour is also carried by line style so the figures survive greyscale printing.
"""

import matplotlib

matplotlib.use('Agg')

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap, BoundaryNorm  # noqa: E402

# Occupancy display classes.
UNKNOWN, FREE, OCCUPIED = 0, 1, 2

# Unknown must read as absent rather than as a third kind of terrain, so it is a
# light neutral; free is the page; occupied is ink.
OCC_CMAP = ListedColormap(['#e8e8e6', '#ffffff', '#111111'])
OCC_NORM = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], OCC_CMAP.N)

# Okabe-Ito: blue, vermillion, bluish green.
SERIES = ['#0072b2', '#d55e00', '#009e73']
SERIES_DASH = [(0, ()), (0, (4, 1.6)), (0, (1.2, 1.2))]

RULE = '#222222'
FAINT = '#b9b9b6'


def apply():
    plt.rcParams.update({
        'figure.dpi': 300,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.02,

        'font.family': 'serif',
        'font.serif': ['STIXGeneral', 'DejaVu Serif'],
        'mathtext.fontset': 'stix',
        'font.size': 8.5,
        'axes.titlesize': 8.5,
        'axes.labelsize': 8.5,
        'xtick.labelsize': 7.5,
        'ytick.labelsize': 7.5,
        'legend.fontsize': 7.5,

        'axes.linewidth': 0.6,
        'axes.edgecolor': RULE,
        'axes.labelcolor': RULE,
        'axes.titlepad': 4.0,
        'axes.spines.top': True,
        'axes.spines.right': True,

        'xtick.direction': 'in',
        'ytick.direction': 'in',
        'xtick.top': True,
        'ytick.right': True,
        'xtick.major.width': 0.6,
        'ytick.major.width': 0.6,
        'xtick.minor.width': 0.4,
        'ytick.minor.width': 0.4,
        'xtick.major.size': 3.0,
        'ytick.major.size': 3.0,
        'xtick.minor.size': 1.6,
        'ytick.minor.size': 1.6,
        'xtick.color': RULE,
        'ytick.color': RULE,

        'legend.frameon': False,
        'legend.handlelength': 1.9,
        'legend.handletextpad': 0.6,
        'legend.labelspacing': 0.35,
        'legend.borderaxespad': 0.5,

        'lines.linewidth': 1.0,
        'lines.antialiased': True,

        'text.color': RULE,
        'figure.facecolor': 'white',
        'axes.facecolor': 'white',
    })


def panel_label(ax, text, loc='upper left'):
    """Journal-style (a), (b), (c) panel tag."""
    x, ha = (0.02, 'left') if 'left' in loc else (0.98, 'right')
    y, va = (0.98, 'top') if 'upper' in loc else (0.02, 'bottom')
    ax.text(x, y, text, transform=ax.transAxes, ha=ha, va=va,
            fontsize=8.5, fontweight='bold', color=RULE,
            bbox=dict(facecolor='white', edgecolor='none', pad=1.4, alpha=0.85))


def scale_bar(ax, length_m, extent, label=None):
    """Metric scale bar, drawn in data coordinates at a fixed inset."""
    x0, x1, y0, y1 = extent
    pad_x = 0.05 * (x1 - x0)
    pad_y = 0.07 * (y1 - y0)
    bx = x1 - pad_x - length_m
    by = y0 + pad_y

    ax.plot([bx, bx + length_m], [by, by], color=RULE, lw=1.6,
            solid_capstyle='butt', zorder=5)
    for end in (bx, bx + length_m):
        ax.plot([end, end], [by - 0.012 * (y1 - y0), by + 0.012 * (y1 - y0)],
                color=RULE, lw=1.0, zorder=5)
    ax.text(bx + length_m / 2.0, by + 0.02 * (y1 - y0),
            label or f'{length_m:g} m', ha='center', va='bottom',
            fontsize=7.0, color=RULE, zorder=5)
