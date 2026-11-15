from pathlib import Path
import shutil
import subprocess
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm


def _pick_cjk_font():
    """Pick an installed CJK-capable font without bundling any font files."""
    candidates = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Arial Unicode MS"]
    for name in candidates:
        try:
            fm.findfont(name, fallback_to_default=False)
            return name
        except Exception:
            pass
    # On Linux, fc-match often sees TTC fonts that matplotlib has not cached yet.
    if shutil.which("fc-match"):
        try:
            path = subprocess.check_output(
                ["fc-match", "-f", "%{file}", "Noto Sans CJK SC"], text=True
            ).strip()
            if path:
                fm.fontManager.addfont(path)
                return fm.FontProperties(fname=path).get_name()
        except Exception:
            pass
    return "DejaVu Sans"


def setup_style():
    """Portable paper-like matplotlib settings with CJK font fallbacks."""
    family = _pick_cjk_font()
    mpl.rcParams.update({
        "font.family": family,
        "axes.unicode_minus": False,
        "mathtext.fontset": "stix",
        "font.size": 10,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "legend.fontsize": 8,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "figure.dpi": 120,
        "savefig.dpi": 300,
        "axes.grid": True,
        "grid.alpha": 0.20,
        "axes.spines.top": False,
        "axes.spines.right": False,
    })


def save_figure(fig, outdir, stem, save_pdf=False):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    png = outdir / f"{stem}.png"
    fig.savefig(png, bbox_inches="tight")
    if save_pdf:
        fig.savefig(outdir / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)
    return png
