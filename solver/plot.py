# Draw ASP bitmap from file (CLI version)
import re
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch
from matplotlib.collections import LineCollection
from math import ceil

def parse_asp_file(path):
    """Parse ASP file with facts like wall(X,Y). robot(X,Y). target(X,Y). restricted_area(X,Y)."""
    with open(path, "r", encoding="utf-8") as f:
        data = f.read()

    size_match = re.search(r"#const\s+size\s*=\s*(\d+)", data)
    size = int(size_match.group(1)) if size_match else 10

    facts = {
        "robot": re.findall(r"robot\((\d+),\s*(\d+)\)", data),
        "target": re.findall(r"target\((\d+),\s*(\d+)\)", data),
        "wall": re.findall(r"wall\((\d+),\s*(\d+)\)", data),
        "restricted_area": re.findall(r"restricted_area\((\d+),\s*(\d+)\)", data),
        "obstacle": re.findall(r"obstacle\((\d+),\s*(\d+)\)", data),
    }
    for k, v in facts.items():
        facts[k] = [(int(x), int(y)) for x, y in v]
    return size, facts

def _paint(grid, points, value):
    """Vectorized cell assignment. Points are 1-based (x,y)."""
    if not points:
        return
    pts = np.asarray(points, dtype=int) - 1  # switch to 0-based
    # Guard against out-of-bounds coordinates in the input file.
    mask = (
        (pts[:, 0] >= 0) & (pts[:, 0] < grid.shape[1]) &
        (pts[:, 1] >= 0) & (pts[:, 1] < grid.shape[0])
    )
    pts = pts[mask]
    if pts.size:
        grid[pts[:, 1], pts[:, 0]] = value

def _add_coarse_grid(ax, size, step):
    """Efficient grid using one LineCollection rather than 1e6 ticks."""
    if step <= 0:
        return
    lo, hi = -0.5, size - 0.5

    # Vertical and horizontal line segments
    xs = np.arange(-0.5, size, step)
    ys = np.arange(-0.5, size, step)

    vlines = [((x, lo), (x, hi)) for x in xs]
    hlines = [((lo, y), (hi, y)) for y in ys]
    segs = vlines + hlines

    lc = LineCollection(segs, linewidths=0.5, colors="black")
    # Rasterize to keep vector output small when saving as PDF/SVG.
    lc.set_rasterized(True)
    ax.add_collection(lc)

def draw_bitmap(size, facts, want_grid=True, max_grid_lines=50):
    """
    Draw map. No per-cell ticks. Optional coarse grid with at most ~max_grid_lines each way.
    """
    if size>200:
        want_grid=False  # disable grid for very large maps by default
    grid = np.zeros((size, size), dtype=np.uint8)

    # Vectorized painting
    _paint(grid, facts.get("wall", []), 1)
    _paint(grid, facts.get("restricted_area", []), 2)
    _paint(grid, facts.get("robot", []), 3)
    _paint(grid, facts.get("target", []), 4)
    _paint(grid, facts.get("obstacle", []), 5)

    # Colormap and normalization
    cmap = ListedColormap(["#ffffff", "#222222", "#f9bf10", "#1f77b4", "#d62728", "#9467bd"])
    norm = BoundaryNorm(np.arange(-0.5, 6), cmap.N)

    # Keep figure size reasonable regardless of grid size
    fig, ax = plt.subplots(figsize=(6, 6), dpi=120)

    # Show as image. nearest avoids blurring. Rasterized keeps vector outputs light.
    im = ax.imshow(grid, cmap=cmap, norm=norm, origin="upper", interpolation="nearest", rasterized=True)

    # No per-cell ticks
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(-0.5, size - 0.5)
    ax.set_ylim(size - 0.5, -0.5)  # origin="upper" layout

    # Coarse grid: aim for <= max_grid_lines lines each direction
    if want_grid and max_grid_lines > 0:
        step = ceil(size / max_grid_lines)
        _add_coarse_grid(ax, size, step)

    # Labels for robot/target remain cheap (usually few points)
    for (x, y) in facts.get("robot", []):
        ax.text(x - 1, y - 1, "R", ha="center", va="center", color="white", weight="bold", fontsize=10)
    for (x, y) in facts.get("target", []):
        ax.text(x - 1, y - 1, "T", ha="center", va="center", color="white", weight="bold", fontsize=10)

    # Legend
    legend = [
        Patch(facecolor="#222222", label="wall"),
        Patch(facecolor="#f9bf10", label="restricted_area"),
        Patch(facecolor="#1f77b4", label="robot"),
        Patch(facecolor="#d62728", label="target"),
        Patch(facecolor="#9467bd", label="obstacle"),
    ]
    ax.legend(handles=legend, loc="upper right", frameon=True, fontsize=8)

    fig.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Draw ASP map as bitmap")
    parser.add_argument("--file", required=True, help="Path to ASP file (e.g., map.lp)")
    parser.add_argument("--no-grid", action="store_true", help="Disable grid overlay")
    parser.add_argument("--max-grid-lines", type=int, default=50, help="Max grid lines per direction")
    args = parser.parse_args()

    size, facts = parse_asp_file(args.file)
    draw_bitmap(size, facts, want_grid=not args.no_grid, max_grid_lines=args.max_grid_lines)
