# Draw ASP bitmap from file (CLI version)
import re
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch

def parse_asp_file(path):
    """Parse ASP file with facts like wall(X,Y). robot(X,Y). target(X,Y). restricted_area(X,Y)."""
    data = open(path).read()
    size_match = re.search(r"#const\s+size\s*=\s*(\d+)", data)
    size = int(size_match.group(1)) if size_match else 10

    facts = {
        "robot": re.findall(r"robot\((\d+),\s*(\d+)\)", data),
        "target": re.findall(r"target\((\d+),\s*(\d+)\)", data),
        "wall": re.findall(r"wall\((\d+),\s*(\d+)\)", data),
        "restricted_area": re.findall(r"restricted_area\((\d+),\s*(\d+)\)", data),
        "obstacle": re.findall(r"obstacle\((\d+),\s*(\d+)\)", data)
    }
    for k, v in facts.items():
        facts[k] = [(int(x), int(y)) for x, y in v]
    return size, facts

def draw_bitmap(size, facts):
    grid = np.zeros((size, size), dtype=int)
    for x, y in facts.get("wall", []): grid[y-1, x-1] = 1
    for x, y in facts.get("restricted_area", []): grid[y-1, x-1] = 2
    for x, y in facts.get("robot", []): grid[y-1, x-1] = 3
    for x, y in facts.get("target", []): grid[y-1, x-1] = 4
    for x, y in facts.get("obstacle", []): grid[y-1, x-1] = 5

    cmap = ListedColormap(["#ffffff", "#222222", "#f9bf10", "#1f77b4", "#d62728", "#9467bd"])
    norm = BoundaryNorm(np.arange(-0.5, 6), cmap.N)
    fig, ax = plt.subplots(figsize=(6,6))
    ax.imshow(grid, cmap=cmap, norm=norm, origin="upper", interpolation="nearest")

    ax.set_xticks(np.arange(-.5, size, 1), minor=True)
    ax.set_yticks(np.arange(-.5, size, 1), minor=True)
    ax.grid(which="minor", color="#000000", linewidth=1)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    for (x, y) in facts.get("robot", []):
        ax.text(x-1, y-1, "P", ha="center", va="center", color="white", weight="bold", fontsize=14)
    for (x, y) in facts.get("target", []):
        ax.text(x-1, y-1, "T", ha="center", va="center", color="white", weight="bold", fontsize=14)

    legend = [
        Patch(facecolor="#222222", label="wall"),
        Patch(facecolor="#f9bf10", label="restricted_area"),
        Patch(facecolor="#1f77b4", label="robot"),
        Patch(facecolor="#d62728", label="target"),
        Patch(facecolor="#9467bd", label="obstacle")
    ]
    ax.legend(handles=legend, loc="upper right", frameon=True)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Draw ASP map as bitmap")
    parser.add_argument("--file", required=True, help="Path to ASP file (e.g., map.lp)")
    args = parser.parse_args()

    size, facts = parse_asp_file(args.file)
    draw_bitmap(size, facts)
