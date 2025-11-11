#!/usr/bin/env python3
"""
pgm_to_gardener_scaled.py

Convert a ROS-style occupancy PGM to a Gardener ASP map,
supporting coordinate origin (1,1) at top-left and scalable cells.

Usage:
  python3 pgm_to_gardener_scaled.py \
    --pgm maze.pgm \
    --out gardener_map.lp \
    --scale 2 \
    --occ-thresh 100 \
    --unknown-as-wall true \
    --invert false
"""
import argparse
from typing import Tuple, List


def read_pgm(path: str) -> Tuple[int, int, int, List[int]]:
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic not in [b"P5", b"P2"]:
            raise ValueError(f"Unsupported PGM format {magic}")
        # skip comments
        def next_line():
            while True:
                l = f.readline()
                if not l:
                    raise EOFError("Unexpected EOF")
                l = l.strip()
                if not l or l.startswith(b"#"):
                    continue
                return l

        dims = next_line().split()
        while len(dims) < 2:
            dims += next_line().split()
        w, h = int(dims[0]), int(dims[1])
        maxval = int(next_line())

        if magic == b"P5":
            pixels = list(f.read(w * h))
        else:
            data = []
            while len(data) < w * h:
                line = f.readline()
                if not line:
                    break
                if line.startswith(b"#"):
                    continue
                data += line.split()
            pixels = list(map(int, data[: w * h]))
    return w, h, maxval, pixels


def block_is_wall(block: List[int], occ_thresh: int, unknown_as_wall: bool, invert: bool, maxval: int) -> bool:
    scale = 255 / maxval
    for val in block:
        pix = int(val * scale + 0.5)
        if invert:
            pix = 255 - pix
        if pix <= occ_thresh:
            return True
        if 250 > pix > occ_thresh and unknown_as_wall:
            return True
    return False


def scale_down_grid(w: int, h: int, data: List[int], scale_factor: int, occ_thresh: int, unknown_as_wall: bool, invert: bool, maxval: int):
    new_w = w // scale_factor
    new_h = h // scale_factor
    walls = set()
    free = []
    for j in range(new_h):  # top to bottom
        for i in range(new_w):  # left to right
            block = []
            for dy in range(scale_factor):
                for dx in range(scale_factor):
                    src_x = i * scale_factor + dx
                    src_y = j * scale_factor + dy
                    if src_x < w and src_y < h:
                        block.append(data[src_y * w + src_x])
            if block_is_wall(block, occ_thresh, unknown_as_wall, invert, maxval):
                walls.add((i + 1, j + 1))  # top-left origin, 1-based
            else:
                free.append((i + 1, j + 1))
    return new_w, new_h, walls, free


def choose_start_goal(free: List[Tuple[int, int]]):
    if not free:
        raise ValueError("No free cell to place player/target.")
    return free[0], free[-1] if len(free) > 1 else free[0]


def write_lp(out_path: str, w: int, h: int, walls, player, target):
    with open(out_path, "w") as f:
        f.write(f"#const size={max(w,h)}.\n\n")
        f.write(f"player({player[0]},{player[1]}).\n")
        f.write(f"target({target[0]},{target[1]}).\n\n")
        for (x, y) in sorted(walls):
            f.write(f"wall({x},{y}).\n")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--pgm", required=True)
    p.add_argument("--out", default="gardener_map.lp")
    p.add_argument("--scale", type=int, default=1, help="Pixel block size per ASP cell")
    p.add_argument("--occ-thresh", type=int, default=100)
    p.add_argument("--unknown-as-wall", type=str, default="true")
    p.add_argument("--invert", type=str, default="false")
    args = p.parse_args()

    def as_bool(x): return x.lower() in ("1", "true", "yes", "y")

    w, h, maxval, data = read_pgm(args.pgm)
    new_w, new_h, walls, free = scale_down_grid(
        w, h, data, args.scale, args.occ_thresh, as_bool(args.unknown_as_wall), as_bool(args.invert), maxval
    )
    player, target = choose_start_goal(free)
    write_lp(args.out, new_w, new_h, walls, player, target)
    print(f"Converted {args.pgm} → {args.out}")
    print(f"Original {w}x{h}, scaled {new_w}x{new_h}, walls={len(walls)}, free={len(free)}")


if __name__ == "__main__":
    main()
