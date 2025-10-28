#!/usr/bin/env python3
"""
pgm2asp.py

Convert a ROS-style occupancy PGM (/mnt/data/maze.pgm by default)
to a Robot ASP map (walls only; restricted_areas/obstacles omitted).

Conventions
- PGM values: 0=occupied (black), ~205=unknown (gray), 254/255=free (white) by default.
- Grid origin: (1,1) at TOP-LEFT. x increases to the right, y increases downward.
- Output: facts for #const size, #const obstacles=0, robot(X,Y), target(X,Y), wall(X,Y).

Usage
  python3 pgm2asp.py \
    --pgm /mnt/data/maze.pgm \
    --out map.lp \
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
            raise ValueError(f"Unsupported PGM magic: {magic}")

        def _read_token():
            while True:
                tok = f.readline()
                if not tok:
                    raise EOFError("Unexpected EOF")
                tok = tok.strip()
                if tok.startswith(b"#") or tok == b"":
                    continue
                return tok

        dims = _read_token().split()
        while len(dims) < 2:
            dims += _read_token().split()
        w, h = int(dims[0]), int(dims[1])

        maxval = int(_read_token())
        if maxval <= 0 or maxval > 65535:
            raise ValueError("Invalid maxval in PGM")

        if magic == b"P5":
            px_size = 1 if maxval < 256 else 2
            raw = f.read(w * h * px_size)
            if len(raw) != w * h * px_size:
                raise EOFError("Not enough pixel data")
            if px_size == 1:
                data = list(raw)
            else:
                data = [raw[i]*256 + raw[i+1] for i in range(0, len(raw), 2)]
        else:
            tokens = []
            while len(tokens) < w * h:
                line = f.readline()
                if not line:
                    break
                if line.startswith(b"#"):
                    continue
                tokens += line.split()
            if len(tokens) < w * h:
                raise EOFError("Not enough ASCII pixel data")
            data = list(map(int, tokens[:w*h]))
    return w, h, maxval, data

def classify_cells(
    w: int,
    h: int,
    maxval: int,
    data: List[int],
    occ_thresh: int = 100,
    unknown_as_wall: bool = True,
    invert: bool = False
):
    """
    Returns:
      walls: set of (x,y) 1-based with origin TOP-LEFT
      free:  list of (x,y)
    """
    walls = set()
    free = []
    scale = 255 / maxval
    for row in range(h):      # 0 = top row in PGM
        for col in range(w):  # 0 = leftmost
            val = int(data[row*w + col] * scale + 0.5)
            pix = 255 - val if invert else val

            if pix <= occ_thresh:
                cls = 'occ'
            elif pix >= 250:
                cls = 'free'
            else:
                cls = 'unk'

            # TOP-LEFT origin: (1,1) is top-left; y grows downward
            x = col + 1
            y = row + 1

            if cls == 'occ' or (unknown_as_wall and cls == 'unk'):
                walls.add((x, y))
            elif cls == 'free' or (not unknown_as_wall and cls == 'unk'):
                free.append((x, y))
            else:
                walls.add((x, y))
    return walls, free


def choose_start_goal(free_cells: List[Tuple[int,int]]) -> Tuple[int,int]:
    if not free_cells:
        raise ValueError("No free cells detected to place robot/target.")
    goal = free_cells[0]
    return goal

def write_asp_map_lp(
    path: str,
    w: int,
    h: int,
    walls: List[Tuple[int,int]],
    target: Tuple[int,int],
    obstacles: int = 0
):
    with open(path, "w") as out:
        out.write(f"#const size={max(w,h)}.\n")
        out.write(f"robot({int(w/2)}, {int(h/2)}).\n")
        out.write(f"target({target[0]}, {target[1]}).\n\n")
        for (x,y) in sorted(walls):
            out.write(f"wall({x}, {y}).\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pgm", default="/mnt/data/maze.pgm")
    ap.add_argument("--out", default="map.lp")
    ap.add_argument("--occ-thresh", type=int, default=100,
                    help="Pixels <= threshold treated as occupied after optional inversion [default 100].")
    ap.add_argument("--unknown-as-wall", type=str, default="true",
                    help="Treat mid-gray unknown as wall [true|false].")
    ap.add_argument("--invert", type=str, default="false",
                    help="Invert pixel values before thresholding [true|false] if your map encoding is flipped.")
    args = ap.parse_args()

    def as_bool(s: str) -> bool:
        return s.strip().lower() in ("1", "true", "t", "yes", "y")

    unknown_as_wall = as_bool(args.unknown_as_wall)
    invert = as_bool(args.invert)

    w, h, maxval, data = read_pgm(args.pgm)
    walls, free_cells = classify_cells(
        w, h, maxval, data,
        occ_thresh=args.occ_thresh,
        unknown_as_wall=unknown_as_wall,
        invert=invert
    )
    target = choose_start_goal(free_cells)
    write_asp_map_lp(args.out, w, h, walls, target)
    print(f"Wrote {args.out}  |  size={w}x{h}  walls={len(walls)}  free={len(free_cells)}  target={target}")

if __name__ == "__main__":
    main()
