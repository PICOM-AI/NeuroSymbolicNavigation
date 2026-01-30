"""
Call each API from apis.py and save result as JSON to output/<api-function>.json.
Requires: pip install nuplan-devkit
Usage: python apis_usage.py [map_root] [map_version] [map_name]
"""

import io
import os
import sys
import json
from contextlib import redirect_stdout, redirect_stderr
from typing import Any, Callable, Optional, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")


def _to_json_serializable(obj: Any) -> Any:
    """Convert result to something json.dumps can serialize (no debug)."""
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj
    if isinstance(obj, (list, tuple)):
        return [_to_json_serializable(x) for x in obj]
    if isinstance(obj, dict):
        return {str(k): _to_json_serializable(v) for k, v in obj.items()}
    try:
        json.dumps(obj, default=str)
        return obj
    except (TypeError, ValueError):
        return None


def run_and_log(json_path: str, func: Callable, *args: Any, **kwargs: Any) -> Tuple[bool, Optional[Exception]]:
    """Run func and write result as JSON to json_path (e.g. <file>.json). No stdout/stderr logging."""
    exc_holder = None
    result = None
    try:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = func(*args, **kwargs)
    except Exception as e:
        exc_holder = e
    payload = (
        {"ok": True, "result": _to_json_serializable(result)}
        if exc_holder is None
        else {"ok": False, "error": f"{type(exc_holder).__name__}: {exc_holder}"}
    )
    os.makedirs(os.path.dirname(json_path) or ".", exist_ok=True)
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, default=str)
    return exc_holder is None, exc_holder


def main() -> None:
    map_root = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("NUPLAN_MAPS_ROOT", "/root/maps")
    map_version = sys.argv[2] if len(sys.argv) > 2 else os.environ.get("NUPLAN_MAP_VERSION", "nuplan-maps-v1.0")
    map_name = sys.argv[3] if len(sys.argv) > 3 else "sg-one-north"
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print("Output:", OUTPUT_DIR, "Map root:", map_root, "Version:", map_version, "Map:", map_name)
    import apis
    map_api = None
    json_path = os.path.join(OUTPUT_DIR, "get_map.json")
    ok, err = run_and_log(json_path, apis.get_map, map_root, map_version, map_name)
    if not ok:
        print("get_map FAIL:", json_path, err)
        return
    with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
        map_api = apis.get_map(map_root, map_version, map_name)
    print("get_map OK")
    run_and_log(os.path.join(OUTPUT_DIR, "get_locations.json"), apis.get_locations, map_root, map_version)
    sample_box = [0.0, 0.0, 500.0, 500.0]
    sample_x, sample_y = 100.0, 100.0
    calls = [
        ("get_vector_layer_names", apis.get_vector_layer_names, [map_api]),
        ("get_all_lanes_polygons", apis.get_all_lanes_polygons, [map_api]),
        ("get_all_lane_connectors", apis.get_all_lane_connectors, [map_api]),
        ("get_all_intersections", apis.get_all_intersections, [map_api]),
        ("get_all_crosswalks", apis.get_all_crosswalks, [map_api]),
        ("get_all_walkways", apis.get_all_walkways, [map_api]),
        ("get_all_stop_polygons", apis.get_all_stop_polygons, [map_api]),
        ("get_all_carpark_areas", apis.get_all_carpark_areas, [map_api]),
        ("get_all_lane_groups_polygons", apis.get_all_lane_groups_polygons, [map_api]),
        ("get_all_boundaries", apis.get_all_boundaries, [map_api]),
        ("get_all_traffic_lights", apis.get_all_traffic_lights, [map_api]),
        ("get_all_road_segments", apis.get_all_road_segments, [map_api]),
        ("get_bounds", apis.get_bounds, [map_api, "lanes_polygons", None]),
        ("get_records_in_patch", apis.get_records_in_patch, [map_api, sample_box]),
        ("layers_on_point", apis.layers_on_point, [map_api, sample_x, sample_y]),
        ("records_on_point", apis.records_on_point, [map_api, sample_x, sample_y, "lanes_polygons"]),
        ("get_map_dimension", apis.get_map_dimension, [map_api]),
    ]
    for name, func, args in calls:
        p = os.path.join(OUTPUT_DIR, f"{name}.json")
        run_and_log(p, func, *args)
        print(" ", name)
    print("Done.")

if __name__ == "__main__":
    main()
