"""
NuPlan Map API - Functions to get each component from the map.

This module wraps the NuPlan devkit map API (GPKG-based) and provides
functions corresponding to the map structure in structure_nuplan_design.json.

Requires: pip install nuplan-devkit
Map root: directory containing {map_version}.json and location/version/map.gpkg files.
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple, Union

if TYPE_CHECKING:
    import geopandas as gpd

try:
    import geopandas as gpd
    GEOPANDAS_AVAILABLE = True
except ImportError:
    GEOPANDAS_AVAILABLE = False
    gpd = None  # type: ignore

# Default map root and version (override with env NUPLAN_MAPS_ROOT, NUPLAN_MAP_VERSION)
DEFAULT_MAP_ROOT = os.environ.get("NUPLAN_MAPS_ROOT", "/root/maps")
DEFAULT_MAP_VERSION = os.environ.get("NUPLAN_MAP_VERSION", "nuplan-maps-v1.0")
# Known locations in NuPlan maps
MAP_LOCATIONS = ("sg-one-north", "us-ma-boston", "us-nv-las-vegas-strip", "us-pa-pittsburgh-hazelwood")


def get_maps_db(map_root: str = DEFAULT_MAP_ROOT, map_version: str = DEFAULT_MAP_VERSION):
    """
    Get the GPKG MapsDB instance.
    Dependencies: nuplan.database.maps_db.gpkg_mapsdb.GPKGMapsDB, get_maps_db from map_factory.
    """
    from nuplan.common.maps.nuplan_map.map_factory import get_maps_db as _get_maps_db
    return _get_maps_db(map_root, map_version)


def get_map(
    map_root: str = DEFAULT_MAP_ROOT,
    map_version: str = DEFAULT_MAP_VERSION,
    map_name: str = "sg-one-north",
    use_wrapper: bool = True,
):
    """
    Load the NuPlan map. Required before calling other map functions.
    :param map_root: Root folder containing map version json and gpkg files.
    :param map_version: Map version string (e.g. nuplan-maps-v1.0).
    :param map_name: Map location (sg-one-north, us-ma-boston, us-nv-las-vegas-strip, us-pa-pittsburgh-hazelwood).
    :param use_wrapper: If True, return NuPlanMapWrapper (has load_vector_layer by name). Else NuPlanMap.
    :return: NuPlanMapWrapper or NuPlanMap.
    """
    maps_db = get_maps_db(map_root, map_version)
    if use_wrapper:
        from nuplan.database.maps_db.map_api import NuPlanMapWrapper
        return NuPlanMapWrapper(maps_db, map_name)
    from nuplan.common.maps.nuplan_map.map_factory import NuPlanMap
    return NuPlanMap(maps_db, map_name.replace(".gpkg", ""))


def get_locations(map_root: str = DEFAULT_MAP_ROOT, map_version: str = DEFAULT_MAP_VERSION) -> List[str]:
    """Get list of available map location names."""
    maps_db = get_maps_db(map_root, map_version)
    return list(maps_db.get_locations())


def get_vector_layer_names(map_api) -> List[str]:
    """Get list of available vector layer names for this map."""
    layers = getattr(map_api, "available_vector_layers", None)
    if layers is None:
        return []
    # May be numpy array (avoid truth test) or list; pyogrio returns (name, geom_type) per row
    layers = list(layers)
    if not layers:
        return []
    first = layers[0]
    if isinstance(first, (list, tuple)):
        return [str(item[0]) for item in layers]
    # Numpy array of shape (N, 2): each row is [name, geom_type]
    if hasattr(first, "__len__") and not isinstance(first, (str, bytes)) and len(first) >= 1:
        return [str(item[0]) for item in layers]
    return [str(x) for x in layers]


def load_vector_layer(map_api, layer_name: str):
    """
    Load a vector layer as GeoDataFrame.
    Dependencies: map must be NuPlanMapWrapper (from get_map(use_wrapper=True)).
    """
    return map_api.load_vector_layer(layer_name)


def get_all_lanes_polygons(map_api) -> List[Dict[str, Any]]:
    """Get all lane polygon records (list of row dicts)."""
    gdf = load_vector_layer(map_api, "lanes_polygons")
    return _gdf_to_records(gdf)


def get_all_lane_connectors(map_api) -> List[Dict[str, Any]]:
    """Get all lane connector records (line layer)."""
    gdf = load_vector_layer(map_api, "lane_connectors")
    return _gdf_to_records(gdf)


def get_all_intersections(map_api) -> List[Dict[str, Any]]:
    """Get all intersection polygon records."""
    gdf = load_vector_layer(map_api, "intersections")
    return _gdf_to_records(gdf)


def get_all_crosswalks(map_api) -> List[Dict[str, Any]]:
    """Get all crosswalk polygon records."""
    gdf = load_vector_layer(map_api, "crosswalks")
    return _gdf_to_records(gdf)


def get_all_walkways(map_api) -> List[Dict[str, Any]]:
    """Get all walkway polygon records."""
    gdf = load_vector_layer(map_api, "walkways")
    return _gdf_to_records(gdf)


def get_all_stop_polygons(map_api) -> List[Dict[str, Any]]:
    """Get all stop polygon records."""
    gdf = load_vector_layer(map_api, "stop_polygons")
    return _gdf_to_records(gdf)


def get_all_carpark_areas(map_api) -> List[Dict[str, Any]]:
    """Get all carpark area polygon records."""
    gdf = load_vector_layer(map_api, "carpark_areas")
    return _gdf_to_records(gdf)


def get_all_lane_groups_polygons(map_api) -> List[Dict[str, Any]]:
    """Get all lane group polygon records."""
    gdf = load_vector_layer(map_api, "lane_groups_polygons")
    return _gdf_to_records(gdf)


def get_all_lane_group_connectors(map_api) -> List[Dict[str, Any]]:
    """Get all lane group connector (line) records."""
    gdf = load_vector_layer(map_api, "lane_group_connectors")
    return _gdf_to_records(gdf)


def get_all_boundaries(map_api) -> List[Dict[str, Any]]:
    """Get all boundary line records."""
    gdf = load_vector_layer(map_api, "boundaries")
    return _gdf_to_records(gdf)


def get_all_traffic_lights(map_api) -> List[Dict[str, Any]]:
    """Get all traffic light point records."""
    gdf = load_vector_layer(map_api, "traffic_lights")
    return _gdf_to_records(gdf)


def get_all_generic_drivable_areas(map_api) -> List[Dict[str, Any]]:
    """Get all generic drivable area polygon records."""
    gdf = load_vector_layer(map_api, "generic_drivable_areas")
    return _gdf_to_records(gdf)


def get_all_road_segments(map_api) -> List[Dict[str, Any]]:
    """Get all road segment polygon records."""
    gdf = load_vector_layer(map_api, "road_segments")
    return _gdf_to_records(gdf)


def get_bounds(map_api, layer_name: str, tokens: Optional[List[str]] = None) -> Tuple[float, float, float, float]:
    """Get bounds (min_x, min_y, max_x, max_y) for a layer, optionally filtered by tokens."""
    return map_api.get_bounds(layer_name, tokens)


def layers_on_point(
    map_api, x: float, y: float, layer_names: Optional[List[str]] = None
) -> Dict[str, List[str]]:
    """Return which layer records contain the point (x, y)."""
    return map_api.layers_on_point(x, y, layer_names=layer_names)


def records_on_point(map_api, x: float, y: float, layer_name: str) -> List[str]:
    """Return record tokens (fids) of the given layer that contain the point."""
    return map_api.records_on_point(x, y, layer_name)


def get_records_in_patch(
    map_api,
    box_coords: List[float],
    layer_names: Optional[List[str]] = None,
    mode: str = "intersect",
) -> Dict[str, List[str]]:
    """Get record tokens in a rectangular patch [x_min, y_min, x_max, y_max]. mode: 'intersect' or 'within'."""
    return map_api.get_records_in_patch(box_coords, layer_names=layer_names, mode=mode)


def get_map_object(map_api, object_id: str, layer: str):
    """
    Get a single map object by id and semantic layer.
    layer: one of LANE, LANE_CONNECTOR, ROADBLOCK, ROADBLOCK_CONNECTOR, STOP_LINE, CROSSWALK, INTERSECTION, WALKWAYS, CARPARK_AREA.
    Requires NuPlanMap (use get_map(use_wrapper=False)) and SemanticMapLayer.
    """
    from nuplan.common.maps.maps_datatypes import SemanticMapLayer
    layer_enum = getattr(SemanticMapLayer, layer, None)
    if layer_enum is None:
        raise ValueError(f"Unknown layer {layer}. Use SemanticMapLayer name.")
    return map_api.get_map_object(object_id, layer_enum)


def get_map_dimension(map_api) -> Tuple[int, int]:
    """Get map dimension (width, height) in pixels for raster layers."""
    return map_api.get_map_dimension()


def _gdf_to_records(gdf) -> List[Dict[str, Any]]:
    """Convert GeoDataFrame to list of dicts (one per row), geometry as wkt or bounds."""
    if not GEOPANDAS_AVAILABLE or gdf is None or len(gdf) == 0:
        return []
    records = []
    for idx, row in gdf.iterrows():
        d = row.to_dict()
        if "geometry" in d and d["geometry"] is not None:
            geom = d["geometry"]
            d["geometry_bounds"] = geom.bounds if hasattr(geom, "bounds") else None
            d["geometry_wkt"] = geom.wkt if hasattr(geom, "wkt") else str(geom)
        records.append(d)
    return records
