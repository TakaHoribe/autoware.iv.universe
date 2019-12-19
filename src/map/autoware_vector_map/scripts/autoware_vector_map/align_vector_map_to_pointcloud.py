import argparse
import shutil
from pathlib import Path

import numpy as np
import open3d as o3d
from shapely.geometry import LinearRing, LineString, Point, Polygon

from autoware_vector_map.map_api import MapApi


def find_nearest_z_value(points, x, y, z):
    # Alias
    xs = points[:, 0]
    ys = points[:, 1]
    zs = points[:, 2]

    # Filter by rectangle
    margin = 5
    idx = np.argwhere((x - margin < xs) & (xs < x + margin) & (y - margin < ys) & (ys < y + margin))

    if not idx.any():
        return None

    new_z = np.min(zs[idx])

    return new_z


def create_new_coord(points, coord):
    x, y, z = (coord[0], coord[1], coord[2])
    new_z = find_nearest_z_value(points, x, y, z)
    return (x, y, new_z)


def create_new_geometry(points, geometry):
    if geometry.geom_type == "Point":
        return create_new_coord(points, geometry.coords[0])

    if geometry.geom_type == "LineString":
        new_coords = []
        for coord in geometry.coords:
            new_coords.append(create_new_coord(points, coord))

        return LineString(new_coords)

    if geometry.geom_type == "LinearRing":
        new_coords = []
        for coord in geometry.coords:
            new_coords.append(create_new_coord(points, coord))

        return LinearRing(new_coords)

    if geometry.geom_type == "Polygon":
        new_exterior = create_new_geometry(points, geometry.exterior).coords

        new_interiors = []
        for interior in geometry.interiors:
            new_interiors.append(create_new_geometry(points, interior).coords)

        return Polygon(new_exterior, new_interiors)


def align_features(map_api, table_name, points):
    gdf = map_api.get_all_features_as_gdf(table_name)

    if not gdf.geometry.any():
        return

    for i, row in gdf.iterrows():
        gdf.loc[i, "geometry"] = create_new_geometry(points, row.geometry)

    map_api.save_gdf(table_name, gdf)


def align_vector_map_to_pointcloud(gpkg_path, pointcloud_path):
    pcd = o3d.io.read_point_cloud(str(pointcloud_path))
    points = np.asarray(pcd.points)

    map_api = MapApi(gpkg_path)

    for table_name in map_api.get_table_names():
        align_features(map_api, table_name, points)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_gpkg_path", type=Path)
    parser.add_argument("input_pointcloud_path", type=Path)
    parser.add_argument("--output-gpkg-path", type=Path)
    ns = parser.parse_args()

    input_gpkg_path = ns.input_gpkg_path
    input_pointcloud_path = ns.input_pointcloud_path

    if ns.output_gpkg_path:
        output_gpkg_path = ns.output_gpkg_path
    else:
        output_gpkg_path = input_gpkg_path.parent / f"{input_gpkg_path.stem}_aligned{input_gpkg_path.suffix}"

    shutil.copy(input_gpkg_path, output_gpkg_path)

    align_vector_map_to_pointcloud(output_gpkg_path, input_pointcloud_path)


if __name__ == "__main__":
    main()
