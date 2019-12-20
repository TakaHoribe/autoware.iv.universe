import argparse
from pathlib import Path

from autoware_vector_map.map_api import MapApi


def create_empty_vector_map(gpkg_path, crs):
    map_api = MapApi(gpkg_path)

    for table_name in map_api.get_table_names():
        map_api.create_table(table_name, crs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-path", type=Path, default="./empty_vector_map.gpkg")
    parser.add_argument("--crs", type=str, default="WGS84")
    ns = parser.parse_args()

    output_path = ns.output_path

    if output_path.exists():
        msg = f"{output_path.absolute()} already exists. Please remove it first."
        raise FileExistsError(msg)

    create_empty_vector_map(output_path, ns.crs)


if __name__ == "__main__":
    main()
