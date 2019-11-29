import argparse
from pathlib import Path

import fiona

from autoware_vector_map.layer import get_autoware_vector_map_layers


def create_empty_vector_map(output_path, crs):
    for layer_name, schema in get_autoware_vector_map_layers().items():
        f = fiona.open(output_path, "w", driver="GPKG", layer=layer_name, crs=crs, schema=schema)
        f.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-path", type=Path, default="./empty_vector_map.gpkg")
    parser.add_argument("--crs", type=str, default="WGS84")
    ns = parser.parse_args()

    create_empty_vector_map(ns.output_path, ns.crs)


if __name__ == "__main__":
    main()
