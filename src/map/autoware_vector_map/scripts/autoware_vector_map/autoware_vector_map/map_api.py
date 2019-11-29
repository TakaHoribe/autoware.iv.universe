import logging

import fiona
import geopandas as gpd

from autoware_vector_map.layer import get_autoware_vector_map_layers


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def gdf_read_file_wrapper(input_path, layer_name):
    id_features = []
    with fiona.open(input_path, "r", layer=layer_name) as features:
        for feature in features:
            id_feature = feature
            id_feature["properties"]["id"] = int(id_feature["id"])
            id_features.append(id_feature)

        crs = features.meta["crs"]
        columns = ["id"] + list(features.meta["schema"]["properties"]) + ["geometry"]

    return gpd.GeoDataFrame.from_features(id_features, crs=crs, columns=columns)


class MapApi:
    def __init__(self, gpkg_path):
        self._gpkg_path = gpkg_path
        self._gdf_map = {}
        self._layers = get_autoware_vector_map_layers()

        # Assuming all crs is the same as Lane
        with fiona.open(self._gpkg_path, "r", driver="GPKG", layer="lanes") as f:
            self._crs = f.crs

        self._create_tables_if_not_exist()
        self._fix_schemas()

        self._reload_all()

    def _create_tables_if_not_exist(self):
        existing_layer_names = fiona.listlayers(self._gpkg_path)
        for table_name, schema in self._layers.items():
            if table_name in existing_layer_names:
                continue
            f = fiona.open(self._gpkg_path, "w", driver="GPKG", crs=self._crs, schema=schema, layer=table_name)
            f.close()

    def _fix_schemas(self):
        for table_name, schema in self._layers.items():
            self._fix_schema(table_name, schema)

    def _fix_schema(self, table_name, desired_schema):
        # Load features
        with fiona.open(self._gpkg_path, "r", layer=table_name) as features:
            crs = features.crs
            database_schema = features.schema
            new_features = list(features)

        # Add missing props
        for prop in desired_schema["properties"]:
            if not prop in database_schema["properties"]:
                logger.info(f"add `{prop}` to {table_name}")
                for feature in new_features:
                    feature["properties"][prop] = None

        # Remove invalid props
        for prop in database_schema["properties"]:
            if not prop in desired_schema["properties"]:
                logger.info(f"delete `{prop}` from {table_name}")
                for feature in new_features:
                    del feature["properties"][prop]

        # Save features
        with fiona.open(self._gpkg_path, "w", driver="GPKG", crs=crs, schema=desired_schema, layer=table_name) as f:
            f.writerecords(new_features)

    def _reload_table(self, table_name):
        self._gdf_map[table_name] = gdf_read_file_wrapper(self._gpkg_path, table_name)

    def _reload_all(self):
        for table_name in self._layers.keys():
            self._reload_table(table_name)

    def _check_non_empty(self, table_name):
        if self._gdf_map[table_name].empty:
            msg = f"table `{table_name}` is empty"
            logger.warn(msg)

    def get_table_names(self):
        return self._layers.keys()

    def save_fiona_objects(self, table_name, objs):
        with fiona.open(
            self._gpkg_path, "w", driver="GPKG", crs=self._crs, schema=self._layers[table_name], layer=table_name
        ) as f:
            f.writerecords(objs)

        self._reload_table(table_name)

    def save_gdf(self, table_name, gdf):
        if gdf.empty:
            return

        gdf.to_file(self._gpkg_path, driver="GPKG", layer=table_name)

        self._reload_table(table_name)

    def get_all_features_as_gdf(self, table_name):
        return self._gdf_map[table_name]

    def get_all_features(self, table_name):
        return list(self._gdf_map[table_name].itertuples())

    def get_feature_by_id(self, table_name, id):
        if isinstance(id, gpd.pd.Series):
            ids = id
        else:
            ids = [id]

        features = self.get_features_by_ids(table_name, ids)

        if not features:
            return []

        return features[0]

    def get_features_by_ids(self, table_name, ids):
        self._check_non_empty(table_name)
        gdf = self._gdf_map[table_name]

        if isinstance(ids, gpd.pd.Series) and ids.empty:
            return []

        row = gdf[gdf["id"].isin(ids)]

        if row.empty:
            return []

        return list(row.itertuples())

    def get_lanes_by_lane_section_id(self, lane_section_id):
        self._check_non_empty("lanes")

        gdf = self._gdf_map["lanes"]
        row = gdf[(gdf["lane_section_id"] == lane_section_id)]

        return list(row.itertuples())

    def get_next_lanes_by_id(self, id):
        self._check_non_empty("lane_connections")

        gdf = self._gdf_map["lane_connections"]
        row = gdf[(gdf["lane_id"] == id)]

        return self.get_features_by_ids("lanes", row["next_lane_id"])

    def get_prev_lanes_by_id(self, id):
        self._check_non_empty("lane_connections")

        gdf = self._gdf_map["lane_connections"]
        row = gdf[(gdf["next_lane_id"] == id)]

        return self.get_features_by_ids("lanes", row["lane_id"])

    def get_adjacent_lane_by_id(self, id, left_right):
        self._check_non_empty("adjacent_lanes")

        gdf = self._gdf_map["adjacent_lanes"]
        row = gdf[(gdf["lane_id"] == id) & (gdf["type"] == left_right)]

        return self.get_feature_by_id("lanes", row["adjacent_lane_id"])

    def find_edge_lane(self, lanes_in_section, left_right):
        edge_lane_id = lanes_in_section[0].id

        while True:
            adjacent_lane = self.get_adjacent_lane_by_id(edge_lane_id, left_right)
            if not adjacent_lane:
                break
            else:
                edge_lane_id = adjacent_lane.id

        return self.get_feature_by_id("lanes", edge_lane_id)

