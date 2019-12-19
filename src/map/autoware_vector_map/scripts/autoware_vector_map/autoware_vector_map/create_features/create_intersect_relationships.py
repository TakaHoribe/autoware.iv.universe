from autoware_vector_map.layer import get_plural_name


def create_intersect_relationship(base_feature_name, target_feature_name, base_feature, target_feature):
    return {
        "geometry": None,
        "properties": {f"{base_feature_name}_id": base_feature.id, f"{target_feature_name}_id": target_feature.id,},
    }


def create_intersect_relationships(map_api, base_feature_name, target_feature_name):
    base_features = map_api.get_all_features_as_gdf(get_plural_name(base_feature_name))
    target_features = map_api.get_all_features_as_gdf(get_plural_name(target_feature_name))

    intersect_relationships = []
    for target_feature in target_features.itertuples():
        intersect_base_features = base_features[base_features.geometry.intersects(target_feature.geometry)]
        for base_feature in intersect_base_features.itertuples():
            intersect_relationships.append(
                create_intersect_relationship(base_feature_name, target_feature_name, base_feature, target_feature)
            )

    return intersect_relationships
