def get_plural_name(name):
    import inflect

    p = inflect.engine()

    return p.plural(name)


def add_feature_layer(layers, singular_feature_name, geometry_type):
    layer_name = get_plural_name(singular_feature_name)

    layers[layer_name] = {
        "geometry": geometry_type,
        "properties": {},
    }

    return layers[layer_name]["properties"]


def add_relationship_layer(layers, singular_feature_name_1, singular_feature_name_2, layer_name=None):
    if not layer_name:
        plural_feature_name_1 = get_plural_name(singular_feature_name_1)
        plural_feature_name_2 = get_plural_name(singular_feature_name_2)
        layer_name = f"{plural_feature_name_1}_{plural_feature_name_2}"

    layers[layer_name] = {
        "geometry": "None",
        "properties": {f"{singular_feature_name_1}_id": "int64", f"{singular_feature_name_2}_id": "int64",},
    }

    return layers[layer_name]["properties"]


def get_autoware_vector_map_layers():
    layers = {}

    # Feature
    p = add_feature_layer(layers, "intersection_area", "Polygon")

    p = add_feature_layer(layers, "lane_section", "Polygon")

    p = add_feature_layer(layers, "lane", "LineString")
    p["lane_section_id"] = "int64"
    p["width"] = "float"
    p["can_left_lane_change"] = "bool"
    p["can_right_lane_change"] = "bool"
    p["is_intersection"] = "bool"
    p["is_left_turn"] = "bool"
    p["is_right_turn"] = "bool"

    p = add_feature_layer(layers, "lane_boundary", "LineString")
    p["width"] = "float"

    p = add_feature_layer(layers, "lane_divider_line", "LineString")
    p["width"] = "float"

    p = add_feature_layer(layers, "road_edge", "LineString")

    p = add_feature_layer(layers, "stop_line", "LineString")
    p["is_reason_forced"] = "bool"
    p["is_reason_crosswalk"] = "bool"
    p["is_reason_traffic_light"] = "bool"
    p["is_reason_standby"] = "bool"
    p["is_reason_virtual"] = "bool"

    p = add_feature_layer(layers, "crosswalk", "LineString")
    p["width"] = "float"

    p = add_feature_layer(layers, "traffic_light", "LineString")
    p["code"] = "str"

    # Relationship
    p = add_relationship_layer(layers, "lane_section", "next_lane_section", "lane_section_connections")

    p = add_relationship_layer(layers, "lane", "next_lane", "lane_connections")

    p = add_relationship_layer(layers, "lane", "adjacent_lane", "adjacent_lanes")
    p["type"] = "str"

    p = add_relationship_layer(layers, "lane", "lane_boundary")
    p["type"] = "str"

    p = add_relationship_layer(layers, "lane", "stop_line")

    p = add_relationship_layer(layers, "lane", "crosswalk")

    p = add_relationship_layer(layers, "lane", "traffic_light")

    return layers
