from autoware_vector_map import map_util


def create_adjacent_lane(lane_id, adjacent_lane_id, left_right):
    return {
        "geometry": None,
        "properties": {"lane_id": lane_id, "adjacent_lane_id": adjacent_lane_id, "type": left_right},
    }


def create_adjacent_lanes(map_api):
    lanes = map_api.get_all_features_as_gdf("lanes")
    all_lane_section_id_set = {lane.lane_section_id for lane in lanes.itertuples()}

    adjacent_lanes = []
    for lane_section_id in all_lane_section_id_set:
        lanes_in_section = map_api.get_lanes_by_lane_section_id(lane_section_id)

        if len(lanes_in_section) < 2:
            continue

        for lane in lanes_in_section:
            left_lane = map_util.find_adjacent_lane(lanes_in_section, lane, "left")
            right_lane = map_util.find_adjacent_lane(lanes_in_section, lane, "right")

            if left_lane:
                adjacent_lanes.append(create_adjacent_lane(lane.id, left_lane.id, "left"))
            if right_lane:
                adjacent_lanes.append(create_adjacent_lane(lane.id, right_lane.id, "right"))

    return adjacent_lanes
