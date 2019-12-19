from autoware_vector_map import map_util


def create_lane_section(coordinates):
    return {"geometry": {"type": "Polygon", "coordinates": coordinates}, "properties": {}}


def create_lane_sections(map_api):
    lanes = map_api.get_all_features_as_gdf("lanes")
    all_lane_section_id_set = {lane.lane_section_id for lane in lanes.itertuples()}

    lane_sections = []
    for lane_section_id in all_lane_section_id_set:
        lanes_in_section = list(lanes[lanes.lane_section_id == lane_section_id].itertuples())

        left_most_lane = map_api.find_edge_lane(lanes_in_section, "left")
        right_most_lane = map_api.find_edge_lane(lanes_in_section, "right")

        # Set offset distance
        margin = 0.5
        left_offset = margin + left_most_lane.width / 2
        right_offset = margin + right_most_lane.width / 2

        # Shift geometry using width
        left_geometry = map_util.parallel_offset_wrapper(left_most_lane.geometry, left_offset, "left")
        right_geometry = map_util.parallel_offset_wrapper(right_most_lane.geometry, right_offset, "right")

        # Create additional points, mainly for intersections
        if len(lanes_in_section) == 1:
            start_additional_points = []
            end_additional_points = []
        else:
            start_additional_points = [
                map_util.parallel_offset_wrapper(left_most_lane.geometry, left_offset, "right").coords[0],
                map_util.parallel_offset_wrapper(right_most_lane.geometry, right_offset, "left").coords[0],
            ]
            end_additional_points = [
                map_util.parallel_offset_wrapper(right_most_lane.geometry, right_offset, "left").coords[-1],
                map_util.parallel_offset_wrapper(left_most_lane.geometry, left_offset, "right").coords[-1],
            ]

        exterior = [
            [left_geometry.coords[0]]
            + start_additional_points
            + list(right_geometry.coords)
            + end_additional_points
            + list(reversed(left_geometry.coords))
        ]

        interiors = []

        coordinates = exterior + interiors

        lane_sections.append(create_lane_section(coordinates))

    return lane_sections
