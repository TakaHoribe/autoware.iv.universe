def create_lane_connection(lane_id, next_lane_id):
    return {"geometry": None, "properties": {"lane_id": lane_id, "next_lane_id": next_lane_id}}


def create_lane_connections(map_api):
    lanes = map_api.get_all_features_as_gdf("lanes")

    lane_connections = []
    for base_lane in lanes.itertuples():
        touch_lanes = lanes[lanes.geometry.touches(base_lane.geometry)]
        for touch_lane in touch_lanes.itertuples():
            if base_lane.geometry.coords[-1] == touch_lane.geometry.coords[0]:
                lane_connections.append(create_lane_connection(base_lane.id, touch_lane.id))

    return lane_connections
