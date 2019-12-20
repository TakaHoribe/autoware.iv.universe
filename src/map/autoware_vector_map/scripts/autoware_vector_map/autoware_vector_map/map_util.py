from shapely.geometry import LinearRing, LineString, Point, Polygon

from . import geo_util


# Create wrapper to avoid the problem written in https://github.com/Toblerity/Shapely/issues/284
def parallel_offset_wrapper(line_string, distance, side):
    offseted = line_string.parallel_offset(distance, side)

    if side == "right":
        offseted = LineString(reversed(offseted.coords))

    z = line_string.coords[0][2]
    coords_3d = [(c[0], c[1], z) for c in offseted.coords]

    return LineString(coords_3d)


def find_adjacent_lane(lanes_in_section, base_lane, left_right):
    base_coords = [Point(c) for c in base_lane.geometry.coords]
    start_azimuth = geo_util.calc_azimuth(base_coords[0], base_coords[1])
    end_azimuth = geo_util.calc_azimuth(base_coords[-2], base_coords[-1])

    # Calculate stats
    stats = []
    for target_lane in lanes_in_section:
        if target_lane.id == base_lane.id:
            continue

        target_coords = [Point(c) for c in target_lane.geometry.coords]

        start_lateral_offset = geo_util.calc_lateral_offset(base_coords[0], target_coords[0], start_azimuth)
        end_lateral_offset = geo_util.calc_lateral_offset(base_coords[-1], target_coords[-1], end_azimuth)

        # To absolute
        if left_right == "right":
            start_lateral_offset = -start_lateral_offset
            end_lateral_offset = -end_lateral_offset

        stats.append(
            {
                "lane": target_lane,
                "start_lateral_offset": start_lateral_offset,
                "end_lateral_offset": end_lateral_offset,
            }
        )

    # Filter by conditions
    th_min_dist = -0.1
    th_max_dist = 20
    candidate_stats = list(
        filter(
            lambda stat: (
                (th_min_dist < stat["start_lateral_offset"] < th_max_dist)
                and (th_min_dist < stat["end_lateral_offset"] < th_max_dist)
            ),
            stats,
        )
    )

    if not candidate_stats:
        return None

    # Sort by score
    sorted_stat = sorted(candidate_stats, key=lambda stat: stat["start_lateral_offset"])

    return sorted_stat[0]["lane"]
