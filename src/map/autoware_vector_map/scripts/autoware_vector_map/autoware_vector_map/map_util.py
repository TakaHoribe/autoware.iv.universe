from shapely.geometry import LineString


# Create wrapper to avoid the problem written in https://github.com/Toblerity/Shapely/issues/284
def parallel_offset_wrapper(line_string: LineString, distance, side):
    offseted = line_string.parallel_offset(distance, side)

    if side == "right":
        offseted = LineString(reversed(offseted.coords))

    z = line_string.coords[0][2]
    coords_3d = [(c[0], c[1], z) for c in offseted.coords]

    return LineString(coords_3d)
