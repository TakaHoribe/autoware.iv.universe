import uuid, unique_id

# Define
# for dist judge
GENERATE_DIST_ALLWAYS = 0
GENERATE_DIST_INAREA = 1
GENERATE_DIST_OUTAREA = 2

# for traffic light judge
GENERATE_TRAFFIC_ALLWAYS = 0
GENERATE_TRAFFIC_GREEN = 1
GENERATE_TRAFFIC_RED = 2

# for alternate generate mode
BEFORE = 0
AFTER = 1

# reference_link
REF_LINK = "map"
SELF_LINK = "base_link"


RAD2DEG = 57.2958
OFS_X = 53059.729
OFS_Y = 82526.936
OFS_TH = 0

s_initpos = {
    "x": 37.586 + OFS_X,
    "y": 160.792 + OFS_Y,
    "th": 3.108 * RAD2DEG,
    "ver_sigma": 0.5,
    "lat_sigma": 0.1,
    "th_sigma": 1.0,
}

s_goalpos = {
    "x": 172.443 + OFS_X,
    "y": 367.806 + OFS_Y,
    "th": -0.582 * RAD2DEG,
    "ver_sigma": 0,
    "lat_sigma": 0,
    "th_sigma": 0,
}

s_checkpoint = [
    {
        "x": -149.6 + OFS_X,
        "y": 230.0 + OFS_Y,
        "th": -2.478 * RAD2DEG,
        "ver_sigma": 0.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
    },
    {
        "x": -227.1 + OFS_X,
        "y": 279.4 + OFS_Y,
        "th": 2.529 * RAD2DEG,
        "ver_sigma": 0.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
    },
    {
        "x": -310.7 + OFS_X,
        "y": 336.2 + OFS_Y,
        "th": 2.529 * RAD2DEG,
        "ver_sigma": 0.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
    },
    {
        "x": -426.2 + OFS_X,
        "y": 417.4 + OFS_Y,
        "th": 2.508 * RAD2DEG,
        "ver_sigma": 0.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
    },
    {
        "x": -618.7 + OFS_X,
        "y": 179.4 + OFS_Y,
        "th": -2.106 * RAD2DEG,
        "ver_sigma": 0.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
    },
    {
        "x": -378.2 + OFS_X,
        "y": 279.9 + OFS_Y,
        "th": 1.000 * RAD2DEG,
        "ver_sigma": 0.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
    },
    {
        "x": -256.277 + OFS_X,
        "y": 420.892 + OFS_Y,
        "th": -0.630 * RAD2DEG,
        "ver_sigma": 0.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
    },
    {"x": -27.468 + OFS_X, "y": 279.267 + OFS_Y, "th": 0.908 * RAD2DEG, "ver_sigma": 0, "lat_sigma": 0, "th_sigma": 0,},
]

s_staticobstacle = []

s_dynamicobstacle = []

# traffic light position
s_tlpos = {
    "x": 0,
    "y": 0,
    "th": 0,
    "judge_dist_xy": 10000000,
    "judge_dist_th": 360,
    "turn_traffic_light_th": 0,
}
