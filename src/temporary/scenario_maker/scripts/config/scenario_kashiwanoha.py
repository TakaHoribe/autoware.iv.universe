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


s_initpos = {"x": -139.1, "y": -24.68, "th": 117.2, "ver_sigma": 0.5, "lat_sigma": 0.1, "th_sigma": 1.0}
s_goalpos = {"x": -119.9, "y": -4.945, "th": 117.2, "ver_sigma": 0.5, "lat_sigma": 0.1, "th_sigma": 1.0}

s_checkpoint = [
    {"x": -60.3, "y": -23.6, "th": -62.8, "ver_sigma": 0.0, "lat_sigma": 0.0, "th_sigma": 0.0},
    {"x": 1.4, "y": 5.4, "th": 117.2, "ver_sigma": 0.0, "lat_sigma": 0.0, "th_sigma": 0.0},
    {"x": -83.5, "y": 14.5, "th": 117.2, "ver_sigma": 0.0, "lat_sigma": 0.0, "th_sigma": 0.0},
    {"x": -43.5, "y": 64.2, "th": 27.2, "ver_sigma": 0.0, "lat_sigma": 0.0, "th_sigma": 0.0},
    {"x": -5.8, "y": -19.8, "th": -152.8, "ver_sigma": 0.0, "lat_sigma": 0.0, "th_sigma": 0.0},
    {"x": -119.1, "y": -63.6, "th": 117.2, "ver_sigma": 0.0, "lat_sigma": 0.0, "th_sigma": 0.0},
]

# obstacle0: avoid by lane change
s_staticobstacle = [
    {
        "x": -121.3,
        "y": 25.87,
        "th": 27.2,
        "ver_sigma": 0.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
        "v": 0,
        "v_sigma": 0,
        "obstacle_type": "car",
        "obstacle_id": 0,
    },
    {
        "x": -17.8,
        "y": 51.2,
        "th": -62.8,
        "ver_sigma": 0.5,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
        "v": 0,
        "v_sigma": 0,
        "obstacle_type": "car",
        "obstacle_id": 0,
    },
]

s_dynamicobstacle = [
    {  # obstacle1: obstacle car in lane change
        "x": -77.8,
        "y": 46.8,
        "th": 27.2,
        "ver_sigma": 20.0,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
        "v": 4.0,
        "v_sigma": 2.0,
        "judge_x": -56.7,
        "judge_y": 57.3,
        "judge_th": 27.2,
        "judge_dist_xy": 50.0,
        "judge_dist_th": 100.0,
        "generate_mode_dist": GENERATE_DIST_INAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_ALLWAYS,
        "generate_once": False,
        "generate_loop": 25.0,
        "obstacle_type": "car",
        "obstacle_id": 1,
        "alternate_mode": False,
        "alternate_timing": None,
    },
    {  # obstacle2: sudden pedestrian
        "x": -95.0,
        "y": -67.0,
        "th": 117.2,
        "ver_sigma": 0.1,
        "lat_sigma": 0.1,
        "th_sigma": 1.0,
        "v": 0.5,
        "v_sigma": 0.1,
        "judge_x": -95.0,
        "judge_y": -67.0,
        "judge_th": -152.8,
        "judge_dist_xy": 35.0,
        "judge_dist_th": 45.0,
        "generate_mode_dist": GENERATE_DIST_INAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_ALLWAYS,
        "generate_once": False,
        "generate_loop": 12.0,
        "obstacle_type": "pedestrian",
        "obstacle_id": 2,
        "alternate_mode": False,
        "alternate_timing": None,
    },
    {  # obstacle3: crossing car 1(crossing with traffic light)
        "x": -121.8,
        "y": -25.6,
        "th": 27.2,
        "ver_sigma": 5.0,
        "lat_sigma": 0.1,
        "th_sigma": 0.0,
        "v": 8.0,
        "v_sigma": 0.5,
        "judge_x": -121.8,
        "judge_y": -25.6,
        "judge_th": 27.2,
        "judge_dist_xy": 30.0,
        "judge_dist_th": 30.0,
        "generate_mode_dist": GENERATE_DIST_OUTAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_RED,
        "generate_once": False,
        "generate_loop": 11.0,
        "obstacle_type": "car",
        "obstacle_id": 3,
        "alternate_mode": False,
        "alternate_timing": None,
    },
    {  # obstacle4: crossing car 2(crossing without traffic light)
        "x": -101.6,
        "y": -65.2,
        "th": 27.2,
        "ver_sigma": 5.0,
        "lat_sigma": 0.1,
        "th_sigma": 0.0,
        "v": 8.0,
        "v_sigma": 1.0,
        "judge_x": -101.6,
        "judge_y": -65.2,
        "judge_th": 27.2,
        "judge_dist_xy": 30.0,
        "judge_dist_th": 30.0,
        "generate_mode_dist": GENERATE_DIST_OUTAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_ALLWAYS,
        "generate_once": False,
        "generate_loop": 11.0,
        "obstacle_type": "car",
        "obstacle_id": 4,
        "alternate_mode": False,
        "alternate_timing": None,
    },
    {  # obstacle 5: crossing pedestrian(crossing with traffic light)
        "x": -83.1,
        "y": 0.8,
        "th": 27.2,
        "ver_sigma": 0.5,
        "lat_sigma": 0.5,
        "th_sigma": 1.0,
        "v": 2.0,
        "v_sigma": 0.2,
        "judge_x": -83.8,
        "judge_y": 2.0,
        "judge_th": 27.2,
        "judge_dist_xy": 0.0,
        "judge_dist_th": 0.0,
        "generate_mode_dist": GENERATE_DIST_ALLWAYS,
        "generate_mode_traffic": GENERATE_TRAFFIC_RED,
        "generate_once": False,
        "generate_loop": 14.0,
        "obstacle_type": "pedestrian",
        "obstacle_id": 5,
        "alternate_mode": False,
        "alternate_timing": None,
    },
    {  # obstacle 6: pause and vanish car
        "x": -33.1,
        "y": -33.6,
        "th": -152.8,
        "ver_sigma": 0.1,
        "lat_sigma": 0.1,
        "th_sigma": 2.0,
        "v": 0.0,
        "v_sigma": 0.0,
        "judge_x": -33.1,
        "judge_y": -33.6,
        "judge_th": -152.8,
        "judge_dist_xy": 30.0,
        "judge_dist_th": 45.0,
        "generate_mode_dist": GENERATE_DIST_INAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_ALLWAYS,
        "generate_once": True,
        "generate_loop": 10.0,
        "obstacle_type": "car",
        "obstacle_id": 6,
        "alternate_mode": False,
        "alternate_timing": None,
    },
    {  # obstacle 7: crossing car 3(crossing with traffic light/stop to see red light) (alternative: obstacle 8)
        "x": -121.8,
        "y": -25.6,
        "th": 27.2,
        "ver_sigma": 0.1,
        "lat_sigma": 0.0,
        "th_sigma": 0.0,
        "v": 4.5,
        "v_sigma": 0.01,
        "judge_x": -121.8,
        "judge_y": -25.6,
        "judge_th": 27.2,
        "judge_dist_xy": 30.0,
        "judge_dist_th": 30.0,
        "generate_mode_dist": GENERATE_DIST_OUTAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_GREEN,
        "generate_once": False,
        "generate_loop": 8.0,
        "obstacle_type": "car",
        "obstacle_id": 7,
        "alternate_mode": True,
        "alternate_timing": BEFORE,
    },
    {  # obstacle 8: stop car (alternative: obstacle 7)
        "x": -86.1,
        "y": -7.5,
        "th": 27.2,
        "ver_sigma": 0.5,
        "lat_sigma": 0.1,
        "th_sigma": 0.0,
        "v": 0.0,
        "v_sigma": 0.0,
        "judge_x": -86.1,
        "judge_y": -7.5,
        "judge_th": 27.2,
        "judge_dist_xy": 40.0,
        "judge_dist_th": 30.0,
        "generate_mode_dist": GENERATE_DIST_OUTAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_GREEN,
        "generate_once": False,
        "generate_loop": 8.0,
        "obstacle_type": "car",
        "obstacle_id": 8,
        "alternate_mode": True,
        "alternate_timing": AFTER,
    },
    {  # obstacle 9:crossing pedestrian2(crossing with traffic light)
        "x": -81.8,
        "y": -2.0,
        "th": -62.8,
        "ver_sigma": 0.5,
        "lat_sigma": 0.5,
        "th_sigma": 2.0,
        "v": 2.0,
        "v_sigma": 0.1,
        "judge_x": -81.8,
        "judge_y": -2.0,
        "judge_th": -62.8,
        "judge_dist_xy": 0.0,
        "judge_dist_th": 0.0,
        "generate_mode_dist": GENERATE_DIST_OUTAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_GREEN,
        "generate_once": False,
        "generate_loop": 4.0,
        "obstacle_type": "pedestrian",
        "obstacle_id": 9,
        "alternate_mode": False,
        "alternate_timing": None,
    },
    {  # obstacle 10:crossing pedestrian3(crossing with traffic light)
        "x": -81.8,
        "y": -2.0,
        "th": -62.8,
        "ver_sigma": 0.5,
        "lat_sigma": 0.5,
        "th_sigma": 2.0,
        "v": 3.0,
        "v_sigma": 0.1,
        "judge_x": -81.8,
        "judge_y": -2.0,
        "judge_th": -62.8,
        "judge_dist_xy": 0.0,
        "judge_dist_th": 0.0,
        "generate_mode_dist": GENERATE_DIST_OUTAREA,
        "generate_mode_traffic": GENERATE_TRAFFIC_GREEN,
        "generate_once": False,
        "generate_loop": 3.0,
        "obstacle_type": "pedestrian",
        "obstacle_id": 10,
        "alternate_mode": False,
        "alternate_timing": None,
    },
]
