import math
import os
import random
import lgsvl
import numpy as np
import pyproj
import transformations as tf
import argparse
import copy

# example python3 lg_generator.py "kashiwanoha" "TierIVLexus" -138.3 -25.9 2


parser = argparse.ArgumentParser()
parser.add_argument("mapname")
parser.add_argument("vehiclename")
parser.add_argument("x")
parser.add_argument("y")
parser.add_argument("theta")
args = parser.parse_args()

map_name = args.mapname
vehicle_name = args.vehiclename
# map_name = "kashiwanoha"
# vehicle_name = "TierIVLexus"

gen_x = float(args.x)
gen_y = float(args.y)
gen_theta = float(args.theta)

sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)


proj_autoware = "+proj=tmerc +lat_0=0 +lon_0=141 +k_0=0.9996 +x_0=100000 +y_0=-3900000 +ellps=WGS84 +units=m +no_defs"
proj_unity = "+proj=tmerc +lat_0=0 +lon_0=141 +k_0=0.9996 +x_0=96259.134544 +y_0=-3973770.93847699 +ellps=WGS84 +units=m +no_defs"
tf_map2map = tf.transformations.translation_matrix((0, 0, 0))  # TODO: rotation​


def vector2array(v):
    return np.array([v.x, v.y, v.z, 1])


def array2vector(a):
    return lgsvl.Vector(a[0], a[1], a[2])


class Transformer:
    def __init__(self, proj_autoware, proj_unity, tf_map2map):
        self.proj_autoware = proj_autoware
        self.proj_unity = proj_unity
        self.tf_map2map = tf_map2map

    def map2map(self, transform):
        new_transform = lgsvl.Transform()

        R = np.linalg.inv(self.tf_map2map)
        t = vector2array(transform.position)
        new_transform.position = array2vector(np.dot(R, t))
        new_transform.rotation = transform.rotation

        return new_transform

    def map2world(self, transform):
        new_transform = lgsvl.Transform()

        R = self.tf_map2map
        t = vector2array(transform.position)

        new_transform.position = array2vector(np.dot(R, t))
        new_transform.rotation = transform.rotation

        return new_transform

    def autoware2unity(self, transform_aw_world):
        transform_aw_map = self.map2map(transform_aw_world)

        x, y = pyproj.transform(
            self.proj_autoware, self.proj_unity, transform_aw_map.position.x, transform_aw_map.position.y
        )

        transform_unity = lgsvl.Transform()
        transform_unity.position.x = x
        transform_unity.position.y = transform_aw_map.position.z
        transform_unity.position.z = y
        transform_unity.rotation.x = -transform_aw_map.rotation.x
        transform_unity.rotation.y = -transform_aw_map.rotation.z
        transform_unity.rotation.z = -transform_aw_map.rotation.y

        return transform_unity

    def unity2autoware(self, transform_unity):
        x, z = pyproj.transform(
            self.proj_unity, self.proj_autoware, transform_unity.position.x, transform_unity.position.z
        )

        transform_aw_map = lgsvl.Transform()
        transform_aw_map.position.x = x
        transform_aw_map.position.y = z
        transform_aw_map.position.z = transform_unity.position.y
        transform_aw_map.rotation.x = -transform_unity.rotation.x
        transform_aw_map.rotation.y = -transform_unity.rotation.z
        transform_aw_map.rotation.z = -transform_unity.rotation.y

        transform_aw_world = self.map2world(transform_aw_map)

        return transform_aw_world


def on_stop_line(agent):
    pass


if sim.current_scene == map_name:
    sim.reset()
else:
    sim.load(map_name)


spawns = sim.get_spawn()
original_state = copy.deepcopy(lgsvl.AgentState())
state = lgsvl.AgentState()

transformer = Transformer(proj_autoware, proj_unity, tf_map2map)

state.transform = transformer.autoware2unity(
    lgsvl.Transform(lgsvl.Vector(gen_x, gen_y, 0.0), lgsvl.Vector(0.0, 0.0, np.rad2deg(gen_theta) - 90))
)
print(state.transform)

forward = lgsvl.utils.transform_to_forward(state.transform)
state.velocity = 0 * forward

## ego
agent = sim.add_agent(vehicle_name, lgsvl.AgentType.EGO, state)
agent.connect_bridge("127.0.0.1", 9090)

"""
## npc
npc_state = lgsvl.AgentState()
sx = original_state.transform.position.x - 20
sy = original_state.transform.position.y
sz = original_state.transform.position.z
point = lgsvl.Vector(sx, sy, sz)
npc_state.transform = sim.map_point_on_lane(point)
npc = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc_state)
npc.follow_closest_lane(True, 5.0)
npc.on_stop_line(on_stop_line)

npc_state2 = lgsvl.AgentState()
sx2 = original_state.transform.position.x + 20
sy2 = original_state.transform.position.y
sz2 = original_state.transform.position.z
point2 = lgsvl.Vector(sx2, sy2, sz2)
npc_state2.transform = sim.map_point_on_lane(point2)
npc2 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc_state2)
npc2.follow_closest_lane(True, 5.0)
npc2.on_stop_line(on_stop_line)
"""

"""
point1 = transformer.autoware2unity(
    lgsvl.Transform(lgsvl.Vector(-68.317, 1.500, 0.0), lgsvl.Vector(0.0, 0.0, np.rad2deg(0.447) - 90))
)

point2 = transformer.autoware2unity(
    lgsvl.Transform(lgsvl.Vector(6.315, 3.400, 0.0), lgsvl.Vector(0.0, 0.0, np.rad2deg(-1.194) - 90))
)

waypoints = [
    lgsvl.DriveWaypoint(lgsvl.Vector(point1.position.x, point1.position.y, point1.position.z), 5,),
    lgsvl.DriveWaypoint(lgsvl.Vector(point2.position.x, point2.position.y, point2.position.z), 5,),
]

npc.follow(waypoints, loop=True)
"""

# signal
"""
# #https://github.com/lgsvl/PythonAPI/blob/master/quickstart/27-control-traffic-lights.py
controllables = sim.get_controllables("signal") #null -> why?
#signal = sim.get_controllable(lgsvl.Vector(19, 5, 21), "signal")
control_policy = "trigger=50;green=3;yellow=2;red=1;loop"
signal.control(control_policy)
"""
sim.run()
