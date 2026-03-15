from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import time
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import XFormPrim

USD_PATH = "/root/Desktop/demo1.usd"

print("A: opening stage")
open_stage(USD_PATH)

print("B: creating world")
world = World()

# 先让场景加载几帧
for _ in range(10):
    simulation_app.update()

print("C: create wrappers")
robot = Articulation("/World/so101_new_calib")
cube = XFormPrim("/World/Cube")
ee = XFormPrim("/World/so101_new_calib/gripper_link/gripper_frame_link")

print("D: initialize robot")
robot.initialize()

# 再更新几帧
for _ in range(10):
    simulation_app.update()

def get_dist():
    cube_pos, _ = cube.get_world_pose()
    ee_pos, _ = ee.get_world_pose()
    return np.linalg.norm(np.array(ee_pos) - np.array(cube_pos)), ee_pos, cube_pos

print("E: test joint positions")
print("joint_positions =", robot.get_joint_positions())
print("num_dof =", robot.num_dof)

dist, ee_pos, cube_pos = get_dist()
print("Initial dist:", dist)
print("EE pos:", ee_pos)
print("Cube pos:", cube_pos)

for i in range(20):
    pos = robot.get_joint_positions()

    if pos is None:
        print(f"step {i}: joint positions not ready")
        simulation_app.update()
        continue

    pos = pos.copy()

    action = np.zeros_like(pos)
    action[:3] = np.random.uniform(-0.02, 0.02, size=3)

    pos += action
    robot.set_joint_positions(pos)

    for _ in range(3):
        simulation_app.update()

    dist, ee_pos, cube_pos = get_dist()
    print(f"step {i:02d} dist={dist:.6f} ee={ee_pos}")

    time.sleep(0.03)

print("Done.")
simulation_app.close()
