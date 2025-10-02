# run_so100_standalone.py
import numpy as np
from isaacsim.simulation_app import SimulationApp

# Start Kit (GUI)
simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.objects.ground_plane import GroundPlane
from pxr import Sdf, UsdGeom, UsdLux

# --- Stage & lighting ---
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Light")).CreateIntensityAttr(300)
GroundPlane(prim_path="/World/GroundPlane", z_position=0.0)

# --- Load SO100 robot ---
assets_root = get_assets_root_path()
so100_usd = assets_root + "/Isaac/Robots/RobotStudio/so100/so100.usd"
arm_path = "/World/Arm"
add_reference_to_stage(so100_usd, arm_path)

arm = Articulation(prim_paths_expr=arm_path, name="Arm")
arm.set_world_poses(positions=np.array([[0.0, -1.0, 0.0]]))

# Create the World AFTER content is on stage
world = World(stage_units_in_meters=1.0)
world.reset()

# Define poses (radians; list length must match the robot's controllable DOFs)
zero_pose   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
left_pose   = [-1.5, 0.0, 0.0, 0.0, 0.0, 0.0]
right_pose  = [ 1.5, 0.0, 0.0, 0.0, 0.0, 0.0]

def smooth_move(world, arm, q_start, q_goal, duration_s=2.5, sim_hz=60):
    steps = max(2, int(duration_s * sim_hz)) # max between 2 frames and duration*hz
    q_start = np.array(q_start, dtype=float) #q_start represents the starting joint positions of the robot arm
    dq = np.array(q_goal, dtype=float) - q_start
    for k in range(steps):
        s = k / (steps - 1)              # 0..1 -> s is the normalized time variable, ranging from 0 to 1,k is the current step index, and steps is the total number of steps.
        s = s * s * (3 - 2 * s)          # smoothstep easing->3s^2 - 2s^3 which results in a smooth transition from 0 to 1 with zero velocity at both ends.
        q = q_start + s * dq #s is from 0 to 1, so this line interpolates between q_start and q_goal based on the eased value s.
        arm.set_joint_positions([q])
        world.step(render=True)

# --- Main loop keeps the app alive until you close the window ---
while simulation_app.is_running():
    # sweep: zero -> left -> zero -> right -> zero
    q_now = arm.get_joint_positions()[0]
    smooth_move(world, arm, q_now, left_pose,  duration_s=2.0, sim_hz=60)
    smooth_move(world, arm, left_pose, zero_pose, duration_s=1.5, sim_hz=60)
    smooth_move(world, arm, zero_pose, right_pose, duration_s=2.0, sim_hz=60)
    smooth_move(world, arm, right_pose, zero_pose, duration_s=1.5, sim_hz=60)

simulation_app.close()