# run_franka_standalone.py
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
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z) #sets the stage up axis to Z (default is Y) and tokens is just a string enum type used in USD API calls
UsdGeom.SetStageMetersPerUnit(stage, 1.0) #sets the stage scale to meters (1.0 = 1 meter, 0.01 = 1 centimeter, etc)

distant = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Light"))
distant.CreateIntensityAttr(300)
GroundPlane(prim_path="/World/GroundPlane", z_position=0.0)

# --- Load Franka (same asset used in the Basic Robot Tutorial) ---
assets_root = get_assets_root_path()
franka_usd = assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
arm_path = "/World/Arm"
add_reference_to_stage(franka_usd, arm_path)

arm = Articulation(prim_paths_expr=arm_path, name="Arm")
arm.set_world_poses(positions=np.array([[0.0, -1.0, 0.0]]))

# Create the World AFTER content is on stage
world = World(stage_units_in_meters=1.0)
world.reset()#world reset is used to initialize the world and its content

# Target pose from the tutorial vibe
target_pose = [-1.5, 0.0, 0.0, -1.5, 0.0, 1.5, 0.5, 0.04, 0.04]
zero_pose   = [ 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.00, 0.00]
arm.set_joint_positions([target_pose])


# --- Main loop keeps the app alive until you close the window ---
while simulation_app.is_running():          #################################### <-- critical line ########################################################
    world.step(render=True)

# Only in standalone scripts you own:
simulation_app.close()
