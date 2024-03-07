class args:
    SIM_HEADLESS = False
    TERRAIN_SIZE = 10.0
    TERRAIN_POPULATION = 300
    BOT_SPAWN_RANGE = min(TERRAIN_SIZE, 5.0)
    BOT_POPULATION = 4

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": args.SIM_HEADLESS})
import carb
import numpy as np
from omni.isaac.cloner import GridCloner
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.prims.rigid_prim_view import RigidPrimView
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
import omni.replicator.core as rep
import omni.replicator.isaac as dr	# set up randomization with omni.replicator.isaac, imported as dr
import omni.isaac.core.utils.prims as prim_utils


""" Environment Generation
"""
world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")
prim_utils.create_prim("/World/Light", "DistantLight", position=np.array([1.0, 1.0, 1.0]), attributes={"inputs:intensity": 1500})		# to see

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder, closing app..")
    simulation_app.close()
usd_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
add_reference_to_stage(usd_path=usd_path, prim_path="/World/defaultGroundPlane")

# Set up environment cloner
cloner = GridCloner(spacing=1.5)
cloner.define_base_env("/World/envs")
define_prim("/World/envs/env_0")

# First environment
for i in range(args.TERRAIN_POPULATION):
    FixedCuboid(
        prim_path=f'/World/terrain/object_{i}', 
        size=np.random.uniform(0.2, 0.4), 
        position=np.array([
            (np.random.beta(.6, .6) - .5) * args.TERRAIN_SIZE * 2,
            (np.random.beta(.6, .6) - .5) * args.TERRAIN_SIZE * 2,
            0.1
        ]))
add_reference_to_stage(usd_path="/home/kevin/Desktop/flockingeaglesisaacsim/flockingbot_script.usd", prim_path="/World/envs/env_0/flockingbot")

# Cloning
num_envs = args.BOT_POPULATION
prim_paths = cloner.generate_paths("/World/envs/env", num_envs)
env_pos = cloner.clone(source_prim_path="/World/envs/env_0", prim_paths=prim_paths)
flocking_bot_view = ArticulationView(prim_paths_expr="/World/envs/*/flockingbot", name="flocking_bot_view")
world.scene.add(flocking_bot_view)
world.reset()

num_dof = flocking_bot_view.num_dof

# Randomization setup
dr.physics_view.register_simulation_context(world)
dr.physics_view.register_articulation_view(flocking_bot_view)

with dr.trigger.on_rl_frame(num_envs=num_envs):
    with dr.gate.on_env_reset():
        dr.physics_view.randomize_articulation_view(
            view_name=flocking_bot_view.name,
            operation="additive",
            orientation=rep.distribution.uniform((0.0, 0.0, 0.0), (0.0, 0.0, 360.0)),
            position=rep.distribution.uniform((-1 * args.BOT_SPAWN_RANGE, -1 * args.BOT_SPAWN_RANGE, 0.1), (args.BOT_SPAWN_RANGE, args.BOT_SPAWN_RANGE, 0.1)),
        )


# TODO: Control flocking bot by script
""" Flocking Bot Script Control
"""
# set the FlockingBot_ScriptControl Main Controller node to a path
# somehow get IR, orientation, and position data out? use that data and create a map, outside of the controller
# output random linear and angular velocities

# end goal is to create a shit load of maps


""" Run Simulation
"""
frame_idx = 0
while simulation_app.is_running():
    if world.is_playing():
        reset_inds = list()
        if frame_idx % 200 == 0:
            # triggers reset every 400 steps
            reset_inds = np.arange(num_envs)
        dr.physics_view.step_randomization(reset_inds)
        world.step(render=True)
        frame_idx += 1

simulation_app.close()

