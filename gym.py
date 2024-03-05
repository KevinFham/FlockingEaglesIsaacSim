from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
import carb
import numpy as np
from omni.isaac.cloner import GridCloner
from omni.isaac.core import World
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims.rigid_prim_view import RigidPrimView
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage

# create the world
world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder, closing app..")
    simulation_app.close()
usd_path = assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
add_reference_to_stage(usd_path=usd_path, prim_path="/World/defaultGroundPlane")

# set up grid cloner
cloner = GridCloner(spacing=1.5)
cloner.define_base_env("/World/envs")
define_prim("/World/envs/env_0")

# set up the first environment
add_reference_to_stage(
    usd_path="/home/kevin/Desktop/flockingeaglesisaacsim/flockingbot_script.usd", prim_path="/World/envs/env_0/flockingbot"
)

# clone environments
num_envs = 4
prim_paths = cloner.generate_paths("/World/envs/env", num_envs)
env_pos = cloner.clone(source_prim_path="/World/envs/env_0", prim_paths=prim_paths)

flocking_bot_view = ArticulationView(prim_paths_expr="/World/envs/*/flockingbot", name="flocking_bot_view")
world.scene.add(flocking_bot_view)

# creates the views and set up world
for i in range(10):
    DynamicCuboid(prim_path=f'/World/object_{i}', size=0.2, position=np.array([0.0, 0.0, 0.0]))
object_view = RigidPrimView(prim_paths_expr="/World/object_*", name="object_view")
world.scene.add(object_view)

world.reset()

num_dof = flocking_bot_view.num_dof

import omni.replicator.core as rep

# set up randomization with omni.replicator.isaac, imported as dr
import omni.replicator.isaac as dr

dr.physics_view.register_simulation_context(world)
dr.physics_view.register_rigid_prim_view(object_view)
dr.physics_view.register_articulation_view(flocking_bot_view)

with dr.trigger.on_rl_frame(num_envs=num_envs):
    with dr.gate.on_env_reset():
        dr.physics_view.randomize_rigid_prim_view(
            view_name=object_view.name,
            operation="additive",
            orientation=rep.distribution.uniform((0.0, 0.0, 0.0), (0.0, 0.0, 360.0)),
            position=rep.distribution.normal((-3.0, 0.1, -3.0), (3.0, 0.1, 3.0)),
            velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        dr.physics_view.randomize_articulation_view(
            view_name=flocking_bot_view.name,
            operation="additive",
            orientation=rep.distribution.uniform((0.0, 0.0, 0.0), (0.0, 0.0, 360.0)),
            position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
        )


# TODO: Control flocking bot by script


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
