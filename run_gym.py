class args:
    SIM_HEADLESS = False
    
    SEED = 69
    DATA_DIR = "/home/kevin/Desktop/flockingeaglesisaacsim/data_generation/data/"
    USE_MAP_N = 2
    
    GRAIN = 10
    BOT_POPULATION = 2
    BOT_SPAWN_RANGE = 5.0
    FLOCKINGBOT_ASSET_DIR = "/World/flockingbot"

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": args.SIM_HEADLESS})
import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.wheeled_robots.controllers import DifferentialController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.sensor import IMUSensor
from omni.isaac.range_sensor._range_sensor import acquire_ultrasonic_sensor_interface
from omni.isaac.core.objects import FixedCuboid, VisualCuboid
#from omni.isaac.core.prims.rigid_prim_view import RigidPrimView
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage #, get_current_stage
#import omni.replicator.core as rep
#import omni.replicator.isaac as dr	# set up randomization with omni.replicator.isaac, imported as dr


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


# Spawn obstacles
spawns = np.load(args.DATA_DIR + f'spawns{args.USE_MAP_N}.npy', allow_pickle=True)
for i, spawn in enumerate(spawns[:-2]):
    FixedCuboid(
        prim_path=f'/World/terrain/object_{i}', 
        size=(spawn[1] * 2 + 1) / args.GRAIN, 
        position=np.array([
            spawn[0][0] / args.GRAIN, 
            spawn[0][1] / args.GRAIN, 
            0.1
        ])
    )

# Spawn bases for visual clarity
for i, spawn in enumerate(spawns[-2:]):
    VisualCuboid(
        prim_path=f'/World/terrain/base_{i}', 
        color=np.array([0, 100, 0]),
        size=(spawn[1] * 2 + 1) / args.GRAIN, 
        position=np.array([
            spawn[0][0] / args.GRAIN, 
            spawn[0][1] / args.GRAIN, 
            0.1
        ])
    )

# Spawn robot
add_reference_to_stage(usd_path="/home/kevin/Desktop/flockingeaglesisaacsim/flockingbot_script.usd", prim_path=args.FLOCKINGBOT_ASSET_DIR)
diff = DifferentialController(name="flockingbot_diff", wheel_radius=0.03, wheel_base=0.1125)
robot = WheeledRobot(
    prim_path=args.FLOCKINGBOT_ASSET_DIR, 
    usd_path="/home/kevin/Desktop/flockingeaglesisaacsim/flockingbot_script.usd", 
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"], 
    wheel_dof_indices=[0,1], 
    create_robot=True,
    position=np.array([spawns[-1][0][0] / args.GRAIN, spawns[-1][0][1] / args.GRAIN, 0.1])
)
world.scene.add(robot)
world.reset()



""" Flocking Bot Script Control
"""
class FlockingBot:
    def __init__(self, robot: WheeledRobot, diff: DifferentialController):
        self.robot = robot
        self.robot.initialize()
        self.diff = diff
        self.imu = IMUSensor(prim_path=args.FLOCKINGBOT_ASSET_DIR + "/chassis/Imu_Sensor")
        self.imu.initialize()
    def normalize_angle(self, angle): return angle + np.pi * 2 if angle < 0.0 else angle
    def quaternion_to_euler_x(self, quat):
        eul_x = np.arctan2( 2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]) )
        return eul_x
    def get_position(self): return self.robot.get_world_pose()[0]
    def get_orientation_quat(self): return self.imu.get_current_frame()['orientation']
    def get_orientation_euler_x(self): return self.quaternion_to_euler_x(self.get_orientation_quat())
    def get_ir_reading(self, ir): return acquire_ultrasonic_sensor_interface().get_depth_data(args.FLOCKINGBOT_ASSET_DIR + f'/chassis/US_sensors/UltrasonicArray_{ir}', 0) / 1000
    def get_all_ir_readings(self): return [self.get_ir_reading(0), self.get_ir_reading(1), self.get_ir_reading(2)]
    def get_heading_angle_rad(self, dest_pos):
        position_diff = np.subtract(self.get_position(), dest_pos)
        heading_angle = np.fmod(-np.arctan2(position_diff[1], position_diff[0]), np.pi * 2)
        return self.normalize_angle(np.fmod(heading_angle - self.get_orientation_euler_x() + np.pi, np.pi * 2))
    def get_distance_to_destination(self, dest_pos):
        position_diff = np.subtract(self.get_position(), dest_pos)
        return np.sqrt(np.power(position_diff[0], 2) + np.power(position_diff[1], 2))
    def forward(self, speed, rotation): self.robot.apply_action(self.diff.forward([speed, rotation]))			# +rotation = left, -rotation = right
    def go_to_position(self, pos):
        heading = self.get_heading_angle_rad(pos) - np.pi
        self.forward(
            0.1 * (np.pi - abs(heading / np.pi)) / np.pi, 
            -0.75 * heading
        )
        


""" Run Simulation
"""
route = np.load(args.DATA_DIR + f'route{args.USE_MAP_N}.npz')['path']
route = [np.append(p, [0.]) / args.GRAIN for p in route]		# Adding a Z dimension and dividing by the grain
route_idx = 0
flockingbot = FlockingBot(robot, diff)

frame_idx = 0
while simulation_app.is_running():
    if world.is_playing():
        flockingbot.go_to_position(route[route_idx])
        if flockingbot.get_distance_to_destination(route[route_idx]) < 0.2:
            route_idx = min(len(route) - 1, route_idx + 1)
        print(flockingbot.get_all_ir_readings())
        
        world.step(render=True)
        frame_idx += 1

simulation_app.close()



