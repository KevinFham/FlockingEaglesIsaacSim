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
import matplotlib.pyplot as plt
from math import isclose, ceil
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

#for i, (x, y) in enumerate([(0,1), (0,-1), (1,0), (-1,0)]):
'''for i, (x, y) in enumerate([(0,1), (0,-1)]):
    FixedCuboid(
        prim_path=f'/World/terrain/object_{i}', 
        size=10,
        position=np.array([
            5.5 * x,
            5.5 * y,
            0.1
        ])
    )'''

# Spawn robot
add_reference_to_stage(usd_path="/home/kevin/Desktop/flockingeaglesisaacsim/flockingbot8IR_script.usd", prim_path=args.FLOCKINGBOT_ASSET_DIR)
#add_reference_to_stage(usd_path="/home/kevin/Desktop/flockingeaglesisaacsim/flockingbot_script.usd", prim_path=args.FLOCKINGBOT_ASSET_DIR)
diff = DifferentialController(name="flockingbot_diff", wheel_radius=0.03, wheel_base=0.1125)
robot = WheeledRobot(
    prim_path=args.FLOCKINGBOT_ASSET_DIR, 
    usd_path="/home/kevin/Desktop/flockingeaglesisaacsim/flockingbot8IR_script.usd", 
    #usd_path="/home/kevin/Desktop/flockingeaglesisaacsim/flockingbot_script.usd", 
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"], 
    wheel_dof_indices=[0,1], 
    create_robot=True,
    position=np.array([spawns[-1][0][0] / args.GRAIN, spawns[-1][0][1] / args.GRAIN, 0.1])		#np.array([0,0,0])
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
        self.mapping_protocol = False
        self.ir_count = 8								# 3
        self.ir_configuration = list(np.linspace(-0.5 * np.pi, 1.25 * np.pi, num=8)) 	# np.linspace(-0.25 * np.pi, 0.25 * np.pi, num=3)
        self.ir_map_size = 20 * args.GRAIN
        self.ir_map = np.full((self.ir_map_size, self.ir_map_size), 0, dtype=np.uint8)
        self.ir_map_origin = [0., 0.]
        self.ir_max_range = 10.0
    def clamp(self, value, minim=0, maxim=None): 
        if not maxim: maxim = self.ir_map_size - 1
        return max(minim, min(value, maxim))
    def normalize_angle(self, angle): return angle + np.pi * 2 if angle < 0.0 else angle
    def quaternion_to_euler_x(self, quat):
        eul_x = np.arctan2( 2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]) )
        return eul_x
    def get_position(self): return self.robot.get_world_pose()[0]
    def get_orientation_quat(self): return self.imu.get_current_frame()['orientation']
    def get_orientation_euler_x(self): return self.quaternion_to_euler_x(self.get_orientation_quat())
    def get_ir_reading(self, ir): return acquire_ultrasonic_sensor_interface().get_linear_depth_data(args.FLOCKINGBOT_ASSET_DIR + f'/chassis/US_sensors/UltrasonicArray_{ir}', 0)[0, 0] * args.GRAIN
    def get_all_ir_readings(self): return [self.get_ir_reading(i) for i in range(self.ir_count)]
    def get_ir_map(self): return self.ir_map
    def set_mapping_enable(self, enable: bool): self.mapping_protocol = enable
    def set_ir_map_origin(self, new_pos): self.ir_map_origin[0], self.ir_map_origin[1] = new_pos[0] * args.GRAIN, new_pos[1] * args.GRAIN
    def map_ir_readings(self):
        position = self.get_position() * args.GRAIN
        position[0], position[1] = ceil(self.ir_map_size / 2 + self.ir_map_origin[0] - position[0]), ceil(self.ir_map_size / 2 + self.ir_map_origin[1] - position[1])
        orient = self.get_orientation_euler_x() + np.pi / 2
        for ir, angle_offset in zip(self.get_all_ir_readings() * args.GRAIN, self.ir_configuration):
            if self.ir_max_range - ir > 0.0:
                x = ceil(ir * np.sin(orient + angle_offset) + position[0])
                y = ceil(ir * np.cos(orient + angle_offset) + position[1])
                if 0 <= x < self.ir_map_size and 0 <= y < self.ir_map_size: ir_map[x][y] = 1
    def reset_map(self): self.ir_map = np.full((self.ir_map_size, self.ir_map_size), 0, dtype=np.uint8)
    def get_heading_angle_rad(self, dest_pos):
        position_diff = np.subtract(self.get_position(), dest_pos)
        heading_angle = np.fmod(-np.arctan2(position_diff[1], position_diff[0]), np.pi * 2)
        return self.normalize_angle(np.fmod(heading_angle - self.get_orientation_euler_x() + np.pi, np.pi * 2))
    def get_distance_to_destination(self, dest_pos):
        position_diff = np.subtract(self.get_position(), dest_pos)
        return np.sqrt(np.power(position_diff[0], 2) + np.power(position_diff[1], 2))
    def forward(self, speed, rotation): self.robot.apply_action(self.diff.forward([speed, rotation]))			# +rotation = left, -rotation = right
    def go_to_position(self, pos, speed_factor=1.0):
        heading = self.get_heading_angle_rad(pos) - np.pi
        self.forward(
            0.1 * speed_factor * (np.pi - abs(heading / np.pi)) / np.pi, 
            -0.75 * speed_factor * heading
        )
        


""" Run Simulation
"""
route = np.load(args.DATA_DIR + f'route{args.USE_MAP_N}.npz')['path']
route = [np.append(p, [0.]) / args.GRAIN for p in route]		# Adding a Z dimension and dividing by the grain
route_idx = 0
flockingbot = FlockingBot(robot, diff)
flockingbot.set_mapping_enable(True)
flockingbot.set_ir_map_origin(flockingbot.get_position()[:-1])
ir_map = flockingbot.get_ir_map()
plt.ion()

frame_idx = 0
while simulation_app.is_running():
    if world.is_playing():
        # Route travel protocol
        flockingbot.go_to_position(route[route_idx], 0.5)
        if flockingbot.get_distance_to_destination(route[route_idx]) < 0.2:
            route_idx = min(len(route) - 1, route_idx + 1)  
            
        # Mapping protocol
        flockingbot.map_ir_readings()
        if np.array_equal(ir_map, flockingbot.get_ir_map()) and frame_idx % 100 == 0:
            ir_map = flockingbot.get_ir_map()
            plt.imshow(ir_map)
            plt.draw()
            plt.pause(0.0001)
            plt.clf()
        
        world.step(render=True)
        frame_idx += 1

simulation_app.close()



