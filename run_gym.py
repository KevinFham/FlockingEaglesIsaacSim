import sys
if len(sys.argv) == 1:
    print(f'{sys.argv[0]} needs an integer argument')
    print(f'Usage: python3 {sys.argv[0]} <num_map>')
    print(f'num_map is the map png, route npz, and spawns npy number that the script pulls from')
    exit()

import numpy as np
from PIL import Image
from time import time_ns
import csv

class args:
    SIM_HEADLESS = False
    
    SEED = 69
    np.random.seed(SEED)
    
    PROJECT_PATH = "/home/kevin/Desktop/flockingeaglesisaacsim"
    DATA_DIR = PROJECT_PATH + "/data_generation/data/"
    USE_MAP_N = sys.argv[1]
    MAP_SIZE = np.asarray(Image.open(DATA_DIR + f'map{USE_MAP_N}.png').convert('RGB'))
    
    GRAIN = 10
    BOT_POPULATION = 1
    BOT_SPAWN_RANGE = 5.0
    FLOCKINGBOT_ASSET_DIR = "/World/flockingbot"
    
class record:
    time_buffer = None
    time_record_file = open(args.PROJECT_PATH + f'/data_generation/time_record{args.USE_MAP_N}.csv', 'w', newline='')
    time_record_writer = csv.writer(time_record_file)
    time_record_writer.writerow(['State', 'Time (ms)'])
    time_record = []
    
    def start_time(state):
        print("Time Start")
        record.time_record = [state]
        record.time_buffer = time_ns() / 1000000
    def record_time():
        time_ms = time_ns() / 1000000
        print(f'{record.time_record}:', time_ms - record.time_buffer)	
        record.time_record_writer.writerow(record.time_record + [str(time_ms - record.time_buffer)])
        record.time_buffer = None	

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": args.SIM_HEADLESS})
import carb
import heapq
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from math import isclose, ceil, sqrt
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
for i, spawn in enumerate(spawns[:-3]):
    FixedCuboid(
        prim_path=f'/World/terrain/object_{i}', 
        size=(spawn[1] * 2 + 1) / args.GRAIN, 
        color=np.array([
            0.9176 + np.random.uniform(-0.04, 0.012), 
            0.4331 + np.random.uniform(-0.02, 0.02), 
            0.0902 + np.random.uniform(-0.01, 0.01)
        ]),
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
        color=np.array([0., 1., 0.]),
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
add_reference_to_stage(usd_path=args.PROJECT_PATH + "/flockingbot8IR_script.usd", prim_path=args.FLOCKINGBOT_ASSET_DIR)
#add_reference_to_stage(usd_path=args.PROJECT_PATH + "/flockingbot_script.usd", prim_path=args.FLOCKINGBOT_ASSET_DIR)
diff = DifferentialController(name="flockingbot_diff", wheel_radius=0.03, wheel_base=0.1125)
robot = WheeledRobot(
    prim_path=args.FLOCKINGBOT_ASSET_DIR, 
    usd_path=args.PROJECT_PATH + "/flockingbot8IR_script.usd", 
    #usd_path=args.PROJECT_PATH + "/flockingbot_script.usd", 
    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"], 
    wheel_dof_indices=[0,1], 
    create_robot=True,
    position=np.array([spawns[-3][0][0] / args.GRAIN, spawns[-3][0][1] / args.GRAIN, 0.1])		#np.array([0,0,0])
)
world.scene.add(robot)
world.reset()



""" Flocking Bot Script Control
"""
class FlockingBot:
    def __init__(self, robot: WheeledRobot, diff: DifferentialController):
        self.ACCESSIBLE_VAL = 2
        self.OBSTACLE_VAL = 1
        self.STATES = ['IDLE', 'RELOCATING', 'GO_A', 'GO_B']

        self.state = 'IDLE'
        self.robot = robot
        self.robot.initialize()
        self.diff = diff
        self.imu = IMUSensor(prim_path=args.FLOCKINGBOT_ASSET_DIR + "/chassis/Imu_Sensor")
        self.imu.initialize()
        self.mapping_protocol = False
        self.global_map = np.zeros((100, 100), dtype=np.uint8)
        for spawn in spawns[:-3]:
            size = spawn[1]
            for i in range(-size, size + 1):
                for j in range(-size, size + 1):
                    self.global_map[max(0, min(spawn[0][0] + i, round(self.global_map.shape[0]) - 1)), max(0, min(spawn[0][1] + j, round(self.global_map.shape[1]) - 1))] = self.OBSTACLE_VAL
        self.global_map_accessible = np.zeros((self.global_map.shape[0], self.global_map.shape[1]), dtype=np.uint8)
        self.global_map_accessible[gaussian_filter(self.global_map.astype('float32'), sigma=1.2) < 0.1] = self.ACCESSIBLE_VAL
        self.route_buffer = []
        self.route_buffer_idx = 0
        self.ir_count = 8								# 3
        self.ir_configuration = list(np.linspace(-0.5 * np.pi, 1.25 * np.pi, num=8)) 	# np.linspace(-0.25 * np.pi, 0.25 * np.pi, num=3)
        self.ir_map_size = 5 * args.GRAIN
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
    def euclidian_distance(self, pos1, pos2):
        return sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
    def dijkstra(self, array2d, start):
        rows, cols = array2d.shape
        visited = np.zeros(array2d.shape, dtype=bool)
        distances = np.full(array2d.shape, np.inf)
        distances[start[0], start[1]] = 0
        priority_queue = [(0, start)]
        while priority_queue:
            current_distance, (current_row, current_col) = heapq.heappop(priority_queue)
            if visited[current_row, current_col]:
                continue
            visited[current_row, current_col] = True
            for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                new_row, new_col = current_row + dr, current_col + dc
                if 0 <= new_row < rows and 0 <= new_col < cols and not visited[new_row, new_col] and self.global_map_accessible[new_row, new_col] == self.ACCESSIBLE_VAL:
                    if array2d[new_row, new_col] != self.OBSTACLE_VAL:
                        distance = current_distance + 1
                        if distance < distances[new_row, new_col]:
                            distances[new_row, new_col] = distance
                            heapq.heappush(priority_queue, (distance, (new_row, new_col)))
        return distances
    def shortest_path(self, array2d, start_point, end_point):
        distances = self.dijkstra(array2d, start_point)
        current_point = end_point
        shortest_path = [current_point]
        next_point = start_point
        interval = 0
        while current_point != start_point:
            min_neighbor_distance = np.inf
            for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                neighbor_row, neighbor_col = current_point[0] + dr, current_point[1] + dc
                if 0 <= neighbor_row < array2d.shape[0] and 0 <= neighbor_col < array2d.shape[1] and distances[neighbor_row, neighbor_col] < min_neighbor_distance:
                    min_neighbor_distance = distances[neighbor_row, neighbor_col]
                    next_point = (neighbor_row, neighbor_col)
            shortest_path.append(next_point)
            current_point = next_point
        return np.array(shortest_path)
    def update_state(self, new_state=None):
        # TODO: expand
        if new_state in self.STATES:
            self.state = new_state
            return
        elif self.state == 'IDLE':
            if record.time_buffer:
                record.record_time()
            if self.get_distance_to_destination(np.append(spawns[-1][0], [0.]) / args.GRAIN) < 0.3:
                self.state = 'GO_A'
                record.start_time(self.state)
                self.route_buffer = np.load(args.DATA_DIR + f'route{args.USE_MAP_N}.npz')['path'] / args.GRAIN
                self.route_buffer_idx = 0
            elif self.get_distance_to_destination(np.append(spawns[-2][0], [0.]) / args.GRAIN) < 0.3:
                self.state = 'GO_B'
                record.start_time(self.state)
                self.route_buffer = np.flip(np.load(args.DATA_DIR + f'route{args.USE_MAP_N}.npz')['path'] / args.GRAIN, axis=0)
                self.route_buffer_idx = 0
            else:
                self.state = 'RELOCATING'
                record.start_time(self.state)
                to_A = self.euclidian_distance(self.get_position(), np.array(spawns[-1][0]) / args.GRAIN)
                to_B = self.euclidian_distance(self.get_position(), np.array(spawns[-2][0]) / args.GRAIN)
                self.route_buffer = self.shortest_path(self.global_map, tuple([self.clamp(ceil(x * args.GRAIN), maxim=self.global_map.shape[0] - 1) for x in self.get_position()[:-1]]), spawns[-1][0] if to_A < to_B else spawns[-2][0]) / args.GRAIN
                self.route_buffer = np.flip(self.route_buffer, axis=0)
                self.route_buffer_idx = 0
                
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
        ir_danger = [self.get_ir_reading(i) / self.ir_max_range for i in range(1, 4)]
        heading = self.get_heading_angle_rad(pos) - np.pi
        self.forward(
            0.1 * speed_factor * (np.pi - abs(heading / np.pi)) / np.pi, 
            -1.0 * speed_factor * heading 
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
        flockingbot.update_state()
        
        # Route travel protocol
        flockingbot.go_to_position(np.append(flockingbot.route_buffer[flockingbot.route_buffer_idx], [0.]), 0.8)
        if flockingbot.get_distance_to_destination(np.append(flockingbot.route_buffer[flockingbot.route_buffer_idx], [0.])) < 0.2:
            flockingbot.route_buffer_idx = min(len(flockingbot.route_buffer) - 1, flockingbot.route_buffer_idx + 1)
            if flockingbot.route_buffer_idx == len(flockingbot.route_buffer) - 1:
                flockingbot.update_state('IDLE')
            
        '''
        # Mapping protocol
        flockingbot.map_ir_readings()
        if np.array_equal(ir_map, flockingbot.get_ir_map()) and frame_idx % 100 == 0:
            ir_map = flockingbot.get_ir_map()
            plt.imshow(ir_map)
            plt.draw()
            plt.pause(0.0001)
            plt.clf()
        '''
        world.step(render=True)
        frame_idx += 1
    '''if frame_idx > 4000:
        np.savez(args.DATA_DIR + f'ir_map{args.USE_MAP_N}.npz', ir_map=ir_map)
        break
    print(frame_idx)'''

simulation_app.close()
record.time_record_file.close()


