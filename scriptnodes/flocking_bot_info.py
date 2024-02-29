'''
 flocking_bot.py

 - NASA Minds Flocking Eagles Swarm Bot Framework

'''
import numpy as np
import omni.graph.core as og
from omni.isaac.core.articulations import Articulation

from omni.isaac.sensor import Camera
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils


# class FlockingBotState:
#     """For maintaining the FlockingBot Articulation object"""

#     def __init__(self):
#         self.flockingbot = FlockingBot.flockingbot

#     def update_state(self):
#         self.flockingbot = 

# class FlockingBot:
# flockingbot = None

    # @staticmethod
    # def internal_state():
    #     return FlockingBotState()

    #__donotaccess = 3.14159265

    # def __init__(self, model_path: string):
    # 	self.model_path = model_path
    # 	self.id = model_path.split('/')[-1]

def euler_x_from_quaternion(x, y, z, w):
    roll_x = np.arctan2(
        2.0 * (w * x + y * z), 
        1.0 - 2.0 * (x * x + y * y)
    )
    
    return roll_x

def setup(db: og.Database):
    pass

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):

    # print(db.per_instance_state)
    # @flockingbot is out of scope for the static function. find a persistent-state variable for the node
    # https://docs.omniverse.nvidia.com/kit/docs/omni.graph.tutorials/1.18.0/tutorial17.html
    # (not as good) https://docs.omniverse.nvidia.com/kit/docs/omni.graph.tutorials/1.18.0/tutorial13.html
    #               https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/howto/RuntimeInitialize.html
    # if not db.state.initialized:
    #     flockingbot = Articulation(prim_path=str(db.inputs.model_path[0]))
    #     flockingbot.initialize()
    #     db.state.initialized = True
    # else:
    #     db.outputs.position_xyz = flockingbot.get_world_pose()[0]
    #     db.outputs.orientation_quaternion = flockingbot.get_world_pose()[1]
    #     db.outputs.linear_velocity = -1.
    #     db.outputs.angular_velocity = -1.

    flockingbot = Articulation(prim_path=str(db.inputs.model_path[0]))
    flockingbot.initialize()
    db.outputs.position_XYZ = flockingbot.get_world_pose()[0]
    db.outputs.orientation_quaternion_WXYZ = flockingbot.get_world_pose()[1]
    db.outputs.bot_orientation = euler_x_from_quaternion(
        db.outputs.orientation_quaternion_WXYZ[0],
        db.outputs.orientation_quaternion_WXYZ[1],
        db.outputs.orientation_quaternion_WXYZ[2],
        db.outputs.orientation_quaternion_WXYZ[3],
    )
    db.outputs.linear_velocity = -1.
    db.outputs.angular_velocity = -1.

    del flockingbot

    return True
