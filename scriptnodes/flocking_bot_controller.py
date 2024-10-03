'''
 flocking_bot_controller.py

 - NASA Minds Flocking Eagles Swarm Bot Framework

'''
import numpy as np
import omni.graph.core as og
from omni.isaac.core.articulations import Articulation

from omni.isaac.sensor import Camera
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils

def setup(db: og.Database):
    pass

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):

    #db.inputs.bot_orientation = .0 (rad)
    #db.inputs.IR_Sensors = [0, 0, 0]
    #db.inputs.linear_acceleration_vector = [.0, .0, .0]
    
    db.outputs.linear_velocity = -1.
    db.outputs.angular_velocity = -1.

    return True
