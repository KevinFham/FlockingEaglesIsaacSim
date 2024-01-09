'''
 calculate_heading.py

 - Calculate the heading based on the robot's current position, orientation, and the destination's position

'''
def normalize_angle(angle):
    return angle + np.pi * 2 if angle < 0.0 else angle


import numpy as np
import omni.graph.core as og

def setup(db: og.Database):
    pass

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):

    position_diff = np.subtract(db.inputs.bot_position, db.inputs.destination_position)

    # https://stackoverflow.com/questions/19058764/calculate-heading-angle-from-x-and-y-information
    heading_angle = np.fmod( -np.arctan2(position_diff[1], position_diff[0]), np.pi * 2)

    # heading_angle = -1.0 * np.arctan(position_diff[1] / position_diff[0])
    # if position_diff[0] < 0:
    #     if heading_angle > 0:
    #         heading_angle -= np.pi
    #     else:
    #         heading_angle += np.pi

    db.outputs.debug = normalize_angle( np.fmod( heading_angle - db.inputs.bot_orientation + np.pi, np.pi * 2) )

    return True
