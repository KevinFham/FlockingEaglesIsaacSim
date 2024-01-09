'''
 calculate_heading.py

 - Calculate the heading based on the robot's current position, orientation, and the destination's position

'''

import numpy as np
import omni.graph.core as og

def setup(db: og.Database):
    pass

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):

    position_diff = np.subtract(db.inputs.bot_position, db.inputs.destination_position)

    heading_angle = -1.0 * np.arctan(position_diff[1] / position_diff[0])
    if position_diff[0] < 0:
        if heading_angle > 0:
            heading_angle -= np.pi
        else:
            heading_angle += np.pi

    db.outputs.debug = max(min(heading_angle - db.inputs.bot_orientation, 3.14), -3.14)     #TODO: figure out the dumb maath problems of using 180 to -180 range instead of 0 to 360. fuck

    return True
