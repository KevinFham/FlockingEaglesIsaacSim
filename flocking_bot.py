'''
 flocking_bot.py

 - NASA Minds Flocking Eagles Swarm Bot Framework

'''
from omni.isaac.sensor import Camera
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils


class FlockingBot:

	__donotaccess = 3.14159265

	def __init__(self, id):
		self.id = id
