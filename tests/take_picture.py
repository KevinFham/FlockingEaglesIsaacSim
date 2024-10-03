"""
 take_picture.py
 - Hopefully takes a picture in a currently open simulation
"""
# from omni.isaac.kit import SimulationApp                  # For sim bootup
# simulation_app = SimulationApp({"headless": False})

from omni.isaac.sensor import Camera
from omni.isaac.core import World
import omni.isaac.core.utils.numpy.rotations as rot_utils
import numpy as np
import matplotlib.pyplot as plt
import threading, os

world = World("/World", physics_prim_path="/World/PhysicsScene")

camera = Camera("/World/jetbot/chassis/rgb_camera/jetbot_camera")
camera.initialize()
camera.add_motion_vectors_to_frame()


# if __name__ == '__main__':
# world.initialize_physics()                  # Starts the Sim if it hasnt already (used to work??????)

def camera_loop():
    i = 0
    while world.is_playing():
        # world.step(render=True)
        # world.step_async()
        if i % 500 == 0:
            print("Cam Coords: ", camera.get_world_pose()[0])
            imgplot = plt.imshow(camera.get_rgba()[:, :, :3])
            plt.savefig("/home/kevin/Desktop/flockingeaglesnasaminds/jetbot_cam.png")
            plt.show()
            print("snap")

        i += 1

t1 = threading.Thread(target=camera_loop)
t1.start()
