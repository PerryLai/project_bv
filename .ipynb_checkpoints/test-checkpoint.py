import carla
import random
import os
from image_converter import depth_to_local_point_cloud, to_rgb_array
import carla
import os
from util import install_camera_set
import time

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
cars = [a for a in vehicle_blueprints if a.get_attribute('number_of_wheels').as_int() == 4]
print(len(cars))

bikes = [a for a in vehicle_blueprints if a.get_attribute('number_of_wheels').as_int() == 2]
print(len(bikes))

walker_blueprints = world.get_blueprint_library().filter('*walker*')
print(len(walker_blueprints))