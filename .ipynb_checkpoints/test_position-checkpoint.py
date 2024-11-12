import carla
import random
import os
# from image_converter import depth_to_local_point_cloud, to_rgb_array
import carla
import os
from util import install_camera_set
import time

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

spawn_points = world.get_map().get_spawn_points()

vehicles = world.get_blueprint_library().filter('*vehicle*')

print([vehicle.id for vehicle in vehicles if int(vehicle.get_attribute('number_of_wheels')) == 2])


# ego_vehicle_spawn_points = random.choice(spawn_points)
# print(ego_vehicle_blueprints.id)
# ego_vehicle = world.spawn_actor(ego_vehicle_blueprints, ego_vehicle_spawn_points)
# # ego_vehicle.set_autopilot(True)

# print(ego_vehicle.type_id)
# print(ego_vehicle.get_location())

# spectator = world.get_spectator()
# transform = ego_vehicle.get_transform()
# spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
# carla.Rotation(-90,0,0)))

# ego_vehicle2_blueprints = world.get_blueprint_library().filter('vehicle.mercedes.sprinter')[0]

# ego_vehicle2_spawn_points = carla.Transform(ego_vehicle_spawn_points.location, ego_vehicle_spawn_points.rotation)
# ego_vehicle2_spawn_points.location.y += 100
# ego_vehicle2 = world.spawn_actor(ego_vehicle2_blueprints, ego_vehicle2_spawn_points)


