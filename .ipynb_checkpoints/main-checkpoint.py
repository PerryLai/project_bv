import carla
import random
import os
from carlaAPI.image_converter import depth_to_local_point_cloud, to_rgb_array
import carla
import os
from util import install_camera_set
import time
import json

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()

for i in range(0, 100):
    print(i)
    path = os.path.join('data', str(i))
    os.mkdir(path)
    # Get the blueprint library and filter for the vehicle blueprints
    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    # Get the map's spawn points
    spawn_points = world.get_map().get_spawn_points()

    ego_vehicle_blueprint = world.get_blueprint_library().filter('*model3*')[0]
    print(ego_vehicle_blueprint)
    ego_vehicle_spawn_point = random.choice(spawn_points)
    print(ego_vehicle_spawn_point)
    ego_vehicle = world.spawn_actor(ego_vehicle_blueprint, ego_vehicle_spawn_point)
    # ego_vehicle.set_autopilot(True)

    ego_json = {'id': ego_vehicle_blueprint.id,
                'x': ego_vehicle_spawn_point.location.x,
                'y': ego_vehicle_spawn_point.location.y,
                'z': ego_vehicle_spawn_point.location.z,
                'pitch': ego_vehicle_spawn_point.rotation.pitch,
                'yaw': ego_vehicle_spawn_point.rotation.yaw,
                'roll': ego_vehicle_spawn_point.rotation.roll}

    with open(os.path.join(path, "ego.json"), "w") as f:
        json.dump(ego_json, f)

    ego_vehicle2_blueprint = world.get_blueprint_library().filter('vehicle.mercedes.sprinter')[0]
    print(ego_vehicle2_blueprint)


    ego_vehicle2 = None


    while ego_vehicle2 is None:
        for point in spawn_points:
            px = point.location.x
            ex = ego_vehicle_spawn_point.location.x
            py = point.location.y
            ey = ego_vehicle_spawn_point.location.y
            if ((px - ex)**2 + (py - ey)**2) < 200 and not (px == ex and py == ey):
                ego_vehicle2_spawn_point = point
                print(ego_vehicle2_spawn_point)
                ego_vehicle2 = world.spawn_actor(ego_vehicle2_blueprint, ego_vehicle2_spawn_point)
                break
        spawn_points = world.get_map().get_spawn_points()

    # ego_vehicle2.set_autopilot(True)

    ego2_json = {'id': ego_vehicle2_blueprint.id,
                'x': ego_vehicle2_spawn_point.location.x,
                'y': ego_vehicle2_spawn_point.location.y,
                'z': ego_vehicle2_spawn_point.location.z,
                'pitch': ego_vehicle2_spawn_point.rotation.pitch,
                'yaw': ego_vehicle2_spawn_point.rotation.yaw,
                'roll': ego_vehicle2_spawn_point.rotation.roll}

    with open(os.path.join(path, "ego2.json"), "w") as f:
        json.dump(ego2_json, f)

    spectator = world.get_spectator()
    transform = ego_vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
    carla.Rotation(pitch=-90)))

    vehicles = []

    # Spawn 50 vehicles randomly distributed throughout the map
    # for each spawn point, we choose a random vehicle from the blueprint library
    for j in range(0,200):
        blueprint = random.choice(vehicle_blueprints)
        spawn_point = random.choice(spawn_points)
        temp = world.try_spawn_actor(blueprint, spawn_point)
        if temp is not None:
            vehicles.append({'id': blueprint.id,
                            'x': spawn_point.location.x,
                            'y': spawn_point.location.y,
                            'z': spawn_point.location.z,
                            'pitch': spawn_point.rotation.pitch,
                            'yaw': spawn_point.rotation.yaw,
                            'roll': spawn_point.rotation.roll})



    vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
    two_wheels = [a for a in vehicle_blueprints if a.get_attribute('number_of_wheels').as_int() == 2]
    hidden_blueprint = random.choice(two_wheels)

    hidden_x = 2*ego_vehicle2_spawn_point.location.x - ego_vehicle_spawn_point.location.x
    hidden_y = 2*ego_vehicle2_spawn_point.location.y - ego_vehicle_spawn_point.location.y
    hidden_z = ego_vehicle_spawn_point.location.z
    hidden_yaw = ego_vehicle2_spawn_point.rotation.yaw

    hidden_transform = carla.Transform(carla.Location(x=hidden_x, y=hidden_y,z=hidden_z),
    carla.Rotation(0,hidden_yaw,0))
    hidden = world.try_spawn_actor(hidden_blueprint, hidden_transform)

    vehicles.append({'id': hidden_blueprint.id,
                    'x': hidden_x,
                    'y': hidden_y,
                    'z': hidden_z,
                    'pitch': 0,
                    'yaw': hidden_yaw,
                    'roll': 0})

    with open(os.path.join(path, "vehicles.json"), "w") as f:
        json.dump(vehicles, f)

    walker_blueprints = world.get_blueprint_library().filter('*walker*')
    walkers = []
    for j in range(0,1000):
        walker_spawn_point = carla.Transform()
        walker_spawn_point.location = world.get_random_location_from_navigation()
        blueprint = random.choice(walker_blueprints)
        temp = world.try_spawn_actor(blueprint, walker_spawn_point)
        if temp is not None:
            walkers.append({'id': blueprint.id,
                            'x': walker_spawn_point.location.x,
                            'y': walker_spawn_point.location.y,
                            'z': walker_spawn_point.location.z,
                            'pitch': walker_spawn_point.rotation.pitch,
                            'yaw': walker_spawn_point.rotation.yaw,
                            'roll': walker_spawn_point.rotation.roll})

    with open(os.path.join(path, "walkers.json"), "w") as f:
        json.dump(walkers, f)


    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.1
    world.apply_settings(settings)

    cameras1 = install_camera_set(world, ego_vehicle, os.path.join(path, "A"), 2)
    cameras2 = install_camera_set(world, ego_vehicle2, os.path.join(path, "B"), 3)

    world.tick()
    time.sleep(5)

    world = client.reload_world()

# print("Before tick")
# world.tick()
# print("After tick")

# settings = world.get_settings()
# settings.synchronous_mode = False
# settings.fixed_delta_seconds = 0.1
# world.apply_settings(settings)

# world.tick()

# while True:
#     try:
#         world.tick()
#         time.sleep(1)
#     except:
#         print("tick lose")
#         time.sleep(2)
#         world.tick()
#     transform = ego_vehicle.get_transform()
#     spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
#     carla.Rotation(pitch=-90)))
