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
client.set_timeout(25.0)
world = client.get_world()

for i in range(76,100):
    print(i)
    path = os.path.join('data_network', str(i))
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
    box = ego_vehicle.bounding_box
    # ego_vehicle.set_autopilot(True)

    ego_json = {'id': ego_vehicle_blueprint.id,
                'x': ego_vehicle_spawn_point.location.x,
                'y': ego_vehicle_spawn_point.location.y,
                'z': ego_vehicle_spawn_point.location.z,
                'pitch': ego_vehicle_spawn_point.rotation.pitch,
                'yaw': ego_vehicle_spawn_point.rotation.yaw,
                'roll': ego_vehicle_spawn_point.rotation.roll,
                'box_x': box.extent.x*2,
                'box_y': box.extent.y*2,
                'box_z': box.extent.z*2}

    with open(os.path.join(path, "ego.json"), "w") as f:
        json.dump(ego_json, f)

    ego_vehicle2_blueprint = world.get_blueprint_library().filter('vehicle.mercedes.sprinter')[0]

    ego_vehicle2 = None

    while ego_vehicle2 is None:
        for point in spawn_points:
            px = point.location.x
            ex = ego_vehicle_spawn_point.location.x
            py = point.location.y
            ey = ego_vehicle_spawn_point.location.y
            if ((px - ex)**2 + (py - ey)**2) < 200 and not (px == ex and py == ey):
                ego_vehicle2_spawn_point = point
                ego_vehicle2 = world.spawn_actor(ego_vehicle2_blueprint, ego_vehicle2_spawn_point)
                box = ego_vehicle2.bounding_box
                break
        spawn_points = world.get_map().get_spawn_points()
        
    ego2_json = {'id': ego_vehicle2_blueprint.id,
                'x': ego_vehicle2_spawn_point.location.x,
                'y': ego_vehicle2_spawn_point.location.y,
                'z': ego_vehicle2_spawn_point.location.z,
                'pitch': ego_vehicle2_spawn_point.rotation.pitch,
                'yaw': ego_vehicle2_spawn_point.rotation.yaw,
                'roll': ego_vehicle2_spawn_point.rotation.roll,
                'box_x': box.extent.x*2,
                'box_y': box.extent.y*2,
                'box_z': box.extent.z*2}

    with open(os.path.join(path, "ego2.json"), "w") as f:
        json.dump(ego2_json, f)
        
    spectator = world.get_spectator()
    transform = ego_vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
    carla.Rotation(pitch=-90)))

    vehicles = []
    four_wheels_vehicles = []

    # Spawn 200 vehicles randomly distributed throughout the map
    # for each spawn point, we choose a random vehicle from the blueprint library
    print("Spawn vehicles")
    for j in range(0,200):
        blueprint = random.choice(vehicle_blueprints)
        spawn_point = random.choice(spawn_points)
        temp = world.try_spawn_actor(blueprint, spawn_point)
        if temp is not None:
            box = temp.bounding_box
            vehicles.append({'id': blueprint.id,
                            'x': spawn_point.location.x,
                            'y': spawn_point.location.y,
                            'z': spawn_point.location.z,
                            'pitch': spawn_point.rotation.pitch,
                            'yaw': spawn_point.rotation.yaw,
                            'roll': spawn_point.rotation.roll,
                            'box_x': box.extent.x*2,
                            'box_y': box.extent.y*2,
                            'box_z': box.extent.z*2})
            if blueprint.get_attribute('number_of_wheels').as_int() == 4:
                four_wheels_vehicles.append({'id': blueprint.id,
                                'x': spawn_point.location.x,
                                'y': spawn_point.location.y,
                                'z': spawn_point.location.z,
                                'pitch': spawn_point.rotation.pitch,
                                'yaw': spawn_point.rotation.yaw,
                                'roll': spawn_point.rotation.roll,
                                'box_x': box.extent.x*2,
                                'box_y': box.extent.y*2,
                                'box_z': box.extent.z*2,
                                'object': temp})

    
    def sortkey(vehicle):
        px = vehicle['x']
        ex = ego_vehicle_spawn_point.location.x
        py = vehicle['y']
        ey = ego_vehicle_spawn_point.location.y
        return (px - ex)**2 + (py - ey)**2
    
    four_wheels_vehicles.sort(key=sortkey)
    

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
    print("Spawn walkers")
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

    cameras1 = install_camera_set(world, ego_vehicle, os.path.join(path, "A"), ego_json['box_z']+0.6)
    cameras2 = install_camera_set(world, ego_vehicle2, os.path.join(path, "B"), ego2_json['box_z']+0.6)
    cameras3 = install_camera_set(world, four_wheels_vehicles[0]['object'], os.path.join(path, "C"),
                                    four_wheels_vehicles[0]['box_z']+0.6)
    with open(os.path.join(path, "ego3.json"), "w") as f:
        del four_wheels_vehicles[0]['object']
        json.dump(four_wheels_vehicles[0], f)
    cameras4 = install_camera_set(world, four_wheels_vehicles[1]['object'], os.path.join(path, "D"),
                                    four_wheels_vehicles[1]['box_z']+0.6)
    with open(os.path.join(path, "ego4.json"), "w") as f:
        del four_wheels_vehicles[1]['object']
        json.dump(four_wheels_vehicles[1], f)
    cameras5 = install_camera_set(world, four_wheels_vehicles[2]['object'], os.path.join(path, "E"),
                                    four_wheels_vehicles[2]['box_z']+0.6)
    with open(os.path.join(path, "ego5.json"), "w") as f:
        del four_wheels_vehicles[2]['object']
        json.dump(four_wheels_vehicles[2], f)
    cameras6 = install_camera_set(world, four_wheels_vehicles[3]['object'], os.path.join(path, "F"),
                                    four_wheels_vehicles[3]['box_z']+0.6)
    with open(os.path.join(path, "ego6.json"), "w") as f:
        del four_wheels_vehicles[3]['object']
        json.dump(four_wheels_vehicles[3], f)

    world.tick()
    print("Sleep 10s")
    time.sleep(10)

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
