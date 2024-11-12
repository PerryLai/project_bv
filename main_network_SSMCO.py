# -*- coding: utf-8 -*-
import carla
import random
import shutil
import os
from carlaAPI.image_converter import depth_to_local_point_cloud, to_rgb_array
# from util import install_camera_set
import time
import json
import numpy as np

def install_camera_set(world, ego_vehicle, dirname, high=2):
    
    camera_list = []
    
    cam_bp = None
    cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_bp.set_attribute("image_size_x",str(1920))
    cam_bp.set_attribute("image_size_y",str(1080))
    cam_location = carla.Location(0,0,80)
    cam_rotation = carla.Rotation(pitch=-90)
    cam_transform = carla.Transform(cam_location,cam_rotation)
    ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    ego_cam.listen(lambda image: image.save_to_disk(dirname+'/rgb.jpg') )
    camera_list.append(cam_bp)
    camera_list.append(ego_cam)
        
    depth1_cam = None
    depth1_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth1_bp.set_attribute("fov", str(90))
    depth1_location = carla.Location(0,0,high)
    depth1_rotation = carla.Rotation(0,0,0)
    depth1_transform = carla.Transform(depth1_location,depth1_rotation)
    depth1_cam = world.spawn_actor(depth1_bp,depth1_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler1(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/1.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '1.ply')
        )

    depth1_cam.listen(handler1)
    camera_list.append(depth1_bp)
    camera_list.append(depth1_cam)

    depth2_cam = None
    depth2_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth2_bp.set_attribute("fov", str(90))
    depth2_location = carla.Location(0,0,high)
    depth2_rotation = carla.Rotation(0,180,0)
    depth2_transform = carla.Transform(depth2_location,depth2_rotation)
    depth2_cam = world.spawn_actor(depth2_bp,depth2_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler2(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/2.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '2.ply')
        )

    depth2_cam.listen(handler2)
    camera_list.append(depth2_bp)
    camera_list.append(depth2_cam)

    depth3_cam = None
    depth3_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth3_bp.set_attribute("fov", str(90))
    depth3_location = carla.Location(0,0,high)
    depth3_rotation = carla.Rotation(0,90,0)
    depth3_transform = carla.Transform(depth3_location,depth3_rotation)
    depth3_cam = world.spawn_actor(depth3_bp,depth3_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler3(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/3.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '3.ply')
        )

    depth3_cam.listen(handler3)
    camera_list.append(depth3_bp)
    camera_list.append(depth3_cam)

    depth4_cam = None
    depth4_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth4_bp.set_attribute("fov", str(90))
    depth4_location = carla.Location(0,0,high)
    depth4_rotation = carla.Rotation(0,270,0)
    depth4_transform = carla.Transform(depth4_location,depth4_rotation)
    depth4_cam = world.spawn_actor(depth4_bp,depth4_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view

    def handler4(image):
        point_cloud = depth_to_local_point_cloud(
            image,
            None,
            max_depth=0.1,
            high=high
        )
        image.save_to_disk(dirname+'/4.jpg',carla.ColorConverter.LogarithmicDepth)
        point_cloud.save_to_disk(os.path.join(
            dirname, '4.ply')
        )

    depth4_cam.listen(handler4)
    camera_list.append(depth4_bp)
    camera_list.append(depth4_cam)
    
    return camera_list

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
client.set_timeout(30.0)
world = client.get_world()

for i in range(88,100): #default=range(100)
    path = os.path.join('data_network_SSMCO', str(i))
    if os.path.exists(path):
        shutil.rmtree(path)  # 刪除目錄及其內容
    os.mkdir(path)  # 創建新目錄
    # os.mkdir(path)
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

    with open(os.path.join(path, "vehicles.json"), "w") as f:
        json.dump(vehicles, f)
    
    ex = ego_vehicle_spawn_point.location.x
    ey = ego_vehicle_spawn_point.location.y
    
    def sortkey(vehicle):
        px = vehicle['x']
        py = vehicle['y']
        return (px - ex)**2 + (py - ey)**2
    
    four_wheels_vehicles.sort(key=sortkey)

    seen_four_wheels_vehicles = []
    
    for vehicle in four_wheels_vehicles:
        x = vehicle['x']
        y = vehicle['y']
        if ((ex-x)**2 + (ey-y)**2) < 2500:
            seen = True
            for seen_vehicle in seen_four_wheels_vehicles:
                a = seen_vehicle['y'] - ey
                b = ex - seen_vehicle['x']
                c = seen_vehicle['x']*ey-seen_vehicle['y']*ex

                distance = np.abs(a*x+b*y+c)/np.sqrt(a**2+b**2)
                if distance <= seen_vehicle['box_x']/2:
                    seen = False
                    break
            if seen:
                seen_four_wheels_vehicles.append(vehicle)
        else:
            break
    
    print(len(seen_four_wheels_vehicles))
    print(seen_four_wheels_vehicles)

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
    
    cameras = []
    for i, vehicle in enumerate(seen_four_wheels_vehicles):
        camera = install_camera_set(world, vehicle['object'], os.path.join(path, chr(ord('A')+i+1)),
                                    vehicle['box_z']+0.6)
        cameras.append(camera)
        with open(os.path.join(path, "ego{}.json").format(i+2), "w") as f:
            del vehicle['object']
            json.dump(vehicle, f)
    
#     cameras3 = install_camera_set(world, four_wheels_vehicles[0]['object'], os.path.join(path, "C"),
#                                     four_wheels_vehicles[0]['box_z']+0.6)
#     with open(os.path.join(path, "ego3.json"), "w") as f:
#         del four_wheels_vehicles[0]['object']
#         json.dump(four_wheels_vehicles[0], f)
#     cameras4 = install_camera_set(world, four_wheels_vehicles[1]['object'], os.path.join(path, "D"),
#                                     four_wheels_vehicles[1]['box_z']+0.6)
#     with open(os.path.join(path, "ego4.json"), "w") as f:
#         del four_wheels_vehicles[1]['object']
#         json.dump(four_wheels_vehicles[1], f)
#     cameras5 = install_camera_set(world, four_wheels_vehicles[2]['object'], os.path.join(path, "E"),
#                                     four_wheels_vehicles[2]['box_z']+0.6)
#     with open(os.path.join(path, "ego5.json"), "w") as f:
#         del four_wheels_vehicles[2]['object']
#         json.dump(four_wheels_vehicles[2], f)
#     cameras6 = install_camera_set(world, four_wheels_vehicles[3]['object'], os.path.join(path, "F"),
#                                     four_wheels_vehicles[3]['box_z']+0.6)
#     with open(os.path.join(path, "ego6.json"), "w") as f:
#         del four_wheels_vehicles[3]['object']
#         json.dump(four_wheels_vehicles[3], f)

    world.tick()
    print("Sleep 5s")
    time.sleep(5)

    world = client.reload_world()
