#! /usr/bin/env python


# Traci test with Artery & Sumo & Carla

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys
import re 
import math
import argparse
# from sumo_integration.bridge_helper import BridgeHelper

try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import carla

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci


class CarlaClient(object):
    def __init__(self, host, port):
        self.client = carla.Client(host, port)
        self.client.set_timeout(2.0)

        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()

    def get_actor(self, actor_id):
        """
        Accessor for carla actor.
        """
        return self.world.get_actor(actor_id)
    
    def camera_track_vehicle(self, actor_id):
        vehicle = self.get_actor(actor_id)
        spectator = self.world.get_spectator()

        transform = vehicle.get_transform()
        camera_rotation = carla.Rotation(pitch=transform.rotation.pitch - 15,
                                         yaw=transform.rotation.yaw, roll=transform.rotation.roll)
        
        carmera_distance = 4
        camera_x = - math.cos(transform.rotation.yaw*math.pi/180) * carmera_distance
        camera_y = - math.sin(transform.rotation.yaw*math.pi/180) * carmera_distance
        spectator.set_transform(carla.Transform(transform.location + carla.Location(x=camera_x,y=camera_y,z=3),
                                                camera_rotation))
    
    def world_camera(self, x=211.70, y=-217.07,z=95.22):
        spectator = self.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x, y, z),
        carla.Rotation(pitch=-90,yaw=-90)))


class ExampleListener(traci.StepListener):
    def step(self, t):
        # do something after every call to simulationStep
        print("ExampleListener called with parameter %s." % t)
        # indicate that the step listener should stay active in the next step
        return True

if __name__== '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--track-vehicle',
                           default="",
                           type=str)
    argparser.add_argument('--case',
                        default="",
                        type=str)
    arguments = argparser.parse_args()

    sumo_PORT = 8813

    # traci.start(['sumo-gui', '-c', '../Town04.sumo.cfg', '--num-clients', '2'], port = sumo_PORT)
    TraciClient = traci.connect(host='localhost', port = sumo_PORT, label='test1')
    TraciClient.setOrder(6)
    # traci.setOrder(2)

    # ============================
    # carla client connection
    # ============================
    myCarla = CarlaClient('127.0.0.1', 2000)


    # listener = ExampleListener()

    traci.switch('test1')
    step = 0
    track_vehicle_id = arguments.track_vehicle
    sumo2carla_ids = {}

    sumo_vehicle_ids = []
    carla_vehicle_ids = []
    sumo_vehicle_ids_pre = []
    carla_vehicle_ids_pre = []

    while step < 5000:
        TraciClient.simulationStep()

        # Getting the new vehicle sumo id for each step 
        for sumo_id in traci.vehicle.getIDList():
            sumo_vehicle_ids.append(sumo_id)
            if (sumo_id == "4"):
                x,y = traci.vehicle.getPosition(sumo_id)
                # print(x)
        # sumo_vehicle_ids = list(dict.fromkeys(sumo_vehicle_ids))
        # print(traci.vehicle.getIDList())

        # Getting the new carla vehicle actor id for each step
        # for vehicle_actor in myCarla.world.get_actors().filter("vehicle.nissan.micra"):
        #     carla_vehicle_ids.append(vehicle_actor.id)

        # carla_vehicle_ids = list(dict.fromkeys(carla_vehicle_ids))
        # print(carla_vehicle_ids)
        # print(myCarla.world.get_actors().filter("vehicle.nissan.micra"))

        # Mapping the vehicle sumo id and vehicle carla id 
        # for i in range(len(sumo_vehicle_ids)):
        #     if not sumo2carla_ids.get(sumo_vehicle_ids[i]):
        #         sumo2carla_ids[sumo_vehicle_ids[i]] = carla_vehicle_ids[i]

        # print(sumo_vehicle_ids)
        # print(carla_vehicle_ids)
        # print(new_vehicle_sumo_ids)
        # print(new_vehicle_carla_ids)



        # Track the focus vehicle
        if track_vehicle_id in traci.vehicle.getIDList():
            myCarla.camera_track_vehicle(sumo2carla_ids[track_vehicle_id])
        else:
            if(arguments.case == "traciApp"):
                myCarla.world_camera(x=241.70, y=-267.07,z=145.22)
            else:
                myCarla.world_camera()


                
        step += 1


    traci.close()