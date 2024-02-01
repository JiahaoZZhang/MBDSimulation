#! /usr/bin/env python


# Traci test with Artery & Sumo & Carla

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

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
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
        carla.Rotation(pitch=-90)))



class ExampleListener(traci.StepListener):
    def step(self, t):
        # do something after every call to simulationStep
        print("ExampleListener called with parameter %s." % t)
        # indicate that the step listener should stay active in the next step
        return True

if __name__== '__main__':

    sumo_PORT = 8813

    # traci.start(['sumo-gui', '-c', '../Tjunction.sumo.cfg', '--num-clients', '2'], port = sumo_PORT)
    # traci.setOrder(5)

    # traci.start(['sumo-gui', '-c', '../Tjunction.sumo.cfg'])
    client = traci.connect(host='localhost', port = sumo_PORT, label='test1')
    client.setOrder(2)
    # ============================
    # carla client connection
    # ============================
    # myCarla = CarlaClient('127.0.0.1', 2000)


    # listener = ExampleListener()

    # traci.switch("test1")
    step = 0
    RouteID = ""
    vType = ""
    depart = "now"
    departLane = ""
    departPos = ""
    vSpeed = 0
    addVehicleID = "ghost"
    vehicle_id = "Obj"

    while step < 1000:
        client.simulationStep()
        if step == 25:
            for vd in traci.vehicle.getIDList():
                # print(vd)
                traci.vehicle.setColor(vd,(0,255,0,255))
                if(vd == vehicle_id):
                    RouteID = traci.vehicle.getRouteID(vehicle_id)
                    vType = traci.vehicle.getTypeID(vehicle_id)
                    departLane = traci.vehicle.getLaneIndex(vehicle_id)
                    departPos = traci.vehicle.getLanePosition(vehicle_id)
                    vSpeed = traci.vehicle.getSpeed(vehicle_id)
                    traci.vehicle.add(addVehicleID,RouteID,vType,depart,departLane,departPos)
                    traci.vehicle.setSpeedMode(addVehicleID, 0)
                    print(vSpeed)

        if step > 25:
            vSpeed += 2
            if addVehicleID in traci.vehicle.getIDList():
                traci.vehicle.setSpeed(addVehicleID,vSpeed)
                traci.vehicle.setColor(addVehicleID,(205,204,0,255))
                # print(vSpeed)
        step += 1

    traci.close()