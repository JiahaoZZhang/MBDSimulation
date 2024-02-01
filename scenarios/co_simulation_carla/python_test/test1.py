#! /usr/bin/env python


# Traci test with Artery & Sumo & Carla

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys
import re 

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

    traci.start(['sumo-gui', '-c', '../Town04.sumo.cfg', '--num-clients', '2'], port = sumo_PORT)
    # TraciClient = traci.connect(host='localhost', port = sumo_PORT, label='test1')
    # TraciClient.setOrder(6)
    traci.setOrder(2)

    # ============================
    # carla client connection
    # ============================
    myCarla = CarlaClient('127.0.0.1', 2000)


    # listener = ExampleListener()

    # traci.switch('test1')
    step = 0
    x = '2'

    SetSpeed_trigger = False


    while step < 5000:
    # while 1:
        # client.addStepListener(listener(traci.simulation.getCurrentTime()))
        # TraciClient.simulationStep()
        traci.simulationStep()
        # if traci.simulation.getTime() == 3:
        # for vehicle_id in traci.vehicle.getIDList():
            # print(vehicle_id)
            # if vehicle_id == '2':
            # print("get id: %s", vehicle_id)
        for actor in myCarla.world.get_actors():
            if actor.type_id == "vehicle.citroen.c3":
                myCarla.camera_track_vehicle(actor.id)  
                # print(actor.id)
                  
        
        if traci.simulation.getTime() > 20:
            # myCarla.camera_track_vehicle(2)
            # print(myCarla.world.get_actors())
        
            traci.vehicle.setSpeed('2', 0)
            if SetSpeed_trigger == False:
                traci.vehicle.setSpeed('3', 60)
                # traci.vehicle.setSpeedMode('3', 0)
                SetSpeed_trigger = True
                traci.vehicle.setMinGap('3', 0)
                traci.vehicle.setTau('3',0.1)

            if len(traci.simulation.getCollidingVehiclesIDList()) != 0:
                print(traci.simulation.getCollidingVehiclesIDList())
                
        step += 1
    traci.close()