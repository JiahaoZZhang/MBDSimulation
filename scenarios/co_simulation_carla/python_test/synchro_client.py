#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and SUMO simulations
"""
# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time
import pandas as pd
import numpy as np
import subprocess, signal
import psutil
import lib_reaction

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os


import sys
from numpy import  random, sqrt


try:
    sys.path.append(
        glob.glob('/home/jiahao/carla/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci


# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================
import sumolib

sys.path.append("./carla/Co-Simulation/Sumo")

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

import sumo_integration.run_synchronization
from sumo_integration.run_synchronization import SimulationSynchronization  

sys.path.append('../CarlaSumoArtery-CoSimulation')
sys.path.append('../CarlaSumoArtery-CoSimulation/simulation_modules')

from carla_artery_connection import ArterySynchronization
# from attacker_module import GhostAheadAttacker
# from attacker_module import RandomOnRoadPositionAttacker
# from attacker_module import RandomOffsetPositionAttacker
# from attacker_module import ConstantOffsetPositionAttacker

from painting_module import Painter
# from detection_module import SSC,RLDetector
from subprocess import Popen

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================
HOST = '127.0.0.1'
RX_PORT = 65432  
TX_PORT = 65433
PER_PORT = 65434

class Simulation():
    def __init__(self,args,rl_model=None):
        print("Creating a new simulation ")
        self.args=args
        self.cams=[]
        if self.args.start_artery:
            self.artery= self.run_artery(args.artery_path, args.project_name, args.config_name)
        self.sumo_simulation = SumoSimulation(args.sumo_cfg_file,args.step_length , args.sumo_host,
                                        args.sumo_port, args.sumo_gui, args.client_order,args.num_clients)
        self.carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

        self.synchronization = SimulationSynchronization(self.sumo_simulation, self.carla_simulation, args.tls_manager,
                                                    args.sync_vehicle_color, args.sync_vehicle_lights)
        self.synchronization.artery2sumo_ids={}
        # self.synchronization.arteryAttackers_ids=set({})
        self.synchronization.net = self.sumo_simulation.net
        self.world = self.carla_simulation.world

        self.blueprint_library = self.world.get_blueprint_library()

        # self.vehicle_bp = random.choice(self.blueprint_library.filter('vehicle.audi.*'))

        self.v2xCar_benign=set([])
        self.v2xCar_malicious=set([])
        self.v2xCar_malicious_detected=set([])
        # self.attackers=[ GhostAheadAttacker('56'),
        #             ConstantOffsetPositionAttacker('21'),
        #             GhostAheadAttacker('1')]

        # self.attackers=[np.random.choice([GhostAheadAttacker,RandomOnRoadPositionAttacker])(str(i)) for i in np.random.randint(1,100,size = 30)]
        self.artery_connRX = ArterySynchronization(HOST, RX_PORT)
        self.artery_connTX = ArterySynchronization(HOST, TX_PORT)
        self.artery_conn_scene = ArterySynchronization(HOST, PER_PORT)

        self.carla_painter = Painter(self.synchronization) #  ~= CPM frequency 

        net_file = "../net/Town04.net.xml"
        self.junction_info = lib_reaction.get_junction_info(net_file)


        # if rl_model :
        #     self.global_detector = RLDetector(rl_model)   
        # else :
        #     self.global_detector = SSC(self.synchronization.net,200)   

        # self.focus_vehicle_id = str(np.random.randint(4,30))
    
    def run_artery(self, artery_path, project_name, config_name):
        if artery_path !="":
            if not os.path.isdir(artery_path):
                raise FileNotFoundError("couldn't find artery in path: '"+artery_path+"'. Please provide valid artery root path")
            
            if project_name == "" or config_name == "":
                raise AttributeError("Please enter your project name and your omnetpp configuration name!")

            if(not self.args.Qtenv):
                project_path = artery_path + '/scenarios/' + project_name
                artery_build_path = artery_path+'/build'
                p = Popen(["cmake --build ."], cwd= artery_build_path, shell=True)
                # if p.wait():
                p = Popen(['sleep 10 ; ' + artery_path +'/build/./run_artery.sh '+ project_path +'/omnetpp.ini -u Cmdenv -c '+config_name], cwd= project_path, shell=True)
 
            else:
                project_path = artery_path + '/scenarios/' + project_name
                artery_build_path = artery_path+'/build'
                p = Popen(["cmake --build ."], cwd= artery_build_path, shell=True) 
                # if p.wait():
                p = Popen(['sleep 10 ; ' + artery_path +'/build/./run_artery.sh '+ project_path +'/omnetpp.ini -c '+config_name], cwd= project_path, shell=True)


        else:
            raise AttributeError("you should specify artery build path !")
            # p = Popen(['sleep 2 ; make run_example'], 
            #           cwd='/home/jiahao/artery/build', shell=True)
        return p
        
    def kill_artery(self,and_sumo=False):
        p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
        out, err = p.communicate()
        for line in out.splitlines():
            # print(line)
            if b'opp_run_release' in line or b'artery' in line or b'omnetpp' in line:
                print(line)
                pid = int(line.split(None, 1)[0])
                os.kill(pid, signal.SIGKILL)
            if and_sumo and b'sumo' in line :
                print(line)
                pid = int(line.split(None, 1)[0])
                os.kill(pid, signal.SIGKILL)
                os.kill(pid, signal.SIGTERM)          
                parent = psutil.Process(pid)
                parent.kill()

    def step(self):
        start = time.time()

        current_step_RXmsg=[]
        current_step_TXmsg=[]

        # synchronize carla with sumo
        self.synchronization.tick()

        # Synchronize artery data with carla
        self.artery_connRX.checkAndConnectclient()
        self.artery_connTX.checkAndConnectclient()
        self.artery_conn_scene.checkAndConnectclient()

        if self.artery_conn_scene.is_connected():
            current_extendedpereceptiontable = self.artery_conn_scene.recieve_cpm_messages(self.synchronization, 2)
            inter_pos = (758,672)
            for Exm in current_extendedpereceptiontable:
                Exm = pd.json_normalize(Exm,"ExtendedPerceivedObjects", ["mySumoId","Xegosumoposition","Yegopsumoposition","longspeed"])
                
                if not Exm.empty and Exm["mySumoId"].iloc[0] == "ego":
                    # temp = pd.json_normalize(Exm)
                    # print(Exm["mySumoId"])
                    # print(Exm["mySumoId"].iloc[0])
                    pd.set_option('display.max_columns', None)
                    Exm = Exm.where(Exm["SumoPerceivedObjectID"] != Exm['mySumoId']).dropna()
                    if not Exm.empty:
                        lib_reaction.make_decision(Exm, self.junction_info)
                    # print(pd.json_normalize(Exm,"ExtendedPerceivedObjects", ["mySumoId","Xegosumoposition","Yegopsumoposition","longspeed"]))


        # perform all attacks
        # for attacker in self.attackers:
        #     attacker.perform_attack(self.synchronization,self.vehicle_bp)

        # apply all detection mechanisms
        # detections = set([])
        # focused_vehicle_cams=[]

        if  self.artery_connRX.is_connected():
            current_step_RXmsg = self.artery_connRX.recieve_cpm_messages(self.synchronization, 0)
            # print("v2xCarList::", self.v2xCar_benign, self.v2xCar_malicious)

            #     current_step_cams=self.artery_conn.recieve_cam_messages(self.synchronization)
            #     focused_vehicle_cams = [cam for cam in current_step_cams if cam["receiver_sumo_id"]==self.focus_vehicle_id ]
            #     detections = self.global_detector.check(self.synchronization.artery2sumo_ids,focused_vehicle_cams,self.synchronization.arteryAttackers_ids)
            #     self.cams.extend(focused_vehicle_cams)
            
            # color communication
            if self.args.color_cpms:
                for cpm in current_step_RXmsg:
                    self.carla_painter.color_communication(cpm)


        if self.artery_connTX.is_connected():
            current_step_TXmsg = self.artery_connTX.recieve_cpm_messages(self.synchronization, 1)
            # color agents accordingly
            if self.args.color_agents:
                for cpm in current_step_TXmsg:
                    if (cpm['sender_type'] == 0): # attacker type:1 , genius type:0
                        if (not cpm["sender_sumo_id"] in self.v2xCar_benign):
                            self.v2xCar_benign.add(cpm["sender_sumo_id"])
                    elif (cpm['sender_type'] == 1):
                        if (not cpm["sender_sumo_id"] in self.v2xCar_malicious):
                            self.v2xCar_malicious.add(cpm["sender_sumo_id"])
                    elif (cpm['sender_type'] == 3):
                        if (cpm["sender_sumo_id"] in self.v2xCar_malicious):
                            self.v2xCar_malicious.remove(cpm["sender_sumo_id"])
                        else:
                            self.v2xCar_malicious_detected.add(cpm["sender_sumo_id"])
                self.carla_painter.color_agents(self.v2xCar_benign, self.v2xCar_malicious, self.v2xCar_malicious_detected)
            
            # color attack
            if self.args.color_attack:
                for cpm in current_step_TXmsg:
                    # try:
                    #     print("perceived object information::",cpm["attacked_obj_sumo_ids"], cpm["attacked_obj_long"]
                    #               ,cpm["attacked_obj_lat"], cpm["attacked_obj_x"], cpm["attacked_obj_y"],cpm["attacked_obj_yaw"])
                    # except:
                    #     pass
                        
                    self.carla_painter.color_attackedObj(cpm)

        end = time.time()

        elapsed = end - start
        if elapsed < self.args.step_length:
            time.sleep(self.args.step_length - elapsed)
    

    def loop(self):
        try:
            while True:
                self.step()
        except KeyboardInterrupt:
            logging.info('Cancelled by user.')
        except Exception as e:
            logging.info("error::", e)

        finally:
            self.close()


    def close(self, on_error = False):
        print('Cleaning synchronization')
        logging.info('Cleaning synchronization')
        
        # cams_df = pd.DataFrame(self.cams)
        # if len(cams_df)>0:
        #      cams_df.assign(label = cams_df['Station ID'].isin(self.synchronization.arteryAttackers_ids)).to_pickle('data/cams'+ str(time.time())+'.pckl') 
        # self.cams=[]
        
        print('******************************* closing artery connection ********************************************')
        self.artery_connRX.shutdownAndClose()
        self.artery_connTX.shutdownAndClose()
        self.artery_conn_scene.shutdownAndClose()
        
        print('******************************* closing synchronization ********************************************')
        if not on_error:
            self.synchronization.close()
        else :
            print ("traci._connections")
        print('******************************* closing all actors ********************************************')
        for actor in self.synchronization.carla.world.get_actors().filter('vehicle.*.*'):
            actor.destroy()
        print('******************************* reset world ********************************************')
        settings = self.synchronization.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None     
        self.synchronization.carla.world.apply_settings(settings)
        time.sleep(5)

        print('******************************* closing artery process ********************************************')
        if self.args.start_artery:
            try:
                os.killpg(os.getpgid(self.artery.pid), signal.SIGTERM)
                # self.artery.terminate()
                # self.artery.kill()
            except:
                print("artery has been closed.")
        else:
            if on_error:
                self.kill_artery(True)
            else:
                self.kill_artery()
                

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('sumo_cfg_file', type=str, help='sumo configuration file',default="../Town04.sumocfg")

    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=8813,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo', default=False)
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=5,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 2)')
    argparser.add_argument('--num-clients',
                           default=1,
                           type=int,
                           help='Number of connected clients on sumo server (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='carla')
    argparser.add_argument('--start-artery', action='store_true', help='start artery (experimental feature)')
    argparser.add_argument('--artery-path', type=str, help='artery root path',default="")
    argparser.add_argument('--project-name', type=str, help='your project name')
    argparser.add_argument('--config-name', type=str, default='General', help='your omnetpp.ini configuration name (default: General)')
    argparser.add_argument('--Qtenv', type=str,default=False, help='open Qtenv (default: False)')

    argparser.add_argument('--color-cpms', action='store_true', help='color cpm communications with red links')
    argparser.add_argument('--color-agents', action='store_true', help='color different agents')
    argparser.add_argument('--color-attack', action='store_true', help='color attack')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    # if arguments.sync_vehicle_all is True:
    #     arguments.sync_vehicle_lights = True
    #     arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    simulation = Simulation(arguments)
    simulation.loop()