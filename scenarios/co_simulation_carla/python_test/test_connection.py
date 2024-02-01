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
import json
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
Extend_PORT = 65434

class Simulation():
    def __init__(self):
        self.artery_connExtend = ArterySynchronization(HOST, Extend_PORT)
        
    def run_artery(self, artery_path, project_name, config_name):
        if artery_path !="":
            if not os.path.isdir(artery_path):
                raise FileNotFoundError("couldn't find artery in path: '"+artery_path+"'. Please provide valid artery root path")
            
            if project_name == "" or config_name == "":
                raise AttributeError("Please enter your project name and your omnetpp configuration name!")

            if(not self.args.Qtenv):
                project_path = artery_path + '/scenarios/' + project_name
                p = Popen(['sleep 2 ; ' + artery_path +'/build/./run_artery.sh '+ project_path +'/omnetpp.ini -u Cmdenv -c '+config_name], cwd= project_path, shell=True)
            else:
                project_path = artery_path + '/scenarios/' + project_name
                p = Popen(['sleep 2 ; ' + artery_path +'/build/./run_artery.sh '+ project_path +'/omnetpp.ini -c '+config_name], cwd= project_path, shell=True)
               
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
        
        # Synchronize artery data with carla
        self.artery_connExtend.checkAndConnectclient()


        if  self.artery_connExtend.is_connected():
            current_step_EXmsg = json.loads(self.artery_connExtend.recieve_cpm_messages(0,2))
            # current_step_EXmsg = self.artery_connExtend.recieve_cpm_messages(0,2)
            # print(current_step_EXmsg.head(1))
            print(current_step_EXmsg)
        end = time.time()

        elapsed = end - start
        if elapsed < 0.5:
            time.sleep(0.5 - elapsed)
    

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

              

if __name__ == '__main__':
    simulation = Simulation()
    simulation.loop()