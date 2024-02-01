#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2021 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
from math import *

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

def compute_tti(i_pos, v_pos, v_speed, v_angle):
    tti = inf

    speed_cond = (v_speed != 0.0)

    # Marge d'erreur si l'intersection n'est pas totalement droite
    # epsilon = 2.0 suffisant pour le Town04-Tjunction.rou.xml
    epsilon = 2.0

    # Conditions pour vérifier si une voiture a dépassé l'intersection ou non (i_pos)
    # Véhicule qui va de bas en haut
    angle_down_cond = abs(v_angle - 0) <= epsilon and v_pos[1] < i_pos[1]
    # Véhicule qui va de gauche à droite
    angle_left_cond = abs(v_angle - 90) <= epsilon and v_pos[0] < i_pos[0]
    # Véhicule qui va de haut en bas
    angle_up_cond = abs(v_angle - 180) <= epsilon and v_pos[1] > i_pos[1]
    # Véhicule qui va de droite à gauche
    angle_right_cond = abs(v_angle - 270) <= epsilon and v_pos[0] > i_pos[0]

    # Si le véhicule n'est pas à l'arrêt et qu'il n'a pas encore atteint l'intersection, calculer tti
    # Sinon tti = infini
    if speed_cond and (angle_left_cond or angle_right_cond or angle_up_cond or angle_down_cond):
        tti = sqrt(pow(i_pos[0] - v_pos[0], 2) + pow(i_pos[1] - v_pos[1], 2)) / v_speed

    return tti

def run():
    traci.simulationStep()

    # Couleurs EGO
    color_green = (0, 222, 0)
    color_blue = (0, 222, 222)
    color_red = (222, 0, 0)

    # Couleur OTHER
    color_purple = (222, 0, 222)

    # Identifiants du véhicule EGO et de OTHER
    ego_id = "ego"
    other_id = "other" 

    # Position (x, y) de l'intersection (centre de l'intersection)
    inter_pos = (760, 672)
    
    # Seuil de danger du time to intersection (TTI)
    limit_tti = 3.0

    # Seuil de danger des differences du TTI entre EGO et OTHER
    limit_diff_tti = 3.0

    # Vitesse finale à atteindre
    final_speed = 0.0 

    # Accélération à prendre en cas de réaction impliquant un ralentissement
    fixed_acceleration = -5.0

    # True si EGO est contrôlé par TRACI, False sinon
    is_traci = False

    # True si EGO est totalement à l'arrêt (forcé par TRACI), False sinon
    is_stopped = False

    while traci.simulation.getMinExpectedNumber() > 0:
        temps_simulation = traci.simulation.getTime()
        
        #print(f"Temps de simulation : {temps_simulation} secondes")  

        if ego_id in traci.vehicle.getIDList() and other_id in traci.vehicle.getIDList():
            if not(is_traci):
                # Couleur verte pour les véhicules EGO et OTHER (DETECTION)
                traci.vehicle.setColor(ego_id, color_green)
                traci.vehicle.setColor(other_id, color_purple)

            # Caractéristiques EGO
            ego_speed = traci.vehicle.getSpeed(ego_id)
            ego_pos = traci.vehicle.getPosition(ego_id)
            ego_angle = traci.vehicle.getAngle(ego_id)

            # Caractéristiques OTHER
            other_speed = traci.vehicle.getSpeed(other_id)
            other_pos = traci.vehicle.getPosition(other_id)
            other_angle = traci.vehicle.getAngle(other_id)

            # Calcul du TTI des véhicules EGO et OTHER
            tti_ego = inf
            tti_other = inf

            tti_ego = compute_tti(inter_pos, ego_pos, ego_speed, ego_angle)

            tti_other = compute_tti(inter_pos, other_pos, other_speed, other_angle)
                
            # Calcul de la différence de TTI entre les véhicules EGO et OTHER
            diff_tti = abs(tti_other - tti_ego)

            # Fixer la vitesse du véhicule EGO à 0 si sa vitesse est proche de 0 
            if is_traci and (is_stopped or (traci.vehicle.getSpeed(ego_id) >= 0 and traci.vehicle.getSpeed(ego_id) <= 2.0)):
                is_stopped = True

                traci.vehicle.setSpeed(ego_id, 0.0)

                # Couleur rouge pour les véhicules EGO et OTHER (ARRET)
                traci.vehicle.setColor(ego_id, color_red)
                traci.vehicle.setColor(other_id, color_purple)
            # Activer TRACI et la décélération
            # - Si le véhicule EGO est proche de l'intersection
            # - Si une collision entre les véhciules EGO et OTHER est proche
            elif (tti_ego < limit_tti and diff_tti < limit_diff_tti) or is_traci:
                is_traci = True

                # Temps de décélération pour atteindre 
                # la vitesse 'final_speed' avec 'fixed_acceleration'
                duration = (final_speed - ego_speed) / fixed_acceleration

                traci.vehicle.slowDown(ego_id, final_speed, ceil(duration))

                # Couleur bleue pour les véhicules EGO et OTHER (FREINAGE)
                traci.vehicle.setColor(ego_id, color_blue)
                traci.vehicle.setColor(other_id, color_purple)
        
        traci.simulationStep()

    traci.close()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "Town04_traciApp.sumo.cfg"])
    run()