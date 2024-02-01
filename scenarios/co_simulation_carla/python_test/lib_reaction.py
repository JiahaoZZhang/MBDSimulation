import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

import math
import traci
import sumolib

def compute_tti_manual(i_pos, v_pos, v_speed):
    if v_speed == 0.0:
        return math.inf
    
    distance = math.sqrt(pow(i_pos[0] - v_pos[0], 2) + pow(i_pos[1] - v_pos[1], 2))
    
    return distance / v_speed

def compute_tti_df(row, inter_pos):
    if row['Velocity'] == 0.0:
        return math.inf
    
    x_diff = inter_pos[0] - row['Xcoordinate']
    y_diff = inter_pos[1] - row['Ycoordinate']
    distance = math.sqrt(pow(x_diff, 2) + pow(y_diff, 2))
    tti = distance / row['Velocity']
    
    return tti

def get_ego_info(df):

    unique_ego_id = df['mySumoId'].iloc[0]
    unique_ego_x_pos = df['Xegosumoposition'].iloc[0]
    unique_ego_y_pos = df['Yegopsumoposition'].iloc[0]
    unique_ego_speed = df['longspeed'].iloc[0]

    # if (unique_ego_id * unique_ego_x_pos * unique_ego_y_pos * unique_ego_speed) != 1 :
    #     print("Error: different SUMO informations for ego vehicle")
    #     return -1
    
    ego_id = unique_ego_id
    ego_xpos = unique_ego_x_pos
    ego_ypos = unique_ego_y_pos
    ego_speed = unique_ego_speed

    return {'ID':ego_id, 'XPOS':ego_xpos, 'YPOS':ego_ypos, 'SPEED':ego_speed}

def get_slowdown_duration(ego_speed, final_speed, fixed_acceleration):
    if fixed_acceleration != 0:
        return (final_speed - ego_speed) / fixed_acceleration
    
    print("Error: fixed acceleration equals to 0")

    return 0

# Charger le réseau SUMO à partir du fichier de configuration
def get_junction_info(net_file):
    net = sumolib.net.readNet(net_file)

   # Récupérer la liste des IDs des junctions
    junction_ids = [node.getID() for node in net.getNodes()]

    junction_info = {}

    # Parcourir chaque junction et récupérer les liens entrants et sortants
    for junction_id in junction_ids:
        junction = net.getNode(junction_id)

        # Récupérer les liens entrants (incoming)
        incoming_links = [edge.getID() for edge in junction.getIncoming()]

        # Récupérer les liens sortants (outgoing)
        outgoing_links = [edge.getID() for edge in junction.getOutgoing()]

        junction_info[junction_id] = {"incoming": incoming_links, "outgoing": outgoing_links}

    return junction_info

def next_junction(road_id, junction_info):
    for key, value in junction_info.items():
        if road_id in value['incoming']:
            return key
    return "none"

def make_decision(df, junction_info, tti_thresold=3.5, tti_diff_thresold=2.8, final_speed=0.0, fixed_acceleration=-5):

    # df = df.where(df["SumoPerceivedObjectID"] != df['mySumoId']).dropna()
    color_green = (0, 255, 0)
    color_blue = (0, 0, 255)
    # print(df)
    ego = get_ego_info(df)
    # if ego['SPEED'] >= 0 and ego['SPEED'] <= 2 and traci.vehicle.getColor(ego['ID']) == color_blue:
    #     traci.vehicle.setSpeed(ego['ID'], 0.0)
    #     return -1

    ego_road_id = traci.vehicle.getRoadID(ego['ID']).split("_")[0]
    # print(traci.vehicle.getRoadID(ego['ID']))
    ego_junction_id = next_junction(ego_road_id, junction_info)

    if ego_junction_id not in ["none"]:
        junction_x, junction_y = traci.junction.getPosition(ego_junction_id)
        # ego_junction_pos = (junction_info[ego_junction_id]['x'], junction_info[ego_junction_id]['y'])
        ego_junction_pos = (junction_x, junction_y)
        ego_tti = compute_tti_manual(ego_junction_pos, (ego['XPOS'], ego['YPOS']), ego['SPEED'])

        if ego_tti < tti_thresold:
            list_id_danger = []
            df['diff_tti'] = abs(ego_tti - df.apply(lambda row:compute_tti_df(row, inter_pos=ego_junction_pos), axis=1))
            # print(df)
            list_id_danger_tti = df[df['diff_tti'] < tti_diff_thresold]['SumoPerceivedObjectID'].tolist()
            # print("list_id_danger_tti::", list_id_danger_tti)
            

            for id_danger in list_id_danger_tti:
                if id_danger in traci.vehicle.getIDList():
                    other_road_id = traci.vehicle.getRoadID(id_danger).split("_")[0]
                    other_junction_id = next_junction(other_road_id, junction_info)

                    # print(f"ego_road_id =  {ego_road_id} - ego_junction_id =  {ego_junction_id} - other_road_id =  {other_road_id} - other_junction_id =  {other_junction_id}")

                    if other_junction_id == ego_junction_id and other_road_id != ego_road_id:
                        list_id_danger.append(id_danger)


            if list_id_danger:    
                traci.vehicle.setColor(ego['ID'], color_blue)

                duration = get_slowdown_duration(ego['SPEED'], final_speed, fixed_acceleration)
                # print("duration:",duration)
                if list_id_danger:
                    print(f"Warning with the following vehicles: {list_id_danger}")
                # traci.vehicle.slowDown(ego['ID'], final_speed, math.ceil(duration))
                traci.vehicle.setSpeed(ego['ID'], 0)
                return -1
        
    if (traci.vehicle.getWaitingTime(ego['ID']) > 3.5):
        traci.vehicle.setColor(ego['ID'], color_green)
        traci.vehicle.setSpeed(ego['ID'], -1)
        return -1


    return 1
