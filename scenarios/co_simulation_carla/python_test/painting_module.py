"""
Script to inject attacks in recieved artery messages in python
"""
# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================
import carla

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position


class Painter(object):

    def __init__(self, synchronization, victim_color=carla.Color(255,255,0) , attacker_color=carla.Color(255,0,0),
                    attacker_detected_color = carla.Color(255,255,255),benign_color=carla.Color(0,255,0), attackedObj_color = carla.Color(0,0,255)):
        self.attacker_color = attacker_color
        self.benign_color = benign_color
        self.attacker_detected_color = attacker_detected_color
        self.victim_color = victim_color
        self.attackedObj_color = attackedObj_color
        self.vehicle_box_dims=carla.Vector3D(1,1,1)
        self.detection_box_dims=carla.Vector3D(0.05,0.05,0.05)
        self.synchronization = synchronization

    # def color_agents(self,synchronization, benign, malicious, freq=0.1):
    #     coloring_brush = synchronization.carla.world.debug
    #     self.colorV2XCar(coloring_brush,synchronization,benign, malicious,freq)
        # self.colorDetections(coloring_brush,synchronization,detections)

    def color_communication(self,cpm, freq=0.3):
        coloring_brush = self.synchronization.carla.world.debug
        if cpm.get('sender_pos_x') and cpm.get('receiver_pos_x'):
            coloring_brush.draw_arrow(
                carla.Location(cpm['sender_pos_x'],cpm['sender_pos_y'],1),
                carla.Location(cpm['receiver_pos_x'],cpm['receiver_pos_y'],1),thickness=0.2,arrow_size=0.1,life_time=freq,color=carla.Color(255,0,0))
            # box_location = carla.Location(cpm['receiver_pos_x'],cpm['receiver_pos_y'],2)
            # coloring_brush.draw_box(carla.BoundingBox(box_location,self.vehicle_box_dims),carla.Rotation(), 1, self.attackedObj_color, freq)


    def color_attackedObj(self, cpm, freq= 0.3):
        coloring_brush = self.synchronization.carla.world.debug

        GHOST = 4
        if cpm.get('attacked_obj_sumo_ids') and cpm['attack_type'] != GHOST:
            victim_carla_ids = [self.synchronization.sumo2carla_ids.get(actor_id) for actor_id in cpm['attacked_obj_sumo_ids']]
            for victim_id in victim_carla_ids:
                try:
                    victim_actor = self.synchronization.carla.world.get_actor(victim_id)
                    coloring_brush.draw_box(carla.BoundingBox(victim_actor.get_transform().location,self.vehicle_box_dims),victim_actor.get_transform().rotation, 1, self.victim_color, freq)
                    print("victim_actor xy::", victim_actor.get_transform().location.x, victim_actor.get_transform().location.y)
                except Exception as e:
                    print(e)

        if cpm.get("attacked_obj_x") and cpm.get('attacked_obj_y'):   
            for index in range(len(cpm['attacked_obj_x'])):
                box_location = carla.Location(cpm['attacked_obj_x'][index], cpm['attacked_obj_y'][index], 0.5)
                #yaw=cpm['attacked_obj_yaw'][index]
                coloring_brush.draw_box(carla.BoundingBox(box_location,self.vehicle_box_dims), carla.Rotation(), 1, self.attackedObj_color, freq)
                print("attacked obj xy::", cpm['attacked_obj_x'][index], cpm['attacked_obj_y'][index])

        if cpm.get("attacked_obj_real_x") and cpm.get('attacked_obj_real_y') and cpm['attack_type'] != GHOST:   
            for index in range(len(cpm['attacked_obj_sumo_ids'])):
                coloring_brush.draw_arrow(
                        carla.Location(cpm['attacked_obj_real_x'][index],cpm['attacked_obj_real_y'][index],1),
                        carla.Location(cpm['attacked_obj_x'][index],cpm['attacked_obj_y'][index],1),thickness=0.2,arrow_size=0.1,life_time=freq,color=carla.Color(255,255,0))
                print("arrow victim_actor xy::", cpm['attacked_obj_real_x'][index], cpm['attacked_obj_real_y'][index])

    def color_agents(self, benign, malicious, malicious_detected,freq=0.1):
        coloring_brush = self.synchronization.carla.world.debug
        benign_carla_ids = [self.synchronization.sumo2carla_ids.get(actor_id) for actor_id in benign]
        malicious_carla_ids = [self.synchronization.sumo2carla_ids.get(actor_id) for actor_id in malicious]
        malicious_detected_carla_ids = [self.synchronization.sumo2carla_ids.get(actor_id) for actor_id in malicious_detected]
        for benign_id in benign_carla_ids:
                try:
                    bengin_actor= self.synchronization.carla.world.get_actor(benign_id)
                    coloring_brush.draw_box(carla.BoundingBox(bengin_actor.get_transform().location,self.vehicle_box_dims),bengin_actor.get_transform().rotation, 1, self.benign_color, freq)
                except:
                     pass
                     
        for malicious_id in malicious_carla_ids:
                try:
                    malicious_actor= self.synchronization.carla.world.get_actor(malicious_id)
                    coloring_brush.draw_box(carla.BoundingBox(malicious_actor.get_transform().location,self.vehicle_box_dims),malicious_actor.get_transform().rotation, 1, self.attacker_color, freq)
                except:
                     pass

        for malicious_detected_id in malicious_detected_carla_ids:
                try:
                    malicious_detected_actor= self.synchronization.carla.world.get_actor(malicious_detected_id)
                    coloring_brush.draw_box(carla.BoundingBox(malicious_detected_actor.get_transform().location,self.vehicle_box_dims),malicious_detected_actor.get_transform().rotation, 1, self.attacker_detected_color, freq)
                except:
                     pass

    # def colorDetections(self,coloring_brush,synchronization,detections):
    #     for actor_id in detections:
    #         if synchronization.sumo2carla_ids.get(actor_id):
    #             actor=synchronization.carla.world.get_actor(synchronization.sumo2carla_ids.get(actor_id))
    #             coloring_brush.draw_box(carla.BoundingBox(actor.get_transform().location+carla.Vector3D(0,0,5),self.detection_box_dims),actor.get_transform().rotation, 3, self.detection_color,self.freq)
    #         else :
    #             ghost_carla_id =list(synchronization.carla2sumo_ids.keys())[list(synchronization.carla2sumo_ids.values()).index(actor_id)]
    #             actor=synchronization.carla.world.get_actor(ghost_carla_id)
    #             coloring_brush.draw_box(carla.BoundingBox(actor.get_transform().location+carla.Vector3D(0,0,5),self.detection_box_dims),actor.get_transform().rotation, 3, self.detection_color,self.freq)
