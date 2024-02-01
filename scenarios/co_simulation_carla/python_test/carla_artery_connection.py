"""
Script to integrate recieve artery messages in python for each simulation step
"""
# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

from ast import Try
import numpy as np
import errno, time
import socket
import select
import carla
import traci
import logging
import traceback
import json
import pandas as pd

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position

# HOST = '127.0.0.1'  
# PORT = 65432      
RX_TYPE = 0  
TX_TYPE = 1
EXTPER_TYPE = 2



class ArterySynchronization(object):

    def __init__(self, host, port):
        self.conn = None
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host, port))
        self.s.setblocking(0)
        self.s.listen(5)
        self.on_going_message_recv={"state":False}
        self.port = port

    def checkAndConnectclient(self):
        if not self.is_connected():
            try :
                readable, _, _ = select.select([self.s], [], [],0.01)
                if readable:
                    print("connecting artery client::", self.port)
                    self.conn, _ = self.s.accept() 
                    self.conn.setblocking(0)
            except Exception as e:
                print("Socket check problem")
                logging.error(traceback.format_exc())    
                
    def is_connected(self):
        return not (self.conn is None)
        
    def getCarlaLocation(self,synchronization,cpm, trans_type):
        if(trans_type == RX_TYPE):
            if (cpm.get('receiver_long') and cpm.get('receiver_lat')):
                x,y = synchronization.net.convertLonLat2XY(cpm['receiver_long'],cpm['receiver_lat'])
                extent    = carla.Vector3D(cpm['Vehicle_Width'], cpm['Vehicle_Length'],)
                transform = carla.Transform(carla.Location(x, y, 0),
                                        carla.Rotation(yaw=cpm['receiver_yaw']))
                receiver_carla_transform = BridgeHelper.get_carla_transform(transform, extent)
                cpm['receiver_pos_x']=receiver_carla_transform.location.x
                cpm['receiver_pos_y']=receiver_carla_transform.location.y

            if (cpm.get('sender_long') and cpm.get('sender_lat')):
                x,y = synchronization.net.convertLonLat2XY(cpm['sender_long'],cpm['sender_lat'])
                extent    = carla.Vector3D(cpm['Vehicle_Width'], cpm['Vehicle_Length'],)
                transform = carla.Transform(carla.Location(x, y, 0),
                                        carla.Rotation(yaw=cpm['sender_yaw']))
                sender_carla_transform = BridgeHelper.get_carla_transform(transform, extent)
                cpm['sender_pos_x']=sender_carla_transform.location.x
                cpm['sender_pos_y']=sender_carla_transform.location.y
        elif(trans_type == TX_TYPE):
            if(cpm.get('attacked_obj_long') and cpm.get('attacked_obj_lat')):
                attacked_obj_x = []
                attacked_obj_y = []
                for i in range(len(cpm['attacked_obj_sumo_ids'])):
                    # print("cpm attacked obj long lat::",cpm['attacked_obj_long'][i],cpm['attacked_obj_lat'][i])
                    x,y = synchronization.net.convertLonLat2XY(cpm['attacked_obj_long'][i],cpm['attacked_obj_lat'][i])
                    extent    = carla.Vector3D(cpm['Vehicle_Width'], cpm['Vehicle_Length'],)
                    transform = carla.Transform(carla.Location(x, y, 0),
                                    carla.Rotation(yaw=cpm['attacked_obj_yaw'][i]))
                    obj_carla_transform = BridgeHelper.get_carla_transform(transform, extent)
                    attacked_obj_x.append(obj_carla_transform.location.x)
                    attacked_obj_y.append(obj_carla_transform.location.y)
                cpm['attacked_obj_x'] = attacked_obj_x
                cpm['attacked_obj_y'] = attacked_obj_y

            if(cpm.get('attacked_obj_real_long') and cpm.get('attacked_obj_real_lat')):
                attacked_obj_x = []
                attacked_obj_y = []
                for i in range(len(cpm['attacked_obj_sumo_ids'])):
                    x,y = synchronization.net.convertLonLat2XY(cpm['attacked_obj_real_long'][i],cpm['attacked_obj_real_lat'][i])

                    # print("attacked_obj_real_geo::", cpm['attacked_obj_real_long'][i],cpm['attacked_obj_real_lat'][i])
                    extent    = carla.Vector3D(cpm['Vehicle_Width'], cpm['Vehicle_Length'],)
                    transform = carla.Transform(carla.Location(x, y, 0),
                                    carla.Rotation(yaw=cpm['attacked_obj_yaw'][i]))
                    obj_carla_transform = BridgeHelper.get_carla_transform(transform, extent)
                    attacked_obj_x.append(obj_carla_transform.location.x)
                    attacked_obj_y.append(obj_carla_transform.location.y)
                    print("attacked_obj_real_xy2::", x,y)
                cpm['attacked_obj_real_x'] = attacked_obj_x
                cpm['attacked_obj_real_y'] = attacked_obj_y
                # print("attacked_obj_real_xy::", cpm['attacked_obj_real_x'],cpm['attacked_obj_real_y'])
                



    def TXcpmToDict(self,synchronization,cpm):
        full_split=[x.split(':') for x in cpm.split('\n')]
        full_cpm= {k:v for k,v in full_split[:-1]}

        full_cpm['sender_artery_id']=int(full_cpm['sender_artery_id'])
        full_cpm['sender_sumo_id']=str(full_cpm['sender_sumo_id'])
        full_cpm['Vehicle_Length'] = float(full_cpm['Vehicle_Length'])
        full_cpm['Vehicle_Width'] = float(full_cpm['Vehicle_Width'])
        full_cpm['sender_type'] = int(full_cpm['sender_type'])
        full_cpm['attack_type'] = int(full_cpm['attack_type'])


        if(full_cpm['sender_type'] == 1 or full_cpm['sender_type'] == 3):
                full_cpm['attacked_obj_sumo_ids'] = ([str(x) for x in full_cpm['attacked_obj_sumo_ids'].split(',')[:-1]])
                full_cpm['attacked_obj_long'] = [np.double(x) for x in full_cpm['attacked_obj_long'].split(',')[:-1]]
                full_cpm['attacked_obj_lat'] = [np.double(x) for x in full_cpm['attacked_obj_lat'].split(',')[:-1]]
                full_cpm['attacked_obj_yaw'] = [np.double(x) for x in full_cpm['attacked_obj_yaw'].split(',')[:-1]]

                if (full_cpm.get('attacked_obj_real_long') and full_cpm.get('attacked_obj_real_lat')):
                    full_cpm['attacked_obj_real_long'] = [np.double(x) for x in full_cpm['attacked_obj_real_long'].split(',')[:-1]]
                    full_cpm['attacked_obj_real_lat'] = [np.double(x) for x in full_cpm['attacked_obj_real_lat'].split(',')[:-1]]
                
        # adding transformations
        self.getCarlaLocation(synchronization,full_cpm,TX_TYPE)

        return full_cpm


    def RXcpmToDict(self,synchronization,cpm):
        full_split=[x.split(':') for x in cpm.split('\n')]
        full_cpm= {k:v for k,v in full_split[:-1]}
        
        full_cpm['sender_artery_id']=int(full_cpm['sender_artery_id'])
        full_cpm['sender_sumo_id']=str(full_cpm['sender_sumo_id'])
        full_cpm['sender_long']= float(full_cpm['sender_long'])
        full_cpm['sender_lat']= float(full_cpm['sender_lat'])
        full_cpm['Vehicle_Length'] = float(full_cpm['Vehicle_Length'])
        full_cpm['Vehicle_Width'] = float(full_cpm['Vehicle_Width'])
        full_cpm['receiver_artery_id']=int(full_cpm['receiver_artery_id'])
        full_cpm['receiver_sumo_id']=str(full_cpm['receiver_sumo_id'])
        full_cpm['receiver_long']= np.double(full_cpm['receiver_long'])
        full_cpm['receiver_lat']= np.double(full_cpm['receiver_lat'])
        full_cpm['receiver_speed']= np.double(full_cpm['receiver_speed'])
        full_cpm['receiver_type'] = int(full_cpm['receiver_type'])
        full_cpm['sender_yaw'] = float(full_cpm['sender_yaw'])
        full_cpm['receiver_yaw'] = float(full_cpm['receiver_yaw'])

        # adding transformations
        self.getCarlaLocation(synchronization,full_cpm,RX_TYPE)

        return full_cpm


    def ExtentedPerceptionToDict(self,extendedperceptiondata):
        extpercept = json.loads(extendedperceptiondata)
        df = pd.DataFrame(extpercept) 
        return extpercept



    def get_ongoing_recv( self ):
        if  self.on_going_message_recv['state'] :
            data = self.conn.recv(self.on_going_message_recv['full_message_size']-len(self.on_going_message_recv['fragment']))
            self.on_going_message_recv['fragment']=self.on_going_message_recv['fragment']+data
            if len(self.on_going_message_recv['fragment'])!= self.on_going_message_recv['full_message_size']:
                self.on_going_message_recv['state']=True
            else:
                self.on_going_message_recv['state']=False
        return self.on_going_message_recv['state']


    def set_ongoing_recv( self,data,size ):
        if len(data)!= size:
            self.on_going_message_recv['state']=True
            self.on_going_message_recv['fragment']=data
            self.on_going_message_recv['full_message_size']=size
        return self.on_going_message_recv['state']


    def recieve_cpm_messages(self,synchronization, trans_type):
        current_step_msg=[]
        while True:
            data = None
            try :
                if self.on_going_message_recv['state']:
                    if  self.get_ongoing_recv():
                        break
                    else:
                        data = self.on_going_message_recv['fragment']
                        if self.on_going_message_recv['full_message_size'] ==6 :
                            next_cpm_size = int(data.decode('utf-8'))
                            data = self.conn.recv(next_cpm_size)
                            if self.set_ongoing_recv(data,next_cpm_size):
                                continue
                else :
                    data = self.conn.recv(6)
                    if len(data)<=0:
                        break
                    if self.set_ongoing_recv(data,6):
                        continue
                    next_cpm_size = int(data.decode('utf-8'))
                    data = self.conn.recv(next_cpm_size)
                    if self.set_ongoing_recv(data,next_cpm_size):
                        continue

            except IOError as e:
                if not data is None :
                    if len(data) == 6:
                        self.set_ongoing_recv(b'', int(data.decode('utf-8')))
                if e.errno != errno.EWOULDBLOCK: 
                    print("e.errno != errno.EWOULDBLOCK",e)
                    logging.error(traceback.format_exc())
                    print(data)
                break

            if(trans_type == RX_TYPE):
                full_cpm = self.RXcpmToDict(synchronization,data.decode('utf-8'))
            elif(trans_type == TX_TYPE):
                full_cpm = self.TXcpmToDict(synchronization,data.decode('utf-8'))
            elif(trans_type == EXTPER_TYPE):
                full_cpm = self.ExtentedPerceptionToDict(data.decode('utf-8'))
                # return full_cpm

            current_step_msg.append(full_cpm)
        return current_step_msg

    def shutdownAndClose(self):
        try :
            self.s.shutdown(socket.SHUT_RDWR)
            self.s.close()
        except OSError as e :
            print("trying to close already closed socket")
            