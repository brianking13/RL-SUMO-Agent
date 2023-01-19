# -*- coding: utf-8 -*-
"""
Created on Fri Nov 25 12:12:57 2022

@author: bking
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Nov 14 21:35:39 2022

@author: bking
"""


# imports for SUMO and general script
import traci
from sumolib import checkBinary
import time
import random
import xml.etree.ElementTree as ET
import os
import numpy as np

#imports for RL
from gym import Env
from gym.spaces import Discrete, Box



# for i in range(1,20):
#     traci.simulationStep()

# tree = ET.parse('network1.net.xml')
# root = tree.getroot()
# for junctions in root.iter('junction'):
#     if junctions.attrib['id'] == 'J1':
#         print(junctions.attrib['x'])
#         junctions.attrib['x'] = '10.00'
#         print(junctions.attrib['x'])

# tree.write('networkVariable.net.xml')    


class VehEnv(Env):
    
    def __init__(self,visualizer):
        
            #actions
            self.action_space = Discrete(22)
            
            #observations
            self.observation_space = Box(low=np.array([0,0,0,0]),high=np.array([1,1,1,1]))
            
            # intialize parameters
            self.fuel_consumption = []

            if visualizer == True:
                self.visualizer= True
                self.speeds=[]
                self.accels=[]
                self.total_reward = []
                self.fuel_consumption = []   
            else:
                self.visualizer = False
                
                
        
    def step(self,action):
    
        # # moving first intersection
        # tree = ET.parse('networkPlain.nod.xml')
        # root = tree.getroot()
        # for nodes in root.iter('nodes'):
        #     for node in nodes.iter('node'):
        #         if node.attrib['id'] == 'J1':
        #             # print(node.attrib['x'])
        #             # intersect = random.randint(10,45)
        #             intersect = 40
        #             print(intersect)
        #             node.attrib['x'] = str(intersect)
        #             # print(node.attrib['x'])
        # tree.write('networkPlain.nod.xml')       
        
        # os.system('netconvert --node-files=networkPlain.nod.xml --edge-files=networkPlain.edg.xml \
        #           --connection-files=networkPlain.con.xml  \
        #   --output-file=networkPlain.net.xml')
        
        
        vehicle_list = traci.vehicle.getIDList()
        self.sim_length -= 1
        reward = 0
        traci.simulationStep()
        
        intersection_distances = []
        phase_durations =[]
        phases= []

        
        
        if 'v_0' in vehicle_list:
            
            
            
            #calculate acceleration
            accel = 0.25*action-3
            traci.vehicle.setAcceleration('v_0',accel*2,1)
            v = traci.vehicle.getSpeed('v_0')
            
            # get data from each stop light
            pos = traci.vehicle.getPosition('v_0')
            light_names = traci.trafficlight.getIDList()

            
            for i in light_names: 
                # print(str(i))
                
                # get distance to each intersection
                intersect = traci.junction.getPosition(i)
                distance_from_light = intersect[0]-pos[0]
                intersection_distances.append(distance_from_light)
                
                # print('Distance to Intersection: ', str(distance_from_light))
                # # get current light state
                # light_states = traci.trafficlight.getPhase(i)
                # print('Light states: ', str(light_states))
                
                # # get times until next switch
                # time_to_switch = traci.trafficlight.getNextSwitch(i) - traci.simulation.getTime()
                # print('Time to switch: ', str(time_to_switch))
                
                # # get phases durations
                # phase_duration = traci.trafficlight.getPhaseDuration(i)
                # print('Phase Duration: ',str(phase_duration))
                # phase_durations.append(phase_duration)
                
                # # get current phases
                # phase = traci.trafficlight.getRedYellowGreenState(i)
                # current_phases = phase[10]
                # phases.append(phase[10])
                # print('Current Phase: ',str(current_phases),"\n")
        else:
            speed = 0 
        
        # reward function
        fc=traci.vehicle.getFuelConsumption('v_0') 
        reward = self.reward_function(v,fc)
        
        if self.visualizer== True:
            time.sleep(.01)
            self.data_collection(v, accel, fc, reward)
            print('fc: ',str(fc))
            print('speed',str(v))
            print('accel', str(accel))
            print('reward', str(reward))
            print('')
            
            
        # create state observations
        # GAP
        pos_intersection_distances = [i for i in intersection_distances if i > 0]
        gap = min(pos_intersection_distances)
        
        # NEXT PHASE TIME and NEXT PHASE
        index = intersection_distances.index(gap) 
        current_light = light_names[index]
        phase_time = traci.trafficlight.getNextSwitch(current_light) - traci.simulation.getTime()
        phases = traci.trafficlight.getRedYellowGreenState(i)
        # current_phase = phases[10]
        current_phase = phases[1]
        if current_phase == 'R' or current_phase == 'r':
            phase = 0
        else:
            phase = 1
        
        
        self.state=[v/self.max_speed,gap/1000,phase_time/100,phase]

        if self.sim_length <= 0 or vehicle_list == []:
            done = True
            print("BOOTSTRAP HERE!!!!")
        else:
            done = False
            
            
        info = {}
        
        return self.state, reward, done, info
    
    
    
    def reward_function(self,v,fc):
        reward =  1/(2.7**(0.05*(fc/500)**2))
        reward = 1/(2.7**(0.05*(10-v)**2))*.1
        if v < .1 or v > 13:
            reward -= 1
        return reward
        
    
    def get_fuelConsumption(self):
        return self.fuel_consumption
    
    def get_speeds(self):
        return self.speeds
    
    def get_totalReward(self):
        return self.total_reward
    
    def get_accelerations(self):
        return self.accels
    
    def data_collection(self,v,accel,fc,reward):
        self.speeds.append(v)
        self.accels.append(accel)
        self.fuel_consumption.append(fc)
        self.total_reward.append(reward)
    
    def step_constantSpeed(self,set_speed):
        
        vehicle_list = traci.vehicle.getIDList()
        if 'v_0' in vehicle_list:
            traci.vehicle.setSpeed('v_0',set_speed)
            
        traci.simulationStep()
        
        fc=traci.vehicle.getFuelConsumption('v_0')
        accel = traci.vehicle.getAcceleration('v_0')
        v = traci.vehicle.getSpeed('v_0')
        reward = self.reward_function(v, fc)
        
        if self.visualizer== True:
            time.sleep(.01)
            self.data_collection(v, accel, fc, reward)
            # print('fc: ',str(fc))
            # print('speed',str(v))
            # print('accel', str(accel))
            # print('reward', str(reward))
            # print('')
    
    def close(self):
        traci.close()
        
    def render(self):
        pass
    
    def reset(self):
        # close running session
        # running simution in SUMO
        try:
            traci.close()
        except:
            pass
        
        if self.visualizer == False:
            sumoBinary = checkBinary('sumo') 
        else:
            sumoBinary = checkBinary('sumo-gui') 
     
        os.system('netconvert --node-files=networkCustom.nod.xml --edge-files=networkCustom.edg.xml \
                  --connection-files=networkCustom.con.xml  \
                      --tllogic-files=networkCustom.tll.xml  \
          --output-file=networkCustom.net.xml')
        traci.start([sumoBinary, "-n", "networkCustom.net.xml", "-r", "routingLarge.rou.xml", "--start", "--quit-on-end"])
        traci.simulationStep()
        
        
        if self.visualizer:
            self.speeds=[]
            self.accels=[]
            self.total_reward = []
            self.fuel_consumption = []        
 
        self.sim_length = 800
        self.max_speed = 15
        traci.vehicle.setMaxSpeed('v_0',self.max_speed)
        v = traci.vehicle.getSpeed('v_0')

        self.state=[v/self.max_speed,0,0,0]
        return self.state
    


# visualizer = True
# env = VehEnv(visualizer)
# # [f,p,v] = env.step(10,0.01)
# for i in range(1,20):
#     env.step(20)