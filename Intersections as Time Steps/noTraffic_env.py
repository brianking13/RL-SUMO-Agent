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
            self.action_space = Discrete(48)
            
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
                self.distance=[]
            else:
                self.visualizer = False
                
                
     
    def step(self,action):
        
        # decrease sim length by one for each step
        self.sim_length -= 1
        # get a list of all vehicles
        vehicle_list = traci.vehicle.getIDList()
        
        # if vehicle exists, set vehicle speed based on action and retrieve data
        if 'v_0' in vehicle_list:
           traci.vehicle.setSpeed('v_0',action+1)
           print(action+1)
           
           # for each intersection, reset the reward to 0
           reward = 0
           
           # for 5000 seconds, run sim. Break if an intersection is passed
           for i in range(1,5000):
               traci.simulationStep()
               self.state, sec_reward, done, info,intersection = self.InterStep(action)
               if done == True:
                   break
               # accumulate the reward over time
               reward = reward + sec_reward
               if intersection == True:
                   break
               
        print(self.state)        
        return self.state, reward, done, info
             
    
    def InterStep(self,action):
        
        
        # get list of all vehicles
        vehicle_list = traci.vehicle.getIDList()
        
        # initialize variables
        reward = 0
        intersection_distances = []
        phase_durations =[]
        phases= []

        
        if 'v_0' in vehicle_list:
            
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
            v = 0 
        
        # reward function
        fc=traci.vehicle.getFuelConsumption('v_0') 
        reward = self.reward_function(v,fc)
        accel = traci.vehicle.getAcceleration('v_0')
        
        if self.visualizer== True:
            time.sleep(.01)
            self.data_collection(v, accel, fc, reward)
            # print('fc: ',str(fc))
            # print('speed',str(v))
            # print('accel', str(accel))
            # print('reward', str(reward))
            # print('')

            
            
        # create state observations
        
        # find all positive gaps
        pos_intersection_distances = [i for i in intersection_distances if i > 0]
        
        # find distance to closest intersection
        gap = min(pos_intersection_distances)
        
        # NEXT PHASE TIME and NEXT PHASE
        index = intersection_distances.index(gap) 
        current_light = light_names[index]
        phase_time = traci.trafficlight.getNextSwitch(current_light) - traci.simulation.getTime()

        current_phase = traci.trafficlight.getRedYellowGreenState(current_light)
        print(current_phase)
        if current_phase == 'RR' or current_phase == 'rr':
            phase = 0
        else:
            phase = 1

        previous_gap = self.state[1]*1000
        if gap > previous_gap:
            # print('reached intersection')
            intersection = True
        else:
            intersection = False
        
        # print("Phase: ",phase)
        # print("Phase time: ",phase_time)
        # print("Gap: ",gap)
        
        self.state=[v/self.max_speed,gap/1000,phase_time/100,phase]

        if self.sim_length <= 0 or vehicle_list == [] or traci.simulation.getTime() > 400:
            done = True
            # print("BOOTSTRAP HERE!!!!")
        else:
            done = False
            
            
        info = {}
        
        return self.state, reward, done, info, intersection
    
    
    
    def reward_function(self,v,fc):
        # reward =  (5000/(fc/(v+.01)-.01)-1)/50-0.01
        # reward =  1/(2.7**(0.05*(fc/500)**2))
        reward = 1/(2.7**(0.005*(20-v)**2))-0.5 # put this in graphing calculator to see speed vs reward
        if v < .1 or v > 40:
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
    
    def get_Distance(self):
        return self.distance
    
    def data_collection(self,v,accel,fc,reward):
        self.speeds.append(v)
        self.accels.append(accel)
        self.fuel_consumption.append(fc)
        self.total_reward.append(reward)
        self.distance.append(traci.vehicle.getPosition('v_0'))
    
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
            
        # edit network each run
        tree = ET.parse('networkCustom.nod.xml')
        root = tree.getroot()
        for nodes in root.iter('nodes'):
            for intersection_num in range(1,17):
                for node in nodes.iter('node'):
                    if node.attrib['id'] == 'J' + str(intersection_num):
                        # print(intersection_num)
                        # print(node.attrib['x']) 
                        intersect = random.randint(800+1000*(intersection_num-1),1200+1000*(intersection_num-1))
                        # print(intersect)
                        node.attrib['x'] = str(intersect)
                        # print(node.attrib['x'])
        tree.write('networkCustom.nod.xml')             
     
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
            self.distance=[]
 
        self.sim_length = 12
        self.max_speed = 40
        traci.vehicle.setMaxSpeed('v_0',self.max_speed)
        v = traci.vehicle.getSpeed('v_0')

        self.state=[v/self.max_speed,0,0,0]

        return self.state
    


# visualizer = True
# env = VehEnv(visualizer)
# # [f,p,v] = env.step(10,0.01)
# for i in range(1,20):
#     env.step(20)