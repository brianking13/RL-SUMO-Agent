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



class VehEnv(Env):
    
    def __init__(self,visualizer):
        
            #actions
            self.action_space = Discrete(48)
            
            #observations
            self.observation_space = Box(low=np.array([0,0,0,0,0]),high=np.array([1,1,1,1,1]))
            
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
           
           # for each intersection, reset the reward to 0
           reward = 0
           
           # initiate next inductor observation 
           inductor_names = traci.inductionloop.getIDList()
           inductor_info = {}
           for i in inductor_names:
               inductor_info[i]=[]
           
           
           
           # for 5000 seconds, run sim. Break if an intersection is passed
           for i in range(1,5000):
               
               traci.simulationStep()
               # time.sleep(.051)   
               
               # get fuel consumption for entire simulation
                # self.simFuelConsumptions()


               
               # for each inductor, find vehicles that passed over
               for i in inductor_names:
                    inductor_info[i]=self.getVehicleCrossings(inductor_info[i],i)
                 
                                 
               self.state, sec_reward, done, info,intersection = self.InterStep(action)
               if done == True:
                   break
               # accumulate the reward over time
               reward = reward + sec_reward
               if intersection == True:
                   break
               
        # find current and upcoming lane id's
        current_lane = traci.vehicle.getLaneID('v_0')
        next_lane = traci.lane.getLinks(current_lane)[0][0]
        
        
        # find number of vehicles that passed over inductors over the entire time step
        inductor_locations = {}
        for j in inductor_names:
            inductor_locations[traci.inductionloop.getLaneID(j)] = j
            inductor_info[j] =len(inductor_info[j])-1
        # print("Cars on inductor: " , inductor_info[inductor_locations[next_lane]])
        
        # find number of vehicles moving along upcoming stretch of corridor
        number_vehicles_upcoming = traci.lane.getLastStepVehicleNumber(current_lane)
        print('Upcoming Lane Car Number',number_vehicles_upcoming)
        
        self.state[len(self.state)-1] = number_vehicles_upcoming/20
        return self.state, reward, done, info
             
    
    
    def getVehicleCrossings(self,inductor_vehicles,name):
        vehicle = traci.inductionloop.getLastStepVehicleIDs(name)
        if vehicle not in inductor_vehicles:
            inductor_vehicles.append(vehicle)
        else:
            None
        
        return inductor_vehicles
    
    
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
            pos = traci.vehicle.getPosition('v_0')
                             
            
            # get data from each stop light for observations
            light_names = traci.trafficlight.getIDList()
            for i in light_names: 
                # print(str(i))
                
                # get distance to each intersection
                intersect = traci.junction.getPosition(i)
                distance_from_light = intersect[0]-pos[0]   #index is to grab x position since y pos doesn't matter
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

            
          
        ## create state observations
        
        # find all positive gaps
        pos_intersection_distances = [i for i in intersection_distances if i > 0]
        
        # find distance to closest intersection
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
        
        previous_gap = self.state[1]*1000
        if gap > previous_gap:
            # print('reached intersection')
            intersection = True
        else:
            intersection = False
        
        self.state=[v/self.max_speed,gap/1000,phase_time/100,phase,0] # 0 is a placeholder for traffic obs

        if self.sim_length <= 0 or vehicle_list == [] or traci.simulation.getTime() > 400:
            done = True
            # print("BOOTSTRAP HERE!!!!")
        else:
            done = False
            
            
        info = {}
        
        return self.state, reward, done, info, intersection
    
    
    
    def reward_function(self,v,fc):
        reward =  (5000/(fc-.1)-1)/8-0.05
        # reward = 1/(2.7**(0.005*(20-v)**2))-0.5 # put this in graphing calculator to see speed vs reward
        if v < .1 or v > 40:
            reward -= 10
        return reward
    
    def inductionCount(self):
        traci.inductionloop.getLastStepVehicleIDs("e1_0")
    
    def get_fuelConsumption(self):
        return self.fuel_consumption
    
    def get_simFuelConsumption(self):
        return self.sim_fuel_consumption
    
    def simFuelConsumptions(self):
        for vehicle in traci.vehicle.getIDList():
            self.sim_fuel_consumption += traci.vehicle.getFuelConsumption(vehicle)
    
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
        self.simFuelConsumptions()
        
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
        

        # edit traffic demand each run
        tree = ET.parse('routingCustom.rou.xml')
        root = tree.getroot()
        for nodes in root.iter('routes'):
            for flow in root.iter('flow'):
                number_cars =str(random.randint(0,1))
                # number_cars = "1"
                flow.attrib['number'] = number_cars
        tree.write('routingCustom.rou.xml')
             
        
        # compile editted network
        os.system('netconvert --node-files=networkCustom.nod.xml --edge-files=networkCustom.edg.xml \
                  --connection-files=networkCustom.con.xml  \
                      --tllogic-files=networkCustom.tll.xml  \
          --output-file=networkCustom.net.xml')
          
        traci.start([sumoBinary, "-n", "networkCustom.net.xml", "-r", "routingCustom.rou.xml", "--start", "--quit-on-end","-a","additionalsCustom.xml"])
        
        
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
        self.sim_fuel_consumption = 0

        self.state=[v/self.max_speed,0,0,0,0]

        return self.state
    

