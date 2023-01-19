# -*- coding: utf-8 -*-
"""
Created on Fri Nov 25 13:26:04 2022

@author: bking
"""

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
            # running simution in SUMO

            sumoBinary = checkBinary('sumo-gui') 
            traci.start([sumoBinary, "-n", "networkPlain.net.xml", "-r", "routing2.rou.xml", "--start", "--quit-on-end"])
            
            if visualizer == True:
                global speeds
                self.visualizer = True
                speeds = []
            else:
                self.visualizer = False
                
                
        
    def step(self,speed,time_step):
    # runs with random intersection 3 times
    
        # moving first intersection
        tree = ET.parse('networkPlain.nod.xml')
        root = tree.getroot()
        for nodes in root.iter('nodes'):
            for node in nodes.iter('node'):
                if node.attrib['id'] == 'J1':
                    # print(node.attrib['x'])
                    # intersect = random.randint(10,45)
                    intersect = 40
                    print(intersect)
                    node.attrib['x'] = str(intersect)
                    # print(node.attrib['x'])
        tree.write('networkPlain.nod.xml')       
        
        os.system('netconvert --node-files=networkPlain.nod.xml --edge-files=networkPlain.edg.xml \
                  --connection-files=networkPlain.con.xml  \
          --output-file=networkPlain.net.xml')
        
        
        
        
        # running simution in SUMO
        sumoBinary = checkBinary('sumo-gui')
        traci.start([sumoBinary, "-n", "networkPlain.net.xml", "-r", "routing2.rou.xml", "--start", "--quit-on-end"])
        
        step = 0
        fuel_consumption = []
        traci.simulationStep()
        phase_durations =[]
        phases= []
        while len(traci.vehicle.getIDList()) > 0:
            vehicle_list = traci.vehicle.getIDList()
            
            if 'v_0' in vehicle_list:
                
                # get data from each stop light
                pos = traci.vehicle.getPosition('v_0')
                light_names = traci.trafficlight.getIDList()
                # get vehicle speed
                v = traci.vehicle.getSpeed('v_0')
                print('Vehicle Speed: ', str(v))
                
                for i in light_names: 
                    print(str(i))
                    # get distance to each intersection
                    intersect = traci.junction.getPosition(i)
                    distance_from_light = intersect[0]-pos[0]
                    print('Distance to Intersection: ', str(distance_from_light))
                    # get current light state
                    light_states = traci.trafficlight.getPhase(i)
                    print('Light states: ', str(light_states))
                    # get times until next switch
                    time_to_switch = traci.trafficlight.getNextSwitch(i) - traci.simulation.getTime()
                    print('Time to switch: ', str(time_to_switch))
                    # get phases durations
                    phase_duration = traci.trafficlight.getPhaseDuration(i)
                    print('Phase Duration: ',str(phase_duration))
                    phase_durations.append(phase_duration)
                    # get current phases
                    phase = traci.trafficlight.getRedYellowGreenState(i)
                    current_phases = phase[10]
                    phases.append(phase[10])
                    print('Current Phase: ',str(current_phases),"\n")
                  
                
                traci.vehicle.setSpeed('v_0',speed)
                fuel_consumption.append(traci.vehicle.getFuelConsumption('v_0'))
            traci.simulationStep()
            time.sleep(time_step)
            
        traci.close()
        #print(fuel_consumption)
        # print(sum(fuel_consumption))
        return fuel_consumption, phase_durations,phases
    
    def close(self):
        traci.close()
    
    def showSpeed(self):
        return speeds
        
    def render(self):
        pass
    
    def reset(self):
        # close running session
        # running simution in SUMO
        traci.close()
        sumoBinary = checkBinary('sumo-gui') 
        traci.start([sumoBinary, "-n", "networkPlain.net.xml", "-r", "routing2.rou.xml", "--start", "--quit-on-end"])
        traci.simulationStep()
        
        if self.visualizer:
            speeds=[]
            
        self.sim_length = 50
        traci.vehicle.setMaxSpeed('v_0',15)
        speed = traci.vehicle.getSpeed('v_0')

        self.state=[speed/30]
        return self.state
    


visualizer = True
env = VehEnv(visualizer)
# [f,p,v] = env.step(10,0.01)
