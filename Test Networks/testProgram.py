# -*- coding: utf-8 -*-
"""
Created on Wed Jan 11 15:48:24 2023

@author: bking
"""

import traci
from sumolib import checkBinary
import time

try:
    traci.close()
except:
    pass

sumoBinary = checkBinary('sumo-gui') 

  
traci.start([sumoBinary, "-n", "testNet.net.xml", "-r", "testRoute.rou.xml", "--start", "--quit-on-end", "-a","testAdditionals.xml"])


vehicles =[]
fuel_consumption = 0

for i in range(1,130):
    traci.simulationStep()
    
    for vehicle in traci.vehicle.getIDList():
        fuel_consumption += traci.vehicle.getFuelConsumption(vehicle)

    
    
    vehicle = traci.inductionloop.getLastStepVehicleIDs("e1_0")
    if vehicle not in vehicles:
        vehicles.append(vehicle)
    else:
        None
       
    time.sleep(.1)

total_vehicles = len(vehicles)-1

print(total_vehicles, "vehicles passed over the induction loop")