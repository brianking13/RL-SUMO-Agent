# -*- coding: utf-8 -*-
"""
Created on Sun Nov 27 12:38:40 2022

@author: bking
"""

from velocity_env import VehEnv
import traci
import matplotlib.pyplot as plt
import time

try:
    traci.close()
except:
    pass

visualize = True
env_base = VehEnv(visualize)
env_base.reset()

# run simulation for 800 steps (1 step = 1 second)
for i in range(400):
    env_base.step_constantSpeed(20)
    time.sleep(.01)
plt.plot(env_base.get_speeds())
plt.ylabel('Speed')
plt.xlabel('Time')
plt.show()

fc_base = env_base.get_fuelConsumption()
reward_base = env_base.get_totalReward()

print('Fuel Consumption: ', sum(fc_base))
print('Reward: ', sum(reward_base))
print('Distance: ', traci.vehicle.getDistance('v_0'))
print('FuelCon per Meter: ',sum(fc_base)/traci.vehicle.getDistance('v_0'))

import csv
fc_speeds = env_base.get_speeds()
with open('ExampleData.csv', 'w',newline="") as f:
      
    # using csv.writer method from CSV package
    write = csv.writer(f)
    for item in fc_speeds :
        write.writerow([item])