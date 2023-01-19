# -*- coding: utf-8 -*-
"""
Created on Sun Nov 27 13:38:45 2022

@author: bking
"""
import matplotlib.pyplot as plt
env = env_vis

r=sum(env.get_totalReward())
print('Total Reward: ',str(r))


fc=sum(env.get_fuelConsumption())
print('Total Fuel Consumption: ', str(fc))

plt.figure(1)
plt.plot(env.get_speeds())
plt.figure(2)
plt.plot(env.get_accelerations())
plt.show()