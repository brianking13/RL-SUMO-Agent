# -*- coding: utf-8 -*-
"""
Created on Sun Nov 27 12:38:40 2022

@author: bking
"""

from noTraffic_env import VehEnv
import traci

try:
    traci.close()
except:
    pass

visualize = True
env_base = VehEnv(visualize)
env_base.reset()

for i in range(800):
    env_base.step_constantSpeed(10)