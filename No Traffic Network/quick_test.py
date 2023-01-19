# -*- coding: utf-8 -*-
"""
Created on Sat Nov 26 11:44:25 2022

@author: bking

"""
from noTraffic_env import VehEnv
import traci

try:
    traci.close()
except:
    pass

env = VehEnv(True)
env.reset()

for i in range(2000):
    env.step(15)

