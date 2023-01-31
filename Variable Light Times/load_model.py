# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 21:29:10 2023

@author: bking
"""


import random

# change this to change vehicle RL type
from noTraffic_env import VehEnv
# from noTraffic_env_everyTimeStep import VehEnv
import matplotlib.pyplot as plt
import traci

# imports for RL
import numpy as np
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten
from tensorflow.keras.optimizers import Adam
from tensorflow import keras
import keras_tuner as kt

from rl.agents import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.policy import EpsGreedyQPolicy
from rl.policy import LinearAnnealedPolicy
from rl.memory import SequentialMemory


# change this to the name and extension of the file
# filename ='model_velocity-based_20.h5'
filename = 'model_acceleration_based_20.h5'


visualization = False
env = VehEnv(visualization)


states = env.observation_space.shape
actions = env.action_space.n

def build_model(states, actions):
    model = Sequential()
    model.add(Flatten(input_shape=(1,states[0])))
    model.add(Dense(128, activation='relu', input_shape = (1,states[0])))
    model.add(Dense(128, activation='relu'))
    model.add(Dense(128, activation='relu'))
    model.add(Dense(128, activation='relu'))
    model.add(Dense(128, activation='relu'))
    model.add(Dense(actions, activation ='linear'))
    return model

model = keras.models.load_model(filename)


def build_agent(model, actions):
    # policy = BoltzmannQPolicy()
    # policy = EpsGreedyQPolicy(eps=0.1)
    policy = LinearAnnealedPolicy(EpsGreedyQPolicy(), attr='eps', value_max=1, value_min=.1, value_test=.00001,
                                  nb_steps=10000)
    memory = SequentialMemory(limit=100000, window_length =1)
    dqn = DQNAgent(model=model, memory=memory, policy= policy, nb_actions=actions, nb_steps_warmup=100, target_model_update = 1e-3)
    return dqn

dqn = build_agent(model, actions)
dqn.compile(Adam(learning_rate=1e-3), metrics = ['mae'])

try:
    env.close()
except:
    pass

x= []
for i in range(1,5):
    visualization = True
    env_vis = VehEnv(visualization)
    test1 = dqn.test(env_vis, nb_episodes = 1, visualize=False)
    plt.plot(env_vis.get_speeds())
    plt.ylabel('Speed')
    plt.xlabel('Time')
    plt.show()
    
    fc_vis = env_vis.get_fuelConsumption()
    reward_vis= env_vis.get_totalReward()
    print('Fuel Consumption: ', sum(fc_vis))
    print('Reward: ', sum(reward_vis))
    print('Distance: ', traci.vehicle.getDistance('v_0'))
    print('FuelCon per Meter: ',sum(fc_vis)/traci.vehicle.getDistance('v_0'))
    x.append(sum(fc_vis)/traci.vehicle.getDistance('v_0'))
    print("")
    
    
import csv
fc_speeds = env_vis.get_speeds()
with open('ExampleData.csv', 'w',newline="") as f:
      
    # using csv.writer method from CSV package
    write = csv.writer(f)
    for item in fc_speeds :
        write.writerow([item])