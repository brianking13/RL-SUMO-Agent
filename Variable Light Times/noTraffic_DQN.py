# -*- coding: utf-8 -*-
"""
Created on Fri Nov 25 12:15:44 2022

@author: bking
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Nov 15 15:13:18 2022

@author: bking
"""


import random
from noTraffic_env import VehEnv
import matplotlib.pyplot as plt

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



# random.seed(99)

visualization = False
env = VehEnv(visualization)


# accel = .32
# deccel = -.1

# time_step = .01
# for i in range(1,50):
#     v= env.step(accel*2, time_step)
#     # print(v)
# for i in range(1,50):
#     v= env.step(deccel*2, time_step)
#     # print(v)
# env.close()   
    


# # test action space
# episodes = 1
# for episode in range(1,episodes+1):
#     state = env.reset()
#     done = False
#     score = 0
    
#     for i in range(1,30):
#         action=8
#         n_state, reward, done, info = env.step(action,.01)
#         score+= reward
#         print(n_state)
#     while not done:
# #         action = env.action_space.sample()  
#         action=2
#         n_state, reward, done, info = env.step(action,.01)
#         score+= reward
#         print(n_state)
#     print('Episode:{} Score:{}'.format(episode,score))

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

try:
    del model
except:
    pass

model = build_model(states, actions)

# OPEN PRE-BUILT MODEL
# model = keras.models.load_model('model-acceleration_based')

print(model.summary())


def build_agent(model, actions):
    # policy = BoltzmannQPolicy()
    policy = EpsGreedyQPolicy(eps=0.3)
    # policy = LinearAnnealedPolicy(EpsGreedyQPolicy(), attr='eps', value_max=.9, value_min=.3, value_test=.01,
                                   # nb_steps=200)
    memory = SequentialMemory(limit=100000, window_length =1)
    dqn = DQNAgent(model=model, memory=memory, policy= policy, nb_actions=actions, nb_steps_warmup=100, target_model_update = 1e-1)
    return dqn

dqn = build_agent(model, actions)
dqn.compile(Adam(learning_rate=1e-3), metrics = ['mae'])

history = dqn.fit(env, nb_steps =1000, visualize=False, verbose =1)
# summarize history for accuracy
plt.plot(history.history['episode_reward'])
plt.ylabel('reward')
plt.xlabel('epoch')
plt.show()
# model.get_weights()

# SAVE MODEL
# model.save('model-acceleration_based_5000000steps.h5')

# # simulate trained model
# scores = dqn.test(env, nb_episodes = 3, visualize=False)
# print(np.mean(scores.history['episode_reward']))


env.close()
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

