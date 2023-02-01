# -*- coding: utf-8 -*-
"""
Created on Mon Jan 30 17:31:46 2023

@author: bking
"""
import numpy as np
data= [i for i in history.history['episode_reward']if i>0]
window = 20
average_data=[]
for ind in range(len(data) - window + 1):
    average_data.append(np.mean(data[ind:ind+window]))
plt.plot(average_data)


plt.plot(env.get_speeds())