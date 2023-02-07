This python program is used to train an RL agent to drive through a set of connected signalized corridors using Eclipse SUMO framework.

Descriptions:
Variable Light time- lights not only change position but also phase times. RL agent commands a velocity at every intersection. No other vehicles in network. 
No Traffic Network- agent makes an acceleration command every second. No other vehicles in network. {issues/deprecated}
Intersections as Time Steps- at each intersection, RL agent commands a velocity. No other vehicles in network.
Network with traffic-  at each intersection, RL agent commands a velocity. Other vehicles included in network {issues/deprecated}
Test Networks- used for testing basic functionality of SUMO

Dependencies:
numpy==1.19.5
keras==2.10.0
keras_rl2==1.0.5
tensorflow==2.5.0
gym==0.17.3