# -*- coding: utf-8 -*-
"""
Created on Fri Jan 27 18:02:10 2023

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


# edit network each run
tree = ET.parse('networkCustom.tll.xml')
root = tree.getroot()
for nodes in root.iter('tlLogics'):
    for logic in root.iter('tlLogic'):
        print('')
        print(logic.attrib['id'])
        rand_time =random.randint(10,50)
        green = rand_time
        print('Green',green)
        red = 60 - rand_time
        print('red:',red)
        for phase in logic.iter('phase'):   
            if phase.attrib['state'] == "GG":
                phase.attrib['duration'] = str(green)
            elif phase.attrib['state'] == "rr":
                phase.attrib['duration'] = str(red)
tree.write('networkCustom.tll.xml')   
