#!/usr/bin/python3
from trackbeebot import BeeBot
import matplotlib.pyplot as plt 
from matplotlib.patches import Polygon
import numpy as np
import math
import json
from FRA333_HW_1 import MyBeeBot

with open('testcase.json') as f:
    lines = f.readlines()
testcases = json.loads(lines[0])
i = 1
for testcase in testcases["testcase"]:
    mytest = MyBeeBot(np.array(testcase["a_i"]))
    W = np.array(testcase["w"])
    c = testcase["c"]
    A, P = mytest.trackBeeBot(c,W)
    ########## Check Solution ###########
    check = True
    try:
        for a, _a in zip(A.T, np.array(testcase["a"]).T):
            if (abs(a[0] - _a[0] <= 0.05) and abs(a[1] - _a[1] <= 0.05)):
                pass
            else:
                check = False
    except:
        check = False
    if check:
        print("Testcase no. " + str(i) + " is TRUE")
    else:
        print("Testcase no. " + str(i) + " is FALSE")
    i = i + 1
    mytest.plot_trackBeeBot(A,testcase["max"],True,W)


