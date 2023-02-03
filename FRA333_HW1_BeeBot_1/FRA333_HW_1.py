#!/usr/bin/python3
from trackbeebot import BeeBot
import matplotlib.pyplot as plt 
from matplotlib.patches import Polygon
import numpy as np
import math
import json


class MyBeeBot(BeeBot):
    def __init__(self,a_i):      
        super().__init__(a_i) 
        self.a_i = a_i # a_i is initial position in hexagon frame(index)
        self.a_i_global = self.idx2pos(a_i[0], a_i[1]) # find initial position in global frame from index to position function 
                                                       # that need initial position in hexagon frame

        # We define x-axis is a robot head so we must rotate x-axis 90 degrees for facing up robot and know initial position of robot
        # then we can create transform matrix that contains of rotate matrix and translation matrix for setup robot
        self.init_pose_matrix = [[np.cos(np.pi/2), -np.sin(np.pi/2), self.a_i_global[0]] 
                                ,[np.sin(np.pi/2),  np.cos(np.pi/2), self.a_i_global[1]] 
                                ,[0, 0, 1]]
        

    
    def trackBeeBot(self, com, W):
        # com is command {'0'->stop, '1'->forward, '2'->backward, '3'->turn left, '4'->turn right}
        # W is wall

        # the direction is list that contains of [Rotate, Translate] from each command(com)       
        # The robot can rotate only 60 degrees at a time.
        # The distance between center of hexagon is square root of 3
        direction = [[0,0] ,[0,np.math.sqrt(3)] ,[0,-np.math.sqrt(3)] ,[60,0] ,[-60,0]]  
 
        self.old_pose = self.init_pose_matrix.copy() # old_pose is old position of robot 
      
        x_g = [self.a_i_global[0]]                   # x_g is list contains x position of robot relate to global frame 
        y_g = [self.a_i_global[1]]                   # y_g is list contains y position of robot relate to global frame 
        idx_x = [self.a_i[0]]                        # idx_x is list contains x position in hexagon frame that robot be 
        idx_y = [self.a_i[1]]                        # idx_y is list contains y position in hexagon frame that robot be

        # Wall
            # from input W(wall) is an array 2 x n then transpost it to get the order of wall in hexagon frame is a integer value
        wall_arr = np.array(W).T
        wall = [[j,k] for j,k in wall_arr] # for loop to make list of wall position

        # Run command!
        for i in com:        
            self.theta = np.radians(direction[int(i)][0]) # check command if command are '3' or '4' robot will rotate 60 degrees and -60 degrees 
                                                          # others is 0 degree                                                        
            self.c, self.s = np.cos(self.theta), np.sin(self.theta) # c is cos(theta) 
                                                                    # s is sin(theta)
            self.Rotate = np.array([[self.c, -self.s, direction[int(i)][1]], [self.s, self.c, 0], [0, 0, 1]]) # Rotate is transforn matrix of robot at current
            self.new_pose = np.matmul(self.old_pose, self.Rotate) # new_pose is new position of robot relate to global frame 
                                                                  # got from old_pose multiply by Rotate

            # Getting Index
            idx = self.pos2idx(self.new_pose[0][2], self.new_pose[1][2]) 
                # pass robot position that relate to global frame to pos2idx function for get robot position in hexagon frame(index)
                # This function return list of index    

            # Check if robot hit the wall               
            if i == '1' or i == '2':    # The robot will hit the wall when command is '1' or '2' (forward and backward) 
                if [int(round(idx[0])), int(round(idx[1]))] not in wall: # if index of robot not equal wall position that mean the robot can forward or backward
                    x_g.append(self.new_pose[0][2])     # keep x and y position of the robot to x_g list                                          
                    y_g.append(self.new_pose[1][2])     # *this is current position of the robot relate to global frame

                    idx_x.append(int(round(idx[0])))    # keep x and y position in hexagon frame. Sometime the value is decimal so use round() 
                    idx_y.append(int(round(idx[1])))    # to given precision in decimal digits and then casting to int()  

                    self.old_pose = self.new_pose       # update position of the robot relate to global frame 

                # if robot hit the wall so do not do anything
            else:                       # if robot stay still or only rotate when command is '0' or '3' or '4' (not translate)
                self.old_pose = self.new_pose           # update position of the robot relate to global frame 
       
        return np.array([idx_x,idx_y]), np.array([x_g,y_g])
  
    # This function use to convert robot position in global frame to hexagon frame
    # This function return list of index (robot position in hexagon frame)
    def pos2idx(self, x, y):
        i = x/3 + y/math.sqrt(3)
        j = -x/3 + y/math.sqrt(3)
        return [i, j] 

# "a_i": [-4, -1],
# "w": [[-1, 0, -2, 0, -4, 2, 5, -2, 2, 1, 4, 2, -2, -4], 
#       [ 1, 0, -1, 5,  2, 2, 1, -4,-5, 1,-5,-1,  4,  0]],
# "c": "10004031304133412113203401202434122434340313312420340143230302444311410144134242324403414413032441433", 
# "a": [[-4, -3, -2, -1, -1, -1, -1, -1, 0, -1, 0, 1, 1, 1, 1, 1, 0, 1, 2, 1, 2, 2, 1, 1, 1, 2, 1, 1, 0, -1, -1, 0, 0], 
#       [-1,  0,  1,  2,  3,  2,  3,  4, 4,  4, 4, 4, 5, 4, 3, 4, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 7, 8, 8,  7,  8, 9, 10]], 
# "p": [[-4.5, -4.5, -4.5, -4.5, -5.999999999999999, -4.5, -5.999999999999999, -7.499999999999998, -5.999999999999999, -7.499999999999998, -5.999999999999999, -4.5, -5.999999999999999, -4.5, -3.000000000000001, -4.5, -4.5, -4.5, -3.000000000000001, -4.5, -3.000000000000001, -4.5, -5.999999999999999, -7.499999999999998, -8.999999999999998, -7.499999999999999, -8.999999999999998, -10.499999999999998, -11.999999999999998, -11.999999999999998, -13.499999999999998, -13.499999999999998, -14.999999999999998],
#       [-4.330127018922193, -2.5980762113533156, -0.8660254037844386, 0.8660254037844382, 1.7320508075688763, 0.866025403784438, 1.7320508075688763, 2.5980762113533147, 3.464101615137753, 2.5980762113533147, 3.464101615137753, 4.330127018922191, 5.196152422706629, 4.330127018922191, 3.4641016151377526, 4.330127018922191, 2.5980762113533142, 4.330127018922191, 5.196152422706629, 4.330127018922191, 5.196152422706629, 6.062177826491068, 5.196152422706629, 6.062177826491068, 6.928203230275506, 7.7942286340599445, 6.928203230275506, 7.7942286340599445, 6.928203230275506, 5.196152422706629, 6.062177826491068, 7.7942286340599445, 8.660254037844382]], 
# "max": 19

if __name__ == "__main__":
    a = MyBeeBot([-4,-1])
    b = a.trackBeeBot("10004031304133412113203401202434122434340313312420340143230302444311410144134242324403414413032441433",[[-1, 0, -2, 0, -4, 2, 5, -2, 2, 1, 4, 2, -2, -4], [1, 0, -1, 5, 2, 2, 1, -4, -5, 1, -5, -1, 4, 0]])
    # print(b)
    
    


   