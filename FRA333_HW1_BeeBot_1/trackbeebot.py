#!/usr/bin/python3

import matplotlib.pyplot as plt 
from matplotlib.patches import Polygon
import numpy as np
import math
import json

class BeeBot():
    def __init__(self,init_idx):
        self.init_idx = init_idx
        self.d = 1.0
    def idx2pos(self, i,j):
        a = np.array([3/2,np.math.sqrt(3)/2])*self.d*(i-1)
        b = np.array([-3/2,np.math.sqrt(3)/2])*self.d*(j-1)
        return a+b+np.array([0,np.math.sqrt(3)])
    def Hexagon(self,center,theta=0,faceColor='w'):
        x = [np.cos(math.pi/3*np.array(range(6)))+center[0]]
        y = [np.sin(math.pi/3*np.array(range(6)))+center[1]]
        xy = np.concatenate((x,y),0)
        return Polygon(xy.T,closed=True,edgecolor='k',facecolor=faceColor)
    # check if obstacle is given
    def plot_trackBeeBot(self,A,max,plot,W=None):
        if W is not None:
            C = np.concatenate((A,W),1)
        else:
            C = A

        fig, ax = plt.subplots(1)
        ax.set_aspect('equal')

        # draw all cells
        for i in range(np.amin(C[0]),np.amax(C[0])+1):
            for j in range(np.amin(C[1]),np.amax(C[1])+1):
                center = self.idx2pos(i,j)
                hex = self.Hexagon(center,0)
                ax.add_patch(hex)
                ax.text(center[0],center[1], str(i)+","+str(j), ha="center", va="center", size=10)
                
        # draw walls
        if W is not None:
            for (i,j) in W.T:
                center = self.idx2pos(i,j)
                hex = self.Hexagon(center,0,'k')
                # hex = self.Hexagon(center,0,'b')
                ax.add_patch(hex)
        
        ax.set_xbound((-max,max))
        ax.set_ybound((-max,max))

        # draw path
        color_dict = {}
        idx = 0
        for (i,j) in A.T:
            center = self.idx2pos(i,j)
            if idx == 0:
                color = 'g'
            elif idx == np.shape(A)[1]-1:
                color = 'r'
            else:
                color = 'b'
            color_dict[(i,j)] = color
            hex = self.Hexagon(center,0,color)
            ax.add_patch(hex)
            if plot:
                plt.pause(0.01)
            if (idx != 0 and idx !=np.shape(A)[1]-1):
                hex = self.Hexagon(center,0,'y')
                ax.add_patch(hex)
                if plot:
                    plt.pause(0.01)
            idx = idx + 1
        if plot:
            plt.show()

    def info2JSON(self,A,W):
        with open('result.json','w') as f:
            json.dump({'A':A.tolist(),'W':W.tolist()},f)

    # a_i is initial position
    # c is command {'0'->stop, '1'->forward, '2'->backward, '3'->turn right, '4'->turn left}
    # o is obstacle
    def trackBeeBot(self, com, W):
        pass

