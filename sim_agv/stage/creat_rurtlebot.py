#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np

path =  os.path.dirname(os.path.realpath(__file__))
class TurtlebotWorld():
    """
    创建turtlebots.txt文本,将文本里面的内容替换至.world文件中批量创建小车
    """
    def __init__(self, total_num=16):
        self.total_num = total_num
        self.interval = 1.8
        
    def create_turtlebot(self, x, y, name):
        with open(path+'/turtlebots.txt', 'a+') as file:
            file.write("turtlebot \n")
            file.write("( \n")
            file.write("  pose [ " + str(x) + " " + str(y) + " 0.0 180.0 ]\n")# 角度
            file.write("  name \" turtlebot" + name + "\"\n")
            file.write("  color \"green\"\n")
            file.write(")")
            file.write("\n")
    def creat_txt(self,start_x=1,x=12,start_y=12, y=6):
        x_coords = np.arange(start_x*self.interval, (start_x+x)*self.interval, self.interval)
        y_coords = np.arange(start_y*self.interval, (start_y+y)*self.interval, self.interval)
        print(x_coords)
        print(y_coords)
        agv_n = x*y
        # import pdb;pdb.set_trace()
        for i in range(x):
            for j in range(y-1,-1,-1):
                # matrix[i][j] = [x_coords[i], y_coords[j]]
                agv_n -= 1
                self.create_turtlebot(x_coords[i],y_coords[j],str(agv_n))
                print("creat: {}_{}: [{},{}]".format(i,j,x_coords[i],y_coords[j]))

    
t = TurtlebotWorld()
t.creat_txt()