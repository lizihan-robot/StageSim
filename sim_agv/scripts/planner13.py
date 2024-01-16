#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import threading
# import signal
# signal.signal(signal.SIGINT, signal.SIG_DFL)
from matrix_map import Map
from agv import AGV
import rospy


class Planner():
    def __init__(self, map=Map, agv=AGV):    
        self.agv = agv
        self.map = map
        self.lock = threading.Lock()  # 添加锁

    def run_agv(self): 
        pose = self.map.get_plane_pose("Plane0_16")
        t = self.agv.move_l(pose)
        t = self.agv.move_ofsc(90)
        
        pose = self.map.get_plane_pose("Plane0_27")
        t = self.agv.move_l(pose)
        t = self.agv.move_ofsc(0)

        pose = self.map.get_plane_pose("Plane16_27")
        t = self.agv.move_l(pose)
        t = self.agv.move_ofsc(90)
        
        pose = self.map.get_plane_pose("Plane16_30")
        t = self.agv.move_l(pose)
        t = self.agv.move_ofsc(0)
        
        pose = self.map.get_plane_pose("Plane17_30")
        t = self.agv.move_l(pose)




if __name__ == "__main__":
    rospy.init_node('agv_dispath')
    m = Map()
    n = 13
    st = 26+n # 启动延时
    col = 2
    wt = 10*col  # 等待延时
    time.sleep(st)
    time.sleep(wt)
    agv= AGV(n)    
    planner = Planner(m, agv)
    print("[Plan]: agv_{} work start".format(n))
    planner.run_agv()
    print("[Plan]: agv_{} work completed".format(n))
