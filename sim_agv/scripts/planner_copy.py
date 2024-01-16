#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import threading
# import signal
# signal.signal(signal.SIGINT, signal.SIG_DFL)
from matrix_map import Map
from agv import AGVDispath

class Planner():
    def __init__(self, map, agvs):    
        self.agvs = agvs
        self.map = map
        self.lock = threading.Lock()  # 添加锁

    def multithreaded(func):
        def wrapper(*args, **kwargs):
            print("[Dispath]: self.agvs excute routine task")
            # return func(*args, **kwargs)
            t = threading.Thread(target=func, args=args, kwargs=kwargs)
            t.daemon = True
            t.start()
            return t
        return wrapper
        
    @multithreaded
    def run_agv1(self): 
        n = 1
        pose = self.map.get_plane_pose("Plane0_8")
        with self.lock:  # 加锁
            t = self.agvs.move_l(n,pose)
            t = self.agvs.move_c(n,-90)

        pose = self.map.get_plane_pose("Plane3_8")

        with self.lock:  # 加锁
            t = self.agvs.move_l(n,pose)
            t = self.agvs.move_c(n,-90)

        pose = self.map.get_plane_pose("Plane3_7")

        with self.lock:  # 加锁
            t = self.agvs.move_l(n,pose)
            t = self.agvs.move_c(n,-90)

        pose = self.map.get_plane_pose("Plane2_7")

        with self.lock:  # 加锁
            t = self.agvs.move_l(n,pose)

        time.sleep(1)
        print("[Plan]: agv_{} work completed".format(n))

    @multithreaded
    def run_agv3(self): 
        n = 3
        pose = self.map.get_plane_pose("Plane1_8")

        with self.lock:  # 加锁
            t = self.agvs.move_l(n,pose)
            t = self.agvs.move_c(n,-90)

        pose = self.map.get_plane_pose("Plane3_8")

        with self.lock:  # 加锁
            t = self.agvs.move_l(n,pose)
            t = self.agvs.move_c(n,-90)

        pose = self.map.get_plane_pose("Plane3_6")

        with self.lock:  # 加锁
            t = self.agvs.move_l(n,pose)
            t = self.agvs.move_c(n,-90)

        pose = self.map.get_plane_pose("Plane2_6")

        with self.lock:  # 加锁
            t = self.agvs.move_l(n,pose)



if __name__ == "__main__":
    # m = Map().save_plane_matrix()
    m = Map()
    t = 2
    agvs= AGVDispath(12)    
    import pdb;
    # pdb.set_trace()
    planner = Planner(m, agvs)
    # planner.test()
    agvs.reset_positions(0)
    planner.run_agv1()
    time.sleep(t)
    planner.run_agv3()
    time.sleep(t)
    # planner.run_agv2()
    # time.sleep(t)
    # planner.run_agv3()
    # time.sleep(t)
    # planner.run_agv4()
    # time.sleep(t)
    # planner.run_agv5()
    # time.sleep(t)
    # planner.run_agv6()
    # time.sleep(t)
    # planner.run_agv7()
    # time.sleep(t)
    # planner.run_agv8()
    # time.sleep(t)
    # planner.run_agv9()
    # time.sleep(t)
    # planner.run_agv10()
    # time.sleep(t)
    # planner.run_agv11()

    # input("<<<")
