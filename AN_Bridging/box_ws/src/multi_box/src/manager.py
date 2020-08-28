#! /usr/bin/env python

import rospy
from std_msgs.msg import Int8, String, Int16
import numpy as np
import os
import sys 
import vrep
import time

VREP_SCENES = [('elevated_scene', '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/Sims/box_simulation.ttt'),
               ('flat_scene', '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/Sims/box_flat_simulation.ttt'),
               ('slope_scene', '/home/austinnguyen517/Documents/Research/BML/MultiRobot/AN_Bridging/Sims/box_slope_single_simulation.ttt')]

class Manager():
    def __init__(self):
        rospy.init_node('Dummy', anonymous = True)
        rospy.Subscriber("/restart", Int8, self.receiveStatus, queue_size = 1)
        fin = rospy.Publisher('/finished', Int8, queue_size = 1)
        report_sim = rospy.Publisher('/simulation', String, queue_size=1)
        starting = rospy.Publisher('/starting', Int16, queue_size = 1)
        vrep.simxFinish(-1) #clean up the previous stuff
        clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if clientID == -1:
            print("Could not connect to server")
            sys.exit()

        first = True
        counter = 0
        while (counter < episodes):
            print("Episode Number ", counter + 1)
            r = 1 
            if True: #not first:
                if (counter) % 50 == 0 and counter != 0:
                    print('Sleep for 60')
                    #time.sleep(60)
                else:
                    time.sleep(3)
                simulation_index = np.random.randint(len(VREP_SCENES))
                sim_name, sim_path = VREP_SCENES[simulation_index]
                msg = String()
                msg.data = sim_name
                report_sim.publish(msg)
                vrep.simxLoadScene(clientID, sim_path, 0, vrep.simx_opmode_blocking)
                time.sleep(2)
                while r != 0:
                    r = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
                msg = Int16()
                msg.data = counter + 1
                starting.publish(msg)
            start = time.time()
            self.restart = False
            elapsed = 0
            while(not self.restart and elapsed < maxTime):
                curr = time.time()
                elapsed = curr - start
            vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
            if not self.restart: #timed out!
                msg = Int8()
                msg.data = 2
                fin.publish(msg)
                print(' #### Timed out!')
            is_running = True
            while is_running:
                error_code, ping_time = vrep.simxGetPingTime(clientID)
                error_code, server_state = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state)
                is_running = server_state & 1
            counter += 1
            first = False
        time.sleep(2)
        msg = Int8()
        msg.data = 1
        fin.publish(msg)

    def receiveStatus(self, message):
        if message.data == 1: #restart
            self.restart = True 
        return

episodes = 1200 # 1600 
maxTime = 21 # 40 with visual. 21 for all...MBRL has 40 steps before timing out on first episode


if __name__ == "__main__":
    manager = Manager()