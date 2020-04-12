#! /usr/bin/env python

import rospy
from std_msgs.msg import Int8, String
import sys 
import vrep
import time

class Manager():
    def __init__(self):
        rospy.init_node('Dummy', anonymous = True)
        rospy.Subscriber("/failure", Int8, self.receiveStatus, queue_size = 1)
        fin = rospy.Publisher('/finished', Int8, queue_size = 1)
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
            if not first and r != 0:
                r = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
            start = time.time()
            self.failure = False
            elapsed = 0
            while(not self.failure and elapsed < maxTime):
                curr = time.time()
                elapsed = curr - start
            if not self.failure: #timed out!
                msg = Int8()
                msg.data = 2
                fin.publish(msg)
            vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
            is_running = True
            while is_running:
                error_code, ping_time = vrep.simxGetPingTime(clientID)
                error_code, server_state = vrep.simxGetInMessageInfo(clientID, vrep.simx_headeroffset_server_state);
                is_running = server_state & 1
            counter += 1
            first = False
        msg = Int8()
        msg.data = 1
        finish_pub.publish(msg)

    def receiveStatus(self, message):
        if message.data == 1: #failure 
            self.failure = True 
        return

episodes = 5
maxTime =  100


if __name__ == "__main__":
    manager = Manager()