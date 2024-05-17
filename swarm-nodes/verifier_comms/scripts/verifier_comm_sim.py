#!/usr/bin/env python3
import sys
import serial
from digi.xbee.devices import *
from digi.xbee.util import utils
from digi.xbee.exception import *
import time
import rospy
from std_msgs.msg import String
from verifier_comms.msg import hyperRectangle
class radioSim(object):
    '''
    Class to simulate Xbee in ROS environment
    
    This node is to do two things:
    1. when a message is received on /my_rs it needs to be publised to /radio_traffic formatted as:
    
    a{SYSID_THISMAV}:x{lower x bound}y{upper x bound}z{lower y bound}vx{upper y bound}vy{lower z bound}vz{upper z bound}
    

    2. When a message is received on /radio_traffic it needs to be publised to /recieved_rs (filtering out this agent's messages): 

    '''

    def radio_callback(self, msg):
        '''string message received on /radio_traffic'''
        print("Received from /radio_traffic: ", msg.data)

    def my_rs_callback(self, msg):
        '''string message received on /my_rs'''
        print("Received from /my_rs: ", msg.id)

    def __init__(self):
        '''Initializes the class'''
        rospy.init_node('radio_sim')

        self.SYSID_THISMAV = rospy.get_param('SYSID_THISMAV', 1)

        rospy.Subscriber("/radio_traffic", String, self.radio_callback)
        print("Subscribed to /radio_traffic")

        rospy.Subscriber("/agent_1/my_reach_state", hyperRectangle, self.my_rs_callback)
        print("Subscried to /my_rs")

        self.radioPub = rospy.Publisher("/radio_traffic", String, queue_size=30)
        print("Publishing to /radio_traffic")


    def main_loop(self):
        print("Verifier Comms node is running")


        rospy.spin()

if __name__ == '__main__':
    radioSim = radioSim()
    radioSim.main_loop()
