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
        '''string message received on /radio_traffic needs to be published to /recieved_rs'''
        #parse the msg.data from a to :
        idx = 1
        tempstr = ""
        while msg.data[idx] != ":":
            tempstr += msg.data[idx]
            idx += 1
        
        if int(tempstr) != self.SYSID_THISMAV:
            #publish the message to /recieved_rs
            #print("Publishing to /recieved_rs: ", msg.data)
            tempHyperRect = hyperRectangle()

            idx += 2
            
            tempHyperRect.id = int(tempstr)

            tempstr = ""


            while msg.data[idx] != "y":
                #idx += 1
                tempstr += msg.data[idx]
                idx += 1
            tempHyperRect.x.min = float(tempstr)
            tempstr = ""
            idx += 1
            while msg.data[idx] != "z":
                #idx += 1
                tempstr += msg.data[idx]
                idx += 1
            tempHyperRect.x.max = float(tempstr)
            tempstr = ""
            idx += 1
            while msg.data[idx] != "v":
                #idx += 1
                tempstr += msg.data[idx]
                idx += 1
            tempHyperRect.y.min = float(tempstr)
            tempstr = ""
            idx += 2
            while msg.data[idx] != "v":
                #idx += 1
                tempstr += msg.data[idx]
                idx += 1
            tempHyperRect.y.max = float(tempstr)
            tempstr = ""
            idx += 2
            while msg.data[idx] != "v":
                #idx += 1
                tempstr += msg.data[idx]
                idx += 1
            tempHyperRect.z.min = float(tempstr)
            tempstr = ""
            idx += 2
            while idx < len(msg.data):
                tempstr += msg.data[idx]
                idx += 1
            tempHyperRect.z.max = float(tempstr)
            self.recievedRsPub.publish(tempHyperRect)


    def my_rs_callback(self, msg):
        '''string message received on /my_rs'''
        #print("Received from /my_rs: ", msg.id)
        myRsString = "a" + str(self.SYSID_THISMAV) + ":x" + format(msg.x.min, '.2f') + "y" + format(msg.x.max, '.2f') + "z" + format(msg.y.min, '.2f') + "vx" + format(msg.y.max, '.2f') + "vy" + format(msg.z.min, '.2f') + "vz" + format(msg.z.max, '.2f')
        #print("Publishing to /radio_traffic: ", myRsString)
        self.radioPub.publish(myRsString)


    def __init__(self):
        '''Initializes the class'''
        rospy.init_node('radio_sim')

        self.SYSID_THISMAV = rospy.get_param('SystemID', 1)

        rospy.Subscriber("/radio_traffic", String, self.radio_callback)
        rospy.Subscriber("/my_reach_state", hyperRectangle, self.my_rs_callback)

        self.radioPub = rospy.Publisher("/radio_traffic", String, queue_size=30)
        self.recievedRsPub = rospy.Publisher("/recieved_rs", hyperRectangle, queue_size=30)


    def main_loop(self):
        print("Verifier Comms node is running")


        rospy.spin()

if __name__ == '__main__':
    radioSim = radioSim()
    radioSim.main_loop()
