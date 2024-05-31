#!/usr/bin/env python3
import sys
import serial

import rospy
from std_msgs.msg import String
from verifier_comms.msg import hyperRectangle

#Radio traffic message protocol:
# {target}:{sending agent}:{message type}:{rebroadcast flag}:{payload}
#First integer will be the target system (0 for all)

#Second integer will be the sending agent sysid

#Third integer will be the message type
# 0 - Hyperrectangle
    # {payload} = {x.min}:{x.max}:{y.min}:{y.max}:{z.min}:{z.max}

# 1 - Command
    # {payload} = {command}:{param1}:{param2}:{param3}:{param4}:{param5}:{param6}:{param7}

    #Commands:
    # 0 - Ping
    # 1 - set flight mode
    # 2 - enable/disable collision avoidance
    # 3 - enable/diable re-broadcasting


class commsHandler(object):
    '''Class to handle radio strings coming from /radio_traffic and reachable sets'''

    def radio_callback(self, msg):
        '''
        When a message is received on /radio_traffic it needs to be parsed and handled as a
        1) command
        2) reachable set
        '''
        messageList = msg.data.split(":")
        #Error checking?
        
        #Check if the message is for this agent
        target = int(messageList[0])
        if target != 0 and target != self.SYSID_THISMAV:
            return

        #Check if the message is from this agent
        sendingAgent = int(messageList[1])
        if sendingAgent == self.SYSID_THISMAV:
            return
        
        #If made it this far, check the message type
        messageType = int(messageList[2])

        #Check if the message needs to be re-broadcast
        rebroadcast = False
        if messageList[3] == '1':
            rebroadcast = True
        

        #Handle the message as a reachable set or a command
        if messageType == 0:
        #reachable set received
            tempHyperRect = hyperRectangle()
            tempHyperRect.id = sendingAgent
            tempHyperRect.x.min = float(messageList[4])
            tempHyperRect.x.max = float(messageList[5])
            tempHyperRect.y.min = float(messageList[6])
            tempHyperRect.y.max = float(messageList[7])
            tempHyperRect.z.min = float(messageList[8])
            tempHyperRect.z.max = float(messageList[9])

            self.recievedRsPub.publish(tempHyperRect)

        #doing a command
        elif messageType == 1:

            commandType = int(messageList[4])
            print("Command received of type" + commandType)

            if commandType == 0:
                print("Ping received from " + sendingAgent)
                #do nothing

            else:
                print("Unknown command received")
                return

        else:
            print("Invalid message type received")
            return
        
    def my_rs_callback(self, msg):
        '''
        When a message is received on /my_reach_state it needs to be publised to /radio_traffic
        '''
        #Format the message and publish it to /radio_traffic

        tempStr =  "0:" + str(self.SYSID_THISMAV) + ":0:0:" + str(msg.x.min) + ":" + str(msg.x.max) + ":" + str(msg.y.min) + ":" + str(msg.y.max) + ":" + str(msg.z.min) + ":" + str(msg.z.max)
        self.radioPub.publish(tempStr)


    def __init__(self):
        rospy.init_node('verifier_comms')

        self.SYSID_THISMAV = rospy.get_param('SystemID', 1)

        rospy.Subscriber('/radio_traffic', String, self.radio_callback) #Done
        rospy.Subscriber('/my_reach_state', hyperRectangle, self.my_rs_callback) 

        self.radioPub = rospy.Publisher('/radio_traffic', String, queue_size=10)
        self.recievedRsPub = rospy.Publisher('/recieved_rs', hyperRectangle, queue_size=10)

    def main_loop(self):
        print("Verifier Comms node is running")
        rospy.spin()

if __name__ == '__main__':
    comms = commsHandler()
    comms.main_loop()
