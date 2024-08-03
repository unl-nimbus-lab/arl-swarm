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

    #commands:
    # 0 - Ping
        # {param1} = 0 (from the origin of the ping), 1 (from the target of the ping) 
    # 1 - set flight mode
        # {param1} = flight mode
    # 2 - arm/disarm
        # {param1} = 0 (disarm), 1 (arm)
    # 3 - enable/disable collision avoidance
    # 4 - enable/diable re-broadcasting

#Incase you have been away for a while, the plan is to have this serve as a base class that will be used by the agents as well as the ground
#station to handle radio traffic

REBROADCAST = 1
NO_REBROADCAST = 0

MODE_MAP = {
    0: "STABILIZE",
    1: "GUIDED",
    2: "AUTO",
    3: "RTL",
    4: "LOITER",
    5: "LAND"
}

COMMAND_MAP = {
    0: "PING",
    1: "SET_MODE",
    2: "ARM",
    3: "COLLISION_AVOIDANCE",
    4: "REBROADCAST",
}

#This stays
def pack_payload(payload):
    '''
    Packs a payload into a string
    '''
    tempStr = ""
    for i in range(len(payload)):
        tempStr += str(payload[i]) + ":"
    return tempStr

#This stays
def pack_message(target, sendingAgent, messageType, rebroadcast, payload):
    '''
    Packs a message into the radio traffic message protocol
    '''

    sting_payload = pack_payload(payload)

    tempStr = str(target) + ":" + str(sendingAgent) + ":" + str(messageType) + ":" + str(rebroadcast) + ":" + sting_payload
    return tempStr

#This stays
def unpack_message(message):
    '''
    Unpacks a message from the radio traffic message protocol into a dict
    '''
    #try:
    messageList = message.split(":")
    messageDict = {}

    messageDict['target'] = int(messageList[0])
    messageDict['sendingAgent'] = int(messageList[1])
    messageDict['messageType'] = int(messageList[2])
    messageDict['rebroadcast'] = bool(messageList[3])
    messageDict['payload'] = messageList[4:] #This gets parsed in a different function depending on the message type, leave as a string for now

    return messageDict

#This stays
def unpack_hyperrect_payload(id, payload_str):
    '''
    Unpacks a payload from a hyperrectangle message
    '''
    try:
        tempHyperRect = hyperRectangle()
        tempHyperRect.id = id
        tempHyperRect.x.min = float(payload_str[0])
        tempHyperRect.x.max = float(payload_str[1])
        tempHyperRect.y.min = float(payload_str[2])
        tempHyperRect.y.max = float(payload_str[3])
        tempHyperRect.z.min = float(payload_str[4])
        tempHyperRect.z.max = float(payload_str[5])

    except:
        print("Error unpacking hyperrectangle payload")
        return None

    return tempHyperRect

def unpack_command_payload(payload_str):
    payload = {
        "command": COMMAND_MAP[int(payload_str[0])],
        "param1": float(payload_str[1]),
        "param2": float(payload_str[2]),
        "param3": float(payload_str[3]),
        "param4": float(payload_str[4]),
        "param5": float(payload_str[5]),
        "param6": float(payload_str[6]),
        "param7": float(payload_str[7])
    }

    return payload

class commsHandler:
    '''Class to handle radio strings coming from /radio_traffic and reachable sets'''
    RADIO_HOST = -1

    def radio_callback(self, msg):

        raise NotImplementedError("This method must be implemented in the child class")

    def __init__(self):

        #rospy.init_node('verifier_comms')
        #self.RADIO_HOST = rospy.get_param('SystemID', 1)

        rospy.Subscriber('/radio_traffic', String, self.radio_callback) #subscribe to incoming radio traffic
        self.radio_pub = rospy.Publisher('/radio_traffic', String, queue_size=10) #publish messages to radio traffic

        #rospy.Subscriber('/radio_traffic', String, self.radio_callback) #Done
        
        #rospy.Subscriber('/my_reach_state', hyperRectangle, self.my_rs_callback) 

        #self.radioPub = rospy.Publisher('/radio_traffic', String, queue_size=10)
        #self.recievedRsPub = rospy.Publisher('/recieved_rs', hyperRectangle, queue_size=10)

    def main_loop(self):
        print("Verifier Comms node is running")
        rospy.spin()
