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
    # 2 - enable/disable collision avoidance
    # 3 - enable/diable re-broadcasting

class commsHandler:
    '''Class to handle radio strings coming from /radio_traffic and reachable sets'''
    RADIO_HOST = -1

    def pack_payload(self, payload):
        '''
        Packs a payload into a string
        '''
        tempStr = ""
        for i in range(len(payload)):
            tempStr += str(payload[i]) + ":"
        return tempStr


    def pack_message(self, target, sendingAgent, messageType, rebroadcast, payload):
        '''
        Packs a message into the radio traffic message protocol
        '''

        sting_payload = self.pack_payload(payload)

        tempStr = str(target) + ":" + str(sendingAgent) + ":" + str(messageType) + ":" + str(rebroadcast) + ":" + sting_payload
        return tempStr
    
    def unpack_message(self, message):
        '''
        Unpacks a message from the radio traffic message protocol into a dict
        '''
        try:
            messageList = message.split(":")
            messageDict = {}
            messageDict['target'] = int(messageList[0])
            messageDict['sendingAgent'] = int(messageList[1])
            messageDict['messageType'] = int(messageList[2])
            messageDict['rebroadcast'] = bool(messageList[3])
            messageDict['payload'] = float(messageList[4:]) #payload should have 8 elements
        except:
            print("Error unpacking message")
            return None

        return messageDict
    
    def unpack_payload_to_hyperrect(self, id, payload):
        '''
        Unpacks a payload from a hyperrectangle message
        '''
        try:
            tempHyperRect = hyperRectangle()
            tempHyperRect.id = id
            tempHyperRect.x.min = float(payload[0])
            tempHyperRect.x.max = float(payload[1])
            tempHyperRect.y.min = float(payload[2])
            tempHyperRect.y.max = float(payload[3])
            tempHyperRect.z.min = float(payload[4])
            tempHyperRect.z.max = float(payload[5])
  
        except:
            print("Error unpacking hyperrectangle payload")
            return None

        return tempHyperRect
    

    def radio_callback(self, msg):
        '''
        When a message is received on /radio_traffic it needs to be parsed and handled as a
        1) command
        2) reachable set
        '''

        #Store the message in a dictionary
        message = self.unpack_message(msg.data)
        
        #Check if the message is for this agent
        if message['target'] != 0 and message['target'] != self.RADIO_HOST:
            return

        #Check if the message is from this agent
        if message['sendingAgent'] == self.RADIO_HOST:
            return
        
        #Handle the message as a reachable set or a command
        if message['messageType'] == 0:
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

            print("Command received of type" + commandType)

            if commandType == 0:
                if messageOptions[0] == 0:
                    print("Ping received from " + str(sendingAgent))
                    #send the ping back to the origin
                    tempStr = str(sendingAgent) + ":" + str(self.RADIO_HOST) + ":1:0:0:1:0:0:0:0:0:0"
                    self.radioPub.publish(tempStr)
                elif messageOptions[0] == 1:
                    print("Ping received from " + str(sendingAgent))

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

        tempStr =  "0:" + str(self.RADIO_HOST) + ":0:0:" + str(msg.x.min) + ":" + str(msg.x.max) + ":" + str(msg.y.min) + ":" + str(msg.y.max) + ":" + str(msg.z.min) + ":" + str(msg.z.max)
        self.radioPub.publish(tempStr)


    def __init__(self):
        rospy.init_node('verifier_comms')

        self.RADIO_HOST = rospy.get_param('SystemID', 1)

        rospy.Subscriber('/radio_traffic', String, self.radio_callback) #Done
        rospy.Subscriber('/my_reach_state', hyperRectangle, self.my_rs_callback) 

        self.radioPub = rospy.Publisher('/radio_traffic', String, queue_size=10)
        self.recievedRsPub = rospy.Publisher('/recieved_rs', hyperRectangle, queue_size=10)

    def main_loop(self):
        print("Verifier Comms node is running")
        rospy.spin()
