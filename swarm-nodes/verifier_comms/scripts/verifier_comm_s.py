#!/usr/bin/env python3
from radio_object import unpack_message, unpack_hyperrect_payload, unpack_command_payload, MODE_MAP, COMMAND_MAP, REBROADCAST, NO_REBROADCAST
import rospy
from verifier_comms.msg import hyperRectangle
from std_msgs.msg import String
from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse, CommandBool, CommandBoolRequest


#set guided 0:0:1:0:1:1:0:0:0:0:0:0
#set arm    0:0:1:0:2:1:0:0:0:0:0:0
class agent_comms:
    ID = -1

    def handle_ping(self, message, payload):
        '''
        Handle a ping command
        '''
        if payload['param1'] == 0: #Ping requested
            print(f"Ping received from {message['sendingAgent']}")
            #send the ping back to the origin
            tempStr = f"{message['sendingAgent']}:{self.ID}:1:0:0:1:0:0:0:0:0:0"
            self.radio_pub.publish(tempStr)
        elif payload['param1'] == 1: #heard a ping response
            print(f"Ping received from {message['sendingAgent']}")

    def handle_set_mode(self, message, payload):
        '''
        Handle a set mode command
        '''
        print(f"Setting mode to {MODE_MAP[payload['param1']]}")
        try:
            self.set_mode_request.custom_mode = MODE_MAP[payload['param1']]
            response = self.set_mode_service.call(self.set_mode_request)
            print(response)
        except:
            print("Invalid mode requested")
            return

    def handle_arm(self, message, payload):
        '''
        Handle an arm/disarm command
        '''
        #try:
        if payload['param1'] == 0:
            self.arming_client_request.value = False
        elif payload['param1'] == 1:
            self.arming_client_request.value = True
        print(f"Swithing arm status to: {self.arming_client_request.value}")
        response = self.arming_client.call(self.arming_client_request)
        print(response)
        #except:
        #    print("Invalid arm/disarm command")


    def radio_traffic_callback(self, msg):
        '''
        When a message is received on /radio_traffic it needs to be unpacked and interpreted
        '''

        #Store the message in a dictionary
        message = unpack_message(msg.data)

        #Check if the message is for this agent
        if message['target'] != 0 and message['target'] != self.ID:
            return
        
        #Check if the message is from this agent
        if message['sendingAgent'] == self.ID:
            return
        
        
        if message["messageType"] == 0:
                
            #reachable set received
            rcvd_rs = unpack_hyperrect_payload(message['sendingAgent'], message['payload'])
            self.rcvd_rs_pub.publish(rcvd_rs)

        elif message["messageType"] == 1:

            #Execute a command
            command = unpack_command_payload(message['payload'])
            print(f"Command received of type {command['command']}")

            
            if command["command"] == "PING":
                self.handle_ping(message, command)
            
            elif command["command"] == "SET_MODE":
                self.handle_set_mode(message, command)
            elif command["command"] == "ARM":
                self.handle_arm(message, command)
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

        tempStr =  f"0:{self.ID}:0:0:{msg.x.min}:{msg.x.max}:{msg.y.min}:{msg.y.max}:{msg.z.min}:{msg.z.max}"
        self.radio_pub.publish(tempStr)

    def __init__(self):
            
        rospy.init_node('agent_comms')
        self.ID = rospy.get_param('SystemID', 1)

        rospy.Subscriber('/radio_traffic', String, self.radio_traffic_callback) #subscribe to incoming radio traffic for commands
        self.radio_pub = rospy.Publisher('/radio_traffic', String, queue_size=10) #publish messages to radio traffic

        rospy.Subscriber('/my_reach_state', hyperRectangle, self.my_rs_callback) #subscribed to this agent's reachable set
        self.rcvd_rs_pub = rospy.Publisher('/recieved_rs', hyperRectangle, queue_size=10) #publish incoming reachable sets to the rest of the nodes

        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_mode_request = SetModeRequest()

        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arming_client_request = CommandBoolRequest()
        
    def main_loop(self):
        print("Agent comms node running")
        rospy.spin()

if __name__ == "__main__":
    agent = agent_comms()
    agent.main_loop()