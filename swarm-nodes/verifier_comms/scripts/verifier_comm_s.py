#!/usr/bin/env python3
from radio_object import unpack_message, unpack_hyperrect_payload, unpack_command_payload, MODE_MAP, COMMAND_MAP, REBROADCAST, NO_REBROADCAST, HYPERRECTANGLE, COMMAND, DISARM, ARM, PING_REQUESTED, PING_RESPONSE, REQUEST_DISARM, REQUEST_ARM, ENABLE_AVOIDANCE, DISABLE_AVOIDANCE
import rospy
from verifier_comms.msg import hyperRectangle
from std_msgs.msg import String, Bool, Float32
from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse, CommandBool, CommandBoolRequest, CommandTOL, CommandTOLRequest, CommandTOLResponse
import math
"""
rostopic pub /radio_traffic std_msgs/String "data: '0:0:1:0:1:1:0:0:0:0:0:0'"
rostopic pub /radio_traffic std_msgs/String "data: '0:0:1:0:2:1:0:0:0:0:0:0'"
rostopic pub /radio_traffic std_msgs/String "data: '0:0:1:0:3:10:0:0:0:0:0:0'"

"""

           #0:1:2:3:4:5:6:7:8:9 
#set guided 0:0:1:0:1:1:0:0:0:0:0:0
#set arm    0:0:1:0:2:1:0:0:0:0:0:0
#takeoff    0:0:1:0:3:10:0:0:0:0:0:0
class agent_comms:
    ID = -1

    def handle_ping(self, message, payload):
        '''
        Handle a ping command
        param1 (int) indicates if this is a request (0) and this agent should respond or a response (1)
        '''
        if payload['param1'] == PING_REQUESTED: #Ping requested
            print(f"Ping received from {message['sendingAgent']}")
            #send the ping back to the origin
            tempStr = f"{message['sendingAgent']}:{self.ID}:1:0:0:1:0:0:0:0:0:0"
            self.radio_pub.publish(tempStr)
        elif payload['param1'] == PING_RESPONSE: #heard a ping response
            print(f"Ping received from {message['sendingAgent']}")

    def handle_set_mode(self, message, payload):
        '''
        Handle a set mode command
        param1: mode (see MODE_MAP)
        '''
        requested_mode = MODE_MAP[payload['param1']]
        print(f"Setting mode to {requested_mode}")

        try:
            self.set_mode_request.custom_mode = requested_mode
            response = self.set_mode_service.call(self.set_mode_request)
            print(response)
        except:
            print("Invalid mode requested")
            return

    def handle_arm(self, message, payload):
        '''
        Handle an arm/disarm command
        param1: 0 (disarm), 1 (arm)
        '''
        #try:
        if payload['param1'] == REQUEST_DISARM:
            self.arming_client_request.value = DISARM
        elif payload['param1'] == REQUEST_ARM:
            self.arming_client_request.value = ARM
        print(f"Swithing arm status to: {self.arming_client_request.value}")
        response = self.arming_client.call(self.arming_client_request)
        print(response)
        #except:
        #    print("Invalid arm/disarm command")

    def handle_takeoff(self, message, payload):
        '''
        Handle a takeoff command
        param1: altitude
        '''
        requested_altitude = float(payload['param1'])

        if requested_altitude == 0:
            #no given altitude
            home_alt = rospy.get_param("homeAlt")
            agent_alt = rospy.get_param("agentAlt")

            requested_altitude = agent_alt - home_alt

        #No real reason for these values:
        yaw = 0
        #try:
        self.takeoff_client_request.altitude = requested_altitude
        self.takeoff_client_request.yaw = yaw
        response = self.takeoff_client.call(self.takeoff_client_request)
        print(response)
        #except:
        #    print("Invalid takeoff command")

    def handle_set_collision_action(self, message, payload):
        if payload["param1"] == ENABLE_AVOIDANCE:
            mode = True
        elif payload["param1"] == DISABLE_AVOIDANCE:
            mode = False

        try:
            self.collision_action_pub.publish(mode)
        except:
            rospy.logwarn("Bad collision action mode")

    def handle_set_reach_time(self, message, payload):
        time = payload["param1"]
        try:
            self.set_reach_time_pub.publish(time)
        except:
            rospy.logwarn("bad reach time")

    def radio_traffic_callback(self, msg):
        '''
        When a message is received on /radio_traffic it needs to be unpacked and interpreted
        '''
        #Store the message in a dictionary
        message = unpack_message(msg.data)
        #Message is now a dict with fields: target, sendingAgent, messageType, rebroadcast, payload

        #Check if the message is for this agent
        if message['target'] != 0 and message['target'] != self.ID:
            return
        
        #Check if the message is from this agent
        if message['sendingAgent'] == self.ID:
            return
        
        if message["messageType"] == HYPERRECTANGLE:
                
            #reachable set received
            rcvd_rs = unpack_hyperrect_payload(message['sendingAgent'], message['payload'])
            send_time = rcvd_rs.rs_time.secs - 4
            recv_time = rospy.Time.now().to_sec()
            broadcast_time = (recv_time - send_time)
            br_time = f"{message['sendingAgent']}:{broadcast_time:.2f}"
            self.broadcast_time_pub.publish(br_time)
            self.rcvd_rs_pub.publish(rcvd_rs)

        elif message["messageType"] == COMMAND:

            #Execute a command
            command = unpack_command_payload(message['payload'])
            print(f"Command received of type {command['command']}")
            
            if command["command"] == "PING":
                self.handle_ping(message, command)
            elif command["command"] == "SET_MODE":
                self.handle_set_mode(message, command)
            elif command["command"] == "ARM":
                self.handle_arm(message, command)
            elif command["command"] == "TAKEOFF":
                self.handle_takeoff(message, command)
            elif command["command"] == "SET_COLLISION_ACTION":
                self.handle_set_collision_action(message, command)
            elif command["command"] == "SET_REACH_TIME":
                self.handle_set_reach_time(message, command)
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
        #Max size is 92 bytes? 66? 84?


        tempStr =  f"0:{self.ID}:0:0:{msg.x.min:.2f}:{msg.x.max:.2f}:{msg.y.min:.2f}:{msg.y.max:.2f}:{msg.z.min:.2f}:{msg.z.max:.2f}:{msg.rs_time.secs}"
        #print(len(tempStr))
        self.radio_pub.publish(tempStr)

    def __init__(self):
            
        rospy.init_node('agent_comms')
        self.ID = rospy.get_param('SystemID', 1)

        rospy.Subscriber('/radio_traffic', String, self.radio_traffic_callback) #subscribe to incoming radio traffic for commands
        self.radio_pub = rospy.Publisher('/radio_traffic', String, queue_size=10) #publish messages to radio traffic

        rospy.Subscriber('/my_reach_state', hyperRectangle, self.my_rs_callback) #subscribed to this agent's reachable set
        self.rcvd_rs_pub = rospy.Publisher('/recieved_rs', hyperRectangle, queue_size=10) #publish incoming reachable sets to the rest of the nodes

        self.broadcast_time_pub = rospy.Publisher('/broadcast_time', String, queue_size=10) #publish the time of the last broadcast
        self.collision_action_pub = rospy.Publisher('/collision_action', Bool, queue_size=1)
        self.set_reach_time_pub = rospy.Publisher('/reach_time', Float32, queue_size=1)

        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_mode_request = SetModeRequest()

        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arming_client_request = CommandBoolRequest()

        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.takeoff_client_request = CommandTOLRequest()
        
    def main_loop(self):
        print("Agent comms node running")
        rospy.spin()

if __name__ == "__main__":
    agent = agent_comms()
    agent.main_loop()