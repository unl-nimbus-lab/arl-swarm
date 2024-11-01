#!/usr/bin/env python3
import sys
import serial

import rospy
from std_msgs.msg import String
from verifier_comms.msg import hyperRectangle


#Radio traffic message protocol:
# {target}:{sending agent}:{message type}:{rebroadcast flag}:{payload}
#     0           1               2               3               4
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
    # 3
    # 3 - enable/disable collision avoidance
    # 4 - enable/diable re-broadcasting

#Incase you have been away for a while, the plan is to have this serve as a base class that will be used by the agents as well as the ground
#station to handle radio traffic

REBROADCAST = 1
NO_REBROADCAST = 0

ENABLE_AVOIDANCE = 1
DISABLE_AVOIDANCE = 0

ENABLE_CONTROL = 1
DISABLE_CONTROL = 0

_COMMAND_MAP = {
    "PING" :    0,
    "SET_MODE": 1,
    "ARM":      2,
    "TAKEOFF" : 3,
    "SET_COLLISION_ACTION" : 4,
    "SET_REACH_TIME": 5,
    "SET_MISSION": 6,
    "ENABLE_POS_CONTROL": 7,
    "SET_LH_TIME": 8,
}

_MODE_MAP = {
    "STABILIZE":    0,
    "GUIDED":       1,
    "AUTO":         2,
    "RTL":          3,
    "LOITER":       4,
    "LAND":         5,
    "BRAKE":        6
}

MODE_MAP = {
    0: "STABILIZE",
    1: "GUIDED",
    2: "AUTO",
    3: "RTL",
    4: "LOITER",
    5: "LAND",
    6: "BRAKE"
}

COMMAND_MAP = {
    0: "PING",
    1: "SET_MODE",
    2: "ARM",
    3: "TAKEOFF",
    4: "SET_COLLISION_ACTION",
    5: "SET_REACH_TIME",
    6: "SET_MISSION",
    7: "ENABLE_POS_CONTROL",
    8: "SET_LH_TIME",
}

#Message type macros
HYPERRECTANGLE = 0
COMMAND = 1

#Arming MACROS
DISARM = False
ARM = True
REQUEST_DISARM = 0
REQUEST_ARM = 1

#ping macros
PING_REQUESTED = 0
PING_RESPONSE = 1

UNIVERSAL_COMMAND_OPTIONS = "target: -t [target_agent], rebroadcast: -r "
SET_MODE_COMMAND_OPTIONS = "mode: -m [STABILIZE GUIDED AUTO RTL LOITER LAND]"
ARM_COMMAND_OPTIONS = "disarm: -d"
ENABLE_COMMAND_OPTIONS = ""


class command:
    """
    Command class that can be used to help build messages
    """
    def __init__(self):
        
        self.command_key = None #Message type essentally, see COMMAND_MAP and _COMMAND_MAP
        
        self.target = 0 #Defaults to the "broadcast so that all listening agents will hear"
        self.sending_agent = 0 #Defaults to the GCS sending agent
        
        self.message_type = COMMAND 
        self.rebroadcast_flag = NO_REBROADCAST
        
        self.command = None
        self.payload = None

        self.param1 = 0
        self.param2 = 0
        self.param3 = 0
        self.param4 = 0
        self.param5 = 0
        self.param6 = 0
        self.param7 = 0

        self.msg_str = None

    def load(self, message_string):
        """
        NOT USED? Used to populate a received message
        """
        message_dict = unpack_message(message_string)
        
        self.target = message_dict["target"]
        self.sending_agent = message_dict["sendingAgent"]
        self.rebroadcast_flag = message_dict["rebroadcast"]
        self.payload = message_dict["payload"]
        self.command = _COMMAND_MAP[self.payload[0]]

    def reset(self):
        """
        NOT USED?
        """
        self.command_key = None
        self.target = 0
        self.sending_agent = 0
        self.message_type = COMMAND
        self.rebroadcast_flag = NO_REBROADCAST
        self.command = None
        self.payload = None

        self.param1 = 0
        self.param2 = 0
        self.param3 = 0
        self.param4 = 0
        self.param5 = 0
        self.param6 = 0
        self.param7 = 0

        self.msg_str = None

    def check_universal_options(self, options):
        """
        options avalible to all types of commands

        -t [target agent]: specify an agent to send this message to 
        -r : rebroadcast this message
        """

        for idx in range(len(options)):
            if options[idx] == "-t":
                try:
                    self.target = int(options[idx+1])
                except ValueError as e:
                    print(f"{e}: bad options")
            elif options[idx] == "-r":
                self.rebroadcast_flag = REBROADCAST

    def check_str(self,input_string):
        """
        Simple checksum to make sure the built message has at least 12 fields
        """
        list = input_string.split(":")
        return len(list) >= 12

    def pack_payload(self):
        """
        Once the command message options and param are set, the are built into a string 
        """
        self.payload = f"{self.command}:{self.param1}:{self.param2}:{self.param3}:{self.param4}:{self.param5}:{self.param6}:{self.param7}"
    
    def build_message(self):
        """
        Once the command message options and param are set, the are built into a string 
        """
        self.payload = f"{self.command}:{self.param1}:{self.param2}:{self.param3}:{self.param4}:{self.param5}:{self.param6}:{self.param7}"
        temp_msg_str = f"{self.target}:{self.sending_agent}:{COMMAND}:{self.rebroadcast_flag}:{self.payload}"
        if (self.check_str(temp_msg_str)):
            self.msg_str = temp_msg_str
        else:
            print("The message string did not pass the checksum")

    def __str__(self):
        return self.msg_str
    
    def prompt_options(self):
        raise NotImplementedError("This method must be implemented in the child class")
    
class ping_command(command):
    """
    Subclass for a ping command

    param1: UNUSED
    param2: UNUSED
    param3: UNUSED
    param4: UNUSED
    param5: UNUSED
    param6: UNUSED
    param7: UNUSED
    
    """
    def __init__(self):
        
        command.__init__(self)

        self.command_key = "PING"
        self.command = _COMMAND_MAP[self.command_key]

    # def prompt_options(self):
    #     return input(f"Options for {self.command_key} command {UNIVERSAL_COMMAND_OPTIONS} >>> ").split()
    
    def process_options(self, options):
        """
        Specify options to modify the command
        command options: list[str]
            -t [target agent id]: specific agent to send message to
            -r : rebroadcast message
        """
        self.check_universal_options(options)

    # def build_from_prompt(self):
    #     options = self.prompt_options()
    #     self.process_options(options)
    #     self.build_message()

class set_mode_command(command):
    """
    
    """
    
    def __init__(self):
        command.__init__(self)

        self.command_key = "SET_MODE"
        self.command = _COMMAND_MAP[self.command_key]

    def check_mode_option(self,options):
        for idx in range(len(options)):
            if options[idx] == "-m":
                self.param1 = _MODE_MAP[options[idx+1]]

    # def prompt_options(self):
    #     return input(f"{self.command_key}:{UNIVERSAL_COMMAND_OPTIONS}{SET_MODE_COMMAND_OPTIONS} >>> ").split()

    def process_options(self, options):
        self.check_universal_options(options)

    # def build_from_prompt(self):
    #     options = self.prompt_options()
    #     self.process_options(options)
    #     self.build_message()

class arm_command(command):
    """
    Subclass for arm command

    param1: 1 (ARM), 0 (DISARM)
    param2: UNUSED
    param3: UNUSED
    param4: UNUSED
    param5: UNUSED
    param6: UNUSED
    param7: UNUSED

    """

    def __init__(self):
        
        command.__init__(self)

        self.command_key = "ARM"
        self.command = _COMMAND_MAP[self.command_key]

        self.param1 = REQUEST_ARM #Behavior defaults to arming

    # def prompt_options(self):
    #     return input(f"{self.command_key}:{UNIVERSAL_COMMAND_OPTIONS}{ARM_COMMAND_OPTIONS} >>> ").split()
    
    def process_options(self,options):
        """
        Specify options to modify the command
        command options: list[str]
            -t [target agent id]: specific agent to send message to
            -r : rebroadcast message
        """
        self.check_universal_options(options)

    # def build_from_prompt(self):
    #     options = self.prompt_options()
    #     self.process_options(options)
    #     self.build_message()

class takeoff_command(command):
    """
    Subclass for a takeoff command

    param1: takeoff altitude (meters AGL) (if left to 0, agent will take off to alt sep) (not checked for) 
    param2: UNUSED
    param3: UNUSED
    param4: UNUSED
    param5: UNUSED
    param6: UNUSED
    param7: UNUSED
    
    """
    
    def __init__(self):

        command.__init__(self)
        self.command_key = "TAKEOFF"
        self.command = _COMMAND_MAP[self.command_key]

    def process_options(self,options):
        """
        Specify options to modify the command
        command options: list[str]
            -t [target agent id]: specific agent to send message to
            -r : rebroadcast message
        """
        self.check_universal_options(options)
        # try:
        #     self.param1 = float(options[0])
        # except ValueError as e:
        #     self.param1 = 0

class avoidance_command(command):
    """
    Subclass for a command to enable collision avoidance

    param1: 1 (Enable avoidance), 0 (disable avoidance)
    param2: UNUSED
    param3: UNUSED
    param4: UNUSED
    param5: UNUSED
    param6: UNUSED
    param7: UNUSED
    
    """

    def __init__(self):

        command.__init__(self)
        self.command_key = "SET_COLLISION_ACTION"
        self.command = _COMMAND_MAP[self.command_key]

    def process_options(self,options):
        """
        Specify options to modify the command
        command options: list[str]
            -t [target agent id]: specific agent to send message to
            -r : rebroadcast message
        """
        self.check_universal_options(options)

class reach_time_command(command):
    """
    Subclass for a command that sets the reach time

    param1: reach time (seconds)
    param2: UNUSED
    param3: UNUSED
    param4: UNUSED
    param5: UNUSED
    param6: UNUSED
    param7: UNUSED
    
    """
    
    def __init__(self):

        command.__init__(self)
        self.command_key = "SET_REACH_TIME"
        self.command = _COMMAND_MAP[self.command_key]

    def process_options(self,options):
        """
        Specify options to modify the command
        command options: list[str]

        """
        self.check_universal_options(options)
        try:
            self.param1 = float(options[0])
        except ValueError as e:
            print(f"{e}")

class lh_time_command(command):
    """
    Subclass for a command that sets the lookahead time

    param1: lookahead time (seconds)
    param2: UNUSED
    param3: UNUSED
    param4: UNUSED
    param5: UNUSED
    param6: UNUSED
    param7: UNUSED
    
    """
    
    def __init__(self):

        command.__init__(self)
        self.command_key = "SET_LH_TIME"
        self.command = _COMMAND_MAP[self.command_key]

    def process_options(self,options):
        """
        Specify options to modify the command
        command options: list[str]

        """
        self.check_universal_options(options)
        try:
            self.param1 = float(options[0])
        except ValueError as e:
            print(f"{e}")

class control_enable_command(command):
    """
    Subclass for a command that enables/disables velocity control node

    param1: 1 : Enable
            0 : Disable
    param2: UNUSED
    param3: UNUSED
    param4: UNUSED
    param5: UNUSED
    param6: UNUSED
    param7: UNUSED
    
    """

    def __init__(self):
        command.__init__(self)
        self.command_key = "ENABLE_POS_CONTROL"
        self.command = _COMMAND_MAP[self.command_key]

    def process_options(self, options):
        """
        Specify options to modify the command
        command options: list[str]
            -t : [target agent id]: specific agent to send message to
            -r : rebroadcast message
        """
        self.check_universal_options(options)
   
class mission_command(command):
    """
    Subclass for a command that sets the next mission waypoint

    param1: 1 (ENBALE CONTROL), 0 (DISABLE CONTROL)
    param2: UNUSED
    param3: UNUSED
    param4: UNUSED
    param5: UNUSED
    param6: UNUSED
    param7: UNUSED    
    """

    def __init__(self):
        command.__init__(self)
        self.command_key = "SET_MISSION"
        self.command = _COMMAND_MAP[self.command_key]

    def process_options(self, options):
        """
        Specify options to modify the command
        command options: list[str]
            -t : [target agent id]: specific agent to send message to
            -r : rebroadcast message
            -m : [mission ID] (int)
            -s : [mission stage] (int)
        """
        self.check_universal_options(options)
        
        #Now check for command specific options
        for idx in range(len(options)):

            if options[idx] == "-m":
                try:
                    self.param1 = int(options[idx+1])
                except ValueError as e:
                    print(f"{e}: bad options for mission id")
            elif options[idx] == "-s":
                try:
                    self.param2 = int(options[idx+1])
                except ValueError as e:
                    print(f"{e}: bad options for mission stage")

def build_ping_command(options):
    ping = ping_command()
    ping.process_options(options)
    ping.build_message()

    return ping.msg_str

def build_arm_command(options):
    arm = arm_command()
    arm.param1 = REQUEST_ARM
    arm.process_options(options)
    arm.build_message()

    return arm.msg_str

def build_disarm_command(options):
    disarm = arm_command()
    disarm.param1 = REQUEST_DISARM
    disarm.process_options(options)
    disarm.build_message()

    return disarm.msg_str

def build_guided_command(options):
    guided = set_mode_command()
    guided.param1 = _MODE_MAP["GUIDED"]
    guided.process_options(options)
    guided.build_message()

    return guided.msg_str

def build_rtl_command(options):
    rtl = set_mode_command()
    rtl.param1 = _MODE_MAP["RTL"]
    rtl.process_options(options)
    rtl.build_message()

    return rtl.msg_str

def build_land_command(options):
    land = set_mode_command()
    land.param1 = _MODE_MAP["LAND"]
    land.process_options(options)
    land.build_message()

    return land.msg_str

def build_auto_command(options):
    auto = set_mode_command()
    auto.param1 = _MODE_MAP["AUTO"]
    auto.process_options(options)
    auto.build_message()

    return auto.msg_str

def build_brake_command(options):
    brake = set_mode_command()
    brake.param1 = _MODE_MAP["BRAKE"]
    brake.process_options(options)
    brake.build_message()

    return brake.msg_str

def build_takeoff_command(options):
    takeoff = takeoff_command()
    takeoff.process_options(options)
    takeoff.build_message()

    return takeoff.msg_str

def build_enable_avoidance_command(options):
    enable_action = avoidance_command()
    enable_action.process_options(options)
    enable_action.param1 = ENABLE_AVOIDANCE
    enable_action.build_message()

    return enable_action.msg_str

def build_disable_avoidance_command(options):
    disable_action = avoidance_command()
    disable_action.process_options(options)
    disable_action.param1 = DISABLE_AVOIDANCE
    disable_action.build_message()

    return disable_action.msg_str

def build_set_reach_time_command(options):
    set = reach_time_command()
    set.process_options(options)
    set.build_message()

    return set.msg_str

def build_set_lh_time_command(options):
    set = lh_time_command()
    set.process_options(options)
    set.build_message()

    return set.msg_str

def build_control_enable_command(options):
    enable_cmd = control_enable_command()
    enable_cmd.param1 = ENABLE_CONTROL
    enable_cmd.process_options(options)
    enable_cmd.build_message()

    return enable_cmd.msg_str

def build_control_disable_command(options):
    disable_cmd = control_enable_command()
    disable_cmd.param1 = DISABLE_CONTROL
    disable_cmd.process_options(options)
    disable_cmd.build_message()

    return disable_cmd.msg_str

def build_mission_command(options):
    mission_cmd = mission_command()
    mission_cmd.process_options(options)
    mission_cmd.build_message()

    return mission_cmd.msg_str

#Pretty sure most of these methods below here are unused
#UPDATE: Some the the methods are used

def pack_payload(payload):
    '''
    Packs a payload into a string
    '''
    tempStr = ""
    for i in range(len(payload)):
        tempStr += str(payload[i]) + ":"
    return tempStr

def pack_message(target, sendingAgent, messageType, rebroadcast, payload):
    '''
    Packs a message into the radio traffic message protocol
    '''

    sting_payload = pack_payload(payload)

    tempStr = str(target) + ":" + str(sendingAgent) + ":" + str(messageType) + ":" + str(rebroadcast) + ":" + sting_payload
    return tempStr

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
    messageDict['rebroadcast'] = str(messageList[3])
    messageDict['payload'] = messageList[4:] #This gets parsed in a different function depending on the message type, leave as a string for now

    return messageDict

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
        tempHyperRect.rs_time.secs = int(payload_str[6])

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
        "param7": float(payload_str[7]),
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
