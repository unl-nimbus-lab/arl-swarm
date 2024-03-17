#!/usr/bin/env python3
import sys
import serial
import rospy
import re
from digi.xbee.devices import *
from digi.xbee.util import utils
from digi.xbee.exception import *
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class TheRadio(object):
    '''Class to get xbee radio working'''

    started = False
    stopped = True
    armed   = False
    takeoff = False
    guided  = False
    rtl     = False
    land    = False
    disarm  = False

    def data_received_callback(self, xb_msg):
        data = xb_msg.data.decode('utf-8')
        print("Data received: " + data)

        command_sender = re.split(r'[CA]',data)
        command = command_sender[1]
        sender  = command_sender[2]

        if command == '0':
            print("HERE!")


        elif command == '1':
            print("Starting HRL Publishing from swarm_cmd")

        elif command == '2':
            print("Stopping HRL Publishing from swarm_cmd")

        elif command == '3':
            arm_cmd.value = True
            arming_client.call(arm_cmd)
            print("Arming from swarm_cmd")

        elif command == '4':
            arm_cmd.value = False
            arming_client.call(arm_cmd)
            print("Disarming from swarm_cmd")

        elif command == '5':
            print("Taking off from swarm_cmd")

        elif command == '6':
            set_mode_client.call(land_set_mode)
            print("Landing from swarm_cmd")

        elif command == '7':
            set_mode_client.call(guided_set_mode)
            print("Set guided from swarm_cmd")

        elif command == '8':
            set_mode_client.call(rtl_set_mode)
            print("RTL from swarm_cmd")

        elif command == '9':
            print("command 9 does nothing")

        else:
            print("unrecognized command")

    def __init__(self):

        radio_port = "/dev/ttyXBEE"
        radio_baud = 57600

        self.radio = XBeeDevice(radio_port, radio_baud)
        self.radio.open()

        self.radio.add_data_received_callback(self.data_received_callback)
        
        print("Polling network...")
        self.xnet = self.radio.get_network()
        self.xnet.start_discovery_process()
        while self.xnet.is_discovery_running():
            time.sleep(0.1)
        self.nodes = self.xnet.get_devices()
        print("Found the following radios:")
        for node in self.nodes:
            print(node.get_node_id())

        #Add network modification callback
        print("Activating network modification callback...")
        self.xnet.add_network_modified_callback(self.net_modified_callback)

    #The net_modified_callback is triggered when the network is modified
    def net_modified_callback(self, event_type, reason, node):
        print("  >>> Network event:")
        print("              Type: %s (%d)" % (event_type.description, event_type.code))
        print("              Reason: %s (%d)" % (reason.description, reason.code))

        if not node:
            return

        print("              Node: %s" % node)


if __name__ == '__main__':
    rospy.init_node('swarm_cmd')
    
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    arming_client   = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    
    guided_set_mode = SetModeRequest()
    guided_set_mode.custom_mode = 'GUIDED'

    land_set_mode = SetModeRequest()
    land_set_mode.custom_mode = 'LAND'

    rtl_set_mode = SetModeRequest()
    rtl_set_mode.custom_mode = 'RTL'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    radio = TheRadio()
    rospy.spin()