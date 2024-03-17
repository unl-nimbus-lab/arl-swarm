#!/usr/bin/env python3
import sys
import serial
import rospy
from digi.xbee.devices import *
from digi.xbee.util import utils
from digi.xbee.exception import *

class TheRadio(object):
    '''Class to get xbee radio working'''

    def data_received_callback(self, xb_msg):
        data = xb_msg.data.decode('utf-8')
        print("Data received: " + data)

    def __init__(self):
        rospy.init_node('swarm_cmd')

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
    radio = TheRadio()
    rospy.spin()
