#!/usr/bin/env python3

import sys
import rospy
import time

from digi.xbee.devices import *
from digi.xbee.util import utils
from digi.xbee.exception import *

from std_msgs.msg import String

class commsBridge():
    """
    ROS node to bridge the xbee radio to the /radio_traffic topic

    verifier_comms_s  -----/radio_traffic_out ------>
                      <----/radio_traffic_in ------- xbee_bridge

    gcs_backend       -----/radio_traffic_out ------>
                      <----/radio_traffic_in ------- xbee_bridge



    
    """
    def traffic_out_callback(self, msg):
        """
        Callback triggered when a message is received on /radio_traffic_out. String needs to be encoded and broadcast on the radio
        """
        string_to_send = msg.data

        try:
            self.radio.send_data_broadcast(string_to_send)
        except TransmitException:
            print("Data block too long")

    def data_rcvd_callback(self,xb_msg):
        """
        Callback triggered with bits are recieved on the radio. bits need to be decoded and published to the /radio_traffic_in topic as a string
        """
        #Decode the message
        data = xb_msg.data.decode('utf8')

        #publish to topic
        self.traffic_in_pub.publish(data)

    def net_modified_callback(self, event_type, reason, node):
        """
        Callback triggered when network is modified
        """
        rospy.loginfo("  >>> Network event:")
        rospy.loginfo("              Type: %s (%d)" % (event_type.description, event_type.code))
        rospy.loginfo("              Reason: %s (%d)" % (reason.description, reason.code))

        if not node:
            return

        rospy.loginfo("              Node: %s" % node)

    def __init__(self):
        """
        Default Constructor to setup the node
        """
        rospy.init_node("mesh_radio")
        
        #Radio settings
        radio_port = '/dev/ttyXBEE'
        radio_baud = 57600

        #publisher that will publish recieved bits to verifier_comms on /radio_traffic_in
        self.traffic_in_pub = rospy.Publisher('/radio_traffic_in', String, queue_size=10)

        #subscriber that will listen to /radio_traffic_out and send them as bits over the radio
        rospy.Subscriber('/radio_traffic_out', String, self.traffic_out_callback)

        self.radio = XBeeDevice(radio_port, radio_baud)
        rospy.loginfo("Opening radio ...")
        self.radio.open()

        #attach the data reception callback
        rospy.loginfo("Attaching serial data callback ...")
        self.radio.add_data_received_callback(self.data_rcvd_callback)

        rospy.loginfo("Polling network ...")
        self.xnet = self.radio.get_network()
        self.xnet.start_discovery_process()
        while self.xnet.is_discovery_running():
            time.sleep(0.1)
        self.nodes = self.xnet.get_devices()
        rospy.loginfo("Found the following radios:")
        for node in self.nodes:
            rospy.loginfo(node.get_node_id())

        #Add network modification callback
        print("Activating network modification callback...")
        self.xnet.add_network_modified_callback(self.net_modified_callback)

    def main_loop(self):
        rospy.loginfo("Communication Bridge is running")
        rospy.spin()


if __name__ == "__main__":
    r = commsBridge()
    r.main_loop()