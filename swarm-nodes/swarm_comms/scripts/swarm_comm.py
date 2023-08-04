#!/usr/bin/env python3
import sys
import serial
from digi.xbee.devices import *
from digi.xbee.util import utils
from digi.xbee.exception import *
import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PointStamped
from swarm_comms.msg import allAgentsPosStamped
from swarm_comms.msg import agentPos
from mavros_msgs.msg import SwarmGPS
from verifier_comms.msg import hyperRectangle
#Jack's agent class here:
from agent import Agent
from reachableState import reachableState

class TheRadio(object):
    '''Class to get xbee data and publish to ros topic'''

    def millis(self):
        return round(time.time() * 1000)

    def process_command(self, data):
        if (data=='g'):
            transmit_default_data()

    #The data_rcvd_callback is triggered when serial data is received
    def data_rcvd_callback(self,xb_msg):
        # time_received = self.millis()
        #print(xb_msg.data.decode('utf8')+'  at: '+ str(time_received))
        #address = xb_msg.remote_device.get_64bit_addr()
        #node_id = self.xnet.get_device_by_64(address).get_node_id()
        data = xb_msg.data.decode('utf8')
        #print(data + ' rcvd at: ' + str(time_received))
        #print("%s: %s" % (node_id, data))
        #add other stuff in here later as needed,
        #such as parsing commands, lat/longs, etc...

        #parse incoming string...
        #print("rcvd:%s" % data)
        #commands recvd from the ground station begin with 'C'
        if (data[0] == 'C'):
            self.process_command(data[1])
        else:
            id_num, xl, xh, yl, yh, zl, zh = self.parse_remote_agent_location(data)
            #self.store_remote_agent_data(data, time_received)
            #now publish remote agent data to the agent_pos topic
            remote_rs = hyperRectangle()
            # for agent in self.agent_list:
            #     a = agentPos()
            #     a.id = agent.node_id
            #     a.x = agent.x
            #     a.y = agent.y
            #     a.z = agent.z
            #     a.vx = agent.vx
            #     a.vy = agent.vy
            #     a.vz = agent.vz
            #     a.ix = agent.ix
            #     a.iy = agent.iy
            #     a.iz = agent.iz
            #     a.time_since_update = agent.time_between_locations
            #     allAgents.agents.append(a)

            remote_rs.x.min = xl
            remote_rs.x.max = xh
            remote_rs.y.min = yl
            remote_rs.y.max = yh
            remote_rs.z.min = zl
            remote_rs.z.max = zh

            # #add time to header
            remote_rs.header.stamp = rospy.Time.now()
                        
            # #publish to topic
            self.received_pub.publish(remote_rs)

    def parse_remote_agent_location(self,data):

        #agent strings start with 'a'
        if (data[0] == 'a'):
            #print('Agent string rcvd: ' + data)

            idx = 1
            temp_str = ''
            
            #grab the id number
            while (data[idx] != ':'):
                temp_str += data[idx]
                idx += 1
            id_num = int(temp_str)
            temp_str = ''

            #received string
            idx += 2
            while (data[idx] != 'x'):
                temp_str += data[idx]
                idx += 1
            xl = float(temp_str)
            temp_str = ''
            idx+=1
            
            while (data[idx] != 'y'):
                temp_str += data[idx]
                idx += 1
            xh = float(temp_str)
            temp_str = ''
            idx+=1
            
            while (data[idx] !='y'):
                temp_str += data[idx]
                idx += 1
            yl = float(temp_str)
            temp_str = ''
            idx+=1

            while (data[idx] !='z'):
                temp_str += data[idx]
                idx += 1
            yh = float(temp_str)
            temp_str = ''
            idx+=1

            while (data[idx] != 'z'):
                temp_str += data[idx]
                idx += 1
            zl = float(temp_str)
            temp_str = ''
            idx+= 1

            while (idx < len(data)):
                temp_str += data[idx]
                idx += 1
            zh = float(temp_str)

            return id_num, xl, xh, yl, yh, zl, zh

    #The net_modified_callback is triggered when the network is modified
    def net_modified_callback(self, event_type, reason, node):
        print("  >>> Network event:")
        print("              Type: %s (%d)" % (event_type.description, event_type.code))
        print("              Reason: %s (%d)" % (reason.description, reason.code))

        if not node:
            return

        print("              Node: %s" % node)

    def rs_callback(self, data):
        self.received_pub.publish(data)

    def update_rs(self, data):
        self.my_rs.xmin = data.x.min
        self.my_rs.xmax = data.x.max
        self.my_rs.ymin = data.y.min
        self.my_rs.ymax = data.y.max
        self.my_rs.zmin = data.z.min
        self.my_rs.zmax = data.z.max

    def mystate_callback(self, data):
        if (self.simming):
            self.rs_pub.publish(data)
        else:
            self.update_rs(data)


    def __init__(self):
        
        # grab the system id as the first command line arg
        if (len(sys.argv) > 1):
            self.sys_id = int(sys.argv[1])
            print(f"Setting sys_id to {self.sys_id}.")

        if (len(sys.argv) > 2):
            if (sys.argv[2] == 'sim'):
                self.simming = True
            else:
                self.simming = False
        else:
            self.simming = False

        print(f"Simming is set to {self.simming}.")

        if (len(sys.argv) > 3 and self.simming == True):
            numberOfAgents = int(sys.argv[3])
            print(f"Connecting to {numberOfAgents} simulated agents")

        #initialize the actual radio hardware if not simming
        if (self.simming == False):
            radio_port = '/dev/ttyXBEE'
            radio_baud = 57600

        self.my_rs = reachableState(self.sys_id)

        rospy.init_node(f"agent_{self.sys_id}_mesh_radio")
        
        #Setup simming publishers and subscribers
        if (self.simming == True):
            self.rs_pub = rospy.Publisher(f"/agent_{self.sys_id}_reachable_state", hyperRectangle, queue_size = 10)
            for i in range(1,numberOfAgents+1):
                if( i != self.sys_id):
                    rospy.Subscriber(f"agent_{i}_reachable_state", hyperRectangle, self.rs_callback)

        #Publishers and subscribers regarless of simming
        self.received_pub = rospy.Publisher(f"/agent_{self.sys_id}/received_state", hyperRectangle, queue_size = 10)
        rospy.Subscriber(f"/agent_{self.sys_id}/my_reach_state", hyperRectangle, self.mystate_callback)

        if (self.simming == False):
            self.radio = XBeeDevice(radio_port, radio_baud)
            print("Opening radio...")
            self.radio.open()

            #Add radio data reception callback
            print("Activating serial data callback...")
            self.radio.add_data_received_callback(self.data_rcvd_callback)


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

    def main_loop(self):
        print("Swarm Comms node is running.")

        #restrict transmission of pos to 10 Hz
        r = rospy.Rate(2)

        while not rospy.is_shutdown():
            #change this to publish the gps coords at 10Hz.
            #Use the serial callback to update the swarm agent
            #data structure

            #configure string position info
            radio_str = 'a'+str(self.sys_id)+':x'+str(self.my_rs.xmin)+'x'+str(self.my_rs.xmax)+'y'+str(self.my_rs.ymin)+'y'+str(self.my_rs.ymax)+'z'+str(self.my_rs.zmin)+'z'+str(self.my_rs.zmax)
            #print(radio_str)
            
            #store our own data here by calling self.store_remote_agent_data() and using radio_str
            
            #time_received = self.millis()
            #self.store_remote_agent_data(radio_str, time_received)

            #print("Sending: " + radio_str)
            #print(f"The length of radio_str is: {len(radio_str)}")

            ########################################################################3
            if (self.simming == False):
                #broadcast on radio if not simming
                try:
                    self.radio.send_data_broadcast(radio_str)
                except TransmitException:
                    print("Data block too long.")
                #print(radio_str)
            # else:
            #     #publish to topic
            #     #s = String()
            #     #s.data = radio_str;
            #     #self.radio_pub.publish(s)
            #     print("simming biotch")
            ########################################################################

            #debug print
            #print(radio_str)

            #check to see if more than 5 seconds have passed on any one
            #agent's update.  If so, remove that agent from the list
            # x = 0
            # for agent in self.agent_list:
            #     if (self.millis() - agent.last_received > 5000):
            #         print("Update delay > 5 seconds. Removing Agent " +str(agent.node_id) + ".")
            #         #remove the agent from the list
            #         self.agent_list.remove(agent)        
            #         #remove node from list
            #         self.node_list.remove(agent.node_id)

            r.sleep()

if __name__ == '__main__':
    r = TheRadio()
    r.main_loop()
