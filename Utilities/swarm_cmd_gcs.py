import sys
import serial
from digi.xbee.devices import *
from digi.xbee.util import utils
from digi.xbee.exception import *

def net_modified_callback(self, event_type, reason, node):
        print("  >>> Network event:")
        print("              Type: %s (%d)" % (event_type.description, event_type.code))
        print("              Reason: %s (%d)" % (reason.description, reason.code))

        if not node:
            return

        print("              Node: %s" % node)


if __name__ == "__main__":

    #Connect to the radio
    radio_port = "/dev/ttyUSB0"
    radio_baud = 57600
    radio = XBeeDevice(radio_port, radio_baud)
    radio.open()

    #Get the network
    print("Polling network...")
    xnet = radio.get_network()
    xnet.start_discovery_process()
    while xnet.is_discovery_running():
        time.sleep(0.1)
    nodes = xnet.get_devices()
    print("Found the following radios:")
    for node in nodes:
        print(node.get_node_id())

    #Add network modification callback
    print("Activating network modification callback...")
    xnet.add_network_modified_callback(net_modified_callback)


    while True:
        command_input = input(">> ")

        #See which radios are out there
        if command_input == "list":
            print("Found the following radios:")
            for node in nodes:
                print(node.get_node_id())

        #Contact ROS
        elif command_input == "whos":
            print("Seeing who's out there...")
            xnet.start_discovery_process()
            while xnet.is_discovery_running():
                time.sleep(0.1)
            nodes = xnet.get_devices()
            print("Found the following radios:")
            for node in nodes:
                print(node.get_node_id())

            try:
                radio.send_data_broadcast("C0A0")
            except TransmitException:    
                print("Failed to send command")

        #Start ROS
        elif command_input == "start":
            print("Starting ROS...")
            try:
                radio.send_data_broadcast("C1A0")
            except TransmitException:
                print("Failed to send command")

        #Stop ROS
        elif command_input == "stop":
            print("Stopping ROS...")
            try:
                radio.send_data_broadcast("C2A0")
            except TransmitException:
                print("Failed to send command")

        #Arm
        elif command_input == "arm":
            print("Arming...")
            try:
                radio.send_data_broadcast("C3A0")
            except TransmitException:
                print("Failed to send command")

        #Disarm
        elif command_input == "disarm":
            print("Disarming...")
            try:
                radio.send_data_broadcast("C4A0")
            except TransmitException:
                print("Failed to send command")

        #Takeoff
        elif command_input == "takeoff":
            print("Taking off...")
            try:
                radio.send_data_broadcast("C5A0")
            except TransmitException:
                print("Failed to send command")

        #Land
        elif command_input == "land":
            print("Landing...")
            try:
                radio.send_data_broadcast("C6A0")
            except TransmitException:
                print("Failed to send command")

        #Guided
        elif command_input == "guided":
            print("Guided mode...")
            try:
                radio.send_data_broadcast("C7A0")
            except TransmitException:
                print("Failed to send command")

        #RTL
        elif command_input == "rtl":
            print("RTL...")
            try:
                radio.send_data_broadcast("C8A0")
            except TransmitException:
                print("Failed to send command")


        
        
        elif command_input == "exit":
            break

        else:
            print("Unknown command")