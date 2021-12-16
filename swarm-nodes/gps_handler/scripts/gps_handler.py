#!/usr/bin/env python

import rospy
import time
import threading
import atexit
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega
# Import Custom Message



class GpsHandler:
    """
    ROS node intended to read incoming GLOBAL_POSITION_INT MAVLink messages and republish
    properly formatted messages
    """
    def __init__(self, outgoing_gps_topic: str, port: str, baud: int, source_system: int, source_component: int) -> None:
        # self.gps_publisher = rospy.Publisher(outgoing_gps_topic, CHANDIMACUSTOMMSG, queue_size=1)

        # Create a new MAVLink master
        self.master = mavutil.mavlink_connection(port, 
                                                 baud=baud, 
                                                 source_system=source_system, 
                                                 source_component=source_component,
                                                 autoreconnect=True)

        # Ensure that a connection has been successfully established
        # Integrate a 2 second timeout
        resp = self.master.wait_heartbeat(timeout=2)

        # Class variables
        self.connected = False

        # Threads
        self.heartbeat_t = threading.Thread(target=self.__heartbeat)
        self.heartbeat_t.daemon = True

        self.incoming_msg_t = threading.Thread(target=self.__message_handler)
        self.incoming_msg_t.daemon = True

        # Register the exit callback
        atexit.register(self.disconnect)


    def start(self) -> None:
        """
        Helper method used to start the threads
        """
        self.connected = True

        self.heartbeat_t.start()
        self.incoming_msg_t.start()

        return

    
    def disconnect(self) -> None:
        """
        Helper method used to stop all threads
        """
        self.connected = False

        if self.heartbeat_t is not None:
            self.heartbeat_t.join()

        if self.incoming_msg_t is not None:
            self.incoming_msg_t.join()

        if self.master is not None:
            self.master.close()

        return


    def __heartbeat(self) -> None:
        """
        Send a heartbeat to the network 
        """
        while self.connected:
            self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GENERIC, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

            # Send a heartbeat every 2 seconds
            time.sleep(2)

        return


    def __message_handler(self) -> None:
        """
        Main loop used to handle incoming messages from the Mavlink network
        """
        while self.connected:
            # Read a new message
            try:
                msg = self.master.recv_msg()
            except mavutil.mavlink.MAVError as e:
                self.logger.debug('An error occurred on MAVLink message reception')
                msg = None
            except Exception:
                # Log any other unexpected exception
                self.logger.exception('Exception while receiving message: ', exc_info=True)
                msg = None
            
            if not msg:
                continue

            # Handle the GPS message and republish the desired ROS message
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                # Read the message values of interest
                sys_id = msg.get_srcSystem()
                comp_id = msg.get_srcComponent()
                lat = msg.lat / 1.0e7
                lon = msg.lon / 1.0e7
                alt = msg.alt / 1000
                vx = msg.vx / 100
                vy = msg.vy / 100
                vz = msg.vz / 100

                # TODO: Construct and publish ROS msg                

        return


def main():
    # Initialize the node
    rospy.init_node('gps_handler_node', anonymous=True)
    
    # Load system params
    gps_topic = rospy.get_param('gps_topic', '/hrl/gps/')
    port = rospy.get_param('port', '/dev/ttyS0')
    baud = rospy.get_param('baudrate', 921600)
    source_system = rospy.get_param('source_system', 230)
    source_component = rospy.get_param('source_component', 3)

    # Create a new GPS handler
    handler = GpsHandler(gps_topic, port, baud, source_system, source_component)

    # Start the handler
    handler.start()

    # Run until the node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass