#!/usr/bin/env python3

import sys
import rospy
import subprocess

from mavros_msgs.msg import State

class loggerHandler():

    def start_logger(self):
        """Start the subprocess if it's not already running."""
        if self.process is None:
            rospy.loginfo("Starting Logger...")
            self.process = subprocess.Popen(["roslaunch", "verifier", "logger.launch"])
            rospy.loginfo("Subprocess started with PID: %s", self.process.pid)

    def stop_logger(self):
        """Stop the subprocess if it's running."""
        if self.process is not None:
            rospy.loginfo("Stopping subprocess with PID: %s", self.process.pid)
            self.process.terminate()  # Sends SIGTERM
            self.process.wait()  # Wait for the process to terminate
            self.process = None
            rospy.loginfo("Logger stopped.")

    
    def state_callback(self, msg):
        """
        Callback from state message to start logger node if needed
        logger should be started if the vehicle has switched from disarmed -> armed

        """
        armed = msg.armed

        if (armed == True and self.previously_armed == False):
            #Start the logger node
            self.start_logger()

        elif (armed == False and self.previously_armed == True):
            #Stop the logger node
            self.stop_logger()

        self.previously_armed = armed
    
    def __init__(self):

        self.process = None
        self.previously_armed = False

        rospy.init_node('logger_handler')

        rospy.Subscriber('/mavros/state', State, self.state_callback)
        
if __name__ == '__main__':
    
    logger_handler = loggerHandler()
    rospy.spin()
