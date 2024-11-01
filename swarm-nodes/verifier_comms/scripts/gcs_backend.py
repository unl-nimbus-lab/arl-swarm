#!/usr/bin/env python3

from radio_object import build_ping_command, build_arm_command, build_disarm_command, build_guided_command, build_auto_command, build_brake_command, build_land_command, build_rtl_command, build_takeoff_command, build_disable_avoidance_command, build_enable_avoidance_command, build_set_reach_time_command, build_control_disable_command, build_control_enable_command, build_mission_command, build_set_lh_time_command
import rospy
from std_msgs.msg import String
import threading

from http.server import BaseHTTPRequestHandler, HTTPServer
import socketserver

"""
This node takes HTTP requests from the web interface and publishes them as strings onto the /radio_traffic topic 
"""

#Networking constants defined here
HOSTNAME = "127.0.0.1"
GETPORT = 8080

class MyServer(BaseHTTPRequestHandler):
    
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        split_slug = self.path.split('/')
        rospy.logwarn(split_slug)

        try:
            command = split_slug[1].upper()
            options = split_slug[2:]

            if command == "ARM":
                msg = build_arm_command(options)
                radio_pub.publish(msg)
            elif command == "DISARM":
                msg = build_disarm_command(options)
                radio_pub.publish(msg)
            elif command == "GUIDED":
                msg = build_guided_command(options)
                radio_pub.publish(msg)
            elif command == "RTL":
                msg = build_rtl_command(options)
                radio_pub.publish(msg)
            elif command == "LAND":
                msg = build_land_command(options)
                radio_pub.publish(msg)
            elif command == "AUTO":
                msg = build_auto_command(options)
                radio_pub.publish(msg)
            elif command == "BRAKE":
                msg = build_brake_command(options)
                radio_pub.publish(msg)
            elif command == "TAKEOFF":
                msg = build_takeoff_command(options)
                radio_pub.publish(msg)
            elif command == "COL_ON":
                msg = build_enable_avoidance_command(options)
                radio_pub.publish(msg)
            elif command == "COL_OFF":
                msg = build_disable_avoidance_command(options)
                radio_pub.publish(msg)
            elif command == "SET":
                msg = build_set_reach_time_command(options)
                radio_pub.publish(msg)
            elif command == "VEL_ON":
                msg = build_control_enable_command(options)
                radio_pub.publish(msg)
            elif command == "VEL_OFF":
                msg = build_control_disable_command(options)
                radio_pub.publish(msg)
            elif command == "MISSION":
                msg = build_mission_command(options)
                radio_pub.publish(msg)
            elif command == "PING":
                msg = build_ping_command(options)
                radio_pub.publish(msg)
            elif command == "SET_LH":
                msg = build_set_lh_time_command(options)
                radio_pub.publish(msg)

        except IndexError as e:
            rospy.logwarn(f"{e}")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()



def run(server_class=HTTPServer, handler_class=MyServer):
    server_address = ('', 8080)  # Run on localhost, port 8000
    httpd = server_class(server_address, handler_class)
    rospy.loginfo("Server running on port 8080...")
    httpd.serve_forever()

#def command_thread(publisher_obj):

if __name__ == "__main__":

    rospy.init_node('swarm_gcs')
    ID = 0 #ID stays fixed for GCS

    simulating = rospy.get_param("simulating", True)

    if (simulating):
        radio_pub = rospy.Publisher("/radio_traffic", String, queue_size=100)
    else:
        radio_pub = rospy.Publisher("/radio_traffic_out", String, queue_size = 100)

    rospy.loginfo("Starting Swarm GCS Node")
    run()

    while not rospy.is_shutdown():
        rospy.spin()
