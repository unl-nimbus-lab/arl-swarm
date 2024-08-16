#!/usr/bin/env python3

from radio_object import unpack_message, unpack_hyperrect_payload, unpack_command_payload, MODE_MAP, COMMAND_MAP, REBROADCAST, NO_REBROADCAST, HYPERRECTANGLE, COMMAND, DISARM, ARM, PING_REQUESTED, PING_RESPONSE, REQUEST_DISARM, REQUEST_ARM, ping_command, set_mode_command, arm_command, build_ping_command, build_arm_command, build_disarm_command, build_guided_command, build_auto_command, build_brake_command, build_land_command, build_rtl_command, build_takeoff_command, build_disable_avoidance_command, build_enable_avoidance_command, build_set_reach_time_command
import rospy
from std_msgs.msg import String
import threading

def radio_callback(msg):
    pass

def user_command_thread(publisher_obj):
    #User chooses commands
    #user chooses command options
    #Command Options: target (defaults to 0, rebroadcast, defaluts to NO)
    #Command Specific options
    # availible_commands = []
    # for command in COMMAND_MAP.values():
    #     availible_commands.append(command)

    while not rospy.is_shutdown():
        requested_command = input(f"gcs >>> ").split()

        try:
            command = requested_command[0].upper()
            options = requested_command[1:]

            if command == "EXIT":
                rospy.signal_shutdown("goodbye ...")
            elif command == "PING":
                msg = build_ping_command(options)
                publisher_obj.publish(msg)
            elif command == "ARM":
                msg = build_arm_command(options)
                publisher_obj.publish(msg)
            elif command == "DISARM":
                msg = build_disarm_command(options)
                publisher_obj.publish(msg)
            elif command == "GUIDED":
                msg = build_guided_command(options)
                publisher_obj.publish(msg)
            elif command == "RTL":
                msg = build_rtl_command(options)
                publisher_obj.publish(msg)
            elif command == "LAND":
                msg = build_land_command(options)
                publisher_obj.publish(msg)
            elif command == "AUTO":
                msg = build_auto_command(options)
                publisher_obj.publish(msg)
            elif command == "BRAKE":
                msg = build_brake_command(options)
                publisher_obj.publish(msg)
            elif command == "TAKEOFF":
                msg = build_takeoff_command(options)
                publisher_obj.publish(msg)
            elif command == "ON":
                msg = build_enable_avoidance_command(options)
                publisher_obj.publish(msg)
            elif command == "OFF":
                msg = build_disable_avoidance_command(options)
                publisher_obj.publish(msg)
            elif command == "SET":
                msg = build_set_reach_time_command(options)
                publisher_obj.publish(msg)
            else:
                print("wut")
        except IndexError as e:
            rospy.logwarn(f"{e}")

if __name__ == "__main__":
    
    rospy.init_node('swarm_gcs')
    ID = 0 #Fixed at 0 for the GCS

    rospy.Subscriber("/radio_traffic", String, radio_callback)
    radio_pub = rospy.Publisher("/radio_traffic", String, queue_size=100)

    command_thread = threading.Thread(target=user_command_thread, args=(radio_pub,))
    command_thread.start()

    while not rospy.is_shutdown():
        rospy.spin()