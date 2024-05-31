#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def welcome_message():
    print("WELCOME TO THE SWARM GCS")
    print("Type 'help' for a list of commands")

def pack_message(messageList):
    message = ""
    for item in messageList:
        message += item + ":"

    message = message[:-1]
    return message


if __name__ == "__main__":

    rospy.init_node('gcs', anonymous=True)

    welcome_message()

    radioPub = rospy.Publisher('/radio_traffic', String, queue_size=10)

    while True:

        command_input = input(">> ")
        input_list = command_input.split(" ")
        messageList = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']
        messageString = ""

        command = input_list[0]

        if len(input_list) > 1:
            target = input_list[1]
        

        if input_list[0] == "ping":
            print("Pinging...")
            messageList[0] = "0" #Target is broadcast
            messageList[1] = "0" #Sending agent is GCS (0)
            messageList[2] = "1" #Message type is command
            messageList[3] = "0" #Not a rebroadcast
            messageList[4] = "0" #Command type is ping
            messageString = pack_message(messageList)
            print(messageString)
            radioPub.publish(messageString)

        elif input_list[0] == "exit":
            print("Exiting...")
            break

        else:    
            print("Invalid command")