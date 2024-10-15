#!/usr/bin/env python3

from mission import missions

import rospy


from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import re

#This node will store 1-3 missions that will be started and advanced using the verifier comms package
#need to redo the takeoff command using the agent alt stuff
#congrats, you can still write a "node"

"""
homeAlt (the ground)
commonAlt (altitude mission takes place)
agentAlt (alt sep)

"""
common_alt = rospy.get_param("commonAlt", 410)
agent_alt = rospy.get_param("agentAlt", 405)

def check_cmd(cmd_input):
    pattern = r'^\d+:\d+$'

    if re.match(pattern, cmd_input):
        return True
    else:
        return False

class swarm_planner:

    def __init__(self):
        
        rospy.init_node("swarm_planner")
        
        self.ID = rospy.get_param('SystemID',1)
        self.common_alt = rospy.get_param("commonAlt", 410)
        self.agent_alt = rospy.get_param("agentAlt", 405)

        self.altitude_dict = {
            "common" : self.common_alt,
            "sep" : self.agent_alt,
        }
        
        #The altitude the drone will jump to if needed:
        self.alt_buffer = rospy.get_param("alt_buffer", 10)

        rospy.Subscriber('/mission_cmd',String, self.mission_cmd_callback)
        #rospy.Subscriber('/agent_1/collision_checker/sub')

        self.reference_pub = rospy.Publisher('/pos_ref', PointStamped, queue_size=1)

        #Load the default position targets to be m1s1 at the agent altitude
        self.x_pos_target = missions["m1"]["s1"]["a" + str(self.ID)]["x"]
        self.y_pos_target = missions["m1"]["s1"]["a" + str(self.ID)]["y"]
        self.z_pos_target = self.altitude_dict["sep"]

    def get_mission_keys(self, lookup_params):
        #build the mission keys into a dictionary
        mission = "m" + lookup_params[0]
        stage = "s" + lookup_params[1]
        agent = "a" + str(self.ID)

        return {"mission" : mission,
                "stage" : stage,
                "agent" : agent}
 
    def mission_lookup(self, mission_keys, mission_dict):
        mission = mission_keys["mission"]
        stage = mission_keys["stage"]
        agent = mission_keys["agent"]

        x = self.x_pos_target
        y = self.y_pos_target

        try:
            x = mission_dict[mission][stage][agent]["x"]
            y = mission_dict[mission][stage][agent]["y"]

        except KeyError as e:
            print("error looking up mission, no changes to xy")

        return x, y

    def altitude_lookup(self, mission_keys, mission_dict, altitude_dict):
        "Figure out which altitude to use here"
        mission = mission_keys["mission"]
        stage = mission_keys["stage"]
        agent = mission_keys["agent"]

        altitude = self.z_pos_target

        try:
            altitude_key = mission_dict[mission][stage][agent]["z"]
            altitude = altitude_dict[altitude_key]
        except KeyError as e:
            print("error looking up mission, no changes to altitude")

        return altitude

    def mission_cmd_callback(self, msg):
        """
        Unpack a cmd message and update the position reference
        """
        payload = msg.data
        
        #Check the message data is properly formatted
        if(check_cmd(payload) == False):
            rospy.logwarn("PLANNER: Bad CMD message")
            return
        
        message_fields = payload.split(":")
        mission_keys = self.get_mission_keys(message_fields)

        #get the new mission targets
        self.x_pos_target, self.y_pos_target = self.mission_lookup(mission_keys, missions)
        self.z_pos_target = self.altitude_lookup(mission_keys, missions, self.altitude_dict)

        #Publish the new reference
        pos_msg = PointStamped()

        pos_msg.point.x = self.x_pos_target
        pos_msg.point.y = self.y_pos_target
        pos_msg.point.z = self.z_pos_target

        self.reference_pub.publish(pos_msg)

    def main_loop(self):
        print("SwarmPlanner node running")
        rospy.spin()


if __name__ == "__main__":
    planner = swarm_planner()
    planner.main_loop()