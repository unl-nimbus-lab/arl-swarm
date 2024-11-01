#!/usr/bin/env python3

from mission import missions

import rospy


from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
import re

from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse, CommandBool, CommandBoolRequest, CommandTOL, CommandTOLRequest, CommandTOLResponse
import time

#This node will store 1-3 missions that will be started and advanced using the verifier comms package
#need to redo the takeoff command using the agent alt stuff

"""
homeAlt (the ground)
commonAlt (altitude mission takes place)
agentAlt (alt sep)

"""
common_alt = rospy.get_param("commonAlt", 410)
agent_alt = rospy.get_param("agentAlt", 405)
home_alt = rospy.get_param("homeAlt", 390)

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
        self.home_lat = rospy.get_param("homeAlt", 390)

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

        #New
        self.block = True

        self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.set_mode_request = SetModeRequest()

        self.takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.takeoff_client_request = CommandTOLRequest()

        self.position_control_enable_pub = rospy.Publisher('/pos_en', Bool, queue_size=1)
        self.collision_action_pub = rospy.Publisher('/collision_action', Bool, queue_size=1)

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

        rospy.logwarn("Starting collision sequence")

        
        #Step 1: call takeoff
        rospy.loginfo("TAKEOFF SEQUENCE")
        self.handle_takeoff()

        #wait for 15? sec
        time.sleep(15)

        #Step 2: publish new reference: m1s1
        rospy.loginfo("UPDATAING REFERENCE TO M1S1")
        self.update_reference(1,1)
        
        #Setp 3: enable velocity controller
        rospy.loginfo("ENABLEING POSITION CONTROLLER")
        self.position_control_enable_pub.publish(True)

        #Step 4: publish new reference: m1s2
        rospy.loginfo("UPDATAING REFERENCE TO M1S2")
        self.update_reference(1,2)

        #wait for 10? sec
        time.sleep(20)

        #Step 5: Turn verifier on
        rospy.loginfo("ENABLING COLLISION AVOIDDANCE")
        self.collision_action_pub.publish(True)

        time.sleep(2)
        
        #Step 6: publish new reference: m1s3
        rospy.loginfo("UPDATING REFERENCE TO M1S3")

        self.update_reference(1,3)

        #wait for 30? sec
        time.sleep(30)

        #Step 7: turn verifier off
        rospy.loginfo("DISABLING COLLISION AVOIDANCE")
        self.collision_action_pub.publish(False)

        #Step 8: publish new reference: m1s2
        rospy.loginfo("UPDATING REFERNCE TO M1S2")
        self.update_reference(1,2)

        #Step 9: call guided
        self.handle_set_mode("GUIDED")

        #wait for 20? sec
        time.sleep(20)

        #Step 10: publish new reference: m1s1
        rospy.loginfo("UPDATING REFERENCE TO M1S1")   
        self.update_reference(1,1)

        #wait for 10? sec
        time.sleep(10)

        #Step 11: call RTL
        rospy.loginfo("RETURNING TO LAUNCH")
        self.handle_set_mode("RTL")

    #New
    def handle_takeoff(self):
        
        requested_alt = self.agent_alt - home_alt
        yaw = 0

        self.takeoff_client_request.altitude = requested_alt
        self.takeoff_client_request.yaw = yaw

        response = self.takeoff_client.call(self.takeoff_client_request)
        rospy.loginfo(response)

    def update_reference(self, mission, stage):
        mission_stage = {"mission": "m" + str(mission),
                        "stage": "s" + str(stage),
                        "agent": "a" + str(self.ID)}
        
        self.x_pos_target, self.y_pos_target = self.mission_lookup(mission_stage, missions)
        self.z_pos_target = self.altitude_lookup(mission_stage, missions, self.altitude_dict)

        pos_msg = PointStamped()

        pos_msg.point.x = self.x_pos_target
        pos_msg.point.y = self.y_pos_target
        pos_msg.point.z = self.z_pos_target

        self.reference_pub.publish(pos_msg)

    def handle_set_mode(self, mode):
        self.set_mode_request.custom_mode = mode
        response = self.set_mode_service.call(self.set_mode_request)
        rospy.loginfo(response)

    def main_loop(self):
        rospy.spin()
        



if __name__ == "__main__":
    planner = swarm_planner()
    planner.main_loop()