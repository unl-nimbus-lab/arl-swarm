#!/usr/bin/env python3

import rospy

#This node will store 1-3 missions that will be started and advanced using the verifier comms package
#need to redo the takeoff command using the agent alt stuff
#congrats, you can still write a "node"

#Mission: (Takeoff, at alt sep) => move to stage 1, (at position 1, at alt sep) => move to common altitude


common_alt = 410 #meters

home_lat = rospy.get_param("homeLat", default = 40.846735)
home_lon = rospy.get_param("homeLon", default = -96.471)

agent_alt = rospy.get_param("agentAlt")

missions = {
    "m1" : {
        "s1" : {
            1 : {},
            2 : {},
        }, #pos 1 alt sep
        "s2" : {
            1 : {},
            2 : {},
        }, #pos 1 alt cmm
        "s3" : {
            1 : {},
            2 : {},
        }, #pos 2 alt cmm
        "s4" : {
            1 : {},
            2 : {},
        }, #pos 2 alt sep
    },
    "m2" : {},
    "m3" : {},
}

if __name__ == "__main__":
    print("FUCK YOU, YOU STUPID PIECE OF SHIT")

