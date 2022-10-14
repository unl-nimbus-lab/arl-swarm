#include "gtest/gtest.h"
#include <ros/ros.h>
#include "../include/clustering.h"


TEST(ClusteringControlTest, AltitudeSepModeTestNoGPS) { 
    // setup
    agent a;
    int originalFlightMode = CLUSTER_GUIDED_LOCAL_MODE;
    int lat = 0, lon = 0, alt = 0;
    a.setFlightMode(originalFlightMode);
    a.setCurrentPos(lat, lon, alt);
    // execute
    a.setFlightMode(ALTITUDE_SEP_MODE);
    // test that mode rejected due to no gps
    ASSERT_EQ(a.getFlightMode(), originalFlightMode); 
}

TEST(ClusteringControlTest, AltitudeSepModeTestWithGPS) { 
    // setup
    agent a;
    int originalFlightMode = CLUSTER_GUIDED_LOCAL_MODE;
    int switchToMode = ALTITUDE_SEP_MODE;
    int lat = 1, lon = 20, alt = 15;
    a.setFlightMode(originalFlightMode);
    a.setCurrentPos(lat, lon, alt);
    // execute
    a.setFlightMode(switchToMode);
    // test that mode works since position is set (gps working)
    ASSERT_EQ(a.getFlightMode(), switchToMode); 
}

TEST(ClusteringControlTest, IgnoreInvalidFlightMode10) { 
    // setup
    agent a;
    int originalFlightMode = CLUSTER_GUIDED_LOCAL_MODE;
    int invalidFlightMode = 10;
    a.setFlightMode(originalFlightMode);
    // execute
    a.setFlightMode(invalidFlightMode);
    // test 
    ASSERT_EQ(a.getFlightMode(), originalFlightMode); 
}

TEST(ClusteringControlTest, IgnoreInvalidFlightMode0) { 
    // setup
    agent a;
    int originalFlightMode = CLUSTER_GUIDED_LOCAL_MODE;
    int invalidFlightMode = 0;
    a.setFlightMode(originalFlightMode);
    // execute
    a.setFlightMode(invalidFlightMode);
    // test
    ASSERT_EQ(a.getFlightMode(), originalFlightMode); 
}

TEST(ClusteringControlTest, ClusterGuidedLocalModeTest) { 
    // setup
    agent a;
    int originalFlightMode = CLUSTER_RALLY_MODE;
    int switchToMode = CLUSTER_GUIDED_LOCAL_MODE;
    a.setFlightMode(originalFlightMode);
    // execute
    a.setFlightMode(switchToMode);
    // test 
    ASSERT_EQ(a.getFlightMode(), switchToMode); 
}

TEST(ClusteringControlTest, AgentStartsFTLDisabled) { 
    // setup
    agent a;
    bool expectedFTL = false;
    // test 
    ASSERT_EQ(a.FTL_ENABLE, expectedFTL); 
}

TEST(ClusteringControlTest, ClusterGuidedLocalModeTestCheckFTLDisabled) { 
    // setup
    agent a;
    int originalFlightMode = CLUSTER_RALLY_MODE;
    int switchToMode = CLUSTER_GUIDED_LOCAL_MODE;
    a.setFlightMode(originalFlightMode);
    bool expectedFTL = false;
    // execute
    a.setFlightMode(switchToMode);
    // test
    ASSERT_EQ(a.FTL_ENABLE, expectedFTL); 
}

TEST(ClusteringControlTest, ClusterRallyModeTestCheckFTLDisabled) { 
    // setup
    agent a;
    int originalFlightMode = FOLLOW_THE_LEADER_MODE;
    int switchToMode = CLUSTER_RALLY_MODE;
    a.setFlightMode(originalFlightMode);
    bool expectedFTL = false;
    // execute
    a.setFlightMode(switchToMode);
    // test 
    ASSERT_EQ(a.FTL_ENABLE, expectedFTL); 
}

TEST(ClusteringControlTest, FollowLeaderTestCheckFTLEnabled) { 
    // setup
    agent a;
    int originalFlightMode = CLUSTER_RALLY_MODE;
    int switchToMode = FOLLOW_THE_LEADER_MODE;
    a.setFlightMode(originalFlightMode);
    bool expectedFTL = true;
    // execute
    a.setFlightMode(switchToMode);
    // test
    ASSERT_EQ(a.FTL_ENABLE, expectedFTL); 
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}