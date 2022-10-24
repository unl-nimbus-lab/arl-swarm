#include "gtest/gtest.h"
#include <ros/ros.h>
#include "../include/clustering.h"


TEST(FlightModeTest, AltitudeSepModeTestNoGPS) { 
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

TEST(FlightModeTest, AltitudeSepModeTestWithGPS) { 
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

TEST(FlightModeTest, IgnoreInvalidFlightMode10) { 
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

TEST(FlightModeTest, IgnoreInvalidFlightMode0) { 
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

TEST(FlightModeTest, ClusterGuidedLocalModeTest) { 
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

TEST(FlightModeTest, AgentStartsFTLDisabled) { 
    // setup
    agent a;
    bool expectedFTL = false;
    // test 
    ASSERT_EQ(a.FTL_ENABLE, expectedFTL); 
}

TEST(FlightModeTest, ClusterGuidedLocalModeTestCheckFTLDisabled) { 
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

TEST(FlightModeTest, ClusterRallyModeTestCheckFTLDisabled) { 
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

TEST(FlightModeTest, FollowLeaderTestCheckFTLEnabled) { 
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

// Validate cluster offsets get set
TEST(OffsetTest, SetClusterOffsetsCircleFormation) { 
    // setup
    agent a;
    // cluster size, cluster order, 
    double clusterSize = 5; // number of drones in the cluster
    double clusterOrder = 2;// agent's spot in the cluster
    double radius = 10;
    a.setClusterRadius(radius);
    a.setClusterSize(clusterSize);
    a.setClusterOrder(clusterOrder);

    double expectedXOffset = a.circleXOffset();
    double expectedYOffset = a.circleYOffset();

    // execute
    a.setClusterOffsets();

    double resultX = a.getClusterXOffset();
    double resultY = a.getClusterYOffset();

    // test
    ASSERT_EQ(resultX,  expectedXOffset); 
    ASSERT_EQ(resultY,  expectedYOffset); 
}

TEST(OffsetTest, SetClusterOffsetsLinearFormation) { 
    // setup
    agent a;
    a.setClusterType(LINEAR_FORMATION);
    // cluster size, cluster order, 
    double clusterSize = 5; // number of drones in the cluster
    double clusterOrder = 2;// agent's spot in the cluster
    double radius = 10;
    a.setClusterRadius(radius);
    a.setClusterSize(clusterSize);
    a.setClusterOrder(clusterOrder);

    double expectedXOffset = a.calculateXLineOffset();
    double expectedYOffset = a.calculateYLineOffset();

    // execute
    a.setClusterOffsets();

    double resultX = a.getClusterXOffset();
    double resultY = a.getClusterYOffset();

    // test
    ASSERT_EQ(resultX,  expectedXOffset); 
    ASSERT_EQ(resultY,  expectedYOffset); 
}

TEST(RallyTest, SettingRally1) {
    agent a;
    double x = 10;
    double y = 10;
    a.setRally1(x, y);

    // test
    ASSERT_EQ(a.getRally1X(), x);
    ASSERT_EQ(a.getRally1Y(), y);
}

TEST(RallyTest, SettingRally1AgainAltitudeUpdates) {
    // setup
    agent a;
    double x = 10;
    double y = 10;
    double alt = 15;

    // execute
    a.setRally1(x, y);
    a.setAgentAlt(alt);
    a.setRally1(x,y);

    // test
    ASSERT_EQ(a.getRally1Z(), alt);
}

TEST(AltitudeLimits, SetAllowableAltitude) {
    // setup
    agent a;
    double alt = 100.32;

    // execute
    a.setAgentAlt(alt);

    // test
    ASSERT_EQ(a.getAgentAlt(), alt);
}

TEST(AltitudeLimits, SetNotAllowableAltitude) {
    // setup
    agent a;
    double alt = 122.32;

    // execute
    a.setAgentAlt(alt);

    // test: ensure that the altitude is set to 
    // max_alt when input is above max_alt
    ASSERT_EQ(a.getAgentAlt(), MAX_ALT);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}