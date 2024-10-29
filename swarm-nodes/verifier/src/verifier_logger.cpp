#include "logger.h"
#include "ros/ros.h"

verifierLogger::verifierLogger(ros::NodeHandle nh) {

    rs_time_sub = nh.subscribe("/rs_time", 1, &verifierLogger::rs_cb, this);
    en_time_sub = nh.subscribe("/en_time", 1, &verifierLogger::en_cb, this);
    br_time_sub = nh.subscribe("/br_time", 1, &verifierLogger::br_cb, this);
    dc_time_sub = nh.subscribe("/dc_time", 1, &verifierLogger::dc_cb, this);
    in_time_sub = nh.subscribe("/in_time", 1, &verifierLogger::in_cb, this);
    my_rs_sub = nh.subscribe("/my_rs"    , 1, &verifierLogger::my_rs_cb, this);
    rc_rs_sub = nh.subscribe("/rc_rs"    , 1, &verifierLogger::rc_rs_cb, this);
    ttc_sub   = nh.subscribe("/ttc"      , 1, &verifierLogger::ttc_cb, this);

    //state_sub = nh.subscribe("/mavros/state", 1, &verifierLogger::state_cb, this);

    if (!std::filesystem::exists(baseLogPath)) {
        std::filesystem::create_directory(baseLogPath);
    }

    //Create a new log directory
    std::filesystem::path logDir = createLogDirectory(baseLogPath);

    //Attach the loggers to the directory
    rs_logger.setLogDirectory(logDir);
    en_logger.setLogDirectory(logDir);
    br_logger.setLogDirectory(logDir);
    dc_logger.setLogDirectory(logDir);
    in_logger.setLogDirectory(logDir);
    my_logger.setLogDirectory(logDir);
    rr_logger.setLogDirectory(logDir);
    ttc_logger.setLogDirectory(logDir);

    //Open the log files
    rs_logger.openLogFile("rs_time.txt");
    en_logger.openLogFile("en_time.txt");
    br_logger.openLogFile("br_time.txt");
    dc_logger.openLogFile("dc_time.txt");
    in_logger.openLogFile("in_time.txt");
    my_logger.openLogFile("my_spaces.txt");
    rr_logger.openLogFile("rc_spaces.txt");
    ttc_logger.openLogFile("ttc_time.txt");

    //write current time to br_time incase there is any offset
    double start_time = getUnixTime();
    br_logger.writeLogFloat(getUnixTime());

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "verifier_logger");
    ros::NodeHandle noodle;
    verifierLogger verifier_logger(noodle);
    ros::spin();

}

// void state_cb(const mavros_msgs::State::ConstPtr& msg) {

// }