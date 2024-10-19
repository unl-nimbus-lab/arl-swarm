#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "mavros_msgs/State.h"

//Function to generate the current timepstamp
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
    std::tm* timeInfo = std::localtime(&currentTime);
    
    std::ostringstream oss;
    oss << std::put_time(timeInfo, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

//Function to find the next "x" for the log directory
int getNextLogDirectoryIndex(const std::string& basePath) {
    int maxIndex = 0;
    
    // Iterate over existing directories
    for (const auto& entry : std::filesystem::directory_iterator(basePath)) {
        if (entry.is_directory()) {
            std::string dirName = entry.path().filename().string();
            
            // Check if directory name matches "log_X_{timestamp}" pattern
            if (dirName.find("log_") == 0) {
                size_t underscorePos = dirName.find('_', 4);
                if (underscorePos != std::string::npos) {
                    std::string indexStr = dirName.substr(4, underscorePos - 4);
                    try {
                        int index = std::stoi(indexStr);
                        maxIndex = std::max(maxIndex, index);
                    } catch (std::invalid_argument&) {
                        // Ignore invalid directory names
                    }
                }
            }
        }
    }
    
    return maxIndex + 1; // Next available index
}

// Function to create the log directory
std::filesystem::path createLogDirectory(const std::string& basePath) {
    int nextIndex = getNextLogDirectoryIndex(basePath);
    std::string timestamp = getCurrentTimestamp();
    
    std::ostringstream oss;
    oss << "log_" << nextIndex << "_" << timestamp;
    std::filesystem::path logDirPath = std::filesystem::path(basePath) / oss.str();
    
    // Create the directory
    if (std::filesystem::create_directory(logDirPath)) {
        std::cout << "Created log directory: " << logDirPath << std::endl;
    } else {
        std::cerr << "Failed to create log directory: " << logDirPath << std::endl;
    }
    
    return logDirPath;
}

// Function to return the unix time
double getUnixTime() {
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    
    // Convert the time point to Unix time (seconds since epoch)
    std::chrono::duration<double> unix_time = now.time_since_epoch();
    
    // Convert the duration to a floating-point number
    double unix_time_float = unix_time.count();
    
    return unix_time_float;
}

class Logger {
    public:
        // Constructor (optional directory initialization)
        Logger(const std::string& logDir = "") : logDirectory(logDir) {
            // You can initialize the directory here if provided
            if (!logDirectory.empty()) {
                ensureDirectoryExists(logDirectory);
            }
        }

        // Function to set the log directory after the object is created
        void setLogDirectory(const std::filesystem::path& newLogDir) {
            // Close the current log file if it's open
            if (logFile.is_open()) {
                logFile.close();
                std::cout << "Previous log file closed." << std::endl;
            }

            // Set the new directory
            logDirectory = newLogDir;
            ensureDirectoryExists(logDirectory);
            std::cout << "Log directory set to: " << logDirectory << std::endl;
        }

        // Function to open a log file later
        void openLogFile(const std::string& fileName) {
            if (logDirectory.empty()) {
                std::cerr << "Log directory not set!" << std::endl;
                return;
            }

            std::filesystem::path filePath = std::filesystem::path(logDirectory) / fileName;
            logFile.open(filePath);
            
            if (logFile.is_open()) {
                std::cout << "Log file opened: " << filePath << std::endl;
            } else {
                std::cerr << "Failed to open log file: " << filePath << std::endl;
            }
        }

        // Function to write to the log file
        void writeLogFloat(const float& message) {
            if (logFile.is_open()) {
                logFile << message << std::endl;
            } else {
                std::cerr << "Log file is not open!" << std::endl;
            }
        }

        void writeLogString(const std::string& message) {
            if (logFile.is_open()) {
                logFile << message << std::endl;
            } else {
                std::cerr << "Log file is not open!" << std::endl;
            }
        }

        // Destructor to close the log file
        ~Logger() {
            if (logFile.is_open()) {
                logFile.close();
                std::cout << "Log file closed." << std::endl;
            }
        }

    private:
        std::string logDirectory;
        std::ofstream logFile; // std::ofstream declared as member variable

        // Helper function to ensure the log directory exists
        void ensureDirectoryExists(const std::string& directory) {
            if (!std::filesystem::exists(directory)) {
                std::filesystem::create_directory(directory);
                std::cout << "Created directory: " << directory << std::endl;
            }
        }
};

class verifierLogger {
    
    public:
        verifierLogger(ros::NodeHandle nh);

        ros::Subscriber rs_time_sub;
        ros::Subscriber en_time_sub;
        ros::Subscriber br_time_sub;
        ros::Subscriber dc_time_sub;
        ros::Subscriber in_time_sub;
        ros::Subscriber my_rs_sub;
        ros::Subscriber rc_rs_sub;

        //Setup the loggers //
        Logger rs_logger;
        Logger en_logger;
        Logger br_logger;
        Logger dc_logger;
        Logger in_logger;
        Logger my_logger;
        Logger rr_logger;

        void rs_cb(const std_msgs::Float32::ConstPtr& msg) {
            rs_logger.writeLogFloat(msg->data);
        }
        void en_cb(const std_msgs::Float32::ConstPtr& msg) {
            en_logger.writeLogFloat(msg->data);
        }
        void br_cb(const std_msgs::Float32::ConstPtr& msg) {
            br_logger.writeLogFloat(msg->data);
        }
        void dc_cb(const std_msgs::Float32::ConstPtr& msg) {
            dc_logger.writeLogFloat(msg->data);
        }
        void in_cb(const std_msgs::Float32::ConstPtr& msg) {
            in_logger.writeLogFloat(msg->data);
        }
        void my_rs_cb(const std_msgs::String::ConstPtr& msg) {
            my_logger.writeLogString(msg->data);
        }
        void rc_rs_cb(const std_msgs::String::ConstPtr& msg) {
            rr_logger.writeLogString(msg->data);
        }


    private:
        std::string baseLogPath = "/home/verifier_logs";

};

#endif