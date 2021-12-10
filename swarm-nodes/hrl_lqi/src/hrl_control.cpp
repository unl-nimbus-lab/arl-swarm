
/* This implements the HRL-LQI controller for the swarm project.
*/

//#include "lqr_control.h"


#include <ros/ros.h>
#include <thread>
#include <string>
#include <mutex>



/*
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <freyja_msgs/CurrentState.h>
#include <freyja_msgs/CtrlCommand.h>
#include <freyja_msgs/ControllerDebug.h>
#include <freyja_msgs/ReferenceState.h>
#include <freyja_msgs/WaypointTarget.h>
#include <freyja_msgs/StateRecorder.h>
#include <eigen3/Eigen/Dense>
#include <time.h>
#include <math.h>
#include <std_msgs/UInt8.h>
#include "std_msgs/String.h"
#include <string>
#include <chrono>
*/

#include "std_msgs/String.h"
#include <math.h>
#include "json.hpp"
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <custom_libs/geo_converter.hpp>

#define ROS_NODE_NAME "hrl_lqi"
#define CONTROLLER_RATE 4    // Rate in Hz 

#define pi 3.1416
#define EARTH_R 6378137 // This is the radius of the earth in meters

using json = nlohmann::json;
using namespace Eigen;


struct Sys {
	MatrixXd A ;
	MatrixXd B ;
//	MatrixXd T ;
//	MatrixXd K ;
} ;

MatrixXd state ; // stores all the states
std::mutex state_mutex ; // mutex for state variable

Matrix<double, 12,1> init_states {7.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} ;

Sys readJSON(std::string path_to_json)
{
	// This function  reads the .json file located through the path_to_json and outputs structure with the following elements :
	//	1. A sys martix
	//	2. B sys matrix 


	std::cout << "INSIDE HRL" ;

	std::ifstream i(path_to_json);
	json j;
	i >> j;

	auto TA_matrix = j["TA"] ;
	auto TB_matrix = j["TB"] ;


	int TA_size =  TA_matrix.size() ;
	int TB_size =  TB_matrix.size() ;

	MatrixXd A_matrix(TA_size,TA_size);
	MatrixXd B_matrix(TB_size,TB_size);

	// Assign each value for A matrix to Eigen matrix
	for (auto& el : TA_matrix.items())
	{
		int i = 0 ;
		for (auto& subel : el.value()){
			A_matrix(stoi(el.key()),i) = subel ;
			i++ ;
		}
	}
	std::cout << A_matrix ;


        // Assign each value for B matrix to Eigen matrix
        for (auto& el : TB_matrix.items())
        {
                int i = 0 ;
                for (auto& subel : el.value()){
                        B_matrix(stoi(el.key()),i) = subel ;
                        i++ ;
                }
        }

        std::cout << B_matrix  ;

	struct Sys s = {A_matrix, B_matrix} ;

	return s ;
}


void estimatorCallback(std_msgs::String newMsg){

	const std::lock_guard<std::mutex> lock(state_mutex);
	// Copy the value from the estimator to the state variable



}


void HRLController(  )
{

	while (ros::ok()) {

		ros::Rate loop_rate(CONTROLLER_RATE);

		//Access state here

		//Calculate U here

		// Do Ax + Bu to get the next waypoint.

		// Publish to the reference topic


		ros::spinOnce();
        	loop_rate.sleep();
	}


}

int main( int argc, char** argv )
{
	ros::init(argc, argv, ROS_NODE_NAME); //Initialize the node


	ros::NodeHandle hl;
	ros::Subscriber estimator_callback = hl.subscribe("inbound", 1, estimatorCallback);


	// Path to the json file
        std::string path_to_json = "state-matrix.json";

	//Read the json file
	Sys ss = readJSON(path_to_json);

	// Create seperate threads for processes with specific Hz requirements
	std::thread hrl_controller (HRLController); //Sperate thread for the controller


	// join the threads at the end
	hrl_controller.join() ;

	return 0;
}
