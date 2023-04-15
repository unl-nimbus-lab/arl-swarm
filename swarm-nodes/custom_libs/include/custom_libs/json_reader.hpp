// This is the header file for the coordinate conversion to cpp

#ifndef JSON_READER_H
#define JSON_READER_H


#include <math.h>
#include <thread>
#include <string>
#include <iostream>
#include <math.h>
#include "json.hpp"
#include <fstream>
#include <eigen3/Eigen/Dense>

using json = nlohmann::json;
using namespace Eigen;


struct Sys {
        MatrixXd A ;
        MatrixXd B ;
        MatrixXd K ;
	MatrixXd Tr ; // Transformation matrix
	MatrixXd Tgt; // Target vector
	MatrixXd Aug ; // Augmentation vector to the integral state
//      MatrixXd Formation;
} ;

class json_reader{

	public:

		Sys readJSON(std::string path_to_json); // This retrns the full info on the json file 
		Sys get_formation(std::string path_to_json); // This returns the agent groupings
		//double gps_to_meters(double lat1, double lon1, double lat2, double lon2) ; // This gives the distance beteen two gps coordiantes in meters
		//double meters_to_lat(double base_lat, double base_lon, double d ); // This gives the latitde from a base lat,lon coordinnates and a particualr distance with a bearing of 0 degrees

};



#endif
