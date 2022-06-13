// This is the header file for the coordinate conversion to cpp

#ifndef GEO_CONVERTER_H
#define GEO_CONVERTER_H

#define _USE_MATH_DEFINES
#include <math.h>

#define EARTH_R 6378137 // This is the radius of the earth in meters
#define PI 3.14159265358979323846

//using namespace std;

class geo_converter{

	public:
		double gps_to_meters(double lat1, double lon1, double lat2, double lon2) ;
		double meters_to_lat(double base_lat, double base_lon, double d );
		double meters_to_lat(double base_lat, double base_lon, double d );


}





#endif
