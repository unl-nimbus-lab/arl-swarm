
/* This implements the functions of the json_reader for the swarm project.
*/

#include "custom_libs/json_reader.hpp"

Sys json_reader::readJSON(std::string path_to_json)
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
//	auto K_matrix = j["K"] ;

	// TODO Make sure the B matrix is a squre on or not. If not square, change TB_size
	int TA_size =  TA_matrix.size() ;
	int TB_size =  TB_matrix.size() ;
//	int K_size = K_matrix.size();


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

	// TODO : assign the K matrix from the json file

	struct Sys s = {A_matrix, B_matrix} ;

	return s ;
}


Sys get_formation(std::string path_to_json){


        struct Sys s = {} ;

        return s ;

}
