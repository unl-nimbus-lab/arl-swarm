/* This implements the functions of the json_reader for the swarm project.
*/

#include "custom_libs/json_reader.hpp"

Sys json_reader::readJSON(std::string path_to_json)
{
	// This function  reads the .json file located through the path_to_json and outputs structure with the following elements :
	//	1. A sys martix
	//	2. B sys matrix 


	std::ifstream i(path_to_json);
	json j;
	i >> j;

	auto TA_matrix = j["TA"] ;
	auto TB_matrix = j["TB"] ;
	auto TG_matrix = j["TG"] ;// This is the K matrix although I name the varibale TG
	auto T_matrix = j["T"];
	auto Tgt_vector = j["TGT"];
	auto Aug_vector = j["AUG"];

	// TODO Make sure the B matrix is a squre on or not. If not square, change TB_size
	int TA_size =  TA_matrix.size() ;
	int TB_rows =  TB_matrix.size() ;
	int TB_cols =  TB_matrix[0].size() ;
	int TG_rows = TG_matrix.size();
	int TG_cols = TG_matrix[0].size();
	int T_rows = T_matrix.size();
	int T_cols = T_matrix[0].size();
	int Tgt_rows = Tgt_vector.size();
	int Aug_ros = Aug_vector.size();


	MatrixXd A_matrix(TA_size,TA_size);
	MatrixXd B_matrix(TB_rows,TB_cols);
	MatrixXd K_matrix(TG_rows,TG_cols);
	MatrixXd T(T_rows,T_cols);
	MatrixXd Tgt_vec(Tgt_rows,1);
	MatrixXd Aug_vec(Aug_ros,1);

	// Assign each value for A matrix to Eigen matrix
	for (auto& el : TA_matrix.items())
	{
		int i = 0 ;
		for (auto& subel : el.value()){
			A_matrix(stoi(el.key()),i) = subel ;
			i++ ;
		}
	}
//	std::cout << A_matrix ;


        // Assign each value for B matrix to Eigen matrix
        for (auto& el : TB_matrix.items())
        {
                int i = 0 ;
                for (auto& subel : el.value()){
                        B_matrix(stoi(el.key()),i) = subel ;
                        i++ ;
                }
        }


	// assign the K matrix from the json file
        for (auto& el : TG_matrix.items())
        {
                int i = 0 ;
                for (auto& subel : el.value()){
                        K_matrix(stoi(el.key()),i) = subel ;
                        i++ ;
                }
        }


        // assign the T matrix from the json file
        for (auto& el : T_matrix.items())
        {
                int i = 0 ;
                for (auto& subel : el.value()){
                        T(stoi(el.key()),i) = subel ;
                        i++ ;
                }
        }


        // assign the target vectorfrom the json file
        int k = 0 ;
	for (auto& el : Tgt_vector.items())
        {
		Tgt_vec(k) = el.value();
		k++ ;
	}

        // assign the target vectorfrom the json file
        k = 0 ;
        for (auto& el : Aug_vector.items())
        {
                Aug_vec(k) = el.value();
                k++ ;
        }

	struct Sys s = {A_matrix, B_matrix, K_matrix, T, Tgt_vec, Aug_vec } ;

	return s ;
}


Sys get_formation(std::string path_to_json){


        struct Sys s = {} ;

        return s ;

}
