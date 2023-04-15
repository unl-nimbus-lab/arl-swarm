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

	//The lines below were commneted out by Grant April 2023
	// auto TA_matrix = j["TA"] ;
	// auto TB_matrix = j["TB"] ;
	// auto TG_matrix = j["TG"] ;// This is the K matrix although I name the varibale TG
	// auto T_matrix = j["T"];
	// auto Tgt_vector = j["TGT"];
	// auto Aug_vector = j["AUG"];

	auto T_json = j["T"];
	auto TA_json = j["TA"];
	auto TB_json = j["TB"];
	auto ZD_json = j["ZD"];
	auto KL_json = j["KL"];
	auto Ka_json = j["Ka"];
	auto DKL_json = j["DKL"];
	auto DTA_json = j["DTA"];
	auto DTB_json = j["DTB"];
	auto aug_state_json = j["aug_state"];

	// TODO Make sure the B matrix is a squre on or not. If not square, change TB_size
	// int TA_size =  TA_matrix.size() ;
	// int TB_rows =  TB_matrix.size() ;
	// int TB_cols =  TB_matrix[0].size() ;
	// int TG_rows = TG_matrix.size();
	// int TG_cols = TG_matrix[0].size();
	// int T_rows = T_matrix.size();
	// int T_cols = T_matrix[0].size();
	// int Tgt_rows = Tgt_vector.size();
	// int Aug_ros = Aug_vector.size();

	int T_rows = T_json.size();
	int T_cols = T_json[0].size();
	int TA_rows = TA_json.size();
	int TA_cols = TA_json[0].size();
	int TB_rows = TB_json.size();
	int TB_cols = TB_json[0].size();
	int ZD_rows = ZD_json.size();
	int ZD_cols = ZD_json[0].size();
	int KL_rows = KL_json.size();
	int KL_cols = KL_json[0].size();
	int Ka_rows = Ka_json.size();
	int Ka_cols = Ka_json[0].size();
	int DKL_rows = DKL_json.size();
	int DKL_cols = DKL_json[0].size();
	int DTA_rows = DTA_json.size();
	int DTA_cols = DTA_json[0].size();
	int DTB_rows = DTB_json.size();
	int DTB_cols = DTB_json[0].size();
	int aug_state_rows = aug_state_json.size();
	int aug_state_cols = aug_state_json[0].size();

	MatrixXd T(T_rows,T_cols);
	MatrixXd TA(TA_rows,TA_cols);
	MatrixXd TB(TB_rows,TB_cols);
	MatrixXd ZD(ZD_rows,ZD_cols);
	MatrixXd KL(KL_rows,KL_cols);
	MatrixXd Ka(Ka_rows,Ka_cols);
	MatrixXd DKL(DKL_rows,DKL_cols);
	MatrixXd DTA(DTA_rows,DTA_cols);
	MatrixXd DTB(DTB_rows,DTB_cols);
	MatrixXd aug_state(aug_state_rows,aug_state_cols);


	// Assign each value for A matrix to Eigen matrix
	// for (auto& el : TA_matrix.items())
	// {
	// 	int i = 0 ;
	// 	for (auto& subel : el.value()){
	// 		A_matrix(stoi(el.key()),i) = subel ;
	// 		i++ ;
	// 	}
	// }
	// std::cout << "A Matrix" << std::endl;
	// std::cout << "======" << std::endl;
	// std::cout << A_matrix << std::endl;


	// // Assign each value for B matrix to Eigen matrix
	// for (auto& el : TB_matrix.items())
	// {
	// 		int i = 0 ;
	// 		for (auto& subel : el.value()){
	// 				B_matrix(stoi(el.key()),i) = subel ;
	// 				i++ ;
	// 		}
	// }
	// std::cout << "B Matrix" << std::endl;
	// std::cout << "======" << std::endl;
	// std::cout << B_matrix << std::endl;


	// // assign the K matrix from the json file
	// for (auto& el : TG_matrix.items())
	// {
	// 		int i = 0 ;
	// 		for (auto& subel : el.value()){
	// 				K_matrix(stoi(el.key()),i) = subel ;
	// 				i++ ;
	// 		}
	// }
	// std::cout << "K Matrix" << std::endl;
	// std::cout << "======" << std::endl;
	// std::cout << K_matrix << std::endl;


	// assign the T matrix from the json file
	for (auto& el : T_json.items())
	{
			int i = 0 ;
			for (auto& subel : el.value()){
					T(stoi(el.key()),i) = subel ;
					i++ ;
			}
	}
	std::cout << "T Matrix" << std::endl;
	std::cout << "======" << std::endl;
	std::cout << T << std::endl;


	// // assign the target vectorfrom the json file
	// int k = 0 ;
	// for (auto& el : Tgt_vector.items())
    //     {
	// 	Tgt_vec(k) = el.value();
	// 	k++ ;
	// }

	// // assign the target vectorfrom the json file
	// k = 0 ;
	// for (auto& el : Aug_vector.items())
	// {
	// 		Aug_vec(k) = el.value();
	// 		k++ ;
	// }

	struct Sys s = {T,TA,TB,ZD,KL,Ka,DKL,DTA,DTB,aug_state} ;

	return s ;
}


Sys get_formation(std::string path_to_json){


        struct Sys s = {} ;

        return s ;

}
