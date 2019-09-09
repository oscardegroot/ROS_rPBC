

#ifndef Helpers_H
#define Helpers_H

#define MAX_HANDSHAKE_ATTEMPTS 100000

#include <string>
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include <franka/exception.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64MultiArray.h"

#include "CustomLog.h"
#include "Exceptions.h"

namespace helpers {

Eigen::VectorXd vectorToEigen(const std::vector<double> values);
double normOf(const Eigen::VectorXd input);
std_msgs::Float64MultiArray eigenToMessage(Eigen::VectorXd vec);
Eigen::VectorXd messageToEigen(std_msgs::Float64MultiArray msg, int l);


template <class T> 
bool safelyRetrieve(ros::NodeHandle& nh, const std::string name, T& param){


	if (!nh.getParam(name, param)) {
		throw RetrievalException("Helpers: Failed to retrieve parameter " + name);
		return false;
	}

	return true;

}

template <class T> 
bool safelyRetrieve(ros::NodeHandle& nh, const std::string name,
					 T& param, const T default_value){

    try{
        safelyRetrieve(nh, name, param);
    }catch(RetrievalException){
		param = default_value;
		logMsg("Helpers", "Set " + name + " to default value: " + std::to_string(default_value), 1);
	}

	return true;
}

/* Turn this into vector! */
template <class T>
bool safelyRetrieveArray(ros::NodeHandle& nh, const std::string name,
						 T& param, const int expected_size){

	safelyRetrieve(nh, name, param);

	if(param.size() != expected_size){
		throw RetrievalException("Helpers: Size of parameter " + name +
	 	" incorrect (size=" + std::to_string(param.size()) +
	 	", expected=" + std::to_string(expected_size) + ")");

		return false;
	}

	return true;
		
}
//
///* Turn this into vector! */
//template <class T>
//bool safelyRetrieveArray(ros::NodeHandle& nh, const std::string name,
//                         T& param){
//
//    if (!nh.getParam(name, param))
//    {
//        throw RetrievalException("Helpers: Failed to retrieve array " + name);
//    }
//
//    return true;
//}

/* Turn this into vector! */
template <class T>
bool safelyRetrieveEigen(ros::NodeHandle& nh, const std::string name,
						 T& param, const int expected_size){

	std::vector<double> result_v;
	safelyRetrieveArray(nh, name, result_v, expected_size);

	param = vectorToEigen(result_v);

	return true;
		
}

template <class T>
bool safelyRetrieveEigen(ros::NodeHandle& nh, const std::string name,
                         T& param){

    std::vector<double> result_v;
    safelyRetrieve(nh, name, result_v);

    param = vectorToEigen(result_v);

    return true;

}

inline void errorRetrieving(std::string name, const char* ex_what){
	logMsg("Helpers", "Exception getting " + name + ex_what, 0);
}

template <int N>
Eigen::VectorXd arrayToVector(std::array<double, N> input){

	Eigen::VectorXd result(N);

	for(int i = 0; i < N; i++){
		result(i) = input[i];
	}

	return result;
}
}

#endif