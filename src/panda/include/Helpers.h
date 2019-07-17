

#ifndef Helpers_H
#define Helpers_H
#include <string>
#include <vector>
#include "ros/ros.h"
#include <franka/exception.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "CustomLog.h"

namespace helpers {

Eigen::VectorXd vectorToEigen(const std::vector<double> values);


template <class T> 
bool safelyRetrieve(ros::NodeHandle& nh, const std::string name, T& param){


	if (!nh.getParam(name, param)) {
		logMsg("Panda", "Failed to retrieve parameter " + name, 0);
		return false;
	}

	return true;

}

template <class T> 
bool safelyRetrieve(ros::NodeHandle& nh, const std::string name,
					 T& param, const T default_value){

	if(!safelyRetrieve(nh, name, param)){
		param = default_value;
		logMsg("Panda", "Set " + name + " to default value: " + std::to_string(default_value), 0);
	}

	return true;
}

template <class T>
bool safelyRetrieveArray(ros::NodeHandle& nh, const std::string name,
						 T& param, const int expected_size){

	if (!nh.getParam(name, param))
	{
		logMsg("Panda", "Failed to retrieve array " + name, 0);
		return false;
	}
	if(param.size() != expected_size){
		logMsg("Panda", "Size of parameter " + name +
			 	" incorrect (size=" + std::to_string(param.size()) +
			 	", expected=" + std::to_string(expected_size) + ")", 0);
		return false;
	}

	return true;
		
}

inline void errorRetrieving(std::string name, const char* ex_what){
	logMsg("Panda", "Exception getting " + name + ex_what, 0);
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