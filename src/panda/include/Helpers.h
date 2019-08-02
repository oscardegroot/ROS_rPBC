

#ifndef Helpers_H
#define Helpers_H
#include <string>
#include <vector>
#include <cmath>
#include "ros/ros.h"
#include <franka/exception.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "CustomLog.h"
#include "Exceptions.h"

namespace helpers {

Eigen::VectorXd vectorToEigen(const std::vector<double> values);
double normOf(const Eigen::VectorXd input);

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

	if(!safelyRetrieve(nh, name, param)){
		param = default_value;
		logMsg("Helpers", "Set " + name + " to default value: " + std::to_string(default_value), 0);
	}

	return true;
}

/* Turn this into vector! */
template <class T>
bool safelyRetrieveArray(ros::NodeHandle& nh, const std::string name,
						 T& param, const int expected_size){

	if (!nh.getParam(name, param))
	{
		throw RetrievalException("Helpers: Failed to retrieve array " + name);
		return false;
	}
	if(param.size() != expected_size){
		throw RetrievalException("Helpers: Size of parameter " + name +
	 	" incorrect (size=" + std::to_string(param.size()) +
	 	", expected=" + std::to_string(expected_size) + ")");

		return false;
	}

	return true;
		
}

/* Turn this into vector! */
template <class T>
bool safelyRetrieveEigen(ros::NodeHandle& nh, const std::string name,
						 T& param, const int expected_size){

	std::vector<double> result_v;
	safelyRetrieveArray(nh, name, result_v, expected_size);

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