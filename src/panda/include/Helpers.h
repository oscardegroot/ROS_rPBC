

#ifndef Helpers_H
#define Helpers_H

//#define MAX_HANDSHAKE_ATTEMPTS 10000

#include <string>
#include <vector>
#include <cmath>
#include <chrono>
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


class Counter{

private:
    unsigned int count;
    unsigned int rate;
public:
    Counter(){
        rate = 0;
        count = 0;
    }
    
    Counter(unsigned int rate_set){
        rate = rate_set;
        count = 0;
    }
    
    bool trigger(){
        
        count = (count + 1)%rate;
        
        return count == 0;
    }
    
};

/** @brief Simple timer class 
 * Times a duration*/
class SimpleTimer{
  
public:
    SimpleTimer(double duration_s)
    : timer_duration(duration_s)
    {        
        start_time = std::chrono::system_clock::now();
    }
    
    bool finished(){
        auto end_time = std::chrono::system_clock::now();
        std::chrono::duration<double> current_duration = end_time - start_time;
        return (current_duration.count() >= timer_duration);
    }
    
private:
    std::chrono::system_clock::time_point start_time;
    double timer_duration;
  
};


template <class T> 
bool safelyRetrieve(ros::NodeHandle& nh, const std::string name, T& param){


	if (!nh.getParam(name, param)) {
		throw RetrievalException("Helpers: Failed to retrieve parameter " + nh.getNamespace() + "/" + name);
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
		logMsg("Helpers", "Set " + nh.getNamespace() + "/"  + name + " to default value: " + std::to_string(default_value), 1);
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

inline bool hasParameter(ros::NodeHandle& nh, const std::string& name){

    return nh.hasParam(name);
}

// Sign Function
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
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

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoInverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}


/** @brief Attempt a boolean function for max_duration seconds */
inline void repeatedAttempts(std::function<bool()> fcnPtr, double max_duration, std::string error_message){
    
    auto start_time = std::chrono::system_clock::now();
    auto end_time = start_time;
    std::chrono::duration<double> loop_duration;
    
    while(!fcnPtr()){
        end_time = std::chrono::system_clock::now();
        loop_duration = (end_time - start_time);

        if(loop_duration.count() >= max_duration){
            throw OperationalException(error_message);
        }
        
        
    }    
}


/** @brief apply lowpass filtering on some vector, matrix or scalar */
template <class T>
void lowpassFilter(T& filtered_input, const T& input, double alpha)

    try{
        filtered_input = (1.0 - alpha) * filtered_input + alpha * input;
    }catch(const std::exception& ex){
        logMsg("Helpers", "Filtering failed! Error: " + std::string(ex.what()));
    }

}

#endif