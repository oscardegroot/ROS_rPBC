#ifndef Helpers_H
#define Helpers_H

#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include "ros/ros.h"
#include "termios.h"
#include <franka/exception.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64MultiArray.h"

#include "CustomLog.h"
#include "Exceptions.h"

/** @brief Class with all helper functions */
namespace helpers {

    // Convert a vector to an Eigen::Vector
    Eigen::VectorXd vectorToEigen(const std::vector<double> values);
    
    // Compute the 2 norm of a vector
    double normOf(const Eigen::VectorXd input);
    
    // Conver an Eigen::Vector to a float64MultiArray msg and vice versa
    std_msgs::Float64MultiArray eigenToMessage(Eigen::VectorXd vec);
    Eigen::VectorXd messageToEigen(std_msgs::Float64MultiArray msg, int l);

    // Simple class for counting until a trigger
    class Counter{

    private:
        int count;
        unsigned int rate;
    public:
        Counter(){
            rate = 0;
            count = 0;
        }
        
        Counter(unsigned int rate_set, bool initial_trigger = false){
            rate = rate_set;
            
            if(initial_trigger){
                count = -1;
            }else{
                count = 0;

            }
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
        
        SimpleTimer(){
            
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

    /**
     * @class Stopwatch
     * @author Oscar
     * @date 29/10/19
     * @file Helpers.h
     * @brief A timer class. The function click gives the duration and resets the timer
     */
    class Stopwatch{
        
        public:
        
        void start(){
            start_time = std::chrono::system_clock::now();
        }
        
        
        
        double click(){
            auto end_time = std::chrono::system_clock::now();
            std::chrono::duration<double> current_duration = end_time - start_time;
            
            // lap
            start_time = end_time;
            
            // Return the duration
            return current_duration.count();
        }

        private:
            std::chrono::system_clock::time_point start_time;

    };

    // Retrieve a parameter from the rosparam server or throw an error when it does not exist (i.e. blocking retrieval)
    template <class T> 
    bool safelyRetrieve(ros::NodeHandle& nh, const std::string name, T& param){


        if (!nh.getParam(name, param)) {
            throw RetrievalException("Helpers: Failed to retrieve parameter " + nh.getNamespace() + "/" + name);
            return false;
        }

        return true;

    }

    // Return true if the parameter on the server is true
    inline bool ifParameter(ros::NodeHandle& nh, const std::string& name){

        bool param;
        if (!nh.getParam(name, param)) {
            logMsg("Helpers", "Failed to find parameter " + nh.getNamespace() + "/" + name + ", defaulted to false", 1);
            return false;
        }

        return param;

    }

    // Retrieve a parameter from the rosparam server or use the default value if it does not exist (i.e. nonblocking retrieval)
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

    /* Safely retrieve an array of said size */
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

    // Return true if a parameter exists
    inline bool hasParameter(ros::NodeHandle& nh, const std::string& name){

        return nh.hasParam(name);
    }

    // Sign Function
    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    /* Retrieve an array of the expected_size and convert it to an Eigen::Vector */
    inline bool safelyRetrieveEigen(ros::NodeHandle& nh, const std::string name,
                             Eigen::VectorXd& param, const int expected_size){

        std::vector<double> result_v;
        safelyRetrieveArray(nh, name, result_v, expected_size);

        param = vectorToEigen(result_v);

        return true;
            
    }

    // Same as above but without a size check
    inline bool safelyRetrieveEigen(ros::NodeHandle& nh, const std::string name,
                             Eigen::VectorXd& param){

        std::vector<double> result_v;
        safelyRetrieve(nh, name, result_v);

        param = vectorToEigen(result_v);

        return true;

    }

    // Wrapper for a retrieving error
    inline void errorRetrieving(std::string name, const char* ex_what){
        logMsg("Helpers", "Exception getting " + name + ex_what, 0);
    }

    // Convert an array to a vector
    template <int N>
    Eigen::VectorXd arrayToVector(std::array<double, N> input){

        Eigen::VectorXd result(N);

        for(int i = 0; i < N; i++){
            result(i) = input[i];
        }

        return result;
    }
    
    // Computation of the pseudo inverse
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
        
        // While the function returns false, try again
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
    void lowpassFilter(T& filtered_input, const T& input, double alpha){

        try{
            filtered_input = (1.0 - alpha) * filtered_input + alpha * input;
        }catch(const std::exception& ex){
            logMsg("Helpers", "Filtering failed! Error: " + std::string(ex.what()));
        }

    }

    // Copied from https://answers.ros.org/question/63491/keyboard-key-pressed/
    // (non blocking input for the interface)
    inline int getch()
    {
      static struct termios oldt, newt;
      tcgetattr( STDIN_FILENO, &oldt);           // save old settings
      newt = oldt;
      newt.c_lflag &= ~(ICANON);                 // disable buffering      
      tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

      int c = getchar();  // read character (non-blocking)

      tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
      return c;
    }

    // Count non zero entries in vec
    inline unsigned int nonZeroCount(const Eigen::VectorXd& vec){
        
        unsigned int result = 0;
        for(unsigned int i = 0; i < vec.rows(); i++){
            if(vec[i] != 0.0){
                result++;
            }
        }
        
        return result;
    }

}
#endif