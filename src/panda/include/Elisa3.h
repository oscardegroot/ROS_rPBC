//
// Created by omdegroot on 06-08-19.
//

#ifndef ELISA3_H
#define ELISA3_H
/*
File: Elisa3.h

*/

#include "ros/ros.h"
#include "System.h"
#include "Helpers.h"
#include "Exceptions.h"

#include <realtime_tools/realtime_publisher.h>
#include "std_msgs/Float64MultiArray.h"
#include "panda/registerElisa3.h"
#include "panda/Move.h"
#include "panda/Readout.h"
#include "panda/colorElisa3.h"

#include "elisa3-lib.h"
#include <cmath>
#include <memory>
/* Possibly divide this up further in a system and a controller */

class Elisa3 : public System{

public:
    Elisa3(int set_address, int set_sampling_rate);
    ~Elisa3();

    // Coordinate count, actuated count
    bool sendInput(const Eigen::VectorXd& tau);
    void readSensors(const panda::Readout::ConstPtr& msg);
    
	Eigen::MatrixXd& M() override;
	Eigen::VectorXd& dVdq() override;
	Eigen::MatrixXd& Psi() override;
    Eigen::MatrixXd& dM() override;
    Eigen::MatrixXd& dPsi() override;
    Eigen::MatrixXd& dMinv() override;

    void checkSafety() override;

    void initMatrices();
    Eigen::VectorXd feedbackLinearisation(const Eigen::VectorXd tau);
    Eigen::VectorXd velocityToWheelSpeed(const Eigen::VectorXd vel);
    void saturateSpeed(signed int & v_r, signed int & v_l);
    void lowpassFilter(Eigen::VectorXd filtered_value,
                        const Eigen::VectorXd & value,
                        const double alpha);

    bool dataReady() override;

    void setColor(int color_type);
    void setColor(int r, int g, int b);

private:

    int address, sampling_rate;
    bool accelerometer_enabled, motor_position_enabled;
    Eigen::VectorXi last_speed_setpoint;
    Eigen::VectorXd q0;

    realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> test_pub;

    bool data_received = false;
    ros::Publisher move_pub;
    panda::Move move_msg;
    ros::Subscriber readout_sub;
    ros::ServiceClient color_client;
    // We need to make a distinction between feedback linerised and general
    Eigen::VectorXd actual_dq;
    Eigen::VectorXd actual_q;

    double L, Ts;

    // Filtering
    double alpha = 0.99;
    //Eigen::VectorXd filtered_dq;

    static constexpr double r{0.0045}; // Wheel radius (9mm/2)
    static constexpr double l{0.0408}; // Distance between wheels (40.8mm)
    static constexpr double m{0.039}; // Total weight (39 g)
    static constexpr double J{0.000012187}; // Inertia (calculated as 0.5mr^2 for massive cylinder

    // Actual max/min at 127
    static constexpr signed int max_speed{50}, min_speed{-30}, max_speed_rate{50};
    // For later
    // d = 50mm (body, not wheel!)
    // max speed 60cm/s


    Eigen::MatrixXd wheel_matrix, F;



};


#endif
