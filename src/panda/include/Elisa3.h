//
// Created by omdegroot on 06-08-19.
//

#ifndef SRC_ELISA3_H
#define SRC_ELISA3_H
/*
File: Elisa3.h

*/

#include "ros/ros.h"

/* Possibly divide this up further in a system and a controller */

class Elisa3{

public:
    Elisa3(int set_address, int set_sampling_rate);
    ~Elisa3();

    // Coordinate count, actuated count
    bool sendInput(Eigen::VectorXd tau);
    Eigen::MatrixXd M();
    Eigen::VectorXd dVdq();
    Eigen::MatrixXd Psi();

    void checkSafety();

    void initWheelMatrix();
    Eigen::VectorXd feedbackLinearisation(const Eigen::VectorXd tau);
    Eigen::VectorXd velocityToWheelSpeed(const Eigen::VectorXd vel);


private:

    int address, sampling_rate;
    bool accelerometer_enabled, motor_position_enabled;
    Eigen::VectorXd last_speed_setpoint;
    // Yolo value
    double L{0.03};

    static constexpr double r{0.009}; // Wheel radius (9mm)
    static constexpr double l{0.0408}; // Distance between wheels (40.8mm)
    static constexpr double m{0.039}; // Total weight (39 g)
    // For later
    // d = 50mm
    // max speed 60cm/s


    Eigen::MatrixXd wheel_matrix;


};


#endif //SRC_ELISA3_H
