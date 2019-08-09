//
// Created by omdegroot on 06-08-19.
//
/*
 * Elisa3.cpp
 *
 * Model for feedback linearised Elisa3 with passivity-based control
 *
 * Measured are: x, y, theta (angle versus world frame)
 *               ddx, ddy
 *
 * This file uses x, y, theta to feedback linearise the system around the "hand point"
 * h = [x;y] + L[cos(theta); sin(theta)]
 * The dynamics around this point are a point mass
 * Since speed control is the only possibility we use
 * dq[k+1] = dq[k] + tau[k]*Ts (note that mass was compensated in feedback linearisation)
 * Then dq[k+1] is the speed target and is converted to wheel speeds
 *
 * Outputs are omega_r, omega_l
 *
 *
 */
#include "Elisa3.h"

Elisa3::Elisa3(int set_address, int set_sampling_rate)
        :System(2, 2)
{
    logMsg("Elisa3", "Initialising...", 2);

    // Retrieve parameters
    address = set_address;
    sampling_rate = set_sampling_rate;
    Ts = 1.0/sampling_rate;

    ros::NodeHandle nh("/Elisa3/"); // Private.

    test_pub.init(nh, "q", 1);

    // Init communications
    startCommunication(&address, 1);

    // (This is used to reset odometry)
    calibrateSensors(address);

    // Initialise all state related values
    // These are the feedback linearised states,...
    this->setState(Eigen::VectorXd::Zero(n),
                   Eigen::VectorXd::Zero(n),
                   Eigen::VectorXd::Zero(n));

    // And these are the actual system states
    actual_q = Eigen::VectorXd::Zero(3);
    actual_dq = Eigen::VectorXd::Zero(3);

    // Initialise the transformation from velocities to wheel velocities
    initMatrices();

    logMsg("Elisa3", "Done!", 2);
}

Elisa3::~Elisa3(){}

/// Convert the control input to the system inputs and actuate
bool Elisa3::sendInput(Eigen::VectorXd tau){

    // Apply feedback linearisation around the hand point (output = F, tau)
    Eigen::VectorXd output = feedbackLinearisation(tau);

    // Artificially implement torque control instead of speed control (output = v, omega (m/s, rad/s))
    output = actual_dq + (F*output)*Ts;
    actual_dq = output;

    // Convert to wheel velocities
    output = wheel_matrix*output;

    // Convert them to integer values
    //signed int wheel_right = std::ceil(output(0)*1000.0/5.0); // maybe just round up?
    signed int wheel_right = static_cast<signed int>(output(0)*1000.0/5.0); // maybe just round up?
    signed int wheel_left = static_cast<signed int>(output(1)*1000.0/5.0);

    // Saturate the speed to the max speed (and scale if necessary)
    saturateSpeed(wheel_right, wheel_left);

    // Apply the speeds
    setRightSpeed(address, wheel_right);
    setLeftSpeed(address, wheel_left);

    return true;
}

/// Read Sensors from the Elisa3 (determines q, h and dh)
void Elisa3::readSensors(){

    /// Retrieve the accelerations ( correct compared with gtronic )
//    Eigen::VectorXd a(2); //SWAP X AND Y HERE!
//    a(0,0) = -getAccY(address);// / 64.0 * 9.81;
//    a(1,0) = getAccX(address);// / 64.0 * 9.81;
//    a *= (9.81/64.0);

    /// Retrieve the state from odometry (ACTUAL STATE)
    Eigen::VectorXd q(3); // X and Y swapped? Yes
    q(0, 0) = -getOdomYpos(address)/1000.0;  // mm to m
    q(1, 0) = getOdomXpos(address)/1000.0;  // mm to m
    q(2, 0) = getOdomTheta(address) / 180.0 * M_PI + M_PI_2; // Theta expressed in a tenth of degree

    // Save values
    actual_q = q;

    if(test_pub.trylock()) {
        test_pub.msg_.data.resize(q.size());

        for (int i = 0; i < q.size(); i++) {
            test_pub.msg_.data[i] = q(i);
        }

        test_pub.unlockAndPublish();
    }

    /// Convert to hand position
    Eigen::VectorXd h(2);
    h(0, 0) = q(0) + L*std::cos(q(2));
    h(1, 0) = q(1) + L*std::sin(q(2));

    /// Find hand velocities for control
    Eigen::VectorXd dh(2);
    Eigen::MatrixXd A(2, 2);
    A << std::cos(q(2)), -L*std::sin(q(2)),
         std::sin(q(2)), L*std::cos(q(2));
    dh = A * actual_dq;

    // Set the public state
    this->setState(h, dh, h);
}

/// Check if program can proceed
void Elisa3::checkSafety(){

    /// Check if robot is within the workspace
    for(int i = 0; i < n; i++){ //todo: 0.8 from parameter file
        if(std::abs(actual_q(i)) > 0.8){
            throw BoundException("Elisa3 out of bounds (state=" + std::to_string(i) +
            ", bound=" + std::to_string(0.8) + ")!");
        }
    }

    return;
}

/// Lowpass filter an input vector
void Elisa3::lowpassFilter(Eigen::VectorXd filtered_value, const Eigen::VectorXd & value,
        const double alpha){

    filtered_value = (1 - alpha) * filtered_value + alpha * value;
}

/// Dynamical Matrices and Vectors
Eigen::MatrixXd Elisa3::M(){

    return Eigen::MatrixXd::Identity(n, n);
}

Eigen::VectorXd Elisa3::dVdq(){

    return Eigen::VectorXd::Zero(n);
}

Eigen::MatrixXd Elisa3::Psi(){
    return Eigen::MatrixXd::Identity(n, n);
}

/// Apply feedback linearisation
Eigen::VectorXd Elisa3::feedbackLinearisation(const Eigen::VectorXd tau){
    // Define the transformation (todo Optimise)
    Eigen::MatrixXd A(n, n);
    Eigen::VectorXd dA(n);
    A << (1.0/m)*std::cos(actual_q(2)), -(L/J)*std::sin(actual_q(2)),
         (1.0/m)*std::sin(actual_q(2)), (L/J)*std::cos(actual_q(2));

    dA << -actual_dq(0)*actual_dq(1)*std::sin(actual_q(2)) - L*actual_dq(1)*actual_dq(1)*std::cos(actual_q(2)),
            actual_dq(0)*actual_dq(1)*std::cos(actual_q(2)) - L*actual_dq(1)*actual_dq(1)*std::sin(actual_q(2));

    // Apply the feedback linearisation (see https://pdfs.semanticscholar.org/4f02/6f6aac864f5bc07b0f065ec2036046f3d306.pdf)
    return A.inverse()*(tau - dA);
}

/// Convert the vector [u, w] into [v_r, v_l]
Eigen::VectorXd Elisa3::velocityToWheelSpeed(const Eigen::VectorXd vel){
    // Gives [Vr, Vl]'
    return wheel_matrix*vel;
}

/// Saturate the speed to the set limit
void Elisa3::saturateSpeed(signed int & v_r, signed int & v_l) {

    // Find the maximum speed between the wheels
    int max_cur_speed = std::max(std::abs(v_r), std::abs(v_l));

    // If the speed has passed the limit
    if (max_cur_speed > max_speed) {
        logMsg("Elisa3", "Speed limit applied to Elisa3", 1);

        // Scale down the wheel speeds
        v_r = static_cast<signed int>(double(v_r) * double(max_speed) / double(max_cur_speed));
        v_l = static_cast<signed int>(double(v_l) * double(max_speed) / double(max_cur_speed));
    }
}

/// Initialise matrices
void Elisa3::initMatrices(){
    // The wheel transformation matrix
    wheel_matrix = Eigen::MatrixXd::Zero(n, n);
    wheel_matrix << 0.5, 0.5, 1.0/l, -1.0/l;
    wheel_matrix = wheel_matrix.inverse();

    // The input matrix
    F = Eigen::MatrixXd::Zero(2, 2);
    F << 1.0/m, 0.0, 0.0, 1.0/J;
}



    //for(int i = 0; i < 2; i++) {

        // Saturate the rate if necessary
//        signed int del_speed = speed(i) - last_speed_setpoint(i);
//        if(abs(del_speed) > max_speed_rate){
//
//            logMsg("Elisa3", "Speed rate saturated! (value=" +
//            std::to_string(del_speed)+ ")", 1);
//
//            // Apparently c++ has no sign function
//            signed int signed_del_speed = (del_speed > 0) - (del_speed < 0);
//            speed(i) = last_speed_setpoint(i) + signed_del_speed*max_speed_rate;
//        }


    //}

    //return speed;


/// Calculate the velocities (ACTUAL DSTATE)
// Difference based
//    Eigen::VectorXd dq(2);
//    double dx = q(0)-actual_q(0); // dx
//    double dy = q(1)-actual_q(1); // dy
//    dq(0) = std::sqrt(dx*dx + dy*dy) / Ts; // This is always positive
//    dq(1) = (q(2)-actual_q(2)) / Ts;


// Filter
//lowpassFilter(actual_dq, dq, alpha);