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

    ros::NodeHandle nh("/Elisa3/"); // Private.

    //helpers::safelyRetrieve(nh, "address", address, 3656);
    //helpers::safelyRetrieve(nh, "sampling_rate", sampling_rate, 100);

    helpers::safelyRetrieve(nh, "enabled/accelerometer", accelerometer_enabled, true);
    helpers::safelyRetrieve(nh, "enabled/motor_pos", motor_position_enabled, true);
//    np.param("theta", init_theta, 0.0);

    // Init communications
    startCommunication(address, 1);

    this->setState(Eigen::VectorXd::Zero(n),
                   Eigen::VectorXd::Zero(n),
                   Eigen::VectorXd::Zero(n));

    last_speed_setpoint = Eigen::VectorXd::Zero(n);
    /* Add publishers & Subscribers */
    //tau_pubs[i] = nh.advertise<std_msgs::Float64>("/robot1/panda_joint" +
      //                                                to_string(i+1) +
     //                                                 "_controller/command", 100);

    //sensor_sub = nh.subscribe("/robot1/joint_states", 100, &PandaSim::readStateCallback, this);
    logMsg("Elisa3", "Done!", 2);
}

bool Elisa3::sendInput(Eigen::VectorXd tau){

    // Apply feedback linearisation around the hand point (output = F, tau)
    Eigen::VectorXd output = feedbackLinearisation(tau);

    // Artificially implement torque control instead of speed control (output = v, omega)
    output = state.dq + output*sampling_rate;
    state.dq = output;
    // Convert tau / omega to wheel speeds (output = omega_r, omega_l)
    output = velocityToWheelSpeed(output);

    setLeftSpeed(address, output(0, 0));
    setRightSpeed(address, output(1, 0));

    return true;
}

void updateSensorData(){

    // Retrieve the accelerations
    Eigen::VectorXd a(3);
    a(0,0) = getAccX(address);
    a(1,0) = getAccY(address);
    a(2,0) = getAccZ(address);

    Eigen::VectorXd q(3);
    q(0, 0) = getOdomXpos(robotAddress[0]);
    q(1, 0) = getOdomYpos(robotAddress[0]);
    q(3, 0) = getOdomTheta(robotAddress[0]);

    // Convert to hand position
    Eigen::VectorXd h(2);
    h(0, 0) = q(0) + L*std::cos(q(2));
    h(1, 0) = q(1) + L*std::sin(q(2));
    // In calculations I need omega, but it is easier to just use the last point...

    this->setState(h, this->state.dq, h);
}

void checkSafety(){
    return;
}

Eigen::MatrixXd Elisa3::M(){

    return Eigen::MatrixXd::Identity(n, n);

}

Eigen::VectorXd Elisa3::dVdq(){

    return Eigen::VectorXd::Zero(n);
}

Eigen::MatrixXd Elisa3::Psi(){
    return Eigen::MatrixXd::Identity(n, n);
}

Eigen::VectorXd Elisa3::feedbackLinearisation(const Eigen::VectorXd tau){
    Eigen::VectorXd q = this->state.q;
    Eigen::VectorXd dq = this->state.dq;

    // Define the transformation (todo Optimise)
    Eigen::MatrixXd A(n, n);
    Eigen::VectorXd dA(n);
    A << 1/m*std::cos(q(3)), -L/J*std::sin(q(3)), 1/m*std::sin(q(3)), L/J*std::cos(q(3));
    dA << -dq(0)*dq(1)*std::sin(q(3)) - L*dq(1)*dq(1)*std::cos(q(3)),
            dq(0)*dq(1)*std::cos(q(3)) - L*dq(1)*dq(1)*std::sin(q(3));

    // Apply the feedback linearisation
    return A.inverse()*(tau - dA);
}

Eigen::VectorXd Elisa3::VelocityToWheelSpeed(const Eigen::VectorXd vel){
    return wheel_matrix*input;
}

void Elisa3::initWheelMatrix(){
    wheel_matrix = Eigen::MatrixXd::Zero(n, n);
    wheel_matrix << 1.0/r, l/(2.0*r), 1.0/r, -l/(2.0*r);
}