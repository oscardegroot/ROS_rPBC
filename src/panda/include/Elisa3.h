#ifndef ELISA3_H
#define ELISA3_H

#include "ros/ros.h"
#include "System.h"
#include "Helpers.h"
#include "Exceptions.h"
#include "Elisa3_Station.h"

#include <realtime_tools/realtime_publisher.h>
#include "std_msgs/Float64MultiArray.h"
#include "panda/registerElisa3.h"
#include "panda/Move.h"
#include "panda/Readout.h"
#include "panda/colorElisa3.h"

#include "elisa3-lib.h"
#include <cmath>
#include <memory>

/**
 * @class Elisa3
 * @author Oscar
 * @date 08/10/19
 * @file Elisa3.h
 * @brief Class for implementation of Passivity-Based Control for an Elisa3 differential drive robot
 * The given code implements feedback linearisation to transform the system to a point mass. Since the inputs are velocities
 * an acceleration is generated using the controller force and is integrated to retrieve a velocity setpoint.
 */
class Elisa3 : public System{

public:
    Elisa3();
    ~Elisa3();

    /**
     * @brief Calculate and apply wheel speed setpoints from the computed control force
     * @param tau Input from the controller (see Controller class)
     */
    bool sendInput(const Eigen::VectorXd& tau) override;
    
    /**
     * @brief Retrieve sensor input from the Elisa3_station
     * @param msg Message with odometry positions
     */
    void readSensors(const panda::Readout::ConstPtr& msg);
    
    /** Dynamic matrices, implements point mass */
	Eigen::MatrixXd& M() override;
	Eigen::VectorXd& dVdq() override;
	Eigen::MatrixXd& Psi() override;
    Eigen::MatrixXd& dM() override;
    Eigen::MatrixXd& dPsi() override;
    Eigen::MatrixXd& dMinv() override;

    void checkSafety() override;
    void initMatrices();
    
    /** Feedback linearise the robot for a given input
     * @param[in] tau Force inputs on the feedback linearised model
     * 
     * @out Input forces to the actual system */
    Eigen::VectorXd feedbackLinearisation(const Eigen::VectorXd& tau);
    
    /** Convert a given velocity to the wheel speed convention of the Elisa3 
     * @param[in] vel Setpoint velocity of the system in m/s
     * 
     * @out Wheel speed that may be send to the robot */
    Eigen::VectorXd velocityToWheelSpeed(const Eigen::VectorXd& vel);
    
    /** Saturate the wheel speeds if necessary */
    void saturateSpeed(signed int & v_r, signed int & v_l);

    bool dataReady() override;
    
    /** Some color function wrappers
     * 
     * @param[in] color Name of the color (blue, green, red, etc.)*/
    void setColor(const std::string& color);
    
    /** @param[in] color_type Index of a color */
    void setColor(int color_type);
    
    /** @param[in] r, g, b Red / Green / Blue intensity to be send to the Elisa3*/
    void setColor(int r, int g, int b);

private:

    Eigen::VectorXi last_speed_setpoint;
    
    int address;
    
    // Initial state
    Eigen::VectorXd q0;

    bool data_received = false;
    
    ros::NodeHandle nh;
    ros::Publisher move_pub;
    panda::Move move_msg;
    ros::Subscriber readout_sub;
    ros::ServiceClient color_client;
    realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> test_pub;
    
    // We need to make a distinction between feedback linearised and general
    Eigen::VectorXd actual_dq;
    Eigen::VectorXd actual_q;
    
    double L, Ts;
    
    /* Constants */
    // Filtering
    double alpha = 0.99;

    static constexpr double r{0.0045}; // Wheel radius (9mm/2)
    static constexpr double l{0.0408}; // Distance between wheels (40.8mm)
    static constexpr double m{0.039}; // Total weight (39 g)
    static constexpr double J{0.000012187}; // Inertia (calculated as 0.5mr^2 for massive cylinder
    static constexpr signed int max_speed{50}, min_speed{-30}, max_speed_rate{50};// Actual max/min at 127
    // For later
    // d = 50mm (body, not wheel!)
    // max speed 60cm/s


    Eigen::MatrixXd wheel_matrix, F;



};


#endif
