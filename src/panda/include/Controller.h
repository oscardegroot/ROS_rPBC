/*

File: Controller.h

An interface for controllers using IDAPBC or rPBC control.

*/

#ifndef Controller_H
#define Controller_H

#include "ros/ros.h"

#include "System.h"
#include "Agent.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64MultiArray.h"

#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>

#include "Helpers.h"
#include <memory>

class Controller{

public:
	Controller(Agent& agent);
	virtual ~Controller();

	ros::NodeHandle nh;

	// Realtime publishing
	franka_hw::TriggerRate tau_rate{1.0};
	franka_hw::TriggerRate z_rate{1.0};
	realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> tau_pub, z_pub;

    // Get the agent output, possibly modified
    virtual
    Eigen::VectorXd getOutput(System& system) = 0;

    // Compute the input (POSSIBLY AT A ROBOTSTATE HERE)
    virtual
    Eigen::VectorXd computeControl(System& system, const Eigen::VectorXd& tau_c) = 0;
    
	/** @brief Local control potential */
    virtual
	Eigen::VectorXd dVsdq(System& system) = 0;

	/** @brief Damping matrix */
	virtual
    Eigen::MatrixXd Kv(System& system);

    void publishValue(realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>& pub,
            franka_hw::TriggerRate trigger_rate, const Eigen::VectorXd values);

protected:
    int l;

    // Gains
	Eigen::VectorXd Vs_gains, theta_star, limit_avoidance_gains, limits_avg;
	bool local_enabled, limit_avoidance_enabled, gravity_enabled,integral_enabled;
	double kq, kz, ki;
};


#endif