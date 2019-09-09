/*
File: Panda.h

An implementation of the agent class for the Franka Emika panda
*/

#ifndef PandaSim_H
#define PandaSim_H

#include "ros/ros.h"
#include "System.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "CustomLog.h"
#include <string>

using namespace std;

class PandaSim : public System{

public:
	PandaSim();

	// See definitions in system.h
	bool sendInput(const Eigen::VectorXd& tau) override;

	void M(Eigen::MatrixXd& M_out) override;
	void dVdq(Eigen::VectorXd& dVdq_out) override;
	void Psi(Eigen::MatrixXd& Psi_out) override;

	Eigen::VectorXd getZ(Eigen::VectorXd q);

	void readStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

private:

	// Get a nodehandle
	ros::NodeHandle nh;

	// Define publishers for the input
	ros::Publisher tau_pubs[7];
	ros::Subscriber sensor_sub;

    const double g = 9.81;
	
};


#endif