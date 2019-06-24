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

using namespace std;

class PandaSim : public System{

public:
	PandaSim();

	// See definitions in system.h
	Eigen::VectorXd readSensors();
	bool sendInput(Eigen::VectorXd tau);
	Eigen::MatrixXd getM();
	Eigen::VectorXd getdHdq();
	void readStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	Eigen::Vector3d getEEPose(Eigen::Matrix<double, 7, 1> q);
	Eigen::VectorXd getdVdq();



private:

	// Get a nodehandle
	ros::NodeHandle nh;

	// Define publishers for the input
	ros::Publisher tau_pubs[7];
	ros::Subscriber sensor_sub;

	// DH Parameters
    Eigen::Matrix<double, 7, 1> DH_a, DH_d, DH_alpha;
    Eigen::Matrix<double, 3, 1> L_cogs[7];

    // The second mass should appear in dV dq (added 0)
    const double mass[10] = {3.66357, 2.04471, 2.56414, 2.48050, 2.02754, 3.09611, 1.66736, 0.75606, 0.1};


    const double g = 9.81;
	
};


#endif