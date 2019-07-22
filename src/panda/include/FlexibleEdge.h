/*
File: FlexibleEdge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#ifndef FlexibleEdge_H
#define FlexibleEdge_H

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include "panda/Waves.h"
#include "CustomLog.h"

class FlexibleEdge{
public:
	// Constructor and destructor
	FlexibleEdge(int i, int j, double gain_set, int l_set);
	~FlexibleEdge();

	void waveCallback(const panda::Waves::ConstPtr& msg);

	Eigen::VectorXd sample(Eigen::VectorXd r_i);
	void setScatteringGain();
	void publishWave(Eigen::VectorXd s_out);
 	void applyWVM(Eigen::VectorXd r_i);
	Eigen::VectorXd applyControls(Eigen::VectorXd r_i, Eigen::VectorXd r_js);


 	Eigen::VectorXd getRjs(Eigen::VectorXd s_in, Eigen::VectorXd tau);
	Eigen::VectorXd getWave(Eigen::VectorXd r_js, Eigen::VectorXd tau_in);

	Eigen::VectorXd elementSign(Eigen::VectorXd s_in);

private:

	int i_ID, j_ID, l;
	double agent_i;
	double gain, b;

	// The network transformation
	Eigen::MatrixXd T;
	Eigen::VectorXd last_tau;

	// The buffer for WVM purposes
	Eigen::VectorXd s_buffer, s_received;

	// The last received timestamp
	int timestamp = 0;
	bool data_received;

	// The Ros node handle
	ros::NodeHandle nh;

	// Define a publisher and subscriber for the waves
	ros::Publisher wave_pub;
	ros::Subscriber wave_sub;


};

#endif