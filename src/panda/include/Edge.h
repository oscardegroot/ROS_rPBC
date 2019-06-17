/*
File: Edge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#ifndef Edge_H
#define Edge_H

//#include <thread>
//#include <mutex>
#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include "panda/Waves.h"
//#include <memory>

class Edge{
public:
	// Constructor and destructor
	Edge(int i, int j, double gain, int l_set);
	~Edge();

	void waveCallback(const panda::Waves::ConstPtr& msg);

	Eigen::VectorXd sampleEdge(Eigen::VectorXd r_i);


private:

	int i_ID, j_ID, l_dim;
	bool is_agent_i;

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd gain_tau, gain_wave;

	// The buffer for WVM purposes
	Eigen::VectorXd s_buffer, s_received;

	void setScatteringGain(double gain);
	void publishWave(Eigen::VectorXd r_i);
	void applyWVM(Eigen::VectorXd & wave_reference, Eigen::VectorXd r_i);
	Eigen::VectorXd getTauST(Eigen::VectorXd s_in, Eigen::VectorXd r_i);
	Eigen::VectorXd getWaveST(Eigen::VectorXd s_in, Eigen::VectorXd r_i);

	// The last received timestamp
	int timestamp = 0;
	bool data_received = false;

	//Threadlock
	//std::unique_ptr<std::mutex> m = std::make_unique<std::mutex>();;
	// The Ros node handle
	ros::NodeHandle nh;

	// Define a publisher and subscriber for the waves
	ros::Publisher wave_pub;
	ros::Subscriber wave_sub;


};

#endif