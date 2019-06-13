/*
File: Edge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include "panda/Waves.h"
// #include <cmath>

class Edge{
public:
	// Constructor and destructor
	Edge(int i, int j, bool agent_i, double gain);
	~Edge();

	void waveCallback(const panda::Waves::ConstPtr& msg);

	Eigen::Matrix<double, 2, 1> sampleEdge(Eigen::Matrix<double, 2, 1> r_i);

private:

	int i_ID, j_ID;
	bool is_agent_i;

	// The wave impedance B and network gain Kd
	Eigen::Matrix<double, 2, 2> B, Kd;

	// The buffer for WVM purposes
	Eigen::Matrix<double, 2, 1> s_buffer;
	// The last received timestamp
	uint64_t last_timestamp;
	// The last received wave
	Eigen::Matrix<double, 2, 1> s_received;


	// The Ros node handle
	ros::NodeHandle nh;

	// Define a publisher and subscriber for the waves
	ros::Publisher wave_pub;
	ros::Subscriber wave_sub;


};