/*
File: Edge.h (Interface)

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#ifndef Edge_H
#define Edge_H

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include "panda/Waves.h"
#include "CustomLog.h"
#include "Helpers.h"

class Edge{
public:
	// Constructor and destructor
	Edge(int i, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);
	~Edge();

	int i_ID, j_ID, l;
	bool is_agent_i;
	Eigen::MatrixXd gain;
	Eigen::VectorXd r_star;
    
    int rate_mp;
    helpers::Counter pub_counter;
    Eigen::VectorXd s_out_compressed, s_out_squared;

	// The last received timestamp
	int timestamp = 0;
	bool data_received = false;

	bool is_activated;

	// The Ros node handle
	ros::NodeHandle nh;

	// Define a publisher and subscriber for the waves
	ros::Publisher wave_pub;
	ros::Subscriber wave_sub;

	// The buffer for WVM purposes
	Eigen::VectorXd s_buffer, s_received;

	// Receive and send waves
	virtual void waveCallback(const panda::Waves::ConstPtr& msg);
	virtual void publishWave(Eigen::VectorXd r_i);
    virtual void initChannels();

	virtual Eigen::VectorXd sample(Eigen::VectorXd r_i);
	
	virtual void applyReconstruction(Eigen::VectorXd & wave_reference,
									 Eigen::VectorXd r_i);
	virtual Eigen::VectorXd calculateControls(const Eigen::VectorXd& s_in,
											const Eigen::VectorXd& r_i) = 0;
	virtual Eigen::VectorXd calculateWaves(const Eigen::VectorXd& s_in,
										const Eigen::VectorXd& r_i) = 0;


private:

};

#endif