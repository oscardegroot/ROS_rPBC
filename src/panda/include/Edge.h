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
#include "Agent.h"
#include "Potential.h"

class Edge{
public:
	// Constructor and destructor
	Edge(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);
	~Edge();

	// Receive and send waves
	virtual void waveCallback(const panda::Waves::ConstPtr& msg);
    
	virtual void publishWave(const Eigen::VectorXd& r_i);
    
    virtual void initChannels();

	virtual Eigen::VectorXd sample(const Eigen::VectorXd& r_i);
	
	virtual void applyReconstruction(Eigen::VectorXd & wave_reference, const Eigen::VectorXd& r_i);
    
	virtual Eigen::VectorXd calculateControls(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i) = 0;
    
	virtual Eigen::VectorXd calculateWaves(const Eigen::VectorXd& s_in,	const Eigen::VectorXd& r_i) = 0;

    // Waves for dealing with sampling frequencies
    void expandWaves(Eigen::VectorXd& waves);
    void compressWaves(const Eigen::VectorXd& waves);
    
    virtual void addObstacle(const std::shared_ptr<Obstacle>& obstacle){};
    
    virtual bool isLeader() const { return false;};


protected:

    int i_ID, j_ID, l;
	Eigen::MatrixXd gain;
	Eigen::VectorXd r_star;
    
    int rate_mp;
    std::unique_ptr<helpers::Counter> publish_counter;

	// The last received timestamp
	int timestamp = 0;
	bool data_received = false;

	// The Ros node handle
	ros::NodeHandle nh;

	// Define a publisher and subscriber for the waves
	ros::Publisher wave_pub;
	ros::Subscriber wave_sub;
    
    // WVM buffer, received buffer, actual used sample (possibly expanded)
	Eigen::VectorXd s_wvm_buffer, s_received, s_sample;
    Eigen::VectorXd s_out_compressed, s_out_squared;

};

#endif