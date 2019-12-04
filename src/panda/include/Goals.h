/*
File: Goals.h

Contains a number of functions describing formations and other cooperative goals
*/

#ifndef Goals_H
#define Goals_H

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include "panda/Waves.h"
#include "CustomLog.h"
#include "Exceptions.h"
#include "Formation.h"

#include "Helpers.h"

// Holds leader data
struct leader {
    int id;
    Eigen::VectorXd ref;
    Eigen::VectorXd gain;
};

/**
 * @class Goals
 * @author Oscar
 * @file Goals.h
 * @brief Class that creates and holds formations and leaders
 */
class Goals{

public:

	// The construct takes the number of agents
	Goals();
    
    // The formation
    std::unique_ptr<Formation> formation;
    Formation getFormation(const std::string& formation_type);
    
    // Leader initialisation and service
	void initLeaders(ros::NodeHandle & nh);
    bool isAgentLeader(u_int id, Eigen::VectorXd & ref, Eigen::VectorXd & gain);


private:
	
	// Agent count and cooperative dimension
	int N, l;
    
	std::vector<leader> leaders;
};




#endif