/*
File: Panda.h

An implementation of the agent class for the Franka Emika panda
*/

#ifndef Panda_H
#define Panda_H

#include "ros/ros.h"

#include "Agent.h"

#include <vector>
#include <string>
#include <sstream>

class Panda : public Agent{

public:
	Panda();

	Eigen::VectorXd readSensors();
	bool sendInput();
	Eigen::MatrixXd getM(Eigen::VectorXd q);
	Eigen::VectorXd getdHdq(Eigen::VectorXd q);


private:
	const int nq = 3;
	const int mq = 3;

	
};


#endif