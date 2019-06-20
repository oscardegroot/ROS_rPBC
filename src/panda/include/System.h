/*
File: Agent.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#ifndef System_H
#define System_H

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "panda/getConnectionsOf.h"

#include <vector>
#include <string>
#include <sstream>

/* Possibly divide this up further in a system and a controller */

struct State{
	Eigen::VectorXd q;
	Eigen::VectorXd dq;
};

class System{

public:
	System(int n_set, int m_set);
	~System();

	// Coordinate count, actuated count
	int n = 0;
	int m = 0;
	State state;

	virtual Eigen::VectorXd readSensors() = 0;
	virtual bool sendInput(Eigen::VectorXd tau) = 0;
	virtual Eigen::MatrixXd getM() = 0;
	virtual Eigen::VectorXd getdHdq() = 0;

	bool dataReady();
	void setDataReady(bool is_ready);
	int getN();
	void setState(Eigen::VectorXd new_q, Eigen::VectorXd new_qd);


private:

	bool data_rdy = true;

	

};


#endif