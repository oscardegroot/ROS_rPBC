/*
File: System.h

Defines an interface for systems controlled by IDAPBC or rPBC
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
	Eigen::VectorXd z;
};

class System{

public:
	System(int n_set, int m_set);
	~System();

	// Coordinate count, actuated count
	int n = 0;
	int m = 0;
	State state;

	// virtual Eigen::VectorXd readSensors() = 0;
	virtual bool sendInput(Eigen::VectorXd tau) = 0;
	virtual Eigen::MatrixXd M() = 0;
	virtual Eigen::VectorXd dVdq() = 0;
	virtual Eigen::MatrixXd Psi() = 0;

	// bool dataReady();
	// void setDataReady(bool is_ready);
	int getN(){return n;};
	void setState(Eigen::VectorXd new_q, Eigen::VectorXd new_qd, Eigen::VectorXd new_z);

	virtual void checkSafety();

private:
	

};


#endif