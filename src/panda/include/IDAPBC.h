/*
File: IDAPBC.h

A standard IDA-PBC controller, implements the controller interface defined in Controller.h
*/

#ifndef IDAPBC_H
#define IDAPBC_H

#include "ros/ros.h"

#include "Controller.h"

#include <vector>
#include <string>
#include <sstream>

class IDAPBC : public Controller{

public:
	IDAPBC();

	Eigen::VectorXd computeControl(VectorXd q_i, VectorXd dq_i);


private:

	Eigen::VectorXd getdVsdq(VectorXd q);

};


#endif