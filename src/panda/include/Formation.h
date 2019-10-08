/** @class Formation: holds formation information. 
 * Derived classes implement a type of formation based on user parameters
 */

#pragma once

#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "CustomLog.h"
#include "Exceptions.h"
#include "Helpers.h"

/* Defines a connection between agents */
struct connection {
	int id;
	Eigen::VectorXd r_star;
};

/** Describes a formation and implements retrieval methods */
class Formation{
    
public:

    Formation();

    /* Retrieve connection information */
	bool retrieveConnectionBetween(u_int id_i, u_int id_j, Eigen::VectorXd & r_star);

protected:

	// Agent count and cooperative dimension
	int N, l;
    ros::NodeHandle nh;
    
    // Convert 2D positions to connections (possibly extend to the N dimensional case)
    void connectionsFromPositions(const std::vector<Eigen::Matrix<double, 2, 1>>& positions);
    
	// A vector with connections for every agent
	std::vector<std::vector<connection>> connections;
};

/* Derived classes, constructors describe the formations*/
class Consensus : public Formation{

public:
    Consensus();
};

class CircleFormation : public Formation{

public:
    CircleFormation();
};

class LineFormation : public Formation{

public:
    LineFormation();
};