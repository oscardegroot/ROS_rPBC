/*
File: Edge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#pragma once

#include "Edge.h"

class EdgeDirect : public Edge{
public:
	// Constructor and destructor
	EdgeDirect(int i, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set);

	void applyReconstruction(Eigen::VectorXd & wave_reference, Eigen::VectorXd r_i);

	Eigen::VectorXd calculateControls(Eigen::VectorXd s_in, Eigen::VectorXd r_i);
	Eigen::VectorXd calculateWaves(Eigen::VectorXd s_in, Eigen::VectorXd r_i);
	Eigen::VectorXd elementSign(Eigen::VectorXd s_in);

	void setScatteringGain(Eigen::MatrixXd gain);

private:

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd gain_tau, gain_wave;

};