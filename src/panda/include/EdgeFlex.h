/*
File: Edge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#pragma once

#include "Edge.h"
#include "math.h"
#include <stdexcept>

class EdgeFlex : public Edge{
public:
	// Constructor and destructor
	EdgeFlex(int i, int j, Eigen::MatrixXd gain_set, int l_set, bool is_integral);

	void applyReconstruction(Eigen::VectorXd & wave_reference, Eigen::VectorXd r_i);

	Eigen::VectorXd sample(Eigen::VectorXd r_i) override;
	Eigen::VectorXd calculateWaveReference(Eigen::VectorXd s_in, Eigen::VectorXd tau);
	Eigen::VectorXd calculateControls(Eigen::VectorXd r_js, Eigen::VectorXd r_i) override;
	Eigen::VectorXd calculateWaves(Eigen::VectorXd s_in, Eigen::VectorXd tau) override;

	// Iterates starting with a tau_0 and the incoming wave
	// No output and tau is modified by reference
	Eigen::VectorXd iterateST(Eigen::VectorXd &s_in, Eigen::VectorXd tau,  Eigen::VectorXd r_i);
	
	Eigen::VectorXd elementSign(Eigen::VectorXd s_in);
	void setScatteringGain(Eigen::MatrixXd gain);
	void lowpassFilter(Eigen::VectorXd& filtered_data,
					 Eigen::VectorXd new_data, double alpha);

	double gamma_wang(Eigen::VectorXd r_i, Eigen::VectorXd r_js);


private:

	// Save of the iterated tau and tau in the last loop
	Eigen::VectorXd tau_last;

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd matrix_ST;

	Eigen::VectorXd r_js_filtered;
};