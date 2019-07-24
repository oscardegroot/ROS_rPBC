/*
File: Edge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#pragma once

#include "Edge.h"
#include "Helpers.h"

class EdgeIntegral : public Edge{
public:
	// Constructor and destructor
	EdgeIntegral(int i, int j, Eigen::MatrixXd gain_set, int l_set, bool is_integral);

	void applyReconstruction(Eigen::VectorXd & wave_reference, Eigen::VectorXd r_i);

	Eigen::VectorXd calculateControls(Eigen::VectorXd r_i, Eigen::VectorXd r_js) override;
	Eigen::VectorXd calculateWaves(Eigen::VectorXd tau, Eigen::VectorXd r_js) override;

	void setScatteringGain(Eigen::MatrixXd gain);
	void saturateIntegral();
	Eigen::VectorXd iterateST(Eigen::VectorXd s_in, Eigen::VectorXd r_i);
	Eigen::VectorXd sample(Eigen::VectorXd r_i) override;
	void reset() override;
	void activate() override;
	void deactivate() override;

private:

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd gain_tau, gain_wave, matrix_ST;

	Eigen::VectorXd integral_state;

	double agent_i;
	double saturation;


};