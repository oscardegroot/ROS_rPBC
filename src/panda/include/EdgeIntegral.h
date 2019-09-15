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
	EdgeIntegral(int i, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);

	void applyReconstruction(Eigen::VectorXd & wave_reference, Eigen::VectorXd r_i);

	Eigen::VectorXd calculateControls(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
	Eigen::VectorXd calculateWaves(const Eigen::VectorXd& tau, const Eigen::VectorXd& r_js) override;

	void initChannels() override;
	void setScatteringGain(Eigen::MatrixXd gain);
	void saturateIntegral();
	Eigen::VectorXd iterateST(Eigen::VectorXd s_in, Eigen::VectorXd r_i);
	Eigen::VectorXd sample(Eigen::VectorXd r_i) override;

	// Functions for integrator windup schemes
	void reset();
	void activate();
	void deactivate();

private:

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd gain_tau, gain_wave, matrix_ST;

	Eigen::VectorXd integral_state;

	double agent_i;
	double saturation;

	




};