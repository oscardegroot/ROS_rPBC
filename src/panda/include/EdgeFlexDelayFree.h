/*
File: FlexibleEdge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#pragma once

#include "Edge.h"
#include "Helpers.h"

class EdgeFlexDelayFree : public Edge{
public:
	// Constructor and destructor
	EdgeFlexDelayFree(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);

	void waveCallback(const panda::Waves::ConstPtr& msg) override;
	void publishWave(Eigen::VectorXd s_out) override;

	Eigen::VectorXd calculateControls(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i);
	Eigen::VectorXd calculateWaves(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
	void applyReconstruction(Eigen::VectorXd & wave_reference,
									 Eigen::VectorXd r_i);

	double gamma_wang(Eigen::VectorXd r_i, Eigen::VectorXd r_js);


private:

	Eigen::VectorXd goal;

};