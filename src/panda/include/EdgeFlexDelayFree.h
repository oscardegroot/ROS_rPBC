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
	EdgeFlexDelayFree(int i, int j, Eigen::MatrixXd gain_set, int l_set, bool integral);

	void waveCallback(const panda::Waves::ConstPtr& msg) override;
	void publishWave(Eigen::VectorXd s_out) override;

	Eigen::VectorXd calculateControls(Eigen::VectorXd s_in,
											Eigen::VectorXd r_i);
	Eigen::VectorXd calculateWaves(Eigen::VectorXd r_i, Eigen::VectorXd r_js);
	void applyReconstruction(Eigen::VectorXd & wave_reference,
									 Eigen::VectorXd r_i);

	double gamma_wang(double d);


private:

	Eigen::VectorXd goal;

};