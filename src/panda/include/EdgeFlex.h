/*
File: Edge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#pragma once

#include "Edge.h"
#include "math.h"
#include <stdexcept>
#include "Potential.h"
#include "Obstacle.h"
#include "Goal.h"

class EdgeFlex : public Edge{
public:
	// Constructor and destructor
	EdgeFlex(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);

	Eigen::VectorXd sample(const Eigen::VectorXd& r_i) override;
	Eigen::VectorXd calculateControls(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
	Eigen::VectorXd calculateWaves(const Eigen::VectorXd& tau, const Eigen::VectorXd& r_js) override;

	Eigen::VectorXd fullSTLoop(Eigen::VectorXd& tau, Eigen::VectorXd& r_js,
									const Eigen::VectorXd& r_i, const Eigen::VectorXd& s_in);

	// Iteratively solves the algebraic loop for r_js
	void STIterations(Eigen::VectorXd& r_js,  const Eigen::VectorXd& r_i, const Eigen::VectorXd&);


	Eigen::VectorXd elementSign(const Eigen::VectorXd& s_in);
	void setScatteringGain(const Eigen::MatrixXd& gain);
//	void lowpassFilter(Eigen::VectorXd& filtered_data,
//					 Eigen::VectorXd new_data, double alpha);

private:

    std::unique_ptr<AdvancedPotential> potential;

    std::unique_ptr<helpers::Counter> retrieval_counter;

	// Save of the iterated tau and tau in the last loop
	double agent_i;

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd matrix_ST;

	Eigen::VectorXd r_js_last;
};