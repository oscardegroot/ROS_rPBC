/*
File: Edge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#pragma once

#include "Edge.h"
#include "math.h"
#include <stdexcept>
#include "Potential.h"

class EdgeFlex : public Edge{
public:
	// Constructor and destructor
	EdgeFlex(int i, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);

	Eigen::VectorXd sample(Eigen::VectorXd r_i) override;
	Eigen::VectorXd calculateControls(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
	Eigen::VectorXd calculateWaves(const Eigen::VectorXd& tau, const Eigen::VectorXd& r_js) override;

	Eigen::VectorXd fullSTLoop(Eigen::VectorXd& tau, Eigen::VectorXd& r_js,
									Eigen::VectorXd r_i, Eigen::VectorXd s_in);

	// Iteratively solves the algebraic loop for r_js
	void STIterations(Eigen::VectorXd& r_js, Eigen::VectorXd r_i,
					 Eigen::VectorXd s_in);


	Eigen::VectorXd elementSign(Eigen::VectorXd s_in);
	void setScatteringGain(Eigen::MatrixXd gain);
	void lowpassFilter(Eigen::VectorXd& filtered_data,
					 Eigen::VectorXd new_data, double alpha);

	double gradient_gamma(double d);
	double gamma(double d);

	double gradient_G(Eigen::VectorXd r_i, Eigen::VectorXd r_js);
	double G(Eigen::VectorXd r_i, Eigen::VectorXd r_js);

	void initGamma();
	void initG();

private:

    Potential potential;

	// Save of the iterated tau and tau in the last loop
	double agent_i;

	// NF variables
	double alpha;

	double Rw, eps, r;
	double a1, b1, a2, b2, c2, d2;

	double b_z, R_z, delta_z;
	double af, bf, cf, df;

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd matrix_ST;

	Eigen::VectorXd r_js_last;
};