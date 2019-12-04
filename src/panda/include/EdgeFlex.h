#pragma once

#include "Edge.h"
#include "math.h"
#include <stdexcept>
#include "Potential.h"
#include "Obstacle.h"
#include "Goal.h"
#include "std_msgs/Float64.h"

/**
 * @class EdgeFlex
 * @author Oscar
 * @file EdgeFlex.h
 * @brief Main class for communication, implements the ST, WVM and Advanced Potentials.
 */
class EdgeFlex : public Edge{
public:

	// Constructor and destructor
	EdgeFlex(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);

    // Override sampling, control and waves
	Eigen::VectorXd sample(const Eigen::VectorXd& r_i) override;
	Eigen::VectorXd calculateControls(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
	Eigen::VectorXd calculateWaves(const Eigen::VectorXd& tau, const Eigen::VectorXd& r_js) override;

    // Compute all ST variables based on the iterative solution to the algebraic loop
	Eigen::VectorXd fullSTLoop(Eigen::VectorXd& tau, Eigen::VectorXd& r_js,
									const Eigen::VectorXd& r_i, const Eigen::VectorXd& s_in);

	// Iteratively solves the algebraic loop for r_js
	void STIterations(Eigen::VectorXd& r_js,  const Eigen::VectorXd& r_i, const Eigen::VectorXd&);

    // Helper function for taking the sign of a vector componentwise
	Eigen::VectorXd elementSign(const Eigen::VectorXd& s_in);
    
    // Function for initialising the scattering gain
	void setScatteringGain(const Eigen::MatrixXd& gain);

    // Add an obstacle to this edge
    void addObstacle(const std::shared_ptr<Obstacle>& obstacle) override;
    
private:

    std::unique_ptr<AdvancedPotential> potential;

    std::unique_ptr<helpers::Counter> retrieval_counter;

	// Save of the iterated tau and tau in the last loop
	double agent_i;
    Eigen::VectorXd impedance;
    
	// The wave impedance B and network gain Kd
	Eigen::MatrixXd matrix_ST;

	Eigen::VectorXd r_js_last;

};