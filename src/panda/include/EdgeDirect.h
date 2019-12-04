#pragma once

#include "Edge.h"

/**
 * @class EdgeDirect
 * @author Oscar
 * @file EdgeDirect.h
 * @brief Class with direct communication, without the ST
 */
class EdgeDirect : public Edge{
public:
	// Constructor and destructor
	EdgeDirect(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);

	void applyReconstruction(Eigen::VectorXd & r_js, const Eigen::VectorXd& r_i) override;

    Eigen::VectorXd calculateControls(const Eigen::VectorXd& r_js, const Eigen::VectorXd& r_i) override;
	Eigen::VectorXd calculateWaves(const Eigen::VectorXd& s_in,	const Eigen::VectorXd& r_i) override;


private:
    std::unique_ptr<helpers::Counter> retrieval_counter;

    std::unique_ptr<AdvancedPotential> potential;

};