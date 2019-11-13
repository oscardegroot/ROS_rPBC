//
// Created by omdegroot on 15-08-19.
//

#ifndef SRC_EDGELEADER_H
#define SRC_EDGELEADER_H

#include "Edge.h"

class EdgeLeader : public Edge{
public:
    // Constructor and destructor
    EdgeLeader(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set);

    Eigen::VectorXd sample(const Eigen::VectorXd& r_i) override;
    Eigen::VectorXd calculateControls(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    Eigen::VectorXd calculateWaves(const Eigen::VectorXd& tau, const Eigen::VectorXd& r_js) override;


    bool isLeader() const override { return true;};


private:

    // Save of the iterated tau and tau in the last loop
    double agent_i;

    // NF variables
    double alpha;

    // The wave impedance B and network gain Kd
    Eigen::MatrixXd matrix_ST;

    Eigen::VectorXd r_js_last;
    std::unique_ptr<AdvancedPotential> potential;
};

#endif //SRC_EDGELEADER_H
