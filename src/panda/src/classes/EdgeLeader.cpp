//
// Created by omdegroot on 15-08-19.
//

/// Stripped down Edge derived class for leader implementation

#include "EdgeLeader.h"

EdgeLeader::EdgeLeader(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set)
    : Edge(agent, -1, gain_set, l_set, r_star_set, 1){
    logTmp(r_star_set);
    logTmp(gain_set);
}

/* Main public function that samples this edge */
Eigen::VectorXd EdgeLeader::sample(const Eigen::VectorXd& r_i){

    return gain*(r_star - r_i);
}

Eigen::VectorXd EdgeLeader::calculateControls(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){

    return Eigen::VectorXd::Zero(l);
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeLeader::calculateWaves(const Eigen::VectorXd& tau,
                                         const Eigen::VectorXd& r_js){

    return Eigen::VectorXd::Zero(l);
}
