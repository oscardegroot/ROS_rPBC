//
// Created by omdegroot on 15-08-19.
//

/// Stripped down Edge derived class for leader implementation

#include "EdgeLeader.h"

EdgeLeader::EdgeLeader(int i, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set)
    : Edge(-1, -1, gain_set, l_set, r_star_set){
    logTmp(r_star_set);
    logTmp(gain_set);
}

/* Main public function that samples this edge */
Eigen::VectorXd EdgeLeader::sample(Eigen::VectorXd r_i){

    return gain*(r_star - r_i);
}

Eigen::VectorXd EdgeLeader::calculateControls(Eigen::VectorXd r_i, Eigen::VectorXd r_js){

    return Eigen::VectorXd::Zero(l);
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeLeader::calculateWaves(Eigen::VectorXd tau,
                                         Eigen::VectorXd r_js){

    return Eigen::VectorXd::Zero(l);
}
