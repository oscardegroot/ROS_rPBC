//
// Created by omdegroot on 15-08-19.
//

#ifndef SRC_EDGELEADER_H
#define SRC_EDGELEADER_H

#include "Edge.h"

class EdgeLeader : public Edge{
public:
    // Constructor and destructor
    EdgeLeader(int i, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set);

    Eigen::VectorXd sample(Eigen::VectorXd r_i) override;
    Eigen::VectorXd calculateControls(Eigen::VectorXd r_i, Eigen::VectorXd r_js) override;
    Eigen::VectorXd calculateWaves(Eigen::VectorXd tau, Eigen::VectorXd r_js) override;

    double gradient_gamma(double d);
    double gamma(double d);

    double gradient_G(Eigen::VectorXd r_i, Eigen::VectorXd r_js);
    double G(Eigen::VectorXd r_i, Eigen::VectorXd r_js);

    void initGamma();
    void initG();

private:

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

#endif //SRC_EDGELEADER_H
