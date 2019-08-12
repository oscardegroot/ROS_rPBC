//
// Created by omdegroot on 12-08-19.
//
// todo Nog veel fixen hier, dit is niet af!
#include "../../include/Potential.h"

Potential::Potential(double alpha_set) {
    alpha = alpha_set;

}

void Potential::addGoalFcn(Goal new_goal) {
    goal = new_goal;

}

void Potential::addObstacleFcn(Obstacle obstacle) {
    obstacles.push_back(obstacle);
}

// todo: Convert these functions to d instead of r_i, r_js!
Eigen::VectorXd Potential::gradient(Eigen::VectorXd r_i, Eigen::VectorXd r_js) {


    double beta_gradient = obstacleGradient(r_i, r_js);
    double beta = obstacleValue(r_i, r_js);
    double gamma_gradient = goal.gradient(r_i, r_js);
    double gamma = goal.value(r_i, r_js);
    // todo: Rewrite from here!
    denom = alpha*std::pow(std::pow(gamma, alpha) + cur_G,  1 + 1 / alpha);

    for(int i = 0; i < l; i++){

        // r_i brings dimensionality, rest stays scalar
        r_js(i, 0) = (matrix_ST(i, i)*s_in(i, 0)
                      - matrix_ST(i, l+i)*gain(i,i)*
                        (alpha*cur_G*cur_ggamma*r_i(i, 0) - cur_gamma*cur_gG*z_i(i, 0))/denom)
                     /(1 - (matrix_ST(i, l+i)*gain(i,i)*alpha*cur_G*cur_ggamma)/denom);
    }

}

double Potential::obstacleGradient(Eigen::VectorXd r_i, Eigen::VectorXd r_js) {

    double result = 0.0;
    double d = normOf(r_i - r_js);
    for(int i = 0; i < obstacles.size(); i++){

        double sub_result += obstacles[i].gradient(d);

        for (int j = 0; j < obstacles.size(); j++){
            if (i != j){
                sub_result *= obstacles[j].value(d);
            }
        }

        result += sub_result;

    }

    return result;

}

double Potential::obstacleValue(Eigen::VectorXd r_i, Eigen::VectorXd r_js) {

    double result = 1.0;
    double d = normOf(r_i - r_js);
    for(int i = 0; i < obstacles.size(); i++){

        result *= obstacles[i].value(d);

    }

    return result;

}