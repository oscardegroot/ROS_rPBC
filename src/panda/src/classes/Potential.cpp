//
// Created by omdegroot on 12-08-19.
//
#include "Potential.h"

Potential::Potential(int l_set){
    l = l_set;
}

QuadraticPotential::QuadraticPotential(int l_set)
    : Potential(l_set)
{
        
}

AdvancedPotential::AdvancedPotential(int l_set)
    : Potential(l_set) {

}

Eigen::VectorXd QuadraticPotential::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){
    return (r_js - r_i);
}

PotentialFactors QuadraticPotential::gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){
    return PotentialFactors(-Eigen::MatrixXd::Identity(l, l), 1.0);
}


NavigationFunction::NavigationFunction(double alpha_set, int l_set)
    : AdvancedPotential(l_set)
{
    alpha = alpha_set;
}

void AdvancedPotential::addGoalFcn(const std::shared_ptr<Goal>& goal_set) {
    goal = goal_set;

}

void AdvancedPotential::addObstacleFcn(const std::shared_ptr<Obstacle>& obstacle) {
    obstacles.push_back(obstacle);
}

// Get the multipliers of r_i and r_js for the current gradient based on the selected potential function
PotentialFactors NavigationFunction::gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    double d = helpers::normOf(r_js - r_i);
    
    // Find all individual gradient values
    PotentialFactors beta_gradient = obstacleGradient(r_i, r_js);
    double beta = obstacleValue(r_i, r_js);
    double gamma_gradient = goal->gradient(d);
    double gamma = goal->value(d);

    double denom = alpha * std::pow(std::pow(gamma, alpha) + beta, 1.0/alpha + 1.0);
    
    // Calculate the factors based on the general NF gradient
    return PotentialFactors((-alpha*beta*gamma_gradient*Eigen::MatrixXd::Identity(l, l) - gamma*beta_gradient.i_matrix) / denom,
                            (alpha*beta*gamma_gradient - gamma*beta_gradient.js_multiplier) / denom);
}

// Get the actual value of the gradient
Eigen::VectorXd NavigationFunction::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    // Get the factors
    PotentialFactors gradient_f = gradient_factors(r_i, r_js);
    
    // Multiply with r_i, r_js respectively
    return gradient_f.i_matrix*r_i + gradient_f.js_multiplier*r_js;
    
}


// Obtain the total gradient of the obstacle function (a matrix for r_i and double for r_js)
PotentialFactors NavigationFunction::obstacleGradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {
    
    // The result contains a matrix for i and double for js
    PotentialFactors result(l);
    
    // Loop through all obstacles
    for(int i = 0; i < obstacles.size(); i++){
        
        // Initialise the result with the gradient of the current obstacle
        PotentialFactors sub_result = obstacles[i]->gradient(r_i, r_js);

        // Product rule: multiply with all other obstacles value
        for (int j = 0; j < obstacles.size(); j++){
            if (i != j){
                sub_result *= obstacles[j]->value(r_i, r_js);
            }
        }

        result += sub_result;
    }

    return result;

}

// Obtain the total obstacle value
double NavigationFunction::obstacleValue(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    double result = 1.0;
    for(int i = 0; i < obstacles.size(); i++){

        result *= obstacles[i]->value(r_i, r_js);
    }
    
    return result;
}
