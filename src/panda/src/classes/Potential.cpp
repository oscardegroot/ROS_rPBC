//
// Created by omdegroot on 12-08-19.
//
#include "Potential.h"

Potential::Potential(int l_set, const Eigen::VectorXd& r_star_set)
    : l(l_set), r_star(r_star_set)
{
    
}

Eigen::VectorXd Potential::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js)
{
    PotentialFactors factors = gradient_factors(r_i, r_js);
    
    return factors.RHS_vector + factors.LHS_multiplier * r_js;
}

QuadraticPotential::QuadraticPotential(int l_set, const Eigen::VectorXd& r_star_set)
    : Potential(l_set, r_star_set)
{
        
}

AdvancedPotential::AdvancedPotential(int l_set, const Eigen::VectorXd& r_star_set)
    : Potential(l_set, r_star_set) {

}

PotentialFactors QuadraticPotential::gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){
    return PotentialFactors(r_i + r_star, -1.0);
}

NavigationFunction::NavigationFunction(Agent& agent, int l_set, const Eigen::VectorXd& r_star_set)
    : AdvancedPotential(l_set, r_star_set)
{

    agent.retrieveParameter("NF/alpha", alpha, 16.0);
    
    /* Retrieve goal parameters and set the goal function */
    std::shared_ptr<Goal> goal_ = std::make_shared<WangGoal>(agent);
    addGoal(goal_);
    
    int k = 0;
    std::string obstacle_path = "NF/beta/obstacle_" + std::to_string(k);
    std::string obstacle_type;
    
    while(agent.hasParameter(obstacle_path)){
        
        agent.retrieveParameter(obstacle_path + "/type", obstacle_type);
        
        if(obstacle_type == "lower_bound_x"){
            addObstacle(std::make_shared<LowerBoundObstacle>(agent, k, l, 0));
        }else if(obstacle_type == "lower_bound_y"){
            addObstacle(std::make_shared<LowerBoundObstacle>(agent, k, l, 1));
        }else if(obstacle_type == "lower_bound_z"){
            addObstacle(std::make_shared<LowerBoundObstacle>(agent, k, l, 2));
        }else if(obstacle_type == "obstacle"){
            addObstacle(std::make_shared<ObjectObstacle>(agent, k, l));
        }
        
        k++;
        obstacle_path = "NF/beta/obstacle_" + std::to_string(k);
    }
}


void AdvancedPotential::addGoal(const std::shared_ptr<Goal>& goal_set) {
    goal = std::shared_ptr<Goal>(goal_set);
}

void AdvancedPotential::addObstacle(const std::shared_ptr<Obstacle>& obstacle) {

    obstacles.push_back(obstacle);
}

// Get the multipliers of r_i and r_js for the current gradient based on the selected potential function
PotentialFactors NavigationFunction::gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    // Find obstacle gradient and value
    PotentialFactors beta_gradient = obstacleGradient(r_i, r_js);
    double beta = obstacleValue(r_i, r_js);
    
//    if(beta < 0.99){
//        logTmp("obstacle value: ", beta);
//    }
    // Find goal gradient and value (IFFY)
    double d = helpers::normOf(r_js - r_i - r_star);
    double gamma_gradient = goal->gradient(d);
    double gamma = goal->value(d);

    double denom = alpha * std::pow(std::pow(gamma, alpha) + beta, 1.0/alpha + 1.0);
    
    // Construct potential factors that implement r_i + r* - r_js (is this correct or *= -1?
    PotentialFactors goal_gradient(r_i + r_star, -1.0);

    // Apply the NF gradient (note: operators implemented in PotentialFactors)
    return goal_gradient*1.0/denom*alpha*beta*gamma_gradient - beta_gradient*1.0/denom * gamma;
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
