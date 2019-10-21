//
// Created by omdegroot on 12-08-19.
//
#include "Potential.h"

Potential::Potential(int l_set, const Eigen::VectorXd& r_star_set)
    : l(l_set), r_star(r_star_set)
{
    
}

QuadraticPotential::QuadraticPotential(int l_set, const Eigen::VectorXd& r_star_set)
    : Potential(l_set, r_star_set)
{
        
}

AdvancedPotential::AdvancedPotential(int l_set, const Eigen::VectorXd& r_star_set)
    : Potential(l_set, r_star_set) {

}

Eigen::VectorXd QuadraticPotential::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){
    return (r_js - r_i - r_star);
}

PotentialFactors QuadraticPotential::gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){
    return PotentialFactors(-Eigen::MatrixXd::Identity(l, l), 1.0, -1.0);
}

NavigationFunction::NavigationFunction(Agent& agent, int l_set, const Eigen::VectorXd& r_star_set)
    : AdvancedPotential(l_set, r_star_set)
{

    agent.retrieveParameter("NF/alpha", alpha, 5.0);
    
    /* Retrieve goal parameters and set the goal function */
    std::shared_ptr<Goal> goal_ = std::make_shared<WangGoal>(agent);
    addGoalFcn(goal_);
    
    int k = 0;
    std::string obstacle_path = "NF/beta/obstacle_" + std::to_string(k);
    std::string obstacle_type;
    
    while(agent.hasParameter(obstacle_path)){
        
        agent.retrieveParameter(obstacle_path + "/type", obstacle_type);
        
        if(obstacle_type == "x_bound"){
            addObstacleFcn(std::make_shared<BoundObstacle>(agent, k, l, 0));
        }else if(obstacle_type == "y_bound"){
            addObstacleFcn(std::make_shared<BoundObstacle>(agent, k, l, 1));
        }else if(obstacle_type == "z_bound"){
            addObstacleFcn(std::make_shared<BoundObstacle>(agent, k, l, 2));
        }else if(obstacle_type == "obstacle"){
            addObstacleFcn(std::make_shared<ObjectObstacle>(agent, k, l));
        }
        
        k++;
        obstacle_path = "NF/beta/obstacle_" + std::to_string(k);
    }
}


void AdvancedPotential::addGoalFcn(const std::shared_ptr<Goal>& goal_set) {
    goal = std::shared_ptr<Goal>(goal_set);
}

void AdvancedPotential::addObstacleFcn(const std::shared_ptr<Obstacle>& obstacle) {
    obstacles.push_back(obstacle);
}

// Get the multipliers of r_i and r_js for the current gradient based on the selected potential function
PotentialFactors NavigationFunction::gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    // Find obstacle gradient and value
    PotentialFactors beta_gradient = obstacleGradient(r_i, r_js);
    double beta = obstacleValue(r_i, r_js);
    
    // Find goal gradient and value
    double d = helpers::normOf(r_js - r_i - r_star);
    double gamma_gradient = goal->gradient(d);
    double gamma = goal->value(d);

    double denom = alpha * std::pow(std::pow(gamma, alpha) + beta, 1.0/alpha + 1.0);
    
    // Calculate the factors based on the general NF gradient
    /** @todo: FIX R_STAR -> POTENTIAL FACTORS NEEDS AN R_STAR TERM!! */
    return PotentialFactors((-alpha*beta*gamma_gradient*Eigen::MatrixXd::Identity(l, l) - gamma*beta_gradient.i_matrix) / denom,
                            (alpha*beta*gamma_gradient - gamma*beta_gradient.js_multiplier) / denom,
                            -(alpha*beta*gamma_gradient)/denom); // Formation is only the goal part and equal to the term of r_i
}

// Get the actual value of the gradient
Eigen::VectorXd NavigationFunction::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    // Get the factors
    PotentialFactors gradient_f = gradient_factors(r_i, r_js);
    
    // Multiply with r_i, r_js respectively
    return gradient_f.i_matrix*r_i + gradient_f.js_multiplier*r_js + gradient_f.formation_multiplier*r_star;
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
