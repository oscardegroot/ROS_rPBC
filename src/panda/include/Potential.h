//
// Created by omdegroot on 12-08-19.
//

#ifndef SRC_POTENTIAL_H
#define SRC_POTENTIAL_H

#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "CustomLog.h"
#include "Helpers.h"
#include <vector>
#include "ros/ros.h"
#include <cmath>
#include "PotentialFactors.h"
#include "Goal.h"
#include "Obstacle.h"
#include "Agent.h"


// Purely virtual potential class. Generates inputs via the gradient and allows for ST calculations via gradient_factors
class Potential{

public:
    Potential(int l_set, const Eigen::VectorXd& r_star_set);
        
    /**
     * @brief Gradient multipliers of this potential function, for the iterative method
     * @param r_i Agent output
     * @param r_js Networked input (reference variable that is)
     * @return RHS vector and LHS multiplier.
     */
    virtual PotentialFactors gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) = 0;
    
    /**
     * @brief Gradient of this potential function
     * @param r_i Agent output
     * @param r_js Networked input (reference variable that is)
     * @return gradient of the potential, Note: NOT the negative gradient but the gradient itself!
     */
    virtual Eigen::VectorXd gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    
protected:
    int l;
    Eigen::VectorXd r_star;
};

class QuadraticPotential : public Potential{
    
public:
    QuadraticPotential(int l_set, const Eigen::VectorXd& r_star_set);
    
    PotentialFactors gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
//    Eigen::VectorXd gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    
};

// Virtual extension of potentials that allow for adding / removing of obstacles and goal functions
class AdvancedPotential : public Potential{

public:    
    AdvancedPotential(int l_set, const Eigen::VectorXd& r_star_set);
    virtual void addGoal(const std::shared_ptr<Goal>& goal_set);
    virtual void addObstacle(const std::shared_ptr<Obstacle>& obstacle);
    
protected:
    std::shared_ptr<Goal> goal;
    std::vector<std::shared_ptr<Obstacle>> obstacles;
};


// Navigation function
class NavigationFunction : public AdvancedPotential {

public:

    // Automatic retrieval and construction
    NavigationFunction(Agent& agent, int l_set, const Eigen::VectorXd& r_star_set);

    PotentialFactors gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    //Eigen::VectorXd gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    PotentialFactors obstacleGradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    double obstacleValue(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    
private:
    double alpha;
    


};

// Some paths for parameter settingg
#define NF_PATH NF
#define BETA_PATH beta
#define GAMMA_PATH gamma



#endif //SRC_POTENTIAL_H
