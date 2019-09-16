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

// Purely virtual potential class. Generates inputs via the gradient and allows for ST calculations via gradient_factors
class Potential{

public:
    Potential(int l);
        
    virtual PotentialFactors gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) = 0;
    virtual Eigen::VectorXd gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) = 0;
    
protected:
    int l;
};

// Virtual extension of potentials that allow for adding / removing of obstacles and goal functions
class AdvancedPotential : public Potential{

public:
    AdvancedPotential(int l_set);
    virtual void addGoalFcn(const std::shared_ptr<Goal>& goal_set);
    virtual void addObstacleFcn(const std::shared_ptr<Obstacle>& obstacle);
    
protected:
    std::shared_ptr<Goal> goal;
    std::vector<std::shared_ptr<Obstacle>> obstacles;
};

class QuadraticPotential : public Potential{
    
public:
    QuadraticPotential(int l_set);
    
    PotentialFactors gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    Eigen::VectorXd gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    
};

// Navigation function
class NavigationFunction : public AdvancedPotential {

public:
    NavigationFunction(double alpha_set, int l_set);

    PotentialFactors gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    Eigen::VectorXd gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;

private:
    double alpha;
    
    PotentialFactors obstacleGradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    double obstacleValue(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);

};



#endif //SRC_POTENTIAL_H
