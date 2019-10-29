
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <memory>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "CustomLog.h"
#include "Helpers.h"
#include <vector>
#include "ros/ros.h"
#include <cmath>
#include <limits>
#include "PotentialFactors.h"
#include "Agent.h"


/**
 * @class ObstacleFunction
 * @author Oscar
 * @date 23/10/19
 * @file Obstacle.h
 * @brief  A purely virtual class for obstacle force functions (distance to value)
 */
class ObstacleFunction{
    
public:
    
    /**
     * @brief Value of the obstacle function (scalar)
     * @param d distance
     * @return scalar value
     */
    virtual double value(const double& d) = 0;
    
    /**
     * @brief Gradient of the obstacle function
     * @param d distance
     * @return scalar gradient value
     */
    virtual double gradient_value(const double& d) = 0;
};

class WangObstacleFunction : public ObstacleFunction{
    
public:
    
    WangObstacleFunction(Agent& agent);
    WangObstacleFunction(Agent& agent, double R_set);

    double value(const double& d) override;
    double gradient_value(const double& d) override;
    
private:
    double R, delta;
	double af, bf, cf, df;
    
    void initParameters();
};

/**
 * @class Obstacle
 * @author Oscar
 * @date 23/10/19
 * @file Obstacle.h
 * @brief Virtual obstacle interface.
 * 
 * Important: Obstacle gradients can have contributions for r_i, r_js, r_jsi* and constant vectors for static objects.
 * This is captured by selector matrices and PotentialFactor structs.
 */
class Obstacle{

public:
    Obstacle(int l_set){
        l = l_set;
    }

    virtual double value(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    virtual PotentialFactors gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    
    //virtual double getDistance(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    
    /**
     * @brief Direction of the gradient, i.e. from r_i to location for an object or from r_js to r_i for collision avoidance, etc.
     * @param r_i Agent output
     * @param r_js Reference variable
     * @return Vector with gradient direction (not normalised!)
     */
    virtual PotentialFactors gradientVector(const Eigen::VectorXd& r_i) = 0;
    
protected:
    int l;
    std::unique_ptr<ObstacleFunction> obstacle_function;

};

// Bound obstacle in 1D
class LowerBoundObstacle : public Obstacle{
    
public:
    LowerBoundObstacle(Agent& agent, int count, int l_set, int dimension_set);
    
    PotentialFactors gradientVector(const Eigen::VectorXd& r_i) override;

private:
    void initParameters();

    Eigen::VectorXd bound;
    Eigen::MatrixXd selector;
    
};

// Literal Obstacle at some position
class ObjectObstacle : public Obstacle{
    
public:
    ObjectObstacle(Agent& agent, int count, int l_set);
    
    // For external instantiation
    ObjectObstacle(Agent& agent, const Eigen::VectorXd& location_set, double radius, int l_set);
    
    //double getDistance(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    PotentialFactors gradientVector(const Eigen::VectorXd& r_i) override;

private:
    void initParameters();

    Eigen::VectorXd location;    
};

#endif
