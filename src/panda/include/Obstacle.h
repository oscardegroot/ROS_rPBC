
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
#include "PotentialFactors.h"

/* Obstacle Functions */
// A purely virtual class for obstacle force functions (since they are often shared over obstacles with different parameters)
class ObstacleFunction{
    
public:
    
    virtual double value(const double& d) = 0;
    virtual double gradient_value(const double& d) = 0;
};

class WangObstacleFunction : public ObstacleFunction{
    
public:
    
    WangObstacleFunction(std::string retrieval_name);
    WangObstacleFunction(std::string retrieval_name, double R_set);

    double value(const double& d) override;
    double gradient_value(const double& d) override;
    
private:
    double R, delta;
	double af, bf, cf, df;
    
    void initParameters();
};

/* Obstacles */
// Virtual obstacle interface.
// Obstacle gradients need to have separate r_i and r_js contributions, such that they can be integrated in the iterative procedure.
// r_i contribution as vector since it can be constrained in a certain dimension only.
// r_js contribution as double (multiplier) since it can only be applied in full.
class Obstacle{

public:
    Obstacle(int l_set);

    virtual double value(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    virtual PotentialFactors gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    
    virtual double getDistance(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) = 0;
    
protected:
    int l;
    std::unique_ptr<ObstacleFunction> obstacle_function;

};

// Bound obstacle in 1D
class BoundObstacle : public Obstacle{
    
public:
    BoundObstacle(int l_set, double lower_bound_set, double upper_bound_set, unsigned int dimension_set);

    PotentialFactors gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    
    double getDistance(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;

private:
    void initParameters();

    double lower_bound, upper_bound;
    unsigned int dimension;
    
};

// Literal Obstacle at some position
class ObjectObstacle : public Obstacle{
    
public:
    ObjectObstacle(int l_set, const Eigen::VectorXd& location_set, double radius);
    
    double getDistance(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;

private:
    void initParameters();

    Eigen::VectorXd location;    
};

#endif
