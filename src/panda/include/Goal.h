
#ifndef GOAL_H
#define GOAL_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "CustomLog.h"
#include "Helpers.h"
#include <vector>
#include "ros/ros.h"
#include <cmath>
#include "PotentialFactors.h"
#include "Agent.h"

// Pure virtual goal interface
// Gradient is always multiplied by r_js - r_i (extensions are not feasible)
class Goal{

public:
    Goal();
    //virtual ~Goal();

    virtual double value(const double& d) = 0;
    virtual double gradient(const double& d) = 0;

protected:
};


// Implementation of a Goal function from Wang et al.
class WangGoal : public Goal{

public:
    WangGoal(Agent& agent);
    
    double value(const double& d) override;
    double gradient(const double& d) override;

private:
    void initParameters();

	double r_w, eps, r;
	double a1, b1, a2, b2, c2, d2;
    
};

#endif