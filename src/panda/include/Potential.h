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


struct ObstacleReturn{
    
    Eigen::MatrixXd i_matrix;
    double js_multiplier;
    
    ObstacleReturn(Eigen::MatrixXd i_matrix_set, double js_multiplier_set)
        : i_matrix(i_matrix_set), js_multiplier(js_multiplier_set)
    {
           // logTmp("Constructor", i_matrix);
            //logTmp("Constructor", js_multiplier);
    }
    
    ObstacleReturn(int l){
        i_matrix = Eigen::MatrixXd::Zero(l, l);
        js_multiplier = 0.0;
    }
    
    ObstacleReturn& operator*=(const double& x) {
        this->i_matrix *= x;
        this->js_multiplier *= x;
        return *this;
        //return ObstacleReturn(i_matrix * x, js_multiplier * x);
    }

    
    ObstacleReturn& operator+=(const ObstacleReturn& x) {
        this->i_matrix += x.i_matrix;
        this->js_multiplier += x.js_multiplier;
        return *this;
    }
};

// Pure virtual goal interface
// Gradient is always multiplied by r_js - r_i (extensions are not feasible)
class Goal{

public:
    Goal(int l_set);
    //virtual ~Goal();

    virtual double value(const double& d) = 0;
    virtual double gradient(const double& d) = 0;

protected:
    int l;
};

// Pure virtual obstacle interface.
// Obstacle gradients need to have separate r_i and r_js contributions, such that they can be integrated in the iterative procedure.
// r_i contribution as vector since it can be constrained in a certain dimension only.
// r_js contribution as double (multiplier) since it can only be applied in full.
class Obstacle{

public:
    Obstacle(int l_set);
    //virtual ~Obstacle();

    virtual double value(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) = 0;
    virtual ObstacleReturn gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) = 0;
    
protected:
    int l;
};



// Navigation function
class Potential {

public:
    Potential(){
        
    }
    
    Potential(double alpha_set, int l_set);
    ~Potential();

    void addGoalFcn(const std::shared_ptr<Goal>& goal_set);
    void addObstacleFcn(const std::shared_ptr<Obstacle>& obstacle);

    ObstacleReturn gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    Eigen::VectorXd gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);

private:
    double alpha;
    int l;
    
    Eigen::MatrixXd ST_matrix, gain;

    std::shared_ptr<Goal> goal;
    std::vector<std::shared_ptr<Obstacle>> obstacles;

    ObstacleReturn obstacleGradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);
    double obstacleValue(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js);

};


// Implementation of a Goal function from Wang et al.
class WangGoal : public Goal{

public:
    WangGoal(int l_set);
    
    double value(const double& d) override;
    double gradient(const double& d) override;

private:
    void initParameters();

	double r_w, eps, r;
	double a1, b1, a2, b2, c2, d2;
    
};


// Bound obstacle in 1D
class BoundObstacle : public Obstacle{
    
public:
    BoundObstacle(int l_set, double lower_bound_set, double upper_bound_set, unsigned int dimension_set);

    double value(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;
    ObstacleReturn gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) override;

private:
    void initParameters();

    double lower_bound, upper_bound;
    unsigned int dimension;

    double b_z, R_z, delta_z;
	double af, bf, cf, df;
    
};


#endif //SRC_POTENTIAL_H
