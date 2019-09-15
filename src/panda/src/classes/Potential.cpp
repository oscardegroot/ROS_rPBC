//
// Created by omdegroot on 12-08-19.
//
// todo Nog veel fixen hier, dit is niet af!
#include "Potential.h"

Goal::Goal(int l_set)
    : l(l_set)
{

}

Obstacle::Obstacle(int l_set)
    : l(l_set)
{

}

Potential::Potential(double alpha_set, int l_set) {
    alpha = alpha_set;
    l = l_set;
}

Potential::~Potential(){
    //delete goal;
}

void Potential::addGoalFcn(const std::shared_ptr<Goal>& goal_set) {
    goal = goal_set;

}

void Potential::addObstacleFcn(const std::shared_ptr<Obstacle>& obstacle) {
    obstacles.push_back(obstacle);
}

// todo: Convert these functions to d instead of r_i, r_js!
ObstacleReturn Potential::gradient_factors(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    double d = helpers::normOf(r_js - r_i);
    
    // Find all individual gradient values
    ObstacleReturn beta_gradient = obstacleGradient(r_i, r_js);
    double beta = obstacleValue(r_i, r_js);
    double gamma_gradient = goal->gradient(d);
    double gamma = goal->value(d);

    double denom = alpha * std::pow(std::pow(gamma, alpha) + beta, 1.0/alpha + 1.0);
    
    // Calculate the factors based on the general NF gradient
    return ObstacleReturn((-alpha*beta*gamma_gradient*Eigen::MatrixXd::Identity(l, l) - gamma*beta_gradient.i_matrix) / denom,
                            (alpha*beta*gamma_gradient - gamma*beta_gradient.js_multiplier) / denom);
}

// Get the actual value of the gradient
Eigen::VectorXd Potential::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    // Get the factors
    ObstacleReturn gradient_f = gradient_factors(r_i, r_js);
    
    // Multiply with r_i, r_js respectively
    return gradient_f.i_matrix*r_i + gradient_f.js_multiplier*r_js;
    
}


// Obtain the total gradient of the obstacle function (a matrix for r_i and double for r_js)
ObstacleReturn Potential::obstacleGradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {
    // The result contains a matrix for i and double for js
    ObstacleReturn result(l);
    
    // Loop through all obstacles
    for(int i = 0; i < obstacles.size(); i++){
        // Initialise the result with the gradient of the current obstacle
        ObstacleReturn sub_result = obstacles[i]->gradient(r_i, r_js);
        //logTmp("sub gradient!", sub_result.i_matrix);

        // Product rule: multiply with all other obstacles value
        for (int j = 0; j < obstacles.size(); j++){
            if (i != j){
                sub_result *= obstacles[j]->value(r_i, r_js);
            }
        }

        result += sub_result;
    }
    //logTmp("Obstacle gradient", result.i_matrix);
    return result;

}

// Obtain the total obstacle value
double Potential::obstacleValue(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js) {

    double result = 1.0;
    for(int i = 0; i < obstacles.size(); i++){

        result *= obstacles[i]->value(r_i, r_js);
    }
    
    //logTmp("Obstacle value", result);

    return result;
}


WangGoal::WangGoal(int l_set)
   : Goal(l_set)
{
    ros::NodeHandle nh;

    // Retrieve goal parameters
	helpers::safelyRetrieve(nh, "/controller/NF/goal/eps", eps);
	helpers::safelyRetrieve(nh, "/controller/NF/goal/Rw", r_w);
    
    initParameters();
}

double WangGoal::gradient(const double& d)
{
    
    double g_gamma = 0.0;

	if(d >= 2*r_w){
        g_gamma = 0.0;
	}else if(d < r){

        g_gamma = 3*a1*d + 2*b1;
	}else{

        g_gamma = (3*a2*d*d+2*b2*d+c2) / d;
	}

	return g_gamma;
}


double WangGoal::value(const double& d)
{
    double v_gamma = 0.0;

    if(d >= 2*r_w){
        v_gamma = 1.0;
    }else if(d < r){

        v_gamma = a1*std::pow(d, 3) + b1*std::pow(d, 2);
    }else{
        v_gamma = a2*std::pow(d, 3) + b2*std::pow(d, 2) + c2*d + d2;
    }

    return v_gamma;
    
}


void WangGoal::initParameters(){
	r = 2*r_w*eps;

	a1 = (4*eps*r_w*r_w + 4*eps*r_w*r - 3*r*r) / (4*r_w*std::pow(r, 3)*(r-2*r_w));
	b1 = (3*r*r-12*eps*r_w*r_w)/(4*r_w*r*r*(r-2*r_w));
	a2 = (4*eps*r_w*r-3*r*r+8*r_w*r-12*eps*r_w*r_w)/(4*r_w*r*std::pow(r-2*r_w, 3));
	b2 = (-12*eps*r_w*r_w*r-24*r_w*r_w*r+3*std::pow(r, 3) + 48*eps*std::pow(r_w, 3))/(4*r_w*r*std::pow(r-2*r_w, 3));
	c2 = (9*r_w*r*r-12*eps*std::pow(r_w, 3) - 3*std::pow(r, 3))/(r*std::pow(r-2*r_w, 3));
	d2 = 1- (r_w*(-4*eps*r_w*r_w*r - 8*r_w*r_w*r + 12*r_w*r*r - 3*std::pow(r, 3)))/(r*std::pow(r - 2*r_w, 3));
}

BoundObstacle::BoundObstacle(int l_set, double lower_bound_set, double upper_bound_set, unsigned int dimension_set)
    : Obstacle(l_set), lower_bound(lower_bound_set), upper_bound(upper_bound_set), dimension(dimension_set)
{
    
    ros::NodeHandle nh;
    
    // Retrieve the lower bound on z
    helpers::safelyRetrieve(nh, "/controller/NF/constraints/lower_bound/R_z", R_z, 0.1);
    helpers::safelyRetrieve(nh, "/controller/NF/constraints/lower_bound/delta_z", delta_z, 0.15);
    
    initParameters();
}


ObstacleReturn BoundObstacle::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){

    // d is distance to the nearest bound
    bool lower_bound_active = true;
    double d = r_i(dimension) - lower_bound;
    
    if(upper_bound - r_i(dimension) < d){
        d = upper_bound - r_i(dimension);
        lower_bound_active = false;
    }

    if (d < 0){
        throw OperationalException("d in G gradient became negative! (value="
        + std::to_string(d));
    }
    
    // Build a selector matrix for this bound
    Eigen::MatrixXd selector = Eigen::MatrixXd::Zero(l, l);
    selector(dimension, dimension) = 1;

    double g_G = 0.0;

    if(d >= R_z + delta_z || d < R_z){
        g_G = 0.0;
    }else{
        g_G = (3*af*d*d + 2*bf*d+cf);//std::abs(r_i(2, 0));
    }

    
    /* Incorrect: should be only that dimension! */ 
    if(lower_bound_active){
        return ObstacleReturn(g_G*selector, 0.0);
    }
    else{
        return ObstacleReturn(-g_G*selector, 0.0);
    }
}

double BoundObstacle::value(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){
    
        // d is distance to the lower bound
    double d = std::min(r_i(dimension) - lower_bound, upper_bound - r_i(dimension));

    if (d < 0){
        throw OperationalException("d in G gradient became negative! (value="
        + std::to_string(d));
    }
    double v_G = 0.0;

    if (d >= R_z + delta_z){
        v_G = 1.0;
    }else if(d < R_z){
        v_G = 0.0;
    }else {
        v_G = af*std::pow(d, 3) + bf*std::pow(d, 2) + cf*d + df;
    }

    return v_G;
}


void BoundObstacle::initParameters(){
    af = 1/std::pow(delta_z, 3);
    bf = -(3*(R_z + delta_z))/std::pow(delta_z, 3);
    cf = 3*std::pow(R_z + delta_z, 2)/std::pow(delta_z, 3);
    df = 1 - std::pow(R_z + delta_z, 3) / std::pow(delta_z, 3);
}