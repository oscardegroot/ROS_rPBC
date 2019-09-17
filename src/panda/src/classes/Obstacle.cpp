
#include "Obstacle.h"

Obstacle::Obstacle(int l_set)
    : l(l_set)
{

}

// In general to get the value
double Obstacle::value(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js)
{
    // Calculate a distance
    double d = getDistance(r_i, r_js);
    
    // Find the function value corresponding to that distance
    return obstacle_function->value(d);
}

// To get the gradient
PotentialFactors Obstacle::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js)
{
    // Get the distance
    double d = getDistance(r_i, r_js);

    // Calculate the gradient value at that distance
    double gradient_value = obstacle_function->gradient_value(d);
    
    // Return the factors
    return PotentialFactors(gradient_value*Eigen::MatrixXd::Identity(l, l), 0.0);
}


BoundObstacle::BoundObstacle(int l_set, double lower_bound_set, double upper_bound_set, unsigned int dimension_set)
    : Obstacle(l_set), lower_bound(lower_bound_set), upper_bound(upper_bound_set), dimension(dimension_set)
{
    
    obstacle_function = std::make_unique<WangObstacleFunction>("/controller/NF/constraints/lower_bound/");
}


PotentialFactors BoundObstacle::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){

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

    // Apply the obstacle function
    double gradient_value = obstacle_function->gradient_value(d);
    
    /* Incorrect: should be only that dimension! */ 
    if(lower_bound_active){
        return PotentialFactors(gradient_value*selector, 0.0);
    }
    else{
        return PotentialFactors(-gradient_value*selector, 0.0);
    }
}

double BoundObstacle::getDistance(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js)
{
    double d = std::min(r_i(dimension) - lower_bound, upper_bound - r_i(dimension));
    
    if (d < 0){
        throw OperationalException("Bound Obstacle has a negative d (limit was passed!)"
        + std::to_string(d));
    }
    
    return d;
}

/* Obstacle Class */
ObjectObstacle::ObjectObstacle(int l_set, const Eigen::VectorXd& location_set, double radius)
    : Obstacle(l_set)
{
    location = location_set;
    obstacle_function = std::make_unique<WangObstacleFunction>("/controller/NF/constraints/obstacle/", radius);
}

double ObjectObstacle::getDistance(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js)
{
    return helpers::normOf(r_i - location);
}

/* Wang Obstacle Function */
WangObstacleFunction::WangObstacleFunction(std::string retrieval_name, double R_set){
        
    R = R_set;
    
    // Retrieve function parameters    
    ros::NodeHandle nh;
    helpers::safelyRetrieve(nh, retrieval_name + "delta", delta, 0.15);
    
    initParameters();
}

WangObstacleFunction::WangObstacleFunction(std::string retrieval_name)
{
    ros::NodeHandle nh;
    
    // Retrieve function parameters
    helpers::safelyRetrieve(nh, retrieval_name + "R", R, 0.1);
    helpers::safelyRetrieve(nh, retrieval_name + "delta", delta, 0.15);
    
    initParameters();
    
}

void WangObstacleFunction::initParameters(){
    // Initialise parameters
    af = 1/std::pow(delta, 3);
    bf = -(3*(R + delta))/std::pow(delta, 3);
    cf = 3*std::pow(R + delta, 2)/std::pow(delta, 3);
    df = 1 - std::pow(R + delta, 3) / std::pow(delta, 3);
}

double WangObstacleFunction::value(const double& d)
{
    double result = 0.0;

    if (d >= R + delta){
        result = 1.0;
    }else if(d < R){
        result = 0.0;
    }else {
        result = af*std::pow(d, 3) + bf*std::pow(d, 2) + cf*d + df;
    }

    return result;
}

double WangObstacleFunction::gradient_value(const double& d)
{
    double gradient = 0.0;

    if(d >= R + delta || d < R){
        gradient = 0.0;
    }else{
        gradient = (3*af*d*d + 2*bf*d+cf);
    }
    
    return gradient;
}
