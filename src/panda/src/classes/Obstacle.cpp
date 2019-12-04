
#include "Obstacle.h"

// In general to get the value
double Obstacle::value(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js)
{
    // Points in the right direction (and parameterised to solve for r_js)
    PotentialFactors gradient_vector = gradientVector(r_i);
    
    // Get the distance if we fill in the current r_js
    double d = gradient_vector.length(r_js);
    
    if(d < 0.0){
        logMsg("Obstacle", "Collision with specified obstacle detected! (distance d = " + std::to_string(d) + ")", 0);
    }
    
    // Find the function value corresponding to that distance
    return obstacle_function->value(d);
}

// To get the gradient
PotentialFactors Obstacle::gradient(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js)
{
    // Points in the right direction (and parameterised to solve for r_js)
    PotentialFactors gradient_vector = gradientVector(r_i);
    
    // Get the distance if we fill in the current r_js
    double d = gradient_vector.length(r_js);

    // Calculate the gradient value at that distance
    double gradient_value = obstacle_function->gradient_value(d);
    
    // Return the factors
    return gradient_vector*gradient_value;
}


ObjectObstacle::ObjectObstacle(Agent& agent, int count, int l_set)
    : Obstacle(l_set)
{
    
    Eigen::VectorXd location_temp;
    // Retrieve object parameters
    double radius;
    agent.retrieveEigen("NF/beta/obstacle_" + std::to_string(count) + "/location", location_temp);
    agent.retrieveParameter("NF/beta/obstacle_" + std::to_string(count) + "/radius", radius);
    
    /** @note Does not take into account the coordinate order, hence does not work in general!!! */
    location = Eigen::VectorXd::Zero(l);
    for(size_t i = 0; i < l; i++){
        location(i) = location_temp(i);
    }
    
    // Create an obstacle function
    obstacle_function = std::make_unique<WangObstacleFunction>(agent, radius);
    
    logMsg("Obstacle", "Obstacle created with radius " + std::to_string(radius) + ".", 2);
}

ObjectObstacle::ObjectObstacle(Agent& agent, const Eigen::VectorXd& location_set, double radius, int l_set)
    : Obstacle(l_set)
{
    /** @note Does not take into account the coordinate order, hence does not work in general!!! */
    location = Eigen::VectorXd::Zero(l);
    for(size_t i = 0; i < l; i++){
        location(i) = location_set(i);
    }
    
    obstacle_function = std::make_unique<WangObstacleFunction>(agent, radius);
    logMsg("Obstacle", "Obstacle created with radius " + std::to_string(radius) + "m, " + 
        " location = (" + std::to_string(location_set(0)) + ", " + std::to_string(location_set(1)) + ").", 2);
}

PotentialFactors ObjectObstacle::gradientVector(const Eigen::VectorXd& r_i)
{
    // No r_js only r_i - r_0
    return PotentialFactors(r_i - location, 0.0);
}


LowerBoundObstacle::LowerBoundObstacle(Agent& agent, int count, int l_set, int dimension)
    : Obstacle(l_set)
{
    // Retrieve parameters or set to infinite when not specified
    double temp_bound;
    agent.retrieveParameter("NF/beta/obstacle_" + std::to_string(count) + "/value", temp_bound);
    
    obstacle_function = std::make_unique<WangObstacleFunction>(agent);
    
    selector = Eigen::MatrixXd::Zero(l, l);
    selector(dimension, dimension) = 1.0;
    
    bound = Eigen::VectorXd::Zero(l);
    bound(dimension) = temp_bound;
}

PotentialFactors LowerBoundObstacle::gradientVector(const Eigen::VectorXd& r_i)
{
    return PotentialFactors(selector*(r_i - bound), 0.0);
}

/* Wang Obstacle Function */
WangObstacleFunction::WangObstacleFunction(Agent& agent, double R_set)
    : R(R_set)
{
    // Retrieve function parameters    
    agent.retrieveParameter("NF/beta/function/delta", delta);
    
    initParameters();
}

WangObstacleFunction::WangObstacleFunction(Agent& agent)
{    
    // Retrieve function parameters
    agent.retrieveParameter("NF/beta/function/R", R, 0.1);
    agent.retrieveParameter("NF/beta/function/delta", delta, 0.15);
    
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
    
    return gradient/d;
}
