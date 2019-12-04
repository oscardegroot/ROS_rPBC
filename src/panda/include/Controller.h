/*

File: Controller.h

An interface for controllers using IDAPBC or rPBC control.

*/

#ifndef Controller_H
#define Controller_H

#include "ros/ros.h"

#include "System.h"
#include "Agent.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "std_msgs/Float64MultiArray.h"

#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>

#include "Helpers.h"
#include <memory>

/**
 * @class SignalPublisher
 * @author Oscar
 * @date 15/10/19
 * @file Controller.h
 * @brief Class for simplifying publication of signals
 */
struct SignalPublisher{
    
    std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> pub;
    franka_hw::TriggerRate rate{1.0};
    std::string var_name;

    /**
     * @brief At initialisation construct a publisher at agents/#/var_name
     */
    SignalPublisher(Agent& agent, ros::NodeHandle& nh, double publish_rate, const std::string& name)
    {
        var_name = name;

        pub = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>
            (nh, "agents/" + std::to_string(agent.getID()) + "/" + var_name, 1);
        
        rate = franka_hw::TriggerRate(publish_rate);
    }
    
    // Does this publisher publish {name}?
    bool publishes(const std::string& name){
        return name == var_name;
    }
    
    /**
     * @brief Publish values to the topic
     */
    void publish(const Eigen::VectorXd& values){
        
        if (rate() && pub->trylock()) {
            
            pub->msg_.data.resize(values.size());

            for (int i = 0; i < values.size(); i++) {
                pub->msg_.data[i] = values(i);
            }

            pub->unlockAndPublish();
        }
    }

};

/**
 * @class Controller
 * @author Oscar
 * @file Controller.h
 * @brief Controller interface. Computes a control input from a cooperative input.
 */
class Controller{

public:
	Controller(Agent& agent);
	virtual ~Controller();

    /** @brief Get the agent output, possibly modified */
    virtual
    Eigen::VectorXd getOutput(System& system) = 0;

    /** @brief Compute the input */
    virtual
    Eigen::VectorXd computeControl(System& system, const Eigen::VectorXd& tau_c) = 0;
    
	/** @brief Local control potential */
    virtual
	Eigen::VectorXd dVsdq(System& system) = 0;

	/** @brief Damping matrix */
	virtual
    Eigen::MatrixXd Kv(System& system);

    /** @brief Publish system values for debugging */
    void publish(const std::string& name, const Eigen::VectorXd& values);
    
    virtual
    void publishAll(System& system){};

protected:

    int l;
	ros::NodeHandle nh;

    std::vector<SignalPublisher> publishers;

    // Gains
	Eigen::VectorXd Vs_gains, theta_star, limit_avoidance_gains, limits_avg, limits_min, limits_max;
	bool local_enabled, limit_avoidance_enabled, gravity_enabled,integral_enabled;
	double kq, kz, ki;
};


#endif