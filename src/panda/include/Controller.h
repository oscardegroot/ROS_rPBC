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
    //realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> pub;
    franka_hw::TriggerRate rate{1.0};
    std::string var_name;
    
    //SignalPublisher() = delete;
    //SignalPublisher(SignalPublisher&& other) = delete;
    
    SignalPublisher(Agent& agent, ros::NodeHandle& nh, double publish_rate, const std::string& name)
    {
        var_name = name;

        pub = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>>
            (nh, "agents/" + std::to_string(agent.getID()) + "/" + var_name, 1);
        
        //pub->init(nh, "agents/" + std::to_string(agent.getID()) + "/" + var_name, 1);
        rate = franka_hw::TriggerRate(publish_rate);
    }
    
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

class Controller{

public:
	Controller(Agent& agent);
	virtual ~Controller();

    // Get the agent output, possibly modified
    virtual
    Eigen::VectorXd getOutput(System& system) = 0;

    // Compute the input (POSSIBLY AT A ROBOTSTATE HERE)
    virtual
    Eigen::VectorXd computeControl(System& system, const Eigen::VectorXd& tau_c) = 0;
    
	/** @brief Local control potential */
    virtual
	Eigen::VectorXd dVsdq(System& system) = 0;

	/** @brief Damping matrix */
	virtual
    Eigen::MatrixXd Kv(System& system);

    void publish(const std::string& name, const Eigen::VectorXd& values);
    
    virtual
    void publishAll(System& system){};

protected:
    int l;
	ros::NodeHandle nh;

    std::vector<SignalPublisher> publishers;

	// Realtime publishing
//	franka_hw::TriggerRate tau_rate{1.0};
//	franka_hw::TriggerRate z_rate{1.0};
//	realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> tau_pub, z_pub, theta_pub;
    
    // Gains
	Eigen::VectorXd Vs_gains, theta_star, limit_avoidance_gains, limits_avg;
	bool local_enabled, limit_avoidance_enabled, gravity_enabled,integral_enabled;
	double kq, kz, ki;
};


#endif