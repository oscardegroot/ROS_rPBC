/**
    @file: Pandasim.h

    @brief A system class for control of the panda in simulation using matrices found in Matlab with the given DH parameters
*/

#ifndef PandaSim_H
#define PandaSim_H

#include "ros/ros.h"
#include "System.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "CustomLog.h"
#include <string>

using namespace std;

class PandaSim : public System{

public:

	PandaSim();

	// See definitions in system.h
	bool sendInput(const Eigen::VectorXd& tau) override;

    /** @brief System matrices */
	virtual
    Eigen::MatrixXd& M() override;
    
	virtual
    Eigen::VectorXd& dVdq() override;
    
	virtual
    Eigen::MatrixXd& Psi() override;
    
    virtual
    Eigen::MatrixXd& dM() override;
    
    virtual
    Eigen::MatrixXd& dPsi() override;
    
    virtual
    Eigen::MatrixXd& dMinv() override;
    
    // Transformation from q to z
	Eigen::VectorXd getZ(const Eigen::VectorXd& q);

    // Read states from simulation
	void readStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

private:

	// Get a nodehandle
	ros::NodeHandle nh;
    
	// Define publishers for the input
	ros::Publisher tau_pubs[7];
	ros::Subscriber sensor_sub;

    const double g = 9.81;
    
    Eigen::MatrixXd M_constants;
	
};


#endif