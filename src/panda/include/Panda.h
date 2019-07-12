/*
File: Panda.h

System file for the real Franka Emika Panda 7DOF robotic manipulator
*/

#pragma once

#include "ros/ros.h"
#include "System.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include <sensor_msgs/JointState.h>
#include "CustomLog.h"
#include <string>

#include "System.h"
#include "Controller.h"
#include "IDAPBC.h"
#include "PandaSim.h"
#include "Panda.h"
#include "CMM.h"

//using namespace std;

namespace panda {


class Panda : public System, public controller_interface::MultiInterfaceController<
					franka_hw::FrankaModelInterface,
					hardware_interface::EffortJointInterface,
					franka_hw::FrankaPoseCartesianInterface>
					{

public:
	Panda();
	~Panda();

	// See definitions in system.h
	// Eigen::VectorXd readSensors();
	// bool sendInput(Eigen::VectorXd tau);
	Eigen::MatrixXd M();
	Eigen::VectorXd dVdq();
	// void readStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	// Eigen::VectorXd getEEPose();
	// Eigen::VectorXd getdVdq();

	bool init (hardware_interface::RobotHW* hw, ros::NodeHandle& nh) override;  // mandatory
	void update (const ros::Time& time, const ros::Duration& period) override;  // mandatory

	bool sendInput(Eigen::VectorXd tau);

	void setState();

	// void starting (const ros::Time& time);   // optional
	// void stopping (const ros::Time& time);  // optional

private:

	// Get a nodehandle
	ros::NodeHandle nh;
	int l;

	// Cartesian handle for the robot state
	std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
	
	// Model handle
	std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

	// Joint effort handle for joint effort control
  	std::vector<hardware_interface::JointHandle> joint_handles_;

	std::unique_ptr<CMM> cmm;
	std::unique_ptr<System> system;
	std::unique_ptr<Controller> controller;

  	inline void errorRetrieving(std::string name, const char* ex_what);

  	template <class T> 
	bool safelyRetrieve(std::string name, T& param);

	template <int N>
	Eigen::VectorXd arrayToVector(std::array<double, N> input);

};
}
