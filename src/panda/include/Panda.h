/*
File: Panda.h

System file for the real Franka Emika Panda 7DOF robotic manipulator
*/

#pragma once

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_control/services.h>
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
#include "Exceptions.h"
#include "Helpers.h"

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

	// Mass and potential
	Eigen::MatrixXd M();
	Eigen::VectorXd dVdq();

	// Ros Control functions
	bool init (hardware_interface::RobotHW* hw, ros::NodeHandle& nh) override;  // mandatory
	void starting(const ros::Time& /*time*/);
	void update (const ros::Time& time, const ros::Duration& period) override;  // mandatory

	// Wrappers for communication
	bool sendInput(Eigen::VectorXd tau);
	void retrieveState(franka::RobotState& robot_state);

	// Safety functions
	void checkSafety() override;
	std::array<double, 7> saturateTorqueRate(
    	const Eigen::VectorXd& torques,
    	const std::array<double, 7>& tau_J_d);

	std::array<double, 7> checkTorque(
		const Eigen::VectorXd& torques,
		const std::array<double, 7>& tau_J_d);

private:

	// Get a nodehandle
	ros::NodeHandle nh;
	ros::ServiceClient connect_client;

	// Bounds for safety
	double z_lower_bound, torque_bound;
	double velocity_element_bound, velocity_norm_bound;
	Eigen::VectorXd last_z;

	// Torque rate limit
	static constexpr double kDeltaTauMax{1.0};

	// Cartesian handle for the robot state
	std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
	
	// Model handle
	std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

	// Joint effort handle for joint effort control
  	std::vector<hardware_interface::JointHandle> joint_handles_;

  	// Necessary for the cartesian handle
  	std::array<double, 16> initial_pose_;

  	// Classes for control
	std::unique_ptr<CMM> cmm;
	std::unique_ptr<Controller> controller;
};
}
