/**
    @file: Panda.h

    @brief System file for the Franka Emika Panda 7DOF robotic manipulator
     * uses franka_ros library
*/

#pragma once

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_control/services.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include "CustomLog.h"
#include <string>
#include <cmath>

#include "System.h"
#include "Controller.h"
#include "IDAPBC.h"
#include "rPBC.h"
#include "PandaSim.h"
#include "Panda.h"
#include "CMM.h"
#include "Exceptions.h"
#include "Helpers.h"

namespace panda {


    class Panda : public System, public controller_interface::MultiInterfaceController<
                        franka_hw::FrankaModelInterface,
                        hardware_interface::EffortJointInterface,
                        franka_hw::FrankaPoseCartesianInterface>
                        {

    public:
        Panda();
        virtual ~Panda();

        /** @brief System matrices */
        Eigen::MatrixXd& M() override;
        Eigen::VectorXd& dVdq() override;
        Eigen::MatrixXd& Psi() override;
        Eigen::MatrixXd& dM() override;
        Eigen::MatrixXd& dPsi() override;
        Eigen::MatrixXd& dMinv() override;
        Eigen::VectorXd& C() override;
        
        /** @brief functions called by ROS Control */
        bool init (hardware_interface::RobotHW* hw, ros::NodeHandle& nh) override;  // mandatory
        void starting(const ros::Time&);
        void update (const ros::Time& time, const ros::Duration& period) override;  // mandatory

        /** Send the input to the robot
         * @param[in] tau vector with inputs */
        virtual bool sendInput(const Eigen::VectorXd& tau);
        
        /** Retrieve system matrices from the robot */
        virtual void retrieveMatrices();
        virtual void retrieveState();

        /** Safety functions 
         * @throws SafetyExceptions */
        void checkSafety() override;
        
        /** Saturate the torque rate to prevent damage to the robot
         * @param[in] torques vector with inputs 
         * @param[in] tau_J_d current inputs from robot 
         * 
         * @returns saturated torques */
        std::array<double, 7> saturateTorqueRate(
            const Eigen::VectorXd& torques,
            const std::array<double, 7>& tau_J_d);

        /** Check if torque is safe
         * @param[in] torques vector with inputs 
         * @param[in] tau_J_d current inputs from robot 
         * 
         * @returns safe torques */
        std::array<double, 7> checkTorque(Eigen::VectorXd& torques, const std::array<double, 7>& tau_J_d);

    protected:

        // ROS classes
        ros::NodeHandle nh;
        ros::ServiceClient connect_client;
        
        ros::Publisher error_recovery;
        
        ros::Time start_time;
        
        // franka classes
        franka::RobotState robot_state;
        
        // Cartesian handle for the robot state
        std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
        std::array<double, 16> initial_pose_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

        // Joint effort handle for joint effort control
        std::vector<hardware_interface::JointHandle> joint_handles_;
        
        // Bounds for safety
        double z_lower_bound, torque_bound, torque_bound_stop;
        double camera_bound_x, camera_bound_z;
        double velocity_element_bound, velocity_norm_bound;
        
        double initial_pause;

        // Filter coefficient (LPF)
        double alpha;
        Eigen::VectorXd dq_filtered;

        Eigen::VectorXd last_z;
        
        // For safety when the geometric coordinates are not selected
        Eigen::VectorXd new_z;

        // Torque rate limit
        static constexpr double kDeltaTauMax{1.0};

        // Pointer to controller
        std::unique_ptr<Controller> controller;
        
        bool initial_matrices = false;
        
        Benchmarker benchmarker;
    };
}
