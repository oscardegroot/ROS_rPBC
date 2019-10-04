
#include "PandaSim.h"

/** Defined here because maybe temporary */
//#include <controller_interface/multi_interface_controller.h>
//#include <hardware_interface/joint_command_interface.h>
//#include <hardware_interface/robot_hw.h>
//#include <realtime_tools/realtime_publisher.h>
//#include <ros/node_handle.h>
//#include <ros/time.h>
//
//#include <franka_example_controllers/JointTorqueComparison.h>
//#include <franka_control/services.h>
//#include <franka_hw/franka_cartesian_command_interface.h>
//#include <franka_hw/franka_model_interface.h>
//#include <franka_hw/trigger_rate.h>
//#include <franka_hw/franka_hw.h>
//#include <array>
//See: https://github.com/frankaemika/franka_ros/blob/kinetic-devel/franka_example_controllers/include/franka_example_controllers/joint_impedance_example_controller.h
//using std;

PandaSim::PandaSim()
	:System(7, 7, 3, "pandasim")
{
	logMsg("PandaSim", "Initialising...", 2);
    
	this->setState(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), getZ(Eigen::VectorXd::Zero(n)));
    
	for(int i = 0; i < n; i++){
		tau_pubs[i] = nh.advertise<std_msgs::Float64>("/robot1/panda_joint" +
                    to_string(i+1) + "_controller/command", 100);
	}
	
    sensor_sub = nh.subscribe("/robot1/joint_states", 100, &PandaSim::readStateCallback, this);
    
    psi_updated = false;
    m_updated = false;
    dvdq_updated = false;
    dpsi_updated = false;
    dm_updated = false;
    dminv_updated = false;
    
    M_constants = Eigen::MatrixXd::Zero(n*6, n*6);
    M_constants(0, 0) = 2.345;
    M_constants(1, 1) = 2.345;
    M_constants(2, 2) = 2.345;
    M_constants(3, 3) = 0.01418;
    M_constants(4, 4) = 0.01181;
    M_constants(4, 5) = 0.00236;
    M_constants(5, 4) = 0.00236;
    M_constants(5, 5) = 0.005515;
    M_constants(6, 6) = 2.364;
    M_constants(7, 7) = 2.364;
    M_constants(8, 8) = 2.364;
    M_constants(9, 9) = 0.014;
    M_constants(9, 10) = 0.0055;
    M_constants(9, 11) = - 0.0023;
    M_constants(10, 9) = 0.0055;
    M_constants(10, 11) = 0.01259;
    M_constants(11, 9) = - 0.0023;
    M_constants(11, 10) = 0.01259;
    M_constants(12, 12) = 2.38;
    M_constants(13, 13) = 2.38;
    M_constants(14, 14) = 2.38;
    M_constants(15, 15) = 0.00716;
    M_constants(15, 16) = 0.0092;
    M_constants(15, 17) = - 0.00102;
    M_constants(16, 15) = 0.0092;
    M_constants(16, 16) = - 0.00102;
    M_constants(16, 17) = 0.007159;
    M_constants(17, 15) = - 0.00102;
    M_constants(17, 16) = 0.007159;
    M_constants(17, 17) = - 0.204;
    M_constants(18, 18) = 2.428;
    M_constants(19, 19) = 2.428;
    M_constants(20, 20) = 2.428;
    M_constants(21, 21) = 0.008164;
    M_constants(21, 22) = 0.008165;
    M_constants(21, 23) = 0.00102;
    M_constants(22, 21) = 0.008165;
    M_constants(22, 22) = 0.003061;
    M_constants(22, 23) = 0.009185;
    M_constants(23, 21) = 0.00102;
    M_constants(23, 22) = 0.009185;
    M_constants(23, 23) = 0.00102;
    M_constants(24, 24) = 3.496;
    M_constants(25, 25) = 3.496;
    M_constants(26, 26) = 3.496;
    M_constants(27, 27) = 0.0307;
    M_constants(27, 28) = 0.02761;
    M_constants(27, 29) = - 0.0613;
    M_constants(28, 27) = 0.02761;
    M_constants(28, 29) = 0.00818;
    M_constants(29, 27) = - 0.0613;
    M_constants(29, 28) = 0.00818;
    M_constants(30, 30) = 1.467;
    M_constants(31, 31) = 1.467;
    M_constants(32, 32) = 1.467;
    M_constants(33, 33) = 0.00205;
    M_constants(33, 34) = 0.004089;
    M_constants(34, 33) = 0.004089;
    M_constants(34, 35) = 0.0049;
    M_constants(35, 34) = 0.0049;
    M_constants(36, 36) = 0.4561;
    M_constants(37, 37) = 0.4561;
    M_constants(38, 38) = 0.4561;
    M_constants(39, 39) = 0.3;
    M_constants(39, 40) = 0.3;
    M_constants(40, 39) = 0.3;
    M_constants(40, 41) = 0.3;
    M_constants(41, 40) = 0.3;
    
    /* Getting around the mass matrix issue... */
    /*ros::NodeHandle nh("panda");
    ros::NodeHandle nh_panda("robot1");

    vector<string> joint_names_v;
	string arm_id;
    helpers::safelyRetrieveArray(nh, "joint_names", joint_names_v, 7);
    //std::string* joint_names = &joint_names_v[0];
    array<string, 7> joint_names;
    copy_n(joint_names_v.begin(), 7, joint_names.begin());

	helpers::safelyRetrieve(nh, "arm_id", arm_id);
    unique_ptr<hardware_interface::RobotHW> hw = make_unique<franka_hw::FrankaHW>(joint_names, arm_id, nh_panda);
    
    // Initialise the model interface 
	franka_hw::FrankaModelInterface* model_interface = hw->get<franka_hw::FrankaModelInterface>();

	if (model_interface == nullptr) {
		throw OperationalException("Error getting model interface from hardware");
	}*/ /** @result gives nullptr for the model interface, probably because we dont extend the correct interfaces... */


//    psi = Eigen::MatrixXd::Zero(n, 3);
//    dpsi = Eigen::MatrixXd::Zero(n, 3);
//
//    m_m = Eigen::MatrixXd::Zero(n, n);
//    dm = Eigen::MatrixXd::Zero(n, n);
//    dvdq = Eigen::VectorXd::Zero(n);

    m_previous = M();
    psi_previous = Psi();

    logMsg("PandaSim", "Done!", 2);

}

// Callback for reading the states
void PandaSim::readStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
	
	Eigen::Matrix<double, 7, 1> new_q, new_qd;

	for( int i = 0; i < 7; i++ ) {
      new_q(i) = msg->position[i];
      new_qd(i) = msg->velocity[i];
    }	

    this->setState(new_q, new_qd, getZ(new_q));
    
    psi_updated = false;
    m_updated = false;
    dvdq_updated = false;
    dpsi_updated = false;
    dm_updated = false;
    dminv_updated = false;
}


bool PandaSim::sendInput(const Eigen::VectorXd & tau){
	
	std_msgs::Float64 msg;

	for(int i = 0; i < n; i++){
		msg.data = tau(i);
		tau_pubs[i].publish(msg);
	}

	return true;
}


Eigen::VectorXd& PandaSim::dVdq(){

        Eigen::VectorXd& q = state.q;
    
    if(!dvdq_updated){
        dvdq[0] = 0;
        dvdq[1] = 0.000301499*cos(q(1)) - 34.2264*sin(q(1)) - 7.38648*cos(q(1))*cos(q(2)) - 0.00645473*cos(q(1))*sin(q(2)) - 17.5852*cos(q(3))*sin(q(1)) + 5.3042*sin(q(1))*sin(q(3)) + 1.22861*sin(q(1))*sin(q(3))*sin(q(4)) + 5.3042*cos(q(1))*cos(q(2))*cos(q(3)) + 17.5852*cos(q(1))*cos(q(2))*sin(q(3)) + 1.22861*cos(q(1))*cos(q(4))*sin(q(2)) - 0.0997992*cos(q(3))*cos(q(5))*sin(q(1)) - 0.02195*cos(q(1))*sin(q(2))*sin(q(4)) + 0.02195*cos(q(4))*sin(q(1))*sin(q(3)) - 1.12787*cos(q(3))*sin(q(1))*sin(q(5)) + 0.02195*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) + 1.22861*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4)) + 0.0997992*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 1.12787*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 1.12787*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 1.12787*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.0997992*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.0997992*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) - 1.12787*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.0997992*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
        dvdq[2] = 7.38648*sin(q(1))*sin(q(2)) - 0.00645473*cos(q(2))*sin(q(1)) - 17.5852*sin(q(1))*sin(q(2))*sin(q(3)) + 1.22861*cos(q(2))*cos(q(4))*sin(q(1)) - 5.3042*cos(q(3))*sin(q(1))*sin(q(2)) - 0.02195*cos(q(2))*sin(q(1))*sin(q(4)) - 0.02195*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2)) + 1.12787*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(4)) - 1.22861*cos(q(3))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.0997992*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(3)) - 0.0997992*cos(q(2))*sin(q(1))*sin(q(4))*sin(q(5)) - 1.12787*sin(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) + 1.12787*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.0997992*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5));
        dvdq[3] = 17.5852*cos(q(2))*cos(q(3))*sin(q(1)) - 17.5852*cos(q(1))*sin(q(3)) - 0.02195*cos(q(1))*cos(q(3))*cos(q(4)) - 5.3042*cos(q(1))*cos(q(3)) - 1.22861*cos(q(1))*cos(q(3))*sin(q(4)) - 0.0997992*cos(q(1))*cos(q(5))*sin(q(3)) - 5.3042*cos(q(2))*sin(q(1))*sin(q(3)) - 1.12787*cos(q(1))*sin(q(3))*sin(q(5)) + 1.12787*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.0997992*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1)) - 0.0997992*cos(q(1))*cos(q(3))*cos(q(4))*sin(q(5)) - 0.02195*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) + 1.12787*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) - 1.22861*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4)) + 1.12787*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.0997992*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5));
        dvdq[4] = 0.02195*cos(q(1))*sin(q(3))*sin(q(4)) - 1.22861*cos(q(1))*cos(q(4))*sin(q(3)) - 0.02195*cos(q(4))*sin(q(1))*sin(q(2)) - 1.22861*sin(q(1))*sin(q(2))*sin(q(4)) + 1.22861*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1)) - 0.02195*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)) + 1.12787*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 1.12787*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) - 0.0997992*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5)) + 0.0997992*cos(q(1))*sin(q(3))*sin(q(4))*sin(q(5)) + 1.12787*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.0997992*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(5));
        dvdq[5] = 1.12787*cos(q(1))*cos(q(3))*cos(q(5)) - 0.0997992*cos(q(1))*cos(q(3))*sin(q(5)) - 0.0997992*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 1.12787*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 1.12787*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.0997992*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.0997992*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 1.12787*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.0997992*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 1.12787*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
        dvdq[6] = 0;
//        dvdq[0] = 0;
//        dvdq[1] = 0.00012753*cos(q(1)) - 13.4638*sin(q(1)) - 2.86303*cos(q(1))*cos(q(2)) + 0.00207972*cos(q(1))*sin(q(2)) - 6.84302*cos(q(3))*sin(q(1)) + 1.99683*sin(q(1))*sin(q(3)) + 0.316304*sin(q(1))*sin(q(3))*sin(q(4)) + 1.99683*cos(q(1))*cos(q(2))*cos(q(3)) + 6.84302*cos(q(1))*cos(q(2))*sin(q(3)) + 0.316304*cos(q(1))*cos(q(4))*sin(q(2)) - 0.0680127*cos(q(3))*cos(q(5))*sin(q(1)) - 0.0062784*cos(q(1))*sin(q(2))*sin(q(4)) + 0.0062784*cos(q(4))*sin(q(1))*sin(q(3)) - 0.50033*cos(q(3))*sin(q(1))*sin(q(5)) + 0.0062784*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) + 0.316304*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4)) + 0.0680127*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 0.50033*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 0.50033*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.50033*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.0680127*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.0680127*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.50033*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.0680127*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
//        dvdq[2] = 0.00207972*cos(q(2))*sin(q(1)) + 2.86303*sin(q(1))*sin(q(2)) - 6.84302*sin(q(1))*sin(q(2))*sin(q(3)) + 0.316304*cos(q(2))*cos(q(4))*sin(q(1)) - 1.99683*cos(q(3))*sin(q(1))*sin(q(2)) - 0.0062784*cos(q(2))*sin(q(1))*sin(q(4)) - 0.0062784*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2)) + 0.50033*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.316304*cos(q(3))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.0680127*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(3)) - 0.0680127*cos(q(2))*sin(q(1))*sin(q(4))*sin(q(5)) - 0.50033*sin(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.50033*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.0680127*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5));
//        dvdq[3] = 6.84302*cos(q(2))*cos(q(3))*sin(q(1)) - 6.84302*cos(q(1))*sin(q(3)) - 0.0062784*cos(q(1))*cos(q(3))*cos(q(4)) - 1.99683*cos(q(1))*cos(q(3)) - 0.316304*cos(q(1))*cos(q(3))*sin(q(4)) - 0.0680127*cos(q(1))*cos(q(5))*sin(q(3)) - 1.99683*cos(q(2))*sin(q(1))*sin(q(3)) - 0.50033*cos(q(1))*sin(q(3))*sin(q(5)) + 0.50033*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.0680127*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1)) - 0.0680127*cos(q(1))*cos(q(3))*cos(q(4))*sin(q(5)) - 0.0062784*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) + 0.50033*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.316304*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4)) + 0.50033*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.0680127*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5));
//        dvdq[4] = 0.0062784*cos(q(1))*sin(q(3))*sin(q(4)) - 0.316304*cos(q(1))*cos(q(4))*sin(q(3)) - 0.0062784*cos(q(4))*sin(q(1))*sin(q(2)) - 0.316304*sin(q(1))*sin(q(2))*sin(q(4)) + 0.316304*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1)) - 0.0062784*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)) + 0.50033*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.50033*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) - 0.0680127*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5)) + 0.0680127*cos(q(1))*sin(q(3))*sin(q(4))*sin(q(5)) + 0.50033*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.0680127*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(5));
//        dvdq[5] = 0.50033*cos(q(1))*cos(q(3))*cos(q(5)) - 0.0680127*cos(q(1))*cos(q(3))*sin(q(5)) - 0.0680127*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.50033*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.50033*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.0680127*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.0680127*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.50033*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.0680127*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.50033*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
//        
        dvdq_updated = true;
    }
    
    return dvdq;  
}

Eigen::VectorXd PandaSim::getZ(const Eigen::VectorXd& q)
{
	Eigen::Vector3d z;

	// z as calculated in Matlab
	z[0] = 0.316*cos(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(2)) + 0.384*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.384*cos(q(0))*cos(q(3))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.088*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
	z[1] = 0.0825*cos(q(0))*sin(q(2)) + 0.316*sin(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(3))*sin(q(2)) + 0.384*cos(q(3))*sin(q(0))*sin(q(1)) - 0.384*cos(q(0))*sin(q(2))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.088*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4)) - 0.384*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) + 0.088*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5)) - 0.088*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.088*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) + 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) + 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0));
	z[2] = 0.316*cos(q(1)) + 0.384*cos(q(1))*cos(q(3)) - 0.0825*cos(q(2))*sin(q(1)) - 0.0825*cos(q(1))*sin(q(3)) + 0.0825*cos(q(2))*cos(q(3))*sin(q(1)) + 0.088*cos(q(1))*cos(q(3))*sin(q(5)) + 0.384*cos(q(2))*sin(q(1))*sin(q(3)) + 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.088*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.333;
	return z;
}

// Define dzdq
Eigen::MatrixXd& PandaSim::Psi(){
    
    //RunCheck check("Psi");

    if(!psi_updated){
 
        Eigen::VectorXd& q = state.q;
    
        // Save the old variable for finite difference approximation
        psi_previous = psi;
          
        psi(0, 0) = 0.0825*sin(q(0))*sin(q(1))*sin(q(3)) - 0.316*sin(q(0))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(2)) - 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) + 0.0825*cos(q(0))*cos(q(3))*sin(q(2)) - 0.384*cos(q(3))*sin(q(0))*sin(q(1)) + 0.384*cos(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) - 0.088*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4)) + 0.384*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.088*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5)) + 0.088*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) + 0.088*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0));
        psi(0, 1) = 0.316*cos(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(2)) + 0.384*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.384*cos(q(0))*cos(q(3))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.088*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
        psi(0, 2) = 0;
        psi(1, 0) = 0.316*cos(q(0))*cos(q(1)) + 0.384*cos(q(0))*cos(q(1))*cos(q(3)) - 0.0825*cos(q(0))*cos(q(2))*sin(q(1)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(1)) + 0.088*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(5)) + 0.384*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3)) + 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.088*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1));
        psi(1, 1) = 0.316*cos(q(1))*sin(q(0)) + 0.384*cos(q(1))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(2))*sin(q(0))*sin(q(1)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(1)) + 0.088*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.384*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3)) + 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1));
        psi(1, 2) = 0.0825*sin(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2)) - 0.384*cos(q(3))*sin(q(1)) - 0.316*sin(q(1)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3)) + 0.384*cos(q(1))*cos(q(2))*sin(q(3)) - 0.088*cos(q(3))*sin(q(1))*sin(q(5)) + 0.088*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
        psi(2, 0) = 0.0825*cos(q(2))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(2)) - 0.0825*cos(q(2))*sin(q(0)) + 0.384*cos(q(2))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2)) + 0.384*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0)) + 0.088*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2));
        psi(2, 1) = 0.0825*cos(q(0))*cos(q(2)) - 0.0825*cos(q(0))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(2))*sin(q(3)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(2)) + 0.0825*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.384*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2));
        psi(2, 2) = 0.0825*sin(q(1))*sin(q(2)) - 0.384*sin(q(1))*sin(q(2))*sin(q(3)) - 0.0825*cos(q(3))*sin(q(1))*sin(q(2)) + 0.088*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.088*sin(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2));
        psi(3, 0) = 0.384*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(3))*sin(q(1)) - 0.384*cos(q(0))*sin(q(1))*sin(q(3)) - 0.0825*sin(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) - 0.088*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(3));
        psi(3, 1) = 0.0825*cos(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(3))*sin(q(2)) - 0.0825*cos(q(3))*sin(q(0))*sin(q(1)) - 0.384*sin(q(0))*sin(q(1))*sin(q(3)) - 0.384*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.0825*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(5)) - 0.088*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3));
        psi(3, 2) = 0.384*cos(q(2))*cos(q(3))*sin(q(1)) - 0.384*cos(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(3)) - 0.0825*cos(q(2))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.088*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) + 0.088*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3));
        psi(4, 0) = 0.088*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.088*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(4));
        psi(4, 1) = 0.088*cos(q(0))*cos(q(2))*cos(q(4))*cos(q(5)) - 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.088*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(4));
        psi(4, 2) = 0.088*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.088*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) + 0.088*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4));
        psi(5, 0) = 0.088*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(1)) + 0.088*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(4))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 0.088*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
        psi(5, 1) = 0.088*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.088*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5)) + 0.088*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(5));
        psi(5, 2) = 0.088*cos(q(1))*cos(q(3))*cos(q(5)) + 0.088*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.088*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.088*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
        
        psi_updated = true;
    }
    return psi;    
}

Eigen::MatrixXd& PandaSim::dMinv(){
    if(!dminv_updated)
    {
        Eigen::MatrixXd dm_temp = -M().inverse()*dM()*M().inverse();
        helpers::lowpassFilter(dminv, dm_temp, 1.0);
        dminv_updated = true;
    }
    
    return dminv;
}

Eigen::MatrixXd& PandaSim::dM(){

    if(!dm_updated){
        helpers::lowpassFilter(dm, this->approximateDerivative(M(), m_previous), 1.0);
        dm_updated = true;
    }

    return dm;
}

Eigen::MatrixXd& PandaSim::dPsi(){
    //RunCheck check("dPsi");
    if(!dpsi_updated){
        helpers::lowpassFilter(dpsi, this->approximateDerivative(Psi(), psi_previous), 1.0);
        dpsi_updated = true;
    }

    return dpsi;
}


/** @idea Make this use the actual panda library (connect to the robot!)*/
Eigen::MatrixXd& PandaSim::M(){
    
    //RunCheck check("M");
    if(!m_updated){

        Eigen::VectorXd& q = state.q;
        m_previous = m_m;
        
        //ScopeTimer timer("Constructing M");
        Eigen::MatrixXd dwdq = Eigen::MatrixXd::Zero(6*n, n);
        
        dwdq(0, 0) = 0.03127*cos(q(0));
dwdq(1, 0) = 0.03127*sin(q(0));
dwdq(6, 0) = - 0.03118*cos(q(0)) - 0.07032*sin(q(0))*sin(q(1));
dwdq(6, 1) = 0.07032*cos(q(0))*cos(q(1));
dwdq(7, 0) = 0.07032*cos(q(0))*sin(q(1)) - 0.03118*sin(q(0));
dwdq(7, 1) = 0.07032*cos(q(1))*sin(q(0));
dwdq(8, 1) = -0.07032*sin(q(1));
dwdq(9, 0) = -1.0*cos(q(0));
dwdq(10, 0) = -1.0*sin(q(0));
dwdq(12, 0) = 0.02493*cos(q(1))*sin(q(0))*sin(q(2)) - 0.04435*cos(q(0))*sin(q(2)) - 0.3541*sin(q(0))*sin(q(1)) - 0.04435*cos(q(1))*cos(q(2))*sin(q(0)) - 0.02493*cos(q(0))*cos(q(2));
dwdq(12, 1) = 3.553e-21*cos(q(0))*(9.968e19*cos(q(1)) - 1.248e19*cos(q(2))*sin(q(1)) + 7.017e18*sin(q(1))*sin(q(2)));
dwdq(12, 2) = 0.02493*sin(q(0))*sin(q(2)) - 0.04435*cos(q(2))*sin(q(0)) - 0.02493*cos(q(0))*cos(q(1))*cos(q(2)) - 0.04435*cos(q(0))*cos(q(1))*sin(q(2));
dwdq(13, 0) = 0.3541*cos(q(0))*sin(q(1)) - 0.02493*cos(q(2))*sin(q(0)) - 0.04435*sin(q(0))*sin(q(2)) + 0.04435*cos(q(0))*cos(q(1))*cos(q(2)) - 0.02493*cos(q(0))*cos(q(1))*sin(q(2));
dwdq(13, 1) = 3.553e-21*sin(q(0))*(9.968e19*cos(q(1)) - 1.248e19*cos(q(2))*sin(q(1)) + 7.017e18*sin(q(1))*sin(q(2)));
dwdq(13, 2) = 0.04435*cos(q(0))*cos(q(2)) - 0.02493*cos(q(0))*sin(q(2)) - 0.02493*cos(q(1))*cos(q(2))*sin(q(0)) - 0.04435*cos(q(1))*sin(q(0))*sin(q(2));
dwdq(14, 1) = 0.02493*cos(q(1))*sin(q(2)) - 0.04435*cos(q(1))*cos(q(2)) - 0.3541*sin(q(1));
dwdq(14, 2) = 3.553e-21*sin(q(1))*(7.017e18*cos(q(2)) + 1.248e19*sin(q(2)));
dwdq(15, 0) = - 1.0*cos(q(0)) - 1.0*sin(q(0))*sin(q(1));
dwdq(15, 1) = cos(q(0))*cos(q(1));
dwdq(16, 0) = cos(q(0))*sin(q(1)) - 1.0*sin(q(0));
dwdq(16, 1) = cos(q(1))*sin(q(0));
dwdq(17, 1) = -1.0*sin(q(1));
dwdq(18, 0) = 0.02472*cos(q(0))*cos(q(2)) - 0.0825*cos(q(0))*sin(q(2)) - 0.316*sin(q(0))*sin(q(1)) + 0.03855*sin(q(0))*sin(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) + 0.03855*cos(q(0))*cos(q(3))*sin(q(2)) - 0.02472*cos(q(1))*sin(q(0))*sin(q(2)) - 0.03953*cos(q(3))*sin(q(0))*sin(q(1)) + 0.03953*cos(q(0))*sin(q(2))*sin(q(3)) + 0.03855*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.03953*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3));
dwdq(18, 1) = 6.842e-53*cos(q(0))*(4.618e51*cos(q(1)) + 5.777e50*cos(q(1))*cos(q(3)) - 1.206e51*cos(q(2))*sin(q(1)) - 5.634e50*cos(q(1))*sin(q(3)) - 3.612e50*sin(q(1))*sin(q(2)) + 5.634e50*cos(q(2))*cos(q(3))*sin(q(1)) + 5.777e50*cos(q(2))*sin(q(1))*sin(q(3)));
dwdq(18, 2) = 0.02472*cos(q(0))*cos(q(1))*cos(q(2)) - 0.02472*sin(q(0))*sin(q(2)) - 0.0825*cos(q(2))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(2)) + 0.03855*cos(q(2))*cos(q(3))*sin(q(0)) + 0.03953*cos(q(2))*sin(q(0))*sin(q(3)) + 0.03855*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2)) + 0.03953*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3));
dwdq(18, 3) = 0.03953*cos(q(3))*sin(q(0))*sin(q(2)) - 0.03855*cos(q(0))*cos(q(3))*sin(q(1)) - 0.03953*cos(q(0))*sin(q(1))*sin(q(3)) - 0.03855*sin(q(0))*sin(q(2))*sin(q(3)) - 0.03953*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + 0.03855*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3));
dwdq(19, 0) = 0.316*cos(q(0))*sin(q(1)) + 0.02472*cos(q(2))*sin(q(0)) - 0.0825*sin(q(0))*sin(q(2)) + 0.03953*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.02472*cos(q(0))*cos(q(1))*sin(q(2)) + 0.03953*cos(q(0))*cos(q(3))*sin(q(1)) - 0.03855*cos(q(0))*sin(q(1))*sin(q(3)) + 0.03855*cos(q(3))*sin(q(0))*sin(q(2)) - 0.03855*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.03953*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3));
dwdq(19, 1) = 6.842e-53*sin(q(0))*(4.618e51*cos(q(1)) + 5.777e50*cos(q(1))*cos(q(3)) - 1.206e51*cos(q(2))*sin(q(1)) - 5.634e50*cos(q(1))*sin(q(3)) - 3.612e50*sin(q(1))*sin(q(2)) + 5.634e50*cos(q(2))*cos(q(3))*sin(q(1)) + 5.777e50*cos(q(2))*sin(q(1))*sin(q(3)));
dwdq(19, 2) = 0.0825*cos(q(0))*cos(q(2)) + 0.02472*cos(q(0))*sin(q(2)) - 0.03855*cos(q(0))*cos(q(2))*cos(q(3)) + 0.02472*cos(q(1))*cos(q(2))*sin(q(0)) - 0.03953*cos(q(0))*cos(q(2))*sin(q(3)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(2)) + 0.03855*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2)) + 0.03953*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3));
dwdq(19, 3) = 0.03855*cos(q(0))*sin(q(2))*sin(q(3)) - 0.03953*cos(q(0))*cos(q(3))*sin(q(2)) - 0.03855*cos(q(3))*sin(q(0))*sin(q(1)) - 0.03953*sin(q(0))*sin(q(1))*sin(q(3)) - 0.03953*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.03855*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3));
dwdq(20, 1) = 0.03855*sin(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2)) - 0.02472*cos(q(1))*sin(q(2)) - 0.03953*cos(q(3))*sin(q(1)) - 0.316*sin(q(1)) + 0.03855*cos(q(1))*cos(q(2))*cos(q(3)) + 0.03953*cos(q(1))*cos(q(2))*sin(q(3));
dwdq(20, 2) = -5.551e-21*sin(q(1))*(4.452e18*cos(q(2)) - 1.486e19*sin(q(2)) + 6.945e18*cos(q(3))*sin(q(2)) + 7.12e18*sin(q(2))*sin(q(3)));
dwdq(20, 3) = 0.03953*cos(q(2))*cos(q(3))*sin(q(1)) - 0.03953*cos(q(1))*sin(q(3)) - 0.03855*cos(q(1))*cos(q(3)) - 0.03855*cos(q(2))*sin(q(1))*sin(q(3));
dwdq(21, 0) = cos(q(0))*cos(q(2)) - 1.0*cos(q(0)) - 1.0*sin(q(0))*sin(q(1)) - 1.0*cos(q(1))*sin(q(0))*sin(q(2));
dwdq(21, 1) = 1.233e-32*cos(q(0))*(8.113e31*cos(q(1)) - 8.113e31*sin(q(1))*sin(q(2)));
dwdq(21, 2) = cos(q(0))*cos(q(1))*cos(q(2)) - 1.0*sin(q(0))*sin(q(2));
dwdq(22, 0) = 1.0*cos(q(0))*sin(q(1)) - 1.0*sin(q(0)) + cos(q(2))*sin(q(0)) + cos(q(0))*cos(q(1))*sin(q(2));
dwdq(22, 1) = 1.233e-32*sin(q(0))*(8.113e31*cos(q(1)) - 8.113e31*sin(q(1))*sin(q(2)));
dwdq(22, 2) = cos(q(0))*sin(q(2)) + cos(q(1))*cos(q(2))*sin(q(0));
dwdq(23, 1) = - 1.0*sin(q(1)) - 1.0*cos(q(1))*sin(q(2));
dwdq(23, 2) = -1.0*cos(q(2))*sin(q(1));
dwdq(24, 0) = 0.0825*sin(q(0))*sin(q(1))*sin(q(3)) - 0.316*sin(q(0))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(2)) - 0.03841*cos(q(0))*cos(q(2))*cos(q(4)) - 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) + 0.0825*cos(q(0))*cos(q(3))*sin(q(2)) - 0.274*cos(q(3))*sin(q(0))*sin(q(1)) + 0.274*cos(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.274*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) + 0.03841*cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2)) + 0.03841*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(4)) + 0.03841*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) + 0.03841*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4));
dwdq(24, 1) = 1.519e-69*cos(q(0))*(2.08e68*cos(q(1)) + 1.804e68*cos(q(1))*cos(q(3)) - 5.43e67*cos(q(2))*sin(q(1)) - 5.43e67*cos(q(1))*sin(q(3)) + 5.43e67*cos(q(2))*cos(q(3))*sin(q(1)) + 1.804e68*cos(q(2))*sin(q(1))*sin(q(3)) + 2.528e67*cos(q(4))*sin(q(1))*sin(q(2)) - 2.528e67*cos(q(1))*sin(q(3))*sin(q(4)) + 2.528e67*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)));
dwdq(24, 2) = 0.0825*cos(q(2))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(2)) - 0.0825*cos(q(2))*sin(q(0)) + 0.274*cos(q(2))*sin(q(0))*sin(q(3)) + 0.03841*cos(q(4))*sin(q(0))*sin(q(2)) - 0.03841*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2)) + 0.274*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3)) + 0.03841*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4)) + 0.03841*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2))*sin(q(4));
dwdq(24, 3) = 0.274*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(3))*sin(q(1)) - 0.274*cos(q(0))*sin(q(1))*sin(q(3)) - 0.0825*sin(q(0))*sin(q(2))*sin(q(3)) - 0.274*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) - 0.03841*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(4)) - 0.03841*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(4)) + 0.03841*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(4));
dwdq(24, 4) = 0.03841*cos(q(2))*sin(q(0))*sin(q(4)) + 0.03841*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4)) - 0.03841*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3)) + 0.03841*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2)) - 0.03841*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4));
dwdq(25, 0) = 0.316*cos(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(2)) + 0.274*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.274*cos(q(0))*cos(q(3))*sin(q(1)) - 0.03841*cos(q(2))*cos(q(4))*sin(q(0)) - 0.0825*cos(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.274*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) - 0.03841*cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2)) - 0.03841*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) + 0.03841*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.03841*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4));
dwdq(25, 1) = 1.519e-69*sin(q(0))*(2.08e68*cos(q(1)) + 1.804e68*cos(q(1))*cos(q(3)) - 5.43e67*cos(q(2))*sin(q(1)) - 5.43e67*cos(q(1))*sin(q(3)) + 5.43e67*cos(q(2))*cos(q(3))*sin(q(1)) + 1.804e68*cos(q(2))*sin(q(1))*sin(q(3)) + 2.528e67*cos(q(4))*sin(q(1))*sin(q(2)) - 2.528e67*cos(q(1))*sin(q(3))*sin(q(4)) + 2.528e67*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)));
dwdq(25, 2) = 0.0825*cos(q(0))*cos(q(2)) - 0.0825*cos(q(0))*cos(q(2))*cos(q(3)) - 0.274*cos(q(0))*cos(q(2))*sin(q(3)) - 0.03841*cos(q(0))*cos(q(4))*sin(q(2)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(2)) - 0.03841*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(0)) - 0.03841*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(4)) + 0.0825*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2)) + 0.274*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.03841*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4));
dwdq(25, 3) = 0.0825*cos(q(0))*sin(q(2))*sin(q(3)) - 0.274*cos(q(0))*cos(q(3))*sin(q(2)) - 0.0825*cos(q(3))*sin(q(0))*sin(q(1)) - 0.274*sin(q(0))*sin(q(1))*sin(q(3)) - 0.274*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.0825*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.03841*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(4)) + 0.03841*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(4)) + 0.03841*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(4));
dwdq(25, 4) = 0.03841*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.03841*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2)) - 0.03841*cos(q(0))*cos(q(2))*sin(q(4)) - 0.03841*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3)) - 0.03841*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0));
dwdq(26, 1) = 0.0825*sin(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2)) - 0.274*cos(q(3))*sin(q(1)) - 0.316*sin(q(1)) + 0.03841*sin(q(1))*sin(q(3))*sin(q(4)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3)) + 0.274*cos(q(1))*cos(q(2))*sin(q(3)) + 0.03841*cos(q(1))*cos(q(4))*sin(q(2)) + 0.03841*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4));
dwdq(26, 2) = -3.553e-20*sin(q(1))*(2.322e18*cos(q(3))*sin(q(2)) - 1.081e18*cos(q(2))*cos(q(4)) - 2.322e18*sin(q(2)) + 7.713e18*sin(q(2))*sin(q(3)) + 1.081e18*cos(q(3))*sin(q(2))*sin(q(4)));
dwdq(26, 3) = 0.274*cos(q(2))*cos(q(3))*sin(q(1)) - 0.274*cos(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(3)) - 0.03841*cos(q(1))*cos(q(3))*sin(q(4)) - 0.0825*cos(q(2))*sin(q(1))*sin(q(3)) - 0.03841*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4));
dwdq(26, 4) = 0.03841*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1)) - 0.03841*cos(q(1))*cos(q(4))*sin(q(3)) - 0.03841*sin(q(1))*sin(q(2))*sin(q(4));

dwdq(27, 0) = 1.0*cos(q(0))*cos(q(2)) - 1.0*cos(q(0)) - 1.0*sin(q(0))*sin(q(1)) - 1.0*cos(q(1))*sin(q(0))*sin(q(2)) - 1.0*cos(q(3))*sin(q(0))*sin(q(1)) + cos(q(0))*sin(q(2))*sin(q(3)) + cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3));
dwdq(27, 1) = 1.519e-64*cos(q(0))*(6.582e63*cos(q(1)) + 6.582e63*cos(q(1))*cos(q(3)) - 6.582e63*sin(q(1))*sin(q(2)) + 6.582e63*cos(q(2))*sin(q(1))*sin(q(3)));
dwdq(27, 2) = 1.0*cos(q(0))*cos(q(1))*cos(q(2)) - 1.0*sin(q(0))*sin(q(2)) + cos(q(2))*sin(q(0))*sin(q(3)) + cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3));
dwdq(27, 3) = cos(q(3))*sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3));

dwdq(28, 0) = 1.0*cos(q(0))*sin(q(1)) - 1.0*sin(q(0)) + 1.0*cos(q(2))*sin(q(0)) + sin(q(0))*sin(q(2))*sin(q(3)) + 1.0*cos(q(0))*cos(q(1))*sin(q(2)) + cos(q(0))*cos(q(3))*sin(q(1)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3));
dwdq(28, 1) = 1.519e-64*sin(q(0))*(6.582e63*cos(q(1)) + 6.582e63*cos(q(1))*cos(q(3)) - 6.582e63*sin(q(1))*sin(q(2)) + 6.582e63*cos(q(2))*sin(q(1))*sin(q(3)));
dwdq(28, 2) = 1.0*cos(q(0))*sin(q(2)) + 1.0*cos(q(1))*cos(q(2))*sin(q(0)) - 1.0*cos(q(0))*cos(q(2))*sin(q(3)) + cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3));
dwdq(28, 3) = - 1.0*sin(q(0))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*cos(q(3))*sin(q(2)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0));

dwdq(29, 1) = cos(q(1))*cos(q(2))*sin(q(3)) - 1.0*cos(q(1))*sin(q(2)) - 1.0*cos(q(3))*sin(q(1)) - 1.0*sin(q(1));
dwdq(29, 2) = -1.233e-32*sin(q(1))*(8.113e31*cos(q(2)) + 8.113e31*sin(q(2))*sin(q(3)));
dwdq(29, 3) = cos(q(2))*cos(q(3))*sin(q(1)) - 1.0*cos(q(1))*sin(q(3));

dwdq(30, 0) = 0.0825*sin(q(0))*sin(q(1))*sin(q(3)) - 0.316*sin(q(0))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(2)) - 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) + 0.0825*cos(q(0))*cos(q(3))*sin(q(2)) - 0.384*cos(q(3))*sin(q(0))*sin(q(1)) + 0.384*cos(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) - 0.051*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4)) + 0.384*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.051*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5)) + 0.051*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.051*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) + 0.051*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.051*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.051*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) - 0.051*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0));
dwdq(30, 1) = 0.316*cos(q(0))*cos(q(1)) + 0.384*cos(q(0))*cos(q(1))*cos(q(3)) - 0.0825*cos(q(0))*cos(q(2))*sin(q(1)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(1)) + 0.051*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(5)) + 0.384*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3)) + 0.051*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.051*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.051*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.051*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1));
dwdq(30, 2) = 0.0825*cos(q(2))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(2)) - 0.0825*cos(q(2))*sin(q(0)) + 0.384*cos(q(2))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2)) + 0.384*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3)) + 0.051*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.051*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.051*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(4)) - 0.051*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0)) + 0.051*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.051*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2));
dwdq(30, 3) = 0.384*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(3))*sin(q(1)) - 0.384*cos(q(0))*sin(q(1))*sin(q(3)) - 0.0825*sin(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) - 0.051*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.051*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.051*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(5)) + 0.051*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.051*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) - 0.051*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(3));
dwdq(30, 4) = -0.051*cos(q(5))*(cos(q(2))*cos(q(4))*sin(q(0)) + cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2)) + cos(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 1.0*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4)) + cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4)));
dwdq(30, 5) = 0.051*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(1)) + 0.051*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.051*cos(q(2))*sin(q(0))*sin(q(4))*sin(q(5)) - 0.051*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 0.051*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) - 0.051*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.051*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.051*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
dwdq(31, 0) = 0.316*cos(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(2)) + 0.384*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.384*cos(q(0))*cos(q(3))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + 0.051*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.051*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.051*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.051*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.051*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.051*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.051*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) + 0.051*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
dwdq(31, 1) = 0.316*cos(q(1))*sin(q(0)) + 0.384*cos(q(1))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(2))*sin(q(0))*sin(q(1)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(1)) + 0.051*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.384*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3)) + 0.051*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3)) + 0.051*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.051*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.051*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1));
dwdq(31, 2) = 0.0825*cos(q(0))*cos(q(2)) - 0.0825*cos(q(0))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(2))*sin(q(3)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(2)) + 0.0825*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2)) - 0.051*cos(q(0))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.051*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.384*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.051*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) - 0.051*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.051*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.051*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2));
dwdq(31, 3) = 0.0825*cos(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(3))*sin(q(2)) - 0.0825*cos(q(3))*sin(q(0))*sin(q(1)) - 0.384*sin(q(0))*sin(q(1))*sin(q(3)) - 0.384*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.0825*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.051*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(5)) - 0.051*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.051*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.051*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.051*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.051*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3));
dwdq(31, 4) = -0.051*cos(q(5))*(cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(2))*cos(q(4)) + cos(q(0))*cos(q(3))*sin(q(2))*sin(q(4)) + sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) + cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4)));
dwdq(31, 5) = 0.051*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.051*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.051*cos(q(0))*cos(q(2))*sin(q(4))*sin(q(5)) - 0.051*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(3)) - 0.051*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5)) + 0.051*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) - 0.051*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.051*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(5));

dwdq(32, 1) = 0.0825*sin(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2)) - 0.384*cos(q(3))*sin(q(1)) - 0.316*sin(q(1)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3)) + 0.384*cos(q(1))*cos(q(2))*sin(q(3)) - 0.051*cos(q(3))*sin(q(1))*sin(q(5)) + 0.051*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 0.051*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.051*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.051*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
dwdq(32, 2) = 8.553e-53*sin(q(1))*(9.646e50*sin(q(2)) - 9.646e50*cos(q(3))*sin(q(2)) - 4.49e51*sin(q(2))*sin(q(3)) - 5.963e50*sin(q(2))*sin(q(3))*sin(q(5)) + 5.963e50*cos(q(2))*cos(q(5))*sin(q(4)) + 5.963e50*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)));
dwdq(32, 3) = 0.384*cos(q(2))*cos(q(3))*sin(q(1)) - 0.384*cos(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(3)) - 0.0825*cos(q(2))*sin(q(1))*sin(q(3)) - 0.051*cos(q(1))*sin(q(3))*sin(q(5)) + 0.051*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.051*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) + 0.051*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3));
dwdq(32, 4) = 0.051*cos(q(5))*(cos(q(4))*sin(q(1))*sin(q(2)) - 1.0*cos(q(1))*sin(q(3))*sin(q(4)) + cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)));
dwdq(32, 5) = 0.051*cos(q(1))*cos(q(3))*cos(q(5)) + 0.051*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.051*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.051*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.051*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
dwdq(33, 0) = 1.0*cos(q(0))*cos(q(2)) - 1.0*cos(q(0)) - 1.0*sin(q(0))*sin(q(1)) + cos(q(0))*cos(q(2))*cos(q(4)) - 1.0*cos(q(1))*sin(q(0))*sin(q(2)) - 1.0*cos(q(3))*sin(q(0))*sin(q(1)) + 1.0*cos(q(0))*sin(q(2))*sin(q(3)) + 1.0*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 1.0*cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(4)) - 1.0*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4));
dwdq(33, 1) = 1.0*cos(q(0))*cos(q(1)) + 1.0*cos(q(0))*cos(q(1))*cos(q(3)) - 1.0*cos(q(0))*sin(q(1))*sin(q(2)) + 1.0*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(2)) + cos(q(0))*cos(q(1))*sin(q(3))*sin(q(4)) - 1.0*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4));
dwdq(33, 2) = 1.0*cos(q(0))*cos(q(1))*cos(q(2)) - 1.0*sin(q(0))*sin(q(2)) + 1.0*cos(q(2))*sin(q(0))*sin(q(3)) - 1.0*cos(q(4))*sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4)) + 1.0*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3)) - 1.0*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4)) - 1.0*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2))*sin(q(4));
dwdq(33, 3) = 1.0*cos(q(3))*sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + cos(q(0))*cos(q(3))*sin(q(1))*sin(q(4)) + sin(q(0))*sin(q(2))*sin(q(3))*sin(q(4)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(4));
dwdq(33, 4) = cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4)) - 1.0*cos(q(2))*sin(q(0))*sin(q(4)) - 1.0*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4));

dwdq(34, 0) = 1.0*cos(q(0))*sin(q(1)) - 1.0*sin(q(0)) + 1.0*cos(q(2))*sin(q(0)) + 1.0*sin(q(0))*sin(q(2))*sin(q(3)) + 1.0*cos(q(0))*cos(q(1))*sin(q(2)) + 1.0*cos(q(0))*cos(q(3))*sin(q(1)) + cos(q(2))*cos(q(4))*sin(q(0)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2)) + cos(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 1.0*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4)) + cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4));
dwdq(34, 1) = 1.0*cos(q(1))*sin(q(0)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)) + 1.0*cos(q(1))*cos(q(3))*sin(q(0)) + 1.0*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3)) - 1.0*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(2)) + cos(q(1))*sin(q(0))*sin(q(3))*sin(q(4)) - 1.0*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(4));
dwdq(34, 2) = 1.0*cos(q(0))*sin(q(2)) + 1.0*cos(q(1))*cos(q(2))*sin(q(0)) - 1.0*cos(q(0))*cos(q(2))*sin(q(3)) + cos(q(0))*cos(q(4))*sin(q(2)) + cos(q(1))*cos(q(2))*cos(q(4))*sin(q(0)) + cos(q(0))*cos(q(2))*cos(q(3))*sin(q(4)) + 1.0*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3)) - 1.0*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4));
dwdq(34, 3) = cos(q(3))*sin(q(0))*sin(q(1))*sin(q(4)) - 1.0*cos(q(0))*cos(q(3))*sin(q(2)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) - 1.0*sin(q(0))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(4)) - 1.0*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(4));
dwdq(34, 4) = cos(q(0))*cos(q(2))*sin(q(4)) + cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2)) - 1.0*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4)) + cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0));

dwdq(35, 1) = 1.0*cos(q(1))*cos(q(2))*sin(q(3)) - 1.0*cos(q(1))*sin(q(2)) - 1.0*cos(q(3))*sin(q(1)) - 1.0*sin(q(1))*sin(q(3))*sin(q(4)) - 1.0*sin(q(1)) - 1.0*cos(q(1))*cos(q(4))*sin(q(2)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4));
dwdq(35, 2) = -1.519e-64*sin(q(1))*(6.582e63*cos(q(2)) + 6.582e63*cos(q(2))*cos(q(4)) + 6.582e63*sin(q(2))*sin(q(3)) - 6.582e63*cos(q(3))*sin(q(2))*sin(q(4)));
dwdq(35, 3) = 1.0*cos(q(2))*cos(q(3))*sin(q(1)) - 1.0*cos(q(1))*sin(q(3)) + cos(q(1))*cos(q(3))*sin(q(4)) + cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4));
dwdq(35, 4) = sin(q(1))*sin(q(2))*sin(q(4)) + cos(q(1))*cos(q(4))*sin(q(3)) - 1.0*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1));

dwdq(36, 0) = 0.0825*sin(q(0))*sin(q(1))*sin(q(3)) - 0.316*sin(q(0))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(2)) - 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) + 0.0825*cos(q(0))*cos(q(3))*sin(q(2)) - 0.384*cos(q(3))*sin(q(0))*sin(q(1)) + 0.384*cos(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) - 0.088*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4)) + 0.088*cos(q(0))*cos(q(2))*cos(q(4))*sin(q(6)) + 0.384*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.088*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5)) + 0.088*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.088*cos(q(0))*cos(q(2))*cos(q(5))*cos(q(6))*sin(q(4)) + 0.088*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(6)) - 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(4))*sin(q(6)) - 0.088*cos(q(3))*cos(q(6))*sin(q(0))*sin(q(1))*sin(q(5)) + 0.088*cos(q(0))*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(2)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4))*sin(q(6)) + 0.088*cos(q(1))*cos(q(2))*cos(q(6))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0));
dwdq(36, 1) = 0.316*cos(q(0))*cos(q(1)) + 0.384*cos(q(0))*cos(q(1))*cos(q(3)) - 0.0825*cos(q(0))*cos(q(2))*sin(q(1)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(1)) + 0.088*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(5)) + 0.384*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3)) + 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.088*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(6))*sin(q(5)) + 0.088*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(6)) + 0.088*cos(q(0))*cos(q(1))*sin(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(3)) - 0.088*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(6)) + 0.088*cos(q(0))*cos(q(2))*cos(q(6))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(5))*cos(q(6))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(1));
dwdq(36, 2) = 0.0825*cos(q(2))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(2)) - 0.0825*cos(q(2))*sin(q(0)) + 0.384*cos(q(2))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2)) + 0.384*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(6)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(4)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(6)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0)) + 0.088*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4))*sin(q(6)) + 0.088*cos(q(2))*cos(q(6))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*cos(q(6))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0)) - 0.088*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2))*sin(q(4))*sin(q(6)) + 0.088*cos(q(0))*cos(q(1))*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(2));
dwdq(36, 3) = 0.384*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(3))*sin(q(1)) - 0.384*cos(q(0))*sin(q(1))*sin(q(3)) - 0.0825*sin(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) - 0.088*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.088*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(6)) + 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(6))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*cos(q(6))*sin(q(0))*sin(q(2))*sin(q(5)) + 0.088*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(3)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(6))*sin(q(5)) + 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(1)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(4))*sin(q(6)) + 0.088*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(3));
dwdq(36, 4) = 0.088*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(6)) - 0.088*cos(q(2))*sin(q(0))*sin(q(4))*sin(q(6)) - 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.088*cos(q(2))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0)) - 0.088*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(6)) - 0.088*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0)) + 0.088*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(6)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(4)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(6)) - 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(2)) - 0.088*cos(q(0))*cos(q(5))*cos(q(6))*sin(q(1))*sin(q(3))*sin(q(4)) + 0.088*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(4));
dwdq(36, 5) = 0.088*(cos(q(6)) + 1.0)*(cos(q(0))*cos(q(3))*cos(q(5))*sin(q(1)) + cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) + cos(q(2))*sin(q(0))*sin(q(4))*sin(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) - 1.0*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) + cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5)));
dwdq(36, 6) = 0.088*cos(q(2))*cos(q(4))*cos(q(6))*sin(q(0)) + 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(6))*sin(q(2)) + 0.088*cos(q(0))*cos(q(6))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5))*sin(q(6)) - 0.088*cos(q(3))*cos(q(6))*sin(q(0))*sin(q(2))*sin(q(4)) + 0.088*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4))*sin(q(6)) - 0.088*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5))*sin(q(6)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(6))*sin(q(4)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5))*sin(q(6)) + 0.088*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4))*sin(q(6)) - 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3))*sin(q(6)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(6)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(6));
dwdq(37, 0) = 0.316*cos(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(2)) + 0.384*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.384*cos(q(0))*cos(q(3))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.088*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*cos(q(2))*cos(q(4))*sin(q(0))*sin(q(6)) + 0.088*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2))*sin(q(6)) + 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) + 0.088*cos(q(0))*cos(q(3))*cos(q(6))*sin(q(1))*sin(q(5)) - 0.088*cos(q(2))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(4)) + 0.088*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(6)) + 0.088*cos(q(6))*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(6))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(1))*sin(q(3)) - 0.088*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(2)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6));
dwdq(37, 1) = 0.316*cos(q(1))*sin(q(0)) + 0.384*cos(q(1))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(2))*sin(q(0))*sin(q(1)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(1)) + 0.088*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.384*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3)) + 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3)) + 0.088*cos(q(1))*cos(q(3))*cos(q(6))*sin(q(0))*sin(q(5)) + 0.088*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(2))*sin(q(6)) + 0.088*cos(q(1))*sin(q(0))*sin(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1)) + 0.088*cos(q(1))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(3)) - 0.088*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(4))*sin(q(6)) + 0.088*cos(q(2))*cos(q(6))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(1));
dwdq(37, 2) = 0.0825*cos(q(0))*cos(q(2)) - 0.0825*cos(q(0))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(2))*sin(q(3)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(2)) + 0.0825*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(4))*sin(q(2))*sin(q(6)) + 0.384*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(0))*sin(q(6)) + 0.088*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(0))*cos(q(2))*cos(q(6))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(4)) + 0.088*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6)) - 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) - 0.088*cos(q(1))*cos(q(2))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(4)) - 0.088*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(6)) + 0.088*cos(q(1))*cos(q(6))*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(2));
dwdq(37, 3) = 0.0825*cos(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(3))*sin(q(2)) - 0.0825*cos(q(3))*sin(q(0))*sin(q(1)) - 0.384*sin(q(0))*sin(q(1))*sin(q(3)) - 0.384*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.0825*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(5)) - 0.088*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*cos(q(6))*sin(q(2))*sin(q(5)) + 0.088*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(4))*sin(q(6)) - 0.088*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(6))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(6))*sin(q(0))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(1)) - 0.088*cos(q(0))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(3));
dwdq(37, 4) = 0.088*cos(q(0))*cos(q(2))*cos(q(4))*cos(q(5)) + 0.088*cos(q(0))*cos(q(2))*sin(q(4))*sin(q(6)) + 0.088*cos(q(0))*cos(q(2))*cos(q(4))*cos(q(5))*cos(q(6)) - 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(6)) - 0.088*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(6)) - 0.088*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) + 0.088*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(6)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(6)) - 0.088*cos(q(1))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(4)) - 0.088*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(0))*sin(q(4));
dwdq(37, 5) = -0.088*(cos(q(6)) + 1.0)*(cos(q(0))*cos(q(5))*sin(q(2))*sin(q(3)) - 1.0*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(1)) + cos(q(0))*cos(q(2))*sin(q(4))*sin(q(5)) + cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(3)) + cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5)) - 1.0*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) + cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(5)));
dwdq(37, 6) = 0.088*cos(q(1))*cos(q(4))*cos(q(6))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(2))*cos(q(4))*cos(q(6)) + 0.088*cos(q(0))*cos(q(3))*cos(q(6))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4))*sin(q(6)) + 0.088*cos(q(6))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5))*sin(q(6)) + 0.088*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5))*sin(q(6)) + 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(6))*sin(q(0))*sin(q(4)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(6)) + 0.088*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5))*sin(q(6)) + 0.088*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(6)) - 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(6)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(6));
dwdq(38, 1) = 0.0825*sin(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2)) - 0.384*cos(q(3))*sin(q(1)) - 0.316*sin(q(1)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3)) + 0.384*cos(q(1))*cos(q(2))*sin(q(3)) - 0.088*cos(q(3))*sin(q(1))*sin(q(5)) + 0.088*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.088*cos(q(1))*cos(q(4))*sin(q(2))*sin(q(6)) - 0.088*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(3))*cos(q(6))*sin(q(1))*sin(q(5)) - 0.088*sin(q(1))*sin(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4))*sin(q(6)) + 0.088*cos(q(1))*cos(q(2))*cos(q(6))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*cos(q(6))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6));
dwdq(38, 2) = 0.0005*sin(q(1))*(165.0*sin(q(2)) - 165.0*cos(q(3))*sin(q(2)) - 768.0*sin(q(2))*sin(q(3)) - 176.0*sin(q(2))*sin(q(3))*sin(q(5)) + 176.0*cos(q(2))*cos(q(5))*sin(q(4)) - 176.0*cos(q(2))*cos(q(4))*sin(q(6)) + 176.0*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) + 176.0*cos(q(2))*cos(q(5))*cos(q(6))*sin(q(4)) + 176.0*cos(q(3))*sin(q(2))*sin(q(4))*sin(q(6)) - 176.0*cos(q(6))*sin(q(2))*sin(q(3))*sin(q(5)) + 176.0*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(2)));
dwdq(38, 3) = 0.384*cos(q(2))*cos(q(3))*sin(q(1)) - 0.384*cos(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(3)) - 0.0825*cos(q(2))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.088*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) + 0.088*cos(q(1))*cos(q(3))*sin(q(4))*sin(q(6)) - 0.088*cos(q(1))*cos(q(6))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*cos(q(6)) + 0.088*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) + 0.088*cos(q(2))*cos(q(3))*cos(q(6))*sin(q(1))*sin(q(5)) + 0.088*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4))*sin(q(6)) + 0.088*cos(q(2))*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(1))*sin(q(3));
dwdq(38, 4) = 0.088*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.088*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) + 0.088*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(6)) + 0.088*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(6)) + 0.088*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(6)) + 0.088*cos(q(4))*cos(q(5))*cos(q(6))*sin(q(1))*sin(q(2)) - 0.088*cos(q(1))*cos(q(5))*cos(q(6))*sin(q(3))*sin(q(4)) + 0.088*cos(q(2))*cos(q(3))*cos(q(5))*cos(q(6))*sin(q(1))*sin(q(4));
dwdq(38, 5) = 0.088*(cos(q(6)) + 1.0)*(cos(q(1))*cos(q(3))*cos(q(5)) + cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 1.0*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 1.0*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5)));
dwdq(38, 6) = 0.088*cos(q(1))*cos(q(6))*sin(q(3))*sin(q(4)) - 0.088*cos(q(4))*cos(q(6))*sin(q(1))*sin(q(2)) - 0.088*cos(q(1))*cos(q(3))*sin(q(5))*sin(q(6)) - 0.088*cos(q(2))*cos(q(3))*cos(q(6))*sin(q(1))*sin(q(4)) - 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3))*sin(q(6)) - 0.088*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5))*sin(q(6)) - 0.088*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(6)) + 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(6));
dwdq(39, 0) = 1.0*cos(q(0))*cos(q(2)) - 1.0*cos(q(0)) - 1.0*sin(q(0))*sin(q(1)) + 1.0*cos(q(0))*cos(q(2))*cos(q(4)) - 1.0*cos(q(1))*sin(q(0))*sin(q(2)) - 1.0*cos(q(3))*sin(q(0))*sin(q(1)) + 1.0*cos(q(0))*sin(q(2))*sin(q(3)) + 1.0*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 1.0*cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(4)) + cos(q(3))*cos(q(5))*sin(q(0))*sin(q(1)) - 1.0*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(3)) - 1.0*cos(q(0))*cos(q(2))*sin(q(4))*sin(q(5)) - 1.0*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4)) - 1.0*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(3)) - 1.0*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5)) + cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) - 1.0*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(5));
dwdq(39, 1) = 1.0*cos(q(0))*cos(q(1)) + 1.0*cos(q(0))*cos(q(1))*cos(q(3)) - 1.0*cos(q(0))*sin(q(1))*sin(q(2)) - 1.0*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(5)) + 1.0*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(2)) + 1.0*cos(q(0))*cos(q(1))*sin(q(3))*sin(q(4)) - 1.0*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)) - 1.0*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) + cos(q(0))*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) + cos(q(0))*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) - 1.0*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
dwdq(39, 2) = 1.0*cos(q(0))*cos(q(1))*cos(q(2)) - 1.0*sin(q(0))*sin(q(2)) + 1.0*cos(q(2))*sin(q(0))*sin(q(3)) - 1.0*cos(q(4))*sin(q(0))*sin(q(2)) + 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4)) + 1.0*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3)) - 1.0*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4)) - 1.0*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(3)) + sin(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2))*sin(q(4)) - 1.0*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(3)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(4))*sin(q(5)) - 1.0*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5));
dwdq(39, 3) = 1.0*cos(q(3))*sin(q(0))*sin(q(2)) - 1.0*cos(q(0))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + 1.0*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(4)) + cos(q(0))*cos(q(5))*sin(q(1))*sin(q(3)) - 1.0*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(2)) + 1.0*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(4)) + cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(4)) + cos(q(0))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5)) + cos(q(4))*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(3))*sin(q(5));
dwdq(39, 4) = 1.0*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3)) - 1.0*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4)) - 1.0*cos(q(2))*sin(q(0))*sin(q(4)) - 1.0*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2)) - 1.0*cos(q(2))*cos(q(4))*sin(q(0))*sin(q(5)) + 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) - 1.0*cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2))*sin(q(5)) - 1.0*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(4))*sin(q(5)) + cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4))*sin(q(5));
dwdq(39, 5) = cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5)) - 1.0*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) - 1.0*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) + cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 1.0*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) + cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
dwdq(40, 0) = 1.0*cos(q(0))*sin(q(1)) - 1.0*sin(q(0)) + 1.0*cos(q(2))*sin(q(0)) + 1.0*sin(q(0))*sin(q(2))*sin(q(3)) + 1.0*cos(q(0))*cos(q(1))*sin(q(2)) + 1.0*cos(q(0))*cos(q(3))*sin(q(1)) + 1.0*cos(q(2))*cos(q(4))*sin(q(0)) - 1.0*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + 1.0*cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2)) - 1.0*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(1)) + 1.0*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 1.0*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4)) - 1.0*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) - 1.0*cos(q(2))*sin(q(0))*sin(q(4))*sin(q(5)) + 1.0*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4)) + cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) - 1.0*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) - 1.0*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(5)) + cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
dwdq(40, 1) = 1.0*cos(q(1))*sin(q(0)) - 1.0*sin(q(0))*sin(q(1))*sin(q(2)) + 1.0*cos(q(1))*cos(q(3))*sin(q(0)) - 1.0*cos(q(1))*cos(q(3))*cos(q(5))*sin(q(0)) + 1.0*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3)) - 1.0*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(2)) + 1.0*cos(q(1))*sin(q(0))*sin(q(3))*sin(q(4)) - 1.0*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(4)) - 1.0*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(4))*sin(q(0))*sin(q(3))*sin(q(5)) + sin(q(0))*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) - 1.0*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(5));
dwdq(40, 2) = 1.0*cos(q(0))*sin(q(2)) + 1.0*cos(q(1))*cos(q(2))*sin(q(0)) - 1.0*cos(q(0))*cos(q(2))*sin(q(3)) + 1.0*cos(q(0))*cos(q(4))*sin(q(2)) + 1.0*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(0)) + 1.0*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(4)) + cos(q(0))*cos(q(2))*cos(q(5))*sin(q(3)) + 1.0*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3)) - 1.0*cos(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) + cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5)) - 1.0*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(4)) - 1.0*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) - 1.0*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(4))*sin(q(5)) - 1.0*cos(q(1))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(5));
dwdq(40, 3) = cos(q(0))*cos(q(3))*cos(q(5))*sin(q(2)) - 1.0*cos(q(0))*cos(q(3))*sin(q(2)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) - 1.0*sin(q(0))*sin(q(1))*sin(q(3)) + 1.0*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(4)) - 1.0*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(4)) + cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(0)) - 1.0*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(4)) + cos(q(3))*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(5)) - 1.0*cos(q(0))*cos(q(4))*sin(q(2))*sin(q(3))*sin(q(5)) - 1.0*cos(q(1))*cos(q(2))*cos(q(4))*sin(q(0))*sin(q(3))*sin(q(5));
dwdq(40, 4) = 1.0*cos(q(0))*cos(q(2))*sin(q(4)) + 1.0*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2)) + cos(q(0))*cos(q(2))*cos(q(4))*sin(q(5)) - 1.0*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4)) + 1.0*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3)) + 1.0*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0)) - 1.0*cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(5)) - 1.0*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(4))*sin(q(5)) - 1.0*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4))*sin(q(5)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(4))*sin(q(5));
dwdq(40, 5) = cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4)) + cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5)) - 1.0*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) + cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) - 1.0*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) - 1.0*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) + cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) + cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0));

dwdq(41, 1) = 1.0*cos(q(1))*cos(q(2))*sin(q(3)) - 1.0*cos(q(1))*sin(q(2)) - 1.0*cos(q(3))*sin(q(1)) - 1.0*sin(q(1))*sin(q(3))*sin(q(4)) - 1.0*sin(q(1)) - 1.0*cos(q(1))*cos(q(4))*sin(q(2)) + cos(q(3))*cos(q(5))*sin(q(1)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4)) - 1.0*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) - 1.0*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) - 1.0*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
dwdq(41, 2) = 1.0*cos(q(3))*sin(q(1))*sin(q(2))*sin(q(4)) - 1.0*sin(q(1))*sin(q(2))*sin(q(3)) - 1.0*cos(q(2))*cos(q(4))*sin(q(1)) - 1.0*cos(q(2))*sin(q(1)) + cos(q(5))*sin(q(1))*sin(q(2))*sin(q(3)) + cos(q(2))*sin(q(1))*sin(q(4))*sin(q(5)) + cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5));
dwdq(41, 3) = 1.0*cos(q(2))*cos(q(3))*sin(q(1)) - 1.0*cos(q(1))*sin(q(3)) + 1.0*cos(q(1))*cos(q(3))*sin(q(4)) + cos(q(1))*cos(q(5))*sin(q(3)) - 1.0*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1)) + cos(q(1))*cos(q(3))*cos(q(4))*sin(q(5)) + 1.0*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4)) + cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5));
dwdq(41, 4) = 1.0*sin(q(1))*sin(q(2))*sin(q(4)) + 1.0*cos(q(1))*cos(q(4))*sin(q(3)) - 1.0*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1)) + cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5)) - 1.0*cos(q(1))*sin(q(3))*sin(q(4))*sin(q(5)) + cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(5));
dwdq(41, 5) = cos(q(1))*cos(q(3))*sin(q(5)) + cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) + cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 1.0*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1));

    m_m = dwdq.transpose()*M_constants*dwdq;
    m_updated = true;

    }
    
    return m_m;
}