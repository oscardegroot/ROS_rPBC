/*
File: Panda.cpp

System file for the real Franka Emika Panda 7DOF robotic manipulator
*/

#include <Panda.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>


namespace panda {

Panda::Panda()
	:System(7, 7)
{
	logMsg("Panda", "Panda Constructor", 2);
}

Panda::~Panda(){};


bool Panda::init (hardware_interface::RobotHW* hw, ros::NodeHandle& nh){
	logMsg("Panda", "Panda Controller Started!", 2);

	// if (!node_handle.getParam("vel_max", vel_max_)) {
	// ROS_INFO_STREAM(
	//     "JointImpedanceExampleController: No parameter vel_max, defaulting to: " << vel_max_);
	// }
	// if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
	// ROS_INFO_STREAM(
	//     "JointImpedanceExampleController: No parameter acceleration_time, defaulting to: "
	//     << acceleration_time_);
	// }

	std::vector<std::string> joint_names;
	if (!nh.getParam("joint_names", joint_names) || joint_names.size() != 7) {

		logMsg("Panda", "Invalid or no joint_names parameters provided!", 0);
		return false;
	}

	// double publish_rate(30.0);
	// if (!node_handle.getParam("publish_rate", publish_rate)) {
	// ROS_INFO_STREAM("JointImpedanceExampleController: publish_rate not found. Defaulting to "
	//                 << publish_rate);
	// }
	// rate_trigger_ = franka_hw::TriggerRate(publish_rate);

	// franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface = hw->get<franka_hw::FrankaPoseCartesianInterface>();
	// if (cartesian_pose_interface == nullptr) {
	// 	logMsg("Panda", "Error getting cartesian pose interface from hardware", 0);
	// 	return false;
	// }
	// try {
	// cartesian_pose_handle_.reset(new franka_hw::FrankaCartesianPoseHandle(
	//     cartesian_pose_interface->getHandle(arm_id + "_robot")));
	// } catch (hardware_interface::HardwareInterfaceException& ex) {
	// 	logMsg("Panda", "Exception getting cartesian pose handle from interface: " << ex.what(), 0);
	// 	return false;
	// }

	// Temporary!
	std::string arm_id = "1";

	franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface =
	      hw->get<franka_hw::FrankaPoseCartesianInterface>();
	if (cartesian_pose_interface == nullptr) {
		logMsg("Panda", "Error getting cartesian pose interface from hardware");
		return false;
	}

	try {
		cartesian_pose_handle_.reset(new franka_hw::FrankaCartesianPoseHandle(
		cartesian_pose_interface->getHandle(arm_id + "_robot")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		errorRetrieving("Cartesian Pose Handle", ex.what());
		return false;
	}

	/* Initialise the model interface */
	franka_hw::FrankaModelInterface* model_interface = hw->get<franka_hw::FrankaModelInterface>();

	if (model_interface == nullptr) {
		logMsg("Panda", "Error getting model interface from hardware", 0);
		return false;
	}

	try {
	model_handle_.reset(
	    new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
			errorRetrieving("model handle", ex.what());
		return false;
	}


	/* Initialise the joint effort interface */
	hardware_interface::EffortJointInterface* effort_joint_interface = hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		logMsg("Panda", "Error getting effort joint interface from hardware", 0);
		return false;
	}

	for (size_t i = 0; i < 7; ++i) {
		try {
		  joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
		} catch (const hardware_interface::HardwareInterfaceException& ex) {
			errorRetrieving("joint interface", ex.what());
		  	return false;
		}
	}


	// Get a nodehandle
	// safelyRetrieve("i_id", i_id);
	safelyRetrieve("/l", l);
	// safelyRetrieve("/N_dim", N);

	// Define the System and the controller		
	system = std::make_unique<PandaSim>();
	controller = std::make_unique<IDAPBC>(l, *system);
	cmm = std::make_unique<CMM>();

	//torques_publisher_.init(nh, "torque_comparison", 1);

	//std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0); -> Check filtering in example!

	return true;
}  // mandatory

// Move to general functionality later:
template <class T> 
bool Panda::safelyRetrieve(std::string name, T& param){

	if (!nh.getParam(name, param)) {
		logMsg("Panda", "Failed to retrieve parameter " + std::to_string(param), 0);
		return false;
		}

	return true;

}

inline void Panda::errorRetrieving(std::string name, const char* ex_what){
	logMsg("Panda", "Exception getting " + name + ex_what, 0);

}


void Panda::setState(){

	franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
	this->state.q = arrayToVector<7>(robot_state.q);
	this->state.dq = arrayToVector<7>(robot_state.dq);
	this->state.z = arrayToVector<16>(robot_state.O_T_EE).block(0,3,3,1);

}

void Panda::update (const ros::Time& time, const ros::Duration& period){

		// Network sampled
		Eigen::VectorXd tau_network = Eigen::VectorXd::Zero(l); //cmm->sample(controller->getOutput(system));

		setState();
		
		// Calculate the control input
		Eigen::VectorXd tau = controller->computeControl((*this), tau_network);
		
		// Send the input
		system->sendInput(tau);

} // mandatory



// void starting (const ros::Time& time);   // optional
// void stopping (const ros::Time& time);  // optional

bool Panda::sendInput(Eigen::VectorXd tau){
	for (size_t i = 0; i < 7; ++i) {
		joint_handles_[i].setCommand(tau[i]);
	}
}

Eigen::MatrixXd Panda::M(){
		std::array<double, 49> mass_array = model_handle_->getMass();
		Eigen::MatrixXd mass(7, 7);
		
		for(int i = 0; i < 7; i++){
			for(int j = 0; j < 7; j++){
				mass(i, j) = mass_array[i*7 + j];
			}
		}

		return mass;
	}

Eigen::VectorXd Panda::dVdq(){
		return arrayToVector<7>(model_handle_->getGravity());
	}

template <int N>
Eigen::VectorXd Panda::arrayToVector(std::array<double, N> input){

		Eigen::VectorXd result(N);

		for(int i = 0; i < N; i++){
			result(i) = input[i];
		}

		return result;
	}

}

// Implementation ..
PLUGINLIB_EXPORT_CLASS(panda::Panda,
                       controller_interface::ControllerBase)