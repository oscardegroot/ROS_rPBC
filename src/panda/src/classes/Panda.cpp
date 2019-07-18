/*
File: Panda.cpp

System file for the real Franka Emika Panda 7DOF robotic manipulator
*/

#include <Panda.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>

namespace panda {

Panda::Panda()
	:System(7, 7)
{
}

Panda::~Panda(){};


bool Panda::init (hardware_interface::RobotHW* hw, ros::NodeHandle& nh){
	
	logMsg("Panda", "Panda Controller Started!", 2);

	/* Not working yet! */
	//Set collision behaviour
	connect_client = nh.serviceClient<franka_control::SetFullCollisionBehavior>("/franka_control/set_full_collision_behavior");

	franka_control::SetFullCollisionBehavior req;
	std::array<double, 7> low = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	std::array<double, 7> high = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
  	std::copy(low.cbegin(),low.cend(), req.request.lower_torque_thresholds_acceleration.begin());
	std::copy(high.cbegin(),high.cend(), req.request.upper_torque_thresholds_acceleration.begin());
	std::copy(low.cbegin(),low.cend(), req.request.lower_torque_thresholds_nominal.begin());
	std::copy(high.cbegin(),high.cend(), req.request.upper_torque_thresholds_nominal.begin());
	
	std::array<double, 6> low_6 = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
	std::array<double, 6> high_6 = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
  	std::copy(low_6.cbegin(),low_6.cend(), req.request.lower_force_thresholds_acceleration.begin());
	std::copy(high_6.cbegin(),high_6.cend(), req.request.upper_force_thresholds_acceleration.begin());
	std::copy(low_6.cbegin(),low_6.cend(), req.request.lower_force_thresholds_nominal.begin());
	std::copy(high_6.cbegin(),high_6.cend(), req.request.upper_force_thresholds_nominal.begin());

	// Call the service
	if(!connect_client.call(req)){

		// If not respondant return false!
		logMsg("Panda", "Failed to set collision behaviour!", 0);
		return false;
	}

	
	/* Retrieve names */
	std::vector<std::string> joint_names;
	std::string arm_id;

	helpers::safelyRetrieveArray(nh, "joint_names", joint_names, 7);
	helpers::safelyRetrieve(nh, "arm_id", arm_id);

	/* Set up interfaces */
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
		helpers::errorRetrieving("Cartesian Pose Handle", ex.what());
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
			helpers::errorRetrieving("model handle", ex.what());
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
			helpers::errorRetrieving("joint interface", ex.what());
		  	return false;
		}
	}

	// Retrieve parameters
	helpers::safelyRetrieve(nh, "velocity_norm_bound", velocity_norm_bound, 0.4);
	helpers::safelyRetrieve(nh, "velocity_element_bound", velocity_element_bound, 0.3);
	helpers::safelyRetrieve(nh, "z_lower_bound", z_lower_bound, 0.2);
	helpers::safelyRetrieve(nh, "torque_bound", torque_bound, 2.0);

	int id;
	helpers::safelyRetrieve(nh, "ID", id);


	// Initialise the controller
	controller = std::make_unique<IDAPBC>(*this);
	cmm = std::make_unique<CMM>(id);

	//torques_publisher_.init(nh, "torque_comparison", 1);
	//std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0); -> Check filtering in example!

	// Get an initial state reading
	robot_state = cartesian_pose_handle_->getRobotState();
	retrieveState();

	logMsg("Panda", "Initialisation Completed!", 2);

	return true;
}

/* Function to check if bounds are not exceeded 
	Throws an error if they are!
*/
void Panda::checkSafety(){

	// Check if the panda is not below its lower bound
	if(this->state.z(2) < z_lower_bound){
		throw BoundException(
			"Panda: Panda fell below lower bound!");
	}

		// Check if the velocity of the EE surpassed its bound
	if((this->state.z - last_z).norm() > velocity_norm_bound){
		throw EEVelocityBoundException(
			"Panda: Panda EE velocity higher the velocity bound" + 
			std::to_string(velocity_norm_bound) + ")");
	}

	// Check if the velocity of the EE surpassed its bound
	for(int i = 0; i < 7; i++){
		if(std::abs(state.dq(i)) > velocity_element_bound){
			throw VelocityElementBoundException(
				"Panda: Panda velocity higher the velocity bound (element=" + 
				std::to_string(i+1) + ", bound=" + 
			std::to_string(velocity_element_bound) + 
			", value=" + std::to_string(std::abs(state.dq(i))) + 
			")");
		}
	}
}


void Panda::retrieveState(){
	// Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
	// Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
	// Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
	// Eigen::Vector3d position(transform.translation());
	// Eigen::Quaterniond orientation(transform.linear());

	this->state.q = helpers::arrayToVector<7>(robot_state.q);
	this->state.dq = helpers::arrayToVector<7>(robot_state.dq);
	std::array<double, 3> z{{robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]}};
	this->state.z = helpers::arrayToVector<3>(z);


}

void Panda::starting(const ros::Time& /*time*/) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
}

void Panda::update (const ros::Time& time, const ros::Duration& period){

		// Retrieve the robot state
		robot_state = cartesian_pose_handle_->getRobotState();
		cartesian_pose_handle_->setCommand(initial_pose_);

		// Network sampled
		retrieveState();

		// Check for errors
		checkSafety();

		//Eigen::VectorXd tau_network = Eigen::VectorXd::Zero(controller->l); //
		Eigen::VectorXd tau_network = cmm->sample(controller->getOutput(*this));
		//logTmp(tau_network);
		// Calculate the control input
		Eigen::VectorXd tau = controller->computeControl((*this), tau_network);
		//std::cout << tau <<std::endl;

		//tau = Eigen::VectorXd::Zero(7);
		
		// Check if the torque bound wasn't passed and saturate the torque bound
		checkTorque(tau, robot_state.tau_J_d);
		// Send the input
		sendInput(tau);

		last_z = state.z;

} // mandatory



// Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
// Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
// Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
// Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
// Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
// Eigen::Vector3d position(transform.translation());
// Eigen::Quaterniond orientation(transform.linear());


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
	
	//Check column Major!
	// for(int i = 0; i < 7; i++){
	// 	for(int j = 0; j < 7; j++){
	// 		mass(j, i) = mass_array[j*7 + i];
	// 	}
	// }

	return mass;
}

Eigen::MatrixXd Panda::Psi(){
	// std::array<double, 42> jacobian = model_handle_->getZeroJacobian(
	// 					franka::Frame::kEndEffector);

	// Eigen::MatrixXd result(6, 7);
	
	// for(int i = 0; i < 7; i++){
	// 	for(int j = 0; j < 6; j++){

	// 		//Column major!
	// 		result(j, i) = jacobian[j*7 + i];
	// 	}
	// }

	// // Account for Psi definition
	// return result.transpose();
	std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(
	 					franka::Frame::kEndEffector);

	Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(
						jacobian_array.data());

	return jacobian.transpose();
}


/* From Franka Emika: Saturates the torque rate (not torque itself)*/
std::array<double, 7> Panda::saturateTorqueRate(
    const Eigen::VectorXd& torques,
    const std::array<double, 7>& tau_J_d) {

  std::array<double, 7> tau_d_saturated{};

  for (size_t i = 0; i < 7; i++) {
    double difference = torques[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }

  return tau_d_saturated;
}

std::array<double, 7> Panda::checkTorque(const Eigen::VectorXd& torques, const std::array<double, 7>& tau_J_d){
	
	// If one of the torques is out of bounds, throw an error
	for(int i = 0; i < 7; i++){
		if(std::abs(torques[i]) > torque_bound){
			throw TorqueBoundException("Panda: Torque "
				 + std::to_string(i) + " out of bounds! (bound=" 
				 + std::to_string(torque_bound) + ", value= " 
				 + std::to_string(torques[i]) + ")");
		}
	}

	// Otherwise saturate the torque rate and return
	return saturateTorqueRate(torques, tau_J_d);
}


Eigen::VectorXd Panda::dVdq(){
	return helpers::arrayToVector<7>(model_handle_->getGravity());
}

}

// Implementation ..
PLUGINLIB_EXPORT_CLASS(panda::Panda,
                       controller_interface::ControllerBase)