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
	:System(7, 7, 6, "panda")
{

}

Panda::~Panda(){};


bool Panda::init (hardware_interface::RobotHW* hw, ros::NodeHandle& nh){
	
    PROFILE_FUNCTION();
	logMsg("Panda", "Panda Controller Started!", 2);

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

	cmm->agent->retrieveArray("joint_names", joint_names, 7);
	cmm->agent->retrieveParameter("arm_id", arm_id);

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
	cmm->agent->retrieveParameter("velocity_norm_bound", velocity_norm_bound, 0.4);
	cmm->agent->retrieveParameter("velocity_element_bound", velocity_element_bound, 0.3);
	cmm->agent->retrieveParameter("safety/lower_bound_z", z_lower_bound, 0.2);
    cmm->agent->retrieveParameter("safety/torque_bound_stop", torque_bound_stop, 80.0);
	cmm->agent->retrieveParameter("torque_bound", torque_bound, 30.0);
	cmm->agent->retrieveParameter("alpha", alpha, 0.99);
	cmm->agent->retrieveParameter("initial_pause", initial_pause, 0.0);
    cmm->agent->retrieveParameter("safety/camera_bound/x", camera_bound_x, -30.0);
	cmm->agent->retrieveParameter("safety/camera_bound/z", camera_bound_z, 20.0);


	// Initialise the controller
    std::string output;
    cmm->agent->retrieveParameter("/output", output);

    if(output == "z"){
        controller = std::make_unique<IDAPBC>(*(cmm->agent));
    }else{
        controller = std::make_unique<rPBC>(*(cmm->agent));
    }

    dq_filtered = Eigen::VectorXd::Zero(n);
    
	// Get an initial state reading
	robot_state = cartesian_pose_handle_->getRobotState();
    
    // Perform handshake once the controller has been initialised
    cmm->performHandshake();
    
	retrieveState();
    retrieveMatrices();
        
	logMsg("Panda", "Initialisation Completed!", 2);

	return true;
}

/* Function to check if bounds are not exceeded 
	Throws an error if they are!
*/
void Panda::checkSafety(){

    PROFILE_FUNCTION();

	// Check if the panda is not below its lower bound
	if(new_z(2) < z_lower_bound){
		throw BoundException(
			"Panda: Panda fell below lower bound! (z = " + std::to_string(new_z(2)) +
            ", z_bound=" + std::to_string(z_lower_bound) + ")");
	}

		// Check if the velocity of the EE surpassed its bound
	if((this->state.z - last_z).norm() > velocity_norm_bound){
		throw EEVelocityBoundException(
			"Panda: Panda EE velocity higher the velocity bound! (" + 
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
    
    /* Safety check for camera placement */
    if(new_z(2) >= camera_bound_z && new_z(0) <= camera_bound_x){
        throw BoundException( "Panda: Camera bound was passed! Robot was brought to a save stop.\n Robot coordinates: (x=" +
            std::to_string(new_z(0)) + ", z=" + std::to_string(new_z(2)) + ")\n Camera bound: (x=" + 
            std::to_string(camera_bound_x) + ", z=" + std::to_string(camera_bound_z) + ")");
    }
}


void Panda::retrieveState(){

    PROFILE_FUNCTION();

    // Retrieve q
	Eigen::VectorXd new_q = helpers::arrayToVector<7>(robot_state.q);
    
    // Retrieve dq and filter it
    Eigen::VectorXd new_dq = helpers::arrayToVector<7>(robot_state.dq);

    helpers::lowpassFilter(dq_filtered, new_dq, alpha);

    // Calculate inverse rotations
    // See https://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation (inverse mapping z,x,z)
    //double alpha = std::atan2(robot_state.O_T_EE[8], -robot_state.O_T_EE[9]);
    //double beta = std::atan2(-robot_state.O_T_EE[9]*std::cos(alpha) + robot_state.O_T_EE[8]*std::sin(alpha), robot_state.O_T_EE[10]);
    //double gamma = std::atan2(robot_state.O_T_EE[2], robot_state.O_T_EE[6]);

    /* The angles dont work in this manner, so they are disabled */
	std::array<double, 3> z{{robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]}};//, alpha, beta, gamma}};
    new_z = helpers::arrayToVector<3>(z);

    // Save the new coordinates and select the required cooperative coordinates
    setState(new_q, dq_filtered, new_z);

}

void Panda::starting(const ros::Time& /*time*/) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  start_time = ros::Time::now();
}

void Panda::update (const ros::Time& time, const ros::Duration& period){

    //benchmarker.start(); // ~100 us!
    PROFILE_FUNCTION();

	// Retrieve the robot state
	robot_state = cartesian_pose_handle_->getRobotState();
	cartesian_pose_handle_->setCommand(initial_pose_);

    // We have to compute all matrices again
    resetUpdatedFlags();

	// Retrieve robot state
	retrieveState();
    
    // Retrieve robot matrices
    retrieveMatrices();
    
	// Check for errors
	checkSafety();

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(m);

	if(time - start_time > ros::Duration(initial_pause)){

        // Sample the network
        Eigen::VectorXd tau_network = cmm->sample(controller->getOutput(*this));

		// Calculate the control input
		tau = controller->computeControl((*this), tau_network);
		
		// Check if the torque bound wasn't passed and saturate the torque bound
		checkTorque(tau, robot_state.tau_J_d);

        // Save the last state for velocity safety
		last_z = state.z;
	}
    
    // Send the input
    sendInput(tau);

} // mandatory

bool Panda::sendInput(const Eigen::VectorXd& tau){
    PROFILE_FUNCTION();

    for (size_t i = 0; i < 7; ++i) {
		joint_handles_[i].setCommand(tau[i]);
	}
}

void Panda::retrieveMatrices(){
    PROFILE_FUNCTION();

    // If this is not the initial run, save the previous run 
    if(initial_matrices){
        
        m_previous = m_m;
        psi_previous = psi;
    }
    
	// Get the Jacobian of the EE
	std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

	Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
	psi = selectPsi(jacobian.transpose());

	// Get the mass matrix
	std::array<double, 49> mass_array = model_handle_->getMass();
	m_m = Eigen::Map<const Eigen::Matrix<double, 7, 7> >(mass_array.data());
    
    c_m = helpers::arrayToVector<7>(model_handle_->getCoriolis());
    
    //Get the gravity vector
    dvdq = helpers::arrayToVector<7>(model_handle_->getGravity());
    
    // If this is the initial run, initialise the previous matrix values to the current ones such that the numerical derivative is zero.
    if(!initial_matrices){
        
        m_previous = m_m;
        psi_previous = psi;
        initial_matrices = true;
    }
}

Eigen::MatrixXd& Panda::M(){
    
	return m_m;
}


Eigen::MatrixXd& Panda::Psi(){
    
	return psi;
}

Eigen::VectorXd& Panda::C(){
    return c_m;
}

/* From Franka Emika: Saturates the torque rate (not torque itself)*/
std::array<double, 7> Panda::saturateTorqueRate(const Eigen::VectorXd& torques, const std::array<double, 7>& tau_J_d) {

    std::array<double, 7> tau_d_saturated{};

    for (size_t i = 0; i < 7; i++) {
        
        double difference = torques[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
    }

    return tau_d_saturated;
}

std::array<double, 7> Panda::checkTorque(Eigen::VectorXd& torques, const std::array<double, 7>& tau_J_d){
	
    
	// If one of the torques is out of bounds, throw an error
	for(int i = 0; i < 7; i++){
        
        if(!std::isfinite(torques[i])){
            throw OperationalException("Torque " + std::to_string(i) + " is not finite (NaN or inf)!");
        }

        
		if(std::abs(torques[i]) > torque_bound){
            std::string message = "Panda: Torque "
				 + std::to_string(i) + " out of bounds! (bound=" 
				 + std::to_string(torque_bound) + ", value= " 
				 + std::to_string(torques[i]) + ")" + "\n" + 
                 "All torques: (";
            
            for(int i = 0; i < 7; i ++){
                message += std::to_string(torques[i]);
                if(i < 6){
                    message += ", ";
                }
                
                
            }
            message += ")";
			throw TorqueBoundException(message);
        }
    }

	// Otherwise saturate the torque rate and return
	return saturateTorqueRate(torques, tau_J_d);
}

Eigen::VectorXd& Panda::dVdq(){
    
    return dvdq;
}


/** @brief Approximate matrix derivatives by finite distance approximation */
Eigen::MatrixXd& Panda::dMinv(){
    
    PROFILE_FUNCTION();

    if(!dminv_updated)
    {
        // Use dMinv = -Minv*dM*Minv
        Eigen::MatrixXd dminv_input = -(M().inverse())*dM()*(M().inverse());
        helpers::lowpassFilter(dminv, dminv_input, alpha);
        dminv_updated = true;
    }

    return dminv;
}

Eigen::MatrixXd& Panda::dM(){
    
    PROFILE_FUNCTION();
    
    if(!dm_updated){
        Eigen::MatrixXd dm_input = this->approximateDerivative(M(), m_previous);
        helpers::lowpassFilter(dm, dm_input, alpha);
        dm_updated = true;
    }

    return dm;
}

Eigen::MatrixXd& Panda::dPsi(){
    
    PROFILE_FUNCTION();

    if(!dpsi_updated){
        Eigen::MatrixXd dpsi_input = this->approximateDerivative(Psi(), psi_previous);
        helpers::lowpassFilter(dpsi, dpsi_input, alpha);
        dpsi_updated = true;
    }

    return dpsi;
}

};
PLUGINLIB_EXPORT_CLASS(panda::Panda,
                       controller_interface::ControllerBase)