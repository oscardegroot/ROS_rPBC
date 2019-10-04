#include "ConnectedPandasim.h"
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

/** @todo I probably need to do the exporting etc. here too. */
namespace panda{


ConnectedPandasim::ConnectedPandasim()
 //   	:System(7, 7, 3, "connected_pandasim")
{
	logMsg("Connected Pandasim", "Initialising...", 2);
    
	this->setState(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(3));
    
	for(int i = 0; i < n; i++){
		tau_pubs[i] = nh.advertise<std_msgs::Float64>("/robot1/panda_joint" +
                    to_string(i+1) + "_controller/command", 100);
	}
	
    sensor_sub = nh.subscribe("/robot1/joint_states", 100, &ConnectedPandasim::readStateCallback, this);
    
}

bool ConnectedPandasim::sendInput(const Eigen::VectorXd& tau)
{
    
    /* Set 0 input */
	for (size_t i = 0; i < 7; ++i) {
		joint_handles_[i].setCommand(0.0);
	}
    
    std_msgs::Float64 msg;

	for(int i = 0; i < n; i++){
		msg.data = tau(i);
		tau_pubs[i].publish(msg);
	}

	return true;
}

void ConnectedPandasim::retrieveMatrices()
{

    std::array<double, 7> q_array;
    for(int i = 0; i < 7; i++){
        q_array[i] = state.q(i);
    }
    
        /* Clean this up ofc */
    std::array<double, 16> identity_16 = {};
    identity_16[0] = 1.0;
    identity_16[5] = 1.0;
    identity_16[10] = 1.0;
    identity_16[15] = 1.0;
    
    /*This really doesnt make sense!*/
    std::array<double, 3> zeros_3 = {0.0, 0.0, 0.0};
//    zeros_3[0] = 1.0;
//    zeros_3[4] = 1.0;
//    zeros_3[8] = 1.0;
    
    // Get the Jacobian of the EE
    // Het lijkt erop dat deze niet klopt in de franka library, ze roepen gewoon *(robot_state) aan ipv mijn argumentjes.
	std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector, q_array, identity_16, identity_16);
	Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
	psi = jacobian.transpose().block(0, 0, 7, 3);
    
	// Get the mass matrix
    std::array<double, 9> zeros = {};
	std::array<double, 49> mass_array = model_handle_->getMass(q_array, zeros, 0.0, zeros_3);
	m_m = Eigen::Map<const Eigen::Matrix<double, 7, 7> >(mass_array.data());
    //logTmp("Mass: ", m);
    
    //Get the gravity vector
    dvdq = helpers::arrayToVector<7>(model_handle_->getGravity(q_array, 0.0, zeros_3));

    psi_updated = true;
    m_updated = true;
    dvdq_updated = true;
}

void ConnectedPandasim::retrieveState()
{
    retrieveMatrices();
}

void ConnectedPandasim::readStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    /* Retrieve q, qd */
    Eigen::Matrix<double, 7, 1> new_q, new_qd;

	for( int i = 0; i < 7; i++ ) {
      new_q(i) = msg->position[i];
      new_qd(i) = msg->velocity[i];
    }
    
    /* Convert q to an array*/
    std::array<double, 7> q_array;
    for(int i = 0; i < 7; i++){
        q_array[i] = new_q(i);
    }
    
    /* No transformation = identity for 4x4 pose */
    std::array<double, 16> identity_16 = {};
    identity_16[0] = 1.0;
    identity_16[5] = 1.0;
    identity_16[10] = 1.0;
    identity_16[15] = 1.0;
    
    // Get the pose of the end effector
    std::array<double, 16> robot_pose = model_handle_->getPose(franka::Frame::kEndEffector, q_array, identity_16, identity_16);

    // Keep only the translation
	std::array<double, 3> z = {robot_pose[12], robot_pose[13], robot_pose[14]};
	Eigen::Matrix<double, 3, 1> new_z = helpers::arrayToVector<3>(z);

    // Write to the state
    setState(new_q, new_qd, new_z);
}

};
PLUGINLIB_EXPORT_CLASS(panda::ConnectedPandasim,
                       controller_interface::ControllerBase)