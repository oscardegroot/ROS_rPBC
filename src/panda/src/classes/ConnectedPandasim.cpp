#include "ConnectedPandasim.h"
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

namespace panda{

ConnectedPandasim::ConnectedPandasim()
{
	logMsg("Connected Pandasim", "Initialising...", 2);
    
	this->setState(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(3));
    state_previous = {Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(3)};
    new_z = Eigen::VectorXd(3);
    new_z(2) = 1.0;
    
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
    
	for(int i = 0; i < 7; i++){
        std_msgs::Float64 msg;
		msg.data = tau(i, 0);
		tau_pubs[i].publish(msg);
	}

	return true;
}

void ConnectedPandasim::retrieveMatrices()
{
    // Compute matrices with the retrieved states only, otherwise keep them as zero
    if(!received_first_state){
        return;
    }
    
    if(initial_matrices)
    {
        m_previous = m_m;
        psi_previous = psi;
    }
    
    /* Convert q to an array*/
    std::array<double, 7> q_array;
    for(int i = 0; i < 7; i++){
        q_array[i] = state.q(i);
    }
    
    /* Convert qdot to an array*/
    std::array<double, 7> dq_array;
    for(int i = 0; i < 7; i++){
        dq_array[i] = state.dq(i);
    }
    
        /* Clean this up ofc */
    std::array<double, 16> identity_16 = {};
    identity_16[0] = 1.0;
    identity_16[5] = 1.0;
    identity_16[10] = 1.0;
    identity_16[15] = 1.0;
    
    
    std::array<double, 3> zeros_3 = {0.0, 0.0, 0.0};

    // Het lijkt erop dat deze niet klopt in de franka library, ze roepen gewoon *(robot_state) aan ipv mijn argumentjes.
	std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector, q_array, identity_16, identity_16);
	Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
	psi = selectPsi(jacobian.transpose());
    
	// Get the mass matrix
    std::array<double, 9> zeros = {};
	std::array<double, 49> mass_array = model_handle_->getMass(q_array, zeros, 0.0, zeros_3);
	m_m = Eigen::Map<const Eigen::Matrix<double, 7, 7> >(mass_array.data());
    
    // Get the coriolis vector
	std::array<double, 7> coriolis_array = model_handle_->getCoriolis(q_array, dq_array, zeros, 0.0, zeros_3);
	c_m = helpers::arrayToVector<7>(coriolis_array);
    
    // Set initial matrices for the first loop
    if(!initial_matrices){
        m_previous = m_m;
        psi_previous = psi;
        initial_matrices = true;
        state_previous = state; // So that the first point is 0.0

    }    
    //Get the gravity vector
    //dvdq = helpers::arrayToVector<7>(model_handle_->getGravity(q_array, 0.0, zeros_3)); Should be right
    
    dVdq();
    
    
    //logTmp(dvdq);
    psi_updated = true;
    m_updated = true;
    dvdq_updated = true;
}

Eigen::VectorXd& ConnectedPandasim::dVdq(){

    Eigen::VectorXd& q = state.q;

    dvdq[0] = 0;
    dvdq[1] = 0.000301499*cos(q(1)) - 34.2264*sin(q(1)) - 7.38648*cos(q(1))*cos(q(2)) - 0.00645473*cos(q(1))*sin(q(2)) - 17.5852*cos(q(3))*sin(q(1)) + 5.3042*sin(q(1))*sin(q(3)) + 1.22861*sin(q(1))*sin(q(3))*sin(q(4)) + 5.3042*cos(q(1))*cos(q(2))*cos(q(3)) + 17.5852*cos(q(1))*cos(q(2))*sin(q(3)) + 1.22861*cos(q(1))*cos(q(4))*sin(q(2)) - 0.0997992*cos(q(3))*cos(q(5))*sin(q(1)) - 0.02195*cos(q(1))*sin(q(2))*sin(q(4)) + 0.02195*cos(q(4))*sin(q(1))*sin(q(3)) - 1.12787*cos(q(3))*sin(q(1))*sin(q(5)) + 0.02195*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) + 1.22861*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4)) + 0.0997992*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 1.12787*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 1.12787*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 1.12787*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.0997992*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.0997992*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) - 1.12787*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.0997992*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
    dvdq[2] = 7.38648*sin(q(1))*sin(q(2)) - 0.00645473*cos(q(2))*sin(q(1)) - 17.5852*sin(q(1))*sin(q(2))*sin(q(3)) + 1.22861*cos(q(2))*cos(q(4))*sin(q(1)) - 5.3042*cos(q(3))*sin(q(1))*sin(q(2)) - 0.02195*cos(q(2))*sin(q(1))*sin(q(4)) - 0.02195*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2)) + 1.12787*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(4)) - 1.22861*cos(q(3))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.0997992*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(3)) - 0.0997992*cos(q(2))*sin(q(1))*sin(q(4))*sin(q(5)) - 1.12787*sin(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) + 1.12787*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.0997992*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5));
    dvdq[3] = 17.5852*cos(q(2))*cos(q(3))*sin(q(1)) - 17.5852*cos(q(1))*sin(q(3)) - 0.02195*cos(q(1))*cos(q(3))*cos(q(4)) - 5.3042*cos(q(1))*cos(q(3)) - 1.22861*cos(q(1))*cos(q(3))*sin(q(4)) - 0.0997992*cos(q(1))*cos(q(5))*sin(q(3)) - 5.3042*cos(q(2))*sin(q(1))*sin(q(3)) - 1.12787*cos(q(1))*sin(q(3))*sin(q(5)) + 1.12787*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.0997992*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1)) - 0.0997992*cos(q(1))*cos(q(3))*cos(q(4))*sin(q(5)) - 0.02195*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) + 1.12787*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) - 1.22861*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4)) + 1.12787*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.0997992*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5));
    dvdq[4] = 0.02195*cos(q(1))*sin(q(3))*sin(q(4)) - 1.22861*cos(q(1))*cos(q(4))*sin(q(3)) - 0.02195*cos(q(4))*sin(q(1))*sin(q(2)) - 1.22861*sin(q(1))*sin(q(2))*sin(q(4)) + 1.22861*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1)) - 0.02195*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)) + 1.12787*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 1.12787*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) - 0.0997992*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5)) + 0.0997992*cos(q(1))*sin(q(3))*sin(q(4))*sin(q(5)) + 1.12787*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.0997992*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(5));
    dvdq[5] = 1.12787*cos(q(1))*cos(q(3))*cos(q(5)) - 0.0997992*cos(q(1))*cos(q(3))*sin(q(5)) - 0.0997992*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 1.12787*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 1.12787*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.0997992*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.0997992*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 1.12787*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.0997992*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 1.12787*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
    dvdq[6] = 0;
    return dvdq;
}

void ConnectedPandasim::retrieveState()
{
    retrieveMatrices();
}

void ConnectedPandasim::readStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    received_first_state = true;
    std::mutex mtx;
    mtx.lock();
    state_previous = state;

    resetUpdatedFlags();
    
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

    // Calculate inverse rotations
    // See https://en.wikibooks.org/wiki/Robotics_Kinematics_and_Dynamics/Description_of_Position_and_Orientation (inverse mapping z,x,z)
//    double roll = std::atan2(robot_pose[6], robot_pose[10]);
//    double yaw = std::atan2(robot_pose[1], robot_pose[0]);
//    double pitch = std::atan2(-robot_pose[2], std::cos(yaw)*robot_pose[0] + std::sin(yaw)*robot_pose[1]);
    // Keep only the translation
	std::array<double, 3> z = {robot_pose[12], robot_pose[13], robot_pose[14]};//, roll, pitch, yaw};
	Eigen::Matrix<double, 3, 1> new_z = helpers::arrayToVector<3>(z);
    
    // Write to the state
    new_z = Eigen::VectorXd(3);
    new_z << new_z[0], new_z[1], new_z[2];
    setState(new_q, new_qd, new_z);
    mtx.unlock();
}

};
PLUGINLIB_EXPORT_CLASS(panda::ConnectedPandasim,
                       controller_interface::ControllerBase)