
#include "PandaSim.h"

//See: https://github.com/frankaemika/franka_ros/blob/kinetic-devel/franka_example_controllers/include/franka_example_controllers/joint_impedance_example_controller.h

PandaSim::PandaSim()
	:System(7, 7, 3)
{
	logMsg("PandaSim", "Initialising...", 2);
	this->setState(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n),
		getZ(Eigen::VectorXd::Zero(n)));


	for(int i = 0; i < n; i++){
		tau_pubs[i] = nh.advertise<std_msgs::Float64>("/robot1/panda_joint" +
													  to_string(i+1) + 
													  "_controller/command", 100);
	}
	
    sensor_sub = nh.subscribe("/robot1/joint_states", 100, &PandaSim::readStateCallback, this);
    //this->setAgent(nh, "pandasim");

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
    //this->setDataReady(true);
}


bool PandaSim::sendInput(const Eigen::VectorXd & tau){
	
	std_msgs::Float64 msg;

	for(int i = 0; i < n; i++){
		msg.data = tau(i);
		tau_pubs[i].publish(msg);
	}

	return true;
}

// Get the mass matrix at q (IMPLEMENT)
void PandaSim::M(Eigen::MatrixXd & M_out){

	// Hardware wise or theoretic?
	M_out = Eigen::MatrixXd::Zero(n, n);

}

void PandaSim::dVdq(Eigen::VectorXd & dVdq_out){

	dVdq_out = Eigen::Matrix<double, 7, 1>::Zero();
	Eigen::Matrix<double, 7, 1> q = this->state.q;

	dVdq_out[0] = 0;
	dVdq_out[1] = 0.00012753*cos(q(1)) - 13.4638*sin(q(1)) - 2.86303*cos(q(1))*cos(q(2)) + 0.00207972*cos(q(1))*sin(q(2)) - 6.84302*cos(q(3))*sin(q(1)) + 1.99683*sin(q(1))*sin(q(3)) + 0.316304*sin(q(1))*sin(q(3))*sin(q(4)) + 1.99683*cos(q(1))*cos(q(2))*cos(q(3)) + 6.84302*cos(q(1))*cos(q(2))*sin(q(3)) + 0.316304*cos(q(1))*cos(q(4))*sin(q(2)) - 0.0680127*cos(q(3))*cos(q(5))*sin(q(1)) - 0.0062784*cos(q(1))*sin(q(2))*sin(q(4)) + 0.0062784*cos(q(4))*sin(q(1))*sin(q(3)) - 0.50033*cos(q(3))*sin(q(1))*sin(q(5)) + 0.0062784*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) + 0.316304*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4)) + 0.0680127*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 0.50033*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 0.50033*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.50033*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.0680127*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.0680127*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.50033*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.0680127*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
	dVdq_out[2] = 0.00207972*cos(q(2))*sin(q(1)) + 2.86303*sin(q(1))*sin(q(2)) - 6.84302*sin(q(1))*sin(q(2))*sin(q(3)) + 0.316304*cos(q(2))*cos(q(4))*sin(q(1)) - 1.99683*cos(q(3))*sin(q(1))*sin(q(2)) - 0.0062784*cos(q(2))*sin(q(1))*sin(q(4)) - 0.0062784*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2)) + 0.50033*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.316304*cos(q(3))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.0680127*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(3)) - 0.0680127*cos(q(2))*sin(q(1))*sin(q(4))*sin(q(5)) - 0.50033*sin(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.50033*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.0680127*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5));
	dVdq_out[3] = 6.84302*cos(q(2))*cos(q(3))*sin(q(1)) - 6.84302*cos(q(1))*sin(q(3)) - 0.0062784*cos(q(1))*cos(q(3))*cos(q(4)) - 1.99683*cos(q(1))*cos(q(3)) - 0.316304*cos(q(1))*cos(q(3))*sin(q(4)) - 0.0680127*cos(q(1))*cos(q(5))*sin(q(3)) - 1.99683*cos(q(2))*sin(q(1))*sin(q(3)) - 0.50033*cos(q(1))*sin(q(3))*sin(q(5)) + 0.50033*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.0680127*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1)) - 0.0680127*cos(q(1))*cos(q(3))*cos(q(4))*sin(q(5)) - 0.0062784*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) + 0.50033*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.316304*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4)) + 0.50033*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.0680127*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5));
	dVdq_out[4] = 0.0062784*cos(q(1))*sin(q(3))*sin(q(4)) - 0.316304*cos(q(1))*cos(q(4))*sin(q(3)) - 0.0062784*cos(q(4))*sin(q(1))*sin(q(2)) - 0.316304*sin(q(1))*sin(q(2))*sin(q(4)) + 0.316304*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1)) - 0.0062784*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)) + 0.50033*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.50033*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) - 0.0680127*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5)) + 0.0680127*cos(q(1))*sin(q(3))*sin(q(4))*sin(q(5)) + 0.50033*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.0680127*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(5));
	dVdq_out[5] = 0.50033*cos(q(1))*cos(q(3))*cos(q(5)) - 0.0680127*cos(q(1))*cos(q(3))*sin(q(5)) - 0.0680127*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.50033*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.50033*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.0680127*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.0680127*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.50033*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.0680127*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.50033*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
}

Eigen::VectorXd PandaSim::getZ(Eigen::VectorXd q)
{
	Eigen::Vector3d z;

	// z as calculated in Matlab
	z[0] = 0.316*cos(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(2)) + 0.384*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.384*cos(q(0))*cos(q(3))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.088*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
	z[1] = 0.0825*cos(q(0))*sin(q(2)) + 0.316*sin(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(3))*sin(q(2)) + 0.384*cos(q(3))*sin(q(0))*sin(q(1)) - 0.384*cos(q(0))*sin(q(2))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.088*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4)) - 0.384*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) + 0.088*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5)) - 0.088*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.088*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) + 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) + 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0));
	z[2] = 0.316*cos(q(1)) + 0.384*cos(q(1))*cos(q(3)) - 0.0825*cos(q(2))*sin(q(1)) - 0.0825*cos(q(1))*sin(q(3)) + 0.0825*cos(q(2))*cos(q(3))*sin(q(1)) + 0.088*cos(q(1))*cos(q(3))*sin(q(5)) + 0.384*cos(q(2))*sin(q(1))*sin(q(3)) + 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.088*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.333;
	return z;
}

// Define dzdq
void PandaSim::Psi(Eigen::MatrixXd & Psi_out){
    
	Psi_out = Eigen::MatrixXd::Zero(n, 3);
	Eigen::Matrix<double, 7, 1> q = state.q;
	Psi_out(0, 0) = 0.0825*sin(q(0))*sin(q(1))*sin(q(3)) - 0.316*sin(q(0))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(2)) - 0.0825*cos(q(1))*cos(q(2))*sin(q(0)) + 0.0825*cos(q(0))*cos(q(3))*sin(q(2)) - 0.384*cos(q(3))*sin(q(0))*sin(q(1)) + 0.384*cos(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) - 0.088*cos(q(0))*cos(q(2))*cos(q(5))*sin(q(4)) + 0.384*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.088*cos(q(3))*sin(q(0))*sin(q(1))*sin(q(5)) + 0.088*cos(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2)) + 0.088*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0));
	Psi_out(0, 1) = 0.316*cos(q(0))*sin(q(1)) - 0.0825*sin(q(0))*sin(q(2)) + 0.384*sin(q(0))*sin(q(2))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2)) + 0.384*cos(q(0))*cos(q(3))*sin(q(1)) - 0.0825*cos(q(0))*sin(q(1))*sin(q(3)) + 0.0825*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.088*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) + 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
	Psi_out(0, 2) = 0;
	Psi_out(1, 0) = 0.316*cos(q(0))*cos(q(1)) + 0.384*cos(q(0))*cos(q(1))*cos(q(3)) - 0.0825*cos(q(0))*cos(q(2))*sin(q(1)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(2))*cos(q(3))*sin(q(1)) + 0.088*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(5)) + 0.384*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3)) + 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.088*cos(q(0))*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1));
	Psi_out(1, 1) = 0.316*cos(q(1))*sin(q(0)) + 0.384*cos(q(1))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(2))*sin(q(0))*sin(q(1)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(1)) + 0.088*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.384*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3)) + 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1));
	Psi_out(1, 2) = 0.0825*sin(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(2)) - 0.384*cos(q(3))*sin(q(1)) - 0.316*sin(q(1)) + 0.0825*cos(q(1))*cos(q(2))*cos(q(3)) + 0.384*cos(q(1))*cos(q(2))*sin(q(3)) - 0.088*cos(q(3))*sin(q(1))*sin(q(5)) + 0.088*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.088*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5));
	Psi_out(2, 0) = 0.0825*cos(q(2))*cos(q(3))*sin(q(0)) - 0.0825*cos(q(0))*cos(q(1))*sin(q(2)) - 0.0825*cos(q(2))*sin(q(0)) + 0.384*cos(q(2))*sin(q(0))*sin(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(3))*sin(q(2)) + 0.384*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(3))*sin(q(5)) + 0.088*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(4)) - 0.088*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0)) + 0.088*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(2));
	Psi_out(2, 1) = 0.0825*cos(q(0))*cos(q(2)) - 0.0825*cos(q(0))*cos(q(2))*cos(q(3)) - 0.384*cos(q(0))*cos(q(2))*sin(q(3)) - 0.0825*cos(q(1))*sin(q(0))*sin(q(2)) + 0.0825*cos(q(1))*cos(q(3))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(4)) + 0.384*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.088*cos(q(0))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(4)) + 0.088*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2));
	Psi_out(2, 2) = 0.0825*sin(q(1))*sin(q(2)) - 0.384*sin(q(1))*sin(q(2))*sin(q(3)) - 0.0825*cos(q(3))*sin(q(1))*sin(q(2)) + 0.088*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.088*sin(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2));
	Psi_out(3, 0) = 0.384*cos(q(3))*sin(q(0))*sin(q(2)) - 0.0825*cos(q(0))*cos(q(3))*sin(q(1)) - 0.384*cos(q(0))*sin(q(1))*sin(q(3)) - 0.0825*sin(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3)) + 0.0825*cos(q(0))*cos(q(1))*cos(q(2))*sin(q(3)) - 0.088*cos(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(5)) + 0.088*cos(q(0))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.088*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(3));
	Psi_out(3, 1) = 0.0825*cos(q(0))*sin(q(2))*sin(q(3)) - 0.384*cos(q(0))*cos(q(3))*sin(q(2)) - 0.0825*cos(q(3))*sin(q(0))*sin(q(1)) - 0.384*sin(q(0))*sin(q(1))*sin(q(3)) - 0.384*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0)) + 0.0825*cos(q(1))*cos(q(2))*sin(q(0))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*sin(q(2))*sin(q(5)) - 0.088*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(0))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.088*cos(q(0))*cos(q(4))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.088*cos(q(1))*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(3));
	Psi_out(3, 2) = 0.384*cos(q(2))*cos(q(3))*sin(q(1)) - 0.384*cos(q(1))*sin(q(3)) - 0.0825*cos(q(1))*cos(q(3)) - 0.0825*cos(q(2))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.088*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) + 0.088*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3));
	Psi_out(4, 0) = 0.088*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(4)) - 0.088*cos(q(0))*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(2)) - 0.088*cos(q(0))*cos(q(5))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(0)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(4));
	Psi_out(4, 1) = 0.088*cos(q(0))*cos(q(2))*cos(q(4))*cos(q(5)) - 0.088*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(0))*sin(q(2)) - 0.088*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.088*cos(q(5))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(4)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(4));
	Psi_out(4, 2) = 0.088*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.088*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) + 0.088*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4));
	Psi_out(5, 0) = 0.088*cos(q(0))*cos(q(3))*cos(q(5))*sin(q(1)) + 0.088*cos(q(5))*sin(q(0))*sin(q(2))*sin(q(3)) + 0.088*cos(q(2))*sin(q(0))*sin(q(4))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 0.088*cos(q(0))*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(0))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) + 0.088*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(5)) - 0.088*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
	Psi_out(5, 1) = 0.088*cos(q(3))*cos(q(5))*sin(q(0))*sin(q(1)) - 0.088*cos(q(0))*cos(q(5))*sin(q(2))*sin(q(3)) - 0.088*cos(q(0))*cos(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(0))*sin(q(3)) - 0.088*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(2))*sin(q(5)) + 0.088*cos(q(1))*sin(q(0))*sin(q(2))*sin(q(4))*sin(q(5)) - 0.088*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.088*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(5));
	Psi_out(5, 2) = 0.088*cos(q(1))*cos(q(3))*cos(q(5)) + 0.088*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.088*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.088*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.088*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
	//std::cout << "Psi: \n" << Psi << "\n";
}


/* Old!! */
// Eigen::Vector3d PandaSim::getEEPose(Eigen::Matrix<double, 7, 1> q){

// 	return (Eigen::Vector3d)getTi(7, q).block(0, 6, 3, 1);
// }

// // Compute the direct kinematics (Assume for now that the base is at the origin at 0 degrees)
// Eigen::Matrix<double,4, 4> PandaSim::getTi(int i, Eigen::Matrix<double, 7, 1> q){

// 	// Initialize transformation matrix for the end effector DH_T
// 	Eigen::Matrix<double, 4, 4> DH_T = Eigen::Matrix<double, 4, 4>::Identity();
// 	Eigen::Matrix<double, 4, 4> DH_A;

// 	// Compute DK
// 	q(1) = -q(1); /*Is the second link inverted??*/
// 	for(int k=0; k<i; k++){
// 	  	DH_A << cos(q(k)), -sin(q(k))*cos(DH_alpha(k)), sin(q(k))*sin(DH_alpha(k)), DH_a(k)*cos(q(k)),
// 	          	sin(q(k)), cos(q(k))*cos(DH_alpha(k)), -cos(q(k))*sin(DH_alpha(k)), DH_a(k)*sin(q(k)),
// 	               0.0,                sin(DH_alpha(k)),              cos(DH_alpha(k)),               DH_d(k),
// 	               0.0,                      0.0,                            0.0,                      1.0;
//   		// Trasfrmation to the joint
//   		DH_T = DH_A*DH_T;
// 	}

// 	return DH_T;
// }