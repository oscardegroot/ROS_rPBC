
#include "PandaSim.h"

//See: https://github.com/frankaemika/franka_ros/blob/kinetic-devel/franka_example_controllers/include/franka_example_controllers/joint_impedance_example_controller.h

PandaSim::PandaSim()
	:System(7, 7)
{
	cout << "[PandaSim] Initialising PandaSim system.. ";
	this->setState(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n));
	
	for(int i = 0; i < n; i++){
		tau_pubs[i] = nh.advertise<std_msgs::Float64>("/robot1/panda_joint" +
													  to_string(i+1) + 
													  "1_controller/command", 20);
	}
	
	
    sensor_sub = nh.subscribe("/robot1/joint_states", 1, &PandaSim::readStateCallback, this);

    DH_a << 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0;
    DH_d << 0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.107;
    DH_alpha << M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, M_PI_2, M_PI_2, 0;

    for(int i = 0; i < 7; i++){
    	L_cogs[i] = Eigen::Matrix<double, 3, 1>::Zero();
    }

  	cout << "done.\n";
}

// Callback for reading the states
void PandaSim::readStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
	
	Eigen::Matrix<double, 7, 1> new_q, new_qd;

	for( int i = 0; i < 7; i++ ) {
      new_q(i) = msg->position[i];
      new_qd(i) = msg->velocity[i];
    }	

    this->setState(new_q, new_qd);
    this->setDataReady(true);
}

Eigen::VectorXd PandaSim::readSensors(){
	return Eigen::VectorXd::Zero(n);

}

bool PandaSim::sendInput(Eigen::VectorXd tau){
	
	std_msgs::Float64 msg;

	for(int i = 0; i < n; i++){
		msg.data = tau(i);
		tau_pubs[i].publish(msg);
	}

	return true;
}

// Get the mass matrix at q (IMPLEMENT)
Eigen::MatrixXd PandaSim::getM(){

	// Hardware wise or theoretic?
	return Eigen::MatrixXd::Zero(n, n);

}

Eigen::VectorXd PandaSim::getdHdq(){


	return getdVdq();

}

Eigen::VectorXd PandaSim::getdVdq(){
	//Eigen::Matrix<double, 3, 1> z_cog, dVdq;
	//dVdq = Eigen::Matrix<double, 7, 1>::Zero();


	/* Calculates V(q) not dVdq(q)!!!*/
	// // For every link
	// for(int i = 0; i < 7; i++){
		
	// 	// Get the transformation to the end of the link
	// 	z_cog = getCOG(i, this->state.q);

	// 	// Add to the dVdq m*g*z
	// 	dVdq = dVdq + z_cog * mass[i] * g;
	// }
	return Eigen::Matrix<double, 7, 1>::Zero();
}

//Link count >= 1
Eigen::Vector3d PandaSim::getCOG(int i, Eigen::Matrix<double, 7, 1> q){

	// Transform to the i-1th link 
	Eigen::Matrix<double,4, 4> T_im1 = getTi(i - 1, q);

	//Transform to the cog of the next link
	Eigen::Matrix<double, 3, 1> O_cog;
	O_cog << cos(q(i))*L_cogs[i](0) - sin(q(i))*L_cogs[i](1), sin(q(i))*L_cogs[i](0) + cos(q(i))*L_cogs[i](1), L_cogs[i](2);
	O_cog = O_cog + T_im1.block(0, 6, 3, 1);

	return O_cog;
}

Eigen::Vector3d PandaSim::getEEPose(Eigen::Matrix<double, 7, 1> q){

	return (Eigen::Vector3d)getTi(7, q).block(0, 6, 3, 1);
}

// Compute the direct kinematics (Assume for now that the base is at the origin at 0 degrees)
Eigen::Matrix<double,4, 4> PandaSim::getTi(int i, Eigen::Matrix<double, 7, 1> q){

	// Initialize transformation matrix for the end effector DH_T
	Eigen::Matrix<double, 4, 4> DH_T = Eigen::Matrix<double, 4, 4>::Identity();
	Eigen::Matrix<double, 4, 4> DH_A;

	// Compute DK
	q(1) = -q(1); /*Is the second link inverted??*/
	for(int k=0; k<i; k++){
	  	DH_A << cos(q(k)), -sin(q(k))*cos(DH_alpha(k)), sin(q(k))*sin(DH_alpha(k)), DH_a(k)*cos(q(k)),
	          	sin(q(k)), cos(q(k))*cos(DH_alpha(k)), -cos(q(k))*sin(DH_alpha(k)), DH_a(k)*sin(q(k)),
	               0.0,                sin(DH_alpha(k)),              cos(DH_alpha(k)),               DH_d(k),
	               0.0,                      0.0,                            0.0,                      1.0;
  		// Trasfrmation to the joint
  		DH_T = DH_A*DH_T;
	}

	return DH_T;
}