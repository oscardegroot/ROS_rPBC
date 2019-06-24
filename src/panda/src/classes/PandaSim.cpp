
#include "PandaSim.h"

#define LOG(msg) std::cout << msg << std::endl
//See: https://github.com/frankaemika/franka_ros/blob/kinetic-devel/franka_example_controllers/include/franka_example_controllers/joint_impedance_example_controller.h

PandaSim::PandaSim()
	:System(7, 7)
{
	cout << "[PandaSim] Initialising PandaSim system.. ";
	this->setState(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n));

	for(int i = 0; i < n; i++){
		tau_pubs[i] = nh.advertise<std_msgs::Float64>("/robot1/panda_joint" +
													  to_string(i+1) + 
													  "_controller/command", 20);
	}
	
    sensor_sub = nh.subscribe("/robot1/joint_states", 20, &PandaSim::readStateCallback, this);

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

	// energy gradient irrelevant for fully actuated systems
	return getdVdq();

}

Eigen::VectorXd PandaSim::getdVdq(){

	Eigen::Matrix<double, 7, 1> dV;
	Eigen::Matrix<double, 7, 1> q = this->state.q;

	// dV as calculated with DH parameters
	dV[0] = 0;
	dV[1] = 0.0001275*cos(q(1)) - 13.46*sin(q(1)) - 0.4351*cos(q(1))*cos(q(2)) + 0.00208*cos(q(1))*sin(q(2)) - 6.843*cos(q(3))*sin(q(1)) - 1.24*sin(q(1))*sin(q(3)) + 0.3163*sin(q(1))*sin(q(3))*sin(q(4)) - 1.24*cos(q(1))*cos(q(2))*cos(q(3)) + 6.843*cos(q(1))*cos(q(2))*sin(q(3)) + 0.3163*cos(q(1))*cos(q(4))*sin(q(2)) - 0.06801*cos(q(3))*cos(q(5))*sin(q(1)) - 0.8156*cos(q(1))*sin(q(2))*sin(q(4)) + 0.8156*cos(q(4))*sin(q(1))*sin(q(3)) - 0.5003*cos(q(3))*sin(q(1))*sin(q(5)) + 0.8156*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) + 0.3163*cos(q(1))*cos(q(2))*cos(q(3))*sin(q(4)) + 0.06801*cos(q(1))*cos(q(2))*cos(q(5))*sin(q(3)) + 0.5003*cos(q(1))*cos(q(2))*sin(q(3))*sin(q(5)) + 0.5003*cos(q(1))*cos(q(5))*sin(q(2))*sin(q(4)) - 0.5003*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.06801*cos(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.06801*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.5003*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.06801*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(5));
	dV[2] = 0.00208*cos(q(2))*sin(q(1)) + 0.4351*sin(q(1))*sin(q(2)) - 6.843*sin(q(1))*sin(q(2))*sin(q(3)) + 0.3163*cos(q(2))*cos(q(4))*sin(q(1)) + 1.24*cos(q(3))*sin(q(1))*sin(q(2)) - 0.8156*cos(q(2))*sin(q(1))*sin(q(4)) - 0.8156*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2)) + 0.5003*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.3163*cos(q(3))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.06801*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(3)) - 0.06801*cos(q(2))*sin(q(1))*sin(q(4))*sin(q(5)) - 0.5003*sin(q(1))*sin(q(2))*sin(q(3))*sin(q(5)) + 0.5003*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.06801*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5));
	dV[3] = 1.24*cos(q(1))*cos(q(3)) - 6.843*cos(q(1))*sin(q(3)) - 0.8156*cos(q(1))*cos(q(3))*cos(q(4)) + 6.843*cos(q(2))*cos(q(3))*sin(q(1)) - 0.3163*cos(q(1))*cos(q(3))*sin(q(4)) - 0.06801*cos(q(1))*cos(q(5))*sin(q(3)) + 1.24*cos(q(2))*sin(q(1))*sin(q(3)) - 0.5003*cos(q(1))*sin(q(3))*sin(q(5)) + 0.5003*cos(q(1))*cos(q(3))*cos(q(4))*cos(q(5)) + 0.06801*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1)) - 0.06801*cos(q(1))*cos(q(3))*cos(q(4))*sin(q(5)) - 0.8156*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) + 0.5003*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(5)) - 0.3163*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(4)) + 0.5003*cos(q(2))*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.06801*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3))*sin(q(5));
	dV[4] = 0.8156*cos(q(1))*sin(q(3))*sin(q(4)) - 0.3163*cos(q(1))*cos(q(4))*sin(q(3)) - 0.8156*cos(q(4))*sin(q(1))*sin(q(2)) - 0.3163*sin(q(1))*sin(q(2))*sin(q(4)) + 0.3163*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1)) - 0.8156*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4)) + 0.5003*cos(q(4))*cos(q(5))*sin(q(1))*sin(q(2)) - 0.5003*cos(q(1))*cos(q(5))*sin(q(3))*sin(q(4)) - 0.06801*cos(q(4))*sin(q(1))*sin(q(2))*sin(q(5)) + 0.06801*cos(q(1))*sin(q(3))*sin(q(4))*sin(q(5)) + 0.5003*cos(q(2))*cos(q(3))*cos(q(5))*sin(q(1))*sin(q(4)) - 0.06801*cos(q(2))*cos(q(3))*sin(q(1))*sin(q(4))*sin(q(5));
	dV[5] = 0.5003*cos(q(1))*cos(q(3))*cos(q(5)) - 0.06801*cos(q(1))*cos(q(3))*sin(q(5)) - 0.06801*cos(q(1))*cos(q(4))*cos(q(5))*sin(q(3)) + 0.5003*cos(q(2))*cos(q(5))*sin(q(1))*sin(q(3)) - 0.5003*cos(q(1))*cos(q(4))*sin(q(3))*sin(q(5)) - 0.06801*cos(q(2))*sin(q(1))*sin(q(3))*sin(q(5)) - 0.06801*cos(q(5))*sin(q(1))*sin(q(2))*sin(q(4)) - 0.5003*sin(q(1))*sin(q(2))*sin(q(4))*sin(q(5)) + 0.06801*cos(q(2))*cos(q(3))*cos(q(4))*cos(q(5))*sin(q(1)) + 0.5003*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(5));
	std::cout << "q(1) = " << q(1) << "\n";

	return dV;
}

Eigen::Vector3d PandaSim::getEEPose(Eigen::Matrix<double, 7, 1> q)
{
	// Fix matlab function that calculates z
	return Eigen::Vector3d::Zero();
}


// //Link count >= 1
// Eigen::Vector3d PandaSim::getCOG(int i, Eigen::Matrix<double, 7, 1> q){

// 	// Transform to the i-1th link 
// 	Eigen::Matrix<double,4, 4> T_im1 = getTi(i - 1, q);

// 	//Transform to the cog of the next link
// 	Eigen::Matrix<double, 3, 1> O_cog;
// 	O_cog << cos(q(i))*L_cogs[i](0) - sin(q(i))*L_cogs[i](1), sin(q(i))*L_cogs[i](0) + cos(q(i))*L_cogs[i](1), L_cogs[i](2);
// 	O_cog = O_cog + T_im1.block(0, 6, 3, 1);

// 	return O_cog;
// }

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