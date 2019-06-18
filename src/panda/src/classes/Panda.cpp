
#include "Panda.h"

//See: https://github.com/frankaemika/franka_ros/blob/kinetic-devel/franka_example_controllers/include/franka_example_controllers/joint_impedance_example_controller.h

Panda::Panda(){


}


Eigen::VectorXd Panda::readSensors(){

}

bool Panda::sendInput(){

}

// Get the mass matrix at q (IMPLEMENT)
Eigen::MatrixXd Panda::getM(Eigen::VectorXd q){

	// Hardware wise or theoretic?
	return Eigen::MatrixXd::Zero(nq, nq);

}




Eigen::VectorXd Panda::getdHdq(Eigen::VectorXd q){
	return Eigen::VectorXd::Zero(nq);

}