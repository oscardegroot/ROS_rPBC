#include "Controller.h"

Controller::Controller(){

}

Controller::~Controller(){};

// Get the agent output, possibly modified
virtual Eigen::VectorXd getOutput(VectorXd z_i, VectorXd dz_i){
	
	// In the simple case return the EE velocity
	return dz_i;
}