/*
File: System.cpp

Defines an interface for systems controlled by IDAPBC or rPBC
*/

#include "System.h"


System::System(int n_set, int m_set, int lmax_set)
	: n(n_set), m(m_set), lmax(lmax_set)
{
    initSelectors();
}
		

System::~System(){
}


// bool System::dataReady(){
// 	return data_rdy;
// }


void System::setState(Eigen::VectorXd new_q, Eigen::VectorXd new_dq, Eigen::VectorXd new_z){
	state.q = new_q;
	state.dq = new_dq;
	state.z = selectZ(new_z);
}

void System::checkSafety(){
	return;
}

void System::readSensors(){
    return;
}

bool System::dataReady(){
    return false;
}


/// Initialise the selector matrix
/* Note that z = S*z_all, Psi = Psi_all * S' */
void System::initSelectors(){

    // This is not nice! maybe this entire thing in helpers?
    ros::NodeHandle nh;
    std::vector<int> selector;
    helpers::safelyRetrieveArray(nh, "/panda/z_select", selector, lmax); //todo: Fix this!

    // Count the number of activated coordinates
    int cur_l = count(selector.begin(), selector.end(), 1);
    S = Eigen::MatrixXd::Zero(cur_l, lmax);

    // Create a matrix that selects the required entries
    int occurences = 0;
    for(int i = 0; i < selector.size(); i++) {

        if(selector[i] == 1){
            S(occurences, i) = 1;
            occurences++;
        }
    }
}

/// Keep only currently cooperative controlled parts
Eigen::MatrixXd System::selectPsi(Eigen::MatrixXd psi){
    return psi * S.transpose();
}

Eigen::VectorXd System::selectZ(Eigen::VectorXd z){
    return S*z;
}

// void System::setDataReady(bool is_ready){
// 	data_rdy = is_ready;
// }
