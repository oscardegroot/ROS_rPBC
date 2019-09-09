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


void System::setAgent(ros::NodeHandle& nh_global, const std::string type_name){
    logTmp(type_name);
    ros::NodeHandle nh_private(type_name); // Declare a private nodehandle
    int id;
    int sampling_rate;
    //Status status = STARTED;
    helpers::safelyRetrieve(nh_private, "ID", id);

    //Does not influence the actual sampling rate, just for
    helpers::safelyRetrieve(nh_private, "sampling_rate", sampling_rate);

    agent = Agent({type_name,
                   ros::this_node::getName(),
                   id,
                   sampling_rate});

    ros::ServiceClient register_client = nh_global.serviceClient<panda::registerAgent>("/registerAgent");
    panda::registerAgent srv = agent.toSrv();
    //agent.print();
    if(register_client.call(srv)){
        logMsg("System", "Registered a " + type_name + " succesfully", 2);
    }else{
        throw RegisteringException("Agent could not register!");
    }
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
Eigen::MatrixXd System::selectPsi(const Eigen::MatrixXd& psi){
    return psi * S.transpose();
}

Eigen::VectorXd System::selectZ(const Eigen::VectorXd& z){
    return S*z;
}

// void System::setDataReady(bool is_ready){
// 	data_rdy = is_ready;
// }
