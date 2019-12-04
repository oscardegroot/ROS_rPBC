/*

File: Controller.cpp

An interface for controllers using IDAPBC or rPBC control.

*/

#include "Controller.h"

Controller::Controller(Agent& agent){

    helpers::safelyRetrieve(nh, "/l", l);

	double publish_rate;
	agent.retrieveParameter("controller/publish_rate", publish_rate, 10.0);
    
    publishers.emplace_back(agent, nh, publish_rate, "z");
    publishers.emplace_back(agent, nh, publish_rate, "z_dot");
    publishers.emplace_back(agent, nh, publish_rate, "theta_dot");
    publishers.emplace_back(agent, nh, publish_rate, "tau");

}

Controller::~Controller(){};

Eigen::MatrixXd Controller::Kv(System& system){
	return Eigen::MatrixXd::Identity(system.n, system.n);
}

void Controller::publish(const std::string& name, const Eigen::VectorXd& values) {
    
    for(size_t i = 0; i < publishers.size(); i++){
        
        if(publishers[i].publishes(name)){
            publishers[i].publish(values);
            return;
        }
    }
}
