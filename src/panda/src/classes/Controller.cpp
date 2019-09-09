/*

File: Controller.cpp

An interface for controllers using IDAPBC or rPBC control.

*/

#include "Controller.h"

Controller::Controller(System& system_set){

	tau_pub.init(nh, "agent_tau", 1);
	z_pub.init(nh, "agent_z", 1);

    helpers::safelyRetrieve(nh, "/l", l);

	double publish_rate;
	helpers::safelyRetrieve(nh, "/controller/publish_rate", publish_rate, 10.0);
	tau_rate = franka_hw::TriggerRate(publish_rate);
	z_rate = franka_hw::TriggerRate(publish_rate);
    
    psi = Eigen::MatrixXd::Zero(system_set.n, l);
    m = Eigen::MatrixXd::Zero(system_set.n, system_set.n);
    dvdq = Eigen::VectorXd::Zero(system_set.n);

}

Controller::~Controller(){};

void Controller::publishValue(realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>& pub,
	franka_hw::TriggerRate trigger_rate, const Eigen::VectorXd values) {

    if (trigger_rate() && pub.trylock()) {

        pub.msg_.data.resize(values.size());

        for (int i = 0; i < values.size(); i++) {
            pub.msg_.data[i] = values(i);
        }

        pub.unlockAndPublish();
    }
}
