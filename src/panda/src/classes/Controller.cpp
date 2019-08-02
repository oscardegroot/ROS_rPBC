/*

File: Controller.cpp

An interface for controllers using IDAPBC or rPBC control.

*/

#include "Controller.h"

Controller::Controller(){

	tau_pub.init(nh, "agent_tau", 1);
	z_pub.init(nh, "agent_z", 1);

	double publish_rate;
	helpers::safelyRetrieve(nh, "/controller/publish_rate", publish_rate, 10.0);
	tau_rate = franka_hw::TriggerRate(publish_rate);
	z_rate = franka_hw::TriggerRate(publish_rate);

}

Controller::~Controller(){};


// void Controller::publishZ(Eigen::VectorXd z){

// 	std_msgs::Float64MultiArray msg;

// 	// Put the wave in the messagge
// 	msg.data.resize(l);
// 	for(int i = 0; i < l; i++){
// 		msg.data[i] = z(i);
// 	}

// 	// Publish the message
// 	z_pub.publish(msg);
// }

void Controller::publishValue(realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>& pub,
	franka_hw::TriggerRate trigger_rate, const Eigen::VectorXd values){

	if (trigger_rate() && pub.trylock()) {

		pub.msg_.data.resize(values.size());

		for(int i = 0; i < values.size(); i++){
			pub.msg_.data[i] = values(i);
		}

		pub.unlockAndPublish();
	}
}

// void Panda::publishTau(const Eigen::VectorXd z){

// 	if (rate_trigger() && tau_pub.trylock()) {

// 		tau_pub.msg_.data.resize(7);

// 		for(int i = 0; i < 7; i++){
// 			tau_pub.msg_.data[i] = torques(i);
// 		}

// 		tau_pub.unlockAndPublish();
// 	}

// }

// void Panda::publishTau(const Eigen::VectorXd tau){

// 	if (rate_trigger() && tau_pub.trylock()) {

// 		tau_pub.msg_.data.resize(7);

// 		for(int i = 0; i < 7; i++){
// 			tau_pub.msg_.data[i] = torques(i);
// 		}

// 		tau_pub.unlockAndPublish();
// 	}

// }