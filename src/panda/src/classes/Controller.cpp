/*

File: Controller.cpp

An interface for controllers using IDAPBC or rPBC control.

*/

#include "Controller.h"

Controller::Controller(){
	z_pub = nh.advertise<std_msgs::Float64MultiArray>("agent_z", 20);
	tau_pub = nh.advertise<std_msgs::Float64MultiArray>("agent_tau", 20);

}

Controller::~Controller(){};


void Controller::publishZ(Eigen::VectorXd z){

	std_msgs::Float64MultiArray msg;

	// Put the wave in the messagge
	msg.data.resize(l);
	for(int i = 0; i < l; i++){
		msg.data[i] = z(i);
	}

	// Publish the message
	z_pub.publish(msg);
}

void Controller::publishTau(Eigen::VectorXd tau){

	std_msgs::Float64MultiArray msg;

	// Put the wave in the messagge
	msg.data.resize(7);
	for(int i = 0; i < 7; i++){
		msg.data[i] = tau(i);
	}

	// Publish the message
	tau_pub.publish(msg);
}