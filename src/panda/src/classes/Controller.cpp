/*

File: Controller.cpp

An interface for controllers using IDAPBC or rPBC control.

*/

#include "Controller.h"

Controller::Controller(){
	z_pub = nh.advertise<std_msgs::Float64MultiArray>("agent_z", 20);
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