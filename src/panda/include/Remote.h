/*
File: Remote.h

A Remote node class that hosts services and implements an interface
*/


#ifndef Remote_H
#define Remote_H

#include "ros/ros.h"
#include "Goals.h"
#include <vector>
#include "std_msgs/Float64MultiArray.h"
#include "panda/getConnectionsOf.h"
#include "panda/isAgentLeader.h"
#include "panda/registerAgent.h"
#include "std_srvs/Empty.h"
#include <string>
#include "CustomLog.h"
#include "Exceptions.h"
#include "Helpers.h"
#include "System.h"
#include <memory>


enum RemoteStatus{
    REGISTERING_AGENTS,
    WAITING_FOR_GAZEBO,
    STARTING_AGENTS,
    WAITING_FOR_CMM,
    SPINNING
};

class Remote{
public:
	Remote();
	~Remote();
    
    RemoteStatus status = REGISTERING_AGENTS;

	bool getConnectionsOf(panda::getConnectionsOf::Request &req,
		 panda::getConnectionsOf::Response &res);

    bool isAgentLeader(panda::isAgentLeader::Request &req, panda::isAgentLeader::Response &res);

    bool registerAgent(panda::registerAgent::Request &req, panda::registerAgent::Response &res);
    bool acknowledgeCMMReady(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    void initiateNetwork();
    bool finishedRegistering();
    
    bool cmm_ready();

    void pauseGazebo();
    void unpauseGazebo();
    
    void fakeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);


private:

	int l, N;

	std::unique_ptr<Goals> goals;
    std::unique_ptr<helpers::SimpleTimer> timer;
    
	// Stores all agent information (namespace / id / sampling_rate)
    std::vector<std::unique_ptr<Agent>> agents;

	// A nodehandle
	ros::NodeHandle n;
    bool ready = false;
    
    unsigned int cmm_count;

	// The server
	ros::ServiceServer connect_server, leader_server, register_server, cmm_server;


};



#endif