
#ifndef Remote_H
#define Remote_H

#include "ros/ros.h"
#include "Goals.h"
#include <vector>
#include "std_msgs/Float64MultiArray.h"
#include "panda/getConnectionsOf.h"
#include <string>
#include <sstream>


class Remote{
public:
	Remote(int l_dim, int N_dim);
	~Remote();

	bool retrieveConnectionsOf(panda::getConnectionsOf::Request &req, panda::getConnectionsOf::Response &res);


private:

	int l, N;
	Goals * goals;

	// A nodehandle
	ros::NodeHandle n;

	// The server
	ros::ServiceServer connect_server;


};



#endif