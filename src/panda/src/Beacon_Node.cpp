/* 
File: Beacon_Node.cpp

A beacon is a system that only contains a CMM such that it may influence convergence of other systems without having actual dynamics.
Thanks to the CMM the communication and convergence properties remain stable.

*/

#include "ros/ros.h"

#include "panda/Waves.h"
#include "std_msgs/Float64MultiArray.h"
#include <visualization_msgs/Marker.h>
#include "panda/getConnectionsOf.h"

#include "Goals.h"
#include "CMM.h"
#include "Helpers.h"

#include <memory>
#include <vector>
#include <string>
#include <sstream>

int agent_id, l, N;
Eigen::VectorXd ref;

void publishReference(ros::Publisher& pub, const Eigen::VectorXd ref);
void plotMarker(ros::Publisher& pub, Eigen::VectorXd ref);
void setGoalType(int goal_type);

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

	ros::NodeHandle n;// = ros::NodeHandle("~");

	// Get a nodehandle
	helpers::safelyRetrieve(n, "/beacon/ID", agent_id);
	helpers::safelyRetrieve(n, "/l", l);
	helpers::safelyRetrieve(n, "/N_agents", N);
	
	// Retrieve the goal
	int goal_type;
	helpers::safelyRetrieve(n, "/beacon/goal_type", goal_type, -1);

	if(goal_type == -1){
		helpers::safelyRetrieveEigen(n, "/beacon/goal", ref, 3);
	}else{
		setGoalType(goal_type);
	}
	
	/* Initialise publishers */
	ros::Publisher pub, marker_pub;
	pub = n.advertise<std_msgs::Float64MultiArray>("/reference", 20);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	CMM cmm(agent_id);

	ros::Rate loop_rate(1000);

	while(ros::ok()){

		// Sample the network
		Eigen::VectorXd tau_network = cmm.sample(ref);
		
		plotMarker(marker_pub, ref);

		publishReference(pub, ref);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;

}

void publishReference(ros::Publisher& pub, const Eigen::VectorXd ref){

	std_msgs::Float64MultiArray msg;

	// Put the wave in the messagge
	msg.data.resize(ref.size());
	for(int i = 0; i < l; i++){
		msg.data[i] = ref(i);
	}

	// Publish the message
	pub.publish(msg);

}

void plotMarker(ros::Publisher& pub, Eigen::VectorXd ref){

	//logTmp(ref);
	visualization_msgs::Marker points;
    points.header.frame_id = "/panda_link0";
    points.header.stamp = ros::Time::now();
    points.ns = "robot_1";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Create the vertices for the points and lines
	geometry_msgs::Point p;
	p.x = ref(0, 0);
	p.y = ref(1, 0);
	p.z = ref(2, 0);

    points.points.push_back(p);

    pub.publish(points);

}

void setGoalType(int goal_type){


	switch(goal_type){
		case 1:
			ref = Eigen::VectorXd::Zero(l);
			ref << 0.45, -0.3, 0.8;
			break;

		case 2:
			ref = Eigen::VectorXd::Zero(l);
			ref << -0.3, 0.3, 0.7;
			break;

		default:
			throw RetrievalException("Unknown goal type!");
			break;

	}
		
	std::string goal_string = "(";

	for (int i = 0; i < l; i++){
		goal_string += std::to_string(ref(i, 0));
		if(i < l - 1){
			goal_string += ", ";
		}else{
			goal_string += ")";
		}
	}

	logMsg("Beacon Node", "Goal set to " + goal_string + " (type=" + std::to_string(goal_type) + ")", 2);

}