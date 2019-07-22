/* 
File: Beacon_Node.cpp

A beacon is a system that only contains a CMM such that it may influence convergence of other systems without having actual dynamics.
Thanks to the CMM the communication and convergence properties remain stable.

*/



#include "ros/ros.h"

#include "panda/Waves.h"
#include "std_msgs/Float64MultiArray.h"
#include "panda/getConnectionsOf.h"

#include "Goals.h"
#include "CMM.h"
#include "Helpers.h"

#include <memory>
#include <vector>
#include <string>
#include <sstream>

int agent_id, l, N;

void publishReference(ros::Publisher& pub, const Eigen::VectorXd ref);

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

	ros::NodeHandle n;// = ros::NodeHandle("~");

	// Get a nodehandle
	helpers::safelyRetrieve(n, "/beacon/ID", agent_id);
	helpers::safelyRetrieve(n, "/l", l);
	helpers::safelyRetrieve(n, "/N_agents", N);
	
	// Retrieve the goal
	double lambda;
	std::vector<double> ref_v;
	helpers::safelyRetrieveArray(n, "/beacon/goal", ref_v, 3);
	helpers::safelyRetrieve(n, "/lambda", lambda, 1.0);


	Eigen::VectorXd ref = helpers::vectorToEigen(ref_v);
	//ref << 0.0, 0.3, 0.8;
	
	ros::Publisher pub;
	pub = n.advertise<std_msgs::Float64MultiArray>("/reference", 20);

	CMM cmm(agent_id);

	ros::Rate loop_rate(1000);

	while(ros::ok()){

		// Sample the network
		Eigen::VectorXd tau_network = cmm.sample(lambda*ref);
		
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