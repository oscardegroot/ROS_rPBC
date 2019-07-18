
#include "Remote_Node.h"

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Remote");

	// Get a nodehandle
	ros::NodeHandle n;
	// helpers::safelyRetrieve(n, "/l", l);
	// helpers::safelyRetrieve(n, "/N_agents", N);

	Remote remote;
	
	ros::spin();

	return 0;

}