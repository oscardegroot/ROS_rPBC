
#include "Remote_Node.h"

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Remote");

	// Get a nodehandle
	ros::NodeHandle n("~");
	n.getParam("/l", l);
	n.getParam("/N_agents", N);

	Remote remote(l, N);
	
	ros::spin();


	return 0;

}