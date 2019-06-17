
#include "Remote_Node.h"

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Remote");

	// Get a nodehandle
	ros::NodeHandle n("~");
	n.getParam("/l_dim", l);
	n.getParam("/N_dim", N);

	Remote remote(l, N);
	
	ros::spin();


	return 0;

}