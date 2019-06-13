#include "ros/ros.h"
#include "panda/Waves.h"

#include <vector>
#include <sstream>


std::vector<float> s;

void readWave(const panda::Waves::ConstPtr& msg)
{
	//ROS_INFO("Wave Received [k=%d]\n", msg->timestamp);
	/*ROS_INFO("	Wave Value [s1]: %f \n", msg->s_1);
	ROS_INFO("	Wave Value [s2]: %f \n", msg->s_2);*/


}

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "CMM_0");

	// Get a nodehandle
	ros::NodeHandle n;

	// Subscribe to the positive wave channel
	ros::Subscriber sub = n.subscribe("waves_ij_p", 1000, readWave);

	// Enter the callback loop
	ros::spin();

	return 0;

}