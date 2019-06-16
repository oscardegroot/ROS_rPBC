#include "ros/ros.h"
#include "panda/Waves.h"
#include "Edge.h"
#include <vector>
#include <string>
#include <sstream>

int agent_id;
int j_id;
int l;

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
	ros::NodeHandle n("~");
	n.getParam("id", agent_id);
	n.getParam("j_id", j_id);
	n.getParam("/l_dim", l);

	Edge edge(agent_id, j_id, 1.0, l);
	Eigen::VectorXd r_i(l);

	ros::Rate loop_rate(0.5);

	while(ros::ok()){
		
		r_i = Eigen::VectorXd::Random(l);
		//Eigen::Matrix<double, 2, 1> r_j = Eigen::Matrix<double, 2, 1>::Random();

		edge.sampleEdge(r_i);
		//edge_j.sampleEdge(r_j);

		ros::spinOnce();
		loop_rate.sleep();
	}

	// Enter the callback loop
	ros::spin();

	return 0;

}