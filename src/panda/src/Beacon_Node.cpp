/* 
File: Beacon_Node.cpp

A beacon is a system that only contains a CMM such that it may influence convergence of other systems without having actual dynamics.
Thanks to the CMM the communication and convergence properties remain stable.

*/

#include "ros/ros.h"

#include "panda/Waves.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16.h"
#include <visualization_msgs/Marker.h>
#include "panda/getConnectionsOf.h"
#include "panda/registerAgent.h"

#include "Goals.h"
#include "CMM.h"
#include "Helpers.h"
#include "System.h"

#include <memory>
#include <vector>
#include <string>
#include <sstream>

int l, N;
Eigen::MatrixXd S;
Status status = INIT_CMM;
Eigen::VectorXd ref, valued_goal, center_point;
std::unique_ptr<CMM> cmm;
bool is_dynamic = false;
double phase = 0;

//ros::NodeHandle nh;
void initSelectors();
void goalCallback(const std_msgs::Int16::ConstPtr & msg);
void publishReference(ros::Publisher& pub, const Eigen::VectorXd ref);
void plotMarker(ros::Publisher& pub, Eigen::VectorXd ref);
void setGoalType(int goal_type);

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

	// = ros::NodeHandle("~");
    ros::NodeHandle nh;

    helpers::safelyRetrieve(nh, "/l", l);
	helpers::safelyRetrieve(nh, "/N_agents", N);
    
	/* Initialise publishers */
	ros::Publisher pub, marker_pub;
	pub = nh.advertise<std_msgs::Float64MultiArray>("/reference", 20);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::Subscriber sub;
    sub = nh.subscribe<std_msgs::Int16>("/goal_setpoint", 1, &goalCallback);
    cmm = std::make_unique<CMM>("beacon");
    
    // Retrieve parameters
    initSelectors();
    
    // Dynamic parameters
	double radius, speed;
    cmm->agent->retrieveParameter("dynamic/radius", radius, 0.1);
    cmm->agent->retrieveParameter("dynamic/speed", speed, 0.1);
    
	// Retrieve the goal
	int goal_type;
    cmm->agent->retrieveParameter("goal_type", goal_type, -1);
    cmm->agent->retrieveEigen("default_goal", valued_goal, l);
    
    double lambda;
    helpers::safelyRetrieve(nh, "/lambda", lambda);

	// For centerpoint initially
    ref = valued_goal;
	setGoalType(goal_type);

    cmm->performHandshake();

	ros::Rate loop_rate(cmm->agent->getSamplingRate());

	while(ros::ok()){

	    if(is_dynamic){
	        ref(0) = center_point(0);
	        ref(1) = center_point(1) + cos(phase)*radius;
	        ref(2) = center_point(2) + sin(phase)*radius;

            phase += speed*(1.0/double(cmm->agent->getSamplingRate()));
	    }

		// Sample the network
        // LOOKS LIKE THE VARIANCE CONTROL PROBLEM IS RELATED TO RPBC, IDAPBC WORKS FINE NOW //
		Eigen::VectorXd tau_network = cmm->sample(ref);//*lambda
		
		plotMarker(marker_pub, ref);

		publishReference(pub, ref);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}

/// Initialise the selector matrix
/* Note that z = S*z_all, Psi = Psi_all * S' */
void initSelectors(){

    // This is not nice! maybe this entire thing in helpers?
    ros::NodeHandle nh;
    std::vector<int> selector;
    cmm->agent->retrieveParameter("z_select", selector);
    
    int lmax = selector.size();

    // Count the number of activated coordinates
    int cur_l = count(selector.begin(), selector.end(), 1);
    S = Eigen::MatrixXd::Zero(cur_l, lmax);

    // Create a matrix that selects the required entries
    int occurences = 0;
    for(int i = 0; i < selector.size(); i++) {

        if(selector[i] == 1){
            S(occurences, i) = 1;
            occurences++;
        }
    }
}

//380mm
void goalCallback(const std_msgs::Int16::ConstPtr & msg){
    setGoalType(msg->data);
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

	geometry_msgs::Point TEST_OBJECT;
	TEST_OBJECT.x = 0;
	TEST_OBJECT.y = 0;
	TEST_OBJECT.z = 1.0;
    points.points.push_back(p);
    points.points.push_back(TEST_OBJECT);

    pub.publish(points);

}

void setGoalType(int goal_type){

    phase = 0;
    // Check for dynamic setpoints
    if(goal_type == -2){
        is_dynamic = true;
        center_point = ref;
        return;
    }else{
        is_dynamic = false;
    }


    if(goal_type == -1){
        ref = S*valued_goal;
        return;
    }

    ros::NodeHandle nh;

    // Get a nodehandle
    Eigen::VectorXd temp_ref;
    helpers::safelyRetrieveEigen(nh, "goals/" + std::to_string(l) +
    "/goal" + std::to_string(goal_type), temp_ref);

    ref = S*temp_ref;
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