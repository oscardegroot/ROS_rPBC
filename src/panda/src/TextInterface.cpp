/*
File: Beacon_Node.cpp

A beacon is a system that only contains a CMM such that it may influence convergence of other systems without having actual dynamics.
Thanks to the CMM the communication and convergence properties remain stable.

*/

#include "ros/ros.h"

#include "std_msgs/Int16.h"
#include "Helpers.h"

int l;

int main(int argc, char **argv){

    // Initialise ROS
    ros::init(argc, argv, "TextInterface");
    ros::NodeHandle nh;

    helpers::safelyRetrieve(nh, "/l", l);

    ros::Publisher publisher;
    publisher = nh.advertise<std_msgs::Int16>("/goal", 1);

    while(ros::ok()){
        ros::Duration(5.0).sleep();
        int goal_select;
        std::cout << "New goal: ";
        std::cin >> goal_select;
        publisher.publish(goal_select);

        logMsg("Interface", "New goal type set to " + std::to_string(goal_select) + ".", 2);

        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    return 0;

}