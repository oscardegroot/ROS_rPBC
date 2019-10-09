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
    publisher = nh.advertise<std_msgs::Int16>("/goal_setpoint", 1);

    while(ros::ok()){
        ros::Duration(0.2).sleep();
        
        std::cout << "Please enter a new goal (1-5): ";
        int goal_select = helpers::getch(); // 48 = 0 on keyboard
        
        // Space bar
        if(goal_select == 32){
            goal_select = -2;
        // number
        }else{
            goal_select -= 48;
        }
        
        if(goal_select > 9 || goal_select < -2){
            logMsg("Interface", "Incorrect goal given! Input is ignored. (goal = " + std::to_string(goal_select) + ")", 1);
            continue;
        }
        //std::cin >> goal_select;
        publisher.publish(goal_select);

        std::cout << "\n";
        logMsg("Interface", "New goal type set to " + std::to_string(goal_select) + ".", 2);

        ros::spinOnce();
    }

    return 0;

}