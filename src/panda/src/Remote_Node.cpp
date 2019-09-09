
#include "Remote_Node.h"

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Remote");

	// Get a nodehandle
	ros::NodeHandle n;

	Remote remote;

//    ros::AsyncSpinner spinner(1); // Use 4 threads
//    spinner.start();
    
    //remote.initiateNetwork();
    while(ros::ok() && remote.status != SPINNING){
                
        switch(remote.status){
            case REGISTERING_AGENTS:
                if(remote.isReady()){
                    remote.status = STARTING_AGENTS;
                }
                break;
                
            case STARTING_AGENTS:
                remote.initiateNetwork();
                break;
                
        }
        
        ros::spinOnce();
    }
    

	ros::spin();

	return 0;

}