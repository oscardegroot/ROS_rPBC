
#include "Remote_Node.h"

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Remote");

	// Get a nodehandle
	ros::NodeHandle n;
    
    bool is_simulation;
    helpers::safelyRetrieve(n, "/is_simulation", is_simulation);

	Remote remote;

//    ros::AsyncSpinner spinner(1); // Use 4 threads
//    spinner.start();
    std::thread t1;
    bool thread_started = false;
    //remote.initiateNetwork();
    while(ros::ok() && remote.status != SPINNING){

        switch(remote.status){
            
            case REGISTERING_AGENTS:
                if(is_simulation){
                    remote.pauseGazebo();
                    if(!thread_started){
                        t1 = std::thread(&Remote::pauseGazebo, &remote);
                        thread_started = true;
                    }
                }

                if(remote.finishedRegistering()){
                    if(is_simulation){
                        remote.status = WAITING_FOR_GAZEBO;
                        logMsg("Remote", "Status Changed To: WAITING FOR GAZEBO", 2);
                    }else{
                        remote.status = STARTING_AGENTS;
                        logMsg("Remote", "Status Changed To: STARTING AGENTS", 2);
                    }
                }
                
                break; 
 
            case WAITING_FOR_GAZEBO:
                    

                remote.status = STARTING_AGENTS;
                logMsg("Remote", "Status Changed To: STARTING AGENTS", 2);
        
                break;
                
            case STARTING_AGENTS:

                //remote.pauseGazebo();
                remote.initiateNetwork();
                remote.status = WAITING_FOR_CMM;
                logMsg("Remote", "Status Changed To: WAITING FOR CMM", 2);
                
                break;
                
            case WAITING_FOR_CMM:

                if(remote.cmm_ready()){

                    if(is_simulation){
                        remote.unpauseGazebo();
                    }
                    remote.status = SPINNING;
                    logMsg("Remote", "Status Changed To: SPINNING", 2);

                }   
                break;             
        }
        ros::spinOnce();
    }
    /** @todo Add case spinning and Elisa_Station */
                    
//    if(is_simulation){
//        remote.unpauseGazebo();
//    }

	ros::spin();

	return 0;

}