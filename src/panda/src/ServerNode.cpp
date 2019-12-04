#include "ros/ros.h"
#include "CustomLog.h"
#include <thread>
#include "Station.h"
#include "ElisaStation.h"
#include "AgentStation.h"

enum ServerState{
    REGISTERING = 0,
    STARTING,
    WAIT_FOR_RESPONSE,
    ENABLE_STATION,
    RUNNING_STATION
};

// The server state
ServerState overall_state = REGISTERING;

// The connected stations
std::vector<std::unique_ptr<Station>> stations;

bool finishedRegistering(const std::vector<std::unique_ptr<Station>>& stations);
bool responsesReceived(const std::vector<std::unique_ptr<Station>>& stations);

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Server");
    ros::NodeHandle nh;
    
    Instrumentor::Get().BeginSession("Server", "server");
    
//    /* The elisa station */
    if(helpers::ifParameter(nh, "/elisa_station")){
        
        stations.push_back(std::make_unique<ElisaStation>());
    }
        
    /* The agent station */
    stations.push_back(std::make_unique<AgentStation>());
    
    // Retrieve and apply the sampling rate 
    double sampling_rate;
    helpers::safelyRetrieve(nh, "/sampling_rate", sampling_rate);
    ros::Rate loop_rate(sampling_rate);

    while(ros::ok()){


        // Depending on the status of the station
        switch(overall_state){
            
            // If we are registering
            case REGISTERING:  
            {              
                PROFILE_SCOPE("Registering");
                // Only continue if all stations finished registering
                if(finishedRegistering(stations)){
                    
                    overall_state = STARTING;
                    logMsg("Server", "Overall state advanced to: STARTING", 2);
                }     
            }
                break; 
                
            // If we are starting
            case STARTING:
                {
                    PROFILE_SCOPE("Starting");
                    // Start all stations
                    for(auto& station : stations){
                        station->starting();
                    }
                    
                    // Proceed with the state
                    overall_state = WAIT_FOR_RESPONSE;
                    logMsg("Server", "Overall state advanced to: WAIT_FOR_RESPONSE", 2);
                }  
                    break;
   
            // If we are waiting for responses
            case WAIT_FOR_RESPONSE:
            {
                PROFILE_SCOPE("Waiting for responses");

                // If all stations received responses
                if(responsesReceived(stations)){
                    
                    // Proceed
                    overall_state = ENABLE_STATION;
                    logMsg("Server", "Overall state advanced to: ENABLE_STATION", 2);
                }
            }   
                break;  
 
            // If we are ready to enable the station
            case ENABLE_STATION:
            {
                PROFILE_SCOPE("Enabling Stations");

                // Enable all stations
                for(auto& station : stations){
                    station->enableStation();
                }

                // Proceed to running
                overall_state = RUNNING_STATION;
                logMsg("Server", "Overall state advanced to: RUNNING_STATION", 2);
            }
                break;
                
            // While we are running the stations
            case RUNNING_STATION:
            {
                PROFILE_SCOPE("Running Stations");

                // Update all stations
                for(auto& station : stations){
                    station->update();
                }
                
                // Run on the given sampling frequency
                loop_rate.sleep();
            }
                break;
                
        }
        
        // Spin every loop
        ros::spinOnce();
    }
    
    // If the program is stopped, stop all stations
    for(auto& station : stations){
        PROFILE_SCOPE("Stopping Stations");

        station->stopping();
    }

    Instrumentor::Get().EndSession();


	return 0;
}

bool finishedRegistering(const std::vector<std::unique_ptr<Station>>& stations){
    for(auto& station : stations){
        if(!station->finishedRegistering()){
            return false;
        }
    }
    
    return true;
}

bool responsesReceived(const std::vector<std::unique_ptr<Station>>& stations){
    for(auto& station : stations){
        if(!station->responsesReceived()){
            return false;
        }
    }
    
    return true;
}