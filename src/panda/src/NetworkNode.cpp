
/**
*
* @brief Node to introduce artificial delays in the network
* 
* Code is of worse quality since it is only research related
*/

#include "ros/ros.h"

#include "Goals.h"
#include "CMM.h"
#include "System.h"

#include "panda/Waves.h"

#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include "Helpers.h"

#include <stdio.h> 
#include <stdlib.h>  
#include <time.h> 

struct DelayedMsg{
    
    panda::Waves::ConstPtr msg;
    helpers::SimpleTimer timer;
    int receiver;
  
};

ros::Subscriber wave_ij_in, wave_ji_in;
ros::Publisher wave_ij_out, wave_ji_out;
std::vector<DelayedMsg> msg_vec;
double delay, max_delay, min_delay, max_step_delay;
bool delay_is_constant;
double sampling_rate;
double drop_chance = 0.0;

// Hardcoded :(
int i_ID = 0;
int j_ID = 1;

// Callback
void waveCallback_i(const panda::Waves::ConstPtr& msg);
void waveCallback_j(const panda::Waves::ConstPtr& msg);
void publish(const DelayedMsg& d_msg);
void updateDelay();
bool messageDropped();

int main(int argc, char **argv){
	
	// Initialise ROS
	ros::init(argc, argv, "Agent");

    ros::NodeHandle nh;

    if(!helpers::ifParameter(nh, "/network/enabled")){
        logMsg("Network Node", "Artificial Network disabled, shutting down..", 2);
        return 0;
    }
	
    wave_ij_in = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p_in", 10, &waveCallback_i);
    wave_ij_out = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p_out", 10);
    
    wave_ji_in = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m_in", 10, &waveCallback_j);
    wave_ji_out = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m_out", 10);

    helpers::safelyRetrieve(nh, "/network/delay/value", delay); // Initial when variable

    helpers::safelyRetrieve(nh, "/network/delay/constant", delay_is_constant);
    
    if(!delay_is_constant){
        srand(std::time(NULL));
        
        helpers::safelyRetrieve(nh, "/network/delay/min_delay", min_delay);
        helpers::safelyRetrieve(nh, "/network/delay/max_delay", max_delay);
        helpers::safelyRetrieve(nh, "/network/delay/change", max_step_delay);
    }
    
    if(helpers::ifParameter(nh, "/network/packet_loss/enabled")){
        
        helpers::safelyRetrieve(nh, "/network/packet_loss/p", drop_chance);
    }

    helpers::safelyRetrieve(nh, "sampling_rate", sampling_rate);
    
	ros::Rate loop_rate(sampling_rate);

	while(ros::ok()){
        
        updateDelay();
        
        for(int i = msg_vec.size() - 1; i >= 0; i--){

            // If the delay is over
            if(msg_vec[i].timer.finished()){
                
                // Relay the message
                publish(msg_vec[i]);
                
                // Erase the entry from the queue
                msg_vec.erase(msg_vec.begin() + i);
            }
        }

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}

/**
 * @brief Callbacks for incoming waves, save the message in a queue like vector
 */
void waveCallback_i(const panda::Waves::ConstPtr& msg){
    
    if(!messageDropped()){
        msg_vec.push_back({msg, helpers::SimpleTimer(delay), j_ID});
    }
}

void waveCallback_j(const panda::Waves::ConstPtr& msg){
    
    if(!messageDropped()){
        msg_vec.push_back({msg, helpers::SimpleTimer(delay), i_ID});
    }
}

/**
 * @brief Publish a delayed message
 */
void publish(const DelayedMsg& d_msg){
    
    if(d_msg.receiver == j_ID){
        wave_ij_out.publish(d_msg.msg);
    }else{
        wave_ji_out.publish(d_msg.msg);
    }

}

/**
 * @brief Return true if a message should be dropped based on a random observation
 * @return if a message should be dropped
 */
bool messageDropped(){

    return (double)rand() / RAND_MAX < drop_chance;
}

/**
 * @brief Update the delay depending on the given parameters
 */
void updateDelay(){
    
    if(delay_is_constant){
        return;
    }
    
    // Random walk
    // (-1.0, 1.0) * % * sampling period
    delay += (-1.0 + (double)rand() / RAND_MAX * 2.0) * max_step_delay * (1.0 / (double)(sampling_rate));
    
    // Check limits of the delay
    delay = std::max(min_delay, std::min(delay, max_delay));
}
