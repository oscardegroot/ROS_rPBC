
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
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

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
ros::Publisher p_wave_ij_in, p_wave_ij_out, p_wave_ji_in, p_wave_ji_out;

ros::Publisher debugPub;

std::vector<DelayedMsg> msg_vec;
double delay_ij, delay_ji, max_delay, min_delay, max_step_delay;
bool delay_is_constant;
double sampling_rate;
double drop_chance = 0.0;

int i_ID, j_ID;

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
    ros::NodeHandle nh_private("~");
    helpers::safelyRetrieve(nh_private, "i_ID", i_ID);
    helpers::safelyRetrieve(nh_private, "j_ID", j_ID);

    if(!helpers::ifParameter(nh, "/network/enabled")){
        logMsg("Network Node", "Artificial Network disabled, shutting down..", 2);
        return 0;
    }
	
    wave_ij_in = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p_in", 10, &waveCallback_i);
    wave_ij_out = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p_out", 10);
    
    wave_ji_in = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m_in", 10, &waveCallback_j);
    wave_ji_out = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m_out", 10);

    /* Publishers for the messages that Matlab can understand */
    p_wave_ij_in = nh.advertise<std_msgs::Float64MultiArray>("p_s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p_in", 10);
    p_wave_ij_out = nh.advertise<std_msgs::Float64MultiArray>("p_s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p_out", 10);
    
    p_wave_ji_in = nh.advertise<std_msgs::Float64MultiArray>("p_s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m_in", 10);
    p_wave_ji_out = nh.advertise<std_msgs::Float64MultiArray>("p_s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m_out", 10);

    debugPub = nh.advertise<std_msgs::Float64>("delays_" + std::to_string(i_ID) + std::to_string(j_ID), 10);

    helpers::safelyRetrieve(nh, "/network/delay/constant", delay_is_constant);
    
    if(!delay_is_constant){
        srand(std::time(NULL));
        
        // Initialise randoml
        delay_ij = min_delay;
        delay_ji = min_delay;
        //delay_ij = min_delay + (max_delay - min_delay)*((double)rand() / RAND_MAX);
        //delay_ji = min_delay + (max_delay - min_delay)*((double)rand() / RAND_MAX);

        helpers::safelyRetrieve(nh, "/network/delay/min_delay", min_delay);
        helpers::safelyRetrieve(nh, "/network/delay/max_delay", max_delay);
        helpers::safelyRetrieve(nh, "/network/delay/change", max_step_delay);
    }else{
        helpers::safelyRetrieve(nh, "/network/delay/value_plus", delay_ij); // Initial when variable
        helpers::safelyRetrieve(nh, "/network/delay/value_minus", delay_ji); // Initial when variable
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
        msg_vec.push_back({msg, helpers::SimpleTimer(delay_ij), j_ID});
        
        std_msgs::Float64MultiArray plot_msg;
        plot_msg = msg->s;
        p_wave_ij_in.publish(plot_msg);
    }
}

void waveCallback_j(const panda::Waves::ConstPtr& msg){
    
    if(!messageDropped()){
        msg_vec.push_back({msg, helpers::SimpleTimer(delay_ji), i_ID});
        
        std_msgs::Float64MultiArray plot_msg;
        plot_msg = msg->s;
        p_wave_ji_in.publish(plot_msg);
    }
}

/**
 * @brief Publish a delayed message
 */
void publish(const DelayedMsg& d_msg){
    
    std_msgs::Float64MultiArray plot_msg;
    
    plot_msg = d_msg.msg->s;
        
    
    if(d_msg.receiver == j_ID){
        wave_ij_out.publish(d_msg.msg);
        p_wave_ij_out.publish(plot_msg);
    }else{
        wave_ji_out.publish(d_msg.msg);
        p_wave_ji_out.publish(plot_msg);
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
    delay_ij += (-1.0 + (double)rand() / RAND_MAX * 2.0) * max_step_delay * (1.0 / (double)(sampling_rate));
    
    // Check limits of the delay
    delay_ij = std::max(min_delay, std::min(delay_ij, max_delay));
    
    
    delay_ji += (-1.0 + (double)rand() / RAND_MAX * 2.0) * max_step_delay * (1.0 / (double)(sampling_rate));
    
    // Check limits of the delay
    delay_ji = std::max(min_delay, std::min(delay_ji, max_delay));
    
    std_msgs::Float64 msg;
    msg.data = delay_ij;
    debugPub.publish(msg);
}
