/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

#include "Edge.h"

Edge::Edge(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set){

	logMsg("Edge", "Initiating edge between " + std::to_string(agent.getID()) + " and " + std::to_string(j) + "..", 2);
    
	//Save the connection between agents
	i_ID = agent.getID();
	j_ID = j;
	l = l_set;
	gain = gain_set;
	r_star = r_star_set;
    
        // Multiply the formation distance by lambda if rPBC is used
    std::string output;
    helpers::safelyRetrieve(nh, "/output", output);
    
    if(output == "r"){
        double lambda;
        helpers::safelyRetrieve(nh, "/lambda", lambda);
        
        r_star *= lambda;
    }
    
    
    rate_mp = rate_mp_set;    
	initChannels();

    publish_counter = std::make_unique<helpers::Counter>(rate_mp);

	// Initialise buffers
	s_wvm_buffer = Eigen::VectorXd::Zero(l);
	s_received = Eigen::VectorXd::Zero(l);
    s_sample = Eigen::VectorXd::Zero(l);
    
    s_out_compressed = Eigen::VectorXd::Zero(l);
    s_out_squared = Eigen::VectorXd::Zero(l);

	logMsg("Edge", "Done!", 2);

}

/// Initialise channel
void Edge::initChannels(){

    // In case of leaders
    if(i_ID == -1 || j_ID == -1){
        return;
    }
    
    std::string to_network = "";
    std::string from_network = "";
    
    if(helpers::ifParameter(nh, "/network/enabled")){
        to_network = "_in";
        from_network = "_out";
    }

    // If this is agent i, send on the positivily defined channel, listen to the negative channel
    if(i_ID < j_ID){
        wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p" + to_network, 100);
        wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m" + from_network, 100, &Edge::waveCallback, this);
    }else /* Otherwise invert these */{
        wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "m" + to_network, 100);
        wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "p" + from_network, 100, &Edge::waveCallback, this);
    }
}

// Destructor
Edge::~Edge(){}

// Sample the edge, retrieving a data point for the cooperative control and returning a wave in the process
Eigen::VectorXd Edge::sample(const Eigen::VectorXd& r_i){
    
	// Reconstruct the wave if nothing was received
	applyReconstruction(s_received, r_i);

	// Calculate the input (I forgot to disable this!)
	Eigen::VectorXd s_out = calculateWaves(s_received, r_i);

	// Publish the returning wave
	publishWave(s_out);

	// Return the input for this edge
	return calculateControls(s_received, r_i);
}

/// Handle a received packet
void Edge::waveCallback(const panda::Waves::ConstPtr& msg){
	
	logMsg("Edge", "Wave received. Timestamp = " + std::to_string(msg->timestamp) + ".", 4);

    /** @todo timestamp check, rejection for t > t_last */

	std::vector<double> s = msg->s.data;

	s_received = Eigen::VectorXd::Zero(l);
	for(int i = 0; i < l; i++){
		s_received(i) = s[i];
        // EXPANDER! -> now seperate
        //helpers::sgn(s[i])*std::sqrt(std::pow(s[i], 2)/double(rate_mp));// / double(rate_mp); // divide by sampling discrepancy! (this is the passive interpolator)
        // NOT WORKING YET!! RECONSTRUCTION IS APPLIED ON THE WRONG SIDE...
	}

	// Data was received
	data_received = true;
}

/// Reconstruct the signal if necessary
void Edge::applyReconstruction(Eigen::VectorXd & wave_reference, const Eigen::VectorXd& r_i){
	return;
}

/// Publish a returning wave
void Edge::publishWave(const Eigen::VectorXd& s_out){
    
    //compressWaves(s_out);
    
    /* If the publish counter is triggered, we send our compressed data sample */
    if(publish_counter->trigger()){
        //logTmp("before", s_out);
        // The message to send
        panda::Waves msg;

        // The sub message containing the data
        std_msgs::Float64MultiArray msg_vec;

        // Put the wave in the messagge
        msg_vec.data.resize(l);
        for(int i = 0; i < l; i++){
            // This is where the compression equation gets applied
            msg_vec.data[i] = s_out(i, 0);
            //msg_vec.data[i] = helpers::sgn(s_out_compressed(i)) * std::sqrt(s_out_squared(i));  
            //logTmp("s_after(" + std::to_string(i) + ")", msg_vec.data[i]);
        }
        
        // Set the message
        msg.s = msg_vec;
        msg.timestamp = timestamp;

        // Publish the message
        wave_pub.publish(msg);

        logMsg("Edge", "Wave sent. Timestamp = " + std::to_string(timestamp) + ".", 4);

        // Increase the timestamp
        timestamp++;
        s_out_compressed = Eigen::VectorXd::Zero(l);
        s_out_squared = Eigen::VectorXd::Zero(l);

    }

}

// Expand waves for sampling
void Edge::expandWaves(Eigen::VectorXd& waves){

    for(size_t i = 0; i < l; i++){
        
        waves(i) = helpers::sgn(waves(i))*std::sqrt(std::pow(waves(i), 2.0) / (double)(rate_mp));
    }
    
}

void Edge::compressWaves(const Eigen::VectorXd& waves){
    
    s_out_compressed += waves;
    
    for(int i = 0; i < l; i++){
        
        s_out_squared(i) += std::pow(waves(i), 2);
    }
    
    
}