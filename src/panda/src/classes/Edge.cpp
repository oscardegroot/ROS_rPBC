/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

#define LOG(msg) std::cout << msg << std::endl

#include "Edge.h"

Edge::Edge(int i, int j, Eigen::MatrixXd gain_set, int l_set, bool is_integral){

	logMsg("Edge", "Initiating edge between " + std::to_string(i) + " and " + std::to_string(j) + "..", 2);

	//Save the connection between agents
	i_ID = i;
	j_ID = j;
	l = l_set;
	gain = gain_set;

	std::string integral_add = "";
	if(is_integral){
		integral_add = "_i";
	}

	// If this is agent i, send on the positivily defined channel, listen to the negative channel
	if(i < j){
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p" + integral_add, 100);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m" + integral_add, 100, &Edge::waveCallback, this);
	}else /* Otherwise invert these */{
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "m" + integral_add, 100);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "p" + integral_add, 100, &Edge::waveCallback, this);
	}

	// Initialise buffers
	s_buffer = Eigen::VectorXd::Zero(l);
	s_received = Eigen::VectorXd::Zero(l);

	logMsg("Edge", "Done!", 2);

}

// Destructor
Edge::~Edge(){}

// Sample the edge, retrieving a data point for the cooperative control and returning a wave in the process
Eigen::VectorXd Edge::sample(Eigen::VectorXd r_i){

	// Reconstruct the wave if nothing was received
	applyReconstruction(s_received, r_i);

	// Calculate the input (I forgot to disable this!)
	Eigen::VectorXd s_out = calculateWaves(s_received, r_i);

	// Publish the returning wave
	publishWave(s_out);

	// Return the input for this edge
	return calculateControls(s_received, r_i);
}

void Edge::waveCallback(const panda::Waves::ConstPtr& msg){
	
	logMsg("Edge", "Wave received. Timestamp = " + std::to_string(msg->timestamp) + ".", 4);

	std::vector<double> s = msg->s.data;

	for(int i = 0; i<l; i++){

		s_received(i, 0) = s[i];
	}

	// Data was received
	data_received = true;
}

// Publish a returning wave
void Edge::publishWave(Eigen::VectorXd s_out){
	// The message to send
	panda::Waves msg;

	// The sub message containing the data
	std_msgs::Float64MultiArray msg_vec;

	// Put the wave in the messagge
	msg_vec.data.resize(l);
	for(int i = 0; i < l; i++){
		msg_vec.data[i] = s_out(i, 0);
	}
	
	// Set the message
	msg.s = msg_vec;
	msg.timestamp = timestamp;

	// Publish the message
	wave_pub.publish(msg);

	logMsg("Edge", "Wave sent. Timestamp = " + std::to_string(timestamp) + ".", 4);

	// Increase the timestamp
	timestamp++;

}