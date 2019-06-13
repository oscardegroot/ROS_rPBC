/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM
*/



#include "Edge.h"

Edge::Edge(int i, int j, bool agent_i, double gain){
	//Save the connection between agents
	i_ID = i;
	j_ID = j;
	is_agent_i = agent_i;

	std::cout << "s_" << i_ID << j_ID << "+";

	// // If this is agent i, send on the positivily defined channel, listen to the negative channel
	if(agent_i){
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "+", 10);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "-", 10, &Edge::waveCallback,this);
	}else /* Otherwise invert these */{
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "-", 10);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "+", 10, &Edge::waveCallback, this);
	}

	timestamp = 0;

	//Define the gains on this edge and the impedance
	Kd = Eigen::Matrix<double, 2, 2>::Identity();
	B = Eigen::Matrix<double, 2, 2>::Identity(); 
	Kd *= gain;
	B *= std::sqrt(gain);

	// Initialise buffers
	s_buffer = Eigen::Matrix<double, 2, 1>::Zero();
	s_received = Eigen::Matrix<double, 2, 1>::Zero();

}

// Destructor
Edge::~Edge(){}


void Edge::waveCallback(const panda::Waves::ConstPtr& msg){
	
	std::cout << "Timestamp: " << msg->timestamp;

	for(int i = 0; i<2; i++){
		std::cout << "Data["<< i<< "]: " << msg->s;
	}



}

Eigen::Matrix<double, 2, 1> Edge::sampleEdge(Eigen::Matrix<double, 2, 1> r_i){
	return Eigen::Matrix<double, 2, 1>::Zero();
}
