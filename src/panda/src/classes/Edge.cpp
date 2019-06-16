/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM
i: agent id
j: connected agent id
agent_i: is this agent the positive power agent (true/false)
gain: gain of the connection 
*/



#include "Edge.h"

Edge::Edge(int i, int j, double gain, int l_dim){

	std::cout << "Initialising an edge between agent " << i << " and " << j << "\n";
	//Save the connection between agents
	i_ID = i;
	j_ID = j;
	l = l_dim;

	//std::cout << "s_" << i_ID << j_ID << "+\n";

	// If this is agent i, send on the positivily defined channel, listen to the negative channel
	if(i < j){
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p", 10);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m", 10, &Edge::waveCallback,this);
	}else /* Otherwise invert these */{
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "m", 10);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "p", 10, &Edge::waveCallback, this);
	}

	// Set the scattering gain
	setScatteringGain(gain);

	// Initialise buffers
	s_buffer = Eigen::VectorXd(l);//Eigen::Matrix<double, l, 1>::Zero();
	s_received = Eigen::VectorXd(l);

}

// Destructor
Edge::~Edge(){}


void Edge::waveCallback(const panda::Waves::ConstPtr& msg){
	
	std::cout << "Data received!\n";
	std::cout << "	[Timestamp]: " << msg->timestamp << "\n";

	std::vector<double> s = msg->s.data;
	// Print data
	for(int i = 0; i<l; i++){
		std::cout << "	[Data "<< i<< "]: " << s[i] << "\n";
		s_received(i) = s[i];
	}

	// Data was received
	data_received = true;

	// Save S
	// for i = 0; i<l
	// s_received(0, 0) = s[0];
	// s_received(1, 0) = s[1];

}



// Sample the edge, retrieving a data point for the cooperative control and returning a wave in the process
Eigen::VectorXd Edge::sampleEdge(Eigen::VectorXd r_i){
	
	// Reconstruct the wave if nothing was received
	applyWVM(&s_received, r_i);
	
	// Reset the buffer 
	s_buffer = s_received;
	data_received = false;

	// Publish the returning wave
	publishWave(s_received, r_i);

	// Return the input for this edge
	return getTauST(s_received, r_i);
}

// Publish a returning wave
void Edge::publishWave(Eigen::VectorXd s_in, Eigen::VectorXd r_i){
	// The message to send
	panda::Waves msg;

	// The sub message containing the data
	std_msgs::Float64MultiArray msg_vec;

	// Calculate the input
	Eigen::VectorXd tau = getTauST(s_in, r_i);


	// Put tau in the messagge
	msg_vec.data.resize(l);
	for(int i = 0; i < l; i++){
		msg_vec.data[i] = tau(i, 0);
	}
	
	
	msg.s = msg_vec;
	msg.timestamp = timestamp;

	// Increase the timestamp
	timestamp++;

	// Publish the message
	wave_pub.publish(msg);

}


/* Reconstruct data if necessary */
void Edge::applyWVM(Eigen::VectorXd* wave_pointer, Eigen::VectorXd r_i){
	 
	 std::cout << "WVM Function:\n";
	/*We need to apply WVM*/
	if(!data_received)
	{
		// Calculate the wave when we apply HLS
		auto s_HLS = getWaveST(s_buffer, r_i);

		// If that's allowed, use it, otherwise reconstruct by amplitude
		if(s_HLS.transpose()*s_HLS > s_buffer.transpose()*s_buffer)
		{
			std::cout << "	[action]: HLS applied\n";
			*wave_pointer = s_buffer;
		}else{
			std::cout << "	[action]: Reconstructed\n";
			*wave_pointer = s_buffer; // Change this!
		}

	}else{
		std::cout << "	[action]: Use received data\n";
	}
}

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd Edge::getTauST(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_tau.block(0, 0, l, l) * s_in + gain_tau.block(0, l, l, l)*r_i;
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd Edge::getWaveST(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_wave.block(0, 0, l, l) * s_in + gain_wave.block(0, l, l, l)*r_i;
}


void Edge::setScatteringGain(double gain){
	// Define the gains on this edge and the impedance
	Eigen::MatrixXd Kd (l, l);
	Eigen::MatrixXd B(l, l);
	Kd = gain*Eigen::MatrixXd::Identity(l, l);
 	B = std::sqrt(gain)*Eigen::MatrixXd::Identity(l, l);

	// Take into account the different ST depending on which agent this is
	int agent_i = 1;
	if(i_ID < j_ID){
		agent_i = -1;
	}

	// Apply the gain
	Eigen::MatrixXd Binv = B.inverse();


	// Define the ST matrix
	Eigen::MatrixXd matrix_ST(2*l, 2*l);
	matrix_ST << agent_i*std::sqrt(2)*Binv, -Binv*Binv,
					-Eigen::MatrixXd::Identity(l, l), agent_i*std::sqrt(2)*Binv;

	// Calculate the transfer function of the controls internally
	Eigen::MatrixXd H = Eigen::MatrixXd::Identity(l, l) - Kd*matrix_ST.block(0,l,l,l);
	H = H.inverse()*Kd;

	gain_tau = Eigen::MatrixXd::Zero(l, 2*l);	
	gain_wave = Eigen::MatrixXd::Zero(l, 2*l);

	gain_tau << H*matrix_ST.block(0,0,l,l), -H;
	gain_wave << matrix_ST.block(l,0,l,l)+matrix_ST.block(l,l,l,l)*H*matrix_ST.block(0,0,l, l), -matrix_ST.block(l, l, l, l)*H;

	std::cout << "Scattering gains initialised\n";
}

