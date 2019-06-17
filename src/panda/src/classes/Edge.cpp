/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l: dimension of the channel
*/



#include "Edge.h"

Edge::Edge(int i, int j, double gain, int l_set){

	std::cout << "Initialising an edge between agent " << i << " and " << j << "\n";

	//Save the connection between agents
	i_ID = i;
	j_ID = j;
	l_dim = l_set;
	
	std::cout << "s_" << i_ID << j_ID << "+\n";

	// If this is agent i, send on the positivily defined channel, listen to the negative channel
	if(i < j){
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p", 100);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m", 100, &Edge::waveCallback, this);
	}else /* Otherwise invert these */{
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "m", 100);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "p", 100, &Edge::waveCallback, this);
	}

	if(!wave_sub)
	{
		std::cout << "problem with the subscriber!\n";
	}
	// Set the scattering gain
	Edge::setScatteringGain(gain);

	// Initialise buffers
	s_buffer = Eigen::VectorXd::Zero(l_dim);//Eigen::Matrix<double, l, 1>::Zero();
	s_received = Eigen::VectorXd::Zero(l_dim);

}

// Destructor
Edge::~Edge(){}


void Edge::waveCallback(const panda::Waves::ConstPtr& msg){
	
	if(l_dim > 4)
	{
		std::cout << "[Error]: Data is corrupted!\n";
		return;
	}
	// Its not the message
	std::cout << "Edge | Data received!\n";
	

	std::vector<double> s = msg->s.data;

	// Print data
	std::cout << "	[Timestamp]: " << msg->timestamp << "\n";

	for(int i = 0; i<l_dim; i++){
		std::cout << "	[Data "<< i<< "]: " << s[i] << "\n";
		s_received(i) = s[i];
	}

	// Data was received
	data_received = true;

}



// Sample the edge, retrieving a data point for the cooperative control and returning a wave in the process
Eigen::VectorXd Edge::sampleEdge(Eigen::VectorXd r_i){
	// Reconstruct the wave if nothing was received
	Edge::applyWVM(s_received, r_i);

	// Publish the returning wave
	publishWave(r_i);

	// Return the input for this edge
	return Edge::getTauST(s_received, r_i);
}

/* Reconstruct data if necessary */
void Edge::applyWVM(Eigen::VectorXd & wave_reference, Eigen::VectorXd r_i){
	 
	 std::cout << "WVM Function | ";

	/*We need to apply WVM*/
	if(!data_received)
	{
		// Calculate the wave when we apply HLS
		Eigen::VectorXd s_HLS = Edge::getWaveST(s_buffer, r_i);

		// If that's allowed, use it, otherwise reconstruct by amplitude
		if(s_HLS.transpose()*s_HLS > s_buffer.transpose()*s_buffer)
		{
			std::cout << " HLS applied\n";
			wave_reference = Eigen::VectorXd::Zero(l_dim);
		}else{
			std::cout << " Reconstructed\n";
			wave_reference = Eigen::VectorXd::Zero(l_dim); // Change this!
		}
		
		
	}else{
		std::cout << " Use received data\n";
		data_received = false;
	}

	// Reset the buffer 
	s_buffer = s_received;
}

// Publish a returning wave
void Edge::publishWave(Eigen::VectorXd r_i){
	// The message to send
	panda::Waves msg;

	// The sub message containing the data
	std_msgs::Float64MultiArray msg_vec;

	// Calculate the input
	Eigen::VectorXd wave = Edge::getWaveST(s_received, r_i);

	// Put the wave in the messagge
	msg_vec.data.resize(l_dim);
	for(int i = 0; i < l_dim; i++){
		msg_vec.data[i] = wave(i, 0);
	}
	
	msg.s = msg_vec;
	msg.timestamp = timestamp;

	// Increase the timestamp
	timestamp++;

	// Publish the message
	wave_pub.publish(msg);

}

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd Edge::getTauST(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_tau.block(0, 0, l_dim, l_dim) * s_in + gain_tau.block(0, l_dim, l_dim, l_dim)*r_i;
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd Edge::getWaveST(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_wave.block(0, 0, l_dim, l_dim) * s_in + gain_wave.block(0, l_dim, l_dim, l_dim)*r_i;
}


// 
void Edge::setScatteringGain(double gain){
	// Define the gains on this edge and the impedance
	Eigen::MatrixXd Kd(l_dim, l_dim);
	Eigen::MatrixXd B(l_dim, l_dim);
	Kd = gain*Eigen::MatrixXd::Identity(l_dim, l_dim);
 	B = std::sqrt(gain)*Eigen::MatrixXd::Identity(l_dim, l_dim);

	// Take into account the different ST depending on which agent this is
	int agent_i = 1;
	if(i_ID < j_ID){
		agent_i = -1;
	}

	// Apply the gain
	Eigen::MatrixXd Binv = B.inverse();


	// Define the ST matrix
	Eigen::MatrixXd matrix_ST(2*l_dim, 2*l_dim);
	matrix_ST << agent_i*std::sqrt(2)*Binv, -Binv*Binv,
					-Eigen::MatrixXd::Identity(l_dim, l_dim), agent_i*std::sqrt(2)*Binv;

	// Calculate the transfer function of the controls internally
	Eigen::MatrixXd H = Eigen::MatrixXd::Identity(l_dim, l_dim) - Kd*matrix_ST.block(0,l_dim,l_dim,l_dim);
	H = H.inverse()*Kd;

	gain_tau = Eigen::MatrixXd::Zero(l_dim, 2*l_dim);	
	gain_wave = Eigen::MatrixXd::Zero(l_dim, 2*l_dim);

	gain_tau << H*matrix_ST.block(0,0,l_dim,l_dim), -H;
	gain_wave << matrix_ST.block(l_dim,0,l_dim,l_dim)+matrix_ST.block(l_dim,l_dim,l_dim,l_dim)*H*matrix_ST.block(0,0,l_dim, l_dim), -matrix_ST.block(l_dim, l_dim, l_dim, l_dim)*H;

	std::cout << "Scattering gains initialised\n";
}

