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

Edge::Edge(int i, int j, double gain_set, int l_set, bool is_integral){

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

	// Set the scattering gain
	Edge::setScatteringGain(gain);

	// Initialise buffers
	s_buffer = Eigen::VectorXd::Zero(l);//Eigen::Matrix<double, l, 1>::Zero();
	s_received = Eigen::VectorXd::Zero(l);

	logMsg("Edge", "Done!", 2);

}

// Destructor
Edge::~Edge(){}


void Edge::waveCallback(const panda::Waves::ConstPtr& msg){
	
	logMsg("Edge", "Wave received. Timestamp = " + std::to_string(msg->timestamp) + ".", 4);

	std::vector<double> s = msg->s.data;

	// Print data
	//std::cout << "	[Timestamp]: " << msg->timestamp << "\n";

	for(int i = 0; i<l; i++){
		//std::cout << "	[Data "<< i<< "]: " << s[i] << "\n";
		s_received(i) = s[i];
	}

	// Data was received
	data_received = true;

}



// Sample the edge, retrieving a data point for the cooperative control and returning a wave in the process
Eigen::VectorXd Edge::sample(Eigen::VectorXd r_i){

	// Reconstruct the wave if nothing was received
	Edge::applyWVM(s_received, r_i);

	// Publish the returning wave
	publishWave(r_i);

	// Eigen::VectorXd tau = Eigen::VectorXd::Zero(l);
	// if(data_received){
	// 	tau = r_i - s_received;
	// }

	// Return the input for this edge
	return Edge::getTauST(s_received, r_i);
}

/* Reconstruct data if necessary */
void Edge::applyWVM(Eigen::VectorXd & wave_reference, Eigen::VectorXd r_i){


	/*We need to apply WVM*/
	if(!data_received)
	{
		// Calculate the wave when we apply HLS
		Eigen::VectorXd s_HLS = Edge::getWaveST(s_buffer, r_i);

		// If that's allowed, use it, otherwise reconstruct by amplitude
		if(s_HLS.transpose()*s_HLS > s_buffer.transpose()*s_buffer)
		{
		 	logMsg("Edge", "WVM applied HLS", 5);
			wave_reference = s_buffer;

		}else{
			logMsg("Edge", "WVM applied reconstruction", 5);
			wave_reference = elementSign(s_buffer).cwiseProduct(s_HLS.cwiseAbs());
		}
		
		
	}else{
		logMsg("Edge", "Used received data", 5);
		data_received = false;
		s_buffer = s_received;
	}
	
}

// Apply the element wise sign operator
Eigen::VectorXd Edge::elementSign(Eigen::VectorXd s_in){

	Eigen::VectorXd s_out = Eigen::VectorXd::Zero(s_in.size());
	for(int i = 0; i < s_in.size(); i++){
		if(s_in[i] > 0){
			s_out[i] = 1;
		}else{
			s_out[i] = -1;
		}
	}
	return s_out;
}

// Publish a returning wave
void Edge::publishWave(Eigen::VectorXd r_i){
	// The message to send
	panda::Waves msg;

	// The sub message containing the data
	std_msgs::Float64MultiArray msg_vec;

	// Calculate the input (I forgot to disable this!)
	Eigen::VectorXd wave = Edge::getWaveST(s_received, r_i);

	// Put the wave in the messagge
	msg_vec.data.resize(l);
	for(int i = 0; i < l; i++){
		msg_vec.data[i] = wave(i, 0);
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

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd Edge::getTauST(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_tau.block(0, 0, l, l) * s_in + gain_tau.block(0, l, l, l)*r_i;
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd Edge::getWaveST(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_wave.block(0, 0, l, l) * s_in + gain_wave.block(0, l, l, l)*r_i;
}


// 
void Edge::setScatteringGain(double gain){
	// Define the gains on this edge and the impedance
	Eigen::MatrixXd Kd(l, l);
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

	//LOG(gain_wave);
	//std::cout << "[Edge]	Scattering Initialised\n";
}

