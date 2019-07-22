/*
File: FlexibleEdge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleFlexibleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

#define LOG(msg) std::cout << msg << std::endl

#include "FlexibleEdge.h"

FlexibleEdge::FlexibleEdge(int i, int j, double gain_set, int l_set){

	logMsg("FlexibleEdge", "Initiating edge between " + std::to_string(i) + " and " + std::to_string(j) + "..", 2);

	//Save the connection between agents
	i_ID = i;
	j_ID = j;
	gain = gain_set;
	l = l_set;

	// If this is agent i, send on the positivily defined channel, listen to the negative channel
	if(i < j){
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "p", 100);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(i_ID) + std::to_string(j_ID) + "m", 100, &FlexibleEdge::waveCallback, this);
	}else /* Otherwise invert these */{
		wave_pub = nh.advertise<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "m", 100);
		wave_sub = nh.subscribe<panda::Waves>("s_" + std::to_string(j_ID) + std::to_string(i_ID) + "p", 100, &FlexibleEdge::waveCallback, this);
	}

	// Set the scattering gain
	setScatteringGain();

	// Initialise buffers
	s_buffer = Eigen::VectorXd::Zero(l);
	s_received = Eigen::VectorXd::Zero(l);
	last_tau = Eigen::VectorXd::Zero(l);

	data_received = false;

	logMsg("FlexibleEdge", "Done!", 2);

}

// Destructor
FlexibleEdge::~FlexibleEdge(){}


void FlexibleEdge::waveCallback(const panda::Waves::ConstPtr& msg){
	
	logMsg("FlexibleEdge", "Wave received. Timestamp = " + std::to_string(msg->timestamp) + ".", 4);

	std::vector<double> s = msg->s.data;

	for(int i = 0; i<l; i++){

		s_received(i,0) = s[i];
	}

	// Data was received
	data_received = true;

}



// Sample the edge, retrieving a data point for the cooperative control and returning a wave in the process
Eigen::VectorXd FlexibleEdge::sample(Eigen::VectorXd r_i){

	// Reconstruct the wave if nothing was received
	//applyWVM(r_i);
	logTmp(s_received);
	Eigen::VectorXd tau_jsi = Eigen::VectorXd::Zero(l);
	Eigen::VectorXd r_js(l);

	//Look at this!
	for(int i = 0; i < 20; i++){
		r_js = getRjs(s_received, tau_jsi);
		tau_jsi = applyControls(r_i, r_js);
	}

	logTmp(tau_jsi);
	//logTmp(r_js);
	
	Eigen::VectorXd s_out = getWave(r_js, tau_jsi);

	// Publish the returning wave
	publishWave(s_out);

	last_tau = tau_jsi;
	// Return the input for this edge
	return tau_jsi;
}

Eigen::VectorXd FlexibleEdge::applyControls(Eigen::VectorXd r_i, Eigen::VectorXd r_js){

	// For now simple and linear
	return gain*(r_js - r_i);

}

/* Reconstruct data if necessary */
void FlexibleEdge::applyWVM(Eigen::VectorXd r_i){


	/*We need to apply WVM*/
	if(!data_received)
	{
		// Calculate the wave when we apply HLS
		// Eigen::VectorXd s_HLS = FlexibleEdge::getWaveST(s_buffer, r_i);

		// // If that's allowed, use it, otherwise reconstruct by amplitude
		// if(s_HLS.transpose()*s_HLS > s_buffer.transpose()*s_buffer)
		// {
		//  	logMsg("FlexibleEdge", "WVM applied HLS", 5);
		// 	wave_reference = s_buffer;

		// }else{
		// 	logMsg("FlexibleEdge", "WVM applied reconstruction", 5);
		// 	wave_reference = elementSign(s_buffer).cwiseProduct(s_HLS.cwiseAbs());
		// }
		s_received = Eigen::VectorXd::Zero(l);
	}else{
		logMsg("FlexibleEdge", "Used received data", 5);
		data_received = false;
		s_buffer = s_received;
	}
	
}

// Apply the element wise sign operator
Eigen::VectorXd FlexibleEdge::elementSign(Eigen::VectorXd s_in){

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
void FlexibleEdge::publishWave(Eigen::VectorXd s_out){
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

	logMsg("FlexibleEdge", "Wave sent. Timestamp = " + std::to_string(timestamp) + ".", 4);

	// Increase the timestamp
	timestamp++;

}

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd FlexibleEdge::getRjs(Eigen::VectorXd s_in, Eigen::VectorXd tau){

	// Good logTmp(agent_i*std::sqrt(2.0*b));
	return (1.0/b)*(agent_i*std::sqrt(2.0*b)*s_in - tau);

	//return T.block(0, 0, l, l) * s_in + T.block(0, l, l, l)*tau_in;
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd FlexibleEdge::getWave(Eigen::VectorXd r_js, Eigen::VectorXd tau_in){

	return (agent_i/(std::sqrt(2.0*b)))*(-tau_in + b*r_js);

	// Careful! r_js here, not s_in!!
	//return T.block(l, 0, l, l) * r_js + T.block(l, l, l, l)*tau_in;
}


// 
void FlexibleEdge::setScatteringGain(){
	// Define the gains on this edge and the impedance
	// Eigen::MatrixXd Kd(l, l);
	// Eigen::MatrixXd B(l, l);
	// Kd = gain*Eigen::MatrixXd::Identity(l, l);
 // 	B = std::sqrt(gain)*Eigen::MatrixXd::Identity(l, l);

	// Take into account the different ST depending on which agent this is
	b = std::sqrt(gain);

	agent_i = 1.0;
	if(i_ID < j_ID){
		agent_i = -1.0;
	}

	// Apply the gain
	// Eigen::MatrixXd Binv = B.inverse();

	// // Define the ST matrix
	// T = Eigen::MatrixXd::Zero(2*l, 2*l);
	// T << agent_i*std::sqrt(2)*Binv, -Binv*Binv,
	// 				-Eigen::MatrixXd::Identity(l, l), agent_i*std::sqrt(2)*Binv;

	// logTmp(T);



	// // Calculate the transfer function of the controls internally
	// Eigen::MatrixXd H = Eigen::MatrixXd::Identity(l, l) - Kd*matrix_ST.block(0,l,l,l);
	// H = H.inverse()*Kd;

	// gain_tau = Eigen::MatrixXd::Zero(l, 2*l);	
	// gain_wave = Eigen::MatrixXd::Zero(l, 2*l);

	// gain_tau << H*matrix_ST.block(0,0,l,l), -H;
	// gain_wave << matrix_ST.block(l,0,l,l)+matrix_ST.block(l,l,l,l)*H*matrix_ST.block(0,0,l, l), -matrix_ST.block(l, l, l, l)*H;

	//LOG(gain_wave);
	//std::cout << "[FlexibleEdge]	Scattering Initialised\n";
}

