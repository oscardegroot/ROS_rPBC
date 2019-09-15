/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

#include "EdgeDirect.h"

EdgeDirect::EdgeDirect(int i, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set)
	: Edge(i, j, gain_set, l_set, r_star_set, rate_mp_set){

	// Set the scattering gain
	setScatteringGain(gain);
}


/* Reconstruct data if necessary */
void EdgeDirect::applyReconstruction(Eigen::VectorXd& wave_reference, Eigen::VectorXd r_i){

	/*We need to apply WVM*/
	if(!data_received)
	{
		// Calculate the wave when we apply HLS
		Eigen::VectorXd s_HLS = calculateWaves(s_buffer, r_i);

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
Eigen::VectorXd EdgeDirect::elementSign(Eigen::VectorXd s_in){

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

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd EdgeDirect::calculateControls(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i){

	return gain_tau.block(0, 0, l, l) * s_in + gain_tau.block(0, l, l, l)*r_i;
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeDirect::calculateWaves(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i){

	return gain_wave.block(0, 0, l, l) * s_in + gain_wave.block(0, l, l, l)*r_i;
}


// 
void EdgeDirect::setScatteringGain(Eigen::MatrixXd gain){
	// Define the gains on this edge and the impedance
	Eigen::MatrixXd Kd(l, l);
	Eigen::MatrixXd B(l, l);
	Kd = gain;
 	B = gain.cwiseSqrt();

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

