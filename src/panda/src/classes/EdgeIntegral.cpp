/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

#include "EdgeIntegral.h"

EdgeIntegral::EdgeIntegral(int i, int j, Eigen::MatrixXd gain_set, int l_set, bool is_integral)
	: Edge(i, j, gain_set, l_set, is_integral){

	// Set the scattering gain
	setScatteringGain(gain);

	integral_state = Eigen::VectorXd::Zero(l);
	is_activated = false;

	helpers::safelyRetrieve(nh, "controller/integral/saturation", saturation);
}


Eigen::VectorXd EdgeIntegral::sample(Eigen::VectorXd r_i){

	// ZOS
	applyReconstruction(s_received, r_i);

	// Set the input by iteration
	Eigen::VectorXd r_js = iterateST(s_received, r_i);

	integral_state += r_js - r_i;

	saturateIntegral();

	Eigen::VectorXd tau = calculateControls(r_i, r_js);

	// Calculate the input
	Eigen::VectorXd s_out = calculateWaves(tau, r_js);
	
	// Publish the returning wave
	publishWave(s_out);

	// Return the input for this edge
	return tau;
}

Eigen::VectorXd EdgeIntegral::iterateST(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	Eigen::VectorXd r_js = Eigen::VectorXd::Zero(l);

	// Calculate r_js analytically (efficiency can be improve by pre calculating)
	for(int i = 0; i < l; i++){
		r_js(i, 0) = (matrix_ST(i, i) * s_in(i, 0) + matrix_ST(i, l+i)*gain(i,i)*(integral_state(i, 0) - r_i(i, 0) ))
			/(1 - matrix_ST(i, l+i)*gain(i,i));
	}

	return r_js;

}

void EdgeIntegral::saturateIntegral(){

	for(int i = 0; i < l; i++){
		//logTmp(integral_state[i]);
		if(integral_state[i] > saturation){
			integral_state[i] = saturation;
		}else if(integral_state[i] < -saturation){
			integral_state[i] = -saturation;
		}
	}
}


/* Reconstruct data if necessary */
void EdgeIntegral::applyReconstruction(Eigen::VectorXd& wave_reference, Eigen::VectorXd r_i){

	/*We need to apply WVM*/
	if(!data_received)
	{
		// Calculate the wave when we apply HLS
		// Eigen::VectorXd s_HLS = calculateWaves(s_buffer, r_i);

		// // If that's allowed, use it, otherwise reconstruct by amplitude
		// if(s_HLS.transpose()*s_HLS > s_buffer.transpose()*s_buffer)
		// {
		//  	logMsg("Edge", "WVM applied HLS", 5);
		// 	wave_reference = s_buffer;

		// }else{
		// 	logMsg("Edge", "WVM applied reconstruction", 5);
		// 	wave_reference = elementSign(s_buffer).cwiseProduct(s_HLS.cwiseAbs());
		// }
		s_received = Eigen::VectorXd::Zero(l);
		
	}else{
		logMsg("Edge", "Used received data", 5);
		data_received = false;
		//s_buffer = s_received;
	}
	
}

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd EdgeIntegral::calculateControls(Eigen::VectorXd r_i, Eigen::VectorXd r_js){

	return gain*(integral_state);
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeIntegral::calculateWaves(Eigen::VectorXd tau, Eigen::VectorXd r_js){

	Eigen::VectorXd result = Eigen::VectorXd::Zero(l);

	for(int i = 0; i < l; i++){
		double b = std::sqrt(gain(i,i));
		result(i, 0) = agent_i/std::sqrt(2*b) * (tau(i, 0) + -b*r_js(i, 0));
	}

	return result;
}


// 
void EdgeIntegral::setScatteringGain(Eigen::MatrixXd gain){
	// Define the gains on this edge and the impedance
	Eigen::MatrixXd Kd(l, l);
	Eigen::MatrixXd B(l, l);
	Kd = gain;
 	B = gain.cwiseSqrt();

	// Take into account the different ST depending on which agent this is
	agent_i = 1;
	if(i_ID < j_ID){
		agent_i = -1;
	}

	// Apply the gain
	Eigen::MatrixXd Binv = B.inverse();


	// Define the ST matrix
	matrix_ST = Eigen::MatrixXd::Zero(2*l, 2*l);
	matrix_ST << agent_i*std::sqrt(2)*Binv, -Binv*Binv,
					-Eigen::MatrixXd::Identity(l, l), agent_i*std::sqrt(2)*Binv;
}

void EdgeIntegral::reset(){
	integral_state = Eigen::VectorXd::Zero(l);
}

void EdgeIntegral::activate(){
	is_activated = true;
}

void EdgeIntegral::deactivate(){
	is_activated = false;
}

