/*
File: FlexibleEdge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleFlexibleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

/* No ST | Flexible controls */

#include "EdgeFlexDelayFree.h"

EdgeFlexDelayFree::EdgeFlexDelayFree(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set)
	: Edge(agent, j, gain_set, l_set, r_star_set, rate_mp_set)
{

	//std::vector<double> goal_v;
	helpers::safelyRetrieveEigen(nh, "/beacon/goal", goal, 3);
	//goal = helpers::vectorToEigen(goal_v);
}



void EdgeFlexDelayFree::waveCallback(const panda::Waves::ConstPtr& msg){
	return;
}

/* Reconstruct data if necessary */
void EdgeFlexDelayFree::applyReconstruction(Eigen::VectorXd & wave_reference,
									 Eigen::VectorXd r_i){

	// Instead of reconstruction we set the data here
	// such to have no delay
	wave_reference = goal;
}

// // Sample the edge, retrieving a data point for the cooperative control and returning a wave in the process
// Eigen::VectorXd FlexibleEdge::sample(Eigen::VectorXd r_i) {

// 	// Reconstruct the wave if nothing was received
// 	//applyWVM(r_i);
// 	Eigen::VectorXd tau_jsi = Eigen::VectorXd::Zero(l);
// 	Eigen::VectorXd r_js(l);

// 	//Look at this!
// 	for(int i = 0; i < 20; i++){
// 		r_js = getRjs(s_received, tau_jsi);
// 		tau_jsi = applyControls(r_i, r_js);
// 	}

// 	logTmp(tau_jsi);
// 	//logTmp(r_js);
	
// 	Eigen::VectorXd s_out = getWave(r_js, tau_jsi);

// 	// Publish the returning wave
// 	publishWave(s_out);

// 	last_tau = tau_jsi;
// 	// Return the input for this edge
// 	return tau_jsi;
// }

void EdgeFlexDelayFree::publishWave(Eigen::VectorXd s_out) {
		
	// Save the outgoing message to calculate the waves in the next 
	// time step
	//s_buffer = s_out;	
	return;
}

// /* Reconstruct data if necessary */
// void applyReconstruction(Eigen::VectorXd & wave_reference,
// 									 Eigen::VectorXd r_i){

// 	/*We need to apply WVM*/
// 	if(!data_received)
// 	{
// 		s_received = Eigen::VectorXd::Zero(l);
// 	}else{
// 		logMsg("FlexibleEdge", "Used received data", 5);
// 		data_received = false;
// 		s_buffer = s_received;
// 	}
	
// }

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd EdgeFlexDelayFree::calculateWaves(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i){

	// The output is the wave = z_j
	return r_i;
}

Eigen::VectorXd EdgeFlexDelayFree::calculateControls(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i){

	// For now simple and linear
	//
	return gamma_wang(r_i, s_in)*gain*(s_in - r_i);

}

double EdgeFlexDelayFree::gamma_wang(Eigen::VectorXd r_i, Eigen::VectorXd r_js){

	double d = std::sqrt((r_js - r_i).transpose()*(r_js - r_i));

	//logTmp(d);
	double Rw = 1.0;
	double eps = 0.001;	
	double r = 2*Rw*eps;

	double a1 = (4*eps*Rw*Rw + 4*eps*Rw*r - 3*r*r) / (4*Rw*std::pow(r, 3)*(r-2*Rw));
	double b1 = (3*r*r-12*eps*Rw*Rw)/(4*Rw*r*r*(r-2*Rw));
	double a2 = (4*eps*Rw*r-3*r*r+8*Rw*r-12*eps*Rw*Rw)/(4*Rw*r*std::pow(r-2*Rw, 3));
	double b2 = (-12*eps*Rw*Rw*r-24*Rw*Rw*r+3*std::pow(r, 3) + 48*eps*std::pow(Rw, 3))/(4*Rw*r*std::pow(r-2*Rw, 3));
	double c2 = (9*Rw*r*r-12*eps*std::pow(Rw, 3) - 3*std::pow(r, 3))/(r*std::pow(r-2*Rw, 3));

	if(d >= 2*Rw){
		return 0.0;
	}

	if(d < r){

	 	return 3*a1*d*d + 2*b1*d;
	}

	return 3*a2*d*d+2*b2*d+c2;

}