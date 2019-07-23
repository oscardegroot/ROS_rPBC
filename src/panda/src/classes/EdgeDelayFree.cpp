/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

#include "EdgeDelayFree.h"

EdgeDelayFree::EdgeDelayFree(int i, int j, Eigen::MatrixXd gain_set, int l_set, bool is_integral)
	: Edge(i, j, gain_set, l_set, is_integral){

	setScatteringGain(gain_set);

	std::vector<double> goal_v;
	helpers::safelyRetrieveArray(nh, "/beacon/goal", goal_v, 3);
	goal = helpers::vectorToEigen(goal_v);
}


void EdgeDelayFree::waveCallback(const panda::Waves::ConstPtr& msg){
	return;
}


/* Reconstruct data if necessary */
void EdgeDelayFree::applyReconstruction(Eigen::VectorXd& wave_reference, Eigen::VectorXd r_i){

	// Instead of reconstruction we set the data here
	// such to have no delay
	wave_reference = calculateBeaconWaves(s_buffer, goal);

}

void EdgeDelayFree::publishWave(Eigen::VectorXd s_out) {
		
	// Save the outgoing message to calculate the waves in the next 
	// time step
	s_buffer = s_out;	
}

// Retrieve tau from the scattering transforma)tion
Eigen::VectorXd EdgeDelayFree::calculateControls(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_tau.block(0, 0, l, l) * s_in + gain_tau.block(0, l, l, l)*r_i;
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeDelayFree::calculateWaves(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_wave.block(0, 0, l, l) * s_in + gain_wave.block(0, l, l, l)*r_i;
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeDelayFree::calculateBeaconWaves(Eigen::VectorXd s_in, Eigen::VectorXd r_i){

	return gain_wave_beacon.block(0, 0, l, l) * s_in + gain_wave_beacon.block(0, l, l, l)*r_i;
}


// 
void EdgeDelayFree::setScatteringGain(Eigen::MatrixXd gain){
	// Define the gains on this edge and the impedance
	Eigen::MatrixXd Kd(l, l);
	Eigen::MatrixXd B(l, l);
	Kd = gain;//gain*Eigen::MatrixXd::Identity(l, l);
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

	// TF for the beacon
	Eigen::MatrixXd matrix_ST_beacon(2*l, 2*l);
	matrix_ST_beacon << -1*agent_i*std::sqrt(2)*Binv, -Binv*Binv,
					-Eigen::MatrixXd::Identity(l, l), -1*agent_i*std::sqrt(2)*Binv;
	
	Eigen::MatrixXd H_beacon = Eigen::MatrixXd::Identity(l, l) - Kd*matrix_ST_beacon.block(0,l,l,l);
	H_beacon = H_beacon.inverse()*Kd;

	//gain_tau = Eigen::MatrixXd::Zero(l, 2*l);	
	gain_wave_beacon = Eigen::MatrixXd::Zero(l, 2*l);

	//gain_tau << H_beacon*matrix_ST_beacon.block(0,0,l,l), -H_beacon;
	gain_wave_beacon << matrix_ST_beacon.block(l,0,l,l)+
	matrix_ST_beacon.block(l,l,l,l)*H_beacon*matrix_ST_beacon.block(0,0,l, l),
	 -matrix_ST_beacon.block(l, l, l, l)*H_beacon;


	//LOG(gain_wave);
	//std::cout << "[Edge]	Scattering Initialised\n";
}

