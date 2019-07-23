/*
File: Edge.h

Define a connection with another agent that uses the ST and WVM to passify communication
*/

#pragma once

#include "Edge.h"
#include "Helpers.h"

class EdgeDelayFree : public Edge{
public:
	// Constructor and destructor
	EdgeDelayFree(int i, int j, Eigen::MatrixXd gain_set, int l_set, bool is_integral);

	void waveCallback(const panda::Waves::ConstPtr& msg) override;
	void publishWave(Eigen::VectorXd s_out) override;
	void applyReconstruction(Eigen::VectorXd & wave_reference, Eigen::VectorXd r_i);

	Eigen::VectorXd calculateControls(Eigen::VectorXd s_in, Eigen::VectorXd r_i);
	Eigen::VectorXd calculateWaves(Eigen::VectorXd s_in, Eigen::VectorXd r_i);
	Eigen::VectorXd elementSign(Eigen::VectorXd s_in);

	void setScatteringGain(Eigen::MatrixXd gain);
	Eigen::VectorXd calculateBeaconWaves(Eigen::VectorXd s_in,
		Eigen::VectorXd r_i);


private:

	Eigen::VectorXd goal;

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd gain_tau, gain_wave;
	Eigen::MatrixXd gain_wave_beacon;




};