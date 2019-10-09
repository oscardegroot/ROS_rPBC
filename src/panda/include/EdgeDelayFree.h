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
	EdgeDelayFree(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set);

	void waveCallback(const panda::Waves::ConstPtr& msg) override;
	void publishWave(const Eigen::VectorXd& s_out) override;
	void applyReconstruction(Eigen::VectorXd & wave_reference, const Eigen::VectorXd& r_i);

	Eigen::VectorXd calculateControls(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i);
	Eigen::VectorXd calculateWaves(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i);
	Eigen::VectorXd elementSign(const Eigen::VectorXd& s_in);

	void setScatteringGain(const Eigen::MatrixXd& gain);
	Eigen::VectorXd calculateBeaconWaves(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i);


private:

	Eigen::VectorXd goal;

	// The wave impedance B and network gain Kd
	Eigen::MatrixXd gain_tau, gain_wave;
	Eigen::MatrixXd gain_wave_beacon;




};