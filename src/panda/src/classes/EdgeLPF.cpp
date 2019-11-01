#include "EdgeLPF.h"



void EdgeLPF::applyReconstruction(Eigen::VectorXd& r_js, const Eigen::VectorXd& r_i)
{
}

Eigen::VectorXd EdgeLPF::calculateControls(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i)
{
}

Eigen::VectorXd EdgeLPF::calculateWaves(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i)
{
}


void EdgeLPF::publishWave(const Eigen::VectorXd& r_i)
{
}

void EdgeLPF::waveCallback(const panda::Waves::ConstPtr& msg)
{
}

Eigen::VectorXd EdgeLPF::sample(const Eigen::VectorXd& r_i)
{
}
