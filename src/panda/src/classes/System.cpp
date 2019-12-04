/*
File: System.cpp

Defines an interface for systems controlled by IDAPBC or rPBC
*/

#include "System.h"


System::System(int n_set, int m_set, int lmax_set, const std::string& name)
	: n(n_set), m(m_set), lmax(lmax_set)
{
    // Visual Benchmarking
    Instrumentor::Get().BeginSession("system", name);

    PROFILE_SCOPE("System Init");
   
    // Initialise the communication management module
    cmm = std::make_unique<CMM>(name);

    ros::NodeHandle nh;
    enable_server = nh.advertiseService("/Agent_" + std::to_string(cmm->agent->getID()) + "/enable", &System::enableSystem, this);

    // Select cooperative coordinates
    selector = std::make_unique<Selector>(*(cmm->agent), lmax, "z_select", std::vector<int>{1, 2, 3});
    
    // Local dimensions
    s = n - selector->dim();
    
    // Use the coordinate count to initialise matrices
    psi = Eigen::MatrixXd::Zero(n, selector->dim());
    dpsi = Eigen::MatrixXd::Zero(n, selector->dim());
    psi_previous = psi;
    
    m_m = Eigen::MatrixXd::Zero(n, n);
    dm = Eigen::MatrixXd::Zero(n, n);
    dminv = Eigen::MatrixXd::Zero(n, n);
    dvdq = Eigen::VectorXd::Zero(n);
    
    m_previous = m_m; /* This should be initialised such that the initial values aren infinite...*/
}
		

System::~System(){
    Instrumentor::Get().EndSession();
}

void System::setState(Eigen::VectorXd new_q, Eigen::VectorXd new_dq, Eigen::VectorXd new_z){
	state.q = new_q;
	state.dq = new_dq;
	state.z = selectZ(new_z);
}

/// Keep only currently cooperative controlled parts
Eigen::MatrixXd System::selectPsi(const Eigen::MatrixXd& psi){
    return selector->select(psi.transpose()).transpose();
}

Eigen::VectorXd System::selectZ(const Eigen::VectorXd& z){
    return selector->select(z);
}

/** @brief Currently backwards finite difference approximation of the derivative */
Eigen::MatrixXd System::approximateDerivative(const Eigen::MatrixXd& A_current, const Eigen::MatrixXd& A_previous) const {
    
    return (double)(cmm->agent->getSamplingRate()) * (A_current - A_previous);
}

void System::resetUpdatedFlags()
{
    m_updated = false;
    psi_updated = false;
    dvdq_updated = false;
    dminv_updated = false;
    dm_updated = false;
    dpsi_updated = false;
}

bool System::enableSystem(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    is_enabled = true;
    
    return true;
}
