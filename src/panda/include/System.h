/*
File: System.h

Defines an interface for systems controlled by IDAPBC or rPBC
*/

#ifndef System_H
#define System_H

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "panda/getConnectionsOf.h"
#include "panda/registerAgent.h"

#include "Helpers.h"

#include <vector>
#include <string>
#include <sstream>


// The relevant information for parameter retrieval and connectivity
struct Agent{
    std::string type; // i.e. "panda" (global implicit = "/panda")
    std::string name; // i.e. "agent_1"
    int ID;
    int sampling_rate;
    //Status status;

    panda::registerAgent toSrv(){
        panda::registerAgent srv;
        srv.request.type = type;
        srv.request.name = name;
        srv.request.id = ID;
        srv.request.sampling_rate = sampling_rate;

        return srv;
    }

    void print(){
        std::cout << "Agent: " << name << " (" << type << ") - ID: " << ID << ", Rate: " << sampling_rate <<".\n";
    }
};

// A data structure for general state information
struct State{
	Eigen::VectorXd q;
	Eigen::VectorXd dq;
	Eigen::VectorXd z;
};

class System{

public:
	System(int n_set, int m_set, int lmax_set);
	virtual ~System();

	// Coordinate count, actuated count
	int n = 0;
	int m = 0;
	int lmax = 0;
	State state;
    Agent agent;

    // Selector for Psi and z
    Eigen::MatrixXd S;
    void initSelectors();
    Eigen::MatrixXd selectPsi(const Eigen::MatrixXd& psi);
    Eigen::VectorXd selectZ(const Eigen::VectorXd& z);

	virtual void readSensors();
	virtual bool sendInput(const Eigen::VectorXd & tau) = 0;
	
    virtual void M(Eigen::MatrixXd& M_out) = 0;
	virtual void dVdq(Eigen::VectorXd& dVdq_out) = 0;
	virtual void Psi(Eigen::MatrixXd& Psi_out) = 0;

    void setAgent(ros::NodeHandle& nh, const std::string type_name);

    virtual bool dataReady();
	// void setDataReady(bool is_ready);
	int getN(){return n;};
	void setState(Eigen::VectorXd new_q, Eigen::VectorXd new_qd, Eigen::VectorXd new_z);

	virtual void checkSafety();

private:
	

};


#endif