/**
    @file: System.h

    @brief Defines an interface for nonlinear control of systems
     * Defines matrices modelled or retrieved for control computation and includes functions to read sensors and actuate 
*/ 

#ifndef System_H
#define System_H

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "panda/getConnectionsOf.h"
#include "panda/registerAgent.h"

#include "Helpers.h"
#include "Agent.h"
#include "CMM.h"

#include <vector>
#include <string>
#include <memory>

/** Saves relevant state information 
 * @variable q Coordinates/State variables of the system
 * @variable dq Derivatives of the above (mechanically: velocities) 
 * @variable z End-effector coordinates */
struct State{
	Eigen::VectorXd q;
	Eigen::VectorXd dq;
	Eigen::VectorXd z;
};

class System{

public:
	System(int n_set, int m_set, int lmax_set, const std::string& name);
	virtual ~System();

    // # Coordinates, # Actuation degree, # Maximal cooperative coordinates of EE
	int n = 0;
	int m = 0;
	int lmax = 0;
    
    // System coordinate state (q, qdot, z)
	State state;
    
    // Communication Management Module, handles cooperative communication
    std::unique_ptr<CMM> cmm;
    
	/** @brief Systems may not always want to cooperate in all the DOF of their end-effectors. 
     * The selector matrices multiply with the z and psi matrices such that only the selected DOF remain */
    void initSelectors();
    Eigen::MatrixXd selectPsi(const Eigen::MatrixXd& psi);
    Eigen::VectorXd selectZ(const Eigen::VectorXd& z);

	virtual
    void readSensors();
    
	virtual
    bool sendInput(const Eigen::VectorXd & tau) = 0;
	
    /** @brief Matrices are saved in this class on the heap and updated only once per sampling instance
    * functions return references*/
    
    /** @return reference to n x n mass matrix */
    virtual
    Eigen::MatrixXd& M() = 0;
    
    /** @return reference to n x 1 gravity vector */
	virtual
    Eigen::VectorXd& dVdq() = 0;
    
    /** @return reference to n x l end-effector Jacobian */
	virtual
    Eigen::MatrixXd& Psi() = 0;
    
    /** @return reference to n x n mass matrix derivative */
    virtual
    Eigen::MatrixXd& dM() = 0;
    
    /** @return reference to n x n derivative of the inverse of the mass matrix (d/dt(M^{-1})) */
    virtual
    Eigen::MatrixXd& dMinv() = 0;

    /** @return reference to n x l derivative of the end-effector Jacobian */
    virtual
    Eigen::MatrixXd& dPsi() = 0;

    virtual
    Eigen::VectorXd Psi_z(){
        return Eigen::VectorXd::Zero(lmax);
    }
    
    virtual double get_z(){
        return 0.0;
    }

    // Test!
    virtual
    Eigen::VectorXd& dTdq();

    virtual
    bool dataReady();
        
    /** Set the internal state of the system
     * @param[in] new_q coordinates.
     * @param[in] new_qd velocities.
     * @param[in] new_z end-effector coordinates. */
	void setState(Eigen::VectorXd new_q, Eigen::VectorXd new_qd, Eigen::VectorXd new_z);

    /** Reset all updated parameters */
    void resetUpdatedFlags();

    /** Check the safety state of this system and throw an exception if safety is violated */
	virtual
    void checkSafety();

    /**
     * @brief Enable this system, starting the controller
     */
    bool enableSystem(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool isEnabled() const { return is_enabled;};

protected:

    // Selector for Psi and z
    //Eigen::MatrixXd S;
    std::unique_ptr<Selector> selector;
    
    // Aproximates derivatives of matrices using (x[k] - x[k-1]) / h
    Eigen::MatrixXd approximateDerivative(const Eigen::MatrixXd& A_current, const Eigen::MatrixXd& A_previous) const;

    // System matrices
    Eigen::MatrixXd m_m, psi, dm, dpsi, dminv;
    Eigen::VectorXd dvdq;
    Eigen::MatrixXd psi_previous, m_previous;

    bool is_enabled = false;
    ros::ServiceServer enable_server;

    // Update parameters
    bool m_updated, dm_updated, psi_updated, dpsi_updated, dvdq_updated, dminv_updated;

};


#endif