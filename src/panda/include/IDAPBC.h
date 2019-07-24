/*
File: IDAPBC.h

A standard IDA-PBC controller, implements the controller interface defined in Controller.h
*/

#ifndef IDAPBC_H
#define IDAPBC_H

#include "ros/ros.h"
#include "Controller.h"
#include "CustomLog.h"
#include "Helpers.h"

class IDAPBC : public Controller{

public:
	IDAPBC(System& system);

	Eigen::VectorXd computeControl(System& system, Eigen::VectorXd tau_c);

	// Get the agent output, possibly modified
	Eigen::VectorXd getOutput(System& system);

	/* Parameters necessary for IDA-PBC control */
	// Define the local gradient
	Eigen::VectorXd getdVsdq(System& system);

	// Define damping
	Eigen::MatrixXd getKv(System& system);

	// Gains
	Eigen::VectorXd Vs_gains, theta_star, limit_avoidance_gains, limits_avg;
	bool local_enabled, limit_avoidance_enabled, gravity_enabled,integral_enabled;
	double kq, kz, ki;

	Eigen::VectorXd integral_state;


	template <class MatT>
	Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
	pseudoInverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
	{
	    typedef typename MatT::Scalar Scalar;
	    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	    const auto &singularValues = svd.singularValues();
	    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
	    singularValuesInv.setZero();
	    for (unsigned int i = 0; i < singularValues.size(); ++i) {
	        if (singularValues(i) > tolerance)
	        {
	            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
	        }
	        else
	        {
	            singularValuesInv(i, i) = Scalar{0};
	        }
	    }
	    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
	}



private:

	Eigen::MatrixXd rightPseudoInverse(Eigen::MatrixXd A);

};


#endif