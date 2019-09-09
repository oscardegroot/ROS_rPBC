/*
File: rPBC.h

A standard IDA-PBC controller, implements the controller interface defined in Controller.h
*/

#ifndef rPBC_H
#define rPBC_H

#include "ros/ros.h"
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include "Controller.h"
#include "CustomLog.h"
#include "Helpers.h"
#include "IDAPBC.h"


class rPBC : public IDAPBC{

public:
	rPBC(System& system);

	Eigen::VectorXd computeControl(System& system, Eigen::VectorXd tau_c) override;

	// Get the agent output, possibly modified
	Eigen::VectorXd getOutput(System& system) override;

	Eigen::MatrixXd rightPseudoInverse(Eigen::MatrixXd A);


private:

	double lambda;

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


};


#endif