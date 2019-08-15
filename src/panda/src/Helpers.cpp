
#include "Helpers.h"

namespace helpers {

	Eigen::VectorXd vectorToEigen(const std::vector<double> values){

		Eigen::VectorXd result(values.size());

		for(int i = 0; i < values.size(); i++){
			result(i, 0) = values[i];
		}

		return result;

	}

	std_msgs::Float64MultiArray eigenToMessage(Eigen::VectorXd vec){

        std_msgs::Float64MultiArray msg;
        msg.data.resize(vec.rows());
        for(int i = 0; i < vec.rows(); i++){
            msg.data[i] = vec(i);
        }

        return msg;

	}

    Eigen::VectorXd messageToEigen(std_msgs::Float64MultiArray msg, int l){

	    Eigen::VectorXd vec = Eigen::VectorXd::Zero(l);
        for(int i = 0; i < l; i++){

            vec(i, 0) = msg.data[i];
        }

        return vec;
    }


    double normOf(const Eigen::VectorXd input){

		return std::sqrt(input.transpose()*input);

	}


}