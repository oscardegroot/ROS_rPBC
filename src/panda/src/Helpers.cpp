
#include "Helpers.h"

namespace helpers {

	Eigen::VectorXd vectorToEigen(const std::vector<double> values){

		Eigen::VectorXd result(values.size());

		for(int i = 0; i < values.size(); i++){
			result(i, 0) = values[i];
		}

		return result;

	}

	double normOf(const Eigen::VectorXd input){

		return std::sqrt(input.transpose()*input);

	}


}