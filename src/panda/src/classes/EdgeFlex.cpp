/*
File: Edge.cpp

Implements communication on an edge on one side using ST and WVM.
Externally calling sampleEdge will sample the buffer and return a wave to the other side of the channel.

i: agent id
j: connected agent id
gain: gain of the connection 
l_set: dimension of the channel
*/

#include "EdgeFlex.h"

EdgeFlex::EdgeFlex(Agent& agent, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set)
	: Edge(agent, j, gain_set, l_set, r_star_set, rate_mp_set){

    // Retrieval is fully internal
    potential = std::make_unique<NavigationFunction>(agent, l);
    /* When bound influences beacond, control goes wrong! */
//    std::shared_ptr<Goal> goal_ = std::make_shared<WangGoal>(l);
//    std::shared_ptr<Obstacle> z_bound_ = std::make_shared<BoundObstacle>(l, b_z, 1.15, 2);
//    
//    Eigen::VectorXd obstacle_location(l);
//    obstacle_location << -0.2, 0.2, 0.7;
//    std::shared_ptr<Obstacle> object_1_ = std::make_shared<ObjectObstacle>(l, obstacle_location, 0.2);
//    potential = std::make_unique<NavigationFunction>(agent, l);
//    potential->addGoalFcn(goal_);
//    potential->addObstacleFcn(z_bound_); 
//    potential->addObstacleFcn(object_1_);

	// Set the scattering gain
	setScatteringGain(gain_set);

	// Initialise r_js at some non-zero point
	r_js_last = Eigen::VectorXd::Zero(l);
	r_js_last << 1, 1, 1;
}

/* Main public function that samples this edge */
Eigen::VectorXd EdgeFlex::sample(Eigen::VectorXd r_i){

	Eigen::VectorXd tau(l), r_js(l), s_out(l);
	bool hls_applied = false;

	if(!data_received){
        
		s_out = fullSTLoop(tau, r_js, r_i, s_buffer); // Gives tau, r_js, sij+

		if(s_out.transpose()*s_out > s_buffer.transpose()*s_buffer){
			
			hls_applied = true;
			// Use this and its values, no more calculations

		}else{

			// Reconstruct s received
			s_received = elementSign(s_buffer).cwiseProduct(s_out.cwiseAbs());
		}

	}else{
		// Otherwise make sure to update the buffer at this time
		s_buffer = s_received;
        data_received = false;
	}

	// If HLS was not applied we still need to calculate tau, r_js and s_out
	if(!hls_applied){
		s_out = fullSTLoop(tau, r_js, r_i, s_received);
	}

	// Save the correct values for the next sampling period
	r_js_last = r_js;

	// Lastly publish the waves and return the output
	publishWave(s_out);
	return tau;
}


// Calculates all variables in the ST loop with input s_in. Returns s_out
Eigen::VectorXd EdgeFlex::fullSTLoop(Eigen::VectorXd& tau, Eigen::VectorXd& r_js,
									Eigen::VectorXd r_i, Eigen::VectorXd s_in){

	// Determine r_js via iterations over the gradient
	// (Solves the algebraic loop of the ST)
	STIterations(r_js, r_i, s_in);

	// Calculate the new controls with r_js
	tau = calculateControls(r_i, r_js);

	// Determine the output waves
	return calculateWaves(tau, r_js);

}


void EdgeFlex::STIterations(Eigen::VectorXd& r_js, Eigen::VectorXd r_i, Eigen::VectorXd s_in){

	// For calculating the improvement over the loop
	Eigen::VectorXd r_js_prev(l);
	r_js_prev = r_js_last;

	// Construct a vector with only the z component
    Eigen::VectorXd z_i(l);
    z_i << 0.0, 0.0, 1.0;

	// Initiate with the final value of the last sampling period
	r_js = r_js_last;
	
    // Keep track of iteration count
	int k = 0;
	double diff = 10.0;
    
	/* Possibly efficiency by iterating 3 times always */
	while (diff > 0.1 && k < 100){

        // Calculate the gradient of the potential function
        PotentialFactors gradient = potential->gradient_factors(r_i, r_js);

		// Calculate r*[k] for all dimensions
		for(int i = 0; i < l; i++){

            // The RHS of the algebraic loop
            double factor_i = matrix_ST(i, i)*s_in(i) + matrix_ST(i, l+i)*gain(i, i) * gradient.i_matrix(i, i)*r_i(i);
            
            // The factor for r_js
            double factor_js = 1 - matrix_ST(i, l+i) * gain(i, i) * gradient.js_multiplier;
            
            // The next iteration step is the RHS divided by the factor before r_js
            r_js(i) = factor_i / factor_js;

		}

		// Calculate the improvement
		diff = helpers::normOf(r_js - r_js_prev);

		// Remember the last value of r_js
		r_js_prev = r_js;

		k++;
	}

	if(k > 80){
		logMsg("EdgeFlex", "Warning: iteration count k > 80, the iteration may not converge!", WARNING);
	}

}

// Apply the element wise sign operator
Eigen::VectorXd EdgeFlex::elementSign(Eigen::VectorXd s_in){

	Eigen::VectorXd s_out = Eigen::VectorXd::Zero(s_in.size());
	for(int i = 0; i < s_in.size(); i++){
		if(s_in[i] > 0){
			s_out[i] = 1;
		}else{
			s_out[i] = -1;
		}
	}
	return s_out;
}

// Make const ref
Eigen::VectorXd EdgeFlex::calculateControls(const Eigen::VectorXd& r_i, const Eigen::VectorXd& r_js){

    // Multiply the gradient with the gain
    return gain*potential->gradient(r_i, r_js);

}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeFlex::calculateWaves(const Eigen::VectorXd& tau, const Eigen::VectorXd& r_js){

	Eigen::VectorXd result = Eigen::VectorXd::Zero(l);

	for(int i = 0; i < l; i++){
		double b = std::sqrt(gain(i,i));
		result(i, 0) = agent_i/std::sqrt(2*b) * (tau(i, 0) -b*r_js(i, 0));
	}

	return result;
}

void EdgeFlex::setScatteringGain(Eigen::MatrixXd gain){
	// Define the gains on this edge and the impedance
	Eigen::MatrixXd B(l, l);
 	B = gain.cwiseSqrt();

	// Take into account the different ST depending on which agent this is
    agent_i = (i_ID < j_ID) ? -1 : 1;

	// Apply the gain
	Eigen::MatrixXd Binv = B.inverse();

	// Define the ST matrix
	matrix_ST = Eigen::MatrixXd(2*l, 2*l);
	matrix_ST << agent_i*std::sqrt(2)*Binv, -Binv*Binv,
					-Eigen::MatrixXd::Identity(l, l), agent_i*std::sqrt(2)*Binv;
}

//void EdgeFlex::lowpassFilter(Eigen::VectorXd& filtered_data, Eigen::VectorXd new_data, double alpha){
//
//  	for (size_t i = 0; i < filtered_data.size(); i++) {
//    	filtered_data[i] = (1 - alpha) * filtered_data[i] + alpha * new_data[i];
//  	}
//}