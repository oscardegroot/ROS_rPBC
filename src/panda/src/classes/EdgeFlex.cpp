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

    // Multiply the formation distance by lambda if rPBC is used
    std::string output;
    helpers::safelyRetrieve(nh, "/output", output);
    
    if(output == "r"){
        double lambda;
        helpers::safelyRetrieve(nh, "/lambda", lambda);
        
        r_star *= lambda;
    }
    
    // Retrieval is fully internal, Formations nog included atm
    potential = std::make_unique<NavigationFunction>(agent, l, r_star);
    
    // Counter that handles sampling rate converting (trigger initially)
    retrieval_counter = std::make_unique<helpers::Counter>(rate_mp, false);

	// Set the scattering gain
	setScatteringGain(gain_set);

	// Initialise r_js at some non-zero point
	r_js_last = Eigen::VectorXd::Zero(l);
	r_js_last << 1, 1, 1;
}

/* Main public function that samples this edge */
Eigen::VectorXd EdgeFlex::sample(const Eigen::VectorXd& r_i){

	Eigen::VectorXd tau(l), r_js(l), s_out(l);
	bool hls_applied = false;

    /* Modified for sampling mismatch */
    if(retrieval_counter->trigger()){
        
        /* Reconstruct */
        if(!data_received){
            
            /* This could be applyWvm();*/
            s_out = fullSTLoop(tau, r_js, r_i, s_wvm_buffer); // Gives tau, r_js, sij+

            if(s_out.transpose()*s_out > s_wvm_buffer.transpose()*s_wvm_buffer){
                
                hls_applied = true;
                s_sample = s_wvm_buffer;
                // Use this and its values, no more calculations (not anymore for sampling rate stuff...)

            }else{

                // Reconstruct s received
                s_sample = elementSign(s_wvm_buffer).cwiseProduct(s_out.cwiseAbs());
                
            }

        /* Buffer data*/
        }else{
            // Otherwise make sure to update the buffer at this time
            s_wvm_buffer = s_received;
            
            // And use received data as sample
            s_sample = s_received;
            data_received = false;
            
        }
        
        /* Expand the incoming wave */
        //logTmp(s_sample);
        // Account for sampling rates
        //expandWaves(s_sample);
        //logTmp("after", s_sample);
    }

	// If HLS was not applied we still need to calculate tau, r_js and s_out
	//if(!hls_applied){
        // Calculate all values
		s_out = fullSTLoop(tau, r_js, r_i, s_sample);
	//}

	// Save the correct values for the next sampling period
	r_js_last = r_js;

	// Lastly publish the waves and return the output
	publishWave(s_out);
    
	return tau;
}




// Calculates all variables in the ST loop with input s_in. Returns s_out
Eigen::VectorXd EdgeFlex::fullSTLoop(Eigen::VectorXd& tau, Eigen::VectorXd& r_js,
									const Eigen::VectorXd& r_i, const Eigen::VectorXd& s_in){

	// Determine r_js via iterations over the gradient
	// (Solves the algebraic loop of the ST)
	STIterations(r_js, r_i, s_in);

	// Calculate the new controls with r_js
	tau = calculateControls(r_i, r_js);

	// Determine the output waves
	return calculateWaves(tau, r_js);
}


void EdgeFlex::STIterations(Eigen::VectorXd& r_js, const Eigen::VectorXd& r_i, const Eigen::VectorXd& s_in){

	// For calculating the improvement over the loop
	Eigen::VectorXd r_js_prev(l);
	r_js_prev = r_js_last;

	// Initiate with the final value of the last sampling period
	r_js = r_js_last;
	
    // Keep track of iteration count
	int k = 0;
	double diff = 10.0;
    
    //PotentialFactors gradient(Eigen::VectorXd::Zero(l), 0.0);
    
	/* Possibly efficiency by iterating 3 times always */
	while (diff > 0.00001 && k < 100){

        // Calculate the gradient of the potential function
        PotentialFactors gradient = potential->gradient_factors(r_i, r_js);
        
		// Calculate r*[k] for all dimensions
		for(int i = 0; i < l; i++){
            double b = std::sqrt(gain(i,i));

            // Calculate RHS and LHS of the gradient iteration
            /*double RHS = matrix_ST(i, i)*s_in(i) - matrix_ST(i, l+i) * gain(i, i) * gradient.RHS_vector(i);
            double LHS = 1.0 + matrix_ST(i, l+i) * gain(i, i) * gradient.LHS_multiplier;*/
            
            // - for gradient, - from scattering = +
            double RHS = agent_i*std::sqrt(2.0/b)*s_in(i) + 1.0/b * gain(i, i) * gradient.RHS_vector(i);
            double LHS = 1.0 - 1.0/b * gain(i, i) * gradient.LHS_multiplier;
            
            
            // Derive the next iteration of r_js
            r_js(i) = RHS / LHS;
		}

		// Calculate the improvement
		diff = helpers::normOf(r_js - r_js_prev);

		// Remember the last value of r_js
		r_js_prev = r_js;
		k++;
	}

	if(k > 80){
        // Safety first ....
        r_js = r_i;
		logMsg("EdgeFlex", "Warning: iteration count k > 80, the iteration may not converge!", WARNING);
        PotentialFactors gradient = potential->gradient_factors(r_i, r_js);
        gradient.print();
	}

}

// Apply the element wise sign operator
Eigen::VectorXd EdgeFlex::elementSign(const Eigen::VectorXd& s_in){

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
    return -gain*potential->gradient(r_i, r_js);

}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeFlex::calculateWaves(const Eigen::VectorXd& tau, const Eigen::VectorXd& r_js){

	Eigen::VectorXd result = Eigen::VectorXd::Zero(l);

    // This is not so efficient or smoart
	for(int i = 0; i < l; i++){
		double b = std::sqrt(gain(i,i));
		result(i, 0) = agent_i/std::sqrt(2*b) * (tau(i, 0) -b*r_js(i, 0));
	}
//    result = matrix_ST.block(l, 0, l, l) * tau + matrix_ST.block(l, l, l, l)
//    matrix_ST(l+i, i)
//            double RHS = matrix_ST(i, i)*s_in(i) - matrix_ST(i, l+i) * gain(i, i) * gradient.RHS_vector(i);

	return result;
}

void EdgeFlex::setScatteringGain(const Eigen::MatrixXd& gain){
	// Define the gains on this edge and the impedance
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(l, l);
    for(int i = 0; i < l; i++){
        B(i, i) = std::sqrt(gain(i, i));
    }

	// Take into account the different ST depending on which agent this is
    agent_i = (i_ID < j_ID) ? -1 : 1;

	// Apply the gain
	Eigen::MatrixXd Binv = B.inverse();

	// Define the ST matrix
	matrix_ST = Eigen::MatrixXd(2*l, 2*l);
	matrix_ST << agent_i*std::sqrt(2)*Binv, -Binv*Binv,
					-Eigen::MatrixXd::Identity(l, l), agent_i*std::sqrt(2)*Binv;
}

void EdgeFlex::addObstacle(const std::shared_ptr<Obstacle>& obstacle)
{
    potential->addObstacle(obstacle);
}
