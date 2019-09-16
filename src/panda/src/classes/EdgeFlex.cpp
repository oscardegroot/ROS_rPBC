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

EdgeFlex::EdgeFlex(int i, int j, Eigen::MatrixXd gain_set, int l_set, Eigen::VectorXd r_star_set, int rate_mp_set)
	: Edge(i, j, gain_set, l_set, r_star_set, rate_mp_set){

    // Retrieve the NF exponential parameter
    helpers::safelyRetrieve(nh, "/controller/NF/alpha", alpha);

    // Retrieve goal parameters
	helpers::safelyRetrieve(nh, "/controller/NF/goal/eps", eps);
	helpers::safelyRetrieve(nh, "/controller/NF/goal/Rw", Rw);

	// Retrieve the lower bound on z
    helpers::safelyRetrieve(nh, "/panda/z_lower_bound", b_z, -10.0);

    /* When bound influences beacond, control goes wrong! */
    std::shared_ptr<Goal> goal_ = std::make_shared<WangGoal>(l);
    std::shared_ptr<Obstacle> z_bound_ = std::make_shared<BoundObstacle>(l, b_z, 1.15, 2);
    
    Eigen::VectorXd obstacle_location(l);
    obstacle_location << -0.2, 0.2, 0.7;
    std::shared_ptr<Obstacle> object_1_ = std::make_shared<ObjectObstacle>(l, obstacle_location, 0.2);
    potential = std::make_unique<NavigationFunction>(alpha, l);
    potential->addGoalFcn(goal_);
    //potential->addObstacleFcn(z_bound_); 
    potential->addObstacleFcn(object_1_);

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
	// Parameterisation here! Also save r_js of the last step?

	int k = 0;
	double cur_ggamma, cur_gamma, cur_gG, cur_G, denom;
	double diff = 10.0;
	double d;

	// For calculating the improvement over the loop
	Eigen::VectorXd r_js_prev(l);
	r_js_prev = r_js_last;

	// Construct a vector with only the z component
    Eigen::VectorXd z_i(l);
    z_i << 0.0, 0.0, 1.0;

	// Initiate with the final value of the last sampling period
	//r_js = Eigen::VectorXd::Zero(l);
	r_js = r_js_last;
	

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
	agent_i = 1;
	if(i_ID < j_ID){
		agent_i = -1;
	}

	// Apply the gain
	Eigen::MatrixXd Binv = B.inverse();


	// Define the ST matrix
	matrix_ST = Eigen::MatrixXd(2*l, 2*l);
	matrix_ST << agent_i*std::sqrt(2)*Binv, -Binv*Binv,
					-Eigen::MatrixXd::Identity(l, l), agent_i*std::sqrt(2)*Binv;

}



void EdgeFlex::lowpassFilter(Eigen::VectorXd& filtered_data, Eigen::VectorXd new_data, double alpha){

  	for (size_t i = 0; i < filtered_data.size(); i++) {
    	filtered_data[i] = (1 - alpha) * filtered_data[i] + alpha * new_data[i];
  	}
}


double EdgeFlex::gradient_gamma(double d){

	double g_gamma = 0.0;

	if(d >= 2*Rw){
        g_gamma = 0.0;
	}else if(d < r){

        g_gamma = 3*a1*d + 2*b1;
	}else{

        g_gamma = (3*a2*d*d+2*b2*d+c2) / d;
	}

	return g_gamma;


}

double EdgeFlex::gamma(double d){

    //double d = helpers::normOf(r_js - r_i);
    double v_gamma = 0.0;

    if(d >= 2*Rw){
        v_gamma = 1.0;
    }else if(d < r){

        v_gamma = a1*std::pow(d, 3) + b1*std::pow(d, 2);
    }else{
        v_gamma = a2*std::pow(d, 3) + b2*std::pow(d, 2) + c2*d + d2;
    }

    return v_gamma;
}

double EdgeFlex::gradient_G(Eigen::VectorXd r_i, Eigen::VectorXd r_js){

    // d is distance to the lower bound
//    double d = r_i(2, 0) - b_z;
//
//    if (d < 0){
//        throw OperationalException("d in G gradient became negative! (value="
//        + std::to_string(d));
//    }
//
//    double g_G = 0.0;
//
//    if(d >= R_z + delta_z || d < R_z){
//        g_G = 0.0;
//    }else{
//        g_G = (3*af*d*d + 2*bf*d+cf);//std::abs(r_i(2, 0));
//    }
    throw OperationalException("");
    return 0.0;
}

double EdgeFlex::G(Eigen::VectorXd r_i, Eigen::VectorXd r_js) {

    // d is distance to the lower bound
    double d = r_i(2, 0) - b_z;

    if (d < 0){
        throw OperationalException("d in G gradient became negative! (value="
        + std::to_string(d));
    }
    double v_G = 0.0;

    if (d >= R_z + delta_z){
        v_G = 1.0;
    }else if(d < R_z){
        v_G = 0.0;
    }else {
        v_G = af*std::pow(d, 3) + bf*std::pow(d, 2) + cf*d + df;
    }

    throw OperationalException("");
    return 0.0;
}


void EdgeFlex::initGamma(){
	r = 2*Rw*eps;

	a1 = (4*eps*Rw*Rw + 4*eps*Rw*r - 3*r*r) / (4*Rw*std::pow(r, 3)*(r-2*Rw));
	b1 = (3*r*r-12*eps*Rw*Rw)/(4*Rw*r*r*(r-2*Rw));
	a2 = (4*eps*Rw*r-3*r*r+8*Rw*r-12*eps*Rw*Rw)/(4*Rw*r*std::pow(r-2*Rw, 3));
	b2 = (-12*eps*Rw*Rw*r-24*Rw*Rw*r+3*std::pow(r, 3) + 48*eps*std::pow(Rw, 3))/(4*Rw*r*std::pow(r-2*Rw, 3));
	c2 = (9*Rw*r*r-12*eps*std::pow(Rw, 3) - 3*std::pow(r, 3))/(r*std::pow(r-2*Rw, 3));
	d2 = 1- (Rw*(-4*eps*Rw*Rw*r - 8*Rw*Rw*r + 12*Rw*r*r - 3*std::pow(r, 3)))/(r*std::pow(r - 2*Rw, 3));
}

void EdgeFlex::initG(){

    af = 1/std::pow(delta_z, 3);
    bf = -(3*(R_z + delta_z))/std::pow(delta_z, 3);
    cf = 3*std::pow(R_z + delta_z, 2)/std::pow(delta_z, 3);
    df = 1 - std::pow(R_z + delta_z, 3) / std::pow(delta_z, 3);
}