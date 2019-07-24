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

EdgeFlex::EdgeFlex(int i, int j, Eigen::MatrixXd gain_set, int l_set, bool is_integral)
	: Edge(i, j, gain_set, l_set, is_integral){

	helpers::safelyRetrieve(nh, "/controller/wang/eps", eps);
	helpers::safelyRetrieve(nh, "/controller/wang/Rw", Rw);

	initWang();


	// Set the scattering gain
	setScatteringGain(gain_set);

	// Initialise r_js at some non-zero point
	r_js_last = Eigen::VectorXd::Zero(l);
	r_js_last << 1, 1, 1;

	tau_last = Eigen::VectorXd::Zero(l_set);
	//r_js_filtered = Eigen::VectorXd::Zero(l_set);
}


Eigen::VectorXd EdgeFlex::sample(Eigen::VectorXd r_i){

	// ZOS
	applyReconstruction(s_received, r_i);

	// Set the input by iteration
	tau_last = iterateST(s_received, tau_last, r_i);

	// Calculate the input
	Eigen::VectorXd s_out = calculateWaves(s_received, tau_last);
	
	// Publish the returning wave
	publishWave(s_out);

	// Return the input for this edge
	return tau_last;
}


Eigen::VectorXd EdgeFlex::iterateST(Eigen::VectorXd &s_in, Eigen::VectorXd tau, Eigen::VectorXd r_i){

	// Parameterisation here! Also save r_js of the last step?

	Eigen::VectorXd r_js, tau_result;
	tau_result = tau;
	r_js = r_js_last;

	int k = 0;
	double gamma;
	double diff = 10.0;
	Eigen::VectorXd r_js_new = Eigen::VectorXd::Zero(l);


	/* Possibly efficiency by iterating 3 times always */
	while (diff > 0.1 && k < 100){

		gamma = gammaWang(r_i, r_js);

		for(int i = 0; i < l; i++){
			r_js_new(i, 0) = (matrix_ST(i, i)*s_in(i) - matrix_ST(i, l+i)*gamma*gain(i,i)*r_i(i))
				/(1 - matrix_ST(i, l+i)*gamma*gain(i,i));
		}

		k++;

		diff = helpers::normOf(r_js - r_js_new);

		r_js = r_js_new;
		r_js_new = Eigen::VectorXd::Zero(l);
	}
	
	if(k > 80){
		logMsg("EdgeFlex", "Warning: iteration count k > 80, the iteration may not converge!", WARNING);
	}

	tau_result = calculateControls(r_js, r_i);
	r_js_last = r_js;
	return tau_result;

}

double EdgeFlex::gammaWang(Eigen::VectorXd r_i, Eigen::VectorXd r_js){

	double d = helpers::normOf(r_js - r_i);
	double dgammadd = 0.0;	

	if(d >= 2*Rw){
		dgammadd = 0.0;
	}else if(d < r){

	 	dgammadd = 3*a1*d*d + 2*b1*d;
	}else{

		dgammadd = 3*a2*d*d+2*b2*d+c2;
	}

	return dgammadd;


}

void EdgeFlex::initWang(){
	r = 2*Rw*eps;

	a1 = (4*eps*Rw*Rw + 4*eps*Rw*r - 3*r*r) / (4*Rw*std::pow(r, 3)*(r-2*Rw));
	b1 = (3*r*r-12*eps*Rw*Rw)/(4*Rw*r*r*(r-2*Rw));
	a2 = (4*eps*Rw*r-3*r*r+8*Rw*r-12*eps*Rw*Rw)/(4*Rw*r*std::pow(r-2*Rw, 3));
	b2 = (-12*eps*Rw*Rw*r-24*Rw*Rw*r+3*std::pow(r, 3) + 48*eps*std::pow(Rw, 3))/(4*Rw*r*std::pow(r-2*Rw, 3));
	c2 = (9*Rw*r*r-12*eps*std::pow(Rw, 3) - 3*std::pow(r, 3))/(r*std::pow(r-2*Rw, 3));
}

void EdgeFlex::lowpassFilter(Eigen::VectorXd& filtered_data, Eigen::VectorXd new_data, double alpha){

  	for (size_t i = 0; i < filtered_data.size(); i++) {
    	filtered_data[i] = (1 - alpha) * filtered_data[i] + alpha * new_data[i];
  	}
}

/* Reconstruct data if necessary */
void EdgeFlex::applyReconstruction(Eigen::VectorXd& wave_reference, Eigen::VectorXd r_i){

	//Eigen::VectorXd tau = iterateST(wave_reference, tau_last);
	if(!data_received){
		wave_reference = Eigen::VectorXd::Zero(l);
		//logTmp("no data!");
	}else{
		data_received = false;
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

// Apply controls (this will vary)
Eigen::VectorXd EdgeFlex::calculateControls(Eigen::VectorXd r_js, Eigen::VectorXd r_i){

	return gammaWang(r_i, r_js)*gain*(r_js - r_i);
	//return 1.0*(r_js - r_i);
}

// Retrieve the new wave from the scattering transformation
Eigen::VectorXd EdgeFlex::calculateWaves(Eigen::VectorXd s_in, Eigen::VectorXd tau){

	return matrix_ST.block(l, 0, l, l) * s_in + matrix_ST.block(l, l, l, l) * tau;
}

// Retrieve the wave reference r_js from the scattering transformation
Eigen::VectorXd EdgeFlex::calculateWaveReference(Eigen::VectorXd s_in, Eigen::VectorXd tau){

	return matrix_ST.block(0, 0, l, l) * s_in + matrix_ST.block(0, l, l, l) * tau;
}


// Set the ST matrix
void EdgeFlex::setScatteringGain(Eigen::MatrixXd gain){
	// Define the gains on this edge and the impedance
	//Eigen::MatrixXd B(l, l);
 	//B = gain.cwiseSqrt();
	double b = sqrt(gain(0,0));

	// Take into account the different ST depending on which agent this is
	double agent_i = 1.0;
	if(i_ID < j_ID){
		agent_i = -1.0;
	}


	// Apply the gain
	//Eigen::MatrixXd Binv = B.inverse();

	// Define the ST matrix
	matrix_ST = Eigen::MatrixXd::Zero(2*l, 2*l);
	matrix_ST << agent_i*std::sqrt(2)*(1/std::sqrt(b))*Eigen::MatrixXd::Identity(l, l),
				-(1/b)*Eigen::MatrixXd::Identity(l, l),
				-Eigen::MatrixXd::Identity(l, l),
				agent_i*std::sqrt(2)*(1/std::sqrt(b))*Eigen::MatrixXd::Identity(l, l);
	logTmp(matrix_ST);
}





	// /*We need to apply WVM*/
	// if(!data_received)
	// {
	// 	// Calculate the wave when we apply HLS
	// 	Eigen::VectorXd s_HLS = calculateControls(s_buffer, r_i);

	// 	// If that's allowed, use it, otherwise reconstruct by amplitude
	// 	if(s_HLS.transpose()*s_HLS > s_buffer.transpose()*s_buffer)
	// 	{
	// 	 	logMsg("Edge", "WVM applied HLS", 5);
	// 		wave_reference = s_buffer;

	// 	}else{
	// 		logMsg("Edge", "WVM applied reconstruction", 5);
	// 		wave_reference = elementSign(s_buffer).cwiseProduct(s_HLS.cwiseAbs());
	// 	}
		
		
	// }else{
	// 	logMsg("Edge", "Used received data", 5);
	// 	data_received = false;
	// 	s_buffer = s_received;
	// }