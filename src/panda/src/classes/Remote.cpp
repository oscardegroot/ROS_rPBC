
#include "Remote.h"

Remote::Remote(int l_dim, int N_dim){

	l = l_dim;
	N = N_dim;
	goals = new Goals();
	goals->Init(N, l);

	// Start a server
	connect_server = n.advertiseService("get_connections_of", &Remote::retrieveConnectionsOf, this);
	std::cout << "Connection server initiated\n";

}

Remote::~Remote(){
	delete goals;
}

bool Remote::retrieveConnectionsOf(panda::getConnectionsOf::Request &req, panda::getConnectionsOf::Response &res){
	
	Eigen::VectorXd r(l);
	res.is_connected = goals->retrieveConnectionBetween(req.id_i, req.id_j, r);

	// Convert from vector to msg
	std_msgs::Float64MultiArray r_vec;

	r_vec.data.resize(l);
	for(int i = 0; i < l; i++){
		r_vec.data[i] = r(i);
	}

	res.r_star = r_vec;

	return true;
}