/*
File: CMM.cpp

Manages cooperative communication by managing a number of connections
*/

#include "CMM.h"


CMM::CMM(std::string agent_type){//int set_id, int set_sampling_rate){

	logMsg("CMM", "Initiating..", 2);
    
    status = STARTED;
    
    // Retrieve parameters
	helpers::safelyRetrieve(nh, "/l", l);
	helpers::safelyRetrieve(nh, "/N_agents", N);

    int network_rate;
    helpers::safelyRetrieve(nh, "sampling_rate", network_rate);
    
	// Connect to the remote
	connect_client = nh.serviceClient<panda::getConnectionsOf>("/getConnectionsOf");
    leader_client = nh.serviceClient<panda::isAgentLeader>("/isAgentLeader");

    /* Create an agent */
    agent = std::make_unique<Agent>(agent_type);
    
    // Make selectors to determine the dimensions of leader and cooperative agents
    coop_selector = std::make_unique<Selector>(*agent, l, std::string("z_select"), std::vector<int>{1, 3});
    leader_selector = std::make_unique<Selector>(*agent, l, std::string("z_select"), std::vector<int>{2, 3});

    // Retrieve the network gain
    Eigen::VectorXd gain_e;
	helpers::safelyRetrieveEigen(nh, "/network_gain", gain_e, l);
    
    // Apply the gain only on selected cooperative coordinates
    gain_e = coop_selector->select(gain_e);
	gain = Eigen::MatrixXd(gain_e.asDiagonal());
    
    init_server = nh.advertiseService("/agent" + std::to_string(agent->getID()) + "/initEdges", &CMM::initEdges, this);
    obstacle_server = nh.advertiseService("/agent" + std::to_string(agent->getID()) + "/addObstacle", &CMM::addObstacle, this);

    // Check if parameters are correct
    if(agent->getSamplingRate() % network_rate != 0){
        throw ParameterException("Sampling rate of agent " + std::to_string(agent->getID()) + " is not a multiple of the network rate!");
    }
    
    // Calculate the rate multiplier
    rate_mp = agent->getSamplingRate() / network_rate;

	// Retrieve connections and create communication edges
    status = WAITING_FOR_OTHER;
    
	logMsg("CMM", "Done!", 2);
}

CMM::~CMM(){};

// Only initialise when all systems have registered
void CMM::performHandshake(){
    
	ros::Rate loop_rate(100);
    while(ros::ok() && status != RUNNING){
        
        switch(status){
            // If registration is not complete, wait for the message from the server
            case(WAITING_FOR_OTHER):
                ros::spinOnce();
                break;
                
            // If the message was received, initialise!
            case INIT_CMM: 
                logMsg("CMM", "Initiating network call from Remote was received, processing now...", 2);
                retrieveConnections();
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

/** @error Memory corruption here?*/
bool CMM::retrieveConnections(){
    
    // Retrieve leader info from the server and create edges if necessary
    setupLeader();
    
    // Retrieve connections to other agents from the server and create edges if necessary
	setupEdges();
    
    // Transition to the running state
    status = RUNNING;
    
    auto ready_client = nh.serviceClient<std_srvs::Empty>("/cmmReady");

    std_srvs::Empty srv;    
    auto ready_call = [&](){
        return ready_client.call(srv);
        };
    
    helpers::repeatedAttempts(ready_call, 1.0, "Failed to get acknowledge from Agent Station!");
        
    return true;
}

void CMM::setupLeader(){
    
    panda::isAgentLeader srv;
    srv.request.id = agent->getID();
    
    // Call for leader information
    if(leader_client.call(srv)){
        if(srv.response.is_leader){
            
            // Parse the retrieved vectors
            Eigen::VectorXd temp_gain = helpers::messageToEigen(srv.response.gain, l);
            Eigen::VectorXd temp_ref = helpers::messageToEigen(srv.response.ref, l);

            temp_gain = leader_selector->select(temp_gain); // Deze select de verkeerde?
            Eigen::MatrixXd leader_gain = Eigen::MatrixXd(temp_gain.asDiagonal());

            // Create an edge
            edges.push_back(std::make_unique<EdgeLeader>(*agent, -1, leader_gain,
                            leader_selector->dim(), leader_selector->select(temp_ref)));
            
            logMsg("CMM", "Leader edge created.", 2);

        }
    }
}

void CMM::setupEdges(){
    
    // For all possible other agents
	for(int j = 0; j < N; j++){
		if(j != agent->getID()){
			
			// Ask the remote if we are connected
			panda::getConnectionsOf srv;
			srv.request.id_i = agent->getID();
			srv.request.id_j = j;

			if(connect_client.call(srv)){

				// If we are
				if(srv.response.is_connected){
                    
                    // Retrieve the formation data
                    Eigen::VectorXd r_star = helpers::vectorToEigen(srv.response.r_star.data);

					// Create a communication edge (select actually connected coordinates)
					edges.push_back(std::make_unique<EdgeFlex>(*agent, j, gain, coop_selector->dim(),
                            coop_selector->select(r_star), rate_mp));
                    //edges.push_back(std::make_unique<EdgeFlex>(*agent, j, gain, l, r_star, rate_mp));

                    logMsg("CMM", "Edge created.", 2);
					
				}
			}else{
				logMsg("CMM", "Failed to obtain formations from the server!", 0);
			}
		}
	}
}

bool CMM::initEdges(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    // Proceed to initialisation
    status = INIT_CMM;
    
    return true;
}

Eigen::VectorXd CMM::sample(const Eigen::VectorXd& r){

	// Initialise the combined input
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(l);

	// Sample all edges
	for(int i = 0; i < edges.size(); i++){
        
        // Note that the inputs are sized based on the size of the edge, so we need to convert the input to the full cooperative size l
        if(edges[i]->isLeader()){
            tau += leader_selector->deselect(edges[i]->sample(leader_selector->select(r)));
        }else{
            tau += coop_selector->deselect(edges[i]->sample(coop_selector->select(r)));
        }
    }
    
	// Return the combined input
	return tau;
}

bool CMM::hasInitialised() const
{
    return initialised;
}

bool CMM::addObstacle(panda::addObstacle::Request& req, panda::addObstacle::Response& res)
{
    logMsg("CMM", "Received addObstacle request, creating obstacle now...", 2);

    Eigen::VectorXd location(coopDim());
    
    if(coopDim() == 2){
        location << req.x, req.y;
    }else if(coopDim() == 3){
        location << req.x, req.y, req.z;
    }else{
        logMsg("CMM", "Obstacle added with more than 3 dimensions, which is not supported! No obstacle was instantiated.", 1);
        return true;
    }
    
    // Convert to m
    location = location / 1000.0;
    
    std::shared_ptr<Obstacle> obstacle = std::make_shared<ObjectObstacle>(*agent, location, req.radius, coopDim());
    
    for(size_t i = 0; i < edges.size(); i++){
        if(!edges[i]->isLeader()){
            edges[i]->addObstacle(obstacle);
        }
    }
    
    logMsg("CMM", "Done!", 2);
    
    return true;

}
