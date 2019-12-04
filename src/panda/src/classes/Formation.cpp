#include "Formation.h"


Formation::Formation()
{    
    helpers::safelyRetrieve(nh, "/N_agents", N);
    helpers::safelyRetrieve(nh, "/l", l);
}

bool Formation::retrieveConnectionBetween(u_int id_i, u_int id_j, Eigen::VectorXd& r_star)
{
    auto connects_i = connections[id_i];
    
    for(size_t j = 0; j < N; j++){

		if(connects_i[j].id == id_j){
			r_star = connects_i[j].r_star;
			return true;
		}
	}
    
    return false;
}

void Formation::connectionsFromPositions(const std::vector<Eigen::VectorXd>& positions){
    
    // For each agent
    for(size_t i = 0; i < N; i++){

        std::vector<connection> connections_i;

        // For each other agent
        for (int j = 0; j < N; j++){

            // If agent i connects to j (and check for equal indices) //graph[i][j] &&  -> default fully connected now
            if(i != j){

                // Calculate the difference and divide by two for ST
                Eigen::VectorXd r_jsi = (positions[j] - positions[i])/2.0; // -> fixed for wave variables

                //connection
                connections_i.push_back({j, r_jsi});
            }
        }
        connections.push_back(connections_i);
    }

}

// Circular formation with radius and phase
CircleFormation::CircleFormation(){
    
    if(l < 2){
        throw OperationalException("Circle formation cant be used for the given coordinate count");
    }
    
    double radius, phase;
    helpers::safelyRetrieve(nh, "/formation/radius", radius);
    helpers::safelyRetrieve(nh, "/formation/phase", phase, 0.0);

    std::vector<Eigen::VectorXd> positions(N);
    
    // Determine positions
    double current_phase = phase;
    for (size_t i = 0; i < N; i++){
        positions[i] = Eigen::VectorXd::Zero(l);
        positions[i](0) = std::cos(current_phase)*radius;
        positions[i](1) = std::sin(current_phase)*radius;

        current_phase += (2.0 * M_PI) / double(N);
    }
    
    connectionsFromPositions(positions);
    
    logMsg("Formation", "Circular formation initialised with radius " + std::to_string(radius)  + ".", 2);
}

// Consensus
Consensus::Consensus()
{
    std::vector<Eigen::VectorXd> positions(N);

    for(size_t i = 0; i < N; i++){
       positions[i] = Eigen::VectorXd::Zero(l);
    }
    
    connectionsFromPositions(positions);
    
    logMsg("Formation", "Consensus formation initialised.", 2);
}

// Formation in line with the given spacing and relative angle
LineFormation::LineFormation()
{
    double angle, spacing;
    helpers::safelyRetrieve(nh, "/formation/angle", angle, 0.0);
    helpers::safelyRetrieve(nh, "/formation/spacing", spacing);

    std::vector<Eigen::VectorXd> positions(N);

    double length = double(N)*spacing;
    
    // Determine positions
    for (int i = 0; i < N; i++){
        positions[i] = Eigen::VectorXd::Zero(l);
        positions[i](0) = std::cos(angle)*(-length/2.0 + double(i)*spacing);
        positions[i](1) = std::sin(angle)*(-length/2.0 + double(i)*spacing);
    }

    connectionsFromPositions(positions);

    logMsg("Goals", "Goal is set to line with spacing " + std::to_string(spacing)  + ".", 2);
}

// Formation from specified array
ManualFormation::ManualFormation(){
    
    std::vector<Eigen::VectorXd> positions(N);
    
    for(size_t i = 0; i < N; i++){
        
        Eigen::VectorXd cur_position;
        helpers::safelyRetrieveEigen(nh, "/formation/position_" + std::to_string(i), cur_position, l);
        positions[i] = cur_position;
    }
    
    connectionsFromPositions(positions);

    logMsg("Goals", "Goal is set to manual formation.", 2);
    
}