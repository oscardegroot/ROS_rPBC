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

void Formation::connectionsFromPositions(const std::vector<Eigen::Matrix<double, 3, 1>>& positions){
    
    // Set connections
    for(size_t i = 0; i < N; i++){

        std::vector<connection> connections_i;

        for (int j = 0; j < N; j++){

            // If agent i connects to j (and check for equal indices) //graph[i][j] &&  -> default fully connected now
            if(i != j){

                // Calculate the difference and divide by two for ST
                Eigen::Matrix<double, 3, 1> r_jsi = (positions[j] - positions[i])/2.0; // -> fixed for wave variables

                //connection
                connections_i.push_back({j, r_jsi});
            }
        }
        connections.push_back(connections_i);
    }

}


CircleFormation::CircleFormation(){
    
    double radius, phase;
    helpers::safelyRetrieve(nh, "/formation/radius", radius);
    helpers::safelyRetrieve(nh, "/formation/phase", phase, 0.0);

    std::vector<Eigen::Matrix<double, 3, 1>> positions(N);
    
    // Determine positions
    double current_phase = phase;
    for (size_t i = 0; i < N; i++){
        positions[i] = Eigen::Matrix<double, 3, 1>::Zero();
        positions[i] << std::cos(current_phase)*radius,
                        std::sin(current_phase)*radius,
                        0.0;

        current_phase += (2.0 * M_PI) / double(N);
    }
    
    connectionsFromPositions(positions);
    
    logMsg("Formation", "Circular formation initialised with radius " + std::to_string(radius)  + ".", 2);
}

Consensus::Consensus()
{
    std::vector<Eigen::Matrix<double, 3, 1>> positions(N);

    for(size_t i = 0; i < N; i++){
       positions[i] = Eigen::Matrix<double, 3, 1>::Zero();
    }
    
    connectionsFromPositions(positions);
    
    logMsg("Formation", "Consensus formation initialised.", 2);
}

LineFormation::LineFormation()
{
    double angle, spacing;
    helpers::safelyRetrieve(nh, "/formation/angle", angle, 0.0);
    helpers::safelyRetrieve(nh, "/formation/spacing", spacing);

    std::vector<Eigen::Matrix<double, 3, 1>> positions(N);

    double length = double(N)*spacing;
    
    // Determine positions
    for (int i = 0; i < N; i++){
        positions[i] = Eigen::Matrix<double, 3, 1>::Zero();
        positions[i] << std::cos(angle)*(-length/2.0 + double(i)*spacing),
                        std::sin(angle)*(-length/2.0 + double(i)*spacing),
                        0.0;
    }

    connectionsFromPositions(positions);

    logMsg("Goals", "Goal is set to line with spacing " + std::to_string(spacing)  + ".", 2);
}
