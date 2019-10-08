/*
File: Goals.cpp

Contains a number of functions describing formations and other cooperative goals
*/

#include "Goals.h"

// The construct takes the number of agents
Goals::Goals(){
    
    ros::NodeHandle nh;
    
    std::string formation_type;
    
    helpers::safelyRetrieve(nh, "/N_agents", N);
    helpers::safelyRetrieve(nh, "/l", l);
    helpers::safelyRetrieve(nh, "/formation/type", formation_type);
    
    formation = std::make_unique<Formation>(getFormation(formation_type));
    
	initLeaders(nh);
};

Formation Goals::getFormation(const std::string& formation_type){
    
    /** Create a formation based on the given type */
    if(formation_type == "circle"){
        return CircleFormation();
    }else if(formation_type == "line"){
        return LineFormation();
    }else{
        return Consensus();
    }
    
}

void Goals::initLeaders(ros::NodeHandle & nh){

    leaders = {};
    int id;
    Eigen::VectorXd gain, ref;

    try {
        helpers::safelyRetrieve(nh, "/leader/id", id);
        helpers::safelyRetrieveEigen(nh, "/leader/gain", gain, l);
        helpers::safelyRetrieveEigen(nh, "/leader/ref", ref, l);

        leaders.push_back({id, ref, gain});
    }
    catch(RetrievalException exc){
        return;
    }

}

bool Goals::isAgentLeader(u_int id, Eigen::VectorXd & ref, Eigen::VectorXd & gain){

    for(int i = 0; i < leaders.size(); i++){
        if(leaders[i].id == id){
            ref = leaders[i].ref;
            gain = leaders[i].gain;
            return true;
        }
    }

    return false;

}
