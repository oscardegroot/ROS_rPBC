/*
 * Agent.h
 * 
 * The agent class is the entity that holds relevant networking data of the agent such as its ID and a nodehandle with its private namespace
 * Parameter retrieval should be done through internal functions
*/

#include "ros/ros.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "panda/getConnectionsOf.h"
#include "panda/registerAgent.h"

#include "CustomLog.h"
#include "Helpers.h"

#include <vector>
#include <string>
#include <memory>
#include <sstream>

#pragma once

// The relevant information for parameter retrieval and connectivity
class Agent{
    
public:

    Agent(const Agent&) = delete;


    /** @brief Constructor for client side
     * Internally sets the node_name from the name of the node
     * 
     * @param type_name_set: type name of the agent (not used atm)
     * */
    Agent(std::string type_name_set);
    
    /** @brief Constructor for server side
     * @param ID_set: agent ID
     * @param sampling_rate_set: sampling rate of the agent
     * @param type_name_set: type name of the agent (not used atm)
     * */
    Agent(int ID_set, int sampling_rate_set, std::string type_name_set);
    ~Agent();
    
    void registerToServer();
    panda::registerAgent agentToSrv();
    
    /** @brief Wrappers for functions in Helpers.h */
    template <class T>
    void retrieveParameter(const std::string& name, T& param){
        helpers::safelyRetrieve((*nh_private), name, param);
    }
    
    template <class T>
    void retrieveParameter(const std::string& name, T& param, const T default_value){
        helpers::safelyRetrieve((*nh_private), name, param, default_value);
    }
    
    void retrieveEigen(const std::string& name, Eigen::VectorXd& param, const int expected_size){
        helpers::safelyRetrieveEigen((*nh_private), name, param, expected_size);
    }
    
    void retrieveEigen(const std::string& name, Eigen::VectorXd& param){
        helpers::safelyRetrieveEigen((*nh_private), name, param);
    }
    
    template <class T>
    bool retrieveArray(const std::string name, T& param, const int expected_size){
        helpers::safelyRetrieveArray((*nh_private), name, param, expected_size);
    }
    
    inline bool ifParameter(const std::string& name){
        return helpers::ifParameter((*nh_private), name);
    }

    
    inline bool hasParameter(const std::string& name){
        return helpers::hasParameter((*nh_private), name);
    }
    
    // Getters
    int getID() const {return ID;}
    int getSamplingRate() const {return sampling_rate;}
    const std::string& getType() const {return type_name;}
    
    ros::NodeHandle& getPrivateNh() const {return (*nh_private);}

private:
    int ID;
    int sampling_rate;
    std::unique_ptr<ros::NodeHandle> nh_private;

    std::string type_name; // not used for parameter retrieval
    std::string node_name; // will be set to the nodename

};


    
    // Implement printing
//    inline std::ostream& operator<<(std::ostream& str, Agent const& v) { 
//        str << "Agent ["<< v.node_name << "] ID: " << v.ID << ", type: " << v.type_name << std::endl;
//        return str;
//    }
//    inline string to_string(Agent const& v) const{
//        return "Agent [" + v.node_name + "] ID: " + to_string(v.ID) + ", type: " + v.type_name;
//    }

//    void print(){
//        std::cout << "Agent: " << name << " (" << type << ") - ID: " << ID << ", Rate: " << sampling_rate <<".\n";
//    }