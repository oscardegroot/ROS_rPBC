#pragma once


#include "ros/ros.h"
#include <chrono>

#include <mutex>
#include <string>
#include <vector>
#include <memory>

#include "Helpers.h"
#include "CustomLog.h"


/**
 * @class Station
 * @author Oscar
 * @date 10/10/19
 * @file Station.h
 * @brief Base class for server side applications. Initialises using FSM based registration. Helps synchronisation of initial processes.
 */
class Station{

public:

    /**
     * @brief Station constructor only initiates a timer
     */
    Station(){
        
        // Retrieve the timer duration
        double register_time;
        helpers::safelyRetrieve(nh, "/register_time", register_time);
        
        // Initiate a timer for register duration
        timer = std::make_unique<helpers::SimpleTimer>(register_time);
    }

    /**
     * @brief Function called when registering is done. Function should call all initialisation functions.
     */
    virtual
    void starting() = 0;
    
    /**
     * @brief Update function called when RUNNING.
     */
    virtual
    void update() = 0;
    
    /**
     * @brief Function called after responses where received. Last initialisation before RUNNING.
     */
    virtual
    void enableStation(){};
    
    /**
     * @brief Function called when the program is shutting down
     */
    virtual
    void stopping() = 0;
    
    /**
     * @brief Check if the registering process is complete. In general this is time-out based.
     * @return If the timer has finished
     */
    virtual
    bool finishedRegistering() { return timer->finished();}

    /**
     * @brief Function to check if reponses where received. Assumed true by default.
     * @return true by default
     */
    virtual
    bool responsesReceived() const {return true;};



protected:

    ros::NodeHandle nh;
    
    // Timer
    std::unique_ptr<helpers::SimpleTimer> timer;
  
  
};