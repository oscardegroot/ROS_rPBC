#pragma once

#define COLOR_BLUE 1
#define COLOR_GREEN 2
#define COLOR_RED 3
#define COLOR_ORANGE 4
#define COLOR_CYAN 5
#define COLOR_PINK 6

#include "Station.h"
#include "elisa3-lib.h"
#include "panda/registerElisa3.h"
#include "panda/Move.h"
#include "panda/Readout.h"
#include "panda/colorElisa3.h"

/**
 * @class ElisaStation
 * @author Oscar
 * @date 10/10/19
 * @file ElisaStation.h
 * @brief Station for central communication from the base PC USB Communication to connected Elisa3 robots. All Elisa3 robots should register if their
 * requests need to be forwarded.
 */
class ElisaStation : public Station{
  
/* See base class */
public:

    ElisaStation();

    void starting() override;
    void update() override;
    void stopping() override;

private:
    
    /**
     * @brief Callback for registering Elisa3 robots
     */
    bool registerElisa3(panda::registerElisa3::Request &req, panda::registerElisa3::Response &res);

    /**
     * @brief Callback from vision module
     */
    void visionCallback(const panda::Readout::ConstPtr& msg);


    /**
     * @brief Callback for setting the color of Elisa3 robots.
     */
    bool colorElisa3(panda::colorElisa3::Request &req, panda::colorElisa3::Response &res);

    /**
     * @brief Callback for setting the speed setpoint of the Elisa3.
     */
    void moveElisa3(const panda::Move::ConstPtr& msg);
    
    /**
     * @brief Wrapper for setting the color of an Elisa3
     * @param address Robot address
     * @param r, g, b Color intensity (0-100)
     */
    void setColor(int address, int r, int g, int b);

    // All registered addresses
    std::vector<int> elisa3_addresses;
    
    // Servers
    ros::ServiceServer elisa3_register, elisa3_color;
    
    // Subscriber to move messages
    std::vector<ros::Publisher> readout_pubs;
    std::vector<ros::Subscriber> move_subs;
    
    std::vector<ros::Subscriber> vision_subs;
};
