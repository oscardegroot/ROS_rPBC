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
#include "panda/addObstacle.h"

    
struct ElisaEntry{
    
    // Needs to be saved to set the calibration
    Eigen::Vector3d odometry;
    
    // Last calibration point for the odometry (when the last vision callback arrived)
    Eigen::Vector3d my_odometry_calibration;
    
    // Vision data
    Eigen::Vector3d vision;
    
    // Both are unique for every Elisa
    int marker_id;
    int address;
    bool init_vision_received;
    
    int counter = 0;
    ros::Publisher coordinate_pub;
    ros::Subscriber move_sub;
    helpers::Stopwatch stopwatch;
    
    ElisaEntry() = delete;
    
    ElisaEntry(int set_address)
        : address(set_address)
    {
        ros::NodeHandle nh;
        
        init_vision_received = false;
        // Initialise measurements
        odometry = Eigen::Vector3d::Zero();
        vision = Eigen::Vector3d::Zero();
        
        // Find the marker ID
        findMarkerID(nh, address);
        
        // Create a subscriber for move commands
        move_sub = nh.subscribe<panda::Move>("elisa3_" + std::to_string(address) + "_move", 20, &ElisaEntry::moveElisa3, this);
    
        // Create a publisher for sensor data
        coordinate_pub = nh.advertise<panda::Readout>("elisa3_" + std::to_string(address) + "_readout", 20);
    }
    
    ~ElisaEntry(){
        logTmp("Vision calls: ", counter);
    }
    
    void findMarkerID(ros::NodeHandle& nh, int address){

        int i = 0;
        while(nh.hasParam("elisa3_" + std::to_string(i) + "/address")){
        
            int found_address;
            helpers::safelyRetrieve(nh, "elisa3_" + std::to_string(i) + "/address", found_address);

            // If it is this elisa, retrieve the marker_id
            if(found_address == address){
                helpers::safelyRetrieve(nh, "elisa3_" + std::to_string(i) + "/marker_id", marker_id);
                break;
            }

            i++;
        }
    }
    
    void moveElisa3(const panda::Move::ConstPtr& msg) {

        setRightSpeed(msg->address, msg->wheel_right);
        setLeftSpeed(msg->address, msg->wheel_left);
    }
    
    /** Update vision data */
    void updateVision(const panda::Readout::ConstPtr& msg){
        counter++;
        vision = Eigen::Vector3d::Zero();
        vision(0) = msg->x / 1000.0;
        vision(1) = msg->y / 1000.0;
        vision(2) = msg->theta;
        
        my_odometry_calibration = odometry;
        
        if(!init_vision_received){
            init_vision_received = true;
            stopwatch.start();
        }else{
            double frequency = 1.0/stopwatch.click();
            if(frequency < 10){
                logMsg("Elisa Station", "Vision for Elisa " + std::to_string(address) +
                       " is running too slow (f = " + std::to_string(frequency) + ").", 1);
            }
        }
        
        //logTmp("Vision!", counter);
    }
    
    void updateOdometry(){
        odometry = Eigen::Vector3d::Zero();
        odometry(0) = -getOdomYpos(address) / 1000.0;  // mm to m
        odometry(1) = getOdomXpos(address) / 1000.0;  // mm to m
        
        // Possibly this angle calculation is not correct with vision??
        odometry(2) = getOdomTheta(address) / 180.0 * M_PI + M_PI_2;
        //logTmp(odometry(2));
    }
    
    void publishCurrentCoordinates(){
                
        // If we did not receive the initial state (via vision), do nothing
        if(!init_vision_received){
            
            return;
        }
        
        // Set the position equal to the vision coordinates
        Eigen::Vector3d current_pos = vision;

        // Add odometry in between
        current_pos += odometry - my_odometry_calibration;
        
        // Create and publish a message with the current position
        panda::Readout msg;
        msg.x = current_pos(0);
        msg.y = current_pos(1);
        msg.theta = current_pos(2);
        msg.address = address;
        
        coordinate_pub.publish(msg);
        
    }
    
};


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
    //void moveElisa3(const panda::Move::ConstPtr& msg);
    
    /**
     * @brief Wrapper for setting the color of an Elisa3
     * @param address Robot address
     * @param r, g, b Color intensity (0-100)
     */
    void setColor(int address, int r, int g, int b, double intensity);

    //void mapMarkers();

    // All registered addresses
    std::vector<int> elisa3_addresses;
    
    std::vector<ElisaEntry> elisa3_entries;
    
    // For dynamic obstacles (Position now only detected at startup)
    std::vector<int> obstacle_marker_ids;
    
    // Last odometry points for all elisas
    //std::vector<Eigen::VectorXd> odometry_calibration_points;
    
    // Servers
    ros::ServiceServer elisa3_register, elisa3_color;
    ros::Subscriber vision_sub;

    
};
