#include "ElisaStation.h"

// Start servers
ElisaStation::ElisaStation(){
    PROFILE_SCOPE("ElisaStation init");

    elisa3_register = nh.advertiseService("/registerElisa3", &ElisaStation::registerElisa3, this);
    elisa3_color = nh.advertiseService("/colorElisa3", &ElisaStation::colorElisa3, this);
    vision_sub = nh.subscribe<panda::Readout>("Camera/Elisa3", 20, &ElisaStation::visionCallback, this);

}

/* Start communication and calibrate internal odometry */
void ElisaStation::starting()
{
    PROFILE_FUNCTION();

    logMsg("Elisa Station", "Starting with " + std::to_string(elisa3_addresses.size()) +
    " Elisa3 robots", 2);

    int * addresses = &elisa3_addresses[0];
    startCommunication(addresses, elisa3_addresses.size());
    calibrateSensorsForAll();
    

}

/* Read the sensors and send them to the robots */
void ElisaStation::update()
{
    PROFILE_FUNCTION();
    /** @odometry */
    for(int i = 0; i < elisa3_addresses.size(); i++){
        
        elisa3_entries[i].updateOdometry();
        
        elisa3_entries[i].publishCurrentCoordinates();
    }
}

/* Stop the robots and the communication */
void ElisaStation::stopping()
{
    PROFILE_FUNCTION();

    logMsg("Elisa3 Station", "Stopping Elisa3 Movement.", 2);
    
    for(int i = 0; i < elisa3_addresses.size(); i++){
        setRightSpeed(elisa3_addresses[i], 0);
        setLeftSpeed(elisa3_addresses[i], 0);
        setColor(elisa3_addresses[i], 0, 0, 0, 0.0);
        transferData();
    }

    stopCommunication();
    
    logMsg("Elisa3 Station", "Succesfully shut down.", 2);
}




/* Register an Elisa3 robot. */
bool ElisaStation::registerElisa3(panda::registerElisa3::Request &req, panda::registerElisa3::Response &res){

    PROFILE_FUNCTION();

    elisa3_entries.emplace_back(req.address);
    elisa3_addresses.push_back(req.address);
    
    logMsg("Elisa Station", "Succesfully added a new Elisa3 with address " + std::to_string(req.address) + "!", 2);

    return true;
}


// Set color of an Elisa3
bool ElisaStation::colorElisa3(panda::colorElisa3::Request &req, panda::colorElisa3::Response &res){
    
    // If color type is 0 then the color is specified fully, otherwise it is some defined color
    if(req.color_type == 0){
        setColor(req.address, req.red, req.green, req.blue, req.intensity);
    }else{
        switch(req.color_type){
            case COLOR_RED:
                setColor(req.address, 80, 0, 0, req.intensity);
                break;
            case COLOR_GREEN:
                setColor(req.address, 0, 80, 0, req.intensity);
                break;
            case COLOR_BLUE:
                setColor(req.address, 0, 0, 80, req.intensity);
                break;
            case COLOR_ORANGE:
                setColor(req.address, 60, 60, 0, req.intensity);
                break;
            case COLOR_CYAN:
                setColor(req.address, 0, 60, 60, req.intensity);
                break;
            case COLOR_PINK:
                setColor(req.address, 60, 0, 60, req.intensity);
                break;
        }
    }

    return true;
}

// Set r,g,b color
void ElisaStation::setColor(int address, int r, int g, int b, double intensity){

    setRed(address, (int)(r*intensity));
    setGreen(address, (int)(g*intensity));
    setBlue(address, (int)(b*intensity));
}

/**
 * @brief Callback when vision data arrives. Find the corresponding ID and
 * @param msg
 */
void ElisaStation::visionCallback(const panda::Readout::ConstPtr& msg)
{
    PROFILE_FUNCTION();
    bool marker_found = false;
    
    // Figure out what the address of the Elisa is based on the marker ID
    for(size_t i = 0; i < elisa3_entries.size(); i++){
        if(elisa3_entries[i].marker_id == msg->address){
            elisa3_entries[i].updateVision(msg);
            return;
        }
    }
    
    // Otherwise is it a detected obstacle (so that we do not need to do anything...)
    for(size_t i = 0; i < obstacle_marker_ids.size(); i++){
        if(obstacle_marker_ids[i] == msg->address){
            return;
        }
    }
    
    // If it is not than we need to add obstacles for all agents
    // We dont know the agents size here
    unsigned int i = 0;
    ros::ServiceClient obstacle_client;
    panda::addObstacle srv;
    
    do{
        obstacle_client = nh.serviceClient<panda::addObstacle>("/agent" + std::to_string(i) + "/addObstacle");
        
        srv.request.x = msg->x;
        srv.request.y = msg->y;
        srv.request.z = 0.0;
        srv.request.radius = 0.14;
        i++;
        
    }while(obstacle_client.call(srv));
    
    // Make sure we dont add it more than once.
    obstacle_marker_ids.push_back(msg->address);
    
}