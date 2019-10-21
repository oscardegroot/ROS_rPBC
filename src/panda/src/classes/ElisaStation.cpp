#include "ElisaStation.h"

// Start servers
ElisaStation::ElisaStation(){
        
    elisa3_register = nh.advertiseService("/registerElisa3", &ElisaStation::registerElisa3, this);
    elisa3_color = nh.advertiseService("/colorElisa3", &ElisaStation::colorElisa3, this);
}

/* Start communication and calibrate internal odometry */
void ElisaStation::starting()
{
    logMsg("Elisa Station", "Starting with " + std::to_string(elisa3_addresses.size()) +
    " Elisa3 robots", 2);

    
    int * addresses = &elisa3_addresses[0];

    startCommunication(addresses, elisa3_addresses.size());
    calibrateSensorsForAll();
    
    ros::Duration(0.2).sleep();
}

/* Read the sensors and send them to the robots */
void ElisaStation::update()
{
    /** @odometry
//    for(int i = 0; i < elisa3_addresses.size(); i++){
//        
//        int address = elisa3_addresses[i];
//        panda::Readout msg;
//
//        // Currently odometry via USB. When camera I probably dont need this in the station!
//        /* WITHOUT CAMERA
//         * msg.x = -getOdomYpos(address)/1000.0;  // mm to m
//        msg.y = getOdomXpos(address)/1000.0;  // mm to m
//        msg.theta = getOdomTheta(address) / 180.0 * M_PI + M_PI_2; // Theta expressed in a tenth of degree
//        readout_pubs[i].publish(msg);*/
//        
//        
//    }*/
}

/* Stop the robots and the communication */
void ElisaStation::stopping()
{
    logMsg("Elisa3 Station", "Stopping Elisa3 Movement.", 2);
    
    for(int i = 0; i < elisa3_addresses.size(); i++){
        setRightSpeed(elisa3_addresses[i], 0);
        setLeftSpeed(elisa3_addresses[i], 0);
        setColor(elisa3_addresses[i], 0, 0, 0);
        transferData();
    }

    stopCommunication();
    
    logMsg("Elisa3 Station", "Succesfully shut down.", 2);
}


void ElisaStation::moveElisa3(const panda::Move::ConstPtr& msg) {

    setRightSpeed(msg->address, msg->wheel_right);
    setLeftSpeed(msg->address, msg->wheel_left);
}

/* Register an Elisa3 robot. */
bool ElisaStation::registerElisa3(panda::registerElisa3::Request &req, panda::registerElisa3::Response &res){

    // Lock to prevent pushing back misordered entries
    std::mutex mtx;
    mtx.lock();
    
    // Save the address
    elisa3_addresses.push_back(req.address);

    // Create a subscriber for move commands
    move_subs.push_back(nh.subscribe<panda::Move>("elisa3_" + std::to_string(req.address) + "_move", 20, &ElisaStation::moveElisa3, this));
    
    // Create a publisher for sensor data
    readout_pubs.push_back(nh.advertise<panda::Readout>("elisa3_" + std::to_string(req.address) + "_readout", 20));
    
    vision_subs.push_back(nh.subscribe<panda::Readout>("Camera/Elisa3/" + std::to_string(req.address), 20, &ElisaStation::visionCallback, this));

    mtx.unlock();
    
    logMsg("Elisa Station", "Succesfully added a new Elisa3 with address " + std::to_string(req.address) + "!", 2);

    return true;
}


// Set color of an Elisa3
bool ElisaStation::colorElisa3(panda::colorElisa3::Request &req, panda::colorElisa3::Response &res){
    
    // If color type is 0 then the color is specified fully, otherwise it is some defined color
    if(req.color_type == 0){
        setColor(req.address, req.red, req.green, req.blue);
    }else{
        switch(req.color_type){
            case COLOR_RED:
                setColor(req.address, 80, 0, 0);
                break;
            case COLOR_GREEN:
                setColor(req.address, 0, 80, 0);
                break;
            case COLOR_BLUE:
                setColor(req.address, 0, 0, 80);
                break;
            case COLOR_ORANGE:
                setColor(req.address, 60, 60, 0);
                break;
            case COLOR_CYAN:
                setColor(req.address, 0, 60, 60);
                break;
            case COLOR_PINK:
                setColor(req.address, 60, 0, 60);
                break;
        }
    }

    return true;
}

// Set r,g,b color
void ElisaStation::setColor(int address, int r, int g, int b){

    setRed(address, r);
    setGreen(address, g);
    setBlue(address, b);
}

void ElisaStation::visionCallback(const panda::Readout::ConstPtr& msg)
{
    for(size_t i = 0; i < readout_pubs.size(); i++){

        if(readout_pubs[i].getTopic() == "/elisa3_" + std::to_string(msg->address) + "_readout"){

            panda::Readout new_msg;
            
            new_msg.x = msg->x/1000.0;
            new_msg.y = msg->y/1000.0;
            new_msg.theta = msg->theta;
            new_msg.address = msg->address;
            readout_pubs[i].publish(new_msg);
        }
    }


}
