//
// Created by omdegroot on 12-08-19.
//

#include "Elisa3_Station.h"


int main(int argc, char **argv){

    // Initialise ROS
    ros::init(argc, argv, "Elisa3_Station");

    nh_ = ros::NodeHandlePtr(new ros::NodeHandle);

    elisa3_register = nh_->advertiseService("/registerElisa3", registerElisa3);
    elisa3_color = nh_->advertiseService("/colorElisa3", colorElisa3);

    int elisa3_rate;
    helpers::safelyRetrieve(*nh_, "/elisa3/sampling_rate", elisa3_rate, 100);
    ros::Rate loop_rate(elisa3_rate);
    //ros::AsyncSpinner spinner(2);
    //spinner.start();
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    int * addresses = &elisa3_addresses[0];

    logMsg("Elisa Station", "Starting with " + std::to_string(elisa3_addresses.size()) +
    " Elisa3 robots", 2);
    
    startCommunication(addresses, elisa3_addresses.size());
    calibrateSensorsForAll();
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    while(ros::ok()){

        for(int i = 0; i < elisa3_addresses.size(); i++){

            int address = elisa3_addresses[i];
            panda::Readout msg;

            msg.x = -getOdomYpos(address)/1000.0;  // mm to m
            msg.y = getOdomXpos(address)/1000.0;  // mm to m
            msg.theta = getOdomTheta(address) / 180.0 * M_PI + M_PI_2; // Theta expressed in a tenth of degree
            readout_pubs[i].publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();

    }

    logMsg("Elisa3_Station", "Stopping Elisa3 Movement.", 2);
    for(int i = 0; i < elisa3_addresses.size(); i++){
        setRightSpeed(elisa3_addresses[i], 0);
        setLeftSpeed(elisa3_addresses[i], 0);
        setColor(elisa3_addresses[i], 0, 0, 0);
        transferData();
    }

    stopCommunication();

    return 0;

}
// Threaded spinner?
bool moveElisa3(const panda::Move::ConstPtr& msg) {

    setRightSpeed(msg->address, msg->wheel_right);
    setLeftSpeed(msg->address, msg->wheel_left);
    return true;

}

bool registerElisa3(panda::registerElisa3::Request &req, panda::registerElisa3::Response &res){

    if(communication_started) {
        //stopCommunication();
    }

    elisa3_addresses.push_back(req.address);


    move_subs.push_back(nh_->subscribe<panda::Move>("elisa3_" +
        std::to_string(req.address) + "_move", 20, &moveElisa3));

    readout_pubs.push_back(nh_->advertise<panda::Readout>("elisa3_" +
        std::to_string(req.address) + "_readout", 20));

    communication_started = true;
    ros::Duration(0.2).sleep();
    logMsg("Elisa Station", "Succesfully added a new Elisa3 with address " + std::to_string(req.address) + "!", 2);

    return true;
}


bool colorElisa3(panda::colorElisa3::Request &req, panda::colorElisa3::Response &res){

    if(req.color_type == 0){
        setColor(req.address, req.red, req.green, req.blue);
        //logMsg("Elisa Station", "Set color to given values!", 2);

    }else{
        switch(req.color_type){
            case 1:
                setColor(req.address, 80, 0, 0);
                break;
            case 2:
                setColor(req.address, 0, 80, 0);
                break;
            case 3:
                setColor(req.address, 0, 0, 80);
                break;
            case 4:
                setColor(req.address, 60, 60, 0);
                break;
            case 5:
                setColor(req.address, 0, 60, 60);
                break;
            case 6:
                setColor(req.address, 60, 0, 60);
                break;
        }
    }



    return true;
}

void setColor(int address, int r, int g, int b){

    setRed(address, r);
    setGreen(address, g);
    setBlue(address, b);
}