//
// Created by omdegroot on 12-08-19.
//

#ifndef SRC_ELISA3_STATION_H
#define SRC_ELISA3_STATION_H

#define COLOR_BLUE 1
#define COLOR_GREEN 2
#define COLOR_RED 3
#define COLOR_ORANGE 4
#define COLOR_CYAN 5
#define COLOR_PINK 6

#include "ros/ros.h"

#include "elisa3-lib.h"
#include "panda/registerElisa3.h"
#include "panda/Move.h"
#include "panda/Readout.h"
#include "panda/colorElisa3.h"

#include <mutex>
#include <string>
#include <vector>

#include "Helpers.h"
#include "CustomLog.h"

bool registerElisa3(panda::registerElisa3::Request &req,
                    panda::registerElisa3::Response &res);

bool colorElisa3(panda::colorElisa3::Request &req,
                    panda::colorElisa3::Response &res);

bool communication_started = false;

bool moveElisa3(const panda::Move::ConstPtr& msg);

void setColor(int address, int r, int g, int b);

std::vector<int> elisa3_addresses;

ros::ServiceServer elisa3_register, elisa3_color;

std::unique_ptr<ros::NodeHandle> nh;

// Subscriber to move messages
std::vector<ros::Publisher> readout_pubs;
std::vector<ros::Subscriber> move_subs;

#endif //SRC_ELISA3_STATION_H
