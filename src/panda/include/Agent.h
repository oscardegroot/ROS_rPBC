
#ifndef Agent_H
#define Agent_H

#include "ros/ros.h"
#include "panda/Waves.h"
#include "std_msgs/Float64MultiArray.h"
#include "panda/getConnectionsOf.h"
#include "Edge.h"
#include "Goals.h"
#include <vector>
#include <string>
#include <sstream>

int agent_id, j_id, l, N;
ros::ServiceClient connect_client;
std::vector<Edge*> edges;

void initiateEdges();
int main(int argc, char **argv);


#endif