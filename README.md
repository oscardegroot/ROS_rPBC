# ROS_rPBC
ROS repository for application of "Distributed Passivity Based Control using r-passivity"

# Feature Description
This implement of cooperative rPBC is split into client-side and server-side modules to facilitate distributed control.

## Client-Side
### System (System.h / System.cpp)
For physical system implementation, a system interface is provided. This repository provides implementations for 
- Panda (7 DOF Robotic Manipulator)
- Elisa3 (Differential Drive Robots)

Additionally the controller interface (Controller.h) provides an interface for control of a specified system. This repository implements rPBC and extended IDA-PBC.

### Communication Management Module (CMM.h / CMM.cpp)
A novel feature of this repository is the implemented CMM , which fully implements communication to other systems. It features:
- Implementation of the ST with
  - WVM for reconstruction in case of lost packages (EdgeFlex.h / EdgeFlex.cpp)
  - Rate transitioning between network and the agent
  - Integration with the potential class (see below), allowing for constraints
  - EdgeLeader.h makes an agent leader via a virtual edge
  - Formations
- Simple interface via
  - Sample(output) -> samples the network, returning the cooperative input and sends a wave variable to all its connected agents
 
### Artificial Potential Functions (APFs) and Navigation Functions (NFs)
The Potential.cpp class contains interfaces for simple potentials (only an attractive goal without changing) and advanced potentials (attractive and multiple repelling potentials changable at run-time). Most notable an implementation of Navigation Functions is provided.  The two related files are
- Obstacle.h: contains different types of obstacles and weight functions and can easily be extended depending on the application.
- Goal.h contains some attractive potentials and weighting functions.
Current implementations rely on the function in https://link.springer.com/content/pdf/10.1007%2Fs10846-016-0450-0.pdf

The integration of Potential.h and EdgeFlex.h is such that the user can influence the control law by simply adding the appropriate goal and obstacle functions in the constructor.

## Server-Side
### ServerNode
The servernode handles all the central processes on the server-side. An FSM is implemented to facilitate synchronisation and registration of systems. This server provides ROS services for formation and leader retrieval which are called by the client-side nodes.

### ElisaStation
The ElisaStation class provides a central interface to communicates with the camera detection algorithm. It additionally collects commands to the Elisa3 robots and forwards them centrally.
