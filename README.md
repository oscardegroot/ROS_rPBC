# ROS_rPBC
ROS repository for application of "Distributed Passivity Based Control using r-passivity"

# Feature Description

## Communication Management Module (CMM)
A novel feature of this repository is the implemented CMM (see CMM.h / CMM.cpp), which fully implements communication to other systems. It features:
- Implementation of the ST with
  - WVM for reconstruction in case of lost packages
  - Rate transitioning between network and the agent
  - Integration with the potential class (see below), allowing for many different constraints
  - EdgeLeader.h makes an agent leader via a virtual edge
  - Formations
- Automatic retrieval via the remote of
  - Connected agents
  - Formations
  - Leaders
- Simple interface via
  - Sample(output) -> samples the network, returning the cooperative input and sends a wave variable to all its connected agents
 
 
## Artificial Potential Functions (APFs) and Navigation Functions (NFs)
The Potential.cpp class contains interfaces for simple potentials (only an attractive goal without changing) and advanced potentials (attractive and multiple repelling potentials changable at run-time). Most notable an implementation of Navigation Functions is provided.  The two related files are
- Obstacle.h: contains different types of obstacles and weight functions and can easily be extended depending on the application.
- Goal.h contains some attractive potentials and weighting functions.
Current implementations rely on the function in https://link.springer.com/content/pdf/10.1007%2Fs10846-016-0450-0.pdf

The integration of Potential.h and EdgeFlex.h is such that the user can influence the control law by simply adding the appropriate goal and obstacle functions in the constructor.
