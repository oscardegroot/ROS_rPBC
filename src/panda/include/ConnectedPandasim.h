/** @class ConnectedPandasim
 * System class and franka controller extended from the panda controller
 * @brief Run a controller in simulation using the matrices from the robot itself
 * 
 * @note The implementation of the Jacobian with coordinates in franka model is wrong. I fixed it by calling the right function.
 * 
 * */

#pragma once
#include "Panda.h"
#include <mutex>

namespace panda{
    
    class ConnectedPandasim : public Panda{
        
    public:
    
        ConnectedPandasim();

        /** @brief send 0 input to the robot to keep it active, then send the input to the sim */
        bool sendInput(const Eigen::VectorXd& tau) override;
        
        /** @brief We retrieve the matrices from the robot directly but with simulation coordinates */
        void retrieveMatrices() override;
        void retrieveState() override;
        
        /** @brief We want to receive simulation states */
        void readStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

        Eigen::VectorXd& dVdq();

    private:
    
        State state_previous;
        
        // Communication with the simulation
        ros::Publisher tau_pubs[7];
        ros::Subscriber sensor_sub;
        
        //bool initial_matrices = false;
        bool received_first_state = false;
    };

}