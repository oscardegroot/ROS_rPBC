
/** @classmaynotbenecessary */
#pragma once
//
//class CooperativeController{
//    
//};


class EdgeLPF : public Edge{
  
    
public:
    
    /**
     * @brief Apply WVM only on the given data
     * @param r_js
     * @param r_i
     */
    void applyReconstruction(Eigen::VectorXd& r_js, const Eigen::VectorXd& r_i);
    
    Eigen::VectorXd sample(const Eigen::VectorXd& r_i);
    
    Eigen::VectorXd calculateControls(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i);
    Eigen::VectorXd calculateWaves(const Eigen::VectorXd& s_in, const Eigen::VectorXd& r_i);
    
    void publishWave(const Eigen::VectorXd& r_i);
    void waveCallback(const panda::Waves::ConstPtr& msg);
    
    
    
        
};