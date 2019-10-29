
#ifndef POTENTIALFACTORS_H
#define POTENTIALFACTORS_H

/**
 * @class PotentialFactors
 * @author Oscar
 * @date 23/10/19
 * @file PotentialFactors.h
 * @brief A struct that contains the RHS of the control law and a multiplier for r_js
 */
struct PotentialFactors{
    
    Eigen::VectorXd RHS_vector;
    double LHS_multiplier;
    
    PotentialFactors(Eigen::VectorXd RHS_vector_set, double LHS_multiplier_set)
        : RHS_vector(RHS_vector_set), LHS_multiplier(LHS_multiplier_set)
    {
    }
    
    PotentialFactors(int l){
        RHS_vector = Eigen::VectorXd::Zero(l);
        LHS_multiplier = 0.0;
    }
    
    PotentialFactors& operator*=(const double& x) {
        this->RHS_vector *= x;
        this->LHS_multiplier *= x;
        return *this;
        //return ObstacleReturn(i_matrix * x, js_multiplier * x);
    }
    
    PotentialFactors operator*(const double& x){
        this->RHS_vector *= x;
        this->LHS_multiplier *= x;
        return *(this);
    }
    
    PotentialFactors operator/(const double& x){
        this->RHS_vector /= x;
        this->LHS_multiplier /= x;
        return *(this);
    }

    
    PotentialFactors& operator+=(const PotentialFactors& x) {
        this->RHS_vector += x.RHS_vector;
        this->LHS_multiplier += x.LHS_multiplier;
        return *this;
    }
    
    PotentialFactors& operator-(const PotentialFactors& x) {
        this->RHS_vector -= x.RHS_vector;
        this->LHS_multiplier -= x.LHS_multiplier;
        return *this;
    }
    
    // Produces the length of this potential factors if r_js is given
    double length(const Eigen::VectorXd& r_js) const{
        return helpers::normOf(RHS_vector + LHS_multiplier * r_js);
    }
    
    void print() const{
        logTmp("RHS Vector", RHS_vector);
        logTmp("LHS", LHS_multiplier);
    }
};

#endif