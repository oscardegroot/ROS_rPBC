
#ifndef POTENTIALFACTORS_H
#define POTENTIALFACTORS_H
// A struct that contains the multipliers for r_i (matrix) and r_js (scalar)
// Implements some operators to ease handling them inside Potential.h
struct PotentialFactors{
    
    Eigen::MatrixXd i_matrix;
    double js_multiplier;
    
    PotentialFactors(Eigen::MatrixXd i_matrix_set, double js_multiplier_set)
        : i_matrix(i_matrix_set), js_multiplier(js_multiplier_set)
    {
    }
    
    PotentialFactors(int l){
        i_matrix = Eigen::MatrixXd::Zero(l, l);
        js_multiplier = 0.0;
    }
    
    PotentialFactors& operator*=(const double& x) {
        this->i_matrix *= x;
        this->js_multiplier *= x;
        return *this;
        //return ObstacleReturn(i_matrix * x, js_multiplier * x);
    }

    
    PotentialFactors& operator+=(const PotentialFactors& x) {
        this->i_matrix += x.i_matrix;
        this->js_multiplier += x.js_multiplier;
        return *this;
    }
};

#endif