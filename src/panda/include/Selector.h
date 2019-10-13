#pragma once

#include <vector>
#include <array>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <algorithm>

#include "Agent.h"

/**
 * @class Selector
 * @author Oscar
 * @date 13/10/19
 * @file Selector.h
 * @brief Generally: Class for selecting coordinates from a full model with more coordinates than used. 
 * In practice: dividing the possible cooperative coordinates into leader coordinates, cooperative coordinates and local coordinates
 */
class Selector{
public:

    // Remove the base constructor
    Selector() = delete;
    
    /**
     * @brief Setup selector matrices
     * @param max_dim Maximal dimension to select from
     * @param selector_param Name of the selector parameter to read
     * @param values Values to select on (e.g. for coop 1, 3)
     */
    Selector(Agent& agent, int max_dim, const std::string& selector_param, const std::vector<int>& values);
    
    /**
    * @brief select: selects the rows from a system matrix of maximal cooperative dimension based on the currently used cooperative coordinates
    * @param input: input matrix (lmax x *) or vector (lmax)
    * @return selected matrix (l_selected x *) or vector (l_selected)
    */
    Eigen::MatrixXd select(const Eigen::MatrixXd& input);
    
    /**
    * @brief deselect: performs the inverse transformation for the input which is of dimension l, while the selected dimension is a subset of that
    * dimension
    * 
    * @param input: vector (l_selected)
    * @return vector (lmax)
    */
    Eigen::VectorXd deselect(const Eigen::VectorXd& input);

    /**
     * @brief Getter for selected dimension
     */
    int dim() const {return m_select.rows();};
    
private:  

    // Back end matrix that multiplies with inputs
    Eigen::MatrixXd m_select;
    
    
    
};