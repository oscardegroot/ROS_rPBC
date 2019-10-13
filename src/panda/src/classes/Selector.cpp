#include "Selector.h"


Selector::Selector(Agent& agent, int max_dim, const std::string& selector_param, const std::vector<int>& values){

    // Retrieve the selector vector.
    Eigen::VectorXd selector_e;
    std::vector<int> selector;
    agent.retrieveEigen(selector_param, selector_e, max_dim);

    // Convert to vector
    for(size_t i = 0; i < selector_e.rows(); i++){
        selector.push_back(selector_e[i]);
    }

    // Count the number of activated coordinates
    int selected_dim = 0;
    for(size_t i = 0; i < values.size(); i++){
        
        selected_dim += count(selector.begin(), selector.end(), values[i]);
    }
    
    // Initialise selector matrices
    m_select = Eigen::MatrixXd::Zero(selected_dim, max_dim);

    // Create a matrix that selects the required entries
    int count_selected = 0;
    for(int i = 0; i < selector.size(); i++) {

        // Check if the current selector value is in the values vector
        if(std::find(values.begin(), values.end(), selector[i]) != values.end()) {
            
            // If it is then this is a selected column
            m_select(count_selected, i) = 1;
            count_selected++;
        }
    }
}
   

Eigen::MatrixXd Selector::select(const Eigen::MatrixXd& input){
    return m_select * input;
}


Eigen::VectorXd Selector::deselect(const Eigen::VectorXd& input){
    return m_select.transpose() * input;    
}