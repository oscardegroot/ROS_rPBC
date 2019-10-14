#include "Selector.h"


Selector::Selector(Agent& agent, int max_dim, const std::string& selector_param, const std::vector<int>& values){

    // Retrieve the selector vector.
    Eigen::VectorXd selector_e;
    std::vector<int> selector;
    
    // Note: size is not checked and may be larger!
    agent.retrieveEigen(selector_param, selector_e);

    // Convert to vector
    for(size_t i = 0; i < selector_e.rows(); i++){
        selector.push_back(selector_e[i]);
    }

    // Count the number of activated coordinates
    int selected_dim = 0;
    for(size_t i = 0; i < values.size(); i++){
        
        selected_dim += count(selector.begin(), selector.end(), values[i]);
    }
    
    if(selected_dim > max_dim){
        throw OperationalException("More dimensions selected than the specified maximum dimension! (dim = " +
            std::to_string(selected_dim) + ", max_dim = " + std::to_string(max_dim) + ")");
    }
    
    // Initialise selector matrices
    m_select = Eigen::MatrixXd::Zero(selected_dim, max_dim);

    // Create a matrix that selects the required entries
    int count_selected = 0;
    int count_zero = 0;
    for(int i = 0; i < selector.size(); i++) {

        // Check if the current selector value is in the values vector
        if(std::find(values.begin(), values.end(), selector[i]) != values.end()) {
            
            // If it is then this is a selected column
            m_select(count_selected, i - count_zero) = 1;
            count_selected++;
            
            // If we are done before the end, stop
            if(count_selected == selected_dim){
                break;
            }
        // Count zero values to compensate in case of selector arrays longer than max_dim with zeroes in between
        }else if(selector.size() > max_dim && selector[i] == 0){
            count_zero++;
        }
    }
}
   

Eigen::MatrixXd Selector::select(const Eigen::MatrixXd& input){
    return m_select * input;
}


Eigen::VectorXd Selector::deselect(const Eigen::VectorXd& input){
    return m_select.transpose() * input;    
}