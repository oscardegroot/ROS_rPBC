//
// Created by omdegroot on 12-08-19.
//

#ifndef SRC_POTENTIAL_H
#define SRC_POTENTIAL_H

// Navigation function
class Potential {

    Potential(double alpha_set);

    void addGoalFcn(Goal goal);
    void addObstacleFcn(Obstacle obstacle);

    Eigen::VectorXd gradient(Eigen::VectorXd r_i, Eigen::VectorXd r_js);

private:
    double alpha;

    Goal new_goal;
    std::Vector<Obstacle> obstacles;

    double obstacleGradient(Eigen::VectorXd r_i, Eigen::VectorXd r_js);
    double obstacleValue(Eigen::VectorXd r_i, Eigen::VectorXd r_js);

};

class Goal {

    Goal();

    virtual double value(Eigen::VectorXd r_i, Eigen::VectorXd r_js) = 0;
    virtual double gradient(Eigen::VectorXd r_i, Eigen::VectorXd r_js) = 0;

};

class Obstacle{

    Obstacle();

    virtual double value(Eigen::VectorXd r_i, Eigen::VectorXd r_js) = 0;
    virtual double gradient(Eigen::VectorXd r_i, Eigen::VectorXd r_js) = 0;
};



#endif //SRC_POTENTIAL_H
