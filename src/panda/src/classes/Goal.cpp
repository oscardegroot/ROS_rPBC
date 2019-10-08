
#include "Goal.h"

Goal::Goal()
{

}

WangGoal::WangGoal(Agent& agent)
{
    agent.retrieveParameter("NF/gamma/eps", eps, 0.15);
    agent.retrieveParameter("NF/gamma/r_w", r_w, 1.0);
    
    /* this needs to be nicer...*/
    ros::NodeHandle nh;
    
    std::string output;
    helpers::safelyRetrieve(nh, "/output", output);

    if(output == "r"){
        double lambda;
        helpers::safelyRetrieve(nh, "/lambda", lambda);
        r_w *= lambda;
        eps *= lambda;
    }

    
    initParameters();
}

double WangGoal::gradient(const double& d)
{
    
    double g_gamma = 0.0;

	if(d >= 2.0*r_w){
        g_gamma = 0.0;
	}else if(d < r){

        g_gamma = 3.0*a1*d + 2.0*b1;
	}else{

        g_gamma = (3.0*a2*d*d+2.0*b2*d+c2) / d;
	}

	return g_gamma;
}


double WangGoal::value(const double& d)
{
    double v_gamma = 0.0;

    if(d >= 2.0*r_w){
        v_gamma = 1.0;
    }else if(d < r){

        v_gamma = a1*std::pow(d, 3.0) + b1*std::pow(d, 2.0);
    }else{
        v_gamma = a2*std::pow(d, 3.0) + b2*std::pow(d, 2.0) + c2*d + d2;
    }

    return v_gamma;
    
}


void WangGoal::initParameters(){
	r = 2.0*r_w*eps;

	a1 = (4.0*eps*r_w*r_w + 4.0*eps*r_w*r - 3.0*r*r) / (4.0*r_w*std::pow(r, 3.0)*(r-2.0*r_w));
	b1 = (3.0*r*r-12.0*eps*r_w*r_w)/(4.0*r_w*r*r*(r-2.0*r_w));
	a2 = (4.0*eps*r_w*r-3.0*r*r+8.0*r_w*r-12.0*eps*r_w*r_w)/(4.0*r_w*r*std::pow(r-2.0*r_w, 3.0));
	b2 = (-12.0*eps*r_w*r_w*r-24.0*r_w*r_w*r+3.0*std::pow(r, 3.0) + 48.0*eps*std::pow(r_w, 3.0))/(4.0*r_w*r*std::pow(r-2.0*r_w, 3.0));
	c2 = (9.0*r_w*r*r-12.0*eps*std::pow(r_w, 3.0) - 3.0*std::pow(r, 3.0))/(r*std::pow(r-2.0*r_w, 3.0));
	d2 = 1.0- (r_w*(-4.0*eps*r_w*r_w*r - 8.0*r_w*r_w*r + 12.0*r_w*r*r - 3.0*std::pow(r, 3.0)))/(r*std::pow(r - 2.0*r_w, 3.0));
}
