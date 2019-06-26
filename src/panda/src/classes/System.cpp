/*
File: System.cpp

Defines an interface for systems controlled by IDAPBC or rPBC
*/

#include "System.h"


System::System(int n_set, int m_set) 
	: n(n_set), m(m_set)
{
	

}
		

System::~System(){
}


bool System::dataReady(){
	return data_rdy;
}


void System::setState(Eigen::VectorXd new_q, Eigen::VectorXd new_qd){
	state = State{new_q, new_qd};
}

void System::setDataReady(bool is_ready){
	data_rdy = is_ready;
}
