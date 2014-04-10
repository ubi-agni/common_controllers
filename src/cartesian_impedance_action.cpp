/*
 * cartesian_trajectory_action.cpp
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#include "cartesian_impedance_action.h"

CartesianImpedanceAction::CartesianImpedanceAction(const std::string& name) : RTT::TaskContext(name) {
	this->ports()->addPort("CartesianImpedanceTrajectoryCommand", port_cartesian_trajectory_command_);
	this->ports()->addPort("impedance", port_cartesian_trajectory_);
}

CartesianImpedanceAction::~CartesianImpedanceAction() {
	// TODO Auto-generated destructor stub
}

bool CartesianImpedanceAction::startHook() {
	return true;
}

void CartesianImpedanceAction::updateHook() {
	cartesian_trajectory_msgs::CartesianImpedanceTrajectory trj;
	if(port_cartesian_trajectory_.read(trj) == RTT::NewData) {
		std::cout << "New trajectory point" << std::endl;
		cartesian_trajectory_msgs::CartesianImpedanceTrajectory* trj_ptr =  new cartesian_trajectory_msgs::CartesianImpedanceTrajectory;
		*trj_ptr = trj;
		cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr trj_cptr = cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr(trj_ptr);

		port_cartesian_trajectory_command_.write(trj_cptr);
	}
}
