/*
 * cartesian_trajectory_action.cpp
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#include "cartesian_trajectory_action.h"

CartesianTrajectoryAction::CartesianTrajectoryAction(const std::string& name) : RTT::TaskContext(name) {
	this->ports()->addPort("CartesianTrajectoryCommand", port_cartesian_trajectory_command_);
	this->ports()->addPort("trajectory", port_cartesian_trajectory_);
}

CartesianTrajectoryAction::~CartesianTrajectoryAction() {
	// TODO Auto-generated destructor stub
}

bool CartesianTrajectoryAction::startHook() {
	return true;
}

void CartesianTrajectoryAction::updateHook() {
	cartesian_trajectory_msgs::CartesianTrajectory trj;
	if(port_cartesian_trajectory_.read(trj) == RTT::NewData) {
		std::cout << "New trajectory point" << std::endl;
		cartesian_trajectory_msgs::CartesianTrajectory* trj_ptr =  new cartesian_trajectory_msgs::CartesianTrajectory;
		*trj_ptr = trj;
		cartesian_trajectory_msgs::CartesianTrajectoryConstPtr trj_cptr = cartesian_trajectory_msgs::CartesianTrajectoryConstPtr(trj_ptr);

		port_cartesian_trajectory_command_.write(trj_cptr);
	}
}
