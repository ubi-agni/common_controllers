// Copyright 2014 WUT
/*
 * cartesian_trajectory_action.cpp
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#include "cartesian_trajectory_action.h"

#include <string>

using ::cartesian_trajectory_msgs::CartesianTrajectory;
using cartesian_trajectory_msgs::CartesianTrajectoryConstPtr;

CartesianTrajectoryAction::CartesianTrajectoryAction(const std::string& name) : RTT::TaskContext(name) {
  this->ports()->addPort("CartesianTrajectoryCommand", port_cartesian_trajectory_command_);
  this->ports()->addPort("trajectory", port_cartesian_trajectory_);
}

CartesianTrajectoryAction::~CartesianTrajectoryAction() {
}

bool CartesianTrajectoryAction::startHook() {
  return true;
}

void CartesianTrajectoryAction::updateHook() {
  cartesian_trajectory_msgs::CartesianTrajectory trj;
  if (port_cartesian_trajectory_.read(trj) == RTT::NewData) {
    std::cout << "New trajectory point" << std::endl;
    CartesianTrajectory* trj_ptr =  new CartesianTrajectory;
    *trj_ptr = trj;
    CartesianTrajectoryConstPtr trj_cptr = CartesianTrajectoryConstPtr(trj_ptr);

    port_cartesian_trajectory_command_.write(trj_cptr);
  }
}
