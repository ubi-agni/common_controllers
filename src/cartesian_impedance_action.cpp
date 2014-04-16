// Copyright 2014 WUT
/*
 * cartesian_trajectory_action.cpp
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#include "cartesian_impedance_action.h"

#include <string>

using ::cartesian_trajectory_msgs::CartesianImpedanceTrajectory;
using ::cartesian_trajectory_msgs::CartesianImpedanceTrajectoryConstPtr;

CartesianImpedanceAction::CartesianImpedanceAction(const std::string& name) : RTT::TaskContext(name) {
  this->ports()->addPort("CartesianImpedanceTrajectoryCommand", port_cartesian_trajectory_command_);
  this->ports()->addPort("impedance", port_cartesian_trajectory_);
}

CartesianImpedanceAction::~CartesianImpedanceAction() {
}

bool CartesianImpedanceAction::startHook() {
  return true;
}

void CartesianImpedanceAction::updateHook() {
  cartesian_trajectory_msgs::CartesianImpedanceTrajectory trj;
  if (port_cartesian_trajectory_.read(trj) == RTT::NewData) {
    std::cout << "New trajectory point" << std::endl;
    CartesianImpedanceTrajectory* trj_ptr =  new CartesianImpedanceTrajectory;
    *trj_ptr = trj;
    CartesianImpedanceTrajectoryConstPtr trj_cptr = CartesianImpedanceTrajectoryConstPtr(trj_ptr);

    port_cartesian_trajectory_command_.write(trj_cptr);
  }
}
