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
  
  as_.addPorts(this->provides());

  as_.registerGoalCallback(boost::bind(&CartesianImpedanceAction::goalCB, this, _1));
  as_.registerCancelCallback(boost::bind(&CartesianImpedanceAction::cancelCB, this, _1));
}

CartesianImpedanceAction::~CartesianImpedanceAction() {
}

bool CartesianImpedanceAction::startHook() {
  if (as_.ready()) {
    as_.start();
  } else {
    return false;
  }

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
  
  if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
    Goal g = active_goal_.getGoal();
    size_t last_point = g->trajectory.points.size() - 1;

    if ((g->trajectory.header.stamp + g->trajectory.points[last_point].time_from_start) < rtt_rosclock::host_now()) {
      active_goal_.setSucceeded();
    }
  }
}

void CartesianImpedanceAction::goalCB(GoalHandle gh) {
  // cancel active goal
  if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
    active_goal_.setCanceled();
  }

  Goal g = gh.getGoal();

  bool valid = true;

  for (size_t i = 0; i < g->trajectory.points.size(); i++) {
    valid = valid && checkImpedance(g->trajectory.points[i].impedance);
  }

  if (!valid) {
    cartesian_trajectory_msgs::CartesianImpedanceResult res;
    res.error_code = cartesian_trajectory_msgs::CartesianImpedanceResult::INVALID_GOAL;
    gh.setRejected(res);
  }

  CartesianImpedanceTrajectory* trj_ptr =  new CartesianImpedanceTrajectory;
  *trj_ptr = g->trajectory;
  CartesianImpedanceTrajectoryConstPtr trj_cptr = CartesianImpedanceTrajectoryConstPtr(trj_ptr);
  
  if (g->trajectory.header.stamp < rtt_rosclock::host_now()) {
    valid = false;
    cartesian_trajectory_msgs::CartesianImpedanceResult res;
    res.error_code = cartesian_trajectory_msgs::CartesianImpedanceResult::OLD_HEADER_TIMESTAMP;
    gh.setRejected(res);
  }
  
  if (valid) {
    port_cartesian_trajectory_command_.write(trj_cptr);

    gh.setAccepted();
    active_goal_ = gh;
  }
}

void CartesianImpedanceAction::cancelCB(GoalHandle gh) {
  if (active_goal_ == gh) {
    port_cartesian_trajectory_command_.write(CartesianImpedanceTrajectoryConstPtr());
    active_goal_.setCanceled();
  }
}

bool CartesianImpedanceAction::checkImpedance(const cartesian_trajectory_msgs::CartesianImpedance& imp) {
  if(imp.stiffness.force.x > 4000.0 || imp.stiffness.force.x < 0.0) {
    return false;
  }

  if(imp.stiffness.force.y > 4000.0 || imp.stiffness.force.y < 0.0) {
    return false;
  }

  if(imp.stiffness.force.z > 4000.0 || imp.stiffness.force.z < 0.0) {
    return false;
  }

  if(imp.stiffness.torque.x > 300.0 || imp.stiffness.torque.x < 0.0) {
    return false;
  }

  if(imp.stiffness.torque.y > 300.0 || imp.stiffness.torque.y < 0.0) {
    return false;
  }

  if(imp.stiffness.torque.z > 300.0 || imp.stiffness.torque.z < 0.0) {
    return false;
  }
  
  if(imp.damping.force.x > 1.0 || imp.damping.force.x < 0.0) {
    return false;
  }

  if(imp.damping.force.y > 1.0 || imp.damping.force.y < 0.0) {
    return false;
  }

  if(imp.damping.force.z > 1.0 || imp.damping.force.z < 0.0) {
    return false;
  }

  if(imp.damping.torque.x > 1.0 || imp.damping.torque.x < 0.0) {
    return false;
  }

  if(imp.damping.torque.y > 1.0 || imp.damping.torque.y < 0.0) {
    return false;
  }

  if(imp.damping.torque.z > 1.0 || imp.damping.torque.z < 0.0) {
    return false;
  }
  
  return true;
}

