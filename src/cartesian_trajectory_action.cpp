// Copyright 2014 WUT
/*
 * cartesian_trajectory_action.cpp
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#include "cartesian_trajectory_action.h"

#include <string>

#include "Eigen/Dense"

#include "rtt_rosclock/rtt_rosclock.h"
#include "eigen_conversions/eigen_msg.h"

using ::cartesian_trajectory_msgs::CartesianTrajectory;
using cartesian_trajectory_msgs::CartesianTrajectoryConstPtr;

CartesianTrajectoryAction::CartesianTrajectoryAction(const std::string& name) : RTT::TaskContext(name) {
  this->ports()->addPort("CartesianTrajectoryCommand", port_cartesian_trajectory_command_);
  this->ports()->addPort("trajectory", port_cartesian_trajectory_);

  as_.addPorts(this->provides());

  as_.registerGoalCallback(boost::bind(&CartesianTrajectoryAction::goalCB, this, _1));
  as_.registerCancelCallback(boost::bind(&CartesianTrajectoryAction::cancelCB, this, _1));
}

CartesianTrajectoryAction::~CartesianTrajectoryAction() {
}

bool CartesianTrajectoryAction::startHook() {
  if (as_.ready()) {
    as_.start();
  } else {
    return false;
  }

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

  if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
    cartesian_trajectory_msgs::CartesianTrajectoryFeedback feedback;
    Eigen::Affine3d actual, desired, error;

    ros::Time now = rtt_rosclock::host_rt_now();

    port_cartesian_position_.read(feedback.actual);
    port_cartesian_position_command_.read(feedback.desired);

    tf::poseMsgToEigen(feedback.actual, actual);
    tf::poseMsgToEigen(feedback.desired, desired);

    error = actual.inverse() * desired;

    tf::poseEigenToMsg(error, feedback.error);
    feedback.header.stamp = now;
    active_goal_.publishFeedback(feedback);

    Goal g = active_goal_.getGoal();

    // TODO(konradb3): check path constraint.


    // TODO(konradb3): check goal constraint.
    size_t last_point = g->trajectory.points.size() - 1;

    if ((g->trajectory.header.stamp + g->trajectory.points[last_point].time_from_start) < now) {
      active_goal_.setSucceeded();
    }
  }
}

void CartesianTrajectoryAction::goalCB(GoalHandle gh) {
  // cancel active goal
  if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
    active_goal_.setCanceled();
  }

  Goal g = gh.getGoal();

  CartesianTrajectory* trj_ptr =  new CartesianTrajectory;
  *trj_ptr = g->trajectory;
  CartesianTrajectoryConstPtr trj_cptr = CartesianTrajectoryConstPtr(trj_ptr);
  port_cartesian_trajectory_command_.write(trj_cptr);

  gh.setAccepted();
  active_goal_ = gh;
}

void CartesianTrajectoryAction::cancelCB(GoalHandle gh) {
  if (active_goal_ == gh) {
    port_cartesian_trajectory_command_.write(CartesianTrajectoryConstPtr());
    active_goal_.setCanceled();
  }
}

