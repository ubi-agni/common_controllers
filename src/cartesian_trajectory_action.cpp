// Copyright 2014 WUT
/*
 * cartesian_trajectory_action.cpp
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#include "cartesian_trajectory_action.h"

#include <string>

#include "rtt_rosclock/rtt_rosclock.h"
#include "eigen_conversions/eigen_msg.h"

using ::cartesian_trajectory_msgs::CartesianTrajectory;
using cartesian_trajectory_msgs::CartesianTrajectoryConstPtr;

CartesianTrajectoryAction::CartesianTrajectoryAction(const std::string& name) : RTT::TaskContext(name) {
  this->ports()->addPort("CartesianTrajectoryCommand", port_cartesian_trajectory_command_);
  this->ports()->addPort("trajectory", port_cartesian_trajectory_);
  this->ports()->addPort("CartesianPosition", port_cartesian_position_);

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

    if (!checkTolerance(error, g->path_tolerance)) {
      port_cartesian_trajectory_command_.write(CartesianTrajectoryConstPtr());
      cartesian_trajectory_msgs::CartesianTrajectoryResult res;
      res.error_code = cartesian_trajectory_msgs::CartesianTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      active_goal_.setAborted(res);
    }

    // TODO(konradb3): check goal constraint.
    size_t last_point = g->trajectory.points.size() - 1;

    if ((g->trajectory.header.stamp + g->trajectory.points[last_point].time_from_start) < now) {
      active_goal_.setSucceeded();
    }
  }
}

bool CartesianTrajectoryAction::checkTolerance(Eigen::Affine3d err, cartesian_trajectory_msgs::CartesianTolerance tol) {
  if ((tol.position.x > 0.0) && (fabs(err.translation().x()) > tol.position.x)) {
    return false;
  }

  if ((tol.position.y > 0.0) && (fabs(err.translation().y()) > tol.position.y)) {
    return false;
  }

  if ((tol.position.z > 0.0) && (fabs(err.translation().z()) > tol.position.z)) {
    return false;
  }

  Eigen::AngleAxisd ax(err.rotation());
  Eigen::Vector3d rot = ax.axis() * ax.angle();

  if ((tol.rotation.x > 0.0) && (fabs(rot(0)) > tol.rotation.x)) {
    return false;
  }

  if ((tol.rotation.y > 0.0) && (fabs(rot(1)) > tol.rotation.y)) {
    return false;
  }

  if ((tol.rotation.z > 0.0) && (fabs(rot(2)) > tol.rotation.z)) {
    return false;
  }

  return true;
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

