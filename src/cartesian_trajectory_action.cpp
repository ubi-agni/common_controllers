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

CartesianTrajectoryAction::CartesianTrajectoryAction(const std::string& name) :
    RTT::TaskContext(name),
    port_cartesian_trajectory_command_("CartesianTrajectoryCommand_OUTPORT", false),
    port_cartesian_trajectory_("trajectory_INPORT"),
    port_cartesian_position_("CartesianPosition_INPORT"),
    port_cartesian_position_command_("CartesianPositionCommand_INPORT"),
    port_cartesian_wrench_("CartesianWrench_INPORT") {

  this->ports()->addPort(port_cartesian_trajectory_command_);
  this->ports()->addEventPort(port_cartesian_trajectory_);
  this->ports()->addPort(port_cartesian_position_);
  this->ports()->addPort(port_cartesian_position_command_);
  this->ports()->addPort(port_cartesian_wrench_);

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
    RTT::Logger::log(RTT::Logger::Debug) << "New trajectory point received" << RTT::endlog();
    CartesianTrajectory* trj_ptr =  new CartesianTrajectory;
    *trj_ptr = trj;
    CartesianTrajectoryConstPtr trj_cptr = CartesianTrajectoryConstPtr(trj_ptr);

    port_cartesian_trajectory_command_.write(trj_cptr);
  }

  if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
    cartesian_trajectory_msgs::CartesianTrajectoryFeedback feedback;
    Eigen::Affine3d actual, desired, error;

    ros::Time now = rtt_rosclock::host_now();

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
      RTT::Logger::log(RTT::Logger::Debug) << "Path tolerance violated "
                                           << RTT::endlog();
      active_goal_.setAborted(res);
    }

    geometry_msgs::Wrench ft;
    port_cartesian_wrench_.read(ft);

    if (!checkWrenchTolerance(ft, g->wrench_constraint)) {
      port_cartesian_trajectory_command_.write(CartesianTrajectoryConstPtr());
      cartesian_trajectory_msgs::CartesianTrajectoryResult res;
      res.error_code = cartesian_trajectory_msgs::CartesianTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      RTT::Logger::log(RTT::Logger::Debug) << "Wrench constraints violated "
                                           << RTT::endlog();
      active_goal_.setAborted(res);
    }

    // TODO(konradb3): check goal constraint.
    size_t last_point = g->trajectory.points.size() - 1;

    if ((g->trajectory.header.stamp + g->trajectory.points[last_point].time_from_start) < now) {
      RTT::Logger::log(RTT::Logger::Debug) << "Goal succeeded"
                                           << RTT::endlog();
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

bool CartesianTrajectoryAction::checkWrenchTolerance(geometry_msgs::Wrench msr, geometry_msgs::Wrench tol) {
  if ((tol.force.x > 0.0) && (fabs(msr.force.x) > tol.force.x)) {
    return false;
  }

  if ((tol.force.y > 0.0) && (fabs(msr.force.y) > tol.force.y)) {
    return false;
  }

  if ((tol.force.z > 0.0) && (fabs(msr.force.z) > tol.force.z)) {
    return false;
  }

  if ((tol.torque.x > 0.0) && (fabs(msr.torque.x) > tol.torque.x)) {
    return false;
  }

  if ((tol.torque.y > 0.0) && (fabs(msr.torque.y) > tol.torque.y)) {
    return false;
  }

  if ((tol.torque.z > 0.0) && (fabs(msr.torque.z) > tol.torque.z)) {
    return false;
  }

  return true;
}

void CartesianTrajectoryAction::goalCB(GoalHandle gh) {
  
  Goal g = gh.getGoal();
  cartesian_trajectory_msgs::CartesianTrajectoryResult res;
  ros::Time now = rtt_rosclock::host_now();
  
  // check timing
  if (g->trajectory.header.stamp.toSec() !=0 )
  {
    if (g->trajectory.header.stamp < now) {
      RTT::Logger::log(RTT::Logger::Error) << "Old header timestamp"
                                         << RTT::endlog();
      res.error_code =
      cartesian_trajectory_msgs::CartesianTrajectoryResult::OLD_HEADER_TIMESTAMP;
      gh.setRejected(res, "old timestamp");
      return;
    }
  }
  
  // check trajectory
  if (g->trajectory.points.size() == 0 )
  {
    RTT::Logger::log(RTT::Logger::Error) << "Empty trajectory"
                                         << RTT::endlog();
      res.error_code =
      cartesian_trajectory_msgs::CartesianTrajectoryResult::INVALID_GOAL;
      gh.setRejected(res, "empty trajectory");
      return;
  }

  // cancel active goal
  RTT::Logger::log(RTT::Logger::Debug) << "Received goal " << RTT::endlog();
  if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
    active_goal_.setCanceled();
    RTT::Logger::log(RTT::Logger::Debug) << "Cancelling current goal" << RTT::endlog();
  }
  
  CartesianTrajectory* trj_ptr =  new CartesianTrajectory;
  *trj_ptr = g->trajectory;
    
  // consider timestamp at 0 means now
  if (g->trajectory.header.stamp.toSec() ==0 )
  {
    trj_ptr->header.stamp = now;
  }

  bool ok = true;

  RTT::TaskContext::PeerList peers = this->getPeerList();
  for (size_t i = 0; i < peers.size(); i++) {
    if(!this->getPeer(peers[i])->isRunning())
    {
      RTT::Logger::log(RTT::Logger::Debug) << "Starting peer : " << peers[i] << RTT::endlog();
      ok = ok && this->getPeer(peers[i])->start();
    }
  }

  if (ok) {
    CartesianTrajectoryConstPtr trj_cptr = CartesianTrajectoryConstPtr(trj_ptr);
    // send the trajectory out
    port_cartesian_trajectory_command_.write(trj_cptr);

    gh.setAccepted();
    active_goal_ = gh;
  } else {
    RTT::Logger::log(RTT::Logger::Error) << "peer did not start : " << RTT::endlog();
    gh.setRejected();
    active_goal_.setCanceled();
  }
}

void CartesianTrajectoryAction::cancelCB(GoalHandle gh) {
  if (active_goal_ == gh) {
    port_cartesian_trajectory_command_.write(CartesianTrajectoryConstPtr());
    active_goal_.setCanceled();
  }
}

