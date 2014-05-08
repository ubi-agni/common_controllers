// Copyright 2014 WUT
/*
 * cartesian_trajectory_action.h
 *
 *  Created on: 5 mar 2014
 *      Author: konradb3
 */

#ifndef CARTESIAN_TRAJECTORY_ACTION_H_
#define CARTESIAN_TRAJECTORY_ACTION_H_

#include <string>

#include "rtt/TaskContext.hpp"
#include "rtt/Port.hpp"
#include "cartesian_trajectory_msgs/CartesianTrajectory.h"
#include "cartesian_trajectory_msgs/CartesianTrajectoryAction.h"
#include "cartesian_trajectory_msgs/CartesianTrajectoryGoal.h"

#include "rtt_actionlib/rtt_actionlib.h"
#include "rtt_actionlib/rtt_action_server.h"

#include "Eigen/Dense"

class CartesianTrajectoryAction: public RTT::TaskContext {
 private:
  typedef actionlib::ServerGoalHandle<cartesian_trajectory_msgs::CartesianTrajectoryAction> GoalHandle;
  typedef boost::shared_ptr<const cartesian_trajectory_msgs::CartesianTrajectoryGoal> Goal;

 public:
  explicit CartesianTrajectoryAction(const std::string& name);
  virtual ~CartesianTrajectoryAction();

  bool startHook();
  void updateHook();

 private:
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

  bool checkTolerance(Eigen::Affine3d err, cartesian_trajectory_msgs::CartesianTolerance tol);

  RTT::OutputPort<cartesian_trajectory_msgs::CartesianTrajectoryConstPtr> port_cartesian_trajectory_command_;
  RTT::InputPort<cartesian_trajectory_msgs::CartesianTrajectory> port_cartesian_trajectory_;
  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_;
  RTT::InputPort<geometry_msgs::Pose> port_cartesian_position_command_;
  rtt_actionlib::RTTActionServer<cartesian_trajectory_msgs::CartesianTrajectoryAction> as_;
  GoalHandle active_goal_;
};

#endif  // CARTESIAN_TRAJECTORY_ACTION_H_
